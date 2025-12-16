# Ultralytics 🚀 AGPL-3.0 License - https://ultralytics.com/license

import io
import os
import sys
import math
import time
import threading
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, List, Tuple
from collections import defaultdict, deque

import cv2
import numpy as np
import torch

# --- 导入 TensorRT 深度工具 ---
try:
    from trt_depth_component import TrtDepthEstimator
except ImportError:
    print("❌ 无法找到 trt_depth_component.py")

current_dir = os.path.dirname(os.path.abspath(__file__))
ultralytics_root = os.path.dirname(os.path.dirname(current_dir))
if ultralytics_root not in sys.path:
    sys.path.insert(0, ultralytics_root)

from ultralytics import YOLO
from ultralytics.utils import LOGGER
from ultralytics.utils.checks import check_requirements

# --- 导入 3D 绘图工具 ---
try:
    from bbox3d_utils import BBox3DEstimator
except ImportError:
    LOGGER.error("❌ 无法导入 bbox3d_utils.py")
    sys.exit(1)

torch.classes.__path__ = []

# --- 缓存加载深度模型 (单例模式) ---
import streamlit as st
@st.cache_resource
def load_depth_engine():
    engine_path = '/home/nvidia/Downloads/Ros/depth-anything-tensorrt-main/Depth-Anything-3-main/ros1-depth-anything-v3-trt/onnx/da3_small.engine'
    if not os.path.exists(engine_path):
        st.error(f"❌ Engine file not found at: {engine_path}")
        return None
    print(f"✅ Loading Depth Engine from: {engine_path}")
    return TrtDepthEstimator(engine_path)

class Inference:
    """
    Dual-Stream Comparison: Geometric (Triangulation) vs Depth Anything (AI)
    """
    def __init__(self, **kwargs: Any) -> None:
        check_requirements("streamlit>=1.29.0")
        self.st = st
        self.source = "webcam"
        self.enable_trk = False
        self.conf = 0.25
        self.iou = 0.45
        self.selected_ind: List[int] = []
        self.model = None

        # --- 强制单摄像头配置 ---
        self.num_cameras = 1
        self.camera_indices = [0]
        self.camera_yaw_offsets = {0: 180.0}
        self.frame_width = 640
        self.frame_height = 480 
        self.camera_caps: List[cv2.VideoCapture] = []
        self.camera_threads: List[threading.Thread] = []
        self.camera_frames: Dict[int, np.ndarray] = {}
        self.camera_running = False
        
        # 相机内参
        self.camera_intrinsics = {'fx': 498.0, 'fy': 498.0, 'cx': 331.2797, 'cy': 240.0}
        
        # 用于几何测距 (Geometric Distance)
        self.class_real_widths = {
            6: 0.23,  # Class 6 = 23cm
            **{i: 0.31 for i in range(6)} # Others = 31cm
        }
        self.default_real_width = 0.31

        # 实例化两个独立的绘图器
        self.estimator_geo = BBox3DEstimator()
        self.estimator_da = BBox3DEstimator()

        self.DEPTH_MIN = 0.5  
        self.DEPTH_MAX = 8.0  

        self.camera_fps: Dict[int, float] = {}
        self.camera_frame_times: Dict[int, List[float]] = {}
        
        self.show_class_names = True
        self.show_distance = True

        # --- 【修复点】初始化 model_path 默认为 None，防止报错 ---
        self.temp_dict = {"model": None, **kwargs}
        self.model_path = None
        if self.temp_dict["model"] is not None:
            self.model_path = self.temp_dict["model"]
        # -----------------------------------------------------

    def _normalize_distance_to_depth_value(self, distance: float) -> float:
        if distance == float('inf'): return 1.0
        dist = np.clip(distance, self.DEPTH_MIN, self.DEPTH_MAX)
        return float(np.clip((dist - self.DEPTH_MIN) / (self.DEPTH_MAX - self.DEPTH_MIN), 0.0, 1.0))
    
    def calculate_spatial_info(self, bbox_xyxy, class_id, cam_index=0):
        """
        几何测距核心函数
        """
        x1, y1, x2, y2 = bbox_xyxy
        box_width_px = x2 - x1
        x_center_px = (x1 + x2) / 2
        if box_width_px <= 0: return float('inf'), 0.0, 0.0

        # 获取真实宽度
        real_width = self.class_real_widths.get(class_id, self.default_real_width)
        
        fx = self.camera_intrinsics['fx']
        # 几何公式：Z = (W_real * f) / W_pixel
        distance = (real_width * fx) / box_width_px
        
        cx = self.camera_intrinsics['cx']
        angle_rad = math.atan((x_center_px - cx) / fx)
        azimuth_deg = math.degrees(angle_rad)
        global_angle = (azimuth_deg + self.camera_yaw_offsets.get(cam_index, 0.0) + 360) % 360

        return distance, azimuth_deg, global_angle

    def web_ui(self) -> None:
        self.st.set_page_config(page_title="Depth Comparison", layout="wide")
        self.st.markdown("""<h1 style="text-align:center;">Depth Comparison</h1>""", unsafe_allow_html=True)
        self.st.markdown("""<h4 style="text-align:center;">Left: Geometric (Fixed Width) | Right: Depth Anything (AI)</h4>""", unsafe_allow_html=True)

    def sidebar(self) -> None:
        with self.st.sidebar:
            self.st.title("Settings")
            self.enable_trk = self.st.radio("Tracking", ("Yes", "No")) == "Yes"
            self.conf = self.st.slider("Conf", 0.0, 1.0, 0.25, 0.01)
            self.iou = self.st.slider("IoU", 0.0, 1.0, 0.45, 0.01)
            
            self.st.markdown("---")
            self.fps_container = self.st.empty()

        # 双列布局
        col1, col2 = self.st.columns(2)
        with col1:
            self.st.info("📏 Geometric (Old Method)")
            self.container_geo = self.st.empty()
        with col2:
            self.st.success("🧠 Depth Anything (New Method)")
            self.container_da = self.st.empty()

    def source_upload(self) -> None: pass

    def configure(self) -> None:
        available_models = ["Custom Car Detection Model (best.engine)"]
        custom_model_path = "/home/nvidia/Downloads/Ros/ballCar2/weights/weights/best.engine"
        if self.model_path: available_models.insert(0, self.model_path)
        selected_model = self.st.sidebar.selectbox("Model", available_models, index=0)
        with self.st.spinner("Loading Model..."):
            model_path = custom_model_path if "Custom" in selected_model else selected_model
            self.model = YOLO(model_path)
            self.selected_ind = list(range(len(self.model.names)))

    def setup_cameras(self) -> bool:
        self.camera_caps = []
        try:
            cap = cv2.VideoCapture(0) # 只开 Cam 0
            if cap.isOpened():
                cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.frame_width)
                cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.frame_height)
                self.camera_caps.append(cap)
                self.camera_frames[0] = np.zeros((480, 640, 3), dtype=np.uint8)
                self.camera_fps[0] = 0.0
                self.camera_frame_times[0] = []
                return True
        except: pass
        self.st.error("❌ Camera 0 failed")
        return False

    def camera_thread(self, idx, cap):
        while self.camera_running and cap.isOpened():
            ret, frame = cap.read()
            if ret: 
                self.camera_frames[idx] = frame.copy()
                now = time.time()
                self.camera_frame_times[idx].append(now)
                if len(self.camera_frame_times[idx]) > 30: self.camera_frame_times[idx].pop(0)
            time.sleep(0.01)

    def multi_camera_inference(self) -> None:
        if not self.setup_cameras(): return
        
        depth_engine = load_depth_engine()
        
        # 【校准系数】基于你之前的测试 (2.47m / 0.9 raw)
        DEPTH_COEFF = 2.14

        self.camera_running = True
        t = threading.Thread(target=self.camera_thread, args=(0, self.camera_caps[0]))
        t.daemon = True
        t.start()
        self.camera_threads.append(t)

        stop_btn = self.st.sidebar.button("Stop Comparison")
        frame_cnt = 0

        try:
            while self.camera_running:
                if stop_btn: break
                
                if 0 in self.camera_frames:
                    frame = self.camera_frames[0].copy()
                    
                    if frame.size > 0:
                        # 1. 深度推理 (只给右边用)
                        raw_depth_map = None
                        if depth_engine:
                            raw_depth_map = depth_engine.infer_raw(frame)
                        
                        # 2. YOLO 推理
                        if self.enable_trk:
                            results = self.model.track(frame, conf=self.conf, iou=self.iou, persist=True, verbose=False)
                        else:
                            results = self.model(frame, conf=self.conf, iou=self.iou, verbose=False)

                        # 复制两份画面用于分别绘制
                        frame_geo = results[0].plot(labels=False)
                        frame_da = frame_geo.copy()

                        if results[0].boxes:
                            active_ids = []
                            for box in results[0].boxes:
                                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                                cls_id = int(box.cls.item())
                                cls_name = self.model.names[cls_id]
                                obj_id = int(box.id.item()) if box.id is not None else None
                                if obj_id: active_ids.append(obj_id)

                                # --- A. 计算几何距离 (使用你的 0.23/0.31 参数) ---
                                geo_dist, _, _ = self.calculate_spatial_info((x1,y1,x2,y2), cls_id)
                                
                                # --- B. 计算 AI 距离 (使用 Depth Anything) ---
                                da_dist = geo_dist # 默认回退
                                if raw_depth_map is not None:
                                    # 坐标映射 frame(640x480) -> model(672x448)
                                    h_m, w_m = 448, 672
                                    h_i, w_i = frame.shape[:2]
                                    
                                    x1_m = max(0, int(x1 * w_m / w_i))
                                    y1_m = max(0, int(y1 * h_m / h_i))
                                    x2_m = min(w_m, int(x2 * w_m / w_i))
                                    y2_m = min(h_m, int(y2 * h_m / h_i))
                                    
                                    roi = raw_depth_map[y1_m:y2_m, x1_m:x2_m]
                                    if roi.size > 0:
                                        med = np.median(roi)
                                        if med > 0.01: 
                                            da_dist = DEPTH_COEFF * med
                                            #da_dist = DEPTH_COEFF / med

                                # --- C. 绘制左图 (Geometric) ---
                                box_in_geo = {
                                    'bbox_2d': (x1, y1, x2, y2),
                                    'depth_value': self._normalize_distance_to_depth_value(geo_dist),
                                    'depth_method': f"Geo: {geo_dist:.2f}m", # 标签
                                    'class_name': cls_name,
                                    'object_id': obj_id
                                }
                                self.estimator_geo.draw_box_3d(frame_geo, box_in_geo, (0, 255, 255)) # 黄框
                                
                                # 强制文字覆盖 (防止工具类没画出来)
                                cv2.putText(frame_geo, f"Geo: {geo_dist:.2f}m", (int(x1), int(y1)-30), 
                                            cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 255), 2)

                                # --- D. 绘制右图 (Depth Anything) ---
                                box_in_da = {
                                    'bbox_2d': (x1, y1, x2, y2),
                                    'depth_value': self._normalize_distance_to_depth_value(da_dist),
                                    'depth_method': f"DA: {da_dist:.2f}m", # 标签
                                    'class_name': cls_name,
                                    'object_id': obj_id
                                }
                                self.estimator_da.draw_box_3d(frame_da, box_in_da, (0, 255, 0)) # 绿框
                                
                                # 强制文字覆盖
                                cv2.putText(frame_da, f"DA: {da_dist:.2f}m", (int(x1), int(y1)-30), 
                                            cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)

                            if self.enable_trk:
                                self.estimator_geo.cleanup_trackers(active_ids)
                                self.estimator_da.cleanup_trackers(active_ids)

                        # 显示画面
                        self.container_geo.image(frame_geo, channels="BGR", caption="Geometric (Simple Math)")
                        self.container_da.image(frame_da, channels="BGR", caption="Depth Anything (AI Model)")
                        
                        fps = self.camera_fps.get(0, 0)
                        if len(self.camera_frame_times[0]) > 1:
                            fps = (len(self.camera_frame_times[0])-1) / (self.camera_frame_times[0][-1] - self.camera_frame_times[0][0])
                        self.fps_container.metric("Camera FPS", f"{fps:.1f}")

                frame_cnt += 1
                if frame_cnt % 30 == 0: print(".", end="", flush=True)
                time.sleep(0.001)

        finally:
            self.camera_running = False
            for t in self.camera_threads: t.join()
            for c in self.camera_caps: c.release()

    def inference(self) -> None:
        self.web_ui()
        self.sidebar()
        self.configure()
        if self.st.sidebar.button("Start Comparison"):
            self.multi_camera_inference()

if __name__ == "__main__":
    Inference().inference()