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

# === ROS 相关库 (保留用于显示，但不参与计算) ===
import rospy
from sensor_msgs.msg import Imu
from tf.transformations import euler_from_quaternion

# 确保使用本地修改的ultralytics版本
current_dir = os.path.dirname(os.path.abspath(__file__))
ultralytics_root = os.path.dirname(os.path.dirname(current_dir))
if ultralytics_root not in sys.path:
    sys.path.insert(0, ultralytics_root)

from ultralytics import YOLO
from ultralytics.utils import LOGGER
from ultralytics.utils.checks import check_requirements

# 导入 3D 辅助工具
try:
    from bbox3d_utils import BBox3DEstimator
except ImportError:
    LOGGER.error("❌ 无法导入 bbox3d_utils.py。请确保该文件存在于当前目录下。")
    sys.exit(1)

torch.classes.__path__ = []  # Torch module __path__._path issue fix


class Inference:
    """
    Streamlit Interface for YOLO object detection, optimized for Multi-Camera.
    Updated: Fusion Strategy (Aspect Ratio Based) for Distance Estimation (No IMU).
    """

    def __init__(self, **kwargs: Any) -> None:
        """
        Initialize the Inference class.
        """
        check_requirements("streamlit>=1.29.0")
        import streamlit as st

        self.st = st
        
        # === ROS IMU 初始化 (保留监听，仅用于Sidebar显示) ===
        self.current_pitch = 0.0 
        self._init_ros_listener()

        # 强制默认为多摄像头模式
        self.source = "multi-webcam" 
        self.enable_trk = False
        self.conf = 0.25
        self.iou = 0.45
        self.selected_ind: List[int] = []
        self.model = None

        # Multi-camera support attributes
        self.num_cameras = 4
        self.camera_indices = [0, 2, 4, 6]
        
        # 相机安装朝向偏移 (机体坐标系)
        self.camera_yaw_offsets = {
            0: 180.0,  # 后
            2: -90.0,  # 右
            4: 0.0,    # 前
            6: 90.0    # 左
        }

        self.frame_width = 640
        self.frame_height = 360 # 保持 360p
        self.camera_caps: List[cv2.VideoCapture] = []
        self.camera_threads: List[threading.Thread] = []
        self.camera_frames: Dict[int, np.ndarray] = {}
        self.camera_running = False
        self.camera_containers: Dict[int, Tuple[Any, Any]] = {}

        # === 空间测量参数 ===
        # 使用提供的标定值
        self.camera_intrinsics = {
            'fx': 498.0,
            'fy': 498.0,      
            'cx': 331.2797,
            'cy': 156.1371    # 你的标定光心 Y
        }
        
        # 几何法参数 (仅用于小车)
        self.CAM_HEIGHT = 0.15      # 相机离地高度
        self.OBJ_HEIGHT = 0.20      # 目标塔顶高度
        
        # 宽度法参数
        # 0-5 类: 小车 (0.31m)
        # 6 类: 篮球 (0.23m)
        self.class_real_widths = {
            6: 0.23,  
            **{i: 0.31 for i in range(6)} # 0-5 都是 0.31
        }
        self.default_real_width = 0.31

        # 3D 边界框估算器
        self.bbox3d_estimator = BBox3DEstimator()
        
        # 深度映射范围
        self.DEPTH_MIN = 0.5
        self.DEPTH_MAX = 8.0

        # FPS monitoring attributes
        self.camera_fps: Dict[int, float] = {}
        self.camera_frame_times: Dict[int, List[float]] = {}
        self.fps_containers: Dict[int, Any] = {}
        self.overall_fps_container = None

        # Detection result saving attributes
        self.save_detections = False
        self.save_images = True
        self.save_labels = True
        self.save_directory = "./detection_results"
        self.detection_count: Dict[int, int] = {}

        # Detection Statistics
        self.class_detection_stats: Dict[int, Dict[str, int]] = {}
        self.last_detection_time: Dict[int, float] = {}

        # Display options
        self.show_class_names = True

        self.temp_dict = {"model": None, **kwargs}
        self.model_path = None
        if self.temp_dict["model"] is not None:
            self.model_path = self.temp_dict["model"]

        LOGGER.info(f"Ultralytics Solutions: ✅ {self.temp_dict}")

    # === ROS 监听逻辑 ===
    def _init_ros_listener(self):
        try:
            if not rospy.core.is_initialized():
                rospy.init_node('yolo_inference_node', anonymous=True, disable_signals=True)
            rospy.Subscriber('/robot/imu', Imu, self._imu_callback)
            LOGGER.info("✅ ROS IMU Listener initialized.")
        except Exception as e:
            LOGGER.error(f"❌ ROS Init Failed: {e}")

    def _imu_callback(self, data):
        try:
            q = [data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w]
            (_, pitch, _) = euler_from_quaternion(q)
            self.current_pitch = pitch
        except:
            pass

    def _normalize_distance_to_depth_value(self, distance: float) -> float:
        if distance == float('inf') or distance is None:
            return 1.0
        dist = np.clip(distance, self.DEPTH_MIN, self.DEPTH_MAX)
        normalized_depth = (dist - self.DEPTH_MIN) / (self.DEPTH_MAX - self.DEPTH_MIN)
        return float(np.clip(normalized_depth, 0.0, 1.0))

    # === [核心修改] 融合测距逻辑 (无IMU, 基于宽高比检测) ===
    def calculate_spatial_info(self, bbox_xyxy, class_id, cam_index=0):
        """
        计算目标的距离和方位角，并进行融合
        :return: best_dist (m), dist_geo (m), dist_width (m), azimuth (deg), global_angle (deg)
        """
        x1, y1, x2, y2 = bbox_xyxy
        x_center_px = (x1 + x2) / 2
        box_width_px = x2 - x1
        box_height_px = y2 - y1

        # ---------------- 1. 宽度测距法 (Width-Based) ----------------
        real_width = self.class_real_widths.get(class_id, self.default_real_width)
        fx = self.camera_intrinsics['fx']
        
        if box_width_px > 0:
            dist_width = (real_width * fx) / box_width_px
        else:
            dist_width = 0.0
        
        # ---------------- 2. 静态几何测距法 (Static Geo-Based) ----------------
        dist_geo = None
        
        # 仅针对 0-5 类 (小车) 启用
        if 0 <= class_id <= 5: 
            # 倒立摄像头: 物理塔顶 = 图像下边缘 (y2)
            y_top_pixel = y2 
            fy = self.camera_intrinsics['fy']
            cy = self.camera_intrinsics['cy']
            
            # 几何投影计算
            v = y_top_pixel - cy
            alpha = math.atan(v / fy)
            
            # 保持无IMU逻辑
            FIXED_INSTALL_PITCH = 0.0 
            total_angle = alpha + FIXED_INSTALL_PITCH
            
            dH = self.OBJ_HEIGHT - self.CAM_HEIGHT
            
            # 简单的防除零保护
            if total_angle > 0.001:
                dist_geo = abs(dH / math.tan(total_angle))
            else:
                dist_geo = 99.9 

        # ---------------- 3. [融合逻辑] 宽高比检查 ----------------
        best_dist = dist_width # 默认兜底

        if dist_geo is not None and dist_geo < 20.0: # 排除极远噪点
            if box_height_px > 0:
                # 计算检测框宽高比
                current_ratio = box_width_px / box_height_px
                
                # 设定阈值: 你的车标准比例约 1.5 (0.31/0.2)
                # 如果比例小于 0.9，说明宽度被严重压缩(遮挡)
                OCCLUSION_RATIO_THRESHOLD = 0.9 
                
                if current_ratio < OCCLUSION_RATIO_THRESHOLD:
                    # 判定为遮挡 -> 强制信任几何法 (抗遮挡)
                    best_dist = dist_geo
                else:
                    # 判定为完整 -> 加权融合 (利用宽度法平滑颠簸)
                    # 0.4 Geo + 0.6 Width (宽度法权重高一点以减少抖动)
                    best_dist = 0.4 * dist_geo + 0.6 * dist_width
            else:
                 best_dist = dist_geo

        # ---------------- 4. 方位角计算 ----------------
        cx = self.camera_intrinsics['cx']
        pixel_offset = x_center_px - cx
        angle_rad = math.atan(pixel_offset / fx)
        azimuth_deg = math.degrees(angle_rad)
        cam_offset = self.camera_yaw_offsets.get(cam_index, 0.0)
        global_angle = (azimuth_deg + cam_offset + 360) % 360

        return best_dist, dist_geo, dist_width, azimuth_deg, global_angle

    def web_ui(self) -> None:
        """Set up the Streamlit web interface."""
        menu_style_cfg = """<style>MainMenu {visibility: hidden;}</style>"""

        main_title_cfg = """<div><h1 style="color:#111F68; text-align:center; font-size:40px; margin-top:-50px;
        font-family: 'Archivo', sans-serif; margin-bottom:20px;">Multi-Camera Spatial Detection</h1></div>"""

        sub_title_cfg = """<div><h5 style="color:#042AFF; text-align:center; font-family: 'Archivo', sans-serif;
        margin-top:-15px; margin-bottom:50px;">Real-time detection & Pseudo 3D Distance Estimation 🚗📹📏</h5></div>"""

        self.st.set_page_config(page_title="Ultralytics Streamlit App", layout="wide")
        self.st.markdown(menu_style_cfg, unsafe_allow_html=True)
        self.st.markdown(main_title_cfg, unsafe_allow_html=True)
        self.st.markdown(sub_title_cfg, unsafe_allow_html=True)

    def sidebar(self) -> None:
        """Configure the Streamlit sidebar settings."""
        with self.st.sidebar:
            logo = "https://raw.githubusercontent.com/ultralytics/assets/main/logo/Ultralytics_Logotype_Original.svg"
            self.st.image(logo, width=250)

        self.st.sidebar.title("User Configuration")
        self.st.sidebar.markdown("### 🎥 Source: **Multi-Webcam**")

        # Camera configuration
        self.st.sidebar.subheader("Camera Settings")
        self.st.sidebar.info("💡 **Multi-Camera Setup**: Optimized for 4 cameras.")
        self.num_cameras = self.st.sidebar.slider("Number of Cameras", 1, 8, 4)

        if self.num_cameras <= 4:
            available_cameras = [0, 2, 4, 6]
            self.camera_indices = available_cameras[:self.num_cameras]
        else:
            preset_configs = {
                "Sequential (0,1,2,3,4,5,6,7)": ",".join(map(str, range(self.num_cameras))),
                "Reliable (0,2,4,6)": "0,2,4,6",
                "Custom": "custom"
            }
            selected_preset = self.st.sidebar.selectbox("Camera Configuration", list(preset_configs.keys()))

            if selected_preset == "Custom":
                camera_indices_str = self.st.sidebar.text_input(
                    "Camera Indices (comma-separated)",
                    ",".join(map(str, self.camera_indices[:self.num_cameras]))
                )
            else:
                camera_indices_str = preset_configs[selected_preset]

            try:
                self.camera_indices = [int(x.strip()) for x in camera_indices_str.split(",")][:self.num_cameras]
                self.num_cameras = len(self.camera_indices)
            except ValueError:
                self.st.sidebar.error("Invalid camera indices format. Using default values.")
                self.camera_indices = [0, 2, 4, 6]
                self.num_cameras = 4

        # Resolution settings
        self.frame_width = 640
        self.frame_height = 360 # 保持 360p
        self.st.sidebar.info(f"🎯 **Resolution**: {self.frame_width}x{self.frame_height}")

        self.enable_trk = self.st.sidebar.radio("Enable Tracking", ("Yes", "No")) == "Yes"

        self.conf = float(self.st.sidebar.slider("Confidence Threshold", 0.0, 1.0, self.conf, 0.01))
        self.iou = float(self.st.sidebar.slider("IoU Threshold", 0.0, 1.0, self.iou, 0.01))

        # Display options
        self.st.sidebar.subheader("Display Options")
        self.show_class_names = self.st.sidebar.checkbox(
            "Show Class Names",
            value=self.show_class_names,
            help="If unchecked, only class IDs will be shown instead of class names"
        )
        self.show_distance = self.st.sidebar.checkbox(
            "Show Distance & Pseudo 3D Box",
            value=True,
            help="Show calculated distance (m) and angle (deg) using a pseudo 3D box"
        )
        # 显示实时 Pitch (仅作参考，不参与计算)
        self.st.sidebar.metric("Ref IMU Pitch (deg)", f"{math.degrees(self.current_pitch):.2f}")

        # Detection saving configuration
        self.st.sidebar.subheader("Detection Saving")
        self.save_detections = self.st.sidebar.checkbox("Save Detection Results", value=self.save_detections)

        if self.save_detections:
            self.save_directory = self.st.sidebar.text_input("Save Directory", value=self.save_directory)
            self.save_images = self.st.sidebar.checkbox("Save Images", value=self.save_images)
            self.save_labels = self.st.sidebar.checkbox("Save Labels (YOLO format)", value=self.save_labels)

            if self.save_directory:
                Path(self.save_directory).mkdir(parents=True, exist_ok=True)
                self.st.sidebar.success(f"Save directory: {self.save_directory}")

        # Create UI layout
        self.overall_fps_container = self.st.empty()
        cols_per_row = 2 if self.num_cameras <= 4 else 3
        rows = (self.num_cameras + cols_per_row - 1) // cols_per_row

        for row in range(rows):
            cols = self.st.columns(cols_per_row)
            for col_idx in range(cols_per_row):
                cam_idx = row * cols_per_row + col_idx
                if cam_idx < self.num_cameras:
                    with cols[col_idx]:
                        self.st.markdown(f"**Camera {self.camera_indices[cam_idx]}**")
                        fps_container = self.st.empty()
                        org_container = self.st.empty()
                        ann_container = self.st.empty()
                        self.camera_containers[cam_idx] = (org_container, ann_container)
                        self.fps_containers[cam_idx] = fps_container

    def source_upload(self) -> None:
        pass

    def configure(self) -> None:
        """Configure the model and load selected classes."""
        available_models = ["Custom Car Detection Model (best.engine)"]
        custom_model_path = "/home/nvidia/Downloads/Ros/ballCar2/weights/weights/best.engine"

        if self.model_path:
            available_models.insert(0, self.model_path)

        selected_model = self.st.sidebar.selectbox("Model", available_models, index=0)

        with self.st.spinner("Model is loading..."):
            if selected_model == "Custom Car Detection Model (best.engine)":
                model_path = custom_model_path
            elif selected_model.endswith((".pt", ".onnx", ".torchscript", ".mlpackage", ".engine")):
                model_path = selected_model
            else:
                model_path = custom_model_path

            self.model = YOLO(model_path)
            class_names = list(self.model.names.values())
        self.st.success("Model loaded successfully!")

        selected_classes = self.st.sidebar.multiselect("Classes", class_names, default=class_names)
        self.selected_ind = [class_names.index(option) for option in selected_classes]
        if not isinstance(self.selected_ind, list):
            self.selected_ind = list(self.selected_ind)

    def setup_cameras(self) -> bool:
        """Initialize multiple cameras."""
        self.camera_caps = []
        self.camera_frames = {}
        self.camera_fps = {}
        self.camera_frame_times = {}
        successful_cameras = 0

        for i, cam_idx in enumerate(self.camera_indices):
            try:
                cap = cv2.VideoCapture(cam_idx)
                if cap.isOpened():
                    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
                    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
                    cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.frame_width)
                    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.frame_height)
                    time.sleep(0.1)

                    test_success = False
                    for _ in range(3):
                        ret, test_frame = cap.read()
                        if ret and test_frame is not None and test_frame.size > 0:
                            test_success = True
                            break
                        time.sleep(0.05)

                    if test_success:
                        actual_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
                        actual_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
                        self.camera_caps.append(cap)
                        self.camera_frames[i] = np.zeros((actual_height, actual_width, 3), dtype=np.uint8)
                        self.camera_fps[i] = 0.0
                        self.camera_frame_times[i] = []
                        successful_cameras += 1
                        LOGGER.info(f"Camera {cam_idx} initialized successfully: {actual_width}x{actual_height}")
                    else:
                        self.camera_caps.append(None)
                        cap.release()
                        LOGGER.warning(f"Camera {cam_idx} opened but cannot read frames")
                else:
                    self.camera_caps.append(None)
                    LOGGER.warning(f"Failed to open camera {cam_idx}")
                time.sleep(0.2)
            except Exception as e:
                self.camera_caps.append(None)
                LOGGER.error(f"Error initializing camera {cam_idx}: {e}")

        return successful_cameras > 0

    def camera_thread(self, camera_index: int, cap: cv2.VideoCapture) -> None:
        """Thread function to capture frames from a single camera."""
        consecutive_failures = 0
        max_failures = 10

        while self.camera_running and cap is not None and cap.isOpened():
            try:
                ret, frame = cap.read()
                if ret and frame is not None and frame.size > 0:
                    self.camera_frames[camera_index] = frame.copy()

                    current_time = time.time()
                    if camera_index in self.camera_frame_times:
                        self.camera_frame_times[camera_index].append(current_time)
                        if len(self.camera_frame_times[camera_index]) > 30:
                            self.camera_frame_times[camera_index].pop(0)

                        if len(self.camera_frame_times[camera_index]) > 1:
                            time_diff = self.camera_frame_times[camera_index][-1] - self.camera_frame_times[camera_index][0]
                            if time_diff > 0:
                                self.camera_fps[camera_index] = (len(self.camera_frame_times[camera_index]) - 1) / time_diff

                    consecutive_failures = 0
                else:
                    consecutive_failures += 1
                    if consecutive_failures >= max_failures:
                        LOGGER.error(f"Camera {camera_index} failed max times, stopping thread")
                        break
                time.sleep(0.01)
            except Exception as e:
                consecutive_failures += 1
                LOGGER.error(f"Error in camera thread {camera_index}: {e}")
                if consecutive_failures >= max_failures:
                    break
                time.sleep(0.1)

    def multi_camera_inference(self) -> None:
        """Perform inference on multiple cameras."""
        self.st.info(f"🎥 Attempting to initialize {len(self.camera_indices)} cameras: {self.camera_indices}")

        if not self.setup_cameras():
            self.st.error("❌ Failed to initialize any cameras.")
            return

        working_cameras = sum(1 for cap in self.camera_caps if cap is not None)
        self.st.success(f"✅ {working_cameras} cameras initialized successfully!")

        self.camera_running = True
        self.camera_threads = []

        for i, cap in enumerate(self.camera_caps):
            if cap is not None:
                thread = threading.Thread(target=self.camera_thread, args=(i, cap))
                thread.daemon = True
                thread.start()
                self.camera_threads.append(thread)

        stop_button = self.st.sidebar.button("Stop")

        try:
            inference_start_time = time.time()
            total_frames_processed = 0

            while self.camera_running:
                if stop_button:
                    break
                
                active_ids = []

                for cam_idx in range(len(self.camera_caps)):
                    if cam_idx in self.camera_frames and cam_idx in self.camera_containers:
                        frame = self.camera_frames[cam_idx].copy()
                        real_cam_id = self.camera_indices[cam_idx]

                        if frame is not None and frame.size > 0:
                            if self.enable_trk:
                                results = self.model.track(
                                    frame, conf=self.conf, iou=self.iou, classes=self.selected_ind,
                                    persist=True, imgsz=(480, 640)
                                )
                            else:
                                results = self.model(frame, conf=self.conf, iou=self.iou, classes=self.selected_ind,
                                                     imgsz=(480, 640))

                            annotated_frame = results[0].plot(labels=self.show_class_names)

                            # --- START: 绘制 Pseudo 3D 框和距离信息 ---
                            if self.show_distance and results[0].boxes is not None:
                                for box in results[0].boxes:
                                    x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                                    class_id = int(box.cls.item())
                                    class_name = self.model.names[class_id]
                                    obj_id = box.id.item() if box.id is not None else None
                                    
                                    if obj_id is not None:
                                        active_ids.append(obj_id)

                                    # 2. 计算双重距离 + 融合逻辑 (返回 5 个值)
                                    best_dist, dist_geo, dist_width, azim, global_ang = self.calculate_spatial_info(
                                        (x1, y1, x2, y2),
                                        class_id,
                                        real_cam_id
                                    )

                                    # 3. 决定显示的 Label
                                    # 显示 Fusion 结果以及 Geo 和 Width 的差值
                                    if dist_geo is not None:
                                        diff = abs(dist_geo - dist_width)
                                        label_str = f"D:{best_dist:.2f} (Diff:{diff:.1f})"
                                    else:
                                        label_str = f"D:{best_dist:.2f}m"

                                    depth_value_norm = self._normalize_distance_to_depth_value(best_dist)
                                    
                                    box_3d_input = {
                                        'bbox_2d': (x1, y1, x2, y2),
                                        'depth_value': depth_value_norm,
                                        'depth_method': label_str, # <--- 显示融合结果
                                        'class_name': class_name,
                                        'object_id': obj_id
                                    }
                                    
                                    if 'car' in class_name.lower():
                                        color = (0, 0, 255)
                                    elif 'ball' in class_name.lower() or class_id == 6:
                                        color = (0, 165, 255) # 橙色
                                    else:
                                        color = (0, 0, 255)
                                        
                                    annotated_frame = self.bbox3d_estimator.draw_box_3d(
                                        annotated_frame, box_3d_input, color=color
                                    )
                            # --- END ---

                            self.save_detection_results(results, frame, real_cam_id)

                            org_container, ann_container = self.camera_containers[cam_idx]
                            org_container.image(frame, channels="BGR", caption=f"Camera {real_cam_id} - Original")
                            ann_container.image(annotated_frame, channels="BGR",
                                                caption=f"Camera {real_cam_id} - Predicted")

                            if cam_idx in self.fps_containers:
                                self.fps_containers[cam_idx].metric(
                                    f"Camera {real_cam_id} FPS",
                                    f"{self.camera_fps.get(cam_idx, 0.0):.1f}"
                                )

                            total_frames_processed += 1
                
                if self.enable_trk:
                    self.bbox3d_estimator.cleanup_trackers(active_ids)

                if self.overall_fps_container:
                    elapsed_time = time.time() - inference_start_time
                    if elapsed_time > 0:
                        overall_fps = total_frames_processed / elapsed_time
                        avg_camera_fps = sum(self.camera_fps.values()) / len(
                            self.camera_fps) if self.camera_fps else 0

                        self.overall_fps_container.markdown(f"""
                        ### 📊 Performance Metrics
                        - **Overall Processing FPS**: {overall_fps:.1f}
                        - **Average Camera FPS**: {avg_camera_fps:.1f}
                        - **Active Cameras**: {working_cameras}
                        """)

                time.sleep(0.01)

        finally:
            self.camera_running = False
            for thread in self.camera_threads:
                if thread.is_alive():
                    thread.join(timeout=1.0)
            for cap in self.camera_caps:
                if cap is not None:
                    cap.release()

    def save_detection_results(self, results, frame: np.ndarray, camera_id: int) -> None:
        """Save detection results to specified directory."""
        if not self.save_detections or not results[0].boxes:
            return

        detected_classes = []
        if results[0].boxes is not None:
            for box in results[0].boxes:
                class_id = int(box.cls.item())
                class_name = self.model.names[class_id]
                detected_classes.append(class_name)

        current_time = time.time()

        if camera_id not in self.class_detection_stats:
            self.class_detection_stats[camera_id] = {}

        for class_name in detected_classes:
            if class_name not in self.class_detection_stats[camera_id]:
                self.class_detection_stats[camera_id][class_name] = 0
            self.class_detection_stats[camera_id][class_name] += 1

        self.last_detection_time[camera_id] = current_time

        if camera_id not in self.detection_count:
            self.detection_count[camera_id] = 0

        camera_dir = Path(self.save_directory) / f"camera_{camera_id}"
        camera_dir.mkdir(parents=True, exist_ok=True)

        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]
        self.detection_count[camera_id] += 1
        base_filename = f"detection_{timestamp}_{self.detection_count[camera_id]:04d}"

        if self.save_images:
            img_path = camera_dir / f"{base_filename}.jpg"
            annotated_frame_save = frame.copy()
            
            if results[0].boxes is not None:
                for box in results[0].boxes:
                    x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                    class_id = int(box.cls.item())
                    class_name = self.model.names[class_id]
                    obj_id = box.id.item() if box.id is not None else None
                    
                    # 融合逻辑
                    best_dist, dist_geo, dist_width, azim, global_ang = self.calculate_spatial_info(
                        (x1, y1, x2, y2), class_id, camera_id
                    )
                    
                    if dist_geo is not None:
                        diff = abs(dist_geo - dist_width)
                        label_str = f"D:{best_dist:.2f} (Diff:{diff:.1f})"
                    else:
                        label_str = f"D:{best_dist:.2f}m"

                    depth_value_norm = self._normalize_distance_to_depth_value(best_dist)

                    box_3d_input = {
                        'bbox_2d': (x1, y1, x2, y2),
                        'depth_value': depth_value_norm,
                        'depth_method': label_str,
                        'class_name': class_name,
                        'object_id': obj_id
                    }

                    if 'car' in class_name.lower():
                        color = (0, 0, 255)  
                    elif 'ball' in class_name.lower():
                        color = (0, 165, 255)
                    else:
                        color = (255, 255, 255)
                        
                    annotated_frame_save = self.bbox3d_estimator.draw_box_3d(
                        annotated_frame_save, box_3d_input, color=color
                    )

            cv2.imwrite(str(img_path), annotated_frame_save)

        if self.save_labels:
            label_path = camera_dir / f"{base_filename}.txt"
            self._save_yolo_labels(results[0], label_path, frame.shape, camera_id)

    def _save_yolo_labels(self, result, label_path: Path, img_shape: tuple, cam_id=0) -> None:
        """Save detection results in YOLO format + Dual Distance Info + Fused Distance."""
        if not result.boxes:
            return

        height, width = img_shape[:2]

        with open(label_path, 'w') as f:
            for box in result.boxes:
                class_id = int(box.cls.item())
                confidence = float(box.conf.item())
                x1, y1, x2, y2 = box.xyxy[0].tolist()

                x_center = (x1 + x2) / 2 / width
                y_center = (y1 + y2) / 2 / height
                bbox_width = (x2 - x1) / width
                bbox_height = (y2 - y1) / height

                best_dist, dist_geo, dist_width, azim, global_ang = self.calculate_spatial_info((x1, y1, x2, y2), class_id, cam_id)
                
                # 处理 Geo 为 None 的情况
                d_g = dist_geo if dist_geo is not None else -1.0
                
                # 格式追加了一列 best_dist: class x y w h conf dist_geo dist_width azimuth global_angle best_dist
                f.write(
                    f"{class_id} {x_center:.6f} {y_center:.6f} {bbox_width:.6f} {bbox_height:.6f} {confidence:.6f} {d_g:.3f} {dist_width:.3f} {azim:.1f} {global_ang:.1f} {best_dist:.3f}\n")

    def inference(self) -> None:
        """Perform real-time object detection inference."""
        self.web_ui()
        self.sidebar()
        self.source_upload()
        self.configure()

        if self.st.sidebar.button("Start"):
            self.multi_camera_inference()
        else:
            self.st.info("Click 'Start' in the sidebar to begin Multi-Camera Detection.")


if __name__ == "__main__":
    args = len(sys.argv)
    model = sys.argv[1] if args > 1 else None
    Inference(model=model).inference()