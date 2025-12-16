import sklearn
from sklearn import metrics # [重要] 必须放在第一行，修复 TLS 报错
import streamlit as st
import cv2
import numpy as np
import torch
import time
import sys
import os

# 忽略 Apple Silicon 设置
if hasattr(torch, 'backends') and hasattr(torch.backends, 'mps') and torch.backends.mps.is_available():
    os.environ['PYTORCH_ENABLE_MPS_FALLBACK'] = '1'

# 导入你的模块
from detection_model import ObjectDetector
from depth_model import DepthEstimator
from bbox3d_utils import BBox3DEstimator, BirdEyeView

# --- 页面配置 ---
st.set_page_config(page_title="VSWARM 3D Perception", layout="wide")
st.title("🤖 VSWARM: 3D Object Detection & Depth")
st.markdown("YOLOv11 + Depth Anything V2 | Running on Jetson")

# --- 侧边栏设置 ---
st.sidebar.header("系统设置")
camera_index = st.sidebar.selectbox("摄像头 ID", [0, 1, 2, 3], index=0)
conf_threshold = st.sidebar.slider("检测阈值 (Confidence)", 0.1, 1.0, 0.40)
enable_bev = st.sidebar.checkbox("显示俯视图 (BEV)", value=True)
enable_depth_overlay = st.sidebar.checkbox("显示深度图 (画中画)", value=True)

# --- 模型加载 (使用缓存，避免重复加载) ---
@st.cache_resource
def load_models():
    print("🚀 Loading AI Models...")
    device = 'cpu' # 强制 CPU 保证稳定性
    
    # 加载检测器
    try:
        detector = ObjectDetector(model_size="nano", conf_thres=0.25, device=device)
    except Exception as e:
        st.error(f"Detector Load Error: {e}")
        detector = ObjectDetector(model_size="nano", conf_thres=0.25, device='cpu')
    
    # 加载深度估计器
    try:
        depth_estimator = DepthEstimator(model_size="small", device=device)
    except Exception as e:
        st.error(f"Depth Load Error: {e}")
        depth_estimator = DepthEstimator(model_size="small", device='cpu')
        
    bbox3d_estimator = BBox3DEstimator()
    bev = BirdEyeView(scale=60, size=(300, 300))
    
    print("✅ Models Loaded!")
    return detector, depth_estimator, bbox3d_estimator, bev

# 加载模型
with st.spinner('正在初始化 AI 引擎 (首次运行需要下载模型)...'):
    detector, depth_estimator, bbox3d_estimator, bev = load_models()
    # 动态更新阈值
    detector.conf_thres = conf_threshold

# --- 主界面布局 ---
col1, col2 = st.columns([3, 1])
with col1:
    st_frame = st.empty() # 视频画面容器
with col2:
    st_fps = st.empty()   # FPS 显示容器
    st_info = st.empty()  # 检测信息容器
    start_btn = st.button("▶️ 开始推流 (Start)", type="primary")
    stop_btn = st.button("⏹️ 停止推流 (Stop)")

def main():
    if start_btn:
        # 尝试打开摄像头 (优先使用 V4L2 后端)
        cap = cv2.VideoCapture(camera_index, cv2.CAP_V4L2)
        if not cap.isOpened():
            cap = cv2.VideoCapture(camera_index)
            
        if not cap.isOpened():
            st.error(f"❌ 无法打开摄像头 {camera_index}。请检查连接或尝试其他 ID。")
            return

        # 降低分辨率以提升流传输速度
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

        width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        
        frame_count = 0
        start_time = time.time()

        try:
            while cap.isOpened():
                if stop_btn:
                    break

                ret, frame = cap.read()
                if not ret:
                    st.warning("无法读取视频帧")
                    break
                
                # 复制帧用于不同处理
                original_frame = frame.copy()
                detection_frame = frame.copy()
                result_frame = frame.copy()
                
                # --- 1. 目标检测 ---
                detection_frame, detections = detector.detect(detection_frame, track=True)
                
                # --- 2. 深度估计 ---
                depth_map = depth_estimator.estimate_depth(original_frame)
                
                # --- 3. 3D 计算 ---
                boxes_3d = []
                active_ids = []
                
                for detection in detections:
                    bbox, score, class_id, obj_id = detection
                    class_name = detector.get_class_names()[class_id]
                    
                    # 深度采样策略
                    if class_name.lower() in ['person', 'cat', 'dog']:
                        center_x = int((bbox[0] + bbox[2]) / 2)
                        center_y = int((bbox[1] + bbox[3]) / 2)
                        depth_value = depth_estimator.get_depth_at_point(depth_map, center_x, center_y)
                        depth_method = 'center'
                    else:
                        depth_value = depth_estimator.get_depth_in_region(depth_map, bbox, method='median')
                        depth_method = 'median'
                    
                    box_3d = {
                        'bbox_2d': bbox, 'depth_value': depth_value, 'depth_method': depth_method,
                        'class_name': class_name, 'object_id': obj_id, 'score': score
                    }
                    boxes_3d.append(box_3d)
                    if obj_id is not None: active_ids.append(obj_id)
                
                # 清理追踪器
                bbox3d_estimator.cleanup_trackers(active_ids)
                
                # --- 4. 绘制 3D 框 ---
                for box_3d in boxes_3d:
                    class_name = box_3d['class_name'].lower()
                    color = (0, 255, 0) if 'person' in class_name else (255, 100, 0) # 人是绿色，其他是蓝色
                    result_frame = bbox3d_estimator.draw_box_3d(result_frame, box_3d, color=color)
                
                # --- 5. 叠加俯视图 (BEV) ---
                if enable_bev:
                    bev.reset()
                    for box_3d in boxes_3d: bev.draw_box(box_3d)
                    bev_image = bev.get_image()
                    
                    # 调整大小放在左下角
                    bev_h = height // 3
                    bev_w = bev_h
                    bev_resized = cv2.resize(bev_image, (bev_w, bev_h))
                    
                    # 简单的遮罩叠加
                    result_frame[height - bev_h:height, 0:bev_w] = bev_resized
                    cv2.rectangle(result_frame, (0, height - bev_h), (bev_w, height), (255, 255, 255), 2)
                    cv2.putText(result_frame, "BEV Map", (5, height - bev_h + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255), 1)

                # --- 6. 叠加深度图 (画中画) ---
                if enable_depth_overlay:
                    depth_colored = depth_estimator.colorize_depth(depth_map)
                    d_h = height // 4
                    d_w = int(d_h * (width / height))
                    d_resized = cv2.resize(depth_colored, (d_w, d_h))
                    # 放在左上角
                    result_frame[0:d_h, 0:d_w] = d_resized
                    cv2.rectangle(result_frame, (0, 0), (d_w, d_h), (255, 255, 255), 1)

                # --- 计算 FPS ---
                frame_count += 1
                elapsed = time.time() - start_time
                fps = frame_count / elapsed if elapsed > 0 else 0
                
                # BGR 转 RGB 供网页显示
                frame_rgb = cv2.cvtColor(result_frame, cv2.COLOR_BGR2RGB)
                
                # --- 更新网页 ---
                st_frame.image(frame_rgb, channels="RGB", use_container_width=True)
                
                st_fps.metric("实时帧率 (FPS)", f"{fps:.1f}")
                
                # 显示检测到的物体列表
                if boxes_3d:
                    info_text = "**检测物体:**\n"
                    for b in boxes_3d:
                        dist = b['depth_value']
                        name = b['class_name']
                        info_text += f"- {name}: {dist:.2f}m\n"
                    st_info.markdown(info_text)
                else:
                    st_info.markdown("*暂无目标*")

        except Exception as e:
            st.error(f"运行出错: {e}")
        finally:
            cap.release()
            st.info("摄像头已释放")

if __name__ == "__main__":
    main()