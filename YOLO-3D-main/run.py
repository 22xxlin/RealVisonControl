#!/usr/bin/env python3
import sklearn
from sklearn import metrics # 1. 强制加载底层库，修复 TLS 报错
import os
import sys
import time
import cv2
import numpy as np
import torch
from pathlib import Path

# 忽略 Apple Silicon 的相关设置，因为你在 Jetson 上
if hasattr(torch, 'backends') and hasattr(torch.backends, 'mps') and torch.backends.mps.is_available():
    os.environ['PYTORCH_ENABLE_MPS_FALLBACK'] = '1'

# Import our modules
from detection_model import ObjectDetector
from depth_model import DepthEstimator
from bbox3d_utils import BBox3DEstimator, BirdEyeView
from load_camera_params import load_camera_params, apply_camera_params_to_estimator

def main():
    """Main function."""
    # Configuration variables
    # ===============================================
    
    # Input/Output
    source = 0  # 0 对应 USB 摄像头 /dev/video0
    output_path = "output.mp4"  # 结果将保存为这个文件
    
    # Model settings
    yolo_model_size = "nano" 
    depth_model_size = "small"
    
    # Device settings
    device = 'cpu'  # 保持 CPU 以确保稳定性
    
    # Detection settings
    conf_threshold = 0.25
    iou_threshold = 0.45
    classes = None 
    
    # Feature toggles
    enable_tracking = True
    enable_bev = True
    
    # ===============================================
    
    print(f"Using device: {device}")
    
    # Initialize models
    print("Initializing models...")
    try:
        detector = ObjectDetector(model_size=yolo_model_size, conf_thres=conf_threshold, iou_thres=iou_threshold, classes=classes, device=device)
    except Exception as e:
        print(f"Error initializing object detector: {e}, falling back to CPU")
        detector = ObjectDetector(model_size=yolo_model_size, conf_thres=conf_threshold, iou_thres=iou_threshold, classes=classes, device='cpu')
    
    try:
        depth_estimator = DepthEstimator(model_size=depth_model_size, device=device)
    except Exception as e:
        print(f"Error initializing depth estimator: {e}, falling back to CPU")
        depth_estimator = DepthEstimator(model_size=depth_model_size, device='cpu')
    
    bbox3d_estimator = BBox3DEstimator()
    
    if enable_bev:
        bev = BirdEyeView(scale=60, size=(300, 300))
    
    # Open video source
    print(f"Opening video source: {source}")
    cap = cv2.VideoCapture(source)
    
    if not cap.isOpened():
        print(f"Error: Could not open video source {source}")
        return
    
    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    fps = int(cap.get(cv2.CAP_PROP_FPS))
    if fps == 0: fps = 30
    
    # Initialize video writer
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    out = cv2.VideoWriter(output_path, fourcc, fps, (width, height))
    
    frame_count = 0
    start_time = time.time()
    
    print("Starting processing... (Press Ctrl+C to stop)")
    print(f"Recording to: {output_path}")

    try:
        while True:
            # Read frame
            ret, frame = cap.read()
            if not ret:
                print("\nEnd of stream or error reading frame.")
                break
            
            original_frame = frame.copy()
            detection_frame = frame.copy()
            result_frame = frame.copy()
            
            # --- 核心处理逻辑 ---
            # 1. Object Detection
            detection_frame, detections = detector.detect(detection_frame, track=enable_tracking)
            
            # 2. Depth Estimation
            depth_map = depth_estimator.estimate_depth(original_frame)
            depth_colored = depth_estimator.colorize_depth(depth_map)
            
            # 3. 3D Bounding Box
            boxes_3d = []
            active_ids = []
            for detection in detections:
                bbox, score, class_id, obj_id = detection
                class_name = detector.get_class_names()[class_id]
                
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
            
            bbox3d_estimator.cleanup_trackers(active_ids)
            
            # 4. Visualization (Draw on result_frame)
            for box_3d in boxes_3d:
                class_name = box_3d['class_name'].lower()
                color = (0, 255, 0) if 'person' in class_name else (255, 255, 255)
                result_frame = bbox3d_estimator.draw_box_3d(result_frame, box_3d, color=color)
            
            # BEV Overlay
            if enable_bev:
                bev.reset()
                for box_3d in boxes_3d: bev.draw_box(box_3d)
                bev_image = bev.get_image()
                bev_height = height // 4
                bev_width = bev_height
                if bev_height > 0:
                    bev_resized = cv2.resize(bev_image, (bev_width, bev_height))
                    result_frame[height - bev_height:height, 0:bev_width] = bev_resized
                    cv2.rectangle(result_frame, (0, height - bev_height), (bev_width, height), (255, 255, 255), 1)

            # FPS Calculation
            frame_count += 1
            elapsed_time = time.time() - start_time
            if elapsed_time > 0:
                fps_value = frame_count / elapsed_time
            else:
                fps_value = 0
            
            # --- 关键修改：强制刷新打印缓存，确保你能看到进度 ---
            print(f"\rFrame: {frame_count} | FPS: {fps_value:.1f} | Objects: {len(boxes_3d)}", end="", flush=True)
            
            # 也在视频里写上FPS
            cv2.putText(result_frame, f"FPS: {fps_value:.1f}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

            # Depth Overlay (Picture in Picture)
            depth_height = height // 4
            depth_width = depth_height * width // height
            depth_resized = cv2.resize(depth_colored, (depth_width, depth_height))
            result_frame[0:depth_height, 0:depth_width] = depth_resized

            # --- 保存视频 ---
            out.write(result_frame)
            
            # 注意：没有任何 cv2.imshow 或 cv2.waitKey

    except KeyboardInterrupt:
        print("\n\nStopping recording (User Interrupt)...")
    except Exception as e:
        print(f"\n\nAn error occurred: {e}")
    finally:
        # Clean up
        print("Cleaning up resources...")
        if 'cap' in locals() and cap.isOpened():
            cap.release()
        if 'out' in locals() and out.isOpened():
            out.release()
        
        # --- 关键修改：彻底删除了 cv2.destroyAllWindows() ---
        
        print(f"Processing complete. Output saved to {output_path}")

if __name__ == "__main__":
    main()