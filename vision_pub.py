#!/usr/bin/env python3
"""
文件: vision_pub.py (V3.1 调试版)
功能: 视觉感知发布
升级点:
  1. 输出 fused (融合), geo (几何), width (宽度) 三种距离供数据记录。
  2. 修复了 Tuple 对比 Float 的类型错误。
  3. ZMQ 输出限频 (10Hz).
"""

import os
import sys
import cv2
import numpy as np
import time
import math
import json
import zmq
import queue
import threading
from collections import defaultdict, deque
from ultralytics import YOLO

# ================= 配置区域 =================
MODEL_PATH = "/home/nvidia/Downloads/Ros/ballCar2/weights/weights/best.engine"
CAMERA_INDICES = [0, 2, 4, 6]
ZMQ_PORT = 5555
PUBLISH_RATE_LIMIT = 0.05  # 限制 ZMQ 发送间隔 (10Hz)
STATE_LOCK_DURATION = 0.5 # 状态切换锁定时间

# === 物理与几何参数 ===
# 真实宽度 (单位: 米)
CLASS_REAL_WIDTHS = {
    "car": 0.31,
    "basketball": 0.23,
    "flag": 0.13
}

# 几何测距高度参数 (单位: 米)
CAM_HEIGHT = 0.15      # 相机离地高度
OBJ_HEIGHT = 0.20      # 小车塔顶高度 (仅用于 0-5 类)

# 相机内参 (请根据实际标定修改)
CAM_INTRINSICS = {
    'fx': 498.0,
    'fy': 498.0,
    'cx': 331.2797,
    'cy': 156.1371
}

# Class ID 映射
CLS_MAP = {
    0: "OFF", 1: "BLUE", 2: "RED", 3: "GREEN", 
    4: "PURPLE", 5: "GRAY", 6: "BALL", 7: "FLAG"
}

# 相机朝向
CAM_MOUNT_YAW_DEG = {0: 180.0, 2: -90.0, 4: 0.0, 6: 90.0}

# ================= 辅助函数 =================
def calculate_azimuth_planar(x_pixel, fx, cx):
    pixel_offset = x_pixel - cx
    angle_rad = math.atan(pixel_offset / fx)
    angle_deg = math.degrees(angle_rad)
    return (angle_deg + 360.0) % 360.0 if angle_deg < 0 else angle_deg

def wrap_deg_360(a):
    return (a + 360.0) % 360.0

# ================= 主类定义 =================
class VisionPublisher:
    def __init__(self, model_path, camera_indices, zmq_port=5555, debounce_maxlen=30):
        self.model_path = model_path
        self.camera_indices = camera_indices
        self.zmq_port = zmq_port
        
        # 加载内参
        self.fx = CAM_INTRINSICS['fx']
        self.fy = CAM_INTRINSICS['fy']
        self.cx = CAM_INTRINSICS['cx']
        self.cy = CAM_INTRINSICS['cy']

        self.queue = queue.Queue(maxsize=100)
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.PUB)
        self.socket.bind(f"tcp://*:{self.zmq_port}")
        
        # 历史缓存: Key=(cam_idx, track_id)
        self.history = defaultdict(lambda: deque(maxlen=debounce_maxlen))
        
        # 状态记忆
        self.state_memory = defaultdict(lambda: {'state': 0, 'pattern': 'OFF', 'last_switch_time': 0.0})
        
        # 发布限频
        self.last_pub_time = 0.0

        print(f"✅ 视觉发布者启动 (调试版 V3.1) | 模型: {self.model_path}")

    def initialize_camera(self, cam_idx):
        try:
            cap = cv2.VideoCapture(cam_idx, cv2.CAP_V4L2)
            if not cap.isOpened(): cap = cv2.VideoCapture(cam_idx)
            if cap.isOpened():
                cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
                cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
                cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
                cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
                return cap
        except Exception: pass
        return None

    def get_stable_state(self, cam_idx, track_id, current_class_id):
        """
        迟滞比较 (Hysteresis) + 状态锁定
        """
        key = (cam_idx, track_id)
        mem = self.state_memory[key]
        now = time.time()

        if current_class_id >= 6: return current_class_id, "SOLID"

        self.history[key].append(current_class_id)
        buffer = list(self.history[key])

        colored_frames = [c for c in buffer if c in [1, 2, 3, 4, 5]]
        if not colored_frames: return 0, "OFF"

        from collections import Counter
        counts = Counter(colored_frames)
        dominant_color, count = counts.most_common(1)[0]
        color_ratio = count / len(buffer)
        
        if mem['pattern'] == 'SOLID':
            new_pattern = 'FLASH' if color_ratio < 0.80 else 'SOLID'
        else:
            new_pattern = 'SOLID' if color_ratio > 0.90 else 'FLASH'

        if new_pattern != mem['pattern'] or dominant_color != mem['state']:
            if now - mem['last_switch_time'] < STATE_LOCK_DURATION:
                return mem['state'], mem['pattern']
            else:
                mem['state'] = dominant_color
                mem['pattern'] = new_pattern
                mem['last_switch_time'] = now
                return dominant_color, new_pattern
        else:
            return dominant_color, new_pattern

    def calculate_fused_distance(self, bbox_xyxy, class_id):
        """
        🔥 核心升级：同时返回 (融合值, 几何值, 宽度值)
        """
        x1, y1, x2, y2 = bbox_xyxy
        box_width = x2 - x1
        box_height = y2 - y1
        
        # 1. 基础检查
        if box_width <= 0 or box_height <= 0: 
            return 999.0, -1.0, -1.0

        # --- A. 宽度测距法 (通用) ---
        if class_id == 6: real_w = CLASS_REAL_WIDTHS["basketball"]
        elif class_id == 7: real_w = CLASS_REAL_WIDTHS["flag"]
        else: real_w = CLASS_REAL_WIDTHS["car"]
        
        dist_width = (real_w * self.fx) / box_width
        
        # 默认值
        dist_geo = -1.0
        fused_dist = dist_width

        # --- B. 融合逻辑 (仅针对小车 0-5) ---
        if 0 <= class_id <= 5:
            # 1. 几何测距 (假设倒立摄像头，取 y2 为塔顶)
            y_top_pixel = y2 
            v = y_top_pixel - self.cy
            
            if abs(v) < 1e-5: v = 1e-5
            
            # 计算俯仰角
            alpha = math.atan(v / self.fy)
            total_angle = alpha + 0.0 
            
            dH = OBJ_HEIGHT - CAM_HEIGHT
            
            if total_angle > 0.001:
                dist_geo = abs(dH / math.tan(total_angle))
            else:
                dist_geo = 99.9 

            # 2. 宽高比检查 (遮挡检测)
            current_ratio = box_width / box_height
            OCCLUSION_THRESHOLD = 0.9 

            if dist_geo < 15.0: 
                if current_ratio < OCCLUSION_THRESHOLD:
                    # ⚠️ 判定为遮挡 -> 全信几何法
                    fused_dist = dist_geo
                else:
                    # ✅ 判定为正常 -> 加权融合
                    fused_dist = 0.4 * dist_geo + 0.6 * dist_width
            else:
                fused_dist = dist_width
        
        # 非小车 (球、旗子) 直接返回宽度距离，dist_geo 保持 -1.0
        return fused_dist, dist_geo, dist_width

    def camera_worker(self, cam_idx):
        try:
            model = YOLO(self.model_path)
            self.queue.put({'type': 'log', 'message': f'✅ Cam {cam_idx}: Ready'})
        except Exception as e:
            self.queue.put({'type': 'log', 'message': f'❌ Cam {cam_idx}: {e}'})
            return

        cap = self.initialize_camera(cam_idx)
        if cap is None: return

        try:
            while True:
                ret, frame = cap.read()
                if not ret: 
                    time.sleep(0.1); continue

                results = model.track(frame, conf=0.45, iou=0.6, imgsz=(480, 640), persist=True, verbose=False)

                if results[0].boxes is not None:
                    for box in results[0].boxes:
                        raw_class_id = int(box.cls.item())
                        track_id = int(box.id.item()) if box.id is not None else -1
                        if track_id < 0: continue

                        xyxy = box.xyxy[0].cpu().numpy()
                        x_center = (xyxy[0] + xyxy[2]) / 2

                        stable_class, pattern = self.get_stable_state(cam_idx, track_id, raw_class_id)
                        
                        if stable_class == 0: continue

                        # 🔥 修复点：正确解包三个返回值
                        fused_dist, dist_geo, dist_width = self.calculate_fused_distance(xyxy, stable_class)
                        
                        azimuth = calculate_azimuth_planar(x_center, self.fx, self.cx)
                        bearing_body = wrap_deg_360(CAM_MOUNT_YAW_DEG.get(cam_idx, 0.0) + azimuth)

                        # 过滤太远的噪点 (使用 fused_dist 比较，而不是 Tuple)
                        if fused_dist < 8.0:
                            data = {
                                'type': 'detection',
                                'cam_idx': cam_idx,
                                'track_id': track_id,
                                'class_id': stable_class,
                                'pattern': pattern,
                                'distance': round(fused_dist, 2),  # 融合后的距离
                                'dist_geo': round(dist_geo, 2),    # 几何原始值
                                'dist_width': round(dist_width, 2),# 宽度原始值
                                'bearing_body': round(bearing_body, 2)
                            }
                            try: self.queue.put_nowait(data)
                            except queue.Full: pass
        except Exception as e:
            print(f"Cam {cam_idx} Error: {e}")
        finally:
            cap.release()

    def run(self):
        print('🚀 视觉系统运行中 (10Hz 限频输出)...')
        
        threads = []
        for idx in self.camera_indices:
            t = threading.Thread(target=self.camera_worker, args=(idx,), daemon=True)
            t.start()
            threads.append(t)

        try:
            while True:
                data = self.queue.get()
                
                if data['type'] == 'log':
                    print(data['message'])
                
                elif data['type'] == 'detection':
                    now = time.time()
                    if now - self.last_pub_time > PUBLISH_RATE_LIMIT:
                        
                        topic = "perception"
                        pub_data = {k:v for k,v in data.items() if k != 'type'}
                        self.socket.send_string(f"{topic} {json.dumps(pub_data)}")
                        self.last_pub_time = now
                        
                        cls_name = CLS_MAP.get(pub_data['class_id'], "UNK")
                        # 打印时显示更多调试信息
                        print(f"👁️ [{cls_name}-{pub_data['cam_idx']}] "
                              f"Fus:{pub_data['distance']} | "
                              f"G:{pub_data['dist_geo']} W:{pub_data['dist_width']}")

                self.queue.task_done()
        except KeyboardInterrupt:
            print("🛑 停止中...")

    def cleanup(self):
        self.socket.close()
        self.context.term()

if __name__ == "__main__":
    pub = VisionPublisher(MODEL_PATH, CAMERA_INDICES, ZMQ_PORT)
    try:
        pub.run()
    except KeyboardInterrupt:
        pass
    finally:
        pub.cleanup()