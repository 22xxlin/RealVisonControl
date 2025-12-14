#!/usr/bin/env python3
"""
文件: vision_pub.py (鲁棒增强版 V2.0)
功能: 视觉感知发布
升级点:
  1. 引入施密特触发器 (迟滞阈值) 防止 FLASH/SOLID 跳变
  2. 引入状态时间锁定 (State Locking) 防止高频切换
  3. ZMQ 输出限频 (10Hz)
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
PUBLISH_RATE_LIMIT = 0.1  # 限制 ZMQ 发送间隔至少 0.1s (10Hz)
STATE_LOCK_DURATION = 0.5 # 状态切换后锁定 0.5s，防止抖动

# 真实宽度 (单位: 米)
CLASS_REAL_WIDTHS = {
    "car": 0.31,
    "basketball": 0.23,
    "flag": 0.13
}

# Class ID 映射
CLS_MAP = {
    0: "OFF", 1: "BLUE", 2: "RED", 3: "GREEN", 
    4: "PURPLE", 5: "GRAY", 6: "BALL", 7: "FLAG"
}

# 相机朝向
CAM_MOUNT_YAW_DEG = {0: 180.0, 2: -90.0, 4: 0.0, 6: 90.0}

# ================= 辅助函数 =================
def create_camera_matrix(f_x=498, f_y=498, c_x=331.2797, c_y=156.1371):
    return np.array([[f_x, 0, c_x], [0, f_y, c_y], [0, 0, 1]])

def calculate_azimuth_planar(x_pixel, camera_params):
    fx = camera_params.get('fx', 498)
    cx = camera_params.get('cx', 331.2797)
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
        
        self.camera_matrix = create_camera_matrix()
        self.camera_matrix_inv = np.linalg.inv(self.camera_matrix)
        self.fx = 1.0 / self.camera_matrix_inv[0][0]
        self.cx = -self.camera_matrix_inv[0][2] * self.fx

        self.queue = queue.Queue(maxsize=100)
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.PUB)
        self.socket.bind(f"tcp://*:{self.zmq_port}")
        
        # 历史缓存: Key=(cam_idx, track_id)
        self.history = defaultdict(lambda: deque(maxlen=debounce_maxlen))
        
        # ⚠️ 新增：状态记忆与锁定
        # memory[key] = {'state': 'IDLE', 'pattern': 'OFF', 'last_switch_time': 0.0}
        self.state_memory = defaultdict(lambda: {'state': 0, 'pattern': 'OFF', 'last_switch_time': 0.0})
        
        # ⚠️ 新增：发布限频记录
        self.last_pub_time = 0.0

        print(f"✅ 视觉发布者启动 (鲁棒版) | 模型: {self.model_path}")

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
        核心逻辑升级：迟滞比较 (Hysteresis) + 状态锁定
        """
        key = (cam_idx, track_id)
        mem = self.state_memory[key]
        now = time.time()

        # 1. 特殊物体直接返回
        if current_class_id >= 6:
            return current_class_id, "SOLID"

        # 2. 存入历史 Buffer
        self.history[key].append(current_class_id)
        buffer = list(self.history[key])

        # 3. 统计颜色
        colored_frames = [c for c in buffer if c in [1, 2, 3, 4, 5]]
        if not colored_frames:
            return 0, "OFF"

        from collections import Counter
        counts = Counter(colored_frames)
        dominant_color, count = counts.most_common(1)[0]
        
        # 4. 计算占比
        total_len = len(buffer)
        color_ratio = count / total_len
        
        # 5. ⚠️ 迟滞逻辑 (Schmitt Trigger)
        # 上一次是 SOLID
        if mem['pattern'] == 'SOLID':
            # 只有比例掉到 0.80 以下，才降级为 FLASH
            if color_ratio < 0.80:
                new_pattern = 'FLASH'
            else:
                new_pattern = 'SOLID'
        # 上一次是 FLASH 或 OFF
        else:
            # 只有比例冲过 0.90，才升级为 SOLID
            if color_ratio > 0.90:
                new_pattern = 'SOLID'
            else:
                new_pattern = 'FLASH'

        # 6. ⚠️ 时间锁定 (防止癫痫式切换)
        # 如果新状态和旧状态不一样
        if new_pattern != mem['pattern'] or dominant_color != mem['state']:
            # 检查是否还在锁定时间内
            if now - mem['last_switch_time'] < STATE_LOCK_DURATION:
                # 还在冷却，保持旧状态
                return mem['state'], mem['pattern']
            else:
                # 冷却结束，允许切换，并更新时间戳
                mem['state'] = dominant_color
                mem['pattern'] = new_pattern
                mem['last_switch_time'] = now
                return dominant_color, new_pattern
        else:
            # 状态没变，直接更新时间戳(可选)或保持
            return dominant_color, new_pattern

    def calculate_distance(self, box_width, class_id):
        if class_id == 6: real_w = CLASS_REAL_WIDTHS["basketball"]
        elif class_id == 7: real_w = CLASS_REAL_WIDTHS["flag"]
        else: real_w = CLASS_REAL_WIDTHS["car"]
        if box_width <= 0: return 999.0
        return (real_w * self.fx) / box_width

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

                # 降低置信度，依赖后处理过滤
                results = model.track(frame, conf=0.45, iou=0.6, imgsz=(480, 640), persist=True, verbose=False)

                if results[0].boxes is not None:
                    for box in results[0].boxes:
                        raw_class_id = int(box.cls.item())
                        track_id = int(box.id.item()) if box.id is not None else -1
                        if track_id < 0: continue

                        x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                        x_center = (x1 + x2) / 2
                        box_width = x2 - x1

                        stable_class, pattern = self.get_stable_state(cam_idx, track_id, raw_class_id)
                        
                        # 过滤无效状态
                        if stable_class == 0: continue

                        distance = self.calculate_distance(box_width, stable_class)
                        azimuth = calculate_azimuth_planar(x_center, {'fx': self.fx, 'cx': self.cx})
                        bearing_body = wrap_deg_360(CAM_MOUNT_YAW_DEG.get(cam_idx, 0.0) + azimuth)

                        if distance < 6.0:
                            data = {
                                'type': 'detection',
                                'cam_idx': cam_idx,
                                'track_id': track_id,
                                'class_id': stable_class,
                                'pattern': pattern,
                                'distance': round(distance, 2),
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
                    # ⚠️ 7. 全局限频 (Throttle)
                    now = time.time()
                    if now - self.last_pub_time > PUBLISH_RATE_LIMIT:
                        
                        topic = "perception"
                        pub_data = {k:v for k,v in data.items() if k != 'type'}
                        self.socket.send_string(f"{topic} {json.dumps(pub_data)}")
                        self.last_pub_time = now # 更新发送时间
                        
                        # 调试打印
                        cls_name = CLS_MAP.get(pub_data['class_id'], "UNK")
                        print(f"👁️ [{cls_name}-{pub_data['cam_idx']}] {pub_data['pattern']:<5} | D={pub_data['distance']}m")

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