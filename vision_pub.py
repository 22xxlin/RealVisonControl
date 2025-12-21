#!/usr/bin/env python3
"""
文件: vision_pub.py (V3.2 内部融合版)
功能: 多摄视觉感知、内部融合、择优发送
更新日志:
  1. 架构升级: 采用 "收集 -> 排序 -> 去重 -> 发送" 的 20Hz 周期性逻辑。
  2. 融合策略: 相同角度的物体，保留检测框面积(Area)最大的那个。
  3. 测距算法: 纯宽度法 (Width-Based)，移除了几何法。
  4. 边缘检测: 识别并标记被截断的物体。
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
FUSION_RATE_HZ = 20        # 融合与发送频率 (20Hz = 50ms)
STATE_LOCK_DURATION = 0.5  # 状态切换锁定时间

# === 物理参数 ===
# 真实宽度 (单位: 米)
CLASS_REAL_WIDTHS = {
    "car": 0.31,
    "basketball": 0.23,
    "flag": 0.13
}

# 相机内参 (标定分辨率: 640x480)
CAM_INTRINSICS = {
    'fx': 498.0,
    'fy': 498.0,
    'cx': 331.2797,
    'cy': 156.1371,
    'calib_width': 640,
    'calib_height': 480
}

# Class ID 映射
CLS_MAP = {
    0: "OFF", 1: "BLUE", 2: "RED", 3: "GREEN", 
    4: "PURPLE", 5: "GRAY", 6: "BALL", 7: "FLAG"
}

# 相机朝向 (机身坐标系)
CAM_MOUNT_YAW_DEG = {0: 180.0, 2: -90.0, 4: 0.0, 6: 90.0}

# ================= 辅助函数 =================
def calculate_azimuth_planar(x_pixel, fx, cx):
    """计算像平面内的偏航角"""
    pixel_offset = x_pixel - cx
    angle_rad = math.atan(pixel_offset / fx)
    return math.degrees(angle_rad)

def wrap_deg_360(a):
    """归一化到 0-360 度"""
    return (a + 360.0) % 360.0

def get_angle_diff(a1, a2):
    """计算两个角度的最小差值 (考虑0/360循环)"""
    diff = abs(a1 - a2)
    return min(diff, 360.0 - diff)

# ================= 主类定义 =================
class VisionPublisher:
    def __init__(self, model_path, camera_indices, zmq_port=5555):
        self.model_path = model_path
        self.camera_indices = camera_indices

        # 核心参数
        self.fx = CAM_INTRINSICS['fx']
        self.cx = CAM_INTRINSICS['cx']
        self.calib_width = CAM_INTRINSICS['calib_width']
        self.calib_height = CAM_INTRINSICS['calib_height']

        # 实际运行分辨率
        self.actual_width = 640
        self.actual_height = 480

        # 通信与队列
        self.queue = queue.Queue(maxsize=200) # 稍微加大队列，防止多摄数据瞬间堆积
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.PUB)
        self.socket.bind(f"tcp://*:{zmq_port}")
        
        # 状态滤波缓存
        self.history = defaultdict(lambda: deque(maxlen=30))
        self.state_memory = defaultdict(lambda: {'state': 0, 'pattern': 'OFF', 'last_switch_time': 0.0})

        print(f"✅ 视觉融合系统 V3.2 启动 | 频率: {FUSION_RATE_HZ}Hz")

    def initialize_camera(self, cam_idx):
        try:
            cap = cv2.VideoCapture(cam_idx, cv2.CAP_V4L2)
            if not cap.isOpened(): cap = cv2.VideoCapture(cam_idx)
            if cap.isOpened():
                cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
                cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
                cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
                cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))

                # 验证实际分辨率并调整 fx
                actual_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
                actual_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
                if actual_w != self.calib_width:
                    scale = actual_w / self.calib_width
                    self.fx = CAM_INTRINSICS['fx'] * scale
                    self.cx = CAM_INTRINSICS['cx'] * scale
                    self.actual_width = actual_w
                    self.actual_height = actual_h
                    print(f"⚠️ Cam {cam_idx}: 分辨率 {actual_w}x{actual_h}, fx 已缩放至 {self.fx:.2f}")

                return cap
        except Exception: pass
        return None

    def get_stable_state(self, cam_idx, track_id, current_class_id):
        """颜色状态防抖动逻辑"""
        if current_class_id >= 6: return current_class_id, "SOLID" # 球和旗子不滤波

        key = (cam_idx, track_id)
        mem = self.state_memory[key]
        now = time.time()

        self.history[key].append(current_class_id)
        buffer = list(self.history[key])

        # 简单的投票机制
        from collections import Counter
        colored_frames = [c for c in buffer if c in [1, 2, 3, 4, 5]]
        if not colored_frames: return 0, "OFF"
        
        dominant_color, count = Counter(colored_frames).most_common(1)[0]
        color_ratio = count / len(buffer)
        
        # 模式判定 (闪烁 vs 常亮)
        target_pattern = 'SOLID' if color_ratio > 0.90 else 'FLASH'
        if mem['pattern'] == 'SOLID' and color_ratio < 0.80: target_pattern = 'FLASH'

        # 状态锁定 (防止颜色并在跳变)
        if (target_pattern != mem['pattern'] or dominant_color != mem['state']):
            if now - mem['last_switch_time'] > STATE_LOCK_DURATION:
                mem['state'] = dominant_color
                mem['pattern'] = target_pattern
                mem['last_switch_time'] = now
        
        return mem['state'], mem['pattern']

    def camera_worker(self, cam_idx):
        """
        相机工作线程：只负责检测和计算基础数据，不负责融合。
        """
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
                        raw_cls = int(box.cls.item())
                        tid = int(box.id.item()) if box.id is not None else -1
                        if tid < 0: continue

                        # 1. 获取检测框数据
                        xyxy = box.xyxy[0].cpu().numpy()
                        x1, y1, x2, y2 = xyxy
                        box_w = x2 - x1
                        box_h = y2 - y1
                        x_center = (x1 + x2) / 2
                        
                        # 计算面积
                        area = box_w * box_h

                        # 2. 截断检测 (Truncated)
                        # 如果框的边缘贴近图像边界 (假设 640x480)，说明物体可能不完整
                        is_truncated = (x1 < 2 or y1 < 2 or x2 > 638 or y2 > 478)

                        # 3. 状态滤波
                        stable_cls, pattern = self.get_stable_state(cam_idx, tid, raw_cls)
                        if stable_cls == 0: continue

                        # 4. 纯宽度测距
                        if stable_cls == 6: real_w = CLASS_REAL_WIDTHS["basketball"]
                        elif stable_cls == 7: real_w = CLASS_REAL_WIDTHS["flag"]
                        else: real_w = CLASS_REAL_WIDTHS["car"]

                        # 确保 box_w 有效
                        if box_w <= 0:
                            continue

                        dist = (real_w * self.fx) / box_w

                        # 5. 角度解算
                        azimuth = calculate_azimuth_planar(x_center, self.fx, self.cx)
                        bearing_body = wrap_deg_360(CAM_MOUNT_YAW_DEG.get(cam_idx, 0.0) + azimuth)

                        # 6. 打包入队 (包含 area 和 truncated 供主线程融合使用)
                        if dist < 8.0:
                            data = {
                                'type': 'detection',
                                'cam_idx': cam_idx,
                                'class_id': stable_cls,
                                'pattern': pattern,
                                'distance': round(dist, 2),
                                'bearing_body': round(bearing_body, 2),
                                'area': int(area),          # 融合权重核心
                                'truncated': is_truncated   # 降权标志
                            }
                            # 非阻塞入队，满了就扔掉旧的
                            try: self.queue.put_nowait(data)
                            except queue.Full: pass
                            
        except Exception as e:
            print(f"Cam {cam_idx} Error: {e}")
        finally:
            cap.release()

    def run(self):
        """
        主循环：周期性融合 (Batch Processing)
        """
        print(f'🚀 视觉融合循环启动... (周期: {1.0/FUSION_RATE_HZ*1000:.1f}ms)')
        
        # 启动线程
        for idx in self.camera_indices:
            threading.Thread(target=self.camera_worker, args=(idx,), daemon=True).start()

        fusion_interval = 1.0 / FUSION_RATE_HZ
        
        try:
            while True:
                cycle_start = time.time()
                
                # --- 1. 收集阶段 (Drain Queue) ---
                # 把当前瞬间队列里所有相机的数据全取出来
                batch_detections = []
                while True:
                    try:
                        item = self.queue.get_nowait()
                        if item['type'] == 'log':
                            print(item['message'])
                        elif item['type'] == 'detection':
                            batch_detections.append(item)
                        self.queue.task_done()
                    except queue.Empty:
                        break # 队列空了，收集完毕
                
                # --- 2. 融合阶段 (Fusion) ---
                final_objects = []

                if batch_detections:
                    # 调试：显示收集到的原始数据
                    if len(batch_detections) > 1:
                        raw_info = [f"Cam{d['cam_idx']}:{CLS_MAP.get(d['class_id'],'?')}@{d['bearing_body']:.0f}°"
                                   for d in batch_detections]
                        print(f"  🔍 收集到 {len(batch_detections)} 个检测: {raw_info}")

                    # A. 排序：面积大的排前面 (相信看得最清楚的)
                    #    被截断的物体降权处理
                    def get_effective_area(obj):
                        area = obj['area']
                        if obj['truncated']:
                            area *= 0.5  # 截断物体权重减半
                        return area

                    batch_detections.sort(key=get_effective_area, reverse=True)

                    # B. 去重 (Suppression) - 仅基于角度
                    for new_obj in batch_detections:
                        is_duplicate = False
                        duplicate_with = None
                        angle_diff = 0.0

                        for existing in final_objects:
                            # 判断是否为同一物体：
                            # 1. 类别必须相同
                            if new_obj['class_id'] != existing['class_id']:
                                continue

                            # 2. 角度接近 (相差 < 100 度) - 只看角度
                            angle_diff = get_angle_diff(new_obj['bearing_body'], existing['bearing_body'])

                            if angle_diff < 100:
                                is_duplicate = True
                                duplicate_with = existing
                                break

                        if is_duplicate:
                            # 调试：显示去重信息
                            print(f"    ❌ 去重: Cam{new_obj['cam_idx']} 的 {CLS_MAP.get(new_obj['class_id'],'?')} "
                                  f"与 Cam{duplicate_with['cam_idx']} 重复 (角度差={angle_diff:.1f}°)")
                        else:
                            final_objects.append(new_obj)

                # --- 3. 发送阶段 (Publish) ---
                # [Refactor] Changed to batch sending
                # 构造包含所有物体的大字典 (Payload)
                topic = "perception"
                timestamp = time.time()

                # 构建物体列表，只包含下游需要的字段
                objects_list = []
                for obj in final_objects:
                    obj_data = {
                        'cam_idx': obj['cam_idx'],
                        'class_id': obj['class_id'],
                        'pattern': obj['pattern'],
                        'distance': obj['distance'],
                        'bearing_body': obj['bearing_body']
                    }
                    objects_list.append(obj_data)

                # 构造完整的 Payload
                payload = {
                    'timestamp': timestamp,
                    'count': len(final_objects),
                    'objects': objects_list
                }

                # [Refactor] 只发送一次，而不是遍历发送
                self.socket.send_string(f"{topic} {json.dumps(payload)}")

                # 打印发送的物体信息
                if final_objects:
                    print(f"📦 [Batch Send] 时间戳={timestamp:.3f} | 物体数={len(final_objects)}")
                    for obj in final_objects:
                        cls_name = CLS_MAP.get(obj['class_id'], "UNK")
                        print(f"  👁️ [{cls_name}-Cam{obj['cam_idx']}] {obj['pattern']:<5} | D={obj['distance']:.2f}m | Ang={obj['bearing_body']:.1f}°")

                # --- 4. 控频休眠 ---
                elapsed = time.time() - cycle_start
                sleep_time = fusion_interval - elapsed
                if sleep_time > 0:
                    time.sleep(sleep_time)

        except KeyboardInterrupt:
            print("🛑 停止中...")
        finally:
            self.socket.close()
            self.context.term()

if __name__ == "__main__":
    pub = VisionPublisher(MODEL_PATH, CAMERA_INDICES, ZMQ_PORT)
    pub.run()