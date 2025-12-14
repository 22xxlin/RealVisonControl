#!/usr/bin/env python3
"""
视觉发布者 - 使用 ZeroMQ PUB 发布检测结果
独立进程，替代原本的 VisionNode 线程

【修改记录】
1. 距离计算适配不同物体尺寸：
   - Class 0-5 (车): 31cm
   - Class 6 (篮球): 23cm
   - Class 7 (Flag): 13cm
2. 模式识别逻辑分流：
   - 车 (0-5): 执行灯语识别 (如 '2200', '110')
   - 篮球/Flag: 直接输出类别名称，不做灯语组合
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
from collections import defaultdict
from ultralytics import YOLO


# 相机安装朝向（机体坐标系，+X为0°，逆时针为正）
CAM_MOUNT_YAW_DEG = {
    0: 180.0,  # cam0 -> -X（后方）
    2: -90.0,  # cam2 -> -Y（右侧）
    4: 0.0,    # cam4 -> +X（前方）
    6: 90.0,   # cam6 -> +Y（左侧）
}

# 【新增】不同类别的真实宽度 (单位: 米)
CLASS_REAL_WIDTHS = {
    "car": 0.31,       # Class 0-5
    "basketball": 0.23, # Class 6
    "flag": 0.13       # Class 7
}

def get_real_width(class_id):
    """根据类别ID获取真实宽度"""
    if 0 <= class_id <= 5:
        return CLASS_REAL_WIDTHS["car"]
    elif class_id == 6:
        return CLASS_REAL_WIDTHS["basketball"]
    elif class_id == 7:
        return CLASS_REAL_WIDTHS["flag"]
    else:
        return CLASS_REAL_WIDTHS["car"] # 默认按车处理


def wrap_deg_360(a):
    """将角度归一化到 [0, 360) 范围"""
    return (a + 360.0) % 360.0


def create_camera_matrix(f_x=498, f_y=498, c_x=331.2797, c_y=156.1371):
    """创建相机内参矩阵"""
    return np.array([
        [f_x, 0, c_x],
        [0, f_y, c_y],
        [0, 0, 1]
    ])


def calculate_azimuth_planar(x_pixel, camera_params):
    """基于平面几何的方位角计算"""
    fx = camera_params.get('fx', 498)
    cx = camera_params.get('cx', 331.2797)
    
    pixel_offset = x_pixel - cx
    angle_rad = math.atan(pixel_offset / fx)
    angle_deg = math.degrees(angle_rad)
    
    if angle_deg < 0:
        azimuth = 360 + angle_deg
    else:
        azimuth = angle_deg
    
    return azimuth


def calculate_distance_planar(detected_width, real_width, fx):
    """基于检测框宽度的距离计算"""
    if detected_width <= 0:
        return float('inf')
    
    distance = (real_width * fx) / detected_width
    return distance


class VisionPublisher:
    """视觉发布者 - 使用 ZeroMQ 发布检测结果"""
    
    def __init__(self, model_path, camera_indices=[0, 2, 4, 6], zmq_port=5555, debounce_threshold=5):
        self.model_path = model_path
        self.camera_indices = camera_indices
        self.zmq_port = zmq_port
        self.debounce_threshold = debounce_threshold

        # 相机参数
        self.frame_width = 640
        self.frame_height = 480
        # self.real_width 已被 get_real_width() 替代
        self.camera_matrix = create_camera_matrix()
        self.camera_matrix_inv = np.linalg.inv(self.camera_matrix)

        # 临时检测缓存（用于识别逻辑）
        self.detection_buffer = defaultdict(list)  # {cam_idx: []}

        # 【去抖动机制】模式稳定性跟踪
        self.pattern_stability = defaultdict(lambda: {
            'candidate': 'IDLE',
            'count': 0,
            'confirmed': 'IDLE'
        })

        # 线程安全队列
        self.queue = queue.Queue(maxsize=100)

        # 初始化 ZeroMQ
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.PUB)
        self.socket.bind(f"tcp://*:{self.zmq_port}")

        print(f"✅ 视觉发布者初始化完成 - ZMQ 绑定到 tcp://*:{self.zmq_port}")
        print(f"🔧 去抖动阈值: {self.debounce_threshold} 帧")

    def initialize_camera(self, cam_idx):
        """初始化单个摄像头"""
        try:
            cap = cv2.VideoCapture(cam_idx, cv2.CAP_V4L2)
            if not cap.isOpened():
                cap = cv2.VideoCapture(cam_idx)

            if cap.isOpened():
                cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.frame_width)
                cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.frame_height)
                cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
                cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
                
                time.sleep(0.3)
                ret, frame = cap.read()
                if ret and frame is not None:
                    print(f'✅ 摄像头 {cam_idx} 初始化成功')
                    return cap
                else:
                    cap.release()
                    return None
            return None
        except Exception as e:
            print(f'❌ 摄像头 {cam_idx} 初始化错误: {e}')
            return None

    def recognize_pattern(self, cam_idx, track_id):
        """
        【修改版】模式识别 + 去抖动
        1. 篮球 (Class 6) -> 输出 'BASKETBALL'
        2. Flag (Class 7) -> 输出 'FLAG'
        3. 车 (Class 0-5) -> 执行原来的灯语逻辑 (如 '2200')
        """
        # 获取该 track_id 的最近检测
        recent_detections = [d for d in self.detection_buffer[cam_idx]
                           if d['track_id'] == track_id and time.time() - d['timestamp'] < 3.0]

        if len(recent_detections) < 5: # 稍微降低一点起步门槛
            raw_pattern = 'IDLE'
        else:
            # 统计最近帧的主要类别
            class_ids = [d['class_id'] for d in recent_detections[-20:]]
            if not class_ids:
                raw_pattern = 'IDLE'
            else:
                # 找出出现次数最多的类别
                main_class = max(set(class_ids), key=class_ids.count)

                # === 分支 1: 特殊物体 (篮球/Flag) ===
                if main_class == 6:
                    raw_pattern = 'BASKETBALL'
                elif main_class == 7:
                    raw_pattern = 'FLAG'
                
                # === 分支 2: 车辆 (Class 0-5) -> 进灯语识别 ===
                elif 0 <= main_class <= 5:
                    # 统计0和非0的比例 (仅在0-5范围内统计)
                    # 过滤掉偶尔跳变的 6/7 干扰
                    car_ids = [cid for cid in class_ids if 0 <= cid <= 5]
                    
                    if not car_ids:
                        raw_pattern = 'IDLE'
                    else:
                        zero_count = sum(1 for cid in car_ids if cid == 0)
                        non_zero_count = len(car_ids) - zero_count
                        
                        if non_zero_count == 0:
                            # 只有0 -> IDLE? 或者特定的0模式? 
                            # 假设 Class 0 是无灯状态，或者是某种特定颜色
                            # 原逻辑: 如果 non_zero_count == 0 -> raw_pattern = 'IDLE'
                            # 但如果你的 Class 0 是红色车，可能需要输出 '0000'
                            # 这里保持原逻辑：
                            raw_pattern = 'IDLE' 
                            # 如果你需要 Class 0 也输出模式，请改为: raw_pattern = '0000'
                        else:
                            # 找出主要的非0 ID
                            non_zero_class = max(set([cid for cid in car_ids if cid != 0]),
                                               key=car_ids.count, default=0)

                            if zero_count == 0:
                                raw_pattern = f'{non_zero_class}{non_zero_class}{non_zero_class}{non_zero_class}'
                            elif non_zero_count / max(1, zero_count) > 1.8:
                                raw_pattern = f'{non_zero_class}{non_zero_class}0'
                            else:
                                raw_pattern = f'{non_zero_class}{non_zero_class}00'
                else:
                    raw_pattern = 'IDLE'

        # ========== 【通用去抖动逻辑】 ==========
        # 无论是什么模式（BASKETBALL, FLAG, 1100, IDLE），都经过这一层过滤
        # 确保只有稳定的检测才会被输出
        key = (cam_idx, track_id)
        stability = self.pattern_stability[key]

        if raw_pattern == stability['candidate']:
            stability['count'] += 1
        else:
            stability['candidate'] = raw_pattern
            stability['count'] = 1

        if stability['count'] >= self.debounce_threshold:
            stability['confirmed'] = raw_pattern
            return raw_pattern
        else:
            return stability['confirmed']

    def camera_worker(self, cam_idx):
        """生产者线程：读取图像 -> YOLO检测 -> 存入队列"""
        try:
            model = YOLO(self.model_path)
            self.queue.put({'type': 'log', 'message': f'✅ 摄像头 {cam_idx} 模型加载成功'})
        except Exception as e:
            self.queue.put({'type': 'log', 'message': f'❌ 摄像头 {cam_idx} 模型加载失败: {e}'})
            return

        cap = self.initialize_camera(cam_idx)
        if cap is None:
            self.queue.put({'type': 'log', 'message': f'❌ 摄像头 {cam_idx} 初始化失败，退出'})
            return

        frame_count = 0
        consec_fail = 0

        try:
            while True:
                ret, frame = cap.read()
                if not ret or frame is None:
                    consec_fail += 1
                    if consec_fail >= 30:
                        self.queue.put({'type': 'log', 'message': f'⚠️ 摄像头 {cam_idx} 连续读帧失败'})
                        break
                    time.sleep(0.05)
                    continue

                consec_fail = 0
                frame_count += 1

                try:
                    # 降低 conf 阈值以提高召回率，依靠后续逻辑过滤
                    results = model.track(frame, conf=0.50, iou=0.6, imgsz=(480, 640), persist=True, verbose=False)

                    if results[0].boxes is not None:
                        for box in results[0].boxes:
                            class_id = int(box.cls.item())
                            track_id = int(box.id.item()) if box.id is not None else -1

                            if track_id < 0:
                                continue

                            x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                            x_center = (x1 + x2) / 2
                            box_width = x2 - x1

                            # 【修改点】动态获取真实宽度
                            real_width_val = get_real_width(class_id)

                            # 计算距离
                            fx = 1.0 / self.camera_matrix_inv[0][0]
                            cx = -self.camera_matrix_inv[0][2] * fx
                            
                            # 传入对应的 real_width
                            distance = calculate_distance_planar(box_width, real_width_val, fx)
                            
                            camera_params = {'fx': fx, 'cx': cx}
                            azimuth = calculate_azimuth_planar(x_center, camera_params)
                            
                            cam_mount_yaw = CAM_MOUNT_YAW_DEG.get(cam_idx, 0.0)
                            bearing_body = wrap_deg_360(cam_mount_yaw + azimuth)

                            # 存储检测历史
                            self.detection_buffer[cam_idx].append({
                                'track_id': track_id,
                                'class_id': class_id,
                                'distance': distance,
                                'azimuth': azimuth,
                                'bearing_body': bearing_body,
                                'timestamp': time.time(),
                                'frame_count': frame_count
                            })
                            
                            # 简单的 buffer 清理
                            if len(self.detection_buffer[cam_idx]) > 200:
                                self.detection_buffer[cam_idx] = self.detection_buffer[cam_idx][-200:]

                            # 识别模式 (含特殊物体处理)
                            pattern = self.recognize_pattern(cam_idx, track_id)

                            detection_data = {
                                'type': 'detection',
                                'distance': float(distance),
                                'azimuth': float(azimuth),
                                'bearing_body': float(bearing_body),
                                'track_id': int(track_id),
                                'cam_idx': int(cam_idx),
                                'pattern': pattern,
                                'class_id': int(class_id),
                                'timestamp': time.time()
                            }

                            try:
                                self.queue.put_nowait(detection_data)
                            except queue.Full:
                                pass

                except Exception as e:
                    # 避免打印过多错误刷屏
                    pass
                    
        except KeyboardInterrupt:
            self.queue.put({'type': 'log', 'message': f'⚠️ 摄像头 {cam_idx} 中断'})
        finally:
            cap.release()
            self.queue.put({'type': 'log', 'message': f'🏁 摄像头 {cam_idx} 结束'})

    def run(self):
        """主线程消费者"""
        print(f'🚀 启动视觉发布者')
        print(f'📏 距离参数: {CLASS_REAL_WIDTHS}')
        
        threads = []
        for cam_idx in self.camera_indices:
            t = threading.Thread(target=self.camera_worker, args=(cam_idx,), daemon=True)
            t.start()
            threads.append(t)

        try:
            while True:
                data = self.queue.get()
                
                if data.get('type') == 'log':
                    print(data['message'])
                
                elif data.get('type') == 'detection':
                    # 发送 ZMQ
                    topic = "perception"
                    msg_dict = {k: v for k, v in data.items() if k != 'type'}
                    self.socket.send_string(f"{topic} {json.dumps(msg_dict)}")

                    # 打印关键信息 (仅当检测稳定时)
                    pat = msg_dict.get('pattern', 'IDLE')
                    if pat != 'IDLE':
                        cls_name = "Car"
                        cid = msg_dict['class_id']
                        if cid == 6: cls_name = "Ball"
                        elif cid == 7: cls_name = "Flag"
                        
                        print(f"👁️ [{cls_name}-{msg_dict['cam_idx']}] {pat} | D={msg_dict['distance']:.2f}m")

                self.queue.task_done()

        except KeyboardInterrupt:
            print('\n🛑 停止中...')

    def cleanup(self):
        self.socket.close()
        self.context.term()

if __name__ == "__main__":
    # 配置
    MODEL_PATH = "/home/nvidia/Downloads/Ros/ballCar2/weights/weights/best.engine"
    CAMERA_INDICES = [0, 2, 4, 6] 
    ZMQ_PORT = 5555

    pub = VisionPublisher(MODEL_PATH, CAMERA_INDICES, ZMQ_PORT)
    try:
        pub.run()
    except KeyboardInterrupt:
        pass
    finally:
        pub.cleanup()