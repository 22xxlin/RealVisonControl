#!/usr/bin/env python3
"""
视觉节点 - 感知线程（Publisher）
持续运行 YOLO 检测，计算空间坐标，解码灯语，更新共享状态
"""

import os
import sys
import cv2
import numpy as np
import time
import math
import re
import threading
from collections import Counter, defaultdict
from ultralytics import YOLO


# 相机安装朝向（机体坐标系，+X为0°，逆时针为正）
CAM_MOUNT_YAW_DEG = {
    0: 180.0,  # cam0 -> -X（后方）
    2: -90.0,  # cam2 -> -Y（右侧）
    4: 0.0,    # cam4 -> +X（前方）
    6: 90.0,   # cam6 -> +Y（左侧）
}


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


def calculate_azimuth_planar(x_pixel, camera_params, debug=False):
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


def calculate_distance_planar(detected_width, real_width, fx, debug=False):
    """基于检测框宽度的距离计算"""
    if detected_width <= 0:
        return float('inf')
    
    distance = (real_width * fx) / detected_width
    return distance


class VisionNode:
    """
    视觉节点 - 只负责感知
    不执行任何机器人控制，只更新共享状态
    """
    
    def __init__(self, robot_state, model_path, camera_indices=[0, 2, 4, 6]):
        """
        初始化视觉节点
        
        Args:
            robot_state: RobotState 实例
            model_path: YOLO 模型路径
            camera_indices: 摄像头索引列表
        """
        self.robot_state = robot_state
        self.model_path = model_path
        self.camera_indices = camera_indices
        
        # 相机参数
        self.frame_width = 640
        self.frame_height = 480
        self.real_width = 0.31  # 目标真实宽度（米）
        self.camera_matrix = create_camera_matrix()
        self.camera_matrix_inv = np.linalg.inv(self.camera_matrix)
        
        # 灯语模式到动作的映射
        self.PATTERN_TO_COMMAND = {
            '220': 'FORWARD', '330': 'LEFT', '110': 'RIGHT', '550': 'REVERSE', '440': 'STOP',
            '2200': 'APPROACH', '1100': 'RETREAT', '4400': 'S_SHAPE', '5500': 'CIRCLE',
            '1111': 'FORWARD', '2222': 'LEFT', '3333': 'RIGHT', '4444': 'STOP', '5555': 'REVERSE',
        }
        
        # 动作描述
        self.ACTION_DESCRIPTIONS = {
            'FORWARD': '前进', 'LEFT': '左移', 'RIGHT': '右移', 'STOP': '停止',
            'REVERSE': '后退', 'APPROACH': '靠近', 'RETREAT': '远离', 
            'S_SHAPE': 'S形', 'CIRCLE': '圆形', 'IDLE': '待机'
        }
        
        # 线程控制
        self.stop_event = threading.Event()
        self.thread = None
        self.running = False
        
        # 临时检测缓存（用于灯语识别）
        self.detection_buffer = defaultdict(list)  # {cam_idx: []}
        self.buffer_lock = threading.Lock()

        print("✅ 视觉节点初始化完成")

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

                # 测试读取
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

    def camera_worker(self, cam_idx):
        """单个摄像头的工作线程"""
        print(f'🚀 启动视觉线程 - 摄像头 {cam_idx}')

        # 加载模型
        try:
            model = YOLO(self.model_path)
            print(f'✅ 摄像头 {cam_idx} 模型加载成功')
        except Exception as e:
            print(f'❌ 摄像头 {cam_idx} 模型加载失败: {e}')
            return

        # 初始化摄像头
        cap = self.initialize_camera(cam_idx)
        if cap is None:
            print(f'❌ 摄像头 {cam_idx} 初始化失败，线程退出')
            return

        frame_count = 0
        consec_fail = 0

        try:
            while not self.stop_event.is_set():
                ret, frame = cap.read()
                if not ret or frame is None:
                    consec_fail += 1
                    if consec_fail >= 30:
                        print(f'⚠️ 摄像头 {cam_idx} 连续读帧失败，退出')
                        break
                    time.sleep(0.05)
                    continue

                consec_fail = 0
                frame_count += 1

                # 执行检测
                try:
                    results = model.track(frame, conf=0.55, iou=0.6, imgsz=(480, 640), persist=True,verbose=False)

                    if results[0].boxes is not None:
                        for box in results[0].boxes:
                            class_id = int(box.cls.item())
                            confidence = float(box.conf.item())
                            track_id = int(box.id.item()) if box.id is not None else -1

                            if track_id < 0:
                                continue

                            x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                            x_center = (x1 + x2) / 2
                            box_width = x2 - x1

                            # 计算距离和方位角
                            fx = 1.0 / self.camera_matrix_inv[0][0]
                            cx = -self.camera_matrix_inv[0][2] * fx

                            distance = calculate_distance_planar(box_width, self.real_width, fx)
                            camera_params = {'fx': fx, 'cx': cx}
                            azimuth = calculate_azimuth_planar(x_center, camera_params)

                            # 转换到机体坐标系
                            cam_mount_yaw = CAM_MOUNT_YAW_DEG.get(cam_idx, 0.0)
                            bearing_body = wrap_deg_360(cam_mount_yaw + azimuth)

                            # 存储检测信息（用于灯语识别）
                            with self.buffer_lock:
                                self.detection_buffer[cam_idx].append({
                                    'track_id': track_id,
                                    'class_id': class_id,
                                    'distance': distance,
                                    'azimuth': azimuth,
                                    'bearing_body': bearing_body,
                                    'timestamp': time.time(),
                                    'frame_count': frame_count
                                })

                                # 限制缓存大小
                                if len(self.detection_buffer[cam_idx]) > 200:
                                    self.detection_buffer[cam_idx] = self.detection_buffer[cam_idx][-200:]

                            # 简单的灯语识别（基于类别ID变化）
                            # 这里简化处理，实际可以根据检测历史进行更复杂的分析
                            command = self.recognize_pattern(cam_idx, track_id)

                            # 更新共享状态
                            self.robot_state.update_perception(
                                distance=distance,
                                azimuth=azimuth,
                                bearing_body=bearing_body,
                                track_id=track_id,
                                cam_idx=cam_idx,
                                command=command,
                                command_params={
                                    'description': self.ACTION_DESCRIPTIONS.get(command, '未知')
                                },
                                class_id=class_id
                            )

                except Exception as e:
                    print(f'❌ 摄像头 {cam_idx} 推理错误: {e}')
                    time.sleep(0.1)

        finally:
            cap.release()
            print(f'🏁 视觉线程结束 - 摄像头 {cam_idx}')

    def recognize_pattern(self, cam_idx, track_id):
        """
        简化的灯语识别
        这里使用简单的启发式规则，实际应用中可以使用更复杂的算法
        """
        with self.buffer_lock:
            # 获取该 track_id 的最近检测
            recent_detections = [d for d in self.detection_buffer[cam_idx]
                               if d['track_id'] == track_id and time.time() - d['timestamp'] < 3.0]

            if len(recent_detections) < 10:
                return 'IDLE'

            # 统计类别ID
            class_ids = [d['class_id'] for d in recent_detections[-36:]]  # 最近36帧

            # 统计0和非0的比例
            zero_count = sum(1 for cid in class_ids if cid == 0)
            non_zero_count = len(class_ids) - zero_count

            if non_zero_count == 0:
                return 'IDLE'

            non_zero_class = max(set([cid for cid in class_ids if cid != 0]),
                               key=class_ids.count, default=0)

            if zero_count == 0:
                # 全是非零 -> xxxx 模式
                pattern = f'{non_zero_class}{non_zero_class}{non_zero_class}{non_zero_class}'
            elif non_zero_count / max(1, zero_count) > 1.8:
                # 非零多 -> xx0 模式
                pattern = f'{non_zero_class}{non_zero_class}0'
            else:
                # 比较均衡 -> xx00 模式
                pattern = f'{non_zero_class}{non_zero_class}00'

            return self.PATTERN_TO_COMMAND.get(pattern, 'IDLE')

    def start(self):
        """启动视觉节点"""
        if self.running:
            print("⚠️ 视觉节点已在运行")
            return

        self.stop_event.clear()
        self.running = True

        # 为每个摄像头启动线程
        self.threads = []
        for cam_idx in self.camera_indices:
            thread = threading.Thread(target=self.camera_worker, args=(cam_idx,), daemon=True)
            thread.start()
            self.threads.append(thread)
            time.sleep(0.3)  # 错开启动时间

        print("✅ 视觉节点已启动")

    def stop(self):
        """停止视觉节点"""
        if not self.running:
            return

        print("🛑 正在停止视觉节点...")
        self.stop_event.set()

        for thread in self.threads:
            thread.join(timeout=2.0)

        self.running = False
        print("✅ 视觉节点已停止")

