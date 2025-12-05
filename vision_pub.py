#!/usr/bin/env python3
"""
视觉发布者 - 使用 ZeroMQ PUB 发布检测结果
独立进程，替代原本的 VisionNode 线程

架构：生产者-消费者模型（Producer-Consumer）
- 生产者（4个子线程）：每个摄像头独立运行 camera_worker，读图、YOLO检测、灯语识别
  将检测数据通过线程安全队列 (queue.Queue) 发送给消费者
- 消费者（主线程）：从队列中取数据，统一负责 ZMQ 发送和日志打印
  确保 ZMQ Socket 的线程安全（Socket 不是线程安全的，只能在主线程操作）
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
    
    def __init__(self, model_path, camera_indices=[0, 2, 4, 6], zmq_port=5555):
        """
        初始化视觉发布者
        
        Args:
            model_path: YOLO 模型路径
            camera_indices: 摄像头索引列表
            zmq_port: ZeroMQ 发布端口
        """
        self.model_path = model_path
        self.camera_indices = camera_indices
        self.zmq_port = zmq_port
        
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
        
        # 临时检测缓存（用于灯语识别）
        self.detection_buffer = defaultdict(list)  # {cam_idx: []}

        # 线程安全队列（生产者-消费者模型）
        self.queue = queue.Queue(maxsize=100)

        # 初始化 ZeroMQ
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.PUB)
        self.socket.bind(f"tcp://*:{self.zmq_port}")

        print(f"✅ 视觉发布者初始化完成 - ZMQ 绑定到 tcp://*:{self.zmq_port}")

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

    def recognize_pattern(self, cam_idx, track_id):
        """
        简化的灯语识别
        基于类别ID变化识别灯语模式
        """
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



    def camera_worker(self, cam_idx):
        """
        单个摄像头的工作循环（生产者线程）
        不直接操作 socket 和 print，而是将数据放入队列
        """
        # 加载模型
        try:
            model = YOLO(self.model_path)
            # 通过队列发送初始化成功消息
            self.queue.put({
                'type': 'log',
                'message': f'✅ 摄像头 {cam_idx} 模型加载成功'
            })
        except Exception as e:
            self.queue.put({
                'type': 'log',
                'message': f'❌ 摄像头 {cam_idx} 模型加载失败: {e}'
            })
            return

        # 初始化摄像头
        cap = self.initialize_camera(cam_idx)
        if cap is None:
            self.queue.put({
                'type': 'log',
                'message': f'❌ 摄像头 {cam_idx} 初始化失败，退出'
            })
            return

        frame_count = 0
        consec_fail = 0

        try:
            while True:
                ret, frame = cap.read()
                if not ret or frame is None:
                    consec_fail += 1
                    if consec_fail >= 30:
                        self.queue.put({
                            'type': 'log',
                            'message': f'⚠️ 摄像头 {cam_idx} 连续读帧失败，退出'
                        })
                        break
                    time.sleep(0.05)
                    continue

                consec_fail = 0
                frame_count += 1

                # 执行检测（静音模式）
                try:
                    results = model.track(frame, conf=0.55, iou=0.6, imgsz=(480, 640), persist=True)

                    if results[0].boxes is not None:
                        for box in results[0].boxes:
                            class_id = int(box.cls.item())
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

                            # 灯语识别
                            command = self.recognize_pattern(cam_idx, track_id)

                            # 打包数据并放入队列（不直接发送）
                            detection_data = {
                                'type': 'detection',
                                'distance': float(distance),
                                'azimuth': float(azimuth),
                                'bearing_body': float(bearing_body),
                                'track_id': int(track_id),
                                'cam_idx': int(cam_idx),
                                'command': command,
                                'description': self.ACTION_DESCRIPTIONS.get(command, '未知'),
                                'class_id': int(class_id),
                                'timestamp': time.time()
                            }

                            # 使用 put_nowait，如果队列满了就丢弃
                            try:
                                self.queue.put_nowait(detection_data)
                            except queue.Full:
                                # 队列满了，丢弃该帧数据
                                pass

                except Exception as e:
                    self.queue.put({
                        'type': 'log',
                        'message': f'❌ 摄像头 {cam_idx} 推理错误: {e}'
                    })
                    time.sleep(0.1)

        except KeyboardInterrupt:
            self.queue.put({
                'type': 'log',
                'message': f'\n⚠️ 摄像头 {cam_idx} 收到中断信号'
            })
        finally:
            cap.release()
            self.queue.put({
                'type': 'log',
                'message': f'🏁 视觉发布者结束 - 摄像头 {cam_idx}'
            })

    def run(self):
        """
        运行视觉发布者（多摄像头并行模式）
        主线程作为消费者，负责从队列取数据并发送 ZMQ 消息
        """
        print(f'🚀 启动视觉发布者 - 多摄像头并行模式')
        print(f'📹 摄像头列表: {self.camera_indices}')

        # 启动所有摄像头的生产者线程
        threads = []
        for cam_idx in self.camera_indices:
            thread = threading.Thread(
                target=self.camera_worker,
                args=(cam_idx,),
                daemon=True,
                name=f"Camera-{cam_idx}"
            )
            thread.start()
            threads.append(thread)
            print(f'✅ 摄像头 {cam_idx} 线程已启动')

        print(f'\n📡 主线程开始消费队列数据...\n')

        # 主线程作为消费者，不断从队列取数据
        try:
            while True:
                # 阻塞等待队列数据
                data = self.queue.get()

                # 处理不同类型的消息
                if data.get('type') == 'log':
                    # 日志消息，直接打印
                    print(data['message'])

                elif data.get('type') == 'detection':
                    # 检测数据，通过 ZMQ 发送
                    topic = "perception"

                    # 移除 type 字段，只发送检测数据
                    detection_data = {k: v for k, v in data.items() if k != 'type'}
                    message = json.dumps(detection_data)
                    self.socket.send_string(f"{topic} {message}")

                    # 简洁的终端日志
                    cmd = detection_data.get('command', 'IDLE')
                    dist = detection_data.get('distance', 0)
                    bearing = detection_data.get('bearing_body', 0)
                    track_id = detection_data.get('track_id', -1)
                    cam_idx = detection_data.get('cam_idx', -1)

                    if cmd != 'IDLE':
                        print(f"🎥 [Cam{cam_idx}] Sent: {cmd} | Dist={dist:.2f}m | Bearing={bearing:.1f}° | TrackID={track_id}")

                # 标记任务完成
                self.queue.task_done()

        except KeyboardInterrupt:
            print('\n\n⚠️ 主线程收到中断信号，等待子线程结束...')
            # 等待所有线程结束（最多等待5秒）
            for thread in threads:
                thread.join(timeout=5.0)
            print('✅ 所有摄像头线程已结束')

    def cleanup(self):
        """清理资源"""
        self.socket.close()
        self.context.term()
        print("✅ ZeroMQ 资源已清理")


if __name__ == "__main__":
    # 配置参数
    MODEL_PATH = "/home/nvidia/Downloads/Ros/0821Car3/weights/best.engine"  # 修改为你的模型路径
    CAMERA_INDICES = [0, 2, 4, 6]  # 4个摄像头并行工作：后、右、前、左
    ZMQ_PORT = 5555

    print("=" * 60)
    print("🎥 视觉发布者 (ZeroMQ PUB) - 多摄像头并行模式")
    print("=" * 60)
    print(f"📡 发布地址: tcp://*:{ZMQ_PORT}")
    print(f"📹 摄像头: {CAMERA_INDICES}")
    print(f"   - Cam0: 后方 (180°)")
    print(f"   - Cam2: 右侧 (-90°)")
    print(f"   - Cam4: 前方 (0°)")
    print(f"   - Cam6: 左侧 (90°)")
    print(f"🤖 模型路径: {MODEL_PATH}")
    print(f"🔄 架构: 生产者-消费者模型 (4个生产者线程 + 1个消费者主线程)")
    print("=" * 60)
    print("按 Ctrl+C 停止\n")

    # 创建并运行发布者
    publisher = VisionPublisher(
        model_path=MODEL_PATH,
        camera_indices=CAMERA_INDICES,
        zmq_port=ZMQ_PORT
    )

    try:
        publisher.run()
    except KeyboardInterrupt:
        print("\n\n🛑 收到停止信号")
    finally:
        publisher.cleanup()
        print("👋 视觉发布者已退出")

