#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
data_logger.py
功能：同时监听 Vicon (真值) 和 ZMQ (视觉)，记录并对比误差。
输出：屏幕打印 + CSV 文件保存
"""

import rospy
import zmq
import json
import time
import math
import csv
import datetime
from geometry_msgs.msg import TransformStamped

# ================= 配置 =================
# Vicon 话题
ROBOT_VICON_TOPIC  = "vicon/VSWARM15/VSWARM15"
TARGET_VICON_TOPIC = "vicon/VSWARM45/VSWARM45" # 你的球

# ZMQ 配置 (对应 vision_pub.py)
ZMQ_PORT = 5555

# ================= 工具 =================
def quat_to_yaw_deg(x, y, z, w):
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return (math.degrees(yaw) + 360.0) % 360.0

def normalize_angle(angle):
    return (angle + 180.0) % 360.0 - 180.0

class CalibrationLogger:
    def __init__(self):
        # 1. 初始化 ROS (Vicon)
        rospy.init_node('calibration_logger', anonymous=True)
        self.poses = {'robot': None, 'target': None}
        
        rospy.Subscriber(ROBOT_VICON_TOPIC, TransformStamped, self._cb, 'robot')
        rospy.Subscriber(TARGET_VICON_TOPIC, TransformStamped, self._cb, 'target')
        
        # 2. 初始化 ZMQ (Vision)
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.SUB)
        self.socket.connect(f"tcp://localhost:{ZMQ_PORT}")
        self.socket.setsockopt_string(zmq.SUBSCRIBE, "perception")
        
        # 3. CSV 文件
        ts = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        self.filename = f"log_vision_vs_vicon_{ts}.csv"
        self.csv_file = open(self.filename, 'w', newline='')
        self.writer = csv.writer(self.csv_file)
        # 写入表头
        self.writer.writerow([
            "Time", 
            "Vicon_Dist", "Vision_Dist", "Dist_Err",
            "Vicon_Angle", "Vision_Angle", "Angle_Err",
            "Vision_Class", "Cam_ID"
        ])
        
        print(f"📝 日志记录中: {self.filename}")
        print(f"📡 等待 Vicon ({ROBOT_VICON_TOPIC}) 和 Vision 数据...")

    def _cb(self, msg, key):
        x = msg.transform.translation.x
        y = msg.transform.translation.y
        rot = msg.transform.rotation
        yaw = quat_to_yaw_deg(rot.x, rot.y, rot.z, rot.w)
        self.poses[key] = {'x': x, 'y': y, 'yaw': yaw, 'ts': time.time()}

    def get_vicon_truth(self):
        p_r = self.poses['robot']
        p_t = self.poses['target']
        now = time.time()
        
        if not p_r or not p_t: return None, None
        if (now - p_r['ts'] > 0.5) or (now - p_t['ts'] > 0.5): return None, None
        
        dx = p_t['x'] - p_r['x']
        dy = p_t['y'] - p_r['y']
        
        dist = math.hypot(dx, dy)
        global_angle = math.degrees(math.atan2(dy, dx))
        # Vicon计算的相对角度 (Body Frame)
        rel_angle = normalize_angle(global_angle - p_r['yaw'])
        
        return dist, rel_angle

    def run(self):
        try:
            while not rospy.is_shutdown():
                # 1. 阻塞接收视觉数据 (以视觉帧为触发)
                try:
                    msg = self.socket.recv_string(flags=zmq.NOBLOCK)
                    payload = json.loads(msg.split(' ', 1)[1])
                except zmq.Again:
                    time.sleep(0.001)
                    continue
                
                # 只关心球 (Class 6)
                if payload.get('class_id') != 6:
                    continue
                    
                vis_dist = payload.get('distance')
                vis_ang  = payload.get('bearing_body')
                cam_id   = payload.get('cam_idx')
                
                # 2. 获取当前时刻的 Vicon 真值
                vic_dist, vic_ang = self.get_vicon_truth()
                
                if vic_dist is not None:
                    # 3. 计算误差
                    dist_err = vis_dist - vic_dist
                    ang_err  = normalize_angle(vis_ang - vic_ang)
                    
                    # 4. 打印与记录
                    # 绿色表示误差小，红色表示误差大 (仅打印效果)
                    status = "✅" if abs(dist_err) < 0.1 else "❌"
                    
                    print(f"{status} [Cam{cam_id}] "
                          f"Dist: {vis_dist:.2f}v / {vic_dist:.2f}gt (Err: {dist_err:+.2f}) | "
                          f"Ang: {vis_ang:5.1f}v / {vic_ang:5.1f}gt (Err: {ang_err:+.1f})")
                    
                    self.writer.writerow([
                        time.time(),
                        f"{vic_dist:.4f}", f"{vis_dist:.4f}", f"{dist_err:.4f}",
                        f"{vic_ang:.4f}", f"{vis_ang:.4f}", f"{ang_err:.4f}",
                        payload['class_id'], cam_id
                    ])
                    
        except KeyboardInterrupt:
            print(f"\n💾 数据已保存至 {self.filename}")
        finally:
            self.csv_file.close()

if __name__ == "__main__":
    logger = CalibrationLogger()
    logger.run()