#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
文件: data_logger.py (双模版)
功能: 
  1. 自动记录球 (Class 6) -> 对比 Target Vicon 真值
  2. 自动记录车 (Class 0-5) -> 对比 Leader Vicon 真值 (用于调试融合参数)
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
# 请确保这些 Topic 对应正确的 Vicon 对象
ROBOT_VICON_TOPIC  = "/vicon/VSWARM15/VSWARM15"  # 观测者 (本机)
TARGET_VICON_TOPIC = "/vicon/VSWARM45/VSWARM45"  # 球 (Ball) 的真值
LEADER_VICON_TOPIC = "/vicon/VSWARM13/VSWARM13"  # 另一辆车 (Leader) 的真值

# ZMQ 配置
ZMQ_PORT = 5555

# ================= 工具 =================
def quat_to_yaw_deg(x, y, z, w):
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return (math.degrees(yaw) + 360.0) % 360.0

def normalize_angle(angle):
    """归一化到 [-180, 180]"""
    return (angle + 180.0) % 360.0 - 180.0

class FullLogger:
    def __init__(self):
        rospy.init_node('full_data_logger', anonymous=True)
        
        # 1. 状态存储
        self.poses = {'robot': None, 'target': None, 'leader': None}
        
        # 2. 订阅 Vicon
        rospy.Subscriber(ROBOT_VICON_TOPIC, TransformStamped, self._cb, 'robot')
        rospy.Subscriber(TARGET_VICON_TOPIC, TransformStamped, self._cb, 'target')
        rospy.Subscriber(LEADER_VICON_TOPIC, TransformStamped, self._cb, 'leader')
        
        # 3. ZMQ 连接
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.SUB)
        self.socket.connect(f"tcp://localhost:{ZMQ_PORT}")
        self.socket.setsockopt_string(zmq.SUBSCRIBE, "perception")
        
        # 4. CSV 初始化
        ts = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        self.filename = f"log_full_{ts}.csv"
        self.csv_file = open(self.filename, 'w', newline='')
        self.writer = csv.writer(self.csv_file)
        
        # CSV 表头
        header = [
            "Time", 
            "Vision_Dist", "Vision_Geo", "Vision_Width", # 视觉数据
            "Vicon_Dist", "Dist_Err",                    # 距离真值与误差
            "Vision_Ang", "Vicon_Ang", "Ang_Err",        # 角度真值与误差
            "Target_Type", "Vision_Class", "Cam_ID"      # 目标类型标记
        ]
        self.writer.writerow(header)
        
        print(f"📝 双模日志记录中: {self.filename}")
        print("   - Class 6 (Ball) -> 对比 Target (VSWARM45)")
        print("   - Class 0-5 (Car) -> 对比 Leader (VSWARM10)")

    def _cb(self, msg, key):
        x = msg.transform.translation.x
        y = msg.transform.translation.y
        rot = msg.transform.rotation
        yaw = quat_to_yaw_deg(rot.x, rot.y, rot.z, rot.w)
        self.poses[key] = {'x': x, 'y': y, 'yaw': yaw, 'ts': time.time()}

    def get_ground_truth(self, target_key):
        """
        动态计算本机(robot)与指定目标(target_key)之间的真值关系
        target_key: 'target' (对于球) 或 'leader' (对于车)
        """
        p_r = self.poses['robot']
        p_t = self.poses[target_key] # 动态取目标
        now = time.time()
        
        # 基础检查
        if not p_r or not p_t: return None
        # 数据超时检查 (0.5s)
        if (now - p_r['ts'] > 0.5) or (now - p_t['ts'] > 0.5): return None
        
        # 计算距离
        dx = p_t['x'] - p_r['x']
        dy = p_t['y'] - p_r['y']
        vicon_dist = math.hypot(dx, dy)
        
        # 计算相对角度 (Bearing)
        global_ang = math.degrees(math.atan2(dy, dx))
        vicon_rel_ang = normalize_angle(global_ang - p_r['yaw'])
        
        return {
            'vicon_dist': vicon_dist,
            'vicon_rel_ang': vicon_rel_ang
        }

    def run(self):
        try:
            while not rospy.is_shutdown():
                try:
                    msg = self.socket.recv_string(flags=zmq.NOBLOCK)
                    payload = json.loads(msg.split(' ', 1)[1])
                except zmq.Again:
                    time.sleep(0.001)
                    continue
                
                cls_id = payload.get('class_id')
                
                # === 自动决定对比目标 ===
                if cls_id == 6:
                    target_key = 'target' # 球 -> 对比球的Vicon
                    target_label = "BALL"
                else:
                    target_key = 'leader' # 车 -> 对比Leader的Vicon (用于校准融合参数)
                    target_label = "CAR " # 加空格为了对齐
                
                # === 获取数据 ===
                vis_dist = payload.get('distance')
                vis_geo = payload.get('dist_geo', -1.0)
                vis_width = payload.get('dist_width', -1.0)
                vis_ang = payload.get('bearing_body')
                cam_id = payload.get('cam_idx')
                
                # === 获取对应的真值 ===
                gt = self.get_ground_truth(target_key)
                
                if gt:
                    dist_err = vis_dist - gt['vicon_dist']
                    ang_err = normalize_angle(vis_ang - gt['vicon_rel_ang'])
                    
                    status = "✅" if abs(dist_err) < 0.15 else "❌"
                    
                    # 打印 Log (包含类型标记)
                    print(f"{status} [{target_label}-Cam{cam_id}] "
                          f"Fus:{vis_dist:.2f} (G:{vis_geo:.1f}/W:{vis_width:.1f}) | "
                          f"Err:{dist_err:+.2f}m")
                    
                    # 写入 CSV
                    self.writer.writerow([
                        time.time(),
                        f"{vis_dist:.4f}", f"{vis_geo:.4f}", f"{vis_width:.4f}",
                        f"{gt['vicon_dist']:.4f}", f"{dist_err:.4f}",
                        f"{vis_ang:.4f}", f"{gt['vicon_rel_ang']:.4f}", f"{ang_err:.4f}",
                        target_label, cls_id, cam_id
                    ])
                    
        except KeyboardInterrupt:
            print(f"\n💾 Log 保存完毕: {self.filename}")
        finally:
            self.csv_file.close()

if __name__ == "__main__":
    logger = FullLogger()
    logger.run()