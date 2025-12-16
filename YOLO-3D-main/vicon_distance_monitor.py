#!/usr/bin/env python3
"""
VICON Distance Monitor
实时监测一个观察者(A)与多个目标(B, C...)之间的距离和相对方位。
无需摄像头，无需 YOLO。

Usage:
  python3 vicon_distance_monitor.py --observer 8 --targets 10,12
"""

import rospy
import math
import time
import argparse
import csv
import os
from geometry_msgs.msg import TransformStamped

def quat_to_yaw_deg(x, y, z, w):
    """将四元数转换为偏航角 (Yaw, degrees)"""
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return (math.degrees(yaw) + 360.0) % 360.0

class ViconMonitor:
    def __init__(self, observer_id, target_ids):
        self.observer_id = observer_id
        self.target_ids = target_ids
        
        # 存储姿态数据: {id: {'x': val, 'y': val, 'yaw': val, 'last_update': time}}
        self.poses = {}
        
        # 初始化数据结构
        self.monitor_ids = [observer_id] + target_ids
        for rid in self.monitor_ids:
            self.poses[rid] = None
            
            # 订阅 VICON topic
            # topic 格式通常是 vicon/VSWARM{id}/VSWARM{id}
            topic = f"vicon/VSWARM{rid}/VSWARM{rid}"
            rospy.Subscriber(topic, TransformStamped, self._callback, callback_args=rid)
            print(f"📡 已订阅: {topic}")

    def _callback(self, msg, robot_id):
        """VICON 数据回调函数"""
        x = msg.transform.translation.x
        y = msg.transform.translation.y
        rot = msg.transform.rotation
        yaw = quat_to_yaw_deg(rot.x, rot.y, rot.z, rot.w)
        
        self.poses[robot_id] = {
            'x': x,
            'y': y,
            'yaw': yaw,
            'last_update': time.time()
        }

    def get_relationship(self, target_id):
        """计算 Observer -> Target 的关系"""
        obs = self.poses[self.observer_id]
        tgt = self.poses[target_id]

        if obs is None or tgt is None:
            return None

        # 1. 计算欧几里得距离 (直线距离)
        dx = tgt['x'] - obs['x']
        dy = tgt['y'] - obs['y']
        distance = math.hypot(dx, dy)

        # 2. 计算相对方位 (Target 在 Observer 的什么角度)
        # Global bearing (地图上的绝对角度)
        global_angle = math.degrees(math.atan2(dy, dx))
        # Relative bearing (相对于 Observer 车头的角度)
        # 0度=正前方, 90度=左侧, -90度=右侧
        relative_angle = (global_angle - obs['yaw'] + 180 + 360) % 360 - 180

        return {
            'distance': distance,
            'global_angle': global_angle,
            'relative_angle': relative_angle, # 负数偏右，正数偏左
            'obs_pos': (obs['x'], obs['y']),
            'tgt_pos': (tgt['x'], tgt['y'])
        }

def main():
    parser = argparse.ArgumentParser(description='VICON Position Relation Monitor')
    parser.add_argument('--observer', type=int, required=True, help='观察者ID (物体A), 例如: 8')
    parser.add_argument('--targets', type=str, required=True, help='目标ID列表 (物体B,C...), 逗号分隔, 例如: 10,12')
    parser.add_argument('--save_csv', action='store_true', help='是否保存数据到CSV文件')
    parser.add_argument('--freq', type=float, default=1, help='打印频率 (Hz)')
    
    args = parser.parse_args()
    
    # 解析目标列表
    try:
        target_list = [int(x.strip()) for x in args.targets.split(',')]
    except ValueError:
        print("❌ 目标ID格式错误，请使用逗号分隔的数字，例如: 10,12")
        return

    # 初始化 ROS
    rospy.init_node('vicon_distance_monitor', anonymous=True)
    
    monitor = ViconMonitor(args.observer, target_list)
    
    # CSV 设置 (可选)
    csv_file = None
    writer = None
    if args.save_csv:
        filename = f"vicon_data_A{args.observer}_vs_{args.targets.replace(',', '_')}.csv"
        csv_file = open(filename, 'w', newline='')
        fieldnames = ['timestamp', 'observer_id', 'target_id', 'distance_m', 'relative_angle_deg']
        writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
        writer.writeheader()
        print(f"💾 数据将保存至: {filename}")

    rate = rospy.Rate(args.freq)
    print("\n🚀 开始监测... (按 Ctrl+C 停止)\n")
    print(f"{'Time':<10} | {'Rel':<8} | {'Dist (m)':<10} | {'Angle (deg)':<12} | {'Status'}")
    print("-" * 60)

    try:
        while not rospy.is_shutdown():
            current_time = time.time()
            
            # 检查观察者数据是否存在
            if monitor.poses[args.observer] is None:
                print(f"等待观察者 (ID {args.observer}) 数据...", end='\r')
                rate.sleep()
                continue

            # 遍历所有目标计算关系
            output_lines = []
            
            for tid in target_list:
                rel = monitor.get_relationship(tid)
                
                if rel:
                    # 打印格式化信息
                    dist_str = f"{rel['distance']:.4f}"
                    ang_str = f"{rel['relative_angle']:.1f}"
                    print(f"{current_time % 100:<10.2f} | {args.observer}->{tid:<3} | {dist_str:<10} | {ang_str:<12} | ✅ OK")
                    
                    # 保存 CSV
                    if writer:
                        writer.writerow({
                            'timestamp': current_time,
                            'observer_id': args.observer,
                            'target_id': tid,
                            'distance_m': rel['distance'],
                            'relative_angle_deg': rel['relative_angle']
                        })
                else:
                    print(f"{current_time % 100:<10.2f} | {args.observer}->{tid:<3} | {'--':<10} | {'--':<12} | ⏳ 等待Target数据")

            # 打印一个空行分隔不同时间戳的组（如果列表很长）
            if len(target_list) > 1:
                print("-" * 60)

            rate.sleep()

    except rospy.ROSInterruptException:
        pass
    finally:
        if csv_file:
            csv_file.close()
        print("\n🛑 监测结束")

if __name__ == '__main__':
    main()