#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
文件: control_single_search.py
目标: 单车寻球闭环测试
逻辑: 搜索(紫闪) -> 发现并靠近(红闪) -> 停稳(红亮)
"""

import zmq
import json
import time
import sys
import random
import rospy

# 初始化 ROS
if not rospy.core.is_initialized():
    rospy.init_node('single_search_test', anonymous=True, disable_signals=True)

try:
    from robot_driver import RobotDriver
    from light_driver import LightDriver 
except ImportError:
    print("❌ 错误: 找不到驱动文件")
    sys.exit(1)

# === 视觉 ID 映射 (必须与 vision_pub.py 一致) ===
CLS_RED    = 2
CLS_PURPLE = 4
CLS_BALL   = 6

class BallHunter:
    def __init__(self):
        # ⚠️ 修改你的 ID
        self.ROBOT_ID = 15
        self.BROKER_IP = "10.0.2.66"

        # === 核心参数 (调试重点) ===
        self.SEARCH_SPEED = 0.15      # 搜索时的转圈/前进速度
        self.VISION_DIST_MAX = 2.0    # 2米内开始响应球
        self.TARGET_DIST = 0.40       # 目标停车距离 (40cm)
        self.STOP_TOLERANCE = 0.05    # 停车误差容忍度 (±5cm) -> 35~45cm 算停稳
        
        # PID 参数 (如果车晃得厉害，调小 Kp)
        self.KP_LINEAR  = 0.6  # 前后速度系数
        self.KP_ANGULAR = 0.02 # 转向速度系数

        # 状态
        self.state = "INIT"
        
        # 驱动
        try:
            self.driver = RobotDriver(self.ROBOT_ID)
            self.light = LightDriver(self.ROBOT_ID, broker_ip=self.BROKER_IP)
        except Exception as e:
            print(f"❌ 驱动失败: {e}")
            sys.exit(1)

        # 视觉通信
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.SUB)
        self.socket.connect("tcp://localhost:5555") 
        self.socket.setsockopt_string(zmq.SUBSCRIBE, "perception")
        self.socket.setsockopt(zmq.RCVTIMEO, 1000)

        # 初始动作
        self.update_state("SEARCH")

    def update_state(self, new_state):
        if self.state == new_state: return
        self.state = new_state
        print(f"🔄 状态切换 -> {new_state}")
        
        if new_state == "SEARCH":
            self.light.set_cmd("SEARCH")        # 紫闪
        elif new_state == "APPROACH":
            self.light.set_cmd("APPROACH_BALL") # 红闪 (表示正在动)
        elif new_state == "ANCHOR":
            self.light.set_cmd("LEADER_WAIT")   # 红亮 (表示停稳)
            self.driver.stop()

    def run(self):
        print(f"🚀 单车寻球测试启动 | ID: {self.ROBOT_ID}")
        print(f"🎯 目标: 找到篮球并停在 {self.TARGET_DIST}m 处")
        
        while True:
            try:
                msg = self.socket.recv_string()
                _, json_str = msg.split(' ', 1)
                data = json.loads(json_str)
                
                class_id = data.get('class_id', -1)
                dist     = data.get('distance', 999.0)
                
                # === 逻辑核心 ===
                
                if class_id == CLS_BALL and dist < self.VISION_DIST_MAX:
                    # --- 发现球 ---
                    
                    # 计算误差
                    err_dist = dist - self.TARGET_DIST
                    
                    # 判断是否停稳 (迟滞比较，防止反复横跳)
                    if abs(err_dist) < self.STOP_TOLERANCE:
                        # 误差小于 5cm，认为到了
                        self.update_state("ANCHOR")
                    else:
                        # 误差较大，需要调整
                        self.update_state("APPROACH")
                        self.visual_servo(data)
                
                else:
                    # --- 没看到球 ---
                    if self.state == "ANCHOR":
                        # 如果之前已经停稳了，偶尔丢一帧不要紧，保持不动
                        # 除非连续丢很久（这里简化处理，不做超时）
                        pass 
                    else:
                        # 正在找，或者跟丢了
                        self.update_state("SEARCH")
                        self.search_move()

            except zmq.Again:
                self.update_state("SEARCH")
                self.search_move()
            except KeyboardInterrupt:
                break
        
        self.driver.stop()
        self.light.stop()

    def visual_servo(self, data):
        """视觉伺服控制 (PID)"""
        dist = data.get('distance', 0.0)
        bearing = data.get('bearing_body', 0.0)
        
        # 1. 距离控制 (Linear P)
        error_dist = dist - self.TARGET_DIST
        
        # 如果距离太近(<0.4)，倒车(负速度)；如果远，前进(正速度)
        # 限制最大速度 0.2 m/s，防止冲太快
        v_cmd = error_dist * self.KP_LINEAR
        v_cmd = max(-0.2, min(0.2, v_cmd))
        
        # 2. 角度控制 (Angular P)
        # 目标是把球放在画面正中间 (bearing = 0)
        w_cmd = bearing * self.KP_ANGULAR
        w_cmd = max(-0.5, min(0.5, w_cmd)) # 限制最大转速
        
        # 3. 执行
        self.driver.send_velocity_command(v_cmd, 0.0, w_cmd)

    def search_move(self):
        """没找到球时的动作"""
        # 简单的原地旋转扫描，或者缓慢向前画圈
        # 这里写死为：慢速左转 + 极慢速前进 (画大圈)
        self.driver.send_velocity_command(0.05, 0.0, 0.15)

if __name__ == "__main__":
    test = BallHunter()
    test.run()