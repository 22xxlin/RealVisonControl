#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
文件: control_sub.py (防抖动优化版)
功能: 篮球寻迹 + 状态机 + 视觉暂留机制
"""

import zmq
import json
import math
import sys
import time

try:
    from robot_driver import RobotDriver
    from light_driver import LightDriver 
except ImportError:
    print("❌ 错误: 找不到驱动文件！")
    sys.exit(1)

class BallController:
    def __init__(self):
        self.ROBOT_ID = 15
        self.TARGET_DIST = 0.2
        self.MAX_SPEED = 0.3
        self.BROKER_IP = "10.0.2.66"
        
        # === 1. 新增：视觉暂留计时器 ===
        self.last_ball_time = 0.0  # 上次看到球的时间戳
        self.LOST_TIMEOUT = 1.0    # 忍受丢失的时间 (秒)，建议 0.5 ~ 1.0
        
        print(f"🏀 控制器启动 | ID: {self.ROBOT_ID}")

        try:
            self.driver = RobotDriver(self.ROBOT_ID)
            self.light = LightDriver(self.ROBOT_ID, broker_ip=self.BROKER_IP) 
        except Exception as e:
            print(f"❌ 驱动初始化失败: {e}")
            sys.exit(1)

        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.SUB)
        self.socket.connect("tcp://localhost:5555") 
        self.socket.setsockopt_string(zmq.SUBSCRIBE, "perception")
        self.socket.setsockopt(zmq.RCVTIMEO, 1000)

        self.state = "INIT"
        self.update_state("SEARCH")

    def update_state(self, new_state):
        if self.state == new_state:
            return
        
        self.state = new_state
        print(f"🔄 状态切换: {new_state}")

        if new_state == "SEARCH":
            self.light.set_cmd("SEARCH")
        elif new_state == "APPROACH":
            self.light.set_cmd("FOUND")
        elif new_state == "ARRIVED":
            self.light.set_cmd("ARRIVED")
            self.driver.stop()
        elif new_state == "LOST":
            self.light.set_cmd("IDLE")
            self.driver.stop()

    def run(self):
        print("🚀 主循环开始...")
        while True:
            try:
                msg = self.socket.recv_string()
                _, json_str = msg.split(' ', 1)
                data = json.loads(json_str)
                pattern = data.get('pattern', 'IDLE')
                
                # 获取当前时间
                now = time.time()

                if pattern == 'BASKETBALL':
                    # === 看到球了 ===
                    # 1. 更新最后一次看到球的时间
                    self.last_ball_time = now  # <--- 修改点：刷新计时器

                    distance = data.get('distance', 0.0)
                    
                    if abs(distance - self.TARGET_DIST) < 0.05:
                        self.update_state("ARRIVED")
                        continue

                    # 只要看到球，就强制切回 APPROACH (除非已经到了)
                    if self.state != "ARRIVED":
                        self.update_state("APPROACH")
                    
                    self.move_to_ball(data)

                else:
                    # === 没看到球 ===
                    if self.state == "ARRIVED": 
                        continue

                    # === 修改点：增加防抖逻辑 ===
                    # 计算距离上次看到球过去了多久
                    time_since_seen = now - self.last_ball_time
                    
                    if time_since_seen < self.LOST_TIMEOUT:
                        # 虽然这一帧没看到，但在“忍受期”内，认为是视觉丢帧
                        # 保持 APPROACH 状态，不要切 SEARCH
                        # 可选：这期间可以让车稍微减速或者维持上一次的速度
                        pass 
                    else:
                        # 真的超时了 (超过1秒没看到)，才认为是真丢了
                        self.update_state("SEARCH")

            except zmq.Again:
                print("⚠️ 视觉连接断开")
                self.update_state("LOST")
                
            except KeyboardInterrupt:
                print("\n🛑 程序中止")
                break
        
        self.light.stop()
        self.driver.stop()

    def move_to_ball(self, data):
        distance = data.get('distance', 0.0)
        bearing = data.get('bearing_body', 0.0)
        
        error = distance - self.TARGET_DIST
        v_cmd = error * 0.8
        v_cmd = max(-self.MAX_SPEED, min(self.MAX_SPEED, v_cmd))
        
        rad = math.radians(bearing)
        vx = v_cmd * math.cos(rad)
        vy = v_cmd * math.sin(rad)
        
        self.driver.send_velocity_command(vx, vy, 0.0)

if __name__ == "__main__":
    c = BallController()
    c.run()