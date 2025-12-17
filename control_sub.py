#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
文件: control_sub.py
版本: 编队调试专用版 (Formation Debug Mode)
功能: 
  1. 专注于调试 "Search -> Discover -> Form Triangle" 流程。
  2. 🚫 禁用了所有 "搬运 (Transport)" 相关的状态跳转。
  3. ✅ 当队形组建完成后，所有车会保持静止并亮灯，方便拍照/测量。
"""

import zmq
import json
import math
import sys
import time
import rospy
import random

if not rospy.core.is_initialized():
    rospy.init_node('swarm_controller', anonymous=True, disable_signals=True)

try:
    from robot_driver import RobotDriver
    from light_driver import LightDriver 
except ImportError:
    print("❌ 错误: 找不到驱动文件")
    sys.exit(1)

# ================= 视觉 Class ID 映射 =================
CLS_BLUE   = 1
CLS_RED    = 2
CLS_GREEN  = 3
CLS_PURPLE = 4
CLS_GRAY   = 5
CLS_BALL   = 6
CLS_FLAG   = 7

def normalize_angle(angle):
    return (angle + 180) % 360 - 180

class SwarmController:
    def __init__(self):
        # === ⚠️⚠️⚠️ 现场必改: 修改每台车的 ID (10 / 13 / 15) ===
        self.ROBOT_ID = 15
        
        self.BROKER_IP = "10.0.2.66"
        
        # === 核心阈值 ===
        self.DIST_FIND_BALL   = 0.8
        self.DIST_STOP_BALL   = 0.23
        self.DIST_TRIANGLE    = 0.5
        self.DIST_FINISH      = 1.5
        
        # === 连续帧滤波参数 ===
        self.RED_CONFIRM_THRESHOLD = 6
        self.red_detect_count = 0 
        self.last_red_time = 0.0
        
        # === 搜索参数 ===
        self.SEARCH_SPEED  = 0.2
        self.DISPERSE_TIME = 5.0
        self.start_time = time.time()
        
        # === 状态变量 ===
        self.last_seen_left_ready = 0.0
        self.last_seen_right_ready = 0.0
        self.state = "INIT"
        self.my_role = "SEARCHER" 
        
        # === 角色分配 ===
        if self.ROBOT_ID == 10:
            self.target_slot_angle = -120.0 
            self.slot_name = "LEFT (-120)"
        elif self.ROBOT_ID == 13:
            self.target_slot_angle = 120.0
            self.slot_name = "RIGHT (+120)"
        else:
            self.target_slot_angle = 120.0
            self.slot_name = "RIGHT (+120)"

        try:
            self.driver = RobotDriver(self.ROBOT_ID)
            self.light = LightDriver(self.ROBOT_ID, broker_ip=self.BROKER_IP)
        except Exception as e:
            print(f"❌ 驱动失败: {e}")
            sys.exit(1)

        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.SUB)
        self.socket.connect("tcp://localhost:5555") 
        self.socket.setsockopt_string(zmq.SUBSCRIBE, "perception")
        self.socket.setsockopt(zmq.RCVTIMEO, 1000)

        self.update_state("SEARCH")

    def update_state(self, new_state):
        if self.state == new_state: return
        self.state = new_state
        print(f"🔄 [State] {self.my_role} -> {new_state}")
        
        if new_state == "SEARCH":           self.light.set_cmd("SEARCH")
        elif new_state == "APPROACH_BALL":  self.light.set_cmd("APPROACH_BALL")
        elif new_state == "LEADER_WAIT":    self.light.set_cmd("LEADER_WAIT"); self.driver.stop()
        elif new_state == "BIDDING":
            if self.target_slot_angle < 0:  self.light.set_cmd("BID_LEFT")
            else:                           self.light.set_cmd("BID_RIGHT")
        elif new_state == "READY":
            # 锁定状态: 绿灯(左) 或 蓝灯(右) 常亮
            if self.target_slot_angle < 0:  self.light.set_cmd("LOCK_LEFT")
            else:                           self.light.set_cmd("LOCK_RIGHT")
            self.driver.stop()
            
        # 🚫 --- 以下状态被屏蔽 (Debug Mode) ---
        # elif new_state == "TRANSPORT_LEADER": self.light.set_cmd("LEADER_GO")
        # elif new_state == "TRANSPORT_FOLLOWER": self.light.set_cmd("FOLLOWER_PUSH")
        # elif new_state == "FINISH":         self.light.set_cmd("OFF"); self.driver.stop()

    def run(self):
        print(f"🚀 编队调试模式启动 | ID: {self.ROBOT_ID} | Slot: {self.slot_name}")
        
        while True:
            try:
                msg = self.socket.recv_string()
                _, json_str = msg.split(' ', 1)
                data = json.loads(json_str)
                
                class_id = data.get('class_id', -1)
                pattern  = data.get('pattern', 'OFF')
                dist     = data.get('distance', 999.0)
                
                raw_bearing = data.get('bearing_body', 0.0)
                bearing = normalize_angle(raw_bearing)
                
                now = time.time()

                # --- A. 搜索者 (SEARCHER) ---
                if self.my_role == "SEARCHER":
                    if class_id == CLS_BALL and dist < self.DIST_FIND_BALL:
                        self.my_role = "LEADER"
                        self.move_to_ball(dist, bearing)
                    
                    elif class_id == CLS_RED:
                        if now - self.last_red_time < 0.3:
                            self.red_detect_count += 1
                        else:
                            self.red_detect_count = 1
                        self.last_red_time = now
                        
                        if self.red_detect_count >= self.RED_CONFIRM_THRESHOLD:
                            print(f"✅ 发现 Leader! 切换身份...")
                            self.my_role = "FOLLOWER"
                            self.update_state("BIDDING")
                    
                    else:
                        if now - self.last_red_time > 1.0: self.red_detect_count = 0
                        self.update_state("SEARCH")
                        self.omni_search_move()

                # --- B. 队长 (LEADER) ---
                elif self.my_role == "LEADER":
                    if self.state == "APPROACH_BALL":
                        if class_id == CLS_BALL: self.move_to_ball(dist, bearing)
                        else: self.driver.stop()
                    
                    elif self.state == "LEADER_WAIT":
                        # 仅检测就位，不触发搬运
                        if class_id == CLS_GREEN and pattern == 'SOLID': self.last_seen_left_ready = now
                        if class_id == CLS_BLUE and pattern == 'SOLID':  self.last_seen_right_ready = now
                        
                        # 检测到双侧就位 -> 仅打印 Log，不跳转
                        is_left_ready = (now - self.last_seen_left_ready < 1.0)
                        is_right_ready = (now - self.last_seen_right_ready < 1.0)
                        
                        if is_left_ready and is_right_ready:
                            print("🎉🎉🎉 [SUCCESS] 编队组建完成！所有单位就位！ 🎉🎉🎉")
                            # 可以在这里让 Leader 闪烁庆祝一下，或者保持静止
                        elif is_left_ready:
                            print("⏳ 左侧就位... 等待右侧")
                        elif is_right_ready:
                            print("⏳ 右侧就位... 等待左侧")
                            
                        # 🚫 屏蔽跳转
                        # if ...: self.update_state("TRANSPORT_LEADER")

                    # 🚫 屏蔽搬运逻辑
                    # elif self.state == "TRANSPORT_LEADER": ...

                # --- C. 队员 (FOLLOWER) ---
                elif self.my_role == "FOLLOWER":
                    # 🚫 屏蔽收到紫色灯的搬运指令
                    # if class_id == CLS_PURPLE: ...
                    
                    if class_id == CLS_RED:
                        self.maintain_formation(dist, bearing)
                    else:
                        # 丢失目标时停车
                        self.driver.stop()

            except zmq.Again:
                if self.my_role == "SEARCHER": self.omni_search_move()
                else: self.driver.stop()
            except KeyboardInterrupt:
                break
        
        self.light.stop()
        self.driver.stop()

    def omni_search_move(self):
        """扇形展开 + S形平推"""
        elapsed = time.time() - self.start_time
        vx, vy = 0.0, 0.0
        if elapsed < self.DISPERSE_TIME:
            if self.ROBOT_ID == 10:   vx = self.SEARCH_SPEED
            elif self.ROBOT_ID == 13: vx = -self.SEARCH_SPEED
            else:                     vy = -self.SEARCH_SPEED
        else:
            vy = -self.SEARCH_SPEED 
            vx = 0.15 * math.sin(elapsed * math.pi)
        self.driver.send_velocity_command(vx, vy, 0.0)

    def move_to_ball(self, dist, bearing):
        """Leader 找球"""
        dist_error = dist - self.DIST_STOP_BALL
        if dist_error > 0:
            self.update_state("APPROACH_BALL")
            total_speed = max(0.0, min(0.2, dist_error * 0.6))
            theta_rad = math.radians(bearing)
            v_x = total_speed * math.cos(theta_rad)
            v_y = total_speed * math.sin(theta_rad)
            self.driver.send_velocity_command(v_x, v_y, 0.0)
        else:
            self.update_state("LEADER_WAIT")

    def maintain_formation(self, dist, bearing):
        """Follower 保持阵型"""
        dist_err = dist - self.DIST_TRIANGLE
        
        # 1. 距离控制 (P控制)
        v_x = max(-0.2, min(0.2, -dist_err * 0.8)) # 负号因为我们要靠近
        
        # 2. 角度控制 (P控制)
        bearing_err = normalize_angle(bearing - self.target_slot_angle)
        v_y = bearing_err * 0.015
        v_y = max(-0.2, min(0.2, v_y))
        
        # 3. 判定是否就位 (阈值要调好，太严很难READY，太松队形不准)
        # 距离误差 < 15cm, 角度误差 < 15度
        if abs(dist_err) < 0.15 and abs(bearing_err) < 15.0:
            self.update_state("READY")
            # ⚠️ 调试核心: 一旦 READY，强制停车，防止在临界点抖动
            self.driver.stop() 
        else:
            self.update_state("BIDDING")
            self.driver.send_velocity_command(v_x, v_y, 0.0) # 只有没就位才动

    def move_towards_flag(self, bearing):
        pass # Debug模式下禁用

if __name__ == "__main__":
    c = SwarmController()
    c.run()