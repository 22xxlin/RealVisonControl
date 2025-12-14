#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
文件: control_sub.py
版本: 最终发布版 (Robust Debounce)
更新点:
  1. 引入连续帧检测: 必须连续 6 帧看到红灯才变 Follower
  2. 保持之前的: 矢量分解(解决乱跑)、全向锁定、无限视距
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
        self.DIST_STOP_BALL   = 0.2
        self.DIST_TRIANGLE    = 0.8
        self.DIST_FINISH      = 1.5
        
        # === 连续帧滤波参数 (关键新增) ===
        self.RED_CONFIRM_THRESHOLD = 6  # 需要连续看到6次
        self.red_detect_count = 0       # 当前计数
        self.last_red_time = 0.0        # 上次看到的时间
        
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
            if self.target_slot_angle < 0:  self.light.set_cmd("LOCK_LEFT")
            else:                           self.light.set_cmd("LOCK_RIGHT")
            self.driver.stop()
        elif new_state == "TRANSPORT_LEADER": self.light.set_cmd("LEADER_GO")
        elif new_state == "TRANSPORT_FOLLOWER": self.light.set_cmd("FOLLOWER_PUSH")
        elif new_state == "FINISH":         self.light.set_cmd("OFF"); self.driver.stop()

    def run(self):
        print(f"🚀 全向蜂群启动 | ID: {self.ROBOT_ID} | Slot: {self.slot_name}")
        
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
                    # 1. 发现球 -> 变 Leader
                    if class_id == CLS_BALL and dist < self.DIST_FIND_BALL:
                        self.my_role = "LEADER"
                        self.move_to_ball(dist, bearing)
                    
                    # 2. 发现红灯 -> 变 Follower (⚠️ 核心修改: 连续帧滤波)
                    elif class_id == CLS_RED:
                        # 如果距离上一次看到红灯不超过 0.3s (说明是连续的)
                        if now - self.last_red_time < 0.3:
                            self.red_detect_count += 1
                        else:
                            # 如果断了很久，重置计数
                            self.red_detect_count = 1
                        
                        # 更新时间戳
                        self.last_red_time = now
                        
                        # 打印调试信息，让你看到进度
                        # print(f"🧐 疑似发现 Leader... 确认度: {self.red_detect_count}/{self.RED_CONFIRM_THRESHOLD}")
                        
                        # 只有攒够 6 次才切换
                        if self.red_detect_count >= self.RED_CONFIRM_THRESHOLD:
                            print(f"✅ 确认发现 Leader (连续 {self.red_detect_count} 帧)! 切换身份...")
                            self.my_role = "FOLLOWER"
                            self.update_state("BIDDING")
                    
                    # 3. 没发现 -> 搜索
                    else:
                        # (可选) 如果很久没看到红灯了，要把计数器清零，防止跨时间累积
                        if now - self.last_red_time > 1.0:
                            self.red_detect_count = 0
                            
                        self.update_state("SEARCH")
                        self.omni_search_move()

                # --- B. 队长 (LEADER) ---
                elif self.my_role == "LEADER":
                    if self.state == "APPROACH_BALL":
                        if class_id == CLS_BALL: self.move_to_ball(dist, bearing)
                        else: self.driver.stop()
                    elif self.state == "LEADER_WAIT":
                        if class_id == CLS_GREEN and pattern == 'SOLID': self.last_seen_left_ready = now
                        if class_id == CLS_BLUE and pattern == 'SOLID':  self.last_seen_right_ready = now
                        if (now - self.last_seen_left_ready < 1.0) and (now - self.last_seen_right_ready < 1.0):
                            self.update_state("TRANSPORT_LEADER")
                    elif self.state == "TRANSPORT_LEADER":
                        if class_id == CLS_FLAG:
                            if dist < self.DIST_FINISH: self.update_state("FINISH")
                            else: self.move_towards_flag(bearing)
                        else:
                            self.driver.send_velocity_command(0.15, 0.0, 0.0)

                # --- C. 队员 (FOLLOWER) ---
                elif self.my_role == "FOLLOWER":
                    if class_id == CLS_PURPLE or self.state == "TRANSPORT_FOLLOWER":
                        self.update_state("TRANSPORT_FOLLOWER")
                        self.driver.send_velocity_command(0.15, 0.0, 0.0)
                    elif class_id == CLS_RED:
                        self.maintain_formation(dist, bearing)
                    else:
                        if self.state == "TRANSPORT_FOLLOWER": self.driver.stop()
                        else: self.driver.stop()

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
        """Leader 找球 (矢量分解版)"""
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
        v_x = max(-0.2, min(0.2, dist_err * 0.8))
        bearing_err = normalize_angle(bearing - self.target_slot_angle)
        v_y = bearing_err * 0.015
        v_y = max(-0.2, min(0.2, v_y))
        
        if abs(dist_err) < 0.15 and abs(bearing_err) < 15.0:
            self.update_state("READY")
        else:
            self.update_state("BIDDING")
        self.driver.send_velocity_command(v_x, v_y, 0.0)

    def move_towards_flag(self, bearing):
        v_y = bearing * 0.01
        self.driver.send_velocity_command(0.15, v_y, 0.0)

if __name__ == "__main__":
    c = SwarmController()
    c.run()