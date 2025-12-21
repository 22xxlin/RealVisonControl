#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
文件: full_mission_controller.py
版本: 最终定稿版 (Final)
功能: Long-Horizon 全流程控制
参数严格对齐:
  1. 队形距离 = 0.25m (Strictly combined_transport)
  2. 启动延迟 = 1.2s  (Strictly combined_transport)
  3. 搬运速度 = 0.15m/s (Strictly combined_transport)
"""

import zmq
import json
import math
import sys
import time
import argparse
import rospy
import numpy as np
from collections import deque, defaultdict

# 检查 ROS 是否初始化
if not rospy.core.is_initialized():
    rospy.init_node('swarm_mission_node', anonymous=True, disable_signals=True)

try:
    from robot_driver import RobotDriver
    from light_driver import LightDriver
except ImportError:
    print("❌ 错误: 找不到驱动文件 (robot_driver.py / light_driver.py)")
    sys.exit(1)

# ================= 1. 全局配置常量 =================
CLS_BLUE   = 1
CLS_RED    = 2
CLS_GREEN  = 3
CLS_PURPLE = 4
CLS_GRAY   = 5
CLS_BALL   = 6

# 任务参数 (严格对齐 combined_transport_node.py)
TRANSPORT_SPEED = 0.15      # 速度: 0.15 m/s
TRANSPORT_DURATION = 5.0    # 搬运时间
START_DELAY = 1.2           # 延迟: 1.2 s

# 工具函数
def mono(): return time.monotonic()
def normalize_angle(angle): return (angle + 180) % 360 - 180
def wrap_rad_pi(a): return (a + math.pi) % (2.0 * math.pi) - math.pi

# ================= 2. 核心算法类 =================

class StallDetector:
    """堵转检测器"""
    def __init__(self):
        self.history = deque(maxlen=50)
        self.stall_start_time = None
        self.is_stalled = False

    def update(self, rel_x, rel_y, cmd_effort):
        now = mono()
        self.history.append((now, rel_x, rel_y))
        past = None
        for record in self.history:
            if now - record[0] >= 0.3: 
                past = record
                break
        if past is None: return False

        dt = now - past[0]
        if dt < 1e-3: return False
        
        real_vel = math.hypot(rel_x - past[1], rel_y - past[2]) / dt
        if (cmd_effort > 0.05) and (real_vel < 0.02):
            if self.stall_start_time is None: self.stall_start_time = now
            elif now - self.stall_start_time > 1.0: self.is_stalled = True
        else:
            self.stall_start_time = None
            self.is_stalled = False
        return self.is_stalled

class BodyFrameKF:
    """卡尔曼滤波器"""
    def __init__(self, name="target"):
        self.x = None 
        self.P = np.eye(2) * 1.0
        self.last_update_time = mono()
        self.last_r = None

    def update(self, distance, bearing_deg, conf=1.0, truncated=False):
        now = mono()
        dt = now - self.last_update_time
        self.last_update_time = now
        
        self.P += (np.eye(2) * 0.01) * (dt * 10.0)

        b_rad = math.radians(bearing_deg)
        if truncated and self.last_r is not None: meas_dist = self.last_r
        else: meas_dist = distance

        z = np.array([meas_dist * math.cos(b_rad), meas_dist * math.sin(b_rad)])
        
        if self.x is None:
            self.x = z
            self.P = np.eye(2) * 0.5
            self.last_r = distance
            return

        if self.last_r is not None and self.last_r < 0.5:
             if np.linalg.norm(z - self.x) > 0.4: return 

        r_sigma = 0.05
        if truncated: r_sigma *= 10.0
        R = np.eye(2) * (r_sigma ** 2)

        try:
            y = z - self.x
            S = self.P + R
            K = self.P @ np.linalg.inv(S)
            self.x = self.x + K @ y
            self.P = (np.eye(2) - K) @ self.P
        except: pass

        if not truncated: self.last_r = math.hypot(self.x[0], self.x[1])

    def get_state(self):
        if self.x is None: return None
        if mono() - self.last_update_time > 1.0: return None
        return float(self.x[0]), float(self.x[1])

class FormationPilot:
    """智能飞行员 (参数严格对齐 combined_transport_node)"""
    def __init__(self, driver: RobotDriver):
        self.driver = driver
        self.kf_ball = BodyFrameKF("Ball")
        self.kf_leader = BodyFrameKF("Leader")
        self.stall_detector = StallDetector()
        
        self.stable_since = None
        self.is_in_deadband = False
        self.latest_side_intent = None 
        
        # === ⚠️ 关键参数修正 ===
        # 0.25m: 严格对齐 combined_transport_node
        self.TARGET_DIST = 0.25 
        self.FORMATION_ANGLE = 120.0

    def update(self, vision_batch):
        raw_ball = self._select_best(vision_batch, CLS_BALL)
        raw_leader = self._select_best(vision_batch, CLS_RED, "SOLID")

        if raw_ball: self.kf_ball.update(raw_ball['distance'], raw_ball['bearing_body'], raw_ball.get('conf',1.0))
        if raw_leader: self.kf_leader.update(raw_leader['distance'], raw_leader['bearing_body'], raw_leader.get('conf',1.0))

        p_ball = self.kf_ball.get_state()
        p_leader = self.kf_leader.get_state()

        target_pos, leader_ref = None, None
        is_virtual = False
        
        if p_leader:
            dist_leader = math.hypot(p_leader[0], p_leader[1])
            if dist_leader > 0.8: 
                if p_ball: target_pos, leader_ref = p_ball, p_leader
            else: 
                if p_ball: target_pos, leader_ref = p_ball, p_leader
                else: target_pos, leader_ref, is_virtual = p_leader, p_leader, True
        
        if not target_pos:
            self.driver.stop()
            self.stable_since = None
            return "LOST"

        xt, yt = target_pos
        curr_dist = math.hypot(xt, yt)
        dist_err = curr_dist - self.TARGET_DIST

        deadband = 0.05 if self.is_in_deadband else 0.02
        self.is_in_deadband = abs(dist_err) < deadband

        theta_robot = math.atan2(-yt, -xt)
        theta_leader = math.atan2(leader_ref[1]-yt, leader_ref[0]-xt)
        
        diff_rad = math.radians(self.FORMATION_ANGLE)
        err_pos = wrap_rad_pi((theta_leader + diff_rad) - theta_robot)
        err_neg = wrap_rad_pi((theta_leader - diff_rad) - theta_robot)
        
        cost_pos = abs(err_pos)
        cost_neg = abs(err_neg)
        
        bias = 0.25
        if self.latest_side_intent == "+": cost_pos -= bias
        elif self.latest_side_intent == "-": cost_neg -= bias
        
        if cost_pos < cost_neg: final_err, side = err_pos, "+"
        else: final_err, side = err_neg, "-"
        self.latest_side_intent = side

        angle_err_deg = math.degrees(final_err)
        v_tan = 0.0
        
        if not (self.is_in_deadband and abs(angle_err_deg) < 10):
            v_tan = max(-0.3, min(0.3, 0.8 * final_err * curr_dist)) 

        v_rad = 0.0
        if not self.is_in_deadband:
            v_rad = -0.35 * dist_err 
            if v_rad < 0 and self.stall_detector.update(xt, yt, abs(v_rad)):
                v_rad = 0.0 
        
        v_rad = max(-0.25, min(0.25, v_rad))

        th = theta_robot
        vx = v_rad * math.cos(th) - v_tan * math.sin(th)
        vy = v_rad * math.sin(th) + v_tan * math.cos(th)
        
        if curr_dist < 0.15: vx, vy = 0.1 * math.cos(th), 0.1 * math.sin(th)

        self.driver.send_velocity_command(vx, vy, 0.0)

        if (not is_virtual) and self.is_in_deadband and abs(angle_err_deg) < 10.0:
            if self.stable_since is None: self.stable_since = mono()
            elif mono() - self.stable_since > 0.8: 
                self.driver.stop()
                return "LOCKED_LEFT" if side == "+" else "LOCKED_RIGHT"
        else:
            self.stable_since = None
            
        return "ADJUSTING"

    def _select_best(self, batch, cls_id, pattern=None):
        cands = [m for m in batch if m.get('class_id') == cls_id]
        if pattern: cands = [m for m in cands if m.get('pattern') == pattern]
        if not cands: return None
        cands.sort(key=lambda m: (-m.get('conf', 0))) 
        return cands[0]

class EventWatcher:
    """光通信监测"""
    def __init__(self):
        self.hist = defaultdict(lambda: deque(maxlen=40))
    def ingest(self, batch):
        t = mono()
        for msg in batch:
            cid = int(msg.get("class_id", -1))
            pat = str(msg.get("pattern", "OFF"))
            self.hist[cid].append((t, pat))
    def stable_pattern(self, cid, pat, need_k, within_s):
        h = self.hist[cid]
        if not h: return False
        t0 = mono() - within_s
        cnt = sum(1 for (t, p) in h if t >= t0 and p == pat)
        return cnt >= need_k

# ================= 3. 主任务控制器 =================

class SwarmMissionController:
    def __init__(self, robot_id):
        self.ROBOT_ID = robot_id
        print(f"🔧 初始化控制器 | Robot ID: {self.ROBOT_ID}")
        
        self.BROKER_IP = "10.0.2.66"

        self.DIST_FIND_BALL = 0.8
        self.DIST_STOP_BALL = 0.23
        
        self.state = "INIT"
        self.my_role = "SEARCHER"
        self.start_time = mono()
        self.last_red_time = 0.0
        self.red_detect_count = 0

        # 新增：强制搜索时间（前N秒忽略红灯）
        self.FORCE_SEARCH_TIME = 3.0  # 前3秒强制搜索，不切换角色
        
        self.t_armed_start = 0.0
        self.t_run_start = 0.0

        try:
            self.driver = RobotDriver(self.ROBOT_ID)
            self.light = LightDriver(self.ROBOT_ID, broker_ip=self.BROKER_IP)
        except Exception as e:
            print(f"❌ 驱动错误: {e}")
            sys.exit(1)

        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.SUB)
        self.socket.connect("tcp://localhost:5555")
        self.socket.setsockopt_string(zmq.SUBSCRIBE, "perception")
        self.socket.setsockopt(zmq.RCVTIMEO, 10)  # 改为10ms，避免阻塞过久

        self.pilot = FormationPilot(self.driver)
        self.watcher = EventWatcher()

        self.update_state("SEARCH")

    def update_state(self, new_state):
        if self.state == new_state: return
        print(f"🔄 [State] {self.my_role} | {self.state} -> {new_state}")
        self.state = new_state
        
        if new_state == "SEARCH":          self.light.set_cmd("SEARCH")     
        elif new_state == "APPROACH_BALL": self.light.set_cmd("APPROACH_BALL") 
        elif new_state == "LEADER_WAIT":   self.light.set_cmd("LEADER_WAIT")  
        elif new_state == "BIDDING":       pass 
        elif new_state == "READY_WAIT":    pass 
        elif new_state == "PREWARM":       self.light.set_cmd("LEADER_GO")    
        elif new_state == "ARMED":         self.light.set_cmd("FOLLOWER_PUSH") 
        elif new_state == "RUN":           pass 
        elif new_state == "DONE":          self.light.set_cmd("OFF"); self.driver.stop()

    def run(self):
        print(f"🚀 任务启动 | ID: {self.ROBOT_ID} | 搜索策略: 13横/10前/15后")
        print(f"📡 ZMQ连接: tcp://localhost:5555 | 订阅: perception")

        loop_count = 0
        last_debug_time = mono()

        while True:
            try:
                loop_count += 1

                # 1. 视觉数据读取 (非阻塞，快速排空队列)
                batch = []
                latest_data = None
                recv_count = 0

                # 快速排空ZMQ队列，只保留最新一帧
                for _ in range(50):  # 最多读50次，防止死循环
                    try:
                        msg = self.socket.recv_string(flags=zmq.NOBLOCK)
                        recv_count += 1
                        _, json_str = msg.split(' ', 1)
                        data = json.loads(json_str)
                        if 'objects' in data:
                            latest_data = data['objects']
                        else:
                            latest_data = [data]
                    except zmq.Again:
                        break  # 队列已空，立即退出
                    except Exception as e:
                        pass  # 忽略解析错误

                # 使用最新一帧数据
                if latest_data is not None:
                    batch = latest_data

                # 调试输出 (每秒一次)
                now = mono()
                if now - last_debug_time > 1.0:
                    print(f"🔍 [Debug] 循环:{loop_count} | 收到帧:{recv_count} | 对象数:{len(batch)} | 角色:{self.my_role} | 状态:{self.state}")
                    last_debug_time = now

                self.watcher.ingest(batch)
                
                has_ball = any(m['class_id'] == CLS_BALL for m in batch)
                ball_dist = 999.0
                ball_bearing = 0.0
                for m in batch:
                    if m['class_id'] == CLS_BALL:
                        ball_dist = m.get('distance', 999.0)
                        ball_bearing = normalize_angle(m.get('bearing_body', 0.0))
                        break
                
                has_red = any(m['class_id'] == CLS_RED for m in batch)
                now = mono()

                # --- 状态机逻辑 ---

                if self.my_role == "SEARCHER":
                    if has_ball and ball_dist < self.DIST_FIND_BALL:
                        self.my_role = "LEADER"
                        self.move_to_ball_simple(ball_dist, ball_bearing)
                    elif has_red and (now - self.start_time > self.FORCE_SEARCH_TIME):
                        if now - self.last_red_time < 0.3: self.red_detect_count += 1
                        else: self.red_detect_count = 1
                        self.last_red_time = now
                        if self.red_detect_count >= 6:
                            print("✅ 发现 Leader! 切换身份 -> FOLLOWER")
                            self.my_role = "FOLLOWER"
                            self.update_state("BIDDING")
                    else:
                        if now - self.last_red_time > 1.0: self.red_detect_count = 0
                        self.update_state("SEARCH")
                        self.omni_search_move()

                elif self.my_role == "LEADER":
                    if self.state == "APPROACH_BALL":
                        if has_ball: self.move_to_ball_simple(ball_dist, ball_bearing)
                        else: self.driver.stop()
                    elif self.state == "LEADER_WAIT":
                        left_ready = self.watcher.stable_pattern(CLS_GREEN, "SOLID", 3, 0.6)
                        right_ready = self.watcher.stable_pattern(CLS_BLUE, "SOLID", 3, 0.6)
                        if left_ready and right_ready:
                            print("🎉 队形组建完成！发送 GO 信号...")
                            self.update_state("PREWARM")
                            self.t_armed_start = now + 2.0 
                        else:
                            self.driver.stop()
                    elif self.state == "PREWARM":
                        if now > self.t_armed_start:
                            self.update_state("ARMED")
                            self.t_run_start = now + START_DELAY
                            print(f"⏱️ Leader 武装! {START_DELAY}s 后出发")
                    elif self.state == "ARMED":
                        if now > self.t_run_start:
                            self.update_state("RUN")
                    elif self.state == "RUN":
                        self.transport_move()

                elif self.my_role == "FOLLOWER":
                    if self.state == "BIDDING":
                        status = self.pilot.update(batch)
                        if self.pilot.latest_side_intent == "+": self.light.set_cmd("BID_LEFT")
                        elif self.pilot.latest_side_intent == "-": self.light.set_cmd("BID_RIGHT")
                        if status.startswith("LOCKED"):
                            self.update_state("READY_WAIT")
                            if "LEFT" in status: self.light.set_cmd("LOCK_LEFT")
                            else: self.light.set_cmd("LOCK_RIGHT")
                            print(f"🔒 {status} - 等待 Leader 信号")
                    elif self.state == "READY_WAIT":
                        self.driver.stop()
                        if self.watcher.stable_pattern(CLS_PURPLE, "SOLID", 3, 0.5):
                            print("🚀 收到 GO 信号!")
                            self.update_state("ARMED")
                            self.t_run_start = now + START_DELAY
                    elif self.state == "ARMED":
                        if now > self.t_run_start:
                            self.update_state("RUN")
                    elif self.state == "RUN":
                        self.transport_move()

                if self.state == "RUN" and (now - self.t_run_start > TRANSPORT_DURATION):
                    self.update_state("DONE")
                
                time.sleep(0.03)

            except KeyboardInterrupt:
                break
            except Exception as e:
                print(f"Error loop: {e}")
        
        self.light.stop()
        self.driver.stop()

    def omni_search_move(self):
        """搜索策略: 13横 / 10前 / 15后"""
        elapsed = mono() - self.start_time
        vx, vy = 0.0, 0.0
        if elapsed < 5.0:
            if self.ROBOT_ID == 13:   vy = -0.2
            elif self.ROBOT_ID == 10: vx = 0.2
            elif self.ROBOT_ID == 15: vx = -0.2
        else:
            vy = -0.2
            vx = 0.15 * math.sin(elapsed * math.pi)
        self.driver.send_velocity_command(vx, vy, 0.0)

    def move_to_ball_simple(self, dist, bearing):
        dist_error = dist - self.DIST_STOP_BALL
        if dist_error > 0:
            self.update_state("APPROACH_BALL")
            speed = max(0.0, min(0.2, dist_error * 0.6))
            rad = math.radians(bearing)
            vx = speed * math.cos(rad)
            vy = speed * math.sin(rad)
            self.driver.send_velocity_command(vx, vy, 0.0)
        else:
            self.update_state("LEADER_WAIT")

    def transport_move(self):
        self.driver.send_velocity_command(TRANSPORT_SPEED, 0.0, 0.0)

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Swarm Mission Controller")
    parser.add_argument("-i", "--id", type=int, required=True, help="Robot ID (e.g., 10, 13, 15)")
    args = parser.parse_args()
    
    print(f"⚙️  正在启动机器人 ID: {args.id} ...")
    c = SwarmMissionController(robot_id=args.id)
    c.run()