#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
文件: full_mission_controller.py
版本: Merged_With_V2.6_Test_Logic
功能: Long-Horizon 全流程控制 (集成 V2.6 平移锁定逻辑)
说明: 
  - 已用 test_formation_lock_vision_ekf.py 的核心算法替换了 FormationPilot
  - 包含了 Tracking -> Sliding (0.2m) -> Finished 的完整流程
"""

import zmq
import json
import math
import sys
import time
import argparse
import rospy
import collections
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
    pass 

# ================= 1. 全局配置常量 =================
CLS_BLUE   = 1
CLS_RED    = 2
CLS_GREEN  = 3
CLS_PURPLE = 4
CLS_GRAY   = 5
CLS_BALL   = 6

# 任务参数
TRANSPORT_SPEED = 0.15      
TRANSPORT_DURATION = 8.0    
START_DELAY = 1.2           

# 工具函数
def mono(): return time.monotonic()
def normalize_angle(angle): return (angle + 180) % 360 - 180
def wrap_rad_pi(a): return (a + math.pi) % (2.0 * math.pi) - math.pi
def wrap_deg_180(a): return (a + 180.0) % 360.0 - 180.0

# ================= 2. 核心算法类 (来自 test_formation_lock_vision_ekf.py) =================

class StallDetector:
    """堵转检测器 (V2.6版)"""
    def __init__(self):
        self.history = collections.deque(maxlen=50)
        self.stall_start_time = None
        self.is_stalled = False
        
        # 参数
        self.STALL_CHECK_WINDOW   = 0.30
        self.STALL_VEL_THRESHOLD  = 0.02
        self.CMD_EFFORT_THRESHOLD = 0.002
        self.STALL_TRIGGER_TIME   = 1.0

    def update(self, rel_x, rel_y, cmd_effort):
        now = mono()
        self.history.append((now, rel_x, rel_y))

        past_record = None
        for record in self.history:
            if now - record[0] >= self.STALL_CHECK_WINDOW:
                past_record = record
                break

        if past_record is None:
            return False, 0.0

        dt = now - past_record[0]
        if dt < 1e-3: return False, 0.0

        dx = rel_x - past_record[1]
        dy = rel_y - past_record[2]
        real_vel = math.hypot(dx, dy) / dt

        is_stucking = (cmd_effort > self.CMD_EFFORT_THRESHOLD) and (real_vel < self.STALL_VEL_THRESHOLD)

        if is_stucking:
            if self.stall_start_time is None:
                self.stall_start_time = now
            elif now - self.stall_start_time > self.STALL_TRIGGER_TIME:
                self.is_stalled = True
        else:
            self.stall_start_time = None
            self.is_stalled = False

        return self.is_stalled, real_vel

class BodyFrameKF:
    """卡尔曼滤波器 (V2.6版 - 带拒绝突变逻辑)"""
    def __init__(self, name="target"):
        self.name = name
        self.x = None
        self.P = np.eye(2) * 1.0
        self.Q_base = np.eye(2) * 0.01
        self.last_update_time = mono()
        self.last_r = None
        self.reject_count = 0
        self.MAX_REJECT = 8

    def predict(self, now, ego_vx=0.0, ego_vy=0.0, ego_w=0.0):
        if self.x is None: return
        dt = now - self.last_update_time
        if dt < 1e-6: return
        self.last_update_time = now
        self.P += self.Q_base * (dt * 10.0)

    def update(self, distance, bearing_deg, ego_vx=0.0, ego_vy=0.0, ego_w=0.0, conf=1.0, truncated=False):
        now = mono()
        self.predict(now, ego_vx, ego_vy, ego_w)

        b_rad = math.radians(bearing_deg)
        if truncated and self.last_r is not None:
            meas_dist = self.last_r
        else:
            meas_dist = distance

        z = np.array([meas_dist * math.cos(b_rad), meas_dist * math.sin(b_rad)])

        if self.x is None:
            self.x = z
            self.P = np.eye(2) * 0.5
            self.last_r = distance
            return

        # 拒绝突变逻辑
        if self.last_r is not None and self.last_r < 0.5:
             dist_diff = np.linalg.norm(z - self.x)
             if dist_diff > 0.4 and self.reject_count < self.MAX_REJECT and not truncated:
                 # print(f"🛡️ [{self.name}] 拒绝突变: {dist_diff:.2f}m")
                 self.reject_count += 1
                 return
        self.reject_count = 0

        r_sigma = 0.05
        if truncated: r_sigma = 1.0 
        if conf < 0.8: r_sigma *= 2.0
        R = np.eye(2) * (r_sigma ** 2)

        y = z - self.x
        S = self.P + R
        try:
            K = self.P @ np.linalg.inv(S)
        except np.linalg.LinAlgError:
            return

        self.x = self.x + K @ y
        self.P = (np.eye(2) - K) @ self.P

        if not truncated:
            self.last_r = math.hypot(self.x[0], self.x[1])

    def get_state(self):
        if self.x is None: return None
        return float(self.x[0]), float(self.x[1])

class FormationPilot:
    """
    智能飞行员 (移植自 test_formation_lock_vision_ekf.py V2.6)
    包含: Tracking -> Sliding -> Finished 状态机
    """
    def __init__(self, driver):
        self.driver = driver
        self.stall_detector = StallDetector()

        self.kf_ball = BodyFrameKF("Ball")
        self.kf_leader = BodyFrameKF("Leader")

        # 状态记录
        self.last_cmd_vx = 0.0
        self.last_cmd_vy = 0.0
        self.last_cmd_w  = 0.0
        self.latest_side_intent = None
        self.mode_tag = "INIT"
        self.last_debug_err = (0.0, 0.0)

        # === 任务阶段控制 ===
        # PHASES: "TRACKING" -> "SLIDING" -> "FINISHED"
        self.mission_phase = "TRACKING"
        self.slide_start_time = None
        self.is_done_state = False # 用于Tracking内部判断是否进入死区

        # === PID & 逻辑参数 ===
        self.FORMATION_ANGLE_DIFF = 120.0
        self.TARGET_DIST = 0.25
        self.BLIND_APPROACH_LIMIT = 0.8
        self.SIDE_HYSTERESIS_BIAS = 0.25

        self.DIST_DEADBAND_ENTER = 0.015
        self.DIST_SOFT_ZONE = 0.08
        self.KP_DIST_FAST = 0.20
        self.KP_DIST_SLOW = 0.10
        self.KP_THETA     = 0.80
        self.FRICTION_FEEDFORWARD = 0.02

        # 平移任务参数
        self.SLIDE_VEL_ABS = 0.1   # m/s
        self.SLIDE_DIST    = 0.2   # m
        self.SLIDE_DURATION = self.SLIDE_DIST / self.SLIDE_VEL_ABS # 2.0s
        self.slide_vel_y_cmd = -0.1

    def update(self, batch):
        """
        每帧调用。
        返回状态字符串: "ADJUSTING", "SLIDING", "LOCKED_LEFT", "LOCKED_RIGHT"
        """
        # 1. 始终运行 EKF
        raw_ball = self._select_best(batch, CLS_BALL)
        raw_leader = self._select_best(batch, CLS_RED, "SOLID")

        now = mono()
        
        if raw_ball:
            self.kf_ball.update(raw_ball['distance'], raw_ball['bearing_body'],
                self.last_cmd_vx, self.last_cmd_vy, self.last_cmd_w,
                raw_ball.get('conf', 1.0), raw_ball.get('truncated', False))
        else:
            self.kf_ball.predict(now, self.last_cmd_vx, self.last_cmd_vy, self.last_cmd_w)

        if raw_leader:
            self.kf_leader.update(raw_leader['distance'], raw_leader['bearing_body'],
                self.last_cmd_vx, self.last_cmd_vy, self.last_cmd_w,
                raw_leader.get('conf', 1.0), raw_leader.get('truncated', False))
        else:
            self.kf_leader.predict(now, self.last_cmd_vx, self.last_cmd_vy, self.last_cmd_w)

        state_ball = self.kf_ball.get_state()
        state_leader = self.kf_leader.get_state()

        vx, vy = 0.0, 0.0
        current_status = "ADJUSTING"

        # 2. 状态机逻辑
        if self.mission_phase == "TRACKING":
            # === 阶段1：视觉锁定 ===
            is_tracking_finished = False
            valid_control = False
            control_ball = None
            is_virtual_ball = False

            if state_leader:
                dist_leader = math.hypot(state_leader[0], state_leader[1])
                if dist_leader > self.BLIND_APPROACH_LIMIT:
                    if state_ball:
                        control_ball = state_ball
                        is_virtual_ball = False
                        self.mode_tag = "FAR_BALL"
                    else:
                        control_ball = state_leader
                        is_virtual_ball = True # 盲追
                        self.mode_tag = "FAR_VIRT"
                    valid_control = True
                else:
                    if state_ball:
                        control_ball = state_ball
                        is_virtual_ball = False
                        self.mode_tag = "NEAR_BALL"
                        valid_control = True
                    else:
                        self.mode_tag = "NEAR_LOST"
                        valid_control = False # 有Leader没球且近，不盲动
            else:
                self.mode_tag = "NO_LEADER"

            # 保护: Leader 数据太旧
            if (mono() - self.kf_leader.last_update_time > 1.0):
                valid_control = False

            if valid_control and control_ball and state_leader:
                # 计算控制量
                vx, vy, is_tracking_finished = self._control_loop(control_ball, state_leader, is_virtual_ball)
                
                # 穿模保护
                curr_dist = math.hypot(control_ball[0], control_ball[1])
                if curr_dist < 0.18:
                    theta_robot = math.atan2(-control_ball[1], -control_ball[0])
                    vx = 0.15 * math.cos(theta_robot)
                    vy = 0.15 * math.sin(theta_robot)
                    is_tracking_finished = False
                    # print(f"⚠️ 穿模保护: {curr_dist:.2f}m")
            else:
                vx, vy = 0.0, 0.0
            
            # [状态转换] Tracking -> Sliding
            if is_tracking_finished:
                print(f"✅ 视觉锁定完成。切换至开环平移模式 (Sliding)... Side: {self.latest_side_intent}")
                self.mission_phase = "SLIDING"
                self.slide_start_time = mono()
                vx, vy = 0.0, 0.0
            
            current_status = "ADJUSTING"

        elif self.mission_phase == "SLIDING":
            # === 阶段2：开环平移 ===
            vx = 0.0
            vy = self.slide_vel_y_cmd

            elapsed = mono() - self.slide_start_time
            if elapsed >= self.SLIDE_DURATION:
                print("✅ 平移完成。")
                self.mission_phase = "FINISHED"
                vx, vy = 0.0, 0.0
            else:
                self.mode_tag = "SLIDING"

            current_status = "SLIDING"

        elif self.mission_phase == "FINISHED":
            # === 阶段3：结束/保持 ===
            vx, vy = 0.0, 0.0
            self.driver.stop()
            # 根据最后一次意图返回状态
            if self.latest_side_intent == "+":
                current_status = "LOCKED_LEFT"
            else:
                current_status = "LOCKED_RIGHT"

        # 3. 发送指令
        if self.driver:
            self.driver.send_velocity_command(vx, vy, 0.0)
        
        self.last_cmd_vx = vx
        self.last_cmd_vy = vy
        
        return current_status

    def _control_loop(self, p_ball, p_leader, is_virtual):
        """
        核心控制算法 (来自 combined_transport_node_v5_hybrid.py V2.6)
        返回值: (vx, vy, is_finished)
        """
        xt, yt = p_ball
        xl, yl = p_leader

        theta_robot = math.atan2(-yt, -xt)
        curr_dist = math.hypot(xt, yt)
        dist_err = curr_dist - self.TARGET_DIST

        # 调试信息更新
        angle_debug = 0.0

        if not self.is_done_state:
            if abs(dist_err) < self.DIST_DEADBAND_ENTER:
                self.is_done_state = True

        angle_err_deg = 0.0

        if is_virtual:
            v_tan = 0.0
        else:
            theta_leader = math.atan2(yl - yt, xl - xt)
            diff_rad = math.radians(self.FORMATION_ANGLE_DIFF)
            target_pos = theta_leader + diff_rad
            target_neg = theta_leader - diff_rad

            err_pos = wrap_rad_pi(target_pos - theta_robot)
            err_neg = wrap_rad_pi(target_neg - theta_robot)

            # 迟滞代价计算
            cost_pos = abs(err_pos)
            cost_neg = abs(err_neg)
            if self.latest_side_intent == "+": cost_pos -= self.SIDE_HYSTERESIS_BIAS
            elif self.latest_side_intent == "-": cost_neg -= self.SIDE_HYSTERESIS_BIAS

            if cost_pos < cost_neg:
                final_err_rad = err_pos
                self.latest_side_intent = "+"
            else:
                final_err_rad = err_neg
                self.latest_side_intent = "-"

            angle_err_deg = math.degrees(final_err_rad)
            angle_debug = angle_err_deg

            if self.is_done_state and abs(angle_err_deg) < 5.0:
                 v_tan = 0.0
            else:
                 v_tan = max(-0.25, min(0.25, self.KP_THETA * final_err_rad * curr_dist))

        v_rad = 0.0
        if self.is_done_state:
            v_rad = 0.0
        else:
            if abs(dist_err) < self.DIST_SOFT_ZONE:
                v_rad = -self.KP_DIST_SLOW * dist_err
                v_rad += math.copysign(self.FRICTION_FEEDFORWARD, v_rad)
            else:
                v_rad = -self.KP_DIST_FAST * dist_err

        check_effort = abs(v_rad) if v_rad < 0 else 0.0
        is_stalled, real_vel = self.stall_detector.update(xt, yt, check_effort)

        if is_stalled and v_rad < 0:
            v_rad = 0.0
            self.mode_tag = "STALL"

        v_rad = max(-0.15, min(0.15, v_rad))

        th = theta_robot
        vx = v_rad * math.cos(th) - v_tan * math.sin(th)
        vy = v_rad * math.sin(th) + v_tan * math.cos(th)

        # 更新调试数据
        self.last_debug_err = (dist_err, angle_debug)

        # 判断 Tracking 阶段是否结束
        is_tracking_finished = False
        if self.is_done_state and vx == 0.0 and vy == 0.0:
            is_tracking_finished = True

        return vx, vy, is_tracking_finished

    def _select_best(self, batch, cls_id, pattern=None):
        cands = [m for m in batch if m.get('class_id') == cls_id]
        if pattern: cands = [m for m in cands if m.get('pattern') == pattern]
        if not cands: return None
        cands.sort(key=lambda m: (m.get('truncated', False), -m.get('conf', 0), -m.get('area', 0)))
        return cands[0]

class EventWatcher:
    """光通信监测器"""
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
    """集群任务控制器"""
    def __init__(self, robot_id):
        self.ROBOT_ID = robot_id
        print(f"🔧 初始化控制器 | Robot ID: {self.ROBOT_ID}")

        self.BROKER_IP = "10.0.2.66"
        self.DIST_FIND_BALL = 0.8
        self.DIST_STOP_BALL = 0.32

        self.state = "INIT"
        self.my_role = "SEARCHER"
        self.start_time = mono()
        self.last_red_time = 0.0
        self.red_detect_count = 0
        self.FORCE_SEARCH_TIME = 3.0
        self.t_armed_start = 0.0
        self.t_run_start = 0.0

        try:
            self.driver = RobotDriver(self.ROBOT_ID)
            self.light = LightDriver(self.ROBOT_ID, broker_ip=self.BROKER_IP)
        except Exception: pass

        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.SUB)
        self.socket.connect("tcp://localhost:5555")
        self.socket.setsockopt_string(zmq.SUBSCRIBE, "perception")
        self.socket.setsockopt(zmq.RCVTIMEO, 10)

        # 这里使用了新的 Pilot
        self.pilot = FormationPilot(self.driver if hasattr(self, 'driver') else None)
        self.watcher = EventWatcher()
        self.kf_ball = BodyFrameKF("LeaderBall") # 仅用于Searcher判断
        self.update_state("SEARCH")

    def update_state(self, new_state):
        if self.state == new_state: return
        print(f"🔄 [State] {self.my_role} | {self.state} -> {new_state}")
        self.state = new_state
        if hasattr(self, 'light'):
            if new_state == "SEARCH":          self.light.set_cmd("SEARCH")
            elif new_state == "APPROACH_BALL": self.light.set_cmd("APPROACH_BALL")
            elif new_state == "LEADER_WAIT":   self.light.set_cmd("LEADER_WAIT")
            elif new_state == "PREWARM":       self.light.set_cmd("LEADER_GO")
            elif new_state == "ARMED":         self.light.set_cmd("FOLLOWER_PUSH")
            elif new_state == "DONE":          self.light.set_cmd("OFF"); self.driver.stop()

    def run(self):
        print(f"🚀 任务启动 | ID: {self.ROBOT_ID}")
        last_debug_log_t = mono()
        while True:
            try:
                batch = []
                latest_data = None
                while True:
                    try:
                        msg = self.socket.recv_string(flags=zmq.NOBLOCK)
                        _, json_str = msg.split(' ', 1)
                        data = json.loads(json_str)
                        if 'objects' in data: latest_data = data['objects']
                        else: latest_data = [data]
                    except zmq.Again: break
                    except Exception: pass

                if latest_data is not None: batch = latest_data
                self.watcher.ingest(batch)

                # [DEBUG] 日志
                if mono() - last_debug_log_t > 1.0:
                    seen_ids = sorted(list(set([m.get('class_id') for m in batch])))
                    debug_msg = f"[DEBUG] State: {self.state:10s} | Role: {self.my_role:8s} | ID: {seen_ids}"
                    if self.state == "BIDDING":
                        p_ball = self.pilot.kf_ball.get_state()
                        err_dist, err_ang = self.pilot.last_debug_err
                        ball_str = f"({p_ball[0]:.2f}, {p_ball[1]:.2f})" if p_ball else "❌"
                        debug_msg += f" | {self.pilot.mode_tag} | 🏀{ball_str} | Err:{err_dist:.2f}m"
                    print(debug_msg)
                    last_debug_log_t = mono()

                # 更新全局KF (Searcher用)
                raw_ball = next((m for m in batch if m['class_id'] == CLS_BALL), None)
                if raw_ball: self.kf_ball.update(raw_ball['distance'], raw_ball.get('bearing_body', 0.0))
                
                kf_state = self.kf_ball.get_state()
                has_ball = False
                ball_dist = 999.0
                ball_bearing = 0.0

                if kf_state:
                    ball_dist = math.hypot(kf_state[0], kf_state[1])
                    ball_bearing = math.degrees(math.atan2(kf_state[1], kf_state[0]))
                    has_ball = True
                elif raw_ball:
                    ball_dist = raw_ball.get('distance', 999.0)
                    ball_bearing = normalize_angle(raw_ball.get('bearing_body', 0.0))
                    has_ball = True

                has_red = any(m['class_id'] == CLS_RED for m in batch)
                now = mono()

                if self.my_role == "SEARCHER":
                    if has_ball and ball_dist < self.DIST_FIND_BALL:
                        self.my_role = "LEADER"
                        self.move_to_ball_simple(ball_dist, ball_bearing)
                    elif has_red and (now - self.start_time > self.FORCE_SEARCH_TIME):
                        if now - self.last_red_time < 0.3: self.red_detect_count += 1
                        else: self.red_detect_count = 1
                        self.last_red_time = now
                        if self.red_detect_count >= 40:
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
                            print("🎉 组队完成 -> PREWARM")
                            self.update_state("PREWARM")
                            self.t_armed_start = now + 2.0
                        else: self.driver.stop()
                    elif self.state == "PREWARM":
                        if now > self.t_armed_start:
                            self.update_state("ARMED")
                            self.t_run_start = now + START_DELAY
                    elif self.state == "ARMED":
                        if now > self.t_run_start: self.update_state("RUN")
                    elif self.state == "RUN":
                        self.transport_move()

                elif self.my_role == "FOLLOWER":
                    if self.state == "BIDDING":
                        # 调用新版 Pilot，内部处理 Tracking -> Sliding
                        status = self.pilot.update(batch)
                        
                        # 根据意图设置灯光
                        if self.pilot.latest_side_intent == "+": self.light.set_cmd("BID_LEFT")
                        elif self.pilot.latest_side_intent == "-": self.light.set_cmd("BID_RIGHT")
                        
                        # 只有当 Pilot 彻底完成 (Finished) 并返回 LOCKED_XXX 时才跳转
                        if status.startswith("LOCKED"):
                            self.update_state("READY_WAIT")
                            if "LEFT" in status: self.light.set_cmd("LOCK_LEFT")
                            else: self.light.set_cmd("LOCK_RIGHT")
                    
                    elif self.state == "READY_WAIT":
                        self.driver.stop()
                        if self.watcher.stable_pattern(CLS_PURPLE, "SOLID", 3, 0.5):
                            print("🚀 GO信号确认")
                            self.update_state("ARMED")
                            self.t_run_start = now + START_DELAY
                    elif self.state == "ARMED":
                        if now > self.t_run_start: self.update_state("RUN")
                    elif self.state == "RUN":
                        self.transport_move()

                if self.state == "RUN" and (now - self.t_run_start > TRANSPORT_DURATION):
                    self.update_state("DONE")
                
                time.sleep(0.03)

            except KeyboardInterrupt: break
            except Exception as e: print(f"Error: {e}")

        if hasattr(self, 'light'): self.light.stop()
        if hasattr(self, 'driver'): self.driver.stop()

    def omni_search_move(self):
        elapsed = mono() - self.start_time
        vx, vy = 0.0, 0.0
        if elapsed < 5.0:
            if self.ROBOT_ID == 15:   vy = -0.2
            elif self.ROBOT_ID == 13: vx = 0.2
            elif self.ROBOT_ID == 10: vx = -0.2
        else:
            vy = -0.2
            vx = 0.15 * math.sin(elapsed * math.pi)
        if hasattr(self, 'driver'): self.driver.send_velocity_command(vx, vy, 0.0)

    def move_to_ball_simple(self, dist, bearing):
        dist_error = dist - self.DIST_STOP_BALL
        if dist_error > 0:
            self.update_state("APPROACH_BALL")
            speed = max(0.0, min(0.2, dist_error * 0.6))
            rad = math.radians(bearing)
            if hasattr(self, 'driver'):
                self.driver.send_velocity_command(speed*math.cos(rad), speed*math.sin(rad), 0.0)
        else: self.update_state("LEADER_WAIT")

    def transport_move(self):
        if hasattr(self, 'driver'): self.driver.send_velocity_command(TRANSPORT_SPEED, 0.0, 0.0)

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("-i", "--id", type=int, required=True)
    args = parser.parse_args()
    c = SwarmMissionController(robot_id=args.id)
    c.run()