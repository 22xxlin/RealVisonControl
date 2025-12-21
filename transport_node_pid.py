#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
combined_transport_node_debug.py
融合版主节点（带调试输出版）：自主入位 -> 角色锁定 -> 协同搬运

主要修改：
在主循环中加入了 [DEBUG] 打印块，每秒显示一次视觉和定位状态，
用于排查为什么 Follower "不动" 的问题。
"""

import argparse
import json
import math
import time
import collections
import zmq
import numpy as np
from collections import defaultdict, deque

# 引入硬件驱动
from light_driver import LightDriver
from robot_driver import RobotDriver

# ==========================================
# 1. 核心参数与常量
# ==========================================
# 颜色类别ID
BLUE, RED, GREEN, PURPLE, GRAY = 1, 2, 3, 4, 5
BALL_CLASS_ID = 6
LEADER_CLASS_ID = 2 # 默认 Leader 是红色(2)

# 入位控制参数
FORMATION_ANGLE_DIFF = 120.0
TARGET_DIST = 0.25
DIST_DEADBAND_ENTER = 0.02
DIST_DEADBAND_EXIT  = 0.05
KP_DIST_FAST = 0.35
KP_DIST_SLOW = 0.20
KP_THETA     = 0.80

LOCK_TIME_THRESHOLD = 2.0  # 稳定保持多久才算锁定

def mono() -> float:
    return time.monotonic()

def wrap_rad_pi(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi

def heading_to_vxy(speed: float, heading_deg: float):
    rad = math.radians(heading_deg)
    return speed * math.cos(rad), speed * math.sin(rad)

def log(msg: str):
    ts = time.strftime("%H:%M:%S")
    print(f"[{ts}] {msg}", flush=True)

# ==========================================
# 2. 辅助类
# ==========================================
class StallDetector:
    def __init__(self):
        self.history = collections.deque(maxlen=50)
        self.stall_start_time = None
        self.is_stalled = False

    def update(self, rel_x, rel_y, cmd_effort):
        now = mono()
        self.history.append((now, rel_x, rel_y))
        past_record = None
        for record in self.history:
            if now - record[0] >= 0.3:
                past_record = record
                break
        if past_record is None: return False
        
        dt = now - past_record[0]
        dx = rel_x - past_record[1]
        dy = rel_y - past_record[2]
        real_vel = math.hypot(dx, dy) / dt

        is_stucking = (cmd_effort > 0.1) and (real_vel < 0.02)
        if is_stucking:
            if self.stall_start_time is None: self.stall_start_time = now
            elif now - self.stall_start_time > 1.0: self.is_stalled = True
        else:
            self.stall_start_time = None
            self.is_stalled = False
        return self.is_stalled

class BodyFrameKF:
    def __init__(self, name="target"):
        self.name = name
        self.x = None 
        self.P = np.eye(2) * 1.0
        self.Q_base = np.eye(2) * 0.01 
        self.last_update_time = mono()
        self.last_r = None 

    def update(self, distance, bearing_deg, conf=1.0, truncated=False):
        now = mono()
        dt = max(0.0, now - self.last_update_time)
        self.last_update_time = now
        self.P += self.Q_base * (dt * 10.0)

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
             dist_diff = np.linalg.norm(z - self.x)
             if dist_diff > 0.4: return 

        r_sigma = 0.05 
        if truncated: r_sigma *= 10.0
        if conf < 0.8: r_sigma *= 2.0
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
        # 如果超过 0.8s 没更新，认为丢失
        if mono() - self.last_update_time > 0.8: return None
        return float(self.x[0]), float(self.x[1])

# ==========================================
# 3. 入位驾驶仪
# ==========================================
class FormationPilot:
    def __init__(self, driver: RobotDriver):
        self.driver = driver
        self.kf_ball = BodyFrameKF("Ball")
        self.kf_leader = BodyFrameKF("Leader")
        self.stall_detector = StallDetector()
        
        self.stable_since = None
        self.current_side_candidate = None 
        self.is_in_deadband = False
        
        # 调试用变量
        self.last_debug_err = (0.0, 0.0) # dist_err, angle_err

    def update(self, vision_batch) -> str:
        # 1. 提取
        raw_ball = self._select_best(vision_batch, BALL_CLASS_ID)
        raw_leader = self._select_best(vision_batch, LEADER_CLASS_ID, "SOLID")

        # 2. EKF
        if raw_ball:
            self.kf_ball.update(raw_ball['distance'], raw_ball['bearing_body'], 
                                raw_ball.get('conf', 1.0), raw_ball.get('truncated', False))
        if raw_leader:
            self.kf_leader.update(raw_leader['distance'], raw_leader['bearing_body'],
                                  raw_leader.get('conf', 1.0), raw_leader.get('truncated', False))

        p_ball = self.kf_ball.get_state()
        p_leader = self.kf_leader.get_state()

        if not p_ball or not p_leader:
            self.driver.stop()
            self.stable_since = None
            return "LOST"

        # 3. 计算
        xt, yt = p_ball
        xl, yl = p_leader
        theta_robot = math.atan2(-yt, -xt)
        curr_dist = math.hypot(xt, yt)
        dist_err = curr_dist - TARGET_DIST
        
        theta_leader = math.atan2(yl - yt, xl - xt)
        deadband = DIST_DEADBAND_EXIT if self.is_in_deadband else DIST_DEADBAND_ENTER
        self.is_in_deadband = abs(dist_err) < deadband

        # 自动择优
        diff_rad = math.radians(FORMATION_ANGLE_DIFF)
        err_pos = wrap_rad_pi((theta_leader + diff_rad) - theta_robot)
        err_neg = wrap_rad_pi((theta_leader - diff_rad) - theta_robot)

        if abs(err_pos) < abs(err_neg):
            final_err_rad = err_pos
            side_char = "+" 
        else:
            final_err_rad = err_neg
            side_char = "-" 

        angle_err_deg = math.degrees(final_err_rad)
        self.last_debug_err = (dist_err, angle_err_deg) # 记录给 debug 用

        # 4. PID
        v_tan = 0.0
        if self.is_in_deadband and abs(angle_err_deg) < 5.0:
            v_tan = 0.0
        else:
            v_tan = KP_THETA * final_err_rad * curr_dist
            v_tan = max(-0.3, min(0.3, v_tan))

        v_rad = 0.0
        if self.is_in_deadband:
            v_rad = 0.0
        else:
            if abs(dist_err) < 0.08: v_rad = -KP_DIST_SLOW * dist_err
            else: v_rad = -KP_DIST_FAST * dist_err

        check_effort = abs(v_rad) if v_rad < 0 else 0.0
        if self.stall_detector.update(xt, yt, check_effort) and v_rad < 0:
            v_rad = 0.0

        v_rad = max(-0.25, min(0.25, v_rad))

        th = theta_robot
        vx = v_rad * math.cos(th) - v_tan * math.sin(th)
        vy = v_rad * math.sin(th) + v_tan * math.cos(th)

        # 穿模保护
        if curr_dist < 0.15:
            vx = 0.1 * math.cos(theta_robot)
            vy = 0.1 * math.sin(theta_robot)

        self.driver.send_velocity_command(vx, vy, 0.0)

        # 5. 判定
        is_pos_stable = (abs(dist_err) < DIST_DEADBAND_ENTER) and (abs(angle_err_deg) < 5.0)
        
        if is_pos_stable:
            if self.stable_since is None:
                self.stable_since = mono()
                self.current_side_candidate = side_char
            elif mono() - self.stable_since > LOCK_TIME_THRESHOLD:
                self.driver.stop()
                return "LOCKED_LEFT" if self.current_side_candidate == "+" else "LOCKED_RIGHT"
        else:
            self.stable_since = None
        
        return "ADJUSTING"

    def _select_best(self, batch, cls_id, pattern=None):
        candidates = [m for m in batch if m.get('class_id') == cls_id]
        if pattern: candidates = [m for m in candidates if m.get('pattern') == pattern]
        if not candidates: return None
        candidates.sort(key=lambda m: (m.get('truncated', False), -m.get('conf', 0), -m.get('area', 0)))
        return candidates[0]

# ==========================================
# 4. 事件监测器
# ==========================================
class EventWatcher:
    def __init__(self):
        self.hist = defaultdict(lambda: deque(maxlen=40))

    def ingest(self, batch):
        t = mono()
        for msg in batch:
            cid = int(msg.get("class_id", -1))
            pat = str(msg.get("pattern", "OFF"))
            self.hist[cid].append((t, pat))
        
    def stable_pattern(self, class_id: int, pattern: str, need_k: int, within_s: float) -> bool:
        h = self.hist[class_id]
        if not h: return False
        t0 = mono() - within_s
        hits = sum(1 for (t, p) in h if t >= t0 and p == pattern)
        return hits >= need_k

# ==========================================
# 5. 主程序
# ==========================================
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--role", choices=["leader", "follower"], required=True)
    ap.add_argument("--robot-id", type=int, required=True)
    ap.add_argument("--side", choices=["left", "right"], default=None)
    
    ap.add_argument("--vision-endpoint", type=str, default="tcp://127.0.0.1:5555")
    ap.add_argument("--delay", type=float, default=1.2)
    ap.add_argument("--speed", type=float, default=0.15)
    ap.add_argument("--heading-deg", type=float, default=0.0)
    ap.add_argument("--move-sec", type=float, default=5.0)
    
    args = ap.parse_args()

    light = LightDriver(args.robot_id)
    base = RobotDriver(robot_id=args.robot_id, ros_topic="/robot/velcmd")
    
    ctx = zmq.Context.instance()
    sock = ctx.socket(zmq.SUB)
    sock.connect(args.vision_endpoint)
    sock.setsockopt_string(zmq.SUBSCRIBE, "perception")
    
    pilot = FormationPilot(base)
    watcher = EventWatcher()
    
    state = "UNKNOWN"
    my_side = args.side

    if args.role == "leader":
        state = "WAIT_FORM"
        light.set_cmd("LEADER_WAIT")
        log("我是 Leader，等待队友就位...")
    else:
        state = "DOCKING"
        light.set_cmd("OFF") 
        log(f"我是 Follower，开始寻找最近队形点... (Vision: {args.vision_endpoint})")

    t_state = mono()
    t_move_start = None
    vx, vy = heading_to_vxy(args.speed, args.heading_deg)
    
    armed = False
    t_armed_start = None

    # [DEBUG] 调试用计时器
    last_debug_log_t = 0.0
    
    try:
        dt = 0.03
        while True:
            tick_start = mono()
            
            # 1. 接收数据
            batch = []
            while True:
                try:
                    s = sock.recv_string(flags=zmq.NOBLOCK)
                    _, payload = s.split(" ", 1)
                    batch.append(json.loads(payload))
                except zmq.Again:
                    break
                except Exception:
                    pass
            
            watcher.ingest(batch)

            # ====================================================
            # [DEBUG] 诊断模块：每 1.0 秒打印一次当前状态和视觉
            # ====================================================
            if mono() - last_debug_log_t > 1.0:
                # 统计当前看到的 ID
                seen_ids = sorted(list(set([m.get('class_id') for m in batch])))
                
                # 构建基础信息字符串
                debug_msg = f"[DEBUG] State: {state:10s} | 消息数: {len(batch)} | 👀 ID: {seen_ids}"
                
                # 如果是 DOCKING 状态，追加 EKF 信息
                if state == "DOCKING":
                    p_ball = pilot.kf_ball.get_state()
                    p_leader = pilot.kf_leader.get_state()
                    
                    ball_str = f"({p_ball[0]:.2f}, {p_ball[1]:.2f})" if p_ball else "❌丢失"
                    leader_str = f"({p_leader[0]:.2f}, {p_leader[1]:.2f})" if p_leader else "❌丢失"
                    
                    err_dist, err_ang = pilot.last_debug_err
                    debug_msg += f" | 🏀:{ball_str} 🚩:{leader_str} | Err: {err_dist:.2f}m, {err_ang:.1f}°"
                
                print(debug_msg)
                last_debug_log_t = mono()
            # ====================================================

            # 2. 状态机
            if state == "DOCKING":
                pilot_status = pilot.update(batch)
                if pilot_status == "LOCKED_LEFT":
                    my_side = "left"
                    state = "WAIT_GO_SIGNAL"
                    light.set_cmd("LOCK_LEFT")
                    log(">>> 入位完成: 左侧 (Green) <<<")
                elif pilot_status == "LOCKED_RIGHT":
                    my_side = "right"
                    state = "WAIT_GO_SIGNAL"
                    light.set_cmd("LOCK_RIGHT")
                    log(">>> 入位完成: 右侧 (Blue) <<<")

            elif state == "WAIT_GO_SIGNAL":
                # 等待紫色常亮 (Leader GO)
                if watcher.stable_pattern(PURPLE, "SOLID", 3, 0.4):
                    state = "ARMED"
                    light.set_cmd("FOLLOWER_PUSH")
                    armed = True
                    t_armed_start = mono() + args.delay
                    log(f"收到 GO 信号，{args.delay}s 后启动")

            elif state == "WAIT_FORM":
                left_ready = watcher.stable_pattern(GREEN, "SOLID", 3, 0.6)
                right_ready = watcher.stable_pattern(BLUE, "SOLID", 3, 0.6)
                if left_ready and right_ready:
                    state = "PREWARM"
                    t_state = mono()
                    light.set_cmd("SEARCH")
                    log("队友已就位，进入预热 (Prewarm)...")

            elif state == "PREWARM":
                if mono() - t_state > 2.0:
                    state = "WAIT_GO_LOCAL"
                    light.set_cmd("LEADER_GO")
                    log("发送 GO 信号！")

            elif state == "WAIT_GO_LOCAL":
                if watcher.stable_pattern(PURPLE, "SOLID", 3, 0.4):
                    state = "ARMED"
                    armed = True
                    t_armed_start = mono() + args.delay
                    log(f"GO 确认，{args.delay}s 后启动")

            elif state == "ARMED":
                if mono() >= t_armed_start:
                    state = "RUN"
                    t_move_start = mono()
                    log(">>> 开始搬运 (RUN) <<<")

            elif state == "RUN":
                base.send_velocity_command(vx, vy, 0.0)
                if mono() - t_move_start >= args.move_sec:
                    state = "DONE"
                    base.stop()
                    log("搬运完成 (DONE)")

            elif state == "DONE":
                base.stop()

            elapsed = mono() - tick_start
            if elapsed < dt:
                time.sleep(dt - elapsed)

    except KeyboardInterrupt:
        log("停止运行")
    finally:
        base.stop()
        light.set_cmd("OFF")
        light.stop()

if __name__ == "__main__":
    main()