#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
combined_transport_node_v5_hybrid.py
[混合架构版]
外层：保留原始的多车交互状态机 (WAIT_GO, PREWARM, ARMED...)
内核：集成 V2.6 的 "一次性锁定 + 平移" 逻辑
"""

import argparse
import json
import math
import time
import collections
import zmq
import numpy as np
from collections import defaultdict, deque

# 确保这两个文件在同一目录下
from light_driver import LightDriver
from robot_driver import RobotDriver

# ==========================================
# 1. 核心参数配置 (集成 V2.6 参数)
# ==========================================
# 颜色类别ID
BLUE, RED, GREEN, PURPLE, GRAY = 1, 2, 3, 4, 5
BALL_CLASS_ID = 6
LEADER_CLASS_ID = 2

# 编队参数
FORMATION_ANGLE_DIFF = 120.0
TARGET_DIST = 0.25
BLIND_APPROACH_LIMIT = 0.8

# V2.6 PID 参数 (一次性逻辑)
DIST_DEADBAND_ENTER = 0.015  # 只进不出
DIST_SOFT_ZONE      = 0.08
KP_DIST_FAST = 0.20
KP_DIST_SLOW = 0.10
KP_THETA     = 0.80
FRICTION_FEEDFORWARD = 0.02

# 堵转检测
STALL_CHECK_WINDOW   = 0.30
STALL_VEL_THRESHOLD  = 0.02
CMD_EFFORT_THRESHOLD = 0.002
STALL_TRIGGER_TIME   = 1.0

# 迟滞参数 (选边)
SIDE_HYSTERESIS_BIAS = 0.25

# [V2.6 新增] 平移任务参数
SLIDE_VEL_Y = -0.1    # m/s, 正值向左，负值向右
SLIDE_DIST  = 0.2    # m
SLIDE_DURATION = SLIDE_DIST / abs(SLIDE_VEL_Y)

def mono() -> float:
    return time.monotonic()

def heading_to_vxy(speed: float, heading_deg: float):
    rad = math.radians(heading_deg)
    return speed * math.cos(rad), speed * math.sin(rad)

def wrap_rad_pi(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi

def log(msg: str):
    ts = time.strftime("%H:%M:%S")
    print(f"[{ts}] {msg}", flush=True)

# ==========================================
# 2. 辅助类 (保持 V2.6 版本)
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
            if now - record[0] >= STALL_CHECK_WINDOW:
                past_record = record
                break
        if past_record is None: return False
        dt = now - past_record[0]
        if dt < 1e-3: return False
        dx = rel_x - past_record[1]
        dy = rel_y - past_record[2]
        real_vel = math.hypot(dx, dy) / dt
        is_stucking = (cmd_effort > CMD_EFFORT_THRESHOLD) and (real_vel < STALL_VEL_THRESHOLD)
        if is_stucking:
            if self.stall_start_time is None: self.stall_start_time = now
            elif now - self.stall_start_time > STALL_TRIGGER_TIME: self.is_stalled = True
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
        self.reject_count = 0
        self.MAX_REJECT = 8

    def predict(self, now):
        if self.x is None: return
        dt = now - self.last_update_time
        if dt < 1e-6: return
        self.last_update_time = now
        self.P += self.Q_base * (dt * 10.0)

    def update(self, distance, bearing_deg, conf=1.0, truncated=False):
        now = mono()
        self.predict(now)
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
             if dist_diff > 0.4 and self.reject_count < self.MAX_REJECT and not truncated:
                 self.reject_count += 1
                 return
        self.reject_count = 0
        r_sigma = 0.05
        if truncated: r_sigma = 1.0
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
        if mono() - self.last_update_time > 1.0: return None
        return float(self.x[0]), float(self.x[1])

# ==========================================
# 3. 入位驾驶仪 (内核升级为 V2.6)
# ==========================================
class FormationPilot:
    """
    内部实现了 Tracking -> Sliding -> Finished 的状态流转。
    对外只暴露：ADJUSTING (正在忙) 或 LOCKED_LEFT/RIGHT (全搞定)。
    """
    def __init__(self, driver: RobotDriver):
        self.driver = driver
        self.kf_ball = BodyFrameKF("Ball")
        self.kf_leader = BodyFrameKF("Leader")
        self.stall_detector = StallDetector()

        # V2.6 内部状态
        self.is_done_state = False  # 距离死区锁
        self.latest_side_intent = None
        self.mode_tag = "INIT"
        self.last_debug_err = (0.0, 0.0)
        
        # 任务阶段: TRACKING -> SLIDING -> FINISHED
        self.mission_phase = "TRACKING"
        self.slide_start_time = None

    def _select_best(self, batch, cls_id, pattern=None):
        candidates = [m for m in batch if m.get('class_id') == cls_id]
        if pattern: candidates = [m for m in candidates if m.get('pattern') == pattern]
        if not candidates: return None
        candidates.sort(key=lambda m: (m.get('truncated', False), -m.get('conf', 0), -m.get('area', 0)))
        return candidates[0]

    def update(self, vision_batch) -> str:
        """
        Main Loop 调用的接口。
        返回: "ADJUSTING", "LOST", "LOCKED_LEFT", "LOCKED_RIGHT"
        """
        # 1. 视觉更新
        raw_ball = self._select_best(vision_batch, BALL_CLASS_ID)
        raw_leader = self._select_best(vision_batch, LEADER_CLASS_ID, "SOLID")

        if raw_ball:
            self.kf_ball.update(raw_ball['distance'], raw_ball['bearing_body'],
                                raw_ball.get('conf', 1.0), raw_ball.get('truncated', False))
        else:
            self.kf_ball.predict(mono())

        if raw_leader:
            self.kf_leader.update(raw_leader['distance'], raw_leader['bearing_body'],
                                  raw_leader.get('conf', 1.0), raw_leader.get('truncated', False))
        else:
            self.kf_leader.predict(mono())

        # 2. 内部多阶段处理
        vx, vy = 0.0, 0.0
        status_to_return = "ADJUSTING"

        if self.mission_phase == "TRACKING":
            # === 阶段1: 视觉闭环 ===
            state_ball = self.kf_ball.get_state()
            state_leader = self.kf_leader.get_state()

            # 目标判定
            valid_control = False
            control_ball = None
            is_virtual = False
            
            if state_leader:
                dist_leader = math.hypot(state_leader[0], state_leader[1])
                if dist_leader > BLIND_APPROACH_LIMIT:
                    if state_ball:
                        control_ball, is_virtual = state_ball, False
                        self.mode_tag = "FAR_BALL"
                    else:
                        control_ball, is_virtual = state_leader, True
                        self.mode_tag = "FAR_VIRT"
                    valid_control = True
                else:
                    if state_ball:
                        control_ball, is_virtual = state_ball, False
                        self.mode_tag = "NEAR_BALL"
                        valid_control = True
                    else:
                        self.mode_tag = "NEAR_LOST"
            else:
                self.mode_tag = "NO_LEADER"

            # 运行 Tracking 控制环
            is_tracking_finished = False
            if valid_control and control_ball and state_leader:
                vx, vy, is_tracking_finished = self._control_loop_tracking(control_ball, state_leader, is_virtual)
                
                # 穿模保护
                curr_dist = math.hypot(control_ball[0], control_ball[1])
                if curr_dist < 0.18:
                    th = math.atan2(-control_ball[1], -control_ball[0])
                    vx, vy = 0.15 * math.cos(th), 0.15 * math.sin(th)
                    is_tracking_finished = False # 保护触发时不结束
            else:
                vx, vy = 0.0, 0.0
                if not state_leader:
                    return "LOST"

            # 转换判定: Tracking -> Sliding
            if is_tracking_finished:
                log(f"✅ 锁定完成 (Side: {self.latest_side_intent})，开始平移...")
                self.mission_phase = "SLIDING"
                self.slide_start_time = mono()
                vx, vy = 0.0, 0.0 # 本帧暂停
            
            status_to_return = "ADJUSTING" # 只要在动，就告诉主循环还在忙

        elif self.mission_phase == "SLIDING":
            # === 阶段2: 开环平移 ===
            vx = 0.0
            vy = SLIDE_VEL_Y # 执行平移
            
            elapsed = mono() - self.slide_start_time
            if elapsed >= SLIDE_DURATION:
                log("✅ 平移完成。")
                self.mission_phase = "FINISHED"
                vx, vy = 0.0, 0.0
            else:
                self.mode_tag = "SLIDING"
            
            status_to_return = "ADJUSTING" # 平移时也告诉主循环还在忙

        elif self.mission_phase == "FINISHED":
            # === 阶段3: 任务结束 ===
            vx, vy = 0.0, 0.0
            self.driver.stop()
            # 只有这里，才真正告诉主循环：我好了，去下个状态吧
            if self.latest_side_intent == "+":
                return "LOCKED_LEFT"
            else:
                return "LOCKED_RIGHT"

        # 执行速度指令
        self.driver.send_velocity_command(vx, vy, 0.0)
        return status_to_return

    def _control_loop_tracking(self, p_ball, p_leader, is_virtual):
        """ V2.6 核心逻辑 """
        xt, yt = p_ball
        xl, yl = p_leader
        theta_robot = math.atan2(-yt, -xt)
        curr_dist = math.hypot(xt, yt)
        dist_err = curr_dist - TARGET_DIST
        
        # 调试信息更新
        angle_debug = 0.0
        
        # 1. 一次性死区
        if not self.is_done_state:
            if abs(dist_err) < DIST_DEADBAND_ENTER:
                self.is_done_state = True
        
        # 2. 切向控制 (智能选边)
        v_tan = 0.0
        angle_err_deg = 0.0
        
        if not is_virtual:
            theta_leader = math.atan2(yl - yt, xl - xt)
            diff_rad = math.radians(FORMATION_ANGLE_DIFF)
            target_pos = theta_leader + diff_rad 
            target_neg = theta_leader - diff_rad 

            err_pos = wrap_rad_pi(target_pos - theta_robot)
            err_neg = wrap_rad_pi(target_neg - theta_robot)

            # 迟滞代价计算
            cost_pos = abs(err_pos)
            cost_neg = abs(err_neg)
            if self.latest_side_intent == "+": cost_pos -= SIDE_HYSTERESIS_BIAS
            elif self.latest_side_intent == "-": cost_neg -= SIDE_HYSTERESIS_BIAS
            
            if cost_pos < cost_neg:
                final_err_rad = err_pos
                self.latest_side_intent = "+"
            else:
                final_err_rad = err_neg
                self.latest_side_intent = "-"
            
            angle_err_deg = math.degrees(final_err_rad)
            angle_debug = angle_err_deg

            # 角度PID
            if self.is_done_state and abs(angle_err_deg) < 5.0:
                 v_tan = 0.0
            else:
                 v_tan = max(-0.25, min(0.25, KP_THETA * final_err_rad * curr_dist))

        # 3. 径向控制
        v_rad = 0.0
        if self.is_done_state:
            v_rad = 0.0
        else:
            if abs(dist_err) < DIST_SOFT_ZONE:
                v_rad = -KP_DIST_SLOW * dist_err
                v_rad += math.copysign(FRICTION_FEEDFORWARD, v_rad)
            else:
                v_rad = -KP_DIST_FAST * dist_err

        # 堵转保护
        check_effort = abs(v_rad) if v_rad < 0 else 0.0
        if self.stall_detector.update(xt, yt, check_effort) and v_rad < 0:
            v_rad = 0.0
            self.mode_tag = "STALL"

        v_rad = max(-0.15, min(0.15, v_rad))

        # 4. 速度合成
        th = theta_robot
        vx = v_rad * math.cos(th) - v_tan * math.sin(th)
        vy = v_rad * math.sin(th) + v_tan * math.cos(th)

        # 更新调试数据
        self.last_debug_err = (dist_err, angle_debug)

        # 5. 结束判定
        is_finished = False
        if self.is_done_state and vx == 0.0 and vy == 0.0:
            is_finished = True

        return vx, vy, is_finished

# ==========================================
# 4. 事件监测器 (原版保留)
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
# 5. 主程序 (状态机逻辑未变)
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

    # 状态初始化
    state = "UNKNOWN"
    my_side = args.side

    if args.role == "leader":
        state = "WAIT_FORM"
        light.set_cmd("LEADER_WAIT")
        log("我是 Leader，等待队友就位 (红灯常亮)...")
    else:
        state = "DOCKING"
        light.set_cmd("OFF")
        log(f"我是 Follower，开始寻找 (Vision: {args.vision_endpoint})")

    t_state = mono()
    t_move_start = None
    vx, vy = heading_to_vxy(args.speed, args.heading_deg)
    armed = False
    t_armed_start = None
    last_debug_log_t = 0.0
    
    try:
        dt = 0.03
        while True:
            tick_start = mono()

            # 1. 接收视觉数据
            batch = []
            latest_payload = None
            while True:
                try:
                    s = sock.recv_string(flags=zmq.NOBLOCK)
                    _, payload_str = s.split(" ", 1)
                    parsed_json = json.loads(payload_str)
                    if 'objects' in parsed_json:
                        latest_payload = parsed_json['objects']
                    else:
                        latest_payload = [parsed_json]
                except zmq.Again:
                    break
                except Exception:
                    pass
            if latest_payload is not None:
                batch = latest_payload
            
            watcher.ingest(batch)

            # [DEBUG] 日志
            if mono() - last_debug_log_t > 1.0:
                seen_ids = sorted(list(set([m.get('class_id') for m in batch])))
                debug_msg = f"[DEBUG] State: {state:10s} | Phase: {pilot.mission_phase:8s} | ID: {seen_ids}"
                if state == "DOCKING":
                    p_ball = pilot.kf_ball.get_state()
                    err_dist, err_ang = pilot.last_debug_err
                    ball_str = f"({p_ball[0]:.2f}, {p_ball[1]:.2f})" if p_ball else "❌"
                    debug_msg += f" | {pilot.mode_tag} | 🏀{ball_str} | Err:{err_dist:.2f}m"
                print(debug_msg)
                last_debug_log_t = mono()

            # 2. 状态机逻辑
            if state == "DOCKING":
                # === 这里的逻辑发生了内核级变化 ===
                # pilot.update 内部包含了 Tracking -> Sliding 的过程
                # 只有全部完成后，才会返回 LOCKED_LEFT/RIGHT
                pilot_status = pilot.update(batch)

                if pilot_status == "ADJUSTING":
                    # 根据选边意图设置灯光 (在 Tracking 和 Sliding 期间都闪烁)
                    if pilot.latest_side_intent == "+":
                        light.set_cmd("BID_LEFT")
                    elif pilot.latest_side_intent == "-":
                        light.set_cmd("BID_RIGHT")
                    else:
                        light.set_cmd("SEARCH")

                elif pilot_status == "LOCKED_LEFT":
                    my_side = "left"
                    state = "WAIT_GO_SIGNAL"
                    light.set_cmd("LOCK_LEFT")
                    log(">>> [Tracking+Sliding] 全部完成: 左侧 (LOCK_LEFT) <<<")

                elif pilot_status == "LOCKED_RIGHT":
                    my_side = "right"
                    state = "WAIT_GO_SIGNAL"
                    light.set_cmd("LOCK_RIGHT")
                    log(">>> [Tracking+Sliding] 全部完成: 右侧 (LOCK_RIGHT) <<<")

                elif pilot_status == "LOST":
                    light.set_cmd("OFF")

            elif state == "WAIT_GO_SIGNAL":
                # 等待紫灯GO信号
                if watcher.stable_pattern(PURPLE, "SOLID", 3, 0.4):
                    state = "ARMED"
                    light.set_cmd("FOLLOWER_PUSH")
                    armed = True
                    t_armed_start = mono() + args.delay
                    log(f"收到 GO 信号，{args.delay}s 后启动")

            elif state == "WAIT_FORM":
                # Leader 等待队友
                left_ready = watcher.stable_pattern(GREEN, "SOLID", 3, 0.6)
                right_ready = watcher.stable_pattern(BLUE, "SOLID", 3, 0.6)
                if left_ready and right_ready:
                    state = "PREWARM"
                    t_state = mono()
                    light.set_cmd("LEADER_GO")
                    log("队友已就位，进入预热...")

            elif state == "PREWARM":
                light.set_cmd("LEADER_GO")
                if mono() - t_state > 2.0:
                    state = "WAIT_GO_LOCAL"
                    log("预热结束，准备出发")

            elif state == "WAIT_GO_LOCAL":
                if args.role == "leader":
                    state = "ARMED"
                    armed = True
                    t_armed_start = mono() + args.delay
                else:
                    if watcher.stable_pattern(PURPLE, "SOLID", 3, 0.4):
                        state = "ARMED"
                        armed = True
                        t_armed_start = mono() + args.delay

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