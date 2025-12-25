#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
test_formation_lock_vision_ekf.py

融合版 V2.4 (Batch适配 + 平移脱困):
- [新增] 平移脱困逻辑: 检测到堵转时，暂停PID，强制执行 "后退+横移" 1.5秒。
- [保留] VisionSubscriber: 适配 vision_pub.py V3.2 Batch 协议。
- [保留] 自动择优 + 迟滞锁定。
"""

import rospy
import math
import time
import collections
import zmq
import json
import numpy as np
from robot_driver import RobotDriver

# =========================
# 1. 核心配置
# =========================
ROBOT_ID = 15

# 视觉 ZMQ
ZMQ_IP   = "127.0.0.1"
ZMQ_PORT = 5555
ZMQ_TOPIC = "perception"

# 目标定义
BALL_CLASS_ID = 6
LEADER_CLASS_ID = 2
LEADER_PATTERN = "SOLID"

# 队形参数
FORMATION_ANGLE_DIFF = 120.0  # 绝对值，程序会自动选择 +120 或 -120
TARGET_DIST = 0.25
BLIND_APPROACH_LIMIT = 0.8 

# PID 参数
DIST_DEADBAND_ENTER = 0.015  # 进入静止
DIST_DEADBAND_EXIT  = 0.035  # 离开静止

DIST_SOFT_ZONE = 0.08
KP_DIST_FAST = 0.35
KP_DIST_SLOW = 0.20
KP_THETA     = 0.80

# 堵转检测
STALL_CHECK_WINDOW   = 0.30
STALL_VEL_THRESHOLD  = 0.02
CMD_EFFORT_THRESHOLD = 0.002
STALL_TRIGGER_TIME   = 1.0
FRICTION_FEEDFORWARD = 0.02

# 脱困参数
ESCAPE_DISTANCE = 0.30      # 脱困平移距离 (米)
ESCAPE_SPEED_Y  = 0.20      # 脱困横移速度 (m/s)
ESCAPE_SPEED_X  = -0.05     # 脱困后退速度 (m/s, 稍微后退以释放压力)
# 计算脱困所需时间: t = d / v
ESCAPE_DURATION = ESCAPE_DISTANCE / ESCAPE_SPEED_Y

# =========================
# 2. 辅助工具
# =========================
def wrap_rad_pi(a):
    """将弧度归一化到 [-pi, pi]"""
    return (a + math.pi) % (2.0 * math.pi) - math.pi

def wrap_deg_180(a):
    """将角度归一化到 [-180, 180]"""
    return (a + 180.0) % 360.0 - 180.0

# =========================
# 3. 堵转检测
# =========================
class StallDetector:
    def __init__(self):
        self.history = collections.deque(maxlen=50)
        self.stall_start_time = None
        self.is_stalled = False

    def update(self, rel_x, rel_y, cmd_effort):
        now = time.time()
        self.history.append((now, rel_x, rel_y))

        past_record = None
        for record in self.history:
            if now - record[0] >= STALL_CHECK_WINDOW:
                past_record = record
                break
        
        if past_record is None:
            return False, 0.0

        dt = now - past_record[0]
        if dt < 1e-3: return False, 0.0

        dx = rel_x - past_record[1]
        dy = rel_y - past_record[2]
        real_vel = math.hypot(dx, dy) / dt

        is_stucking = (cmd_effort > CMD_EFFORT_THRESHOLD) and (real_vel < STALL_VEL_THRESHOLD)

        if is_stucking:
            if self.stall_start_time is None:
                self.stall_start_time = now
            elif now - self.stall_start_time > STALL_TRIGGER_TIME:
                self.is_stalled = True
        else:
            self.stall_start_time = None
            self.is_stalled = False

        return self.is_stalled, real_vel

    def reset(self):
        """重置检测器状态 (脱困后调用)"""
        self.history.clear()
        self.stall_start_time = None
        self.is_stalled = False

# =========================
# 4. Body Frame KF
# =========================
class BodyFrameKF:
    def __init__(self, name="target"):
        self.name = name
        self.x = None 
        self.P = np.eye(2) * 1.0
        self.Q_base = np.eye(2) * 0.01 
        self.last_update_time = time.time()
        self.last_r = None 
        self.reject_count = 0 
        self.MAX_REJECT = 8

    def predict(self, now):
        if self.x is None: return
        dt = now - self.last_update_time
        if dt < 0: dt = 0
        self.last_update_time = now
        self.P += self.Q_base * (dt * 10.0)

    def update(self, distance, bearing_deg, conf=1.0, truncated=False):
        now = time.time()
        self.predict(now)

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

        # Gating
        if self.last_r is not None and self.last_r < 0.5:
             dist_diff = np.linalg.norm(z - self.x)
             if dist_diff > 0.4 and self.reject_count < self.MAX_REJECT:
                 print(f"🛡️ [{self.name}] 拒绝突变: {dist_diff:.2f}m")
                 self.reject_count += 1
                 return
        self.reject_count = 0

        r_sigma = 0.05 
        if truncated: r_sigma *= 10.0
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

# =========================
# 5. 视觉通信
# =========================
class VisionSubscriber:
    def __init__(self, ip, port, topic):
        self.ctx = zmq.Context()
        self.sub = self.ctx.socket(zmq.SUB)
        self.sub.connect(f"tcp://{ip}:{port}")
        self.sub.setsockopt_string(zmq.SUBSCRIBE, topic)
        self.sub.setsockopt(zmq.CONFLATE, 0) 
    
    def poll_batch(self):
        """接收并解析 Batch 数据"""
        batch = []
        try:
            while True:
                if self.sub.poll(0) == 0: break
                msg = self.sub.recv_string(flags=zmq.NOBLOCK)
                _, payload = msg.split(" ", 1)
                data = json.loads(payload)
                
                if 'objects' in data and isinstance(data['objects'], list):
                    batch.extend(data['objects'])
                elif isinstance(data, dict) and 'class_id' in data:
                    batch.append(data)
        except Exception: pass
        return batch

    def select_best(self, batch, cls_id, pattern=None):
        candidates = [m for m in batch if m.get('class_id') == cls_id]
        if pattern:
            candidates = [m for m in candidates if m.get('pattern') == pattern]
        if not candidates: return None
        # 直接信任发布端的排序（面积优先）
        return candidates[0]

# =========================
# 6. 控制器 (V2.4: 自动择优 + 迟滞 + 脱困)
# =========================
class FormationController:
    def __init__(self):
        rospy.init_node('formation_lock_vision_body', anonymous=True)
        self.driver = RobotDriver(ROBOT_ID)
        self.vision = VisionSubscriber(ZMQ_IP, ZMQ_PORT, ZMQ_TOPIC)
        self.stall_detector = StallDetector()
        
        self.kf_ball = BodyFrameKF("Ball")
        self.kf_leader = BodyFrameKF("Leader")

        self.ctx_pub = zmq.Context()
        self.pub_sock = self.ctx_pub.socket(zmq.PUB)
        self.pub_sock.bind("tcp://*:5556")
        self.rate = rospy.Rate(30)

        self.is_done_state = False

        # --- 脱困状态变量 ---
        self.unstuck_mode = False
        self.unstuck_start_time = 0.0
        self.unstuck_direction = 1.0  # 1.0=左, -1.0=右

    def run(self):
        print(f"🚀 V2.4 脱困增强版 | 目标: ±{FORMATION_ANGLE_DIFF}° | 脱困: 平移{ESCAPE_DISTANCE}m")
        
        while not rospy.is_shutdown():
            batch = self.vision.poll_batch()
            raw_ball = self.vision.select_best(batch, BALL_CLASS_ID)
            raw_leader = self.vision.select_best(batch, LEADER_CLASS_ID, LEADER_PATTERN)

            if raw_ball:
                self.kf_ball.update(raw_ball['distance'], raw_ball['bearing_body'], 
                                    raw_ball.get('conf', 1.0), raw_ball.get('truncated', False))
            if raw_leader:
                self.kf_leader.update(raw_leader['distance'], raw_leader['bearing_body'],
                                      raw_leader.get('conf', 1.0), raw_leader.get('truncated', False))

            state_ball = self.kf_ball.get_state()
            state_leader = self.kf_leader.get_state()
            
            valid_control = False
            control_ball = None
            is_virtual_ball = False

            if state_leader:
                dist_leader = math.hypot(state_leader[0], state_leader[1])
                if dist_leader > BLIND_APPROACH_LIMIT:
                    if state_ball:
                        control_ball = state_ball
                        is_virtual_ball = False
                    else:
                        control_ball = state_leader 
                        is_virtual_ball = True      
                    valid_control = True
                else:
                    if state_ball:
                        control_ball = state_ball
                        is_virtual_ball = False
                        valid_control = True
                    else:
                        valid_control = False
            else:
                valid_control = False

            now = time.time()
            if (now - self.kf_leader.last_update_time > 1.0):
                valid_control = False

            if valid_control and control_ball and state_leader:
                self._control_loop(control_ball, state_leader, is_virtual_ball)
                self._publish_debug(control_ball, state_leader)
            else:
                self.driver.stop()
                self.is_done_state = False 
                # 如果丢失目标，也要重置脱困状态，防止下次直接触发
                self.unstuck_mode = False

            self.rate.sleep()

    def _control_loop(self, p_ball, p_leader, is_virtual):
        # === [新增] 0. 脱困模式 (最高优先级) ===
        if self.unstuck_mode:
            elapsed = time.time() - self.unstuck_start_time
            
            if elapsed < ESCAPE_DURATION:
                # 执行动作：稍微后退 + 横向平移
                # ESCAPE_SPEED_Y * direction (1.0 for Left, -1.0 for Right)
                cmd_vx = ESCAPE_SPEED_X 
                cmd_vy = ESCAPE_SPEED_Y * self.unstuck_direction
                
                self.driver.send_velocity_command(cmd_vx, cmd_vy, 0.0)
                # 降频打印
                if int(elapsed * 10) % 5 == 0:
                    dir_str = "LEFT" if self.unstuck_direction > 0 else "RIGHT"
                    print(f"⚠️ [UNSTUCK] 堵转脱困中({dir_str})... {elapsed:.1f}s / {ESCAPE_DURATION:.1f}s")
                return # 🚨 关键：直接返回，跳过 PID 计算
            else:
                # 时间到，退出脱困模式
                self.unstuck_mode = False
                self.stall_detector.reset() # 清空历史，防止连击
                self.driver.stop() # 先停一下
                print("✅ [UNSTUCK] 脱困完成，恢复控制")
                # 继续往下执行 PID

        # === 正常的 PID 控制逻辑 ===
        xt, yt = p_ball
        xl, yl = p_leader

        theta_robot = math.atan2(-yt, -xt)
        curr_dist = math.hypot(xt, yt)
        dist_err = curr_dist - TARGET_DIST
        abs_err = abs(dist_err)

        if self.is_done_state:
            if abs_err > DIST_DEADBAND_EXIT: self.is_done_state = False
        else:
            if abs_err < DIST_DEADBAND_ENTER: self.is_done_state = True

        angle_err_deg = 0.0
        side_indicator = ""

        if is_virtual:
            v_tan = 0.0
            mode_tag = "FAR_APP"
        else:
            theta_leader = math.atan2(yl - yt, xl - xt)
            diff_rad = math.radians(FORMATION_ANGLE_DIFF)
            target_pos = theta_leader + diff_rad 
            target_neg = theta_leader - diff_rad 

            err_pos = wrap_rad_pi(target_pos - theta_robot)
            err_neg = wrap_rad_pi(target_neg - theta_robot)

            if abs(err_pos) < abs(err_neg):
                final_err_rad = err_pos
                side_indicator = "+" # 正向/左
            else:
                final_err_rad = err_neg
                side_indicator = "-" # 负向/右

            angle_err_deg = math.degrees(final_err_rad)
            
            if self.is_done_state and abs(angle_err_deg) < 5.0:
                 v_tan = 0.0
            else:
                 v_tan = KP_THETA * final_err_rad * curr_dist
                 v_tan = max(-0.3, min(0.3, v_tan))
            
            if not self.is_done_state: mode_tag = f"LOCK{side_indicator}"

        v_rad = 0.0
        if self.is_done_state:
            v_rad = 0.0
            v_tan = 0.0
            if not is_virtual: mode_tag = "DONE_L"
        else:
            if abs_err < DIST_SOFT_ZONE:
                v_rad = -KP_DIST_SLOW * dist_err
                v_rad += math.copysign(FRICTION_FEEDFORWARD, v_rad)
                if not is_virtual: mode_tag = f"SOFT{side_indicator}"
            else:
                v_rad = -KP_DIST_FAST * dist_err
                if not is_virtual: mode_tag = f"FAST{side_indicator}"

        # === [修改] 堵转检测与触发 ===
        check_effort = abs(v_rad) if v_rad < 0 else 0.0
        is_stalled, real_vel = self.stall_detector.update(xt, yt, check_effort)
        
        if is_stalled and v_rad < 0:
            # 触发脱困
            self.unstuck_mode = True
            self.unstuck_start_time = time.time()
            
            # 智能选择方向：顺着想去的方向滑
            # side_indicator == "+" 意味着目标在左边 (+120)，所以往左平移
            if side_indicator == "+": 
                self.unstuck_direction = 1.0  # 向左
            elif side_indicator == "-": 
                self.unstuck_direction = -1.0 # 向右
            else:
                self.unstuck_direction = 1.0  # 默认向左
            
            mode_tag = "STALL"
            print(f"🛑 检测到堵转！启动平移脱困 (方向: {self.unstuck_direction})")
            return # 结束本帧，下一帧进入上面的 unstuck logic

        v_rad = max(-0.25, min(0.25, v_rad))

        # 速度合成
        th = theta_robot
        vx = v_rad * math.cos(th) - v_tan * math.sin(th)
        vy = v_rad * math.sin(th) + v_tan * math.cos(th)

        if curr_dist < 0.18:
            esc_x = 0.15 * math.cos(theta_robot)
            esc_y = 0.15 * math.sin(theta_robot)
            self.driver.send_velocity_command(esc_x, esc_y, 0.0)
            print(f"⚠️ 穿模保护: {curr_dist:.2f}m")
        else:
            self.driver.send_velocity_command(vx, vy, 0.0)
            print(f"[{mode_tag:7}] D:{curr_dist:.2f} | D_Err:{dist_err:.3f} | A_Err:{angle_err_deg:.1f}°")

    def _publish_debug(self, pb, pl):
        msg = {
            "ts": time.time(),
            "ball": {"x": pb[0], "y": pb[1]},
            "leader": {"x": pl[0], "y": pl[1]}
        }
        try:
            self.pub_sock.send_string(f"ekf_debug {json.dumps(msg)}")
        except: pass

if __name__ == "__main__":
    try:
        ctrl = FormationController()
        ctrl.run()
    except rospy.ROSInterruptException:
        pass