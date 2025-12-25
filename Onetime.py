#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
test_formation_lock_vision_ekf.py

融合版 V2.6 (结束前平移版):
- [新增] 任务阶段管理 (Tracking -> Sliding -> Finished)。
- [新增] 在锁定目标后，执行一次开环的向左平移 0.2m，然后结束程序。
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
ROBOT_ID = 10

# 视觉 ZMQ
ZMQ_IP   = "127.0.0.1"
ZMQ_PORT = 5555
ZMQ_TOPIC = "perception"

# 目标定义
BALL_CLASS_ID = 6
LEADER_CLASS_ID = 2
LEADER_PATTERN = "SOLID"

# 队形参数
FORMATION_ANGLE_DIFF = 120.0  
TARGET_DIST = 0.25
BLIND_APPROACH_LIMIT = 0.8 

# PID 参数
DIST_DEADBAND_ENTER = 0.015
DIST_SOFT_ZONE = 0.08
KP_DIST_FAST = 0.20
KP_DIST_SLOW = 0.10
KP_THETA     = 0.80

# 堵转检测
STALL_CHECK_WINDOW   = 0.30
STALL_VEL_THRESHOLD  = 0.02
CMD_EFFORT_THRESHOLD = 0.002
STALL_TRIGGER_TIME   = 1.0
FRICTION_FEEDFORWARD = 0.02

# [新增] 平移任务参数
SLIDE_VEL_Y = -0.1   # m/s, 正值通常为左 (根据你的底盘坐标系调整)
SLIDE_DIST  = 0.2   # m
SLIDE_DURATION = SLIDE_DIST / abs(SLIDE_VEL_Y) # 2.0秒

# =========================
# 2. 辅助工具 (保持不变)
# =========================
def wrap_rad_pi(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi

def wrap_deg_180(a):
    return (a + 180.0) % 360.0 - 180.0

# =========================
# 3. 堵转检测 (保持不变)
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

# =========================
# 4. Body Frame KF (保持不变)
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

    def predict(self, now, ego_vx=0.0, ego_vy=0.0, ego_w=0.0):
        if self.x is None: return
        dt = now - self.last_update_time
        if dt < 1e-6: return
        
        theta = -ego_w * dt
        c, s = math.cos(theta), math.sin(theta)
        px, py = self.x[0], self.x[1]
        
        px_rot = px * c - py * s
        py_rot = px * s + py * c
        
        self.x[0] = px_rot - ego_vx * dt
        self.x[1] = py_rot - ego_vy * dt
        
        self.last_update_time = now
        self.P += self.Q_base * (dt * 10.0)

    def update(self, distance, bearing_deg, ego_vx=0.0, ego_vy=0.0, ego_w=0.0, conf=1.0, truncated=False):
        now = time.time()
        self.predict(now, ego_vx, ego_vy, ego_w)

        if truncated:
            conf *= 0.1 

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

        if self.last_r is not None and self.last_r < 0.5:
             dist_diff = np.linalg.norm(z - self.x)
             if dist_diff > 0.4 and self.reject_count < self.MAX_REJECT and not truncated:
                 print(f"🛡️ [{self.name}] 拒绝突变: {dist_diff:.2f}m")
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

# =========================
# 5. 视觉通信 (保持不变)
# =========================
class VisionSubscriber:
    def __init__(self, ip, port, topic):
        self.ctx = zmq.Context()
        self.sub = self.ctx.socket(zmq.SUB)
        self.sub.connect(f"tcp://{ip}:{port}")
        self.sub.setsockopt_string(zmq.SUBSCRIBE, topic)
        self.sub.setsockopt(zmq.CONFLATE, 0)

    def poll_batch(self):
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
        return candidates[0]

# =========================
# 6. 控制器 (V2.6: Tracking -> Sliding -> Finish)
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
        
        self.last_cmd_vx = 0.0
        self.last_cmd_vy = 0.0
        self.last_cmd_w  = 0.0 

        # [新增] 任务阶段控制
        # PHASES: "TRACKING" -> "SLIDING" -> "FINISHED"
        self.mission_phase = "TRACKING" 
        self.slide_start_time = None

    def run(self):
        print(f"🚀 V2.6 自动流程版 | 锁定 -> 左移0.2m -> 结束")

        while not rospy.is_shutdown():
            # 1. 始终运行 EKF 保持状态更新（即使在 Sliding 阶段，也记录数据供调试）
            batch = self.vision.poll_batch()
            raw_ball = self.vision.select_best(batch, BALL_CLASS_ID)
            raw_leader = self.vision.select_best(batch, LEADER_CLASS_ID, LEADER_PATTERN)

            if raw_ball:
                self.kf_ball.update(raw_ball['distance'], raw_ball['bearing_body'],
                    self.last_cmd_vx, self.last_cmd_vy, self.last_cmd_w,
                    raw_ball.get('conf', 1.0), raw_ball.get('truncated', False))
            else:
                self.kf_ball.predict(time.time(), self.last_cmd_vx, self.last_cmd_vy, self.last_cmd_w)

            if raw_leader:
                self.kf_leader.update(raw_leader['distance'], raw_leader['bearing_body'],
                    self.last_cmd_vx, self.last_cmd_vy, self.last_cmd_w,
                    raw_leader.get('conf', 1.0), raw_leader.get('truncated', False))
            else:
                self.kf_leader.predict(time.time(), self.last_cmd_vx, self.last_cmd_vy, self.last_cmd_w)

            state_ball = self.kf_ball.get_state()
            state_leader = self.kf_leader.get_state()

            # 2. 状态机逻辑
            vx, vy = 0.0, 0.0

            if self.mission_phase == "TRACKING":
                # === 阶段1：视觉锁定 ===
                is_tracking_finished = False
                
                # 判断是否有有效控制
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
                
                # 超时检查
                if (time.time() - self.kf_leader.last_update_time > 1.0):
                    valid_control = False

                if valid_control and control_ball and state_leader:
                    # 计算控制量
                    vx, vy, is_tracking_finished = self._control_loop(control_ball, state_leader, is_virtual_ball)
                    self._publish_debug(control_ball, state_leader)
                    
                    # 穿模保护
                    curr_dist = math.hypot(control_ball[0], control_ball[1])
                    if curr_dist < 0.18:
                        theta_robot = math.atan2(-control_ball[1], -control_ball[0])
                        vx = 0.15 * math.cos(theta_robot)
                        vy = 0.15 * math.sin(theta_robot)
                        is_tracking_finished = False # 保护触发时不结束
                        print(f"⚠️ 穿模保护: {curr_dist:.2f}m")
                else:
                    vx, vy = 0.0, 0.0
                
                # [状态转换] Tracking -> Sliding
                if is_tracking_finished:
                    print("✅ 视觉锁定完成。切换至开环平移模式 (Sliding)...")
                    self.mission_phase = "SLIDING"
                    self.slide_start_time = time.time()
                    vx, vy = 0.0, 0.0 # 本帧先停一下

            elif self.mission_phase == "SLIDING":
                # === 阶段2：开环平移 ===
                # 这里不看视觉，直接发指令
                # 假设 vy > 0 是左移 (麦轮常见定义: x前, y左, z上)
                vx = 0.0
                vy = SLIDE_VEL_Y
                
                elapsed = time.time() - self.slide_start_time
                remaining = SLIDE_DURATION - elapsed
                
                if elapsed % 0.5 < 0.05: # 每0.5秒打印一次
                     print(f"[SLIDING] 向左平移中... 剩余 {remaining:.1f}s")

                # [状态转换] Sliding -> Finished
                if elapsed >= SLIDE_DURATION:
                    print("✅ 平移完成。")
                    self.mission_phase = "FINISHED"
                    vx, vy = 0.0, 0.0

            elif self.mission_phase == "FINISHED":
                # === 阶段3：结束 ===
                print("🎉 所有任务结束。停止程序。")
                self.driver.send_velocity_command(0.0, 0.0, 0.0)
                time.sleep(0.2)
                break

            # 3. 发送指令
            self.driver.send_velocity_command(vx, vy, 0.0)
            
            self.last_cmd_vx = vx
            self.last_cmd_vy = vy
            self.last_cmd_w  = 0.0 

            self.rate.sleep()

    def _control_loop(self, p_ball, p_leader, is_virtual):
        """
        返回值: (vx, vy, is_finished)
        """
        xt, yt = p_ball
        xl, yl = p_leader

        theta_robot = math.atan2(-yt, -xt)
        curr_dist = math.hypot(xt, yt)
        dist_err = curr_dist - TARGET_DIST
        abs_err = abs(dist_err)

        if not self.is_done_state:
            if abs_err < DIST_DEADBAND_ENTER: 
                self.is_done_state = True

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
                side_indicator = "+" 
            else:
                final_err_rad = err_neg
                side_indicator = "-" 

            angle_err_deg = math.degrees(final_err_rad)

            if self.is_done_state and abs(angle_err_deg) < 5.0:
                 v_tan = 0.0
            else:
                 v_tan = KP_THETA * final_err_rad * curr_dist
                 v_tan = max(-0.20, min(0.20, v_tan))

            if not self.is_done_state: mode_tag = f"LOCK{side_indicator}"

        v_rad = 0.0
        if self.is_done_state:
            v_rad = 0.0
            if v_tan == 0.0:
                if not is_virtual: mode_tag = "DONE_L"
            else:
                mode_tag = "ALIGN"
        else:
            if abs_err < DIST_SOFT_ZONE:
                v_rad = -KP_DIST_SLOW * dist_err
                v_rad += math.copysign(FRICTION_FEEDFORWARD, v_rad)
                if not is_virtual: mode_tag = f"SOFT{side_indicator}"
            else:
                v_rad = -KP_DIST_FAST * dist_err
                if not is_virtual: mode_tag = f"FAST{side_indicator}"

        check_effort = abs(v_rad) if v_rad < 0 else 0.0
        is_stalled, real_vel = self.stall_detector.update(xt, yt, check_effort)

        if is_stalled and v_rad < 0:
            v_rad = 0.0
            mode_tag = "STALL"

        v_rad = max(-0.10, min(0.10, v_rad))

        th = theta_robot
        vx = v_rad * math.cos(th) - v_tan * math.sin(th)
        vy = v_rad * math.sin(th) + v_tan * math.cos(th)

        print(f"[{mode_tag:7}] D:{curr_dist:.2f} | D_Err:{dist_err:.3f} | A_Err:{angle_err_deg:.1f}°")
        
        # 判断 Tracking 阶段是否结束
        is_tracking_finished = False
        if self.is_done_state and vx == 0.0 and vy == 0.0:
            is_tracking_finished = True

        return vx, vy, is_tracking_finished

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