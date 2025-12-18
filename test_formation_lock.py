#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
test_formation_soft_stall.py
功能：专为解决“软着陆时不触发堵转检测”而优化。
策略：
  1. 堵转检测阈值极低 (0.002)，确保任何微小推力都被监控。
  2. 引入“最小进给力” (Feedforward)，防止静摩擦导致误判。
"""

import rospy
import math
import time
import collections
import numpy as np
from geometry_msgs.msg import TransformStamped
from robot_driver import RobotDriver

# =========================
# 1. 核心参数
# =========================
ROBOT_ID = 15
TARGET_ID = 45

# Topics
LEADER_TOPIC = "/vicon/VSWARM13/VSWARM13"
TARGET_TOPIC = "vicon/VSWARM45/VSWARM45"
ROBOT_TOPIC  = f"vicon/VSWARM{ROBOT_ID}/VSWARM{ROBOT_ID}"

# 几何参数
FORMATION_ANGLE_DIFF = -120.0 
TARGET_DIST = 0.25       # 目标距离 (稍大一点，利用死区)

# --- 区域控制 ---
DIST_DEADBAND  = 0.015   # [LOCKED] 死区 1.5cm
DIST_SOFT_ZONE = 0.06    # [SOFT]   软着陆区 6cm

# --- PID 参数 ---
KP_DIST_FAST = 0.35      # 远距离 P
KP_DIST_SLOW = 0.25      # 近距离 P (稍微加大，配合摩擦力补偿)
KP_THETA     = 0.80

MAX_SPEED = 0.35

# --- 堵转检测 (你的逻辑) ---
STALL_CHECK_WINDOW   = 0.30  
STALL_VEL_THRESHOLD  = 0.02  # 速度小于 2cm/s 算没动

# 【关键修改】: 降到极低，只要有指令就监控
CMD_EFFORT_THRESHOLD = 0.002 

# 触发时间: 给起步留出 1.0s 的宽容度
STALL_TRIGGER_TIME   = 1.0   

# --- 物理特性 (新增) ---
# 这是一个极其重要的参数：克服静摩擦的最小指令
# 如果 P 算出来的力气小于这个，车是根本动不了的，必须补偿
FRICTION_FEEDFORWARD = 0.02 

# =========================
# 2. 功能类
# =========================
class StallDetector:
    def __init__(self):
        self.history = collections.deque(maxlen=50) 
        self.stall_start_time = None
        self.is_stalled = False

    def update(self, current_x, current_y, cmd_vel_mag):
        now = time.time()
        self.history.append((now, current_x, current_y))
        
        # 寻找历史数据
        past_record = None
        for record in self.history:
            if now - record[0] >= STALL_CHECK_WINDOW:
                past_record = record
                break
        
        # 启动初期或数据不足
        if past_record is None: return False, 0.0
        
        dt = now - past_record[0]
        if dt < 1e-3: return False, 0.0
        
        # 计算真实速度
        real_vel = math.hypot(current_x - past_record[1], current_y - past_record[2]) / dt

        # === 核心判定 (采用你的低阈值逻辑) ===
        # 只要有一丁点指令 (0.002) 且 没动 (<0.02)
        is_stucking = (cmd_vel_mag > CMD_EFFORT_THRESHOLD) and (real_vel < STALL_VEL_THRESHOLD)

        if is_stucking:
            if self.stall_start_time is None:
                self.stall_start_time = now
            elif now - self.stall_start_time > STALL_TRIGGER_TIME:
                self.is_stalled = True
        else:
            self.stall_start_time = None
            self.is_stalled = False 
            
        return self.is_stalled, real_vel

class ViconSystem:
    def __init__(self):
        self.poses = {'robot': None, 'target': None, 'leader': None}
        rospy.Subscriber(ROBOT_TOPIC, TransformStamped, self._cb, 'robot')
        rospy.Subscriber(TARGET_TOPIC, TransformStamped, self._cb, 'target')
        rospy.Subscriber(LEADER_TOPIC, TransformStamped, self._cb, 'leader')

    def _cb(self, msg, key):
        x, y = msg.transform.translation.x, msg.transform.translation.y
        rot = msg.transform.rotation
        # Quat -> Yaw
        siny = 2.0 * (rot.w * rot.z + rot.x * rot.y)
        cosy = 1.0 - 2.0 * (rot.y * rot.y + rot.z * rot.z)
        yaw = (math.degrees(math.atan2(siny, cosy)) + 360.0) % 360.0
        self.poses[key] = {'x': x, 'y': y, 'yaw': yaw, 'ts': time.time()}

    def get_state(self):
        p_r, p_t, p_l = self.poses['robot'], self.poses['target'], self.poses['leader']
        if not (p_r and p_t and p_l): return None
        if time.time() - p_r['ts'] > 0.5: return None

        dx, dy = p_r['x'] - p_t['x'], p_r['y'] - p_t['y']
        curr_dist = math.hypot(dx, dy)
        theta_robot = math.degrees(math.atan2(dy, dx))
        
        theta_leader = math.degrees(math.atan2(p_l['y'] - p_t['y'], p_l['x'] - p_t['x']))
        target_ang = theta_leader + FORMATION_ANGLE_DIFF
        angle_err = (target_ang - theta_robot + 180) % 360 - 180

        return {
            'dist_err': curr_dist - TARGET_DIST,
            'angle_err': angle_err,
            'robot_yaw': p_r['yaw'],
            'theta_robot_global': theta_robot,
            'current_dist': curr_dist,
            'robot_pos': (p_r['x'], p_r['y'])
        }

# =========================
# 3. 主程序
# =========================
def run_soft_stall_control():
    rospy.init_node('formation_soft_stall', anonymous=True)
    driver = RobotDriver(ROBOT_ID)
    vicon = ViconSystem()
    stall_detector = StallDetector()
    rate = rospy.Rate(30)
    
    print(f"🚀 启动软着陆优化版: Threshold={CMD_EFFORT_THRESHOLD}, FeedForward={FRICTION_FEEDFORWARD}")

    try:
        while not rospy.is_shutdown():
            state = vicon.get_state()
            if state:
                d_err = state['dist_err']
                abs_err = abs(d_err)
                v_rad = 0.0
                mode_log = "FAST"

                # --- 1. 死区逻辑 ---
                if abs_err < DIST_DEADBAND:
                    v_rad = 0.0
                    mode_log = "LOCK"
                
                # --- 2. 软着陆逻辑 (含摩擦力补偿) ---
                elif abs_err < DIST_SOFT_ZONE:
                    mode_log = "SOFT"
                    # 基础 P 控制
                    raw_p = -1.0 * KP_DIST_SLOW * d_err
                    
                    # 【重要优化】: 只要误差存在，就加上最小推力
                    # 如果 raw_p 是 0.004，加上 0.04 -> 0.044 (足够推动车)
                    # 如果 加上后车还不动 -> 下面的 0.002 阈值检测器 就会报警
                    if raw_p > 0:
                        v_rad = raw_p + FRICTION_FEEDFORWARD
                    elif raw_p < 0:
                        v_rad = raw_p - FRICTION_FEEDFORWARD
                    else:
                        v_rad = 0
                
                # --- 3. 正常逻辑 ---
                else:
                    v_rad = -1.0 * KP_DIST_FAST * d_err

                # --- 4. 堵转检测 (你的低阈值策略) ---
                # 只有当我们想靠近球(v_rad<0)时才检测
                check_force = abs(v_rad) if v_rad < 0 else 0.0
                
                is_stalled, real_vel = stall_detector.update(
                    state['robot_pos'][0], 
                    state['robot_pos'][1], 
                    check_force 
                )

                if is_stalled and v_rad < 0:
                    v_rad = 0.0
                    mode_log = "STALLED"

                # 限幅
                v_rad = max(-0.25, min(0.25, v_rad))

                # --- 5. 切向控制与执行 ---
                v_tan = KP_THETA * math.radians(state['angle_err']) * state['current_dist']
                v_tan = max(-0.3, min(0.3, v_tan))

                # 坐标变换
                th = math.radians(state['theta_robot_global'])
                vx_w = v_rad * math.cos(th) - v_tan * math.sin(th)
                vy_w = v_rad * math.sin(th) + v_tan * math.cos(th)
                
                yaw = math.radians(state['robot_yaw'])
                vx_b =  math.cos(yaw)*vx_w + math.sin(yaw)*vy_w
                vy_b = -math.sin(yaw)*vx_w + math.cos(yaw)*vy_w

                # 穿模保护
                if state['current_dist'] < 0.18:
                    driver.send_velocity_command(0.1, 0.0, 0.0)
                    print(f"⚠️ 穿模倒车")
                else:
                    driver.send_velocity_command(vx_b, vy_b, 0.0)
                    # 打印详细信息方便你看效果
                    print(f"[{mode_log:7}] Err:{d_err:.3f} | Cmd:{v_rad:.3f} | RealV:{real_vel:.3f}")

            else:
                driver.stop()
            rate.sleep()

    except KeyboardInterrupt:
        pass
    finally:
        driver.stop()

if __name__ == "__main__":
    run_soft_stall_control()