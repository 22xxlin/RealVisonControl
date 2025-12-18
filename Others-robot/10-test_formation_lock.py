#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
test_formation_lock_optimized.py
功能：基于 Vicon 的围捕控制（带死区与软着陆优化），防止由于传感器噪声导致的推球/顶牛。
"""

import rospy
import math
import time
import numpy as np
from geometry_msgs.msg import TransformStamped
from robot_driver import RobotDriver

# =========================
# 1. 配置参数
# =========================
ROBOT_ID = 10
TARGET_ID = 45 # 球

# Vicon Topic
LEADER_TOPIC = "/vicon/VSWARM13/VSWARM13"
TARGET_TOPIC = "vicon/VSWARM45/VSWARM45"
ROBOT_TOPIC  = f"vicon/VSWARM{ROBOT_ID}/VSWARM{ROBOT_ID}"

# 编队几何参数
FORMATION_ANGLE_DIFF = 120.0  # 相对 Leader 的角度偏移
# 【重要】物理半径 + 安全余量。建议比实际紧贴距离大 1-2cm
TARGET_DIST = 0.27           

# 运动学限制
MAX_SPEED = 0.3
MAX_RADIAL_SPEED = 0.25
MAX_TANGENT_SPEED = 0.30
CONTROL_HZ = 30

# 优化控制参数 (死区与分段P)
DIST_DEADBAND = 0.010   # 死区: 1.5cm (在此误差内不输出径向速度)
DIST_SOFT_ZONE = 0.05   # 软着陆区: 5cm (在此范围内降低增益)

KP_DIST_FAST = 0.3     # 远距离径向增益
KP_DIST_SLOW = 0.15     # 近距离径向增益 (软着陆)
KP_THETA     = 0.80     # 切向(角度)增益

# =========================
# 2. 数学工具函数
# =========================
def quat_to_yaw_deg(x, y, z, w):
    """四元数转 Yaw 角度 (0~360)"""
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return (math.degrees(yaw) + 360.0) % 360.0

def normalize_angle_deg(angle):
    """角度归一化到 [-180, 180]"""
    return (angle + 180.0) % 360.0 - 180.0

def limit_vector(vx, vy, vmax):
    """向量限幅"""
    s = math.hypot(vx, vy)
    if s <= vmax or s < 1e-9:
        return vx, vy
    k = vmax / s
    return vx * k, vy * k

# =========================
# 3. Vicon 数据处理类
# =========================
class ViconSystem:
    def __init__(self):
        self.poses = {'robot': None, 'target': None, 'leader': None}
        self._sub_robot = rospy.Subscriber(ROBOT_TOPIC, TransformStamped, self._cb, 'robot')
        self._sub_target = rospy.Subscriber(TARGET_TOPIC, TransformStamped, self._cb, 'target')
        self._sub_leader = rospy.Subscriber(LEADER_TOPIC, TransformStamped, self._cb, 'leader')
        print("📡 Vicon 监听中...")

    def _cb(self, msg, key):
        """通用回调，提取位置和偏航角"""
        x = msg.transform.translation.x
        y = msg.transform.translation.y
        rot = msg.transform.rotation
        yaw = quat_to_yaw_deg(rot.x, rot.y, rot.z, rot.w)
        self.poses[key] = {'x': x, 'y': y, 'yaw': yaw, 'ts': time.time()}

    def get_formation_state(self):
        """
        计算控制所需的相对状态
        Returns: {dist_err, angle_err, robot_yaw, theta_robot_global, current_dist} 或 None
        """
        p_r, p_t, p_l = self.poses['robot'], self.poses['target'], self.poses['leader']
        now = time.time()

        # 完整性与超时检查 (0.5s)
        if not (p_r and p_t and p_l): return None
        if any((now - p['ts'] > 0.5) for p in [p_r, p_t, p_l]): return None

        # 1. 计算 Leader-球 角度 (基准角度)
        theta_leader = math.degrees(math.atan2(p_l['y'] - p_t['y'], p_l['x'] - p_t['x']))

        # 2. 计算 Robot-球 状态 (当前状态)
        dx, dy = p_r['x'] - p_t['x'], p_r['y'] - p_t['y']
        curr_dist = math.hypot(dx, dy)
        theta_robot = math.degrees(math.atan2(dy, dx))

        # 3. 计算误差
        target_angle = theta_leader + FORMATION_ANGLE_DIFF
        angle_err = normalize_angle_deg(target_angle - theta_robot)
        dist_err = curr_dist - TARGET_DIST  # >0: 太远, <0: 太近

        return {
            'dist_err': dist_err,
            'angle_err': angle_err,
            'robot_yaw': p_r['yaw'],          # 车身朝向
            'theta_robot_global': theta_robot,# 车相对于球的方位角
            'current_dist': curr_dist
        }

# =========================
# 4. 主控制逻辑
# =========================
def run_formation():
    rospy.init_node('formation_lock_opt', anonymous=True)
    driver = RobotDriver(ROBOT_ID)
    vicon = ViconSystem()
    rate = rospy.Rate(CONTROL_HZ)
    
    print(f"🚀 启动优化版围捕: Deadband={DIST_DEADBAND}m, TargetDist={TARGET_DIST}m")

    try:
        while not rospy.is_shutdown():
            state = vicon.get_formation_state()

            if state:
                # --- A. 径向控制 (带死区 + 软着陆) ---
                d_err = state['dist_err']
                abs_d_err = abs(d_err)
                v_rad = 0.0
                mode_log = "FAST"

                # 1. 死区保护 (防止贴紧时抖动)
                if abs_d_err < DIST_DEADBAND:
                    v_rad = 0.0
                    mode_log = "LOCK"
                
                # 2. 软着陆 (接近目标时降低 P)
                elif abs_d_err < DIST_SOFT_ZONE:
                    v_rad = -1.0 * KP_DIST_SLOW * d_err
                    mode_log = "SOFT"
                
                # 3. 快速接近 (距离较远)
                else:
                    v_rad = -1.0 * KP_DIST_FAST * d_err
                
                # 径向限幅
                v_rad = max(-MAX_RADIAL_SPEED, min(MAX_RADIAL_SPEED, v_rad))

                # --- B. 切向控制 (角度追踪) ---
                # 将角度误差转为切向线速度
                # arc_velocity = angular_err(rad) * radius * k
                v_tan = KP_THETA * math.radians(state['angle_err']) * state['current_dist']
                v_tan = max(-MAX_TANGENT_SPEED, min(MAX_TANGENT_SPEED, v_tan))

                # --- C. 速度合成 (世界坐标系 -> 车身坐标系) ---
                # 1. 计算世界坐标系下的单位向量
                # Ur: 径向单位向量 (从球指向车)
                # Ut: 切向单位向量 (逆时针)
                theta_rad = math.radians(state['theta_robot_global'])
                ur = np.array([math.cos(theta_rad), math.sin(theta_rad)])
                ut = np.array([-math.sin(theta_rad), math.cos(theta_rad)])

                # 2. 合成世界速度向量
                v_world = v_rad * ur + v_tan * ut

                # 3. 旋转到车身系 (Body Frame)
                # R_bw = [[cos, sin], [-sin, cos]]
                yaw_rad = math.radians(state['robot_yaw'])
                c, s = math.cos(yaw_rad), math.sin(yaw_rad)
                
                vx_body =  c * v_world[0] + s * v_world[1]
                vy_body = -s * v_world[0] + c * v_world[1]

                # --- D. 最终执行 ---
                # 物理碰撞极限保护 (如果小于0.18m，强制后退，防止嵌入)
                if state['current_dist'] < 0.18:
                    print("⚠️ 距离过近，强制后退!")
                    driver.send_velocity_command(0.1, 0.0, 0.0) # 假设 X+ 是车头，且车头对着球，需根据车体调整方向
                else:
                    vx_lim, vy_lim = limit_vector(vx_body, vy_body, MAX_SPEED)
                    driver.send_velocity_command(vx_lim, vy_lim, 0.0)
                    
                    # 简洁的调试信息
                    print(f"[{mode_log}] DistErr:{d_err:6.3f} | AngErr:{state['angle_err']:6.1f}° | V_rad:{v_rad:.2f}")

            else:
                driver.stop()
                print("⏳ 等待 Vicon 数据...", end='\r')

            rate.sleep()

    except KeyboardInterrupt:
        print("\n🛑 用户终止")
    finally:
        driver.stop()

if __name__ == "__main__":
    run_formation()