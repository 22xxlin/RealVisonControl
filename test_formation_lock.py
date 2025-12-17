#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
test_formation_vicon.py (120度 围捕版)
功能：利用 Vicon 真值，控制 Robot 15 保持在 球(Target) 的指定半径上，
      并始终与 Leader 车保持 120 度夹角。
"""

import rospy
import math
import time
import numpy as np
from geometry_msgs.msg import TransformStamped
from robot_driver import RobotDriver

# =========================
# 1. VICON 角色配置
# =========================
ROBOT_ID = 15
TARGET_ID = 45 # 球

# 你的 Leader 车 (请修改这里!)
LEADER_VICON_TOPIC = "/vicon/VSWARM13/VSWARM13"  # 假设 Leader 是 08
TARGET_VICON_TOPIC = "vicon/VSWARM45/VSWARM45"
ROBOT_VICON_TOPIC  = f"vicon/VSWARM{ROBOT_ID}/VSWARM{ROBOT_ID}"

# =========================
# 2. 编队参数
# =========================
FORMATION_ANGLE_DIFF = 120.0  # 你要在 Leader 的 +120 度位置 (逆时针)
# 如果你是要在另一侧，改成 -120.0

TARGET_DIST = 0.20       # 围捕半径
MAX_SPEED   = 0.35       # 最大合速度

# 径向控制 (保持距离)
KP_DIST     = 0.25       
MAX_RADIAL  = 0.25

# 切向控制 (保持角度)
# 这是一个位置环，P给大一点没关系，因为我们要追角度
KP_THETA    = 0.8        # 切向速度增益: v_tangent = error_rad * KP
MAX_TANGENT = 0.30       # 切向限幅

CONTROL_HZ = 30

# =========================
# 工具与数学
# =========================
def quat_to_yaw_deg(x, y, z, w):
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return (math.degrees(yaw) + 360.0) % 360.0

def normalize_angle_deg(angle):
    """将角度归一化到 [-180, 180]"""
    return (angle + 180.0) % 360.0 - 180.0

def limit_vector(vx, vy, vmax):
    s = math.hypot(vx, vy)
    if s <= vmax or s < 1e-9: return vx, vy
    k = vmax / s
    return vx * k, vy * k

class ViconSystem:
    def __init__(self):
        self.poses = {'robot': None, 'target': None, 'leader': None}
        
        # 订阅三个对象
        rospy.Subscriber(ROBOT_VICON_TOPIC, TransformStamped, self._cb, 'robot')
        rospy.Subscriber(TARGET_VICON_TOPIC, TransformStamped, self._cb, 'target')
        rospy.Subscriber(LEADER_VICON_TOPIC, TransformStamped, self._cb, 'leader')
        
        print("📡 Vicon 系统就绪 (Robot, Target, Leader)")

    def _cb(self, msg, key):
        x = msg.transform.translation.x
        y = msg.transform.translation.y
        rot = msg.transform.rotation
        yaw = quat_to_yaw_deg(rot.x, rot.y, rot.z, rot.w)
        self.poses[key] = {'x': x, 'y': y, 'yaw': yaw, 'ts': time.time()}

    def get_formation_state(self):
        """
        计算相对于球的坐标系信息
        返回: 
           dist_err (距离误差), 
           angle_err (角度误差, deg), 
           robot_yaw_relative_to_ball (用于速度分解)
        """
        p_r = self.poses['robot']
        p_t = self.poses['target']
        p_l = self.poses['leader']
        now = time.time()

        # 1. 完整性检查
        if not (p_r and p_t and p_l): return None
        if (now - p_r['ts']>0.5) or (now - p_t['ts']>0.5) or (now - p_l['ts']>0.5):
            return None # 超时

        # 2. 计算【Leader】相对于【球】的世界角度
        dx_lt = p_l['x'] - p_t['x']
        dy_lt = p_l['y'] - p_t['y']
        theta_leader_deg = math.degrees(math.atan2(dy_lt, dx_lt)) # 全局角度

        # 3. 计算【Robot】相对于【球】的当前状态
        dx_rt = p_r['x'] - p_t['x']
        dy_rt = p_r['y'] - p_t['y']
        current_dist = math.hypot(dx_rt, dy_rt)
        theta_robot_deg = math.degrees(math.atan2(dy_rt, dx_rt)) # 全局角度

        # 4. 计算目标角度 (Robot 应该在哪)
        # 目标 = Leader角度 + 120度
        target_angle_deg = theta_leader_deg + FORMATION_ANGLE_DIFF
        
        # 5. 计算误差
        dist_err = current_dist - TARGET_DIST
        
        # 角度误差 (需要处理 360 跳变)
        angle_err = normalize_angle_deg(target_angle_deg - theta_robot_deg)
        
        # 6. 计算 Robot 车头相对于 Robot-球连线的夹角 (用于把 Vr/Vt 分解成 Vx/Vy)
        # 这是一个坐标变换的关键点
        # 我们需要在 Robot 自身的坐标系下执行 Vr (前进/后退) 和 Vt (横移/旋转)
        # 但这里的底盘是全向的，我们可以直接合成世界坐标系速度，再转回车身系
        
        return {
            'dist_err': dist_err,
            'angle_err': angle_err,
            'robot_yaw': p_r['yaw'],
            'theta_robot_global': theta_robot_deg,
            'current_dist': current_dist
        }

# =========================
# 主逻辑
# =========================
def run_formation():
    rospy.init_node('formation_test', anonymous=True)
    driver = RobotDriver(ROBOT_ID)
    vicon = ViconSystem()
    rate = rospy.Rate(CONTROL_HZ)
    
    print(f"🚀 启动围捕模式: 保持 {FORMATION_ANGLE_DIFF}° 相对 Leader")

    try:
        while not rospy.is_shutdown():
            state = vicon.get_formation_state()

            if state:
                # --- A. 控制律计算 ---
                
                # 1. 径向速度 (保持距离) -> 靠近/远离球
                # 距离大于目标 -> 负速度(靠近); 距离小于目标 -> 正速度(远离)
                # 注意：这里我们定义 v_rad 指向圆心。
                # 按照通常习惯：Error = Current - Target. 
                # 如果 Current > Target (太远), Error > 0. 我们需要靠近 (Velocity 指向球).
                # 世界坐标系中，从球指向车的向量是 (cos(theta), sin(theta))
                # 所以 速度向量 = -1 * P * err * (cos, sin)
                
                v_rad_mag = -1.0 * KP_DIST * state['dist_err']
                v_rad_mag = max(-MAX_RADIAL, min(MAX_RADIAL, v_rad_mag))
                
                # 2. 切向速度 (追赶角度) -> 沿圆周运动
                # Angle Err = Target - Current.
                # 如果 Target 在 Current 的逆时针方向 (+), Err > 0.
                # 我们需要逆时针转. 切向向量是 (-sin, cos).
                
                # 把角度误差转换成弧度距离: arc_len = r * theta_rad
                # 但直接用 P 控制角度差更简单
                v_tan_mag = KP_THETA * math.radians(state['angle_err']) * state['current_dist']
                v_tan_mag = max(-MAX_TANGENT, min(MAX_TANGENT, v_tan_mag))

                # --- B. 速度合成 (世界坐标系) ---
                theta_global_rad = math.radians(state['theta_robot_global'])
                
                # 径向单位向量 (从球指向车)
                ur_x = math.cos(theta_global_rad)
                ur_y = math.sin(theta_global_rad)
                
                # 切向单位向量 (逆时针方向)
                ut_x = -math.sin(theta_global_rad)
                ut_y = math.cos(theta_global_rad)
                
                # 世界坐标系速度
                vx_world = v_rad_mag * ur_x + v_tan_mag * ut_x
                vy_world = v_rad_mag * ur_y + v_tan_mag * ut_y
                
                # --- C. 转换到 Robot 车身坐标系 ---
                # Robot Yaw (deg) -> rad
                yaw_rad = math.radians(state['robot_yaw'])
                
                # 旋转矩阵 R^T (World -> Body)
                # vx_body =  cos(yaw)*vx_w + sin(yaw)*vy_w
                # vy_body = -sin(yaw)*vx_w + cos(yaw)*vy_w
                cmd_vx =  math.cos(yaw_rad) * vx_world + math.sin(yaw_rad) * vy_world
                cmd_vy = -math.sin(yaw_rad) * vx_world + math.cos(yaw_rad) * vy_world
                
                # --- D. 限幅与发送 ---
                cmd_vx, cmd_vy = limit_vector(cmd_vx, cmd_vy, MAX_SPEED)
                
                # 刹车区
                if state['current_dist'] < 0.20:
                    driver.stop()
                    print("🛑 太近了，紧急避障")
                else:
                    driver.send_velocity_command(cmd_vx, cmd_vy, 0.0)
                    print(f"ERR: Ang{state['angle_err']:5.1f}° Dist{state['dist_err']:5.2f}m | CMD: {cmd_vx:.2f}, {cmd_vy:.2f}")

            else:
                driver.stop()
                print("⚠️ 等待 Vicon (Target/Leader 缺失)...")

            rate.sleep()

    except KeyboardInterrupt:
        pass
    finally:
        driver.stop()

if __name__ == "__main__":
    run_formation()