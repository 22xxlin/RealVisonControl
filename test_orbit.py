#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
orbit_smooth_vector.py（增强 + 盲区续航版）
- bearing unwrap + LPF
- dist LPF
- 限幅
- 丢失时：用简单死算(p_est)继续绕行以跨越FOV盲区
- 超时后：降级“搜索绕行”，再超时才停
"""

import zmq
import json
import math
import time
import rospy
from robot_driver import RobotDriver

# =========================
# 配置（按场地再调）
# =========================
ROBOT_ID = 15

TARGET_DIST = 0.70
ORBIT_SPEED = -0.15      # 切向速度（绕圈速度）
KP_DIST = 0.25           # 径向P

# 控制与安全
MAX_RADIAL = 0.25        # 径向速度限幅（m/s）
MAX_SPEED = 0.35         # 合速度限幅（m/s）
MIN_DIST_SAFE = 0.25     # 小于这个距离，强制保护（避免贴脸）
MAX_DIST_VALID = 6.0     # 视觉距离有效上限

# 滤波（alpha 越大越“跟得紧”，越小越“稳”）
ALPHA_DIST = 0.20
ALPHA_THETA = 0.30

# 丢失策略（为盲区优化）
PREDICT_KEEP_S = 4.0     # 丢失后，最多用 p_est 预测续航这么久（不中断绕行）
SEARCH_KEEP_S  = 10.0    # 丢失更久进入“搜索绕行”，超过这个仍丢则停
SEARCH_TANGENT_SCALE = 0.55  # 搜索阶段切向速度比例（更慢更稳）
PREDICT_TANGENT_SCALE = 0.75 # 预测阶段切向速度比例
PREDICT_KP_SCALE = 0.70      # 预测阶段径向P比例

# 循环频率
CONTROL_HZ = 30

# =========================
# 工具函数
# =========================
def clamp(v, lo, hi):
    return max(lo, min(hi, v))

def lpf(prev, x, alpha):
    return x if prev is None else (alpha * x + (1.0 - alpha) * prev)

def unwrap_deg(prev_unwrapped, new_deg):
    """让角度连续：把 new_deg 映射到离 prev_unwrapped 最近的等价值（+/-360k）"""
    if prev_unwrapped is None:
        return float(new_deg)
    prev_mod = prev_unwrapped % 360.0
    delta = float(new_deg) - prev_mod
    delta = (delta + 180.0) % 360.0 - 180.0  # -> [-180, 180)
    return prev_unwrapped + delta

def limit_vector(vx, vy, vmax):
    s = math.hypot(vx, vy)
    if s <= vmax or s < 1e-9:
        return vx, vy
    k = vmax / s
    return vx * k, vy * k

# =========================
# 主逻辑
# =========================
def run_test():
    rospy.init_node('orbit_smooth', anonymous=True)
    driver = RobotDriver(ROBOT_ID)

    # 连接视觉
    context = zmq.Context()
    socket = context.socket(zmq.SUB)
    socket.connect("tcp://localhost:5555")
    socket.setsockopt_string(zmq.SUBSCRIBE, "perception")

    # 只保留最新一条，避免控制用到旧数据
    try:
        socket.setsockopt(zmq.CONFLATE, 1)
    except Exception:
        pass

    # 接收超时：避免 perception 停了导致 recv 永久阻塞
    socket.setsockopt(zmq.RCVTIMEO, 200)  # ms

    print("🚀 启动：矢量合成绕球（盲区续航版）")

    rate = rospy.Rate(CONTROL_HZ)

    # 滤波状态
    dist_f = None
    theta_unwrapped = None
    theta_f = None

    # 目标状态
    last_seen_t = None
    last_loop_t = time.time()

    # 上一次已发送速度（用于丢失时死算）
    last_cmd_vx, last_cmd_vy = 0.0, 0.0

    # 目标相对位置估计（车体坐标，x前 y左）
    p_est_x, p_est_y = None, None

    try:
        while not rospy.is_shutdown():
            now = time.time()
            dt = max(1e-3, now - last_loop_t)
            last_loop_t = now

            # 尝试接收视觉
            data = None
            try:
                msg = socket.recv_string()
                payload = msg.split(' ', 1)[1]
                data = json.loads(payload)
            except zmq.Again:
                data = None
            except Exception:
                data = None

            # 解析是否看到球
            seen_ball = False
            dist = None
            theta_deg = None
            if data and data.get('class_id') == 6:
                dist = data.get('distance', None)
                theta_deg = data.get('bearing_body', None)
                if dist is not None and theta_deg is not None:
                    try:
                        dist = float(dist)
                        theta_deg = float(theta_deg)
                        if (0.05 < dist < MAX_DIST_VALID) and math.isfinite(dist) and math.isfinite(theta_deg):
                            seen_ball = True
                    except Exception:
                        seen_ball = False

            if seen_ball:
                last_seen_t = now

                # 近距离保护
                if dist <= MIN_DIST_SAFE:
                    driver.stop()
                    last_cmd_vx, last_cmd_vy = 0.0, 0.0
                    # 看到球但太近：也更新 p_est，避免恢复时突变
                    theta_unwrapped = unwrap_deg(theta_unwrapped, theta_deg)
                    theta_f = lpf(theta_f, theta_unwrapped, ALPHA_THETA)
                    dist_f = lpf(dist_f, dist, ALPHA_DIST)
                    th = math.radians(theta_f)
                    p_est_x = dist_f * math.cos(th)
                    p_est_y = dist_f * math.sin(th)
                    rate.sleep()
                    continue

                # bearing 连续化 + 滤波
                theta_unwrapped = unwrap_deg(theta_unwrapped, theta_deg)
                theta_f = lpf(theta_f, theta_unwrapped, ALPHA_THETA)

                # distance 滤波
                dist_f = lpf(dist_f, dist, ALPHA_DIST)

                theta_rad = math.radians(theta_f)

                # 更新目标相对位置估计
                p_est_x = dist_f * math.cos(theta_rad)
                p_est_y = dist_f * math.sin(theta_rad)

                # 径向P
                v_radial = (dist_f - TARGET_DIST) * KP_DIST
                v_radial = clamp(v_radial, -MAX_RADIAL, MAX_RADIAL)

                # 切向恒速
                v_tangent = ORBIT_SPEED

                # 合成
                cmd_vx = v_radial * math.cos(theta_rad) - v_tangent * math.sin(theta_rad)
                cmd_vy = v_radial * math.sin(theta_rad) + v_tangent * math.cos(theta_rad)
                cmd_vx, cmd_vy = limit_vector(cmd_vx, cmd_vy, MAX_SPEED)

                driver.send_velocity_command(cmd_vx, cmd_vy, 0.0)
                last_cmd_vx, last_cmd_vy = cmd_vx, cmd_vy

            else:
                # ========== 丢失：盲区续航 ==========
                if last_seen_t is None or p_est_x is None or p_est_y is None:
                    driver.stop()
                    last_cmd_vx, last_cmd_vy = 0.0, 0.0
                    rate.sleep()
                    continue

                dt_lost = now - last_seen_t

                # 用上一次命令速度做死算：p_est -= v * dt
                # （假设目标短时间近似静止、底盘跟速，且 omega=0）
                p_est_x -= last_cmd_vx * dt
                p_est_y -= last_cmd_vy * dt

                dist_est = math.hypot(p_est_x, p_est_y)
                theta_est = math.atan2(p_est_y, p_est_x)  # rad

                # 基本安全检查：估计距离炸了就停
                if (not math.isfinite(dist_est)) or dist_est < MIN_DIST_SAFE or dist_est > MAX_DIST_VALID:
                    driver.stop()
                    last_cmd_vx, last_cmd_vy = 0.0, 0.0
                    rate.sleep()
                    continue

                # 1) 预测续航阶段：尽量不断圈，但降速、降增益
                if dt_lost <= PREDICT_KEEP_S:
                    v_tangent = ORBIT_SPEED * PREDICT_TANGENT_SCALE
                    kp = KP_DIST * PREDICT_KP_SCALE
                    v_radial = (dist_est - TARGET_DIST) * kp
                    v_radial = clamp(v_radial, -MAX_RADIAL, MAX_RADIAL)

                    cmd_vx = v_radial * math.cos(theta_est) - v_tangent * math.sin(theta_est)
                    cmd_vy = v_radial * math.sin(theta_est) + v_tangent * math.cos(theta_est)
                    cmd_vx, cmd_vy = limit_vector(cmd_vx, cmd_vy, MAX_SPEED)

                    driver.send_velocity_command(cmd_vx, cmd_vy, 0.0)
                    last_cmd_vx, last_cmd_vy = cmd_vx, cmd_vy

                # 2) 搜索阶段：更慢地继续绕（主要靠切向把球带回视野）
                elif dt_lost <= SEARCH_KEEP_S:
                    v_tangent = ORBIT_SPEED * SEARCH_TANGENT_SCALE
                    v_radial = 0.0  # 搜索阶段避免径向漂太多（也可改成很小的回半径修正）

                    cmd_vx = - v_tangent * math.sin(theta_est)
                    cmd_vy = + v_tangent * math.cos(theta_est)
                    cmd_vx, cmd_vy = limit_vector(cmd_vx, cmd_vy, MAX_SPEED * 0.8)

                    driver.send_velocity_command(cmd_vx, cmd_vy, 0.0)
                    last_cmd_vx, last_cmd_vy = cmd_vx, cmd_vy

                # 3) 丢太久：停（避免一直“瞎绕”）
                else:
                    driver.stop()
                    last_cmd_vx, last_cmd_vy = 0.0, 0.0

            rate.sleep()

    except KeyboardInterrupt:
        pass
    finally:
        driver.stop()
        try:
            socket.close(0)
            context.term()
        except Exception:
            pass

if __name__ == "__main__":
    run_test()
