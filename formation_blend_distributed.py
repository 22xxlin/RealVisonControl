#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import math
import time
from geometry_msgs.msg import TransformStamped
from robot_driver import RobotDriver

def quat_to_yaw_rad(x, y, z, w):
    siny_cosp = 2.0 * (w*z + x*y)
    cosy_cosp = 1.0 - 2.0 * (y*y + z*z)
    return math.atan2(siny_cosp, cosy_cosp)

def wrap_deg(a):
    return (a + 180.0) % 360.0 - 180.0

def clamp(x, lo, hi):
    return lo if x < lo else (hi if x > hi else x)

def norm2(x, y):
    return math.hypot(x, y)

def limit_xy(vx, vy, vmax):
    s = math.hypot(vx, vy)
    if s < 1e-9 or s <= vmax:
        return vx, vy
    k = vmax / s
    return vx*k, vy*k

def blend_angle_deg(a_deg, b_deg, beta):
    """圆周插值，避免 179° 与 -179° 线性插值炸掉"""
    a = math.radians(a_deg)
    b = math.radians(b_deg)
    x = (1.0-beta)*math.cos(a) + beta*math.cos(b)
    y = (1.0-beta)*math.sin(a) + beta*math.sin(b)
    return math.degrees(math.atan2(y, x))

def ensure_abs_topic(t):
    t = t.strip()
    if not t.startswith("/"):
        t = "/" + t
    return t

class PoseCache:
    def __init__(self):
        self.robot  = None  # {x,y,yaw,ts}
        self.target = None  # {x,y,ts}
        self.leader = None  # {x,y,ts}

    def cb_robot(self, msg):
        x = msg.transform.translation.x
        y = msg.transform.translation.y
        q = msg.transform.rotation
        yaw = quat_to_yaw_rad(q.x, q.y, q.z, q.w)
        self.robot = {"x": x, "y": y, "yaw": yaw, "ts": time.time()}

    def cb_target(self, msg):
        x = msg.transform.translation.x
        y = msg.transform.translation.y
        self.target = {"x": x, "y": y, "ts": time.time()}

    def cb_leader(self, msg):
        x = msg.transform.translation.x
        y = msg.transform.translation.y
        self.leader = {"x": x, "y": y, "ts": time.time()}

def main():
    rospy.init_node("formation_blend_distributed", anonymous=True)

    # ---- role params ----
    robot_id = int(rospy.get_param("~robot_id"))
    role = rospy.get_param("~role", "follower")  # "leader" or "follower"
    offset_deg = float(rospy.get_param("~offset_deg", 0.0))  # follower: 120 / -120, leader通常0

    # ---- topics (你的命名：vicon/VSWARM45/VSWARM45) ----
    robot_topic  = ensure_abs_topic(rospy.get_param("~robot_topic",  f"vicon/VSWARM{robot_id}/VSWARM{robot_id}"))
    target_topic = ensure_abs_topic(rospy.get_param("~target_topic", "vicon/VSWARM45/VSWARM45"))
    leader_topic = ensure_abs_topic(rospy.get_param("~leader_topic", "vicon/VSWARM13/VSWARM13"))

    # ---- formation params ----
    radius = float(rospy.get_param("~radius", 0.20))
    control_hz = float(rospy.get_param("~control_hz", 30.0))

    kp_dist = float(rospy.get_param("~kp_dist", 0.35))
    kp_theta = float(rospy.get_param("~kp_theta", 0.30))  # 切向做“软一点”更不散

    vmax = float(rospy.get_param("~vmax", 0.30))
    max_rad = float(rospy.get_param("~max_radial", 0.25))
    max_tan = float(rospy.get_param("~max_tangent", 0.20))

    # safety: 别在 radius 附近 stop！只在真正危险才处理
    min_dist = float(rospy.get_param("~min_dist", 0.12))
    push_vmax = float(rospy.get_param("~push_vmax", 0.25))
    push_k = float(rospy.get_param("~push_k", 1.0))

    # blend (leader_line -> ball_vel)
    v_low  = float(rospy.get_param("~v_low", 0.02))   # 球速低于这个：完全用 leader_line
    v_high = float(rospy.get_param("~v_high", 0.10))  # 球速高于这个：完全用 ball_vel
    theta_lpf_alpha = float(rospy.get_param("~theta_lpf_alpha", 0.7))  # 参考角低通(0~1)

    # follower command smoothing
    cmd_alpha = float(rospy.get_param("~cmd_alpha", 0.7))

    # leader mode
    leader_mode = rospy.get_param("~leader_mode", "hold")  # "hold" or "radial"
    use_body_frame = bool(rospy.get_param("~use_body_frame", True))

    cache = PoseCache()
    rospy.Subscriber(robot_topic,  TransformStamped, cache.cb_robot,  queue_size=1)
    rospy.Subscriber(target_topic, TransformStamped, cache.cb_target, queue_size=1)
    rospy.Subscriber(leader_topic, TransformStamped, cache.cb_leader, queue_size=1)

    driver = RobotDriver(robot_id)

    last_ball = None
    theta_ref_deg = float(rospy.get_param("~init_theta_ref_deg", 0.0))
    cmd_vx_prev, cmd_vy_prev = 0.0, 0.0

    rate = rospy.Rate(control_hz)
    rospy.loginfo(f"[{robot_id}] role={role} offset={offset_deg} "
                  f"robot_topic={robot_topic} target_topic={target_topic} leader_topic={leader_topic}")

    try:
        while not rospy.is_shutdown():
            if cache.robot is None or cache.target is None:
                driver.stop()
                rate.sleep()
                continue

            pr = cache.robot
            pt = cache.target

            # leader pose needed for leader_line (对 follower 必需；对 leader 自己也可用于初始化)
            if cache.leader is None:
                # 启动时 leader 丢失就先不动，避免乱跑
                driver.stop()
                rate.sleep()
                continue
            pl = cache.leader

            # ----- compute theta_leader_line -----
            dx_lt = pl["x"] - pt["x"]
            dy_lt = pl["y"] - pt["y"]
            theta_leader_deg = math.degrees(math.atan2(dy_lt, dx_lt))

            # ----- estimate ball velocity direction -----
            theta_ball_deg = None
            ball_speed = 0.0
            if last_ball is not None:
                dt = pt["ts"] - last_ball["ts"]
                if dt > 1e-3:
                    vx = (pt["x"] - last_ball["x"]) / dt
                    vy = (pt["y"] - last_ball["y"]) / dt
                    ball_speed = math.hypot(vx, vy)
                    if ball_speed > 1e-6:
                        theta_ball_deg = math.degrees(math.atan2(vy, vx))
            last_ball = {"x": pt["x"], "y": pt["y"], "ts": pt["ts"]}

            # ----- beta from ball speed -----
            if theta_ball_deg is None:
                beta = 0.0
            else:
                if v_high <= v_low:
                    beta = 1.0 if ball_speed > v_low else 0.0
                else:
                    beta = clamp((ball_speed - v_low) / (v_high - v_low), 0.0, 1.0)

            # ----- blend reference angle -----
            theta_ref_raw = theta_leader_deg if theta_ball_deg is None else blend_angle_deg(theta_leader_deg, theta_ball_deg, beta)

            # low-pass filter theta_ref (reduce jitter)
            theta_ref_deg = blend_angle_deg(theta_ref_deg, theta_ref_raw, theta_lpf_alpha)

            # =========================
            # Leader behavior
            # =========================
            if role == "leader":
                if leader_mode == "hold":
                    # 你手摆好就锁死（最符合你的偏好）
                    driver.send_velocity_command(0.0, 0.0, 0.0)
                    rate.sleep()
                    continue
                # leader_mode == "radial": 只做径向保持半径，不绕圈
                # fall through to shared controller with kp_theta=0
                local_kp_theta = 0.0
            else:
                local_kp_theta = kp_theta

            # =========================
            # Follower (or leader radial) controller
            # =========================
            dx = pr["x"] - pt["x"]
            dy = pr["y"] - pt["y"]
            dist = math.hypot(dx, dy)
            if dist < 1e-6:
                driver.stop()
                rate.sleep()
                continue

            # outward radial unit
            urx, ury = dx/dist, dy/dist
            # CCW tangent unit
            utx, uty = -ury, urx

            theta_robot_deg = math.degrees(math.atan2(dy, dx))  # angle(ball->robot)

            target_angle_deg = theta_ref_deg + offset_deg
            ang_err_deg = wrap_deg(target_angle_deg - theta_robot_deg)

            # safety: too close -> push outward, kill tangent
            if dist < min_dist:
                push = clamp(push_k*(min_dist - dist), 0.0, push_vmax)
                vx_w = push * urx
                vy_w = push * ury
            else:
                dist_err = dist - radius
                v_rad = clamp(-kp_dist * dist_err, -max_rad, +max_rad)
                v_tan = clamp(local_kp_theta * math.radians(ang_err_deg) * dist, -max_tan, +max_tan)

                vx_w = v_rad * urx + v_tan * utx
                vy_w = v_rad * ury + v_tan * uty

            vx_w, vy_w = limit_xy(vx_w, vy_w, vmax)

            # world -> body (match your original style)
            if use_body_frame:
                cy = math.cos(pr["yaw"])
                sy = math.sin(pr["yaw"])
                vx_b =  cy*vx_w + sy*vy_w
                vy_b = -sy*vx_w + cy*vy_w
            else:
                vx_b, vy_b = vx_w, vy_w

            # smooth
            vx_cmd = cmd_alpha*vx_b + (1.0-cmd_alpha)*cmd_vx_prev
            vy_cmd = cmd_alpha*vy_b + (1.0-cmd_alpha)*cmd_vy_prev
            cmd_vx_prev, cmd_vy_prev = vx_cmd, vy_cmd

            driver.send_velocity_command(vx_cmd, vy_cmd, 0.0)  # 车头锁死

            rate.sleep()

    finally:
        driver.stop()

if __name__ == "__main__":
    main()
