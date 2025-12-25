#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import rospy
from geometry_msgs.msg import TransformStamped
from robot_driver import RobotDriver


# ----------------- helpers -----------------
def quat_to_yaw_rad(x, y, z, w):
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)

def wrap_deg(a):
    return (a + 180.0) % 360.0 - 180.0

def clamp(x, lo, hi):
    return lo if x < lo else (hi if x > hi else x)

def limit_xy(vx, vy, vmax):
    s = math.hypot(vx, vy)
    if s < 1e-9 or s <= vmax:
        return vx, vy
    k = vmax / s
    return vx * k, vy * k

def ensure_abs_topic(t):
    t = (t or "").strip()
    if not t.startswith("/"):
        t = "/" + t
    return t

def ts_from_msg(msg: TransformStamped) -> float:
    # Use ROS time for consistency across machines
    return msg.header.stamp.to_sec() if msg.header.stamp else rospy.get_time()

def blend_angle_deg(a_deg, b_deg, beta):
    """Circular interpolation to avoid wrap problems."""
    a = math.radians(a_deg)
    b = math.radians(b_deg)
    x = (1.0 - beta) * math.cos(a) + beta * math.cos(b)
    y = (1.0 - beta) * math.sin(a) + beta * math.sin(b)
    return math.degrees(math.atan2(y, x))


# ----------------- pose cache -----------------
class PoseCache:
    def __init__(self):
        self.robots = {}   # id -> {x,y,yaw,ts}
        self.ball = None   # {x,y,ts}

    def make_robot_cb(self, rid: int):
        def _cb(msg: TransformStamped):
            x = msg.transform.translation.x
            y = msg.transform.translation.y
            q = msg.transform.rotation
            yaw = quat_to_yaw_rad(q.x, q.y, q.z, q.w)
            self.robots[rid] = {"x": x, "y": y, "yaw": yaw, "ts": ts_from_msg(msg)}
        return _cb

    def cb_ball(self, msg: TransformStamped):
        x = msg.transform.translation.x
        y = msg.transform.translation.y
        self.ball = {"x": x, "y": y, "ts": ts_from_msg(msg)}


# ----------------- main -----------------
def main():
    rospy.init_node("formation_carry_teamready", anonymous=True)

    # ---- IDs ----
    robot_id = int(rospy.get_param("~robot_id"))        # this process controls which robot
    robot_ids = rospy.get_param("~robot_ids", [10, 13, 15])  # the whole team
    robot_ids = [int(x) for x in robot_ids]

    ball_id = int(rospy.get_param("~ball_id", 45))
    leader_id = int(rospy.get_param("~leader_id", 13))

    # ---- topics ----
    # Your naming: vicon/VSWARM{ID}/VSWARM{ID}
    robot_topics = {}
    for rid in robot_ids:
        robot_topics[rid] = ensure_abs_topic(rospy.get_param(
            f"~robot_topic_{rid}",
            f"vicon/VSWARM{rid}/VSWARM{rid}"
        ))
    ball_topic = ensure_abs_topic(rospy.get_param("~ball_topic", f"vicon/VSWARM{ball_id}/VSWARM{ball_id}"))

    # ---- formation ----
    radius = float(rospy.get_param("~radius", 0.20))
    kp_dist = float(rospy.get_param("~kp_dist", 0.35))
    kp_theta = float(rospy.get_param("~kp_theta", 0.30))     # tangential softer than radial is usually safer
    vmax = float(rospy.get_param("~vmax", 0.30))
    max_rad = float(rospy.get_param("~max_radial", 0.25))
    max_tan = float(rospy.get_param("~max_tangent", 0.20))

    # safety: true danger zone (DON'T stop at radius)
    min_dist = float(rospy.get_param("~min_dist", 0.12))
    push_k = float(rospy.get_param("~push_k", 1.0))
    push_vmax = float(rospy.get_param("~push_vmax", 0.25))

    # ---- reference angle: leader-line -> ball-velocity (blend) ----
    v_low = float(rospy.get_param("~v_low", 0.02))
    v_high = float(rospy.get_param("~v_high", 0.10))
    theta_lpf_alpha = float(rospy.get_param("~theta_lpf_alpha", 0.7))
    theta_ref_deg = float(rospy.get_param("~init_theta_ref_deg", 0.0))
    last_ball = None

    # ---- auto-carry (TEAM READY trigger + ramp) ----
    auto_carry = bool(rospy.get_param("~auto_carry", True))
    dist_tol = float(rospy.get_param("~dist_tol", 0.02))          # |dist-R| < 2cm
    ang_tol_deg = float(rospy.get_param("~ang_tol_deg", 10.0))    # |angle error| < 10deg
    center_tol = float(rospy.get_param("~center_tol", 0.05))      # ball near centroid
    settle_time = float(rospy.get_param("~settle_time", 1.2))     # must stay ready for this long
    carry_ramp_time = float(rospy.get_param("~carry_ramp_time", 2.0))  # ramp-up seconds

    # ---- carry command (fixed world heading) ----
    carry_target_speed = float(rospy.get_param("~carry_speed", 0.05))    # final carry speed
    carry_heading_deg = float(rospy.get_param("~carry_heading_deg", 0.0)) # 0=+X, 90=+Y (adjust for your world)
    press_bias = float(rospy.get_param("~press_bias", 0.02))             # inward (helps keep contact when side-pushing)
    carry_enable = bool(rospy.get_param("~carry_enable", True))

    # ---- offsets (120° triangle but “carry-friendly”: rear pusher + two front stabilizers) ----
    # Default: leader=rear(180°), others = +60°/-60° (still 120° apart).
    # If you want the classic 0/+120/-120, pass offsets_deg list.
    offsets_deg = rospy.get_param("~offsets_deg", None)
    offset_map = {}
    if offsets_deg is not None:
        # Expect same length as robot_ids
        offsets_deg = [float(x) for x in offsets_deg]
        if len(offsets_deg) != len(robot_ids):
            raise ValueError("~offsets_deg length must match ~robot_ids")
        for rid, off in zip(robot_ids, offsets_deg):
            offset_map[rid] = off
    else:
        # Build default mapping
        others = sorted([rid for rid in robot_ids if rid != leader_id])
        offset_map[leader_id] = 180.0
        if len(others) == 2:
            offset_map[others[0]] = 60.0
            offset_map[others[1]] = -60.0
        else:
            # fallback
            for i, rid in enumerate(others):
                offset_map[rid] = (i * 120.0) % 360.0

    # Local override (only if you REALLY need it; better pass offsets_deg on all nodes)
    if rospy.has_param("~offset_deg"):
        offset_map[robot_id] = float(rospy.get_param("~offset_deg"))

    # ---- output / body-frame conversion ----
    use_body_frame = bool(rospy.get_param("~use_body_frame", True))  # match your style
    cmd_alpha = float(rospy.get_param("~cmd_alpha", 0.7))            # smoothing

    control_hz = float(rospy.get_param("~control_hz", 30.0))

    # ---- subscribe ----
    cache = PoseCache()
    for rid in robot_ids:
        rospy.Subscriber(robot_topics[rid], TransformStamped, cache.make_robot_cb(rid), queue_size=1)
    rospy.Subscriber(ball_topic, TransformStamped, cache.cb_ball, queue_size=1)

    driver = RobotDriver(robot_id)

    # ---- carry state ----
    team_ready_since = None
    carry_start_ts = None

    # ---- cmd smoothing ----
    cmd_vx_prev, cmd_vy_prev = 0.0, 0.0

    rate = rospy.Rate(control_hz)
    rospy.loginfo(
        f"[{robot_id}] TEAM={robot_ids} leader={leader_id} ball={ball_id} "
        f"offset_map={offset_map} carry={carry_target_speed}m/s heading={carry_heading_deg}deg "
        f"ball_topic={ball_topic}"
    )

    try:
        while not rospy.is_shutdown():
            # Need ball + all robots for TEAM READY trigger
            if cache.ball is None:
                driver.stop()
                rate.sleep()
                continue

            missing = [rid for rid in robot_ids if rid not in cache.robots]
            if missing:
                driver.stop()
                rate.sleep()
                continue

            # Need this robot's pose
            pr = cache.robots[robot_id]
            pb = cache.ball
            pl = cache.robots[leader_id]

            # ---------------- reference angle (leader-line -> ball-vel) ----------------
            # leader-line
            theta_leader_deg = math.degrees(math.atan2(pl["y"] - pb["y"], pl["x"] - pb["x"]))

            # ball velocity
            theta_ball_deg = None
            ball_speed = 0.0
            if last_ball is not None:
                dt = pb["ts"] - last_ball["ts"]
                if dt > 1e-3:
                    vx_b = (pb["x"] - last_ball["x"]) / dt
                    vy_b = (pb["y"] - last_ball["y"]) / dt
                    ball_speed = math.hypot(vx_b, vy_b)
                    if ball_speed > 1e-6:
                        theta_ball_deg = math.degrees(math.atan2(vy_b, vx_b))
            last_ball = {"x": pb["x"], "y": pb["y"], "ts": pb["ts"]}

            if theta_ball_deg is None:
                beta = 0.0
            else:
                if v_high <= v_low:
                    beta = 1.0 if ball_speed > v_low else 0.0
                else:
                    beta = clamp((ball_speed - v_low) / (v_high - v_low), 0.0, 1.0)

            theta_ref_raw = theta_leader_deg if theta_ball_deg is None else blend_angle_deg(theta_leader_deg, theta_ball_deg, beta)
            theta_ref_deg = blend_angle_deg(theta_ref_deg, theta_ref_raw, theta_lpf_alpha)

            # ---------------- compute TEAM READY ----------------
            # centroid of team
            cx = sum(cache.robots[rid]["x"] for rid in robot_ids) / len(robot_ids)
            cy = sum(cache.robots[rid]["y"] for rid in robot_ids) / len(robot_ids)
            center_err = math.hypot(pb["x"] - cx, pb["y"] - cy)

            all_good = (center_err < center_tol)

            # Each robot must satisfy dist + angle error
            for rid in robot_ids:
                pi = cache.robots[rid]
                dx = pi["x"] - pb["x"]
                dy = pi["y"] - pb["y"]
                di = math.hypot(dx, dy)
                if di < 1e-6:
                    all_good = False
                    break

                theta_i_deg = math.degrees(math.atan2(dy, dx))  # angle(ball->robot_i)
                target_i_deg = theta_ref_deg + offset_map[rid]
                ang_err_i = abs(wrap_deg(target_i_deg - theta_i_deg))

                if abs(di - radius) > dist_tol or ang_err_i > ang_tol_deg:
                    all_good = False
                    break

            now = rospy.get_time()
            if auto_carry and carry_enable and carry_target_speed > 1e-6:
                if carry_start_ts is None:
                    if all_good:
                        if team_ready_since is None:
                            team_ready_since = now
                        elif (now - team_ready_since) >= settle_time:
                            carry_start_ts = now
                    else:
                        team_ready_since = None
            else:
                # manual / disabled
                team_ready_since = None

            # compute ramped carry speed
            carry_speed_cmd = 0.0
            if carry_start_ts is not None:
                if carry_ramp_time <= 1e-3:
                    carry_speed_cmd = carry_target_speed
                else:
                    s = clamp((now - carry_start_ts) / carry_ramp_time, 0.0, 1.0)
                    carry_speed_cmd = carry_target_speed * s

            # ---------------- controller for THIS robot ----------------
            dx = pr["x"] - pb["x"]
            dy = pr["y"] - pb["y"]
            dist = math.hypot(dx, dy)
            if dist < 1e-6:
                driver.stop()
                rate.sleep()
                continue

            urx, ury = dx / dist, dy / dist     # outward radial
            utx, uty = -ury, urx                # CCW tangent

            theta_robot_deg = math.degrees(math.atan2(dy, dx))  # angle(ball->this_robot)
            target_angle_deg = theta_ref_deg + offset_map[robot_id]
            ang_err_deg = wrap_deg(target_angle_deg - theta_robot_deg)

            # safety: too close -> push outward, no tangent
            if dist < min_dist:
                push = clamp(push_k * (min_dist - dist), 0.0, push_vmax)
                vx_w = push * urx
                vy_w = push * ury
            else:
                dist_err = dist - radius
                v_rad = clamp(-kp_dist * dist_err, -max_rad, +max_rad)

                # when carrying, add inward press to keep side contact with ball
                if carry_speed_cmd > 1e-6:
                    v_rad = v_rad - abs(press_bias)

                v_tan = clamp(kp_theta * math.radians(ang_err_deg) * dist, -max_tan, +max_tan)

                vx_w = v_rad * urx + v_tan * utx
                vy_w = v_rad * ury + v_tan * uty

            # carry term (world)
            if carry_speed_cmd > 1e-6:
                h = math.radians(carry_heading_deg)
                vx_w += carry_speed_cmd * math.cos(h)
                vy_w += carry_speed_cmd * math.sin(h)

            # limit
            vx_w, vy_w = limit_xy(vx_w, vy_w, vmax)

            # world -> body (omni, head locked)
            if use_body_frame:
                cyaw = math.cos(pr["yaw"])
                syaw = math.sin(pr["yaw"])
                vx_b = cyaw * vx_w + syaw * vy_w
                vy_b = -syaw * vx_w + cyaw * vy_w
            else:
                vx_b, vy_b = vx_w, vy_w

            # smooth
            vx_cmd = cmd_alpha * vx_b + (1.0 - cmd_alpha) * cmd_vx_prev
            vy_cmd = cmd_alpha * vy_b + (1.0 - cmd_alpha) * cmd_vy_prev
            cmd_vx_prev, cmd_vy_prev = vx_cmd, vy_cmd

            driver.send_velocity_command(vx_cmd, vy_cmd, 0.0)  # omega locked
            rate.sleep()

    finally:
        driver.stop()


if __name__ == "__main__":
    main()
