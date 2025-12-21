#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
combined_transport_node_v3_6.py
[最终修复版]
1. 导航逻辑：基于最小代价(Cost-based)自动选择 +120° 或 -120°。
2. 防抖动：增加了 Side Selection Hysteresis (选边迟滞)，防止在 180° 处震荡。
3. 系统稳定性：保留了之前的 NoneType 检查和 Stall 检测。
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
# 1. 核心参数配置
# ==========================================
# 颜色类别ID定义（用于识别不同颜色的灯光信号）
BLUE, RED, GREEN, PURPLE, GRAY = 1, 2, 3, 4, 5
BALL_CLASS_ID = 6  # 球的类别ID（用于视觉识别目标球）
LEADER_CLASS_ID = 2  # 领队机器人的类别ID（红色灯光）

# 编队几何参数
FORMATION_ANGLE_DIFF = 120.0  # 编队角度差（度），跟随者相对领队的角度偏移（±120度形成三角形编队）
TARGET_DIST = 0.25  # 目标距离（米），跟随者与球之间的期望距离
BLIND_APPROACH_LIMIT = 0.8  # 盲接近限制（米），当距离领队超过此值时，优先接近球而非直接接近领队

# PID控制器参数 & 死区控制
DIST_DEADBAND_ENTER = 0.02 # 距离死区进入阈值（米），当误差小于此值时进入死区，停止距离调整
DIST_DEADBAND_EXIT  = 0.05  # 距离死区退出阈值（米），当误差大于此值时退出死区，恢复距离调整（防止抖动）
DIST_SOFT_ZONE      = 0.08   # 距离软区间（米），在此范围内使用较慢的PID增益，实现平滑控制
KP_DIST_FAST = 0.35  # 距离PID快速增益，用于较大误差时的快速响应
KP_DIST_SLOW = 0.20  # 距离PID慢速增益，用于小误差时的精细调整
KP_THETA     = 0.80  # 角度PID增益，用于控制机器人的切向运动（绕球旋转）
FRICTION_FEEDFORWARD = 0.02  # 摩擦力前馈补偿（米/秒），用于克服静摩擦力，避免低速时的爬行现象

# 锁定与防堵转参数
LOCK_TIME_THRESHOLD = 0.8  # 锁定时间阈值（秒），位置稳定超过此时间后认为入位完成
STALL_CHECK_WINDOW   = 0.30  # 堵转检测时间窗口（秒），在此时间内检测机器人是否移动
STALL_VEL_THRESHOLD  = 0.02  # 堵转速度阈值（米/秒），实际速度低于此值认为可能堵转
CMD_EFFORT_THRESHOLD = 0.002  # 命令力度阈值，发送的速度命令大于此值时才检测堵转
STALL_TRIGGER_TIME   = 1.0  # 堵转触发时间（秒），持续堵转超过此时间后触发保护

# 迟滞参数（关键 - 用于防止选边抖动）
SIDE_HYSTERESIS_BIAS = 0.25  # 选边迟滞偏置（弧度，约14度），给当前选择的边增加优势，防止在180度附近震荡

def mono() -> float:
    """获取单调时间（秒），用于计时，不受系统时间调整影响"""
    return time.monotonic()

def wrap_rad_pi(a):
    """
    将角度归一化到 [-pi, pi] 区间
    参数:
        a: 输入角度（弧度）
    返回:
        归一化后的角度（弧度）
    """
    return (a + math.pi) % (2.0 * math.pi) - math.pi

def heading_to_vxy(speed: float, heading_deg: float):
    """
    将速度和航向角转换为x、y方向的速度分量
    参数:
        speed: 速度大小（米/秒）
        heading_deg: 航向角（度），0度为x轴正方向
    返回:
        (vx, vy): x和y方向的速度分量
    """
    rad = math.radians(heading_deg)
    return speed * math.cos(rad), speed * math.sin(rad)

def log(msg: str):
    """
    带时间戳的日志输出
    参数:
        msg: 日志消息内容
    """
    ts = time.strftime("%H:%M:%S")
    print(f"[{ts}] {msg}", flush=True)

# ==========================================
# 2. 辅助类 (Stall & KF)
# ==========================================
class StallDetector:
    """
    堵转检测器 - 用于检测机器人是否被卡住无法移动
    通过比较发送的速度命令和实际移动速度来判断是否堵转
    """
    def __init__(self):
        self.history = collections.deque(maxlen=50)  # 历史位置记录队列，最多保存50个记录
        self.stall_start_time = None  # 堵转开始时间
        self.is_stalled = False  # 当前是否处于堵转状态

    def update(self, rel_x, rel_y, cmd_effort):
        """
        更新堵转检测状态
        参数:
            rel_x: 相对x坐标（米）
            rel_y: 相对y坐标（米）
            cmd_effort: 命令力度（速度命令的绝对值）
        返回:
            是否检测到堵转
        """
        now = mono()
        self.history.append((now, rel_x, rel_y))

        # 查找时间窗口前的历史记录
        past_record = None
        for record in self.history:
            if now - record[0] >= STALL_CHECK_WINDOW:
                past_record = record
                break

        if past_record is None: return False

        dt = now - past_record[0]
        if dt < 1e-3: return False

        # 计算实际移动速度
        dx = rel_x - past_record[1]
        dy = rel_y - past_record[2]
        real_vel = math.hypot(dx, dy) / dt

        # 判断是否堵转：有命令但速度很小
        is_stucking = (cmd_effort > CMD_EFFORT_THRESHOLD) and (real_vel < STALL_VEL_THRESHOLD)

        if is_stucking:
            if self.stall_start_time is None: self.stall_start_time = now
            elif now - self.stall_start_time > STALL_TRIGGER_TIME: self.is_stalled = True
        else:
            self.stall_start_time = None
            self.is_stalled = False

        return self.is_stalled

class BodyFrameKF:
    """
    机体坐标系卡尔曼滤波器 - 用于平滑和预测目标位置
    通过融合多次观测数据，减少噪声影响，提供更稳定的位置估计
    """
    def __init__(self, name="target"):
        self.name = name  # 滤波器名称（用于调试）
        self.x = None  # 状态向量 [x, y]，表示目标在机体坐标系中的位置
        self.P = np.eye(2) * 1.0  # 状态协方差矩阵，表示位置估计的不确定性
        self.Q_base = np.eye(2) * 0.01  # 过程噪声基础值，模拟目标运动的不确定性
        self.last_update_time = mono()  # 上次更新时间
        self.last_r = None  # 上次测量的距离
        self.reject_count = 0  # 连续拒绝测量的次数（用于异常值检测）
        self.MAX_REJECT = 8  # 最大拒绝次数，超过后强制接受测量

    def predict(self, now):
        """
        预测步骤 - 根据时间推进状态估计
        参数:
            now: 当前时间戳
        """
        if self.x is None: return
        dt = now - self.last_update_time
        if dt < 0: dt = 0
        self.last_update_time = now
        # 增加不确定性（协方差随时间增长）
        self.P += self.Q_base * (dt * 10.0)

    def update(self, distance, bearing_deg, conf=1.0, truncated=False):
        """
        更新步骤 - 融合新的观测数据
        参数:
            distance: 测量距离（米）
            bearing_deg: 测量方位角（度），相对于机体坐标系
            conf: 置信度（0-1），用于调整测量噪声
            truncated: 是否被截断（目标部分在视野外），截断时使用上次距离
        """
        now = mono()
        self.predict(now)
        b_rad = math.radians(bearing_deg)

        # 如果被截断，使用上次的距离
        if truncated and self.last_r is not None: meas_dist = self.last_r
        else: meas_dist = distance

        # 将极坐标转换为笛卡尔坐标
        z = np.array([meas_dist * math.cos(b_rad), meas_dist * math.sin(b_rad)])

        # 初始化状态
        if self.x is None:
            self.x = z
            self.P = np.eye(2) * 0.5
            self.last_r = distance
            return

        # Gating 门控 - 防止突变（异常值检测）
        if self.last_r is not None and self.last_r < 0.5:
             dist_diff = np.linalg.norm(z - self.x)
             if dist_diff > 0.4:  # 位置变化超过0.4米，可能是异常值
                 self.reject_count += 1
                 if self.reject_count < self.MAX_REJECT: return  # 拒绝此次测量
             else:
                 self.reject_count = 0

        # 设置测量噪声
        r_sigma = 0.05  # 基础测量噪声标准差
        if truncated: r_sigma *= 10.0  # 截断时增加噪声
        if conf < 0.8: r_sigma *= 2.0  # 低置信度时增加噪声
        R = np.eye(2) * (r_sigma ** 2)

        # 卡尔曼滤波更新
        try:
            y = z - self.x  # 测量残差
            S = self.P + R  # 残差协方差
            K = self.P @ np.linalg.inv(S)  # 卡尔曼增益
            self.x = self.x + K @ y  # 状态更新
            self.P = (np.eye(2) - K) @ self.P  # 协方差更新
        except: pass

        # 更新距离记录
        if not truncated: self.last_r = math.hypot(self.x[0], self.x[1])

    def get_state(self):
        """
        获取当前状态估计
        返回:
            (x, y) 或 None（如果状态无效或超时）
        """
        if self.x is None: return None
        if mono() - self.last_update_time > 1.0: return None  # 超过1秒未更新，认为失效
        return float(self.x[0]), float(self.x[1])

# ==========================================
# 3. 入位驾驶仪 (FormationPilot) - 核心逻辑修复
# ==========================================
class FormationPilot:
    """
    编队入位驾驶仪 - 控制跟随机器人自动入位到编队位置
    核心功能：
    1. 融合球和领队的视觉信息
    2. 智能选择左侧或右侧入位（基于代价最小原则）
    3. 使用PID控制器调整位置和角度
    4. 防止堵转和抖动
    """
    def __init__(self, driver: RobotDriver):
        self.driver = driver  # 机器人驱动器
        self.kf_ball = BodyFrameKF("Ball")  # 球的卡尔曼滤波器
        self.kf_leader = BodyFrameKF("Leader")  # 领队的卡尔曼滤波器
        self.stall_detector = StallDetector()  # 堵转检测器

        self.stable_since = None  # 位置稳定开始时间
        self.is_in_deadband = False  # 是否在死区内（距离误差很小）
        self.latest_side_intent = None  # [新增] 记录选边意图（"+" 表示左侧，"-" 表示右侧）
        self.last_debug_err = (0.0, 0.0)  # 上次的误差（用于调试）
        self.mode_tag = "INIT"  # 当前模式标签（用于调试）

    def update(self, vision_batch) -> str:
        """
        更新控制逻辑
        参数:
            vision_batch: 视觉检测结果列表
        返回:
            状态字符串: "LOST", "ADJUSTING", "LOCKED_LEFT", "LOCKED_RIGHT"
        """
        # 1. 视觉更新 - 从视觉数据中提取球和领队信息
        raw_ball = self._select_best(vision_batch, BALL_CLASS_ID)
        raw_leader = self._select_best(vision_batch, LEADER_CLASS_ID, "SOLID")

        # 更新卡尔曼滤波器
        if raw_ball:
            self.kf_ball.update(raw_ball['distance'], raw_ball['bearing_body'],
                                raw_ball.get('conf', 1.0), raw_ball.get('truncated', False))
        if raw_leader:
            self.kf_leader.update(raw_leader['distance'], raw_leader['bearing_body'],
                                  raw_leader.get('conf', 1.0), raw_leader.get('truncated', False))

        # 获取滤波后的位置
        p_ball, p_leader = self.kf_ball.get_state(), self.kf_leader.get_state()

        # 2. 目标决策 - 决定追踪球还是领队
        target_pos, leader_pos_ref, is_virtual_ball = None, None, False
        if p_leader:
            dist_to_leader = math.hypot(p_leader[0], p_leader[1])
            if dist_to_leader > BLIND_APPROACH_LIMIT:  # 距离领队较远
                if p_ball: target_pos, leader_pos_ref, self.mode_tag = p_ball, p_leader, "FAR_BALL"
                else: target_pos, leader_pos_ref, is_virtual_ball, self.mode_tag = p_leader, p_leader, True, "FAR_VIRT"
            else:  # 距离领队较近
                if p_ball: target_pos, leader_pos_ref, self.mode_tag = p_ball, p_leader, "NEAR_BALL"
                else: self.mode_tag = "NEAR_LOST"
        else: self.mode_tag = "NO_LEADER"

        # 如果没有目标，停止运动
        if not target_pos:
            self.driver.stop()
            self.stable_since = None
            self.is_in_deadband = False
            self.latest_side_intent = None
            return "LOST"

        # 3. 计算 PID 控制量
        xt, yt = target_pos  # 目标位置（球或虚拟球）
        xl, yl = leader_pos_ref  # 领队位置
        theta_robot = math.atan2(-yt, -xt)  # 机器人朝向目标的角度
        curr_dist = math.hypot(xt, yt)  # 当前距离
        dist_err = curr_dist - TARGET_DIST  # 距离误差

        # 死区控制 - 防止在目标距离附近抖动
        deadband = DIST_DEADBAND_EXIT if self.is_in_deadband else DIST_DEADBAND_ENTER
        self.is_in_deadband = abs(dist_err) < deadband

        v_tan, angle_err_deg, side_char = 0.0, 0.0, "?"  # 切向速度、角度误差、选边标记

        if is_virtual_ball:
            pass # 虚拟球模式不需要选边（直接接近领队）
        else:
            # ==========================================
            # [核心修复] 智能择优 & 迟滞逻辑
            # ==========================================
            theta_leader = math.atan2(yl - yt, xl - xt)  # 从球指向领队的角度
            diff_rad = math.radians(FORMATION_ANGLE_DIFF)  # 120度（弧度）

            # 1. 计算两个候选点的角度误差
            err_pos = wrap_rad_pi((theta_leader + diff_rad) - theta_robot)  # 对应 +120° (左侧)
            err_neg = wrap_rad_pi((theta_leader - diff_rad) - theta_robot)  # 对应 -120° (右侧)

            # 2. 计算基础代价（角度差的绝对值）
            cost_pos = abs(err_pos)
            cost_neg = abs(err_neg)

            # 3. [迟滞] 偏向保持之前的选择，防止在180度附近震荡
            if self.latest_side_intent == "+":
                cost_pos -= SIDE_HYSTERESIS_BIAS  # 给左侧减少代价
            elif self.latest_side_intent == "-":
                cost_neg -= SIDE_HYSTERESIS_BIAS  # 给右侧减少代价

            # 4. 择优 - 选择代价更小的一侧
            if cost_pos < cost_neg:
                final_err_rad, side_char = err_pos, "+"
            else:
                final_err_rad, side_char = err_neg, "-"

            # 5. 更新意图
            self.latest_side_intent = side_char

            angle_err_deg = math.degrees(final_err_rad)

            # 切向控制 (PID) - 控制绕球旋转的速度
            if self.is_in_deadband and abs(angle_err_deg) < 10.0:
                v_tan = 0.0  # 在死区内且角度误差小，停止切向运动
            else:
                v_tan = max(-0.3, min(0.3, KP_THETA * final_err_rad * curr_dist))
            # ==========================================

        self.last_debug_err = (dist_err, angle_err_deg)

        # 4. 径向控制 - 控制接近或远离目标的速度
        v_rad = 0.0
        if not self.is_in_deadband:
            if abs(dist_err) < DIST_SOFT_ZONE:
                # 软区间：使用慢速增益
                v_rad = -KP_DIST_SLOW * dist_err
                # 摩擦前馈补偿：克服静摩擦力
                if abs(v_rad) > 0.001: v_rad += math.copysign(FRICTION_FEEDFORWARD, v_rad)
            else:
                # 大误差：使用快速增益
                v_rad = -KP_DIST_FAST * dist_err

            # 堵转保护：检测是否被卡住
            check_effort = abs(v_rad) if v_rad < 0 else 0.0  # 只在接近时检测
            if self.stall_detector.update(xt, yt, check_effort) and v_rad < 0:
                v_rad = 0.0  # 检测到堵转，停止接近
                self.mode_tag = "STALL"

        v_rad = max(-0.25, min(0.25, v_rad))  # 限制径向速度

        # 5. 速度合成 - 将径向和切向速度转换为机体坐标系的vx、vy
        th = theta_robot
        vx = v_rad * math.cos(th) - v_tan * math.sin(th)
        vy = v_rad * math.sin(th) + v_tan * math.cos(th)

        # 穿模保护：距离过近时逃逸（防止碰撞）
        if curr_dist < 0.15:
            vx, vy = 0.1 * math.cos(th), 0.1 * math.sin(th)

        self.driver.send_velocity_command(vx, vy, 0.0)

        # 6. 判定锁定状态 - 位置和角度都稳定时认为入位完成
        is_pos_stable = (not is_virtual_ball) and self.is_in_deadband and (abs(angle_err_deg) < 10.0)

        if is_pos_stable:
            if self.stable_since is None:
                self.stable_since = mono()
            elif mono() - self.stable_since > LOCK_TIME_THRESHOLD:
                self.driver.stop()
                return "LOCKED_LEFT" if side_char == "+" else "LOCKED_RIGHT"
        else:
            self.stable_since = None

        return "ADJUSTING"

    def _select_best(self, batch, cls_id, pattern=None):
        """
        从视觉检测结果中选择最佳目标
        参数:
            batch: 视觉检测结果列表
            cls_id: 目标类别ID
            pattern: 灯光模式（可选），如 "SOLID" 表示常亮
        返回:
            最佳目标字典，如果没有则返回 None
        """
        candidates = [m for m in batch if m.get('class_id') == cls_id]
        if pattern: candidates = [m for m in candidates if m.get('pattern') == pattern]
        if not candidates: return None
        # 排序优先级：未截断 > 高置信度 > 大面积
        candidates.sort(key=lambda m: (m.get('truncated', False), -m.get('conf', 0), -m.get('area', 0)))
        return candidates[0]

# ==========================================
# 4. 事件监测器
# ==========================================
class EventWatcher:
    """
    事件监测器 - 用于检测灯光信号的稳定模式
    通过统计一段时间内的灯光模式，判断是否满足触发条件
    """
    def __init__(self):
        self.hist = defaultdict(lambda: deque(maxlen=40))  # 每个类别ID的历史记录

    def ingest(self, batch):
        """
        接收视觉数据并记录
        参数:
            batch: 视觉检测结果列表
        """
        t = mono()
        for msg in batch:
            cid = int(msg.get("class_id", -1))
            pat = str(msg.get("pattern", "OFF"))
            self.hist[cid].append((t, pat))

    def stable_pattern(self, class_id: int, pattern: str, need_k: int, within_s: float) -> bool:
        """
        检测指定类别的灯光模式是否稳定
        参数:
            class_id: 类别ID（如 PURPLE=4）
            pattern: 灯光模式（如 "SOLID" 表示常亮）
            need_k: 需要的最小匹配次数
            within_s: 时间窗口（秒）
        返回:
            是否满足稳定条件
        """
        h = self.hist[class_id]
        if not h: return False
        t0 = mono() - within_s
        hits = sum(1 for (t, p) in h if t >= t0 and p == pattern)
        return hits >= need_k

# ==========================================
# 5. 主程序
# ==========================================
def main():
    # 命令行参数解析
    ap = argparse.ArgumentParser()
    ap.add_argument("--role", choices=["leader", "follower"], required=True,
                    help="机器人角色：leader（领队）或 follower（跟随者）")
    ap.add_argument("--robot-id", type=int, required=True,
                    help="机器人ID编号")
    ap.add_argument("--side", choices=["left", "right"], default=None,
                    help="预设入位侧（left/right），可选")
    ap.add_argument("--vision-endpoint", type=str, default="tcp://127.0.0.1:5555",
                    help="视觉系统ZMQ端点地址")
    ap.add_argument("--delay", type=float, default=1.2,
                    help="收到GO信号后的延迟启动时间（秒）")
    ap.add_argument("--speed", type=float, default=0.15,
                    help="搬运阶段的移动速度（米/秒）")
    ap.add_argument("--heading-deg", type=float, default=0.0,
                    help="搬运阶段的移动方向（度）")
    ap.add_argument("--move-sec", type=float, default=5.0,
                    help="搬运阶段的持续时间（秒）")
    args = ap.parse_args()

    # 初始化驱动器
    light = LightDriver(args.robot_id)  # 灯光驱动器
    base = RobotDriver(robot_id=args.robot_id, ros_topic="/robot/velcmd")  # 底盘驱动器

    # 初始化ZMQ订阅器（接收视觉数据）
    ctx = zmq.Context.instance()
    sock = ctx.socket(zmq.SUB)
    sock.connect(args.vision_endpoint)
    sock.setsockopt_string(zmq.SUBSCRIBE, "perception")

    # 初始化控制器
    pilot = FormationPilot(base)  # 编队入位驾驶仪
    watcher = EventWatcher()  # 事件监测器

    # 状态机初始化
    state = "UNKNOWN"  # 当前状态
    my_side = args.side  # 记录入位侧（left/right）

    # 根据角色设置初始状态
    if args.role == "leader":
        state = "WAIT_FORM"  # 领队：等待队友就位
        light.set_cmd("LEADER_WAIT")  # 红灯常亮
        log("我是 Leader，等待队友就位 (红灯常亮)...")
    else:
        state = "DOCKING"  # 跟随者：开始入位
        light.set_cmd("OFF")  # 关闭灯光
        log(f"我是 Follower，开始寻找 (Vision: {args.vision_endpoint})")

    # 时间戳和状态变量
    t_state = mono()  # 状态切换时间戳
    t_move_start = None  # 搬运开始时间
    vx, vy = heading_to_vxy(args.speed, args.heading_deg)  # 搬运阶段的速度分量

    armed = False  # 是否已武装（准备搬运）
    t_armed_start = None  # 武装开始时间
    last_debug_log_t = 0.0  # 上次调试日志时间
    
    try:
        dt = 0.03  # 控制循环周期（秒）
        while True:
            tick_start = mono()

            # 1. 接收数据 (修复版：只取最新一帧，防止积压导致的状态误判)
            batch = []
            latest_payload = None

            # 循环读空 ZMQ 缓冲区，但只保留最后一次收到的包
            while True:
                try:
                    s = sock.recv_string(flags=zmq.NOBLOCK)
                    _, payload_str = s.split(" ", 1)
                    parsed_json = json.loads(payload_str)

                    # 兼容性处理
                    if 'objects' in parsed_json:
                        latest_payload = parsed_json['objects']  # 新版打包格式
                    else:
                        latest_payload = [parsed_json]  # 旧版格式包装成 list

                except zmq.Again:
                    break
                except Exception as e:
                    pass

            # 如果这一轮循环收到了数据，使用最新的一帧
            if latest_payload is not None:
                batch = latest_payload

            watcher.ingest(batch)

            # [DEBUG] 调试日志输出
            if mono() - last_debug_log_t > 1.0:
                seen_ids = sorted(list(set([m.get('class_id') for m in batch])))
                debug_msg = f"[DEBUG] State: {state:10s} | ID: {seen_ids}"
                if state == "DOCKING":
                    p_ball = pilot.kf_ball.get_state()
                    err_dist, err_ang = pilot.last_debug_err
                    ball_str = f"({p_ball[0]:.2f}, {p_ball[1]:.2f})" if p_ball else "❌"
                    intent = pilot.latest_side_intent if pilot.latest_side_intent else "?"
                    debug_msg += f" | {pilot.mode_tag} | 🏀{ball_str} | Err:{err_dist:.2f}m | Intent:{intent}"
                print(debug_msg)
                last_debug_log_t = mono()

            # 2. 状态机
            if state == "DOCKING":
                # 跟随者入位状态
                pilot_status = pilot.update(batch)

                if pilot_status == "ADJUSTING":
                    # 根据选边意图设置灯光
                    if pilot.latest_side_intent == "+":
                        light.set_cmd("BID_LEFT")  # 绿闪
                    elif pilot.latest_side_intent == "-":
                        light.set_cmd("BID_RIGHT") # 蓝闪
                    else:
                        light.set_cmd("SEARCH")    # 紫闪

                elif pilot_status == "LOCKED_LEFT":
                    my_side = "left"
                    state = "WAIT_GO_SIGNAL"
                    light.set_cmd("LOCK_LEFT")    # 绿常亮
                    log(">>> 入位完成: 左侧 (LOCK_LEFT) <<<")

                elif pilot_status == "LOCKED_RIGHT":
                    my_side = "right"
                    state = "WAIT_GO_SIGNAL"
                    light.set_cmd("LOCK_RIGHT")   # 蓝常亮
                    log(">>> 入位完成: 右侧 (LOCK_RIGHT) <<<")

                elif pilot_status == "LOST":
                    light.set_cmd("OFF")

            elif state == "WAIT_GO_SIGNAL":
                # 跟随者等待GO信号
                # 灯光已在 LOCKED 时设置，等待 GO 信号
                if watcher.stable_pattern(PURPLE, "SOLID", 3, 0.4):
                    state = "ARMED"
                    light.set_cmd("FOLLOWER_PUSH")  # 变灰常亮
                    armed = True
                    t_armed_start = mono() + args.delay
                    log(f"收到 GO 信号 (紫常亮)，{args.delay}s 后启动")

            elif state == "WAIT_FORM":
                # 领队等待队友就位
                left_ready = watcher.stable_pattern(GREEN, "SOLID", 3, 0.6)
                right_ready = watcher.stable_pattern(BLUE, "SOLID", 3, 0.6)

                if left_ready and right_ready:
                    state = "PREWARM"
                    t_state = mono()
                    light.set_cmd("LEADER_GO")
                    log("队友已就位，进入预热 (Prewarm/GO)...")

            elif state == "PREWARM":
                # 预热阶段（给跟随者反应时间）
                light.set_cmd("LEADER_GO")
                if mono() - t_state > 2.0:
                    state = "WAIT_GO_LOCAL"
                    log("预热结束，准备出发")

            elif state == "WAIT_GO_LOCAL":
                # 等待本地GO信号
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
                # 武装状态（延迟等待）
                if mono() >= t_armed_start:
                    state = "RUN"
                    t_move_start = mono()
                    log(">>> 开始搬运 (RUN) <<<")

            elif state == "RUN":
                # 搬运阶段（按设定速度和方向移动）
                base.send_velocity_command(vx, vy, 0.0)
                if mono() - t_move_start >= args.move_sec:
                    state = "DONE"
                    base.stop()
                    log("搬运完成 (DONE)")

            elif state == "DONE":
                # 完成状态（停止运动）
                base.stop()

            # 控制循环频率
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