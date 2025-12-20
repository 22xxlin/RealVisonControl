#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
transport_node.py - 多机器人协同搬运控制节点
单文件、求稳版：状态机 + 固定周期循环 + 超时安全 + 关键日志

每台车运行：
  1) python3 vision_pub_v3_3.py  # 视觉识别节点
  2) python3 transport_node.py --role leader/follower ...  # 控制节点

同步起步（工业常用思路）：
  看到 GO(紫SOLID) 事件 -> 记录 t_detect -> t_start=t_detect+delay -> 到点再发速度
  目的：补偿各车视觉识别的时间差，实现精确同步启动
"""

import argparse
import json
import math
import time
from collections import defaultdict, deque

import zmq

from light_driver import LightDriver  # 灯光控制驱动
from robot_driver import RobotDriver  # 机器人底盘驱动

# === 颜色类别ID，与 vision_pub_v3_3.py 的 CLS_MAP 对齐 ===
# 蓝色=右侧车，红色=Leader锚点，绿色=左侧车，紫色=GO信号，灰色=搬运中
BLUE, RED, GREEN, PURPLE, GRAY = 1, 2, 3, 4, 5


def mono() -> float:
    """返回单调时间戳（秒），用于计时不受系统时间调整影响"""
    return time.monotonic()


def log(msg: str):
    """带时间戳的日志输出，便于调试和记录"""
    ts = time.strftime("%H:%M:%S")
    print(f"[{ts}] {msg}", flush=True)


def heading_to_vxy(speed: float, heading_deg: float):
    """
    将速度和航向角转换为机器人坐标系的 vx, vy
    0°=前进(+x), 90°=向左(+y), 180°=后退, 270°=向右
    """
    rad = math.radians(heading_deg)
    return speed * math.cos(rad), speed * math.sin(rad)


class PerceptionWatcher:
    """
    感知监听器：订阅本机 vision_pub_v3_3.py 的 ZMQ topic: perception
    功能：
    1. 接收视觉识别的灯光信号（颜色+闪烁模式）
    2. 维护每个颜色类别的历史记录（最近40条）
    3. 提供去抖判稳功能，避免光线变化、遮挡等导致的误触发
    """
    def __init__(self, endpoint: str, topic: str = "perception"):
        self.endpoint = endpoint
        self.topic = topic
        # 创建 ZMQ 订阅者套接字，连接到视觉节点
        self.ctx = zmq.Context.instance()
        self.sock = self.ctx.socket(zmq.SUB)
        self.sock.connect(endpoint)
        self.sock.setsockopt_string(zmq.SUBSCRIBE, topic)

        self.last_rx_t = None  # 最后一次接收消息的时间戳
        # 每个颜色类别的历史记录：class_id -> deque[(时间, 模式)]
        self.hist = defaultdict(lambda: deque(maxlen=40))

    def poll(self, budget_ms: int = 20):
        """
        轮询接收消息，在指定时间预算内尽可能多地接收
        budget_ms: 轮询时间预算（毫秒）
        """
        deadline = mono() + budget_ms / 1000.0
        while mono() < deadline:
            try:
                # 非阻塞接收，避免卡住主循环
                s = self.sock.recv_string(flags=zmq.NOBLOCK)
            except zmq.Again:
                break  # 没有更多消息了

            try:
                # 解析消息格式："topic payload"
                topic, payload = s.split(" ", 1)
                if topic != self.topic:
                    continue
                msg = json.loads(payload)
                cid = int(msg.get("class_id", -1))  # 颜色类别ID
                pat = str(msg.get("pattern", "OFF"))  # 闪烁模式（SOLID/BLINK/OFF）
                t = mono()
                self.last_rx_t = t
                self.hist[cid].append((t, pat))  # 记录到历史队列
            except Exception:
                continue  # 解析失败，跳过该消息

    def perception_age(self) -> float:
        """
        返回距离上次接收消息的时间（秒）
        用于检测感知超时：如果太久没收到数据，说明视觉节点可能挂了
        """
        if self.last_rx_t is None:
            return float("inf")
        return mono() - self.last_rx_t

    def stable_pattern(self, class_id: int, pattern: str, need_k: int, within_s: float) -> bool:
        """
        去抖判稳：判断某个颜色的某个模式是否稳定出现
        class_id: 颜色类别ID（如 PURPLE=4）
        pattern: 期望的模式（如 "SOLID" 常亮）
        need_k: 需要命中的次数阈值（如 3次）
        within_s: 时间窗口（秒，如 0.4秒）
        返回：在 within_s 秒内，该模式出现次数 >= need_k，则认为稳定
        """
        h = self.hist[class_id]
        if not h:
            return False
        t0 = mono() - within_s
        hits = sum(1 for (t, p) in h if t >= t0 and p == pattern)
        return hits >= need_k


class SyncStart:
    """
    同步启动器：事件触发 + 延迟执行
    工作流程：
    1. 检测到 GO 信号 -> arm（上膛）-> 记录 t_detect
    2. 计算 t_start = t_detect + delay_s
    3. 等到 mono() >= t_start 时，ready_to_start() 返回 True
    4. 主循环检测到 ready 后，开始发送速度指令
    
    目的：补偿各车视觉识别的时间差，实现精确同步
    """
    def __init__(self, delay_s: float):
        self.delay_s = delay_s  # 延迟时间（秒）
        self.armed = False  # 是否已上膛
        self.t_detect = None  # 检测到GO信号的时间
        self.t_start = None  # 计划启动的时间

    def arm_if_go(self, go: bool):
        """
        如果检测到GO信号且尚未上膛，则上膛并记录时间
        go: 是否检测到GO信号（紫色常亮）
        """
        if self.armed:
            return  # 已经上膛了，不重复
        if go:
            self.armed = True
            self.t_detect = mono()
            self.t_start = self.t_detect + self.delay_s

    def ready_to_start(self) -> bool:
        """判断是否到达启动时刻"""
        return self.armed and mono() >= self.t_start

    def remaining(self) -> float:
        """返回剩余等待时间（秒）"""
        if not self.armed:
            return float("inf")
        return max(0.0, self.t_start - mono())


def main():
    # === 命令行参数解析 ===
    ap = argparse.ArgumentParser()
    # 角色：leader（领导车）或 follower（跟随车）
    ap.add_argument("--role", choices=["leader", "follower"], required=True)
    # 侧面：follower 需要指定是左侧还是右侧
    ap.add_argument("--side", choices=["left", "right"], default=None, help="follower 需要指定 left/right")
    # 机器人ID
    ap.add_argument("--robot-id", type=int, required=True)

    # 视觉节点的ZMQ端点
    ap.add_argument("--vision-endpoint", type=str, default="tcp://127.0.0.1:5555")
    # 主循环频率（Hz）
    ap.add_argument("--loop-hz", type=float, default=50.0)
    # 感知超时阈值（秒），超过该时间没收到数据就停车
    ap.add_argument("--perception-timeout", type=float, default=0.5, help="超过该时间没收到 perception 就停车/abort")

    # 去抖参数（求稳默认）
    ap.add_argument("--stable-within", type=float, default=0.4, help="判稳窗口(s)")
    ap.add_argument("--stable-k", type=int, default=3, help="窗口内命中次数阈值")

    # 同步与运动参数
    ap.add_argument("--delay", type=float, default=1.2, help="GO 后延迟 Δ(s) 再起步；对照组用 0.0")
    ap.add_argument("--prewarm", type=float, default=2.0, help="leader: SEARCH 紫闪预热(s)")
    ap.add_argument("--speed", type=float, default=0.25, help="平移速度(m/s)")
    ap.add_argument("--heading-deg", type=float, default=0.0, help="机身坐标系航向角(度): 0前 90左")
    ap.add_argument("--omega", type=float, default=0.0, help="自转(rad/s)")
    ap.add_argument("--move-sec", type=float, default=5.0, help="搬运时长(s)")

    args = ap.parse_args()

    # follower 必须指定侧面
    if args.role == "follower" and args.side is None:
        raise SystemExit("follower 必须加 --side left 或 --side right")

    log(
        f"START role={args.role} side={args.side} robot_id={args.robot_id} "
        f"vision={args.vision_endpoint} delay={args.delay} prewarm={args.prewarm} "
        f"speed={args.speed} heading_deg={args.heading_deg} omega={args.omega} move_sec={args.move_sec} "
        f"loop_hz={args.loop_hz} perception_timeout={args.perception_timeout} "
        f"stable_k={args.stable_k} stable_within={args.stable_within}"
    )

    # === 初始化驱动 ===
    light = LightDriver(args.robot_id)  # 灯光控制
    base = RobotDriver(robot_id=args.robot_id, ros_topic="/robot/velcmd")  # 底盘控制
    percep = PerceptionWatcher(args.vision_endpoint)  # 感知监听
    starter = SyncStart(delay_s=args.delay)  # 同步启动器

    # 计算循环周期和速度分量
    dt = 1.0 / max(1e-6, args.loop_hz)
    vx, vy = heading_to_vxy(args.speed, args.heading_deg)

    # === 状态机初始化 ===
    state = "WAIT_FORM"  # 初始状态：等待编队
    t_state = mono()  # 进入当前状态的时间
    t_move_start = None  # 开始移动的时间
    last_go_seen = False  # 上次是否看到GO信号（用于边沿检测）
    last_abort_log_t = 0.0  # 上次打印ABORT日志的时间（防止刷屏）

    def set_state(new_state: str, reason: str = ""):
        """状态切换函数，带日志输出"""
        nonlocal state, t_state
        if new_state != state:
            log(f"STATE {state} -> {new_state}" + (f" ({reason})" if reason else ""))
        state = new_state
        t_state = mono()

    try:
        # === 初始灯态设置 ===
        if args.role == "leader":
            light.set_cmd("LEADER_WAIT")  # 红常亮，作为锚点让follower识别
        else:
            # follower 亮绿（左）或蓝（右）常亮，表示就位
            light.set_cmd("LOCK_LEFT" if args.side == "left" else "LOCK_RIGHT")

        # === 主循环：固定周期状态机 ===
        while True:
            tick_start = mono()
            # 轮询接收感知消息
            percep.poll(int(dt * 1000))

            # --- 超时安全：感知断了就停车并进入 ABORT ---
            age = percep.perception_age()
            if age > args.perception_timeout:
                base.stop()
                light.set_cmd("OFF")
                # 不要疯狂刷屏：每 1 秒打一条
                if mono() - last_abort_log_t > 1.0 and state != "DONE":
                    log(f"ABORT: perception timeout age={age:.3f}s > {args.perception_timeout}s")
                    last_abort_log_t = mono()
                set_state("ABORT", "perception timeout")
                # 控频后继续循环
                elapsed = mono() - tick_start
                if elapsed < dt:
                    time.sleep(dt - elapsed)
                continue

            # --- 事件判定（去抖） ---
            # GO信号：紫色常亮，稳定出现3次以上
            go_seen = percep.stable_pattern(PURPLE, "SOLID", need_k=args.stable_k, within_s=args.stable_within)
            # 左侧就位：绿色常亮
            left_ready = percep.stable_pattern(GREEN, "SOLID", need_k=args.stable_k, within_s=0.6)
            # 右侧就位：蓝色常亮
            right_ready = percep.stable_pattern(BLUE, "SOLID", need_k=args.stable_k, within_s=0.6)

            # GO 事件边沿检测（只打印一次）
            if go_seen and not last_go_seen:
                log("EVENT GO detected: PURPLE SOLID (debounced)")
            last_go_seen = go_seen

            # === 状态机逻辑 ===
            if state == "WAIT_FORM":
                # 等待编队完成
                if args.role == "leader":
                    # leader 等左右就位（绿SOLID + 蓝SOLID）
                    if left_ready and right_ready:
                        light.set_cmd("SEARCH")  # 紫闪预热，提醒follower准备
                        set_state("PREWARM", "left&right SOLID")
                else:
                    # follower 等 GO 信号
                    if go_seen:
                        light.set_cmd("FOLLOWER_PUSH")  # 灰常亮确认进入搬运
                        starter.arm_if_go(True)  # 上膛
                        set_state("ARMED", "GO seen -> arm")

            elif state == "PREWARM":
                # leader 紫闪一段时间后切 GO
                if mono() - t_state >= args.prewarm:
                    light.set_cmd("LEADER_GO")  # 紫常亮 GO 事件
                    set_state("WAIT_GO_LOCAL", "prewarm done -> send GO")

            elif state == "WAIT_GO_LOCAL":
                # leader 自己也要"看到紫常亮"才 arm（与 follower 同逻辑）
                # 这样可以补偿自己的视觉识别延迟
                starter.arm_if_go(go_seen)
                if starter.armed:
                    set_state("ARMED", f"leader armed (delay={args.delay})")

            elif state == "ARMED":
                # 已上膛，等待延迟时间到达
                starter.arm_if_go(go_seen)  # 幂等操作
                if starter.ready_to_start():
                    t_move_start = mono()
                    set_state("RUN", f"start now (delay={args.delay})")

            elif state == "RUN":
                # 运行状态：固定周期发速度指令
                base.send_velocity_command(vx, vy, args.omega)
                if (mono() - t_move_start) >= args.move_sec:
                    base.stop()
                    set_state("DONE", "move complete")

            elif state == "DONE":
                # 完成状态：保持停车
                base.stop()
                # 保持灯态不动（leader: 紫常亮；follower: 灰常亮）
                elapsed = mono() - tick_start
                if elapsed < dt:
                    time.sleep(dt - elapsed)
                continue

            elif state == "ABORT":
                # 异常状态：停车等待
                base.stop()
                elapsed = mono() - tick_start
                if elapsed < dt:
                    time.sleep(dt - elapsed)
                continue

            # === 控频：保证固定周期 ===
            elapsed = mono() - tick_start
            if elapsed < dt:
                time.sleep(dt - elapsed)

    except KeyboardInterrupt:
        log("KeyboardInterrupt -> stop")
    finally:
        # 清理：停车、关灯
        base.stop()
        light.set_cmd("OFF")
        try:
            light.stop()
        except Exception:
            pass
        log("EXIT")


if __name__ == "__main__":
    main()

