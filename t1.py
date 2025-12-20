#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
transport_node.py
单文件、求稳版：状态机 + 固定周期循环 + 超时安全 + 关键日志

每台车运行：
  1) python3 vision_pub_v3_3.py
  2) python3 transport_node.py --role leader/follower ...

同步起步（工业常用思路）：
  看到 GO(紫SOLID) 事件 -> 记录 t_detect -> t_start=t_detect+delay -> 到点再发速度
"""

import argparse
import json
import math
import time
from collections import defaultdict, deque

import zmq

from light_driver import LightDriver
from robot_driver import RobotDriver

# === class_id 对齐 vision_pub_v3_3.py 的 CLS_MAP ===
BLUE, RED, GREEN, PURPLE, GRAY = 1, 2, 3, 4, 5


def mono() -> float:
    return time.monotonic()


def log(msg: str):
    ts = time.strftime("%H:%M:%S")
    print(f"[{ts}] {msg}", flush=True)


def heading_to_vxy(speed: float, heading_deg: float):
    """
    0°=前进(+x), 90°=向左(+y)
    """
    rad = math.radians(heading_deg)
    return speed * math.cos(rad), speed * math.sin(rad)


class PerceptionWatcher:
    """
    订阅本机 vision_pub_v3_3.py 的 ZMQ topic: perception
    只做“求稳的事件判定”：维护每个 class_id 最近 pattern 历史并提供去抖判稳。
    """
    def __init__(self, endpoint: str, topic: str = "perception"):
        self.endpoint = endpoint
        self.topic = topic
        self.ctx = zmq.Context.instance()
        self.sock = self.ctx.socket(zmq.SUB)
        self.sock.connect(endpoint)
        self.sock.setsockopt_string(zmq.SUBSCRIBE, topic)

        self.last_rx_t = None
        self.hist = defaultdict(lambda: deque(maxlen=40))  # class_id -> deque[(t, pattern)]

    def poll(self, budget_ms: int = 20):
        deadline = mono() + budget_ms / 1000.0
        while mono() < deadline:
            try:
                s = self.sock.recv_string(flags=zmq.NOBLOCK)
            except zmq.Again:
                break

            try:
                topic, payload = s.split(" ", 1)
                if topic != self.topic:
                    continue
                msg = json.loads(payload)
                cid = int(msg.get("class_id", -1))
                pat = str(msg.get("pattern", "OFF"))
                t = mono()
                self.last_rx_t = t
                self.hist[cid].append((t, pat))
            except Exception:
                continue

    def perception_age(self) -> float:
        if self.last_rx_t is None:
            return float("inf")
        return mono() - self.last_rx_t

    def stable_pattern(self, class_id: int, pattern: str, need_k: int, within_s: float) -> bool:
        """
        最近 within_s 秒内，pattern 命中次数 >= need_k
        """
        h = self.hist[class_id]
        if not h:
            return False
        t0 = mono() - within_s
        hits = sum(1 for (t, p) in h if t >= t0 and p == pattern)
        return hits >= need_k


class SyncStart:
    """
    事件触发 + 时间戳执行：
    看到 GO -> t_start = t_detect + delay_s -> 到点才开始发速度
    """
    def __init__(self, delay_s: float):
        self.delay_s = delay_s
        self.armed = False
        self.t_detect = None
        self.t_start = None

    def arm_if_go(self, go: bool):
        if self.armed:
            return
        if go:
            self.armed = True
            self.t_detect = mono()
            self.t_start = self.t_detect + self.delay_s

    def ready_to_start(self) -> bool:
        return self.armed and mono() >= self.t_start

    def remaining(self) -> float:
        if not self.armed:
            return float("inf")
        return max(0.0, self.t_start - mono())


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--role", choices=["leader", "follower"], required=True)
    ap.add_argument("--side", choices=["left", "right"], default=None, help="follower 需要指定 left/right")
    ap.add_argument("--robot-id", type=int, required=True)

    ap.add_argument("--vision-endpoint", type=str, default="tcp://127.0.0.1:5555")
    ap.add_argument("--loop-hz", type=float, default=50.0)
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

    if args.role == "follower" and args.side is None:
        raise SystemExit("follower 必须加 --side left 或 --side right")

    log(
        f"START role={args.role} side={args.side} robot_id={args.robot_id} "
        f"vision={args.vision_endpoint} delay={args.delay} prewarm={args.prewarm} "
        f"speed={args.speed} heading_deg={args.heading_deg} omega={args.omega} move_sec={args.move_sec} "
        f"loop_hz={args.loop_hz} perception_timeout={args.perception_timeout} "
        f"stable_k={args.stable_k} stable_within={args.stable_within}"
    )

    # drivers
    light = LightDriver(args.robot_id)
    base = RobotDriver(robot_id=args.robot_id, ros_topic="/robot/velcmd")
    percep = PerceptionWatcher(args.vision_endpoint)
    starter = SyncStart(delay_s=args.delay)

    dt = 1.0 / max(1e-6, args.loop_hz)
    vx, vy = heading_to_vxy(args.speed, args.heading_deg)

    # 状态机
    state = "WAIT_FORM"
    t_state = mono()
    t_move_start = None
    last_go_seen = False
    last_abort_log_t = 0.0

    def set_state(new_state: str, reason: str = ""):
        nonlocal state, t_state
        if new_state != state:
            log(f"STATE {state} -> {new_state}" + (f" ({reason})" if reason else ""))
        state = new_state
        t_state = mono()

    try:
        # 初始灯态
        if args.role == "leader":
            light.set_cmd("LEADER_WAIT")  # 红常亮锚点
        else:
            light.set_cmd("LOCK_LEFT" if args.side == "left" else "LOCK_RIGHT")  # 绿/蓝常亮

        while True:
            tick_start = mono()
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
                # 控频
                elapsed = mono() - tick_start
                if elapsed < dt:
                    time.sleep(dt - elapsed)
                continue

            # --- 事件判定（去抖） ---
            go_seen = percep.stable_pattern(PURPLE, "SOLID", need_k=args.stable_k, within_s=args.stable_within)
            left_ready = percep.stable_pattern(GREEN, "SOLID", need_k=args.stable_k, within_s=0.6)
            right_ready = percep.stable_pattern(BLUE, "SOLID", need_k=args.stable_k, within_s=0.6)

            # GO 事件边沿打印（只打印一次）
            if go_seen and not last_go_seen:
                log("EVENT GO detected: PURPLE SOLID (debounced)")
            last_go_seen = go_seen

            if state == "WAIT_FORM":
                if args.role == "leader":
                    # leader 等左右就位（绿SOLID + 蓝SOLID）
                    if left_ready and right_ready:
                        light.set_cmd("SEARCH")  # 紫闪预热
                        set_state("PREWARM", "left&right SOLID")
                else:
                    # follower 等 GO
                    if go_seen:
                        light.set_cmd("FOLLOWER_PUSH")  # 灰常亮确认进入搬运
                        starter.arm_if_go(True)
                        set_state("ARMED", "GO seen -> arm")

            elif state == "PREWARM":
                # leader 紫闪一段时间后切 GO
                if mono() - t_state >= args.prewarm:
                    light.set_cmd("LEADER_GO")  # 紫常亮 GO 事件
                    set_state("WAIT_GO_LOCAL", "prewarm done -> send GO")

            elif state == "WAIT_GO_LOCAL":
                # leader 自己也要“看到紫常亮”才 arm（与 follower 同逻辑）
                starter.arm_if_go(go_seen)
                if starter.armed:
                    set_state("ARMED", f"leader armed (delay={args.delay})")

            elif state == "ARMED":
                starter.arm_if_go(go_seen)  # 幂等
                if starter.ready_to_start():
                    t_move_start = mono()
                    set_state("RUN", f"start now (delay={args.delay})")

            elif state == "RUN":
                # 固定周期发速度
                base.send_velocity_command(vx, vy, args.omega)
                if (mono() - t_move_start) >= args.move_sec:
                    base.stop()
                    set_state("DONE", "move complete")

            elif state == "DONE":
                base.stop()
                # 保持灯态不动（你也可以改成 OFF）
                # leader: 紫常亮；follower: 灰常亮
                # 控频
                elapsed = mono() - tick_start
                if elapsed < dt:
                    time.sleep(dt - elapsed)
                continue

            elif state == "ABORT":
                base.stop()
                # 控频
                elapsed = mono() - tick_start
                if elapsed < dt:
                    time.sleep(dt - elapsed)
                continue

            # 控频
            elapsed = mono() - tick_start
            if elapsed < dt:
                time.sleep(dt - elapsed)

    except KeyboardInterrupt:
        log("KeyboardInterrupt -> stop")
    finally:
        base.stop()
        light.set_cmd("OFF")
        try:
            light.stop()
        except Exception:
            pass
        log("EXIT")


if __name__ == "__main__":
    main()
