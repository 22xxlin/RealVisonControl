#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
transport_node.py
修复版：解决了 Leader 无法看到自己发出的 GO 信号导致不走的问题。

每台车运行：
  1) python3 vision_pub_v3_3.py
  2) python3 transport_node.py --role leader/follower ...
"""

import argparse
import json
import math
import time
from collections import defaultdict, deque

import zmq

from light_driver import LightDriver
from robot_driver import RobotDriver

# === 颜色类别ID，与 vision_pub_v3_3.py 的 CLS_MAP 对齐 ===
BLUE, RED, GREEN, PURPLE, GRAY = 1, 2, 3, 4, 5


def mono() -> float:
    """返回单调时间戳（秒），用于计时不受系统时间调整影响"""
    return time.monotonic()


def log(msg: str):
    """带时间戳的日志输出"""
    ts = time.strftime("%H:%M:%S")
    print(f"[{ts}] {msg}", flush=True)


def heading_to_vxy(speed: float, heading_deg: float):
    """
    将速度和航向角转换为机器人坐标系的 vx, vy
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

        # 调试计数器
        self.total_msgs_received = 0
        self.last_debug_log_t = 0.0

        log(f"[DEBUG] PerceptionWatcher 初始化完成: endpoint={endpoint}, topic={topic}")

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
                # dist = msg.get("distance", -1)
                # bearing = msg.get("bearing_body", -1)
                t = mono()
                self.last_rx_t = t
                self.hist[cid].append((t, pat))
                self.total_msgs_received += 1

                # 映射 class_id 到名称 (仅调试用)
                # CLS_NAMES = {1: "BLUE", 2: "RED", 3: "GREEN", 4: "PURPLE", 5: "GRAY"}
                
                # 每1秒打印一次接收统计
                if mono() - self.last_debug_log_t > 1.0:
                    # log(f"[DEBUG] 心跳: 已接收 {self.total_msgs_received} 条消息, age={self.perception_age():.3f}s")
                    self.last_debug_log_t = mono()
            except Exception as e:
                log(f"[DEBUG] 解析消息失败: {e}")
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


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--role", choices=["leader", "follower"], required=True)
    ap.add_argument("--side", choices=["left", "right"], default=None, help="follower 需要指定 left/right")
    ap.add_argument("--robot-id", type=int, required=True)

    ap.add_argument("--vision-endpoint", type=str, default="tcp://127.0.0.1:5555")
    ap.add_argument("--loop-hz", type=float, default=50.0)
    ap.add_argument("--perception-timeout", type=float, default=0.5, help="超过该时间没收到 perception 就停车/abort")

    # 去抖参数
    ap.add_argument("--stable-within", type=float, default=0.4, help="判稳窗口(s)")
    ap.add_argument("--stable-k", type=int, default=3, help="窗口内命中次数阈值")

    # 同步与运动参数
    ap.add_argument("--delay", type=float, default=1.7, help="GO 后延迟 Δ(s) 再起步")
    ap.add_argument("--prewarm", type=float, default=2.0, help="leader: SEARCH 紫闪预热(s)")
    ap.add_argument("--speed", type=float, default=0.15, help="平移速度(m/s)")
    ap.add_argument("--heading-deg", type=float, default=0.0, help="机身坐标系航向角(度): 0前 90左")
    ap.add_argument("--omega", type=float, default=0.0, help="自转(rad/s)")
    ap.add_argument("--move-sec", type=float, default=5.0, help="搬运时长(s)")

    args = ap.parse_args()

    if args.role == "follower" and args.side is None:
        raise SystemExit("follower 必须加 --side left 或 --side right")

    log(
        f"START role={args.role} side={args.side} robot_id={args.robot_id} "
        f"vision={args.vision_endpoint} delay={args.delay} "
        f"speed={args.speed} move_sec={args.move_sec}"
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
    t_start_program = mono()

    # 调试日志频率控制
    last_debug_msg_t = 0.0

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

        log(f"Waiting for team... (Role: {args.role})")

        while True:
            tick_start = mono()
            percep.poll(int(dt * 1000))

            # --- 超时安全 ---
            age = percep.perception_age()
            # 启动前5秒不杀，给视觉一点启动时间
            if age > args.perception_timeout and (mono() - t_start_program) > 5.0:
                base.stop()
                if mono() - last_abort_log_t > 1.0 and state != "DONE":
                    log(f"ABORT: perception timeout age={age:.3f}s")
                    last_abort_log_t = mono()
                set_state("ABORT", "perception timeout")
                
                # 简单控频
                elapsed = mono() - tick_start
                if elapsed < dt:
                    time.sleep(dt - elapsed)
                continue

            # --- 事件判定 ---
            go_seen = percep.stable_pattern(PURPLE, "SOLID", need_k=args.stable_k, within_s=args.stable_within)
            left_ready = percep.stable_pattern(GREEN, "SOLID", need_k=args.stable_k, within_s=0.6)
            right_ready = percep.stable_pattern(BLUE, "SOLID", need_k=args.stable_k, within_s=0.6)

            # 调试：显示GO信号状态
            if go_seen and not last_go_seen:
                log("EVENT GO detected: PURPLE SOLID")
            last_go_seen = go_seen

            # === 状态机逻辑 ===

            if state == "WAIT_FORM":
                if args.role == "leader":
                    # leader 等左右就位
                    if left_ready and right_ready:
                        light.set_cmd("SEARCH")  # 紫闪预热
                        set_state("PREWARM", "left&right SOLID detected")
                    else:
                        # 偶尔打印一下等待状态
                        if mono() - last_debug_msg_t > 2.0:
                            log(f"[DEBUG] Leader Waiting: Left(G)={left_ready}, Right(B)={right_ready}")
                            last_debug_msg_t = mono()
                else:
                    # follower 等 GO
                    if go_seen:
                        light.set_cmd("FOLLOWER_PUSH")  # 灰常亮确认
                        starter.arm_if_go(True)
                        set_state("ARMED", "GO seen -> arm")

            elif state == "PREWARM":
                # leader 紫闪预热
                if mono() - t_state >= args.prewarm:
                    light.set_cmd("LEADER_GO")  # 紫常亮
                    set_state("WAIT_GO_LOCAL", "prewarm done -> send GO")

            elif state == "WAIT_GO_LOCAL":
                # === 关键修复点 ===
                if args.role == "leader":
                    # Leader 不需要看视觉（看不到自己的紫灯），直接 Arm！
                    starter.arm_if_go(True)
                else:
                    # Follower 需要看视觉（这里是为了代码鲁棒性，实际上Follower一般直接从WAIT_FORM跳走）
                    starter.arm_if_go(go_seen)

                if starter.armed:
                    set_state("ARMED", f"Armed! Starting countdown (delay={args.delay}s)")

            elif state == "ARMED":
                # 再次检查 go_seen 是为了防止信号闪烁？这里如果是 Leader 就不需要检查了
                # SyncStart 内部逻辑是 armed 之后只看时间，不看 go 了
                # 但为了 Follower 被干扰导致信号中断，可以保留 arm_if_go(go_seen) 来刷新 t_detect?
                # 不，SyncStart 的 arm_if_go 有 guards，一旦 armed 就不再变 t_detect
                
                if starter.ready_to_start():
                    t_move_start = mono()
                    set_state("RUN", "Countdown finished -> MOVE")

            elif state == "RUN":
                base.send_velocity_command(vx, vy, args.omega)
                if (mono() - t_move_start) >= args.move_sec:
                    base.stop()
                    set_state("DONE", "Movement duration complete")

            elif state == "DONE":
                base.stop()
                # 任务完成，保持停止，保持灯光（Leader紫/Follower灰）
                pass

            elif state == "ABORT":
                base.stop()
                pass

            # 控频
            elapsed = mono() - tick_start
            if elapsed < dt:
                time.sleep(dt - elapsed)

    except KeyboardInterrupt:
        log("KeyboardInterrupt -> stop")
    finally:
        base.stop()
        light.set_cmd("OFF")
        light.stop()
        log("EXIT")


if __name__ == "__main__":
    main()