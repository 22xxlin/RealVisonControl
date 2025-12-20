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
        log(f"[DEBUG] ZMQ SUB socket 已连接，等待接收消息...")

        # 测试连接（非阻塞）
        try:
            test_msg = self.sock.recv_string(flags=zmq.NOBLOCK)
            log(f"[DEBUG] 立即收到消息: {test_msg[:100]}")
        except zmq.Again:
            log(f"[DEBUG] 暂无消息（正常，等待 vision_pub 发送）")

    def poll(self, budget_ms: int = 20):
        deadline = mono() + budget_ms / 1000.0
        msgs_this_poll = 0
        while mono() < deadline:
            try:
                s = self.sock.recv_string(flags=zmq.NOBLOCK)
                msgs_this_poll += 1
            except zmq.Again:
                break

            try:
                topic, payload = s.split(" ", 1)
                if topic != self.topic:
                    log(f"[DEBUG] 收到非目标topic: {topic}, 期望: {self.topic}")
                    continue
                msg = json.loads(payload)
                cid = int(msg.get("class_id", -1))
                pat = str(msg.get("pattern", "OFF"))
                dist = msg.get("distance", -1)
                bearing = msg.get("bearing_body", -1)
                t = mono()
                self.last_rx_t = t
                self.hist[cid].append((t, pat))
                self.total_msgs_received += 1

                # 映射 class_id 到名称
                CLS_NAMES = {1: "BLUE", 2: "RED", 3: "GREEN", 4: "PURPLE", 5: "GRAY", 6: "BALL", 7: "FLAG"}
                cls_name = CLS_NAMES.get(cid, f"UNK({cid})")

                # 每秒打印一次接收统计
                if mono() - self.last_debug_log_t > 1.0:
                    log(f"[DEBUG] 已接收 {self.total_msgs_received} 条消息")
                    log(f"[DEBUG] 最新: {cls_name} {pat} @ {dist:.2f}m, {bearing:.1f}°")
                    log(f"[DEBUG] perception_age = {self.perception_age():.3f}s")

                    # 显示最近检测到的所有颜色
                    recent_colors = set()
                    now = mono()
                    for color_id in [1, 2, 3, 4, 5]:  # BLUE, RED, GREEN, PURPLE, GRAY
                        if color_id in self.hist:
                            recent = [t for t, p in self.hist[color_id] if now - t < 2.0]
                            if recent:
                                recent_colors.add(CLS_NAMES[color_id])
                    if recent_colors:
                        log(f"[DEBUG] 最近2秒内检测到的颜色: {', '.join(sorted(recent_colors))}")

                    self.last_debug_log_t = mono()
            except Exception as e:
                log(f"[DEBUG] 解析消息失败: {e}, 原始消息: {s[:100]}")
                continue

        # 如果这次poll收到了消息，打印一下
        if msgs_this_poll > 0:
            pass  # 已经在上面打印了

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
    ap.add_argument("--speed", type=float, default=0.15, help="平移速度(m/s)")
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

    # 调试：记录主循环开始时间
    log(f"[DEBUG] 主循环即将开始，等待 vision_pub 消息...")
    log(f"[DEBUG] 连接到: {args.vision_endpoint}")
    last_no_msg_warn_t = 0.0

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

        t_start_program = mono()  # 记录程序启动时间

        while True:
            tick_start = mono()
            percep.poll(int(dt * 1000))

            # --- 调试：定期检查是否收到消息 ---
            if mono() - last_no_msg_warn_t > 3.0:
                age = percep.perception_age()
                if percep.total_msgs_received == 0:
                    log(f"[DEBUG] ⚠️ 警告：尚未收到任何 vision_pub 消息！")
                    log(f"[DEBUG] 请检查:")
                    log(f"[DEBUG]   1. vision_pub.py 是否正在运行？")
                    log(f"[DEBUG]   2. 端口是否正确？当前连接: {args.vision_endpoint}")
                    log(f"[DEBUG]   3. ZMQ topic 是否匹配？当前订阅: perception")
                else:
                    log(f"[DEBUG] ✅ 已接收 {percep.total_msgs_received} 条消息, age={age:.3f}s, state={state}")
                last_no_msg_warn_t = mono()

            # --- 超时安全：感知断了就停车并进入 ABORT ---
            # 给5秒宽容期，允许视觉节点晚启动
            age = percep.perception_age()
            if age > args.perception_timeout and (mono() - t_start_program) > 5.0:
                base.stop()
                # 注意：不关灯！灯是给其他车看的，必须保持初始状态
                # light.set_cmd("OFF")  # 注释掉，保持初始灯态
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
                        # 调试：显示等待状态
                        if mono() - last_no_msg_warn_t > 5.0:
                            log(f"[DEBUG] Leader 等待编队: GREEN={'✅' if left_ready else '❌'}, BLUE={'✅' if right_ready else '❌'}")
                else:
                    # follower 等 GO
                    if go_seen:
                        light.set_cmd("FOLLOWER_PUSH")  # 灰常亮确认进入搬运
                        starter.arm_if_go(True)
                        set_state("ARMED", "GO seen -> arm")
                    else:
                        # 调试：显示等待状态
                        if mono() - last_no_msg_warn_t > 5.0:
                            log(f"[DEBUG] Follower 等待 GO 信号: PURPLE={'✅' if go_seen else '❌ 未检测到'}")

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
        log("正在清理资源...")
        try:
            base.stop()
        except Exception as e:
            log(f"base.stop() 失败: {e}")

        try:
            light.set_cmd("OFF")
        except Exception as e:
            log(f"light.set_cmd(OFF) 失败: {e}")

        # light.stop() 可能卡住，使用超时保护
        import signal
        def timeout_handler(_signum, _frame):
            raise TimeoutError("light.stop() 超时")

        try:
            signal.signal(signal.SIGALRM, timeout_handler)
            signal.alarm(2)  # 2秒超时
            light.stop()
            signal.alarm(0)  # 取消超时
        except TimeoutError:
            log("light.stop() 超时，强制退出")
        except Exception as e:
            log(f"light.stop() 失败: {e}")

        log("EXIT")


if __name__ == "__main__":
    main()
