#!/usr/bin/env python3
"""
控制订阅者 - 使用 ZeroMQ SUB 接收检测结果并执行控制
独立进程，替代原本的 ControlNode 线程

【架构解耦 - 决策层】
- 职责：负责"大脑决策"，将原始 Pattern 翻译成具体控制指令
- 接收 Vision 端发送的原始 Pattern（如 '2200', '110', 'IDLE'）
- 使用内部映射表 PATTERN_TO_COMMAND 进行决策
- 执行对应的运动控制函数
"""

import os
import sys
import time
import math
import json
import zmq

# 添加机器人控制模块路径
sys.path.append(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'real_ws', 'control'))

try:
    from simple_robot_control import SimpleRobotController
    ROBOT_CONTROL_AVAILABLE = True
    print("✅ 机器人控制模块加载成功")
except ImportError as e:
    print(f"⚠️ 机器人控制模块加载失败: {e}")
    print("💡 将仅输出日志，不执行机器人控制")
    ROBOT_CONTROL_AVAILABLE = False


class ControlSubscriber:
    """控制订阅者 - 使用 ZeroMQ 接收检测结果并执行控制"""
    
    def __init__(self, robot_id=8, enable_control=True, zmq_address="tcp://localhost:5555", timeout_ms=1000):
        """
        初始化控制订阅者

        Args:
            robot_id: 机器人ID
            enable_control: 是否启用实际控制
            zmq_address: ZeroMQ 订阅地址
            timeout_ms: 接收超时时间（毫秒）
        """
        self.robot_id = robot_id
        self.enable_control = enable_control and ROBOT_CONTROL_AVAILABLE
        self.zmq_address = zmq_address
        self.timeout_ms = timeout_ms

        # 【架构解耦 - 决策层】Pattern 到 Command 的映射表
        # 从 vision_pub.py 迁移过来，由 Control 端负责决策
        self.PATTERN_TO_COMMAND = {
            # 基本运动模式（3位）
            '220': 'FORWARD',   # 红红黑 -> 前进
            '330': 'LEFT',      # 绿绿黑 -> 左移
            '110': 'RIGHT',     # 蓝蓝黑 -> 右移
            '550': 'REVERSE',   # 黄黄黑 -> 后退
            '440': 'STOP',      # 紫紫黑 -> 停止

            # 高级运动模式（4位）
            '2200': 'APPROACH', # 红红黑黑 -> 靠近
            '1100': 'RETREAT',  # 蓝蓝黑黑 -> 远离
            '4400': 'S_SHAPE',  # 紫紫黑黑 -> S形轨迹
            '5500': 'CIRCLE',   # 黄黄黑黑 -> 圆形轨迹

            # 连续模式（4位）
            '1111': 'FORWARD',  # 蓝蓝蓝蓝 -> 前进
            '2222': 'LEFT',     # 红红红红 -> 左移
            '3333': 'RIGHT',    # 绿绿绿绿 -> 右移
            '4444': 'STOP',     # 紫紫紫紫 -> 停止
            '5555': 'REVERSE',  # 黄黄黄黄 -> 后退
        }

        # 动作描述（用于日志输出）
        self.ACTION_DESCRIPTIONS = {
            'FORWARD': '前进', 'LEFT': '左移', 'RIGHT': '右移', 'STOP': '停止',
            'REVERSE': '后退', 'APPROACH': '靠近', 'RETREAT': '远离',
            'S_SHAPE': 'S形', 'CIRCLE': '圆形', 'IDLE': '待机'
        }
        
        # 初始化机器人控制器
        self.robot_controller = None
        if self.enable_control:
            try:
                self.robot_controller = SimpleRobotController(
                    robot_id=robot_id,
                    transport='ros',
                    ros_topic='/robot/velcmd'
                )
                print(f"🤖 机器人控制器初始化成功 - ID: {robot_id}")
            except Exception as e:
                print(f"❌ 机器人控制器初始化失败: {e}")
                self.enable_control = False
                self.robot_controller = None
        else:
            print("⚠️ 机器人控制功能已禁用（Mock模式）")
        
        # 初始化 ZeroMQ
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.SUB)
        self.socket.connect(self.zmq_address)
        self.socket.setsockopt_string(zmq.SUBSCRIBE, "perception")
        self.socket.setsockopt(zmq.RCVTIMEO, self.timeout_ms)
        
        # 控制参数
        self.control_dt = 0.05  # 控制周期（秒）
        self.last_command = 'IDLE'
        self.last_message_time = time.time()
        
        # 运动状态
        self.motion_state = {
            's_shape': {'active': False, 'start_time': 0, 'duration': 20.0},
            'circle': {'active': False, 'start_time': 0, 'duration': 8.0}
        }
        
        print(f"✅ 控制订阅者初始化完成 - 订阅 {self.zmq_address}")

    def safety_stop(self):
        """安全停止"""
        try:
            if self.robot_controller:
                self.robot_controller.send_velocity_command(0.0, 0.0, 0.0)
                time.sleep(0.05)
                self.robot_controller.execute_basic_command('stop', duration=0.3)
            else:
                print("🛑 [Mock] 安全停止")
        except Exception as e:
            print(f"❌ 安全停止失败: {e}")

    def execute_approach_step(self, target_info):
        """
        执行靠近动作的单步（非阻塞）
        每次收到消息时计算一次速度并发送
        """
        try:
            distance = float(target_info['distance'])
            bearing_body = float(target_info['bearing_body'])

            target_distance = 0.5  # 目标距离
            error = max(0.0, distance - target_distance)

            # 到位判定
            if error < 0.05:
                print(f"🎯 APPROACH: 已到达目标距离")
                self.safety_stop()
                return True

            # 计算速度
            duration = 3.0
            v = error / duration

            # 分解到机体坐标系
            theta_rad = math.radians(bearing_body)
            vx = v * math.cos(theta_rad)
            vy = v * math.sin(theta_rad)
            omega = 0.0

            print(f"➡️ APPROACH: 误差{error:.2f}m | 方向{bearing_body:.1f}° | vx={vx:.3f}, vy={vy:.3f}")

            if self.robot_controller:
                self.robot_controller.send_velocity_command(vx, vy, omega)
            else:
                print(f"🕒 [Mock] APPROACH: vx={vx:.3f}, vy={vy:.3f}")

            return False  # 未完成

        except Exception as e:
            print(f"❌ APPROACH 执行错误: {e}")
            return False

    def execute_retreat_step(self, target_info):
        """
        执行远离动作的单步（非阻塞）
        每次收到消息时计算一次速度并发送
        """
        try:
            distance = float(target_info['distance'])
            bearing_body = float(target_info['bearing_body'])

            retreat_target_distance = 2.0  # 安全距离
            error = max(0.0, retreat_target_distance - distance)

            # 到位判定
            if error < 0.05:
                print(f"🎯 RETREAT: 已到达安全距离")
                self.safety_stop()
                return True

            # 计算速度（向相反方向运动）
            v_max = 0.4
            kp = 0.6
            v = min(v_max, kp * error)

            # 远离方向：与目标方向相反
            retreat_bearing = (bearing_body + 180.0) % 360.0
            theta_rad = math.radians(retreat_bearing)
            vx = v * math.cos(theta_rad)
            vy = v * math.sin(theta_rad)
            omega = 0.0

            print(f"⬅️ RETREAT: 误差{error:.2f}m | 方向{retreat_bearing:.1f}° | vx={vx:.3f}, vy={vy:.3f}")

            if self.robot_controller:
                self.robot_controller.send_velocity_command(vx, vy, omega)
            else:
                print(f"🕒 [Mock] RETREAT: vx={vx:.3f}, vy={vy:.3f}")

            return False  # 未完成

        except Exception as e:
            print(f"❌ RETREAT 执行错误: {e}")
            return False

    def execute_s_shape_step(self):
        """
        执行S形运动的单步（非阻塞）
        基于时间生成正弦波轨迹
        """
        try:
            state = self.motion_state['s_shape']

            if not state['active']:
                # 首次启动
                state['active'] = True
                state['start_time'] = time.time()
                print(f"🔄 S_SHAPE: 开始执行，持续时间={state['duration']}s")

            elapsed = time.time() - state['start_time']

            # 检查是否完成
            if elapsed >= state['duration']:
                print(f"🎯 S_SHAPE: 执行完成")
                self.safety_stop()
                state['active'] = False
                return True

            # 计算速度
            amplitude = 0.5
            frequency = 0.2
            forward_speed = 0.1

            vx = forward_speed
            vy = amplitude * math.sin(2 * math.pi * frequency * elapsed)
            omega = 0.0

            if self.robot_controller:
                self.robot_controller.send_velocity_command(vx, vy, omega)
            else:
                print(f"🕒 [Mock] S_SHAPE: t={elapsed:.1f}s, vx={vx:.3f}, vy={vy:.3f}")

            return False  # 未完成

        except Exception as e:
            print(f"❌ S_SHAPE 执行错误: {e}")
            self.motion_state['s_shape']['active'] = False
            return False

    def execute_circle_step(self):
        """
        执行圆形运动的单步（非阻塞）
        基于时间生成圆形轨迹
        """
        try:
            state = self.motion_state['circle']

            if not state['active']:
                # 首次启动
                state['active'] = True
                state['start_time'] = time.time()
                print(f"🔄 CIRCLE: 开始执行，持续时间={state['duration']}s")

            elapsed = time.time() - state['start_time']

            # 检查是否完成
            if elapsed >= state['duration']:
                print(f"🎯 CIRCLE: 执行完成")
                self.safety_stop()
                state['active'] = False
                return True

            # 计算速度
            radius = 1.0
            angular_velocity = 2 * math.pi / state['duration']
            linear_velocity = radius * abs(angular_velocity)

            vx = linear_velocity * math.cos(angular_velocity * elapsed)
            vy = linear_velocity * math.sin(angular_velocity * elapsed)
            omega = math.degrees(angular_velocity)

            if self.robot_controller:
                self.robot_controller.send_velocity_command(vx, vy, omega)
            else:
                print(f"🕒 [Mock] CIRCLE: t={elapsed:.1f}s, vx={vx:.3f}, vy={vy:.3f}")

            return False  # 未完成

        except Exception as e:
            print(f"❌ CIRCLE 执行错误: {e}")
            self.motion_state['circle']['active'] = False
            return False

    def execute_basic_command(self, command):
        """执行基本命令（前进、后退、左移、右移、停止）"""
        try:
            action_name = command.lower()
            duration = 4.0
            print(f"🎮 基本命令: {action_name} ({duration}s)")

            if self.robot_controller:
                self.robot_controller.execute_basic_command(action_name, duration=duration)
            else:
                print(f"🕒 [Mock] {action_name} 执行 {duration:.2f}s")
                time.sleep(duration)

            return True

        except Exception as e:
            print(f"❌ 基本命令执行错误: {e}")
            return False

    def run(self):
        """
        【架构解耦 - 决策层】运行控制订阅者主循环
        接收原始 Pattern，翻译成 Command，执行控制
        """
        print(f"🚀 启动控制订阅者")
        print(f"📡 订阅地址: {self.zmq_address}")
        print(f"⏱️ 超时保护: {self.timeout_ms}ms")
        print(f"🧠 决策模式: Pattern -> Command 映射")
        print(f"📋 支持的 Pattern: {list(self.PATTERN_TO_COMMAND.keys())}\n")

        try:
            while True:
                try:
                    # 阻塞接收消息（带超时）
                    message = self.socket.recv_string()
                    self.last_message_time = time.time()

                    # 解析消息
                    parts = message.split(' ', 1)
                    if len(parts) != 2:
                        continue

                    topic, json_data = parts
                    data = json.loads(json_data)

                    # 【关键变更】提取原始 Pattern（而非 command）
                    pattern = data.get('pattern', 'IDLE')
                    target_info = {
                        'distance': data.get('distance', 0),
                        'bearing_body': data.get('bearing_body', 0),
                        'track_id': data.get('track_id', -1),
                        'cam_idx': data.get('cam_idx', -1)
                    }

                    # 【架构解耦 - 决策层】Pattern -> Command 翻译
                    command = self.PATTERN_TO_COMMAND.get(pattern, 'IDLE')

                    # 决策日志
                    if command != 'IDLE':
                        description = self.ACTION_DESCRIPTIONS.get(command, '未知')
                        print(f"🧠 Decision: Pattern '{pattern}' -> Action '{command}' ({description})")

                    # 执行控制逻辑
                    if command == 'APPROACH':
                        self.execute_approach_step(target_info)
                    elif command == 'RETREAT':
                        self.execute_retreat_step(target_info)
                    elif command == 'S_SHAPE':
                        self.execute_s_shape_step()
                    elif command == 'CIRCLE':
                        self.execute_circle_step()
                    elif command in ['FORWARD', 'LEFT', 'RIGHT', 'REVERSE', 'STOP']:
                        if command != self.last_command:
                            self.execute_basic_command(command)
                            self.last_command = command
                    elif command == 'IDLE':
                        # 空闲状态，继续接收
                        pass

                except zmq.Again:
                    # 超时：没有收到消息
                    time_since_last = time.time() - self.last_message_time
                    if time_since_last > 1.0:
                        print(f"⚠️ 超时 {time_since_last:.2f}s，执行安全停止")
                        self.safety_stop()
                        self.last_message_time = time.time()  # 重置计时
                    time.sleep(0.05)

        except KeyboardInterrupt:
            print("\n\n🛑 收到停止信号")
        finally:
            self.cleanup()

    def cleanup(self):
        """清理资源"""
        self.safety_stop()
        self.socket.close()
        self.context.term()
        print("✅ ZeroMQ 资源已清理")


if __name__ == "__main__":
    # 配置参数
    ROBOT_ID = 8
    ENABLE_CONTROL = True  # 设为 False 进入 Mock 模式
    ZMQ_ADDRESS = "tcp://localhost:5555"
    TIMEOUT_MS = 1000  # 1秒超时

    print("=" * 60)
    print("🤖 控制订阅者 (ZeroMQ SUB)")
    print("=" * 60)
    print(f"📡 订阅地址: {ZMQ_ADDRESS}")
    print(f"🆔 机器人ID: {ROBOT_ID}")
    print(f"🎛️ 控制模式: {'实际控制' if ENABLE_CONTROL else 'Mock模式'}")
    print(f"⏱️ 超时保护: {TIMEOUT_MS}ms")
    print("=" * 60)
    print("按 Ctrl+C 停止\n")

    # 创建并运行订阅者
    subscriber = ControlSubscriber(
        robot_id=ROBOT_ID,
        enable_control=ENABLE_CONTROL,
        zmq_address=ZMQ_ADDRESS,
        timeout_ms=TIMEOUT_MS
    )

    try:
        subscriber.run()
    except KeyboardInterrupt:
        print("\n\n🛑 收到停止信号")
    finally:
        print("👋 控制订阅者已退出")

