#!/usr/bin/env python3
"""
控制节点 - 行动线程（Subscriber）
从共享状态读取感知数据，执行机器人控制逻辑
"""

import os
import sys
import time
import math
import threading

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


class ControlNode:
    """
    控制节点 - 只负责行动
    从共享状态读取数据，执行机器人控制
    """
    
    def __init__(self, robot_state, robot_id=8, enable_control=True, control_hz=20.0):
        """
        初始化控制节点
        
        Args:
            robot_state: RobotState 实例
            robot_id: 机器人ID
            enable_control: 是否启用实际控制
            control_hz: 控制循环频率（Hz）
        """
        self.robot_state = robot_state
        self.robot_id = robot_id
        self.enable_control = enable_control and ROBOT_CONTROL_AVAILABLE
        self.control_hz = control_hz
        self.control_dt = 1.0 / control_hz
        
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
        
        # 线程控制
        self.stop_event = threading.Event()
        self.thread = None
        self.running = False
        
        # 控制参数
        self.target_timeout = 1.0  # 目标信息超时时间（秒）
        
        print("✅ 控制节点初始化完成")
    
    def control_loop(self):
        """控制循环 - 核心逻辑"""
        print(f"🚀 启动控制循环 @ {self.control_hz}Hz")
        
        last_command = 'IDLE'
        
        try:
            while not self.stop_event.is_set():
                loop_start = time.time()
                
                # 获取最新状态
                state = self.robot_state.get_latest_state()
                target_info = state['target_info']
                command = state['command']
                command_params = state['command_params']
                time_since_update = state['time_since_update']
                is_executing = state['is_executing']
                
                # 超时检查
                if time_since_update > self.target_timeout:
                    # 数据超时，执行安全停止
                    if last_command != 'TIMEOUT_STOP':
                        print(f"⚠️ 目标数据超时 ({time_since_update:.2f}s)，执行安全停止")
                        self.safety_stop()
                        last_command = 'TIMEOUT_STOP'
                    time.sleep(self.control_dt)
                    continue
                
                # 如果正在执行动作，跳过本次循环
                if is_executing:
                    time.sleep(self.control_dt)
                    continue
                
                # 执行指令
                if command != 'IDLE' and command != last_command:
                    print(f"📥 收到新指令: {command} - {command_params.get('description', '')}")
                    
                    # 标记正在执行
                    self.robot_state.set_executing(True)
                    
                    try:
                        # 根据指令类型执行不同的控制策略
                        if command == 'APPROACH' and target_info:
                            self.execute_approach(target_info)
                        elif command == 'RETREAT' and target_info:
                            self.execute_retreat(target_info)
                        elif command == 'S_SHAPE':
                            self.execute_s_shape()
                        elif command == 'CIRCLE':
                            self.execute_circle()
                        elif command in ['FORWARD', 'LEFT', 'RIGHT', 'REVERSE', 'STOP']:
                            self.execute_basic_command(command)
                        else:
                            print(f"⚠️ 未知指令: {command}")
                    
                    except Exception as e:
                        print(f"❌ 执行指令时出错: {e}")
                    
                    finally:
                        # 执行完成，清除指令并标记
                        self.robot_state.clear_command()
                        self.robot_state.set_executing(False)
                        last_command = command
                
                # 控制循环频率
                elapsed = time.time() - loop_start
                sleep_time = max(0, self.control_dt - elapsed)
                time.sleep(sleep_time)
        
        except Exception as e:
            print(f"❌ 控制循环异常: {e}")
        finally:
            self.safety_stop()
            print("🏁 控制循环结束")

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

    def execute_approach(self, target_info):
        """
        执行靠近动作 - 基于实时的距离和角度
        使用全向运动，不转向
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

            print(f"➡️ APPROACH: 距离误差{error:.2f}m | 方向{bearing_body:.1f}° | vx={vx:.3f}, vy={vy:.3f}")

            if self.robot_controller:
                # 连续发布速度
                t_start = time.time()
                while time.time() - t_start < duration:
                    self.robot_controller.send_velocity_command(vx, vy, omega)
                    time.sleep(self.control_dt)

                self.safety_stop()
            else:
                print(f"🕒 [Mock] APPROACH 执行 {duration:.2f}s")
                time.sleep(duration)

            return True

        except Exception as e:
            print(f"❌ APPROACH 执行错误: {e}")
            return False

    def execute_retreat(self, target_info):
        """
        执行远离动作 - 基于实时的距离和角度
        使用全向运动，不转向
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

            # 预计时间
            est_time = error / max(1e-3, v_max)
            duration = min(10.0, max(0.5, est_time))

            print(f"⬅️ RETREAT: 距离误差{error:.2f}m | 方向{retreat_bearing:.1f}° | vx={vx:.3f}, vy={vy:.3f}")

            if self.robot_controller:
                # 连续发布速度
                t_start = time.time()
                while time.time() - t_start < duration:
                    self.robot_controller.send_velocity_command(vx, vy, omega)
                    time.sleep(self.control_dt)

                self.safety_stop()
            else:
                print(f"🕒 [Mock] RETREAT 执行 {duration:.2f}s")
                time.sleep(duration)

            return True

        except Exception as e:
            print(f"❌ RETREAT 执行错误: {e}")
            return False

    def execute_s_shape(self, amplitude=0.5, frequency=0.2, forward_speed=0.1, total_duration=20.0):
        """执行S形运动"""
        try:
            print(f"🔄 S_SHAPE: 幅度={amplitude}m, 频率={frequency}Hz, 持续时间={total_duration}s")

            if self.robot_controller:
                t_start = time.time()
                while time.time() - t_start < total_duration:
                    t = time.time() - t_start
                    vx = forward_speed
                    vy = amplitude * math.sin(2 * math.pi * frequency * t)
                    omega = 0.0

                    self.robot_controller.send_velocity_command(vx, vy, omega)
                    time.sleep(self.control_dt)

                self.safety_stop()
            else:
                print(f"🕒 [Mock] S_SHAPE 执行 {total_duration:.2f}s")
                time.sleep(total_duration)

            return True

        except Exception as e:
            print(f"❌ S_SHAPE 执行错误: {e}")
            return False

    def execute_circle(self, radius=1.0, desired_duration=8.0, direction='left'):
        """执行圆形运动"""
        try:
            print(f"🔄 CIRCLE: 半径={radius}m, 持续时间={desired_duration}s")

            # 计算角速度和线速度
            angular_velocity = 2 * math.pi / desired_duration
            if direction == 'right':
                angular_velocity = -angular_velocity
            linear_velocity = radius * abs(angular_velocity)

            if self.robot_controller:
                t_start = time.time()
                while time.time() - t_start < desired_duration:
                    t = time.time() - t_start
                    vx = linear_velocity * math.cos(angular_velocity * t)
                    vy = linear_velocity * math.sin(angular_velocity * t)
                    omega = math.degrees(angular_velocity)

                    self.robot_controller.send_velocity_command(vx, vy, omega)
                    time.sleep(self.control_dt)

                self.safety_stop()
            else:
                print(f"🕒 [Mock] CIRCLE 执行 {desired_duration:.2f}s")
                time.sleep(desired_duration)

            return True

        except Exception as e:
            print(f"❌ CIRCLE 执行错误: {e}")
            return False

    def execute_basic_command(self, command, duration=4.0):
        """执行基本命令（前进、后退、左移、右移、停止）"""
        try:
            action_name = command.lower()
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

    def start(self):
        """启动控制节点"""
        if self.running:
            print("⚠️ 控制节点已在运行")
            return

        self.stop_event.clear()
        self.running = True

        # 启动控制线程
        self.thread = threading.Thread(target=self.control_loop, daemon=True)
        self.thread.start()

        print("✅ 控制节点已启动")

    def stop(self):
        """停止控制节点"""
        if not self.running:
            return

        print("🛑 正在停止控制节点...")
        self.stop_event.set()

        if self.thread:
            self.thread.join(timeout=2.0)

        self.safety_stop()
        self.running = False
        print("✅ 控制节点已停止")

