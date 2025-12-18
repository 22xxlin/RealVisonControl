#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
test_motor_friction.py
功能：自动测试机器人的最小启动动力 (静摩擦阈值)。
原理：从 0 开始缓慢增加电机指令，直到 Vicon 检测到机器人移动超过 1cm。
"""

import rospy
import time
import math
from geometry_msgs.msg import TransformStamped
from robot_driver import RobotDriver

# =========================
# 配置
# =========================
ROBOT_ID = 15
ROBOT_TOPIC = f"vicon/VSWARM{ROBOT_ID}/VSWARM{ROBOT_ID}"

# 测试参数
START_CMD = 0.02      # 起始指令
STEP_CMD  = 0.002     # 每次增加的指令大小 (0.2%)
STEP_TIME = 1.0       # 每个指令持续测试时间 (秒)
MOVE_THRESHOLD = 0.01 # 判定移动的阈值 (1cm)
MAX_TEST_CMD = 0.15   # 安全上限，如果加到0.15还没动，说明出大问题了

class FrictionTester:
    def __init__(self):
        self.current_pos = None
        self.start_pos = None
        rospy.Subscriber(ROBOT_TOPIC, TransformStamped, self._cb)
        
    def _cb(self, msg):
        self.current_pos = {
            'x': msg.transform.translation.x,
            'y': msg.transform.translation.y
        }

    def wait_for_vicon(self):
        print("⏳ 等待 Vicon 数据...", end="", flush=True)
        while self.current_pos is None and not rospy.is_shutdown():
            time.sleep(0.1)
        print(" ✅ 就绪!")
        time.sleep(1.0) # 等一秒稳定数据
        self.start_pos = self.current_pos # 记录初始位置

    def get_displacement(self):
        """计算相对于起点的位移"""
        if not self.current_pos or not self.start_pos: return 0.0
        dx = self.current_pos['x'] - self.start_pos['x']
        dy = self.current_pos['y'] - self.start_pos['y']
        return math.hypot(dx, dy)

def run_test():
    rospy.init_node('friction_test', anonymous=True)
    driver = RobotDriver(ROBOT_ID)
    tester = FrictionTester()
    
    try:
        tester.wait_for_vicon()
        
        current_cmd = START_CMD
        print("\n🚀 开始摩擦力测试 (请确保机器人周围空旷)")
        print("-" * 40)
        
        while not rospy.is_shutdown():
            # 1. 发送指令
            print(f"测试指令: {current_cmd:.3f} ... ", end="", flush=True)
            driver.send_velocity_command(current_cmd, 0.0, 0.0) # 只给X方向速度
            
            # 2. 持续一段时间，观察是否移动
            start_wait = time.time()
            moved = False
            while time.time() - start_wait < STEP_TIME:
                dist = tester.get_displacement()
                if dist > MOVE_THRESHOLD:
                    moved = True
                    break
                time.sleep(0.05)
            
            # 3. 判定结果
            if moved:
                print(f"🚗 动了! (位移 {dist*100:.1f}cm)")
                print("-" * 40)
                print(f"✅ 测得最小启动动力 (FeedForward) ≈ {current_cmd:.3f}")
                print(f"建议设置 FRICTION_FEEDFORWARD = {current_cmd + 0.005:.3f} (加一点余量)")
                break
            else:
                print(f"没动 (位移 {dist*100:.2f}cm)")
            
            # 4. 增加指令
            current_cmd += STEP_CMD
            
            # 5. 安全检查
            if current_cmd > MAX_TEST_CMD:
                print("\n⚠️ 警告: 指令已超过 0.15 但车还没动！可能被卡住了或没电了。")
                break

    except KeyboardInterrupt:
        pass
    finally:
        driver.stop()
        print("\n🛑 测试结束，机器人已停止。")

if __name__ == "__main__":
    run_test()