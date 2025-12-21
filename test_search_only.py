#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
测试脚本：强制保持 SEARCHER 角色，测试搜索动作
"""
import sys
import time
import argparse

# 添加路径
sys.path.insert(0, '/home/nvidia/Downloads/Ros/pseudo_ros_architecture')

from robot_driver import RobotDriver
from light_driver import LightDriver
import math

def mono():
    return time.monotonic()

def test_search_move(robot_id):
    print(f"🧪 测试搜索动作 | Robot ID: {robot_id}")

    driver = RobotDriver(robot_id)
    light = LightDriver(robot_id, broker_ip="10.0.2.66")

    light.set_cmd("SEARCH")  # 紫闪

    start_time = mono()

    try:
        while True:
            elapsed = mono() - start_time
            vx, vy = 0.0, 0.0

            # 阶段1: 散开 (5秒)
            if elapsed < 5.0:
                if robot_id == 13:
                    vy = 0.2
                    print(f"⏱️  {elapsed:.1f}s | 13号横移 | vy={vy}")
                elif robot_id == 10:
                    vx = 0.2
                    print(f"⏱️  {elapsed:.1f}s | 10号前进 | vx={vx}")
                elif robot_id == 15:
                    vx = -0.2
                    print(f"⏱️  {elapsed:.1f}s | 15号后退 | vx={vx}")
            # 阶段2: S形搜索
            else:
                vy = -0.2
                vx = 0.15 * math.sin(elapsed * math.pi)
                print(f"⏱️  {elapsed:.1f}s | S形搜索 | vx={vx:.2f}, vy={vy:.2f}")

            driver.send_velocity_command(vx, vy, 0.0)
            time.sleep(1.0)  # 每秒打印一次

            if elapsed > 10.0:
                print("✅ 测试完成 (10秒)")
                break
                
    except KeyboardInterrupt:
        print("\n⏹️  测试中断")
    finally:
        driver.stop()
        light.set_cmd("OFF")
        light.stop()

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--id", type=int, required=True, help="Robot ID")
    args = parser.parse_args()
    
    test_search_move(args.id)

