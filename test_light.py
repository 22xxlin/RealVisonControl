#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
文件: test_light_vision.py
功能: 手动控制灯光，用于验证 vision_pub.py 的识别效果
用法: 
  1. 运行此脚本控制灯光切换
  2. 在另一个终端运行 vision_pub.py 查看识别结果
"""

import sys
import time
import threading

try:
    from light_driver import LightDriver
except ImportError:
    print("❌ 错误: 找不到 light_driver.py")
    sys.exit(1)

# === 测试菜单定义 ===
# 格式: (指令Key, 描述, 预期视觉输出)
TEST_MENU = [
    ("OFF",           "⚫ 关闭",                   "Class 0 (OFF)"),
    ("SEARCH",        "🟣 紫色闪烁 (搜索)",         "Class 4 (PURPLE) - FLASH"),
    ("APPROACH_BALL", "🔴 红色闪烁 (靠近球)",       "Class 2 (RED)    - FLASH"),
    ("LEADER_WAIT",   "🔴 红色常亮 (Leader锚点)",   "Class 2 (RED)    - SOLID"),
    ("BID_LEFT",      "🟢 绿色闪烁 (抢左槽)",       "Class 3 (GREEN)  - FLASH"),
    ("LOCK_LEFT",     "🟢 绿色常亮 (左就位)",       "Class 3 (GREEN)  - SOLID"),
    ("BID_RIGHT",     "🔵 蓝色闪烁 (抢右槽)",       "Class 1 (BLUE)   - FLASH"),
    ("LOCK_RIGHT",    "🔵 蓝色常亮 (右就位)",       "Class 1 (BLUE)   - SOLID"),
    ("LEADER_GO",     "🟣 紫色常亮 (Leader搬运)",   "Class 4 (PURPLE) - SOLID"),
    ("FOLLOWER_PUSH", "⚪ 灰色常亮 (Follower搬运)", "Class 5 (GRAY)   - SOLID"),
]

def print_menu():
    print("\n" + "="*50)
    print("🚦 灯光与视觉联合测试工具")
    print("="*50)
    for i, (key, desc, expect) in enumerate(TEST_MENU):
        print(f"[{i}] {desc:<20} -> 👁️ 预期: {expect}")
    print("[a] 自动循环测试 (每5秒切一次)")
    print("[q] 退出并熄灯")
    print("="*50)

def auto_cycle(driver):
    print("\n🔄 开始自动循环测试 (按 Ctrl+C 停止)...")
    try:
        while True:
            for key, desc, expect in TEST_MENU:
                if key == "OFF": continue
                print(f"\n👉 正在执行: {desc}")
                print(f"👁️ 请检查视觉端是否输出: {expect}")
                driver.set_cmd(key)
                
                # 倒计时
                for k in range(5, 0, -1):
                    print(f"   保持 {k}s...", end="\r")
                    time.sleep(1)
    except KeyboardInterrupt:
        print("\n⏸️ 自动循环停止")

def main():
    # 1. 初始化驱动
    robot_id = 15 # 测试用的默认ID
    print(f"正在连接 MQTT (Robot ID: {robot_id})...")
    
    try:
        driver = LightDriver(robot_id, broker_ip="10.0.2.66")
        time.sleep(1) # 等待连接
    except Exception as e:
        print(f"❌ 驱动启动失败: {e}")
        return

    # 2. 交互循环
    while True:
        print_menu()
        user_input = input("请输入指令序号: ").strip().lower()

        if user_input == 'q':
            print("👋 退出中...")
            driver.stop()
            break
        
        elif user_input == 'a':
            auto_cycle(driver)
            driver.set_cmd("OFF") # 循环结束后熄灭

        elif user_input.isdigit():
            idx = int(user_input)
            if 0 <= idx < len(TEST_MENU):
                key, desc, expect = TEST_MENU[idx]
                print(f"\n✅ 已发送: {key}")
                print(f"📝 状态: {desc}")
                print(f"👁️ 预期视觉输出: {expect}")
                driver.set_cmd(key)
            else:
                print("❌ 无效序号")
        else:
            print("❌ 无效输入")

if __name__ == "__main__":
    main()