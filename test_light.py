#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
文件: test_light.py
功能: 单独测试 LightDriver 的状态切换和 MQTT 发送
"""

import time
import sys
# 确保导入你刚才保存的驱动文件
try:
    from light_driver import LightDriver
except ImportError:
    print("❌ 找不到 light_driver.py，请确保它在同一目录下！")
    sys.exit(1)

def main():
    # 配置
    ROBOT_ID = 15
    BROKER_IP = "10.0.2.66"  # 如果是本机测试，改成 "localhost"
    
    print(f"🚀 开始测试 LightDriver (ID: {ROBOT_ID}, IP: {BROKER_IP})")
    print("按 Ctrl+C 强行中止")
    
    try:
        # 1. 初始化驱动
        driver = LightDriver(robot_id=ROBOT_ID, broker_ip=BROKER_IP)
        time.sleep(1) # 等待 MQTT 连接建立

        # 2. 测试循环
        # 测试序列：(指令名称, 预期效果, 持续时间)
        test_sequence = [
            ("SEARCH",  "🟣 搜索状态 (紫色闪烁)", 5),
            ("FOUND",   "🔵 发现状态 (蓝色闪烁)", 5),
            ("ARRIVED", "🟢 到达状态 (绿色常亮)", 5),
            ("IDLE",    "⚪ 待机状态 (灰色闪烁)", 5),
            ("OFF",     "⚫ 关闭 (熄灭)", 3)
        ]

        for cmd, desc, duration in test_sequence:
            print(f"\n👉 发送指令: {cmd} -> {desc}")
            driver.set_cmd(cmd)
            
            # 倒计时显示
            for i in range(duration, 0, -1):
                print(f"   保持 {i} 秒...", end="\r")
                time.sleep(1)
            print("   完成!            ")

        print("\n✅ 测试序列结束")
        driver.stop()

    except KeyboardInterrupt:
        print("\n🛑 用户中止")
        driver.stop()
    except Exception as e:
        print(f"\n❌ 发生错误: {e}")
        # 尝试安全停止
        try:
            driver.stop()
        except:
            pass

if __name__ == "__main__":
    main()