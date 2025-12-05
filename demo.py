#!/usr/bin/env python3
"""
演示脚本 - 展示伪ROS架构的使用方式（不需要真实硬件）
模拟视觉线程发布数据，控制线程订阅并响应
"""

import sys
import os
import time
import threading

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from robot_shared_state import RobotState

print("="*70)
print("🎬 伪ROS架构演示 - 生产者-消费者模式")
print("="*70)

# 创建共享状态
robot_state = RobotState()

# 模拟视觉线程（生产者）
def mock_vision_thread():
    """模拟视觉线程，持续发布感知数据"""
    print("\n📹 [视觉线程] 启动...")
    
    for i in range(20):
        # 模拟检测到目标，距离逐渐减小
        distance = 3.0 - i * 0.1
        angle = 45.0 + i * 2.0
        bearing = (90.0 + i * 5.0) % 360.0
        
        # 根据距离改变指令
        if distance > 2.0:
            command = 'APPROACH'
        elif distance > 1.0:
            command = 'IDLE'
        else:
            command = 'STOP'
        
        # 更新共享状态
        robot_state.update_perception(
            distance=distance,
            azimuth=angle,
            bearing_body=bearing,
            track_id=5,
            cam_idx=0,
            command=command,
            command_params={'description': f'指令-{command}'}
        )
        
        print(f"📹 [视觉线程] 发布 #{i+1}: 距离={distance:.2f}m, "
              f"角度={bearing:.1f}°, 指令={command}")
        
        time.sleep(0.5)  # 模拟检测频率 2Hz
    
    print("📹 [视觉线程] 结束")

# 模拟控制线程（消费者）
def mock_control_thread():
    """模拟控制线程，持续读取并响应"""
    print("\n🤖 [控制线程] 启动...")
    
    last_command = 'IDLE'
    
    for i in range(40):  # 运行更久，以接收所有视觉数据
        # 获取最新状态
        state = robot_state.get_latest_state()
        
        target_info = state['target_info']
        command = state['command']
        time_since_update = state['time_since_update']
        
        # 超时检查
        if time_since_update > 1.0:
            if last_command != 'TIMEOUT':
                print(f"🤖 [控制线程] ⚠️ 数据超时 ({time_since_update:.2f}s)，安全停止")
                last_command = 'TIMEOUT'
        elif target_info and command != 'IDLE':
            if command != last_command:
                print(f"🤖 [控制线程] 执行指令: {command} | "
                      f"距离={target_info['distance']:.2f}m | "
                      f"角度={target_info['bearing_body']:.1f}°")
                last_command = command
        
        time.sleep(0.25)  # 控制频率 4Hz
    
    print("🤖 [控制线程] 结束")

# 主函数
def main():
    print("\n💡 说明: 这是一个演示程序，展示视觉线程和控制线程如何通过共享状态通信")
    print("   - 📹 视觉线程: 模拟目标检测，更新距离/角度/指令")
    print("   - 🤖 控制线程: 读取最新状态，执行相应动作")
    print("   - 🔒 线程安全: 使用锁保护共享状态的读写")
    print("\n" + "="*70)
    
    # 启动线程
    vision_thread = threading.Thread(target=mock_vision_thread, daemon=True)
    control_thread = threading.Thread(target=mock_control_thread, daemon=True)
    
    vision_thread.start()
    time.sleep(0.5)  # 让视觉线程先启动
    control_thread.start()
    
    # 等待线程完成
    vision_thread.join()
    control_thread.join()
    
    print("\n" + "="*70)
    print("✅ 演示完成")
    print("\n📊 最终状态:")
    final_state = robot_state.get_latest_state()
    if final_state['target_info']:
        print(f"   - 最后距离: {final_state['target_info']['distance']:.2f}m")
        print(f"   - 最后角度: {final_state['target_info']['bearing_body']:.1f}°")
        print(f"   - 最后指令: {final_state['command']}")
    print("="*70)
    
    print("\n💡 要运行完整系统（需要摄像头和模型）:")
    print("   python3 main_system.py")

if __name__ == '__main__':
    main()

