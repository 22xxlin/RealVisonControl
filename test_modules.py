#!/usr/bin/env python3
"""
快速测试脚本 - 验证模块导入和基本功能
"""

import sys
import os
import time

# 添加父目录到路径
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

print("="*60)
print("🧪 伪ROS架构 - 模块测试")
print("="*60)

# 测试1: 导入共享状态模块
print("\n1️⃣ 测试共享状态模块导入...")
try:
    from robot_shared_state import RobotState
    print("   ✅ RobotState 导入成功")
except Exception as e:
    print(f"   ❌ RobotState 导入失败: {e}")
    sys.exit(1)

# 测试2: 共享状态
print("\n2️⃣ 测试共享状态...")
try:
    state = RobotState()
    
    # 写入数据
    state.update_perception(
        distance=1.5,
        azimuth=45.0,
        bearing_body=90.0,
        track_id=5,
        cam_idx=0,
        command='APPROACH',
        command_params={'description': '靠近'},
        class_id=2
    )
    
    # 读取数据
    result = state.get_latest_state()
    assert result['target_info'] is not None
    assert result['target_info']['distance'] == 1.5
    assert result['command'] == 'APPROACH'
    
    print("   ✅ 共享状态读写正常")
    print(f"      - 距离: {result['target_info']['distance']:.2f}m")
    print(f"      - 角度: {result['target_info']['bearing_body']:.1f}°")
    print(f"      - 指令: {result['command']}")
except Exception as e:
    print(f"   ❌ 共享状态测试失败: {e}")
    import traceback
    traceback.print_exc()

# 测试3: 超时检测
print("\n3️⃣ 测试超时检测...")
try:
    state2 = RobotState()
    state2.update_perception(1.0, 0.0, 0.0, 1, 0)
    
    assert state2.is_target_valid(timeout=1.0) == True
    print("   ✅ 数据有效（未超时）")
    
    time.sleep(1.2)
    assert state2.is_target_valid(timeout=1.0) == False
    print("   ✅ 超时检测正常")
except Exception as e:
    print(f"   ❌ 超时检测失败: {e}")

# 测试4: 线程安全（简单测试）
print("\n4️⃣ 测试线程安全...")
try:
    import threading
    
    state3 = RobotState()
    errors = []
    
    def writer():
        try:
            for i in range(100):
                state3.update_perception(
                    distance=float(i),
                    azimuth=0.0,
                    bearing_body=0.0,
                    track_id=i,
                    cam_idx=0
                )
        except Exception as e:
            errors.append(e)
    
    def reader():
        try:
            for i in range(100):
                result = state3.get_latest_state()
                time.sleep(0.001)
        except Exception as e:
            errors.append(e)
    
    threads = []
    for _ in range(5):
        threads.append(threading.Thread(target=writer))
        threads.append(threading.Thread(target=reader))
    
    for t in threads:
        t.start()
    
    for t in threads:
        t.join()
    
    if not errors:
        print("   ✅ 线程安全测试通过（10个线程，1000次操作）")
    else:
        print(f"   ❌ 线程安全测试失败: {errors}")
except Exception as e:
    print(f"   ❌ 线程安全测试失败: {e}")

print("\n" + "="*60)
print("✅ 所有测试完成")
print("="*60)
print("\n💡 提示: 运行 'python3 main_system.py' 启动完整系统")

