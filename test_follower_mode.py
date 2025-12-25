#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
测试 Follower 模式：模拟只看到 Leader (红灯)，没有球的情况
验证虚拟球模式是否工作
"""
import zmq
import json
import time
import sys

def publish_fake_leader():
    """发布假的 Leader 视觉数据 (红色常亮灯)"""
    context = zmq.Context()
    socket = context.socket(zmq.PUB)
    socket.bind("tcp://*:5556")  # 使用不同端口避免冲突
    
    time.sleep(0.5)  # 等待连接建立
    
    print("📡 开始发布假 Leader 数据 (红色常亮，距离 1.7m，角度 290°)")
    print("   预期行为: Follower 应该使用虚拟球模式，开始移动接近 Leader")
    print("")
    
    count = 0
    try:
        while True:
            # 模拟 Leader 数据 (红色常亮)
            fake_data = {
                "timestamp": time.time(),
                "count": 1,
                "objects": [{
                    "cam_idx": 2,
                    "class_id": 2,  # RED
                    "pattern": "SOLID",  # 常亮
                    "distance": 1.7,  # 1.7米
                    "bearing_body": 290.0,  # 角度
                    "conf": 0.95,
                    "area": 1200,
                    "truncated": False
                }]
            }
            
            msg = f"perception {json.dumps(fake_data)}"
            socket.send_string(msg)
            
            count += 1
            if count % 30 == 0:  # 每秒打印一次 (30Hz)
                print(f"📤 已发送 {count} 帧 | Leader: D=1.7m, Ang=290°")
            
            time.sleep(1.0/30.0)  # 30Hz
            
    except KeyboardInterrupt:
        print("\n⏹️  停止发布")
    finally:
        socket.close()
        context.term()

if __name__ == "__main__":
    print("=" * 60)
    print("测试 Follower 虚拟球模式")
    print("=" * 60)
    print("")
    print("使用方法:")
    print("  终端1: python3 test_follower_mode.py")
    print("  终端2: python3 full_mission_controller.py --id 15")
    print("")
    print("注意: 需要修改 full_mission_controller.py 的 ZMQ 端口为 5556")
    print("      或者停止 vision_pub.py，让本脚本使用 5555 端口")
    print("")
    
    publish_fake_leader()

