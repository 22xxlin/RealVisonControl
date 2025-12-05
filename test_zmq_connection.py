#!/usr/bin/env python3
"""
ZeroMQ 连接测试脚本
用于验证 PUB-SUB 通信是否正常
"""

import zmq
import json
import time
import sys
from threading import Thread


def publisher_test(port=5555, duration=10):
    """测试发布者"""
    print(f"📡 启动测试发布者 - 端口 {port}")
    
    context = zmq.Context()
    socket = context.socket(zmq.PUB)
    socket.bind(f"tcp://*:{port}")
    
    # 等待订阅者连接
    time.sleep(1)
    
    print("🚀 开始发送测试消息...\n")
    
    start_time = time.time()
    count = 0
    
    try:
        while time.time() - start_time < duration:
            # 构造测试消息
            data = {
                'distance': 1.5 + count * 0.1,
                'bearing_body': 45.0,
                'track_id': 1,
                'cam_idx': 4,
                'command': 'APPROACH',
                'description': '靠近',
                'timestamp': time.time()
            }
            
            message = f"perception {json.dumps(data)}"
            socket.send_string(message)
            
            count += 1
            print(f"✅ 发送消息 #{count}: dist={data['distance']:.2f}m")
            time.sleep(0.5)
    
    except KeyboardInterrupt:
        print("\n⚠️ 发布者收到中断信号")
    finally:
        socket.close()
        context.term()
        print(f"🏁 发布者结束 - 共发送 {count} 条消息")


def subscriber_test(address="tcp://localhost:5555", duration=10):
    """测试订阅者"""
    print(f"📡 启动测试订阅者 - 地址 {address}")
    
    context = zmq.Context()
    socket = context.socket(zmq.SUB)
    socket.connect(address)
    socket.setsockopt_string(zmq.SUBSCRIBE, "perception")
    socket.setsockopt(zmq.RCVTIMEO, 1000)  # 1秒超时
    
    print("🚀 开始接收测试消息...\n")
    
    start_time = time.time()
    count = 0
    
    try:
        while time.time() - start_time < duration:
            try:
                message = socket.recv_string()
                parts = message.split(' ', 1)
                
                if len(parts) == 2:
                    topic, json_data = parts
                    data = json.loads(json_data)
                    
                    count += 1
                    print(f"✅ 接收消息 #{count}: dist={data['distance']:.2f}m, cmd={data['command']}")
            
            except zmq.Again:
                print("⏱️ 超时：1秒内未收到消息")
                time.sleep(0.1)
    
    except KeyboardInterrupt:
        print("\n⚠️ 订阅者收到中断信号")
    finally:
        socket.close()
        context.term()
        print(f"🏁 订阅者结束 - 共接收 {count} 条消息")


def run_full_test():
    """运行完整测试（发布者+订阅者）"""
    print("=" * 60)
    print("🧪 ZeroMQ PUB-SUB 连接测试")
    print("=" * 60)
    print("此测试将同时启动发布者和订阅者")
    print("测试时长: 10秒\n")
    
    # 启动发布者线程
    pub_thread = Thread(target=publisher_test, args=(5555, 10), daemon=True)
    pub_thread.start()
    
    # 等待发布者启动
    time.sleep(1.5)
    
    # 启动订阅者（主线程）
    subscriber_test("tcp://localhost:5555", 10)
    
    # 等待发布者线程结束
    pub_thread.join(timeout=2)
    
    print("\n" + "=" * 60)
    print("✅ 测试完成！")
    print("=" * 60)


if __name__ == "__main__":
    if len(sys.argv) > 1:
        mode = sys.argv[1].lower()
        
        if mode == "pub":
            print("📡 仅运行发布者模式")
            publisher_test()
        elif mode == "sub":
            print("📡 仅运行订阅者模式")
            subscriber_test()
        else:
            print("❌ 未知模式，请使用: pub 或 sub")
            print("💡 或者不带参数运行完整测试")
    else:
        run_full_test()

