#!/usr/bin/env python3
"""
测试脚本：演示 vision_pub.py 的多摄像头并行架构

【架构解耦后更新】
- 现在接收的是原始 Pattern（如 '2200', '110'）而非 Command
- 兼容新的数据格式：{'pattern': '2200', ...}
"""

import time
import zmq
import json
import threading

def zmq_subscriber(port=5555, duration=10):
    """
    ZMQ 订阅者，用于接收 vision_pub.py 发送的检测数据
    
    Args:
        port: ZMQ 端口
        duration: 运行时长（秒）
    """
    context = zmq.Context()
    socket = context.socket(zmq.SUB)
    socket.connect(f"tcp://localhost:{port}")
    socket.setsockopt_string(zmq.SUBSCRIBE, "perception")
    
    print(f"🔌 ZMQ 订阅者已连接到 tcp://localhost:{port}")
    print(f"⏱️  将运行 {duration} 秒...\n")
    
    start_time = time.time()
    message_count = 0
    cam_stats = {0: 0, 2: 0, 4: 0, 6: 0}
    
    try:
        while time.time() - start_time < duration:
            try:
                # 设置超时，避免阻塞
                if socket.poll(timeout=1000):  # 1秒超时
                    message = socket.recv_string()
                    
                    # 解析消息
                    topic, data_str = message.split(' ', 1)
                    data = json.loads(data_str)
                    
                    message_count += 1
                    cam_idx = data.get('cam_idx', -1)
                    if cam_idx in cam_stats:
                        cam_stats[cam_idx] += 1

                    # 【架构解耦后更新】打印检测信息
                    # 现在接收的是 'pattern' 而非 'command'
                    pattern = data.get('pattern', 'IDLE')
                    dist = data.get('distance', 0)
                    bearing = data.get('bearing_body', 0)
                    track_id = data.get('track_id', -1)

                    if pattern != 'IDLE':
                        print(f"📥 [Cam{cam_idx}] Received Pattern: '{pattern}' | "
                              f"Dist={dist:.2f}m | Bearing={bearing:.1f}° | "
                              f"TrackID={track_id}")
            
            except zmq.Again:
                continue
            except Exception as e:
                print(f"❌ 接收错误: {e}")
    
    except KeyboardInterrupt:
        print("\n⚠️ 订阅者收到中断信号")
    
    finally:
        # 统计信息
        elapsed = time.time() - start_time
        print(f"\n{'='*60}")
        print(f"📊 统计信息")
        print(f"{'='*60}")
        print(f"运行时长: {elapsed:.1f} 秒")
        print(f"总消息数: {message_count}")
        print(f"平均速率: {message_count/elapsed:.1f} msg/s")
        print(f"\n各摄像头消息数:")
        for cam_idx, count in sorted(cam_stats.items()):
            print(f"  - Cam{cam_idx}: {count} 条")
        print(f"{'='*60}")
        
        socket.close()
        context.term()
        print("✅ ZMQ 订阅者已关闭")


if __name__ == "__main__":
    print("=" * 60)
    print("🧪 vision_pub.py 测试脚本")
    print("=" * 60)
    print("📝 说明:")
    print("   1. 先运行 vision_pub.py（发布者）")
    print("   2. 再运行本脚本（订阅者）")
    print("   3. 本脚本会接收并统计来自4个摄像头的检测数据")
    print("=" * 60)
    print()
    
    # 运行订阅者
    zmq_subscriber(port=5555, duration=3000)

