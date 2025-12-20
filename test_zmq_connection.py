#!/usr/bin/env python3
"""
ZMQ 连接测试脚本
用于验证 vision_pub.py 和 transport_node.py 之间的通信
"""

import zmq
import time
import json

def test_subscriber():
    """测试订阅端（模拟 transport_node.py）"""
    print("=" * 60)
    print("ZMQ 订阅端测试")
    print("=" * 60)
    
    endpoint = "tcp://127.0.0.1:5555"
    topic = "perception"
    
    print(f"连接到: {endpoint}")
    print(f"订阅 topic: {topic}")
    
    ctx = zmq.Context()
    sock = ctx.socket(zmq.SUB)
    sock.connect(endpoint)
    sock.setsockopt_string(zmq.SUBSCRIBE, topic)
    
    # 设置接收超时
    sock.setsockopt(zmq.RCVTIMEO, 5000)  # 5秒超时
    
    print("\n等待消息... (5秒超时)")
    print("-" * 60)
    
    msg_count = 0
    start_time = time.time()
    
    try:
        while time.time() - start_time < 10:  # 最多等待10秒
            try:
                raw_msg = sock.recv_string(flags=zmq.NOBLOCK)
                msg_count += 1
                
                # 解析消息
                try:
                    topic_recv, payload = raw_msg.split(" ", 1)
                    data = json.loads(payload)
                    
                    print(f"\n✅ 收到消息 #{msg_count}:")
                    print(f"   Topic: {topic_recv}")
                    print(f"   Payload: {json.dumps(data, indent=2)}")
                    
                except Exception as e:
                    print(f"\n⚠️ 消息解析失败: {e}")
                    print(f"   原始消息: {raw_msg[:200]}")
                    
            except zmq.Again:
                # 没有消息，等待一下
                time.sleep(0.1)
                
    except KeyboardInterrupt:
        print("\n\n用户中断")
    
    print("\n" + "=" * 60)
    print(f"测试结束: 共收到 {msg_count} 条消息")
    print("=" * 60)
    
    if msg_count == 0:
        print("\n❌ 未收到任何消息！")
        print("\n可能的原因:")
        print("  1. vision_pub.py 未运行")
        print("  2. vision_pub.py 使用了不同的端口")
        print("  3. vision_pub.py 使用了不同的 topic 名称")
        print("  4. 防火墙阻止了本地连接")
        print("\n建议:")
        print("  1. 检查 vision_pub.py 是否正在运行:")
        print("     ps aux | grep vision_pub")
        print("  2. 检查端口是否被占用:")
        print("     netstat -tulpn | grep 5555")
        print("  3. 确认 vision_pub.py 中的配置:")
        print("     ZMQ_PORT = 5555")
        print("     topic = 'perception'")
    else:
        print("\n✅ 通信正常！")
    
    sock.close()
    ctx.term()

def test_publisher():
    """测试发布端（模拟 vision_pub.py）"""
    print("=" * 60)
    print("ZMQ 发布端测试")
    print("=" * 60)
    
    port = 5555
    topic = "perception"
    
    print(f"绑定端口: {port}")
    print(f"发布 topic: {topic}")
    
    ctx = zmq.Context()
    sock = ctx.socket(zmq.PUB)
    sock.bind(f"tcp://*:{port}")
    
    print("\n等待订阅者连接... (2秒)")
    time.sleep(2)  # 等待订阅者连接
    
    print("\n开始发送测试消息...")
    print("-" * 60)
    
    try:
        for i in range(5):
            test_data = {
                'cam_idx': 0,
                'class_id': 4,  # PURPLE
                'pattern': 'SOLID',
                'distance': 1.5,
                'bearing_body': 180.0
            }
            
            msg = f"{topic} {json.dumps(test_data)}"
            sock.send_string(msg)
            
            print(f"✅ 发送消息 #{i+1}: {test_data}")
            time.sleep(0.5)
            
    except KeyboardInterrupt:
        print("\n\n用户中断")
    
    print("\n" + "=" * 60)
    print("测试结束")
    print("=" * 60)
    
    sock.close()
    ctx.term()

if __name__ == "__main__":
    import sys
    
    if len(sys.argv) < 2:
        print("用法:")
        print("  测试订阅端: python3 test_zmq_connection.py sub")
        print("  测试发布端: python3 test_zmq_connection.py pub")
        sys.exit(1)
    
    mode = sys.argv[1].lower()
    
    if mode == "sub":
        test_subscriber()
    elif mode == "pub":
        test_publisher()
    else:
        print(f"未知模式: {mode}")
        print("请使用 'sub' 或 'pub'")
        sys.exit(1)

