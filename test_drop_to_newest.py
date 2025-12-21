#!/usr/bin/env python3
"""
测试脚本：验证 Drop to Newest 策略是否生效

使用方法：
1. 启动 vision_pub.py
2. 运行本脚本: python3 test_drop_to_newest.py
3. 观察输出，验证是否只处理最新帧

预期结果：
- 如果积压了多帧，应该只看到最新帧的数据
- 时间戳应该是连续的，不会出现"瞬间收到多帧"的情况
"""

import zmq
import json
import time

def test_receiver():
    ctx = zmq.Context()
    sock = ctx.socket(zmq.SUB)
    sock.connect("tcp://127.0.0.1:5555")
    sock.setsockopt_string(zmq.SUBSCRIBE, "perception")
    
    print("🧪 开始测试 Drop to Newest 策略...")
    print("=" * 60)
    
    frame_count = 0
    
    try:
        while True:
            # 模拟卡顿：每隔一段时间休眠，让数据积压
            if frame_count % 5 == 0 and frame_count > 0:
                print(f"\n⏸️  模拟卡顿 0.3 秒 (让数据积压)...\n")
                time.sleep(0.3)
            
            # 接收数据 (使用 Drop to Newest 策略)
            batch = []
            latest_payload = None
            received_count = 0
            
            while True:
                try:
                    s = sock.recv_string(flags=zmq.NOBLOCK)
                    _, payload_str = s.split(" ", 1)
                    parsed_json = json.loads(payload_str)
                    
                    received_count += 1  # 统计收到了多少个包
                    
                    if 'objects' in parsed_json:
                        latest_payload = parsed_json['objects']
                    else:
                        latest_payload = [parsed_json]
                        
                except zmq.Again:
                    break
                except Exception as e:
                    print(f"❌ 解析错误: {e}")
                    pass
            
            if latest_payload is not None:
                batch = latest_payload
                frame_count += 1
                
                # 打印结果
                timestamp = time.time()
                print(f"📦 帧 #{frame_count} | 时间: {timestamp:.3f}")
                print(f"   收到包数: {received_count} 个")
                print(f"   处理物体: {len(batch)} 个")
                
                if received_count > 1:
                    print(f"   ⚠️  检测到积压! 收到 {received_count} 个包，但只处理最新的 1 个 ✅")
                
                for obj in batch:
                    print(f"   - Cam{obj.get('cam_idx')}: "
                          f"Class={obj.get('class_id')} "
                          f"Dist={obj.get('distance'):.2f}m "
                          f"Ang={obj.get('bearing_body'):.1f}°")
                print()
            
            time.sleep(0.05)  # 正常处理间隔
            
    except KeyboardInterrupt:
        print("\n🛑 测试结束")
    finally:
        sock.close()
        ctx.term()

if __name__ == "__main__":
    test_receiver()

