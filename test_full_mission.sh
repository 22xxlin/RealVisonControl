#!/bin/bash
# 测试 full_mission_controller.py 的启动脚本

echo "=========================================="
echo "测试 Full Mission Controller"
echo "=========================================="
echo ""

# 检查 ZMQ 端口是否有数据
echo "1️⃣ 检查 ZMQ 端口 (tcp://localhost:5555)..."
timeout 2 python3 -c "
import zmq
ctx = zmq.Context()
sock = ctx.socket(zmq.SUB)
sock.connect('tcp://localhost:5555')
sock.setsockopt_string(zmq.SUBSCRIBE, 'perception')
sock.setsockopt(zmq.RCVTIMEO, 2000)
try:
    msg = sock.recv_string()
    print('✅ ZMQ端口有数据:', msg[:100])
except:
    print('⚠️  ZMQ端口无数据 (这是正常的，如果视觉系统未启动)')
"

echo ""
echo "2️⃣ 启动机器人 ID=15 (5秒测试)..."
echo "   预期行为: 应该看到机器人向前移动 (vx=0.2)"
echo ""

timeout 5 python3 ./full_mission_controller.py --id 15

echo ""
echo "=========================================="
echo "测试完成"
echo "=========================================="

