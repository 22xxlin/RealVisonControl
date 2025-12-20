#!/bin/bash
# 测试 transport_node.py 的调试输出

echo "=========================================="
echo "启动 transport_node.py (调试模式)"
echo "=========================================="
echo ""
echo "参数说明:"
echo "  --role: leader (领队) 或 follower (跟随)"
echo "  --robot-id: 机器人ID"
echo "  --perception-timeout: 感知超时时间 (默认0.5秒，建议改大)"
echo ""
echo "当前配置:"
echo "  - 角色: leader"
echo "  - ID: 15"
echo "  - 超时: 5.0 秒 (放宽以便调试)"
echo ""
echo "=========================================="
echo ""

cd /home/nvidia/Downloads/Ros/pseudo_ros_architecture

python3 transport_node.py \
    --role leader \
    --robot-id 15 \
    --perception-timeout 5.0 \
    --delay 0.0 \
    --prewarm 0.0 \
    --speed 0.0 \
    --heading-deg 0.0 \
    --move-sec 5.0

