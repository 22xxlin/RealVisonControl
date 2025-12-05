#!/bin/bash
# 快速启动脚本 - ZeroMQ 分布式架构

echo "============================================================"
echo "🚀 ZeroMQ 分布式架构 - 快速启动"
echo "============================================================"
echo ""
echo "请选择启动模式："
echo ""
echo "  1) 测试 ZeroMQ 连接（推荐首次使用）"
echo "  2) 启动视觉发布者"
echo "  3) 启动控制订阅者"
echo "  4) 查看使用指南"
echo "  5) 退出"
echo ""
read -p "请输入选项 [1-5]: " choice

case $choice in
    1)
        echo ""
        echo "🧪 运行 ZeroMQ 连接测试..."
        echo "============================================================"
        python3 test_zmq_connection.py
        ;;
    2)
        echo ""
        echo "🎥 启动视觉发布者..."
        echo "============================================================"
        echo "💡 提示：在另一个终端运行 './quick_start.sh' 并选择选项 3"
        echo ""
        python3 vision_pub.py
        ;;
    3)
        echo ""
        echo "🤖 启动控制订阅者..."
        echo "============================================================"
        echo "💡 提示：确保视觉发布者已在另一个终端运行"
        echo ""
        python3 control_sub.py
        ;;
    4)
        echo ""
        echo "📖 使用指南"
        echo "============================================================"
        echo ""
        cat << 'EOF'
快速使用步骤：

1. 首次使用：
   ./quick_start.sh
   选择 1 - 测试 ZeroMQ 连接

2. 正式运行：
   
   终端1：
   ./quick_start.sh
   选择 2 - 启动视觉发布者
   
   终端2：
   ./quick_start.sh
   选择 3 - 启动控制订阅者

3. 配置修改：
   - 摄像头：编辑 vision_pub.py 第 332 行
   - 机器人ID：编辑 control_sub.py 第 357 行
   - Mock模式：编辑 control_sub.py 第 358 行

4. 详细文档：
   - 使用指南：cat ZMQ_DISTRIBUTED_GUIDE.md
   - 重构总结：cat MIGRATION_SUMMARY.md

5. 停止运行：
   按 Ctrl+C

EOF
        echo ""
        read -p "按 Enter 键返回..."
        ./quick_start.sh
        ;;
    5)
        echo ""
        echo "👋 再见！"
        exit 0
        ;;
    *)
        echo ""
        echo "❌ 无效选项，请重新运行"
        exit 1
        ;;
esac

