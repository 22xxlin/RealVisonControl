#!/usr/bin/env python3
"""
主系统 - 多线程"伪ROS"架构入口
组装视觉节点、控制节点和共享状态，实现感知与行动的解耦
"""

import os
import sys
import time
import signal

# 导入自定义模块
from robot_shared_state import RobotState
from vision_node import VisionNode
from control_node import ControlNode


class PseudoROSSystem:
    """
    伪ROS系统 - 单进程多线程架构
    
    组件：
    1. RobotState - 共享状态（话题）
    2. VisionNode - 视觉线程（发布者）
    3. ControlNode - 控制线程（订阅者）
    """
    
    def __init__(self, model_path, robot_id=8, enable_control=True, camera_indices=[0, 2, 4, 6]):
        """
        初始化伪ROS系统
        
        Args:
            model_path: YOLO 模型路径
            robot_id: 机器人ID
            enable_control: 是否启用实际控制
            camera_indices: 摄像头索引列表
        """
        print("="*60)
        print("🚀 初始化伪ROS系统（多线程架构）")
        print("="*60)
        
        # 1. 初始化共享状态（话题）
        print("\n1️⃣ 初始化共享状态...")
        self.robot_state = RobotState()
        
        # 2. 初始化视觉节点（发布者）
        print("\n2️⃣ 初始化视觉节点...")
        self.vision_node = VisionNode(
            robot_state=self.robot_state,
            model_path=model_path,
            camera_indices=camera_indices
        )
        
        # 3. 初始化控制节点（订阅者）
        print("\n3️⃣ 初始化控制节点...")
        self.control_node = ControlNode(
            robot_state=self.robot_state,
            robot_id=robot_id,
            enable_control=enable_control,
            control_hz=20.0  # 20Hz 控制频率
        )
        
        # 系统运行标志
        self.running = False
        
        print("\n" + "="*60)
        print("✅ 伪ROS系统初始化完成")
        print("="*60)
    
    def start(self):
        """启动系统"""
        if self.running:
            print("⚠️ 系统已在运行")
            return
        
        print("\n🚀 启动伪ROS系统...")
        print("="*60)
        
        # 启动视觉节点
        print("\n📹 启动视觉节点...")
        self.vision_node.start()
        time.sleep(2.0)  # 等待视觉线程启动
        
        # 启动控制节点
        print("\n🤖 启动控制节点...")
        self.control_node.start()
        
        self.running = True
        
        print("\n" + "="*60)
        print("✅ 系统运行中 - 感知与控制已解耦")
        print("   📹 视觉线程: 持续检测 + 更新共享状态")
        print("   🤖 控制线程: 读取状态 + 执行动作")
        print("="*60)
    
    def stop(self):
        """停止系统"""
        if not self.running:
            return
        
        print("\n🛑 停止伪ROS系统...")
        print("="*60)
        
        # 停止控制节点（先停止动作）
        print("\n🤖 停止控制节点...")
        self.control_node.stop()
        
        # 停止视觉节点
        print("\n📹 停止视觉节点...")
        self.vision_node.stop()
        
        # 重置共享状态
        self.robot_state.reset()
        
        self.running = False
        
        print("\n" + "="*60)
        print("✅ 系统已停止")
        print("="*60)
    
    def run_forever(self):
        """持续运行（阻塞）"""
        self.start()
        
        try:
            print("\n💡 系统正在运行，按 Ctrl+C 停止...")
            while self.running:
                time.sleep(1.0)
                
                # 可选：定期打印系统状态
                state = self.robot_state.get_latest_state()
                if state['target_info']:
                    target = state['target_info']
                    print(f"📊 [状态] 距离:{target['distance']:.2f}m | "
                          f"角度:{target['bearing_body']:.1f}° | "
                          f"指令:{state['command']}")
        
        except KeyboardInterrupt:
            print("\n⚠️ 收到中断信号")
        finally:
            self.stop()


def check_environment():
    """检查环境依赖"""
    print("🔍 检查环境依赖...")
    try:
        import cv2
        import numpy as np
        from ultralytics import YOLO
        print("✅ 依赖检查通过")
        return True
    except ImportError as e:
        print(f"❌ 依赖检查失败: {e}")
        return False


def main():
    """主函数"""
    print("\n" + "="*60)
    print("🎥 多线程伪ROS架构 - 感知与控制解耦系统")
    print("="*60)
    
    # 环境检查
    if not check_environment():
        return
    
    # 模型路径
    model_path = '/home/nvidia/Downloads/Ros/ballCar2/weights/weights/best.engine'
    
    if not os.path.exists(model_path):
        print(f"❌ 模型文件不存在: {model_path}")
        return
    
    print(f"✅ 模型文件: {model_path}")
    
    # 创建系统
    try:
        system = PseudoROSSystem(
            model_path=model_path,
            robot_id=8,
            enable_control=True,  # 设置为 False 进入 Mock 模式
            camera_indices=[0, 2, 4, 6]
        )
        
        # 运行系统
        system.run_forever()
    
    except Exception as e:
        print(f"❌ 系统错误: {e}")
        import traceback
        traceback.print_exc()
    
    print("\n👋 程序退出")


if __name__ == '__main__':
    main()

