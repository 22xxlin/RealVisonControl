#!/usr/bin/env python3
"""
伪ROS架构 - 多线程感知与控制解耦系统

模块：
- RobotState: 线程安全的共享状态
- VisionNode: 视觉感知节点（发布者）
- ControlNode: 机器人控制节点（订阅者）
- PseudoROSSystem: 主系统管理器
"""

from .robot_shared_state import RobotState
from .vision_node import VisionNode
from .control_node import ControlNode

__version__ = '1.0.0'
__all__ = ['RobotState', 'VisionNode', 'ControlNode']

