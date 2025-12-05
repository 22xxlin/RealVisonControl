#!/usr/bin/env python3
"""
机器人共享状态模块 - 线程安全的状态管理
类似 ROS 话题的生产者-消费者模式
"""

import time
import threading
from typing import Optional, Dict, Any


class RobotState:
    """
    线程安全的机器人状态类
    
    存储感知系统的最新信息（距离、角度、TrackID、指令等）
    供控制线程读取，视觉线程写入
    """
    
    def __init__(self):
        """初始化共享状态"""
        self._lock = threading.Lock()
        
        # 最新的目标信息
        self._latest_target_info = None  # Dict: {dist, angle, bearing_body, track_id, cam_idx, class_id}
        self._last_update_time = 0.0  # 时间戳
        
        # 当前执行的指令
        self._current_command = 'IDLE'  # 'IDLE', 'APPROACH', 'RETREAT', 'FORWARD', 'STOP', 'S_SHAPE', 'CIRCLE'
        self._command_params = {}  # 指令相关参数（如 pattern, action_name, action_desc）
        
        # 系统状态
        self._is_executing = False  # 是否正在执行动作
        self._last_detection_time = 0.0  # 最后一次检测到目标的时间
    
    def update_perception(self, 
                         distance: float, 
                         azimuth: float, 
                         bearing_body: float,
                         track_id: int,
                         cam_idx: int,
                         command: str = 'IDLE',
                         command_params: Optional[Dict[str, Any]] = None,
                         class_id: int = 0):
        """
        更新感知数据（视觉线程调用）
        
        Args:
            distance: 目标距离（米）
            azimuth: 相机坐标系的方位角（度）
            bearing_body: 机体坐标系的方位角（度）
            track_id: 目标跟踪ID
            cam_idx: 摄像头索引
            command: 识别到的指令
            command_params: 指令参数
            class_id: 目标类别ID
        """
        with self._lock:
            self._latest_target_info = {
                'distance': distance,
                'azimuth': azimuth,
                'bearing_body': bearing_body,
                'track_id': track_id,
                'cam_idx': cam_idx,
                'class_id': class_id
            }
            self._last_update_time = time.time()
            self._last_detection_time = time.time()
            
            if command != 'IDLE':
                self._current_command = command
                self._command_params = command_params if command_params else {}
    
    def get_latest_state(self) -> Dict[str, Any]:
        """
        获取最新状态（控制线程调用）
        
        Returns:
            包含所有状态信息的字典
        """
        with self._lock:
            return {
                'target_info': self._latest_target_info.copy() if self._latest_target_info else None,
                'command': self._current_command,
                'command_params': self._command_params.copy(),
                'last_update_time': self._last_update_time,
                'last_detection_time': self._last_detection_time,
                'is_executing': self._is_executing,
                'time_since_update': time.time() - self._last_update_time if self._last_update_time > 0 else float('inf')
            }
    
    def clear_command(self):
        """清除当前指令（控制线程在执行完成后调用）"""
        with self._lock:
            self._current_command = 'IDLE'
            self._command_params = {}
    
    def set_executing(self, is_executing: bool):
        """设置是否正在执行动作"""
        with self._lock:
            self._is_executing = is_executing
    
    def is_target_valid(self, timeout: float = 1.0) -> bool:
        """
        检查目标信息是否有效（未超时）
        
        Args:
            timeout: 超时时间（秒）
        
        Returns:
            True if 目标信息有效
        """
        with self._lock:
            if self._latest_target_info is None:
                return False
            time_since_update = time.time() - self._last_update_time
            return time_since_update < timeout
    
    def get_target_info(self) -> Optional[Dict[str, Any]]:
        """仅获取目标信息"""
        with self._lock:
            return self._latest_target_info.copy() if self._latest_target_info else None
    
    def get_command(self) -> tuple:
        """获取当前指令和参数"""
        with self._lock:
            return self._current_command, self._command_params.copy()
    
    def reset(self):
        """重置所有状态"""
        with self._lock:
            self._latest_target_info = None
            self._current_command = 'IDLE'
            self._command_params = {}
            self._is_executing = False
            self._last_update_time = 0.0
            self._last_detection_time = 0.0

