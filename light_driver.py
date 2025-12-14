#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
文件: light_driver.py
功能: 封装 MQTT 灯光控制，独立线程，包含 ledup 和 leddown 双指令
"""

import json
import time
import threading
from paho.mqtt import client as mqtt_client

# --- 1. 定义颜色和模式 ---
RED, GREEN, BLUE, PURPLE, GRAY, BLACK = 0x3f0000, 0x003f00, 0x00003f, 0x3f003f, 0x202020, 0x000000
FLASH, EQUAL, ALWAYS_ON = [6, 2], [2, 6], [8] 

LED_PATTERNS = {
    # 状态映射
    "SEARCH":  {"colors": [PURPLE, BLACK], "counts": FLASH, "freq": 14.4}, 
    "FOUND":   {"colors": [BLUE, BLACK],   "counts": FLASH, "freq": 14.4}, 
    "ARRIVED": {"colors": [GREEN],         "counts": ALWAYS_ON, "freq": 14.4}, 
    "OFF":     {"colors": [BLACK],         "counts": ALWAYS_ON, "freq": 14.4}, 
    "IDLE":    {"colors": [GRAY, BLACK],   "counts": FLASH, "freq": 14.4}, 
}

class LightDriver:
    def __init__(self, robot_id, broker_ip="10.0.2.66", port=1883):
        self.robot_id = robot_id
        self.topic = f"/VSWARM{self.robot_id}_robot/cmd"
        
        # MQTT 连接
        self.client = mqtt_client.Client(f"LightDriver_{robot_id}")
        try:
            self.client.connect(broker_ip, port, 60)
            self.client.loop_start() 
            print(f"💡 [Light] 灯光驱动就绪 | 目标: {broker_ip}")
        except Exception as e:
            print(f"⚠️ [Light] MQTT连接失败: {e}")

        # 线程控制变量
        self.current_pattern = LED_PATTERNS["OFF"] 
        self.running = True
        self.lock = threading.Lock()
        
        # 启动后台线程
        self.thread = threading.Thread(target=self._led_loop, daemon=True)
        self.thread.start()

    def set_cmd(self, cmd_name):
        """主程序调用接口"""
        if cmd_name not in LED_PATTERNS:
            print(f"⚠️ 未知灯光指令: {cmd_name}")
            return

        new_pattern = LED_PATTERNS[cmd_name]
        with self.lock:
            if self.current_pattern != new_pattern:
                self.current_pattern = new_pattern

    def stop(self):
        """停止驱动"""
        self.set_cmd("OFF")
        time.sleep(0.5) 
        self.running = False
        self.client.loop_stop()

    def _led_loop(self):
        """后台线程：处理闪烁逻辑"""
        color_idx = 0
        count_tick = 0
        
        while self.running:
            with self.lock:
                pattern = self.current_pattern
            
            colors = pattern["colors"]
            counts = pattern["counts"]
            freq = pattern["freq"]
            
            # 计数逻辑
            count_tick += 1
            if count_tick >= counts[color_idx % len(counts)]:
                color_idx = (color_idx + 1) % len(colors)
                count_tick = 0
            
            current_color = colors[color_idx % len(colors)]

            # 发送双指令
            self._send_protocol(current_color)

            time.sleep(1.0 / freq)

    def _send_protocol(self, color):
        """
        此处恢复了原始逻辑：
        1. 发送 ledup (亮度 14)
        2. 发送 leddown (亮度 30)
        """
        self._publish_one("ledup", color, 14)
        self._publish_one("leddown", color, 30)

    def _publish_one(self, cmd_type, color, brightness):
        msg = {
            "cmd_type": cmd_type,
            "args_length": 6,
            "args": {
                "0": color, "1": brightness, 
                "2": color, "3": brightness, 
                "4": color, "5": brightness
            },
        }
        # 使用 qos=0 保证控制回路不阻塞，原始代码是qos=1，但在高频控制中0更流畅
        self.client.publish(self.topic, json.dumps(msg), qos=0)