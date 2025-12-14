#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
文件: light_driver.py
功能: 最终版协议 (RGB阵型 + 紫灰搬运)
"""

import json
import time
import threading
from paho.mqtt import client as mqtt_client

# --- 1. 颜色定义 ---
RED    = 0x3f0000
GREEN  = 0x003f00
BLUE   = 0x00003f
PURPLE = 0x3f003f
GRAY   = 0x202020  # 灰色/白光
BLACK  = 0x000000

# --- 2. 计数器 ---
FLASH = [6, 2]     # 闪烁 (过程态)
SOLID = [100]      # 常亮 (稳定态)

# --- 3. 最终协议表 ---
LED_PATTERNS = {
    "OFF":           {"colors": [BLACK],         "counts": SOLID, "freq": 14.4},
    
    # === 1. 搜索 (全员紫闪) ===
    "SEARCH":        {"colors": [PURPLE, BLACK], "counts": FLASH, "freq": 14.4},
    
    # === 2. 发现 (红闪=靠近, 红亮=锚点) ===
    "APPROACH_BALL": {"colors": [RED, BLACK],    "counts": FLASH, "freq": 14.4}, # 新增: 靠近中
    "LEADER_WAIT":   {"colors": [RED],           "counts": SOLID, "freq": 14.4}, # 锚点
    
    # === 3. 竞标 (绿闪=左, 蓝闪=右) ===
    "BID_LEFT":      {"colors": [GREEN, BLACK],  "counts": FLASH, "freq": 14.4},
    "BID_RIGHT":     {"colors": [BLUE, BLACK],   "counts": FLASH, "freq": 14.4},
    
    # === 4. 锁定 (绿亮=左, 蓝亮=右) ===
    "LOCK_LEFT":     {"colors": [GREEN],         "counts": SOLID, "freq": 14.4},
    "LOCK_RIGHT":    {"colors": [BLUE],          "counts": SOLID, "freq": 14.4},
    
    # === 5. 搬运 (紫亮=GO, 灰亮=Push) ===
    "LEADER_GO":     {"colors": [PURPLE],        "counts": SOLID, "freq": 14.4}, # Leader变紫常亮
    "FOLLOWER_PUSH": {"colors": [GRAY],          "counts": SOLID, "freq": 14.4}, # Follower变灰常亮
}

class LightDriver:
    def __init__(self, robot_id, broker_ip="10.0.2.66", port=1883):
        self.robot_id = robot_id
        self.topic = f"/VSWARM{self.robot_id}_robot/cmd"
        
        self.client = mqtt_client.Client(f"LightDriver_{robot_id}")
        try:
            self.client.connect(broker_ip, port, 60)
            self.client.loop_start() 
            print(f"💡 [Light] 驱动就绪 (ID:{robot_id})")
        except Exception as e:
            print(f"⚠️ [Light] MQTT连接失败: {e}")

        self.current_pattern = LED_PATTERNS["OFF"] 
        self.running = True
        self.lock = threading.Lock()
        self.thread = threading.Thread(target=self._led_loop, daemon=True)
        self.thread.start()

    def set_cmd(self, cmd_name):
        if cmd_name not in LED_PATTERNS: return
        new_pattern = LED_PATTERNS[cmd_name]
        with self.lock:
            if self.current_pattern != new_pattern:
                self.current_pattern = new_pattern

    def stop(self):
        self.set_cmd("OFF")
        time.sleep(0.5) 
        self.running = False
        self.client.loop_stop()

    def _led_loop(self):
        color_idx = 0
        current_count = 0
        while self.running:
            with self.lock:
                pattern = self.current_pattern
            
            colors = pattern["colors"]
            counts = pattern["counts"]
            
            current_count += 1
            if current_count >= counts[color_idx % len(counts)]:
                color_idx = (color_idx + 1) % len(colors)
                current_count = 0
            
            current_color = colors[color_idx % len(colors)]
            self._send_protocol(current_color)
            time.sleep(1.0 / pattern["freq"])

    def _send_protocol(self, color):
        self._publish_one("ledup", color, 14)
        self._publish_one("leddown", color, 30)

    def _publish_one(self, cmd_type, color, brightness):
        msg = {
            "cmd_type": cmd_type, "args_length": 6,
            "args": {"0": color, "1": brightness, "2": color, "3": brightness, "4": color, "5": brightness},
        }
        self.client.publish(self.topic, json.dumps(msg), qos=0)