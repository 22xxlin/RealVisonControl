#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
单小车LED控制器
用法: python led_sync_single.py <模式名或ID> <小车ID> [--cycles N] [--frequency Hz]
"""

import os
import json
import argparse
import rospy
from paho.mqtt import client as mqtt_client

# 颜色常量
RED, GREEN, BLUE, PURPLE, GRAY, BLACK = 0x3f0000, 0x003f00, 0x00003f, 0x3f003f, 0x202020, 0x000000

# 计数模式
FLASH, EQUAL, ALWAYS_ON = [6, 2], [2, 6], [8]

# 预设LED模式
LED_PATTERNS = {
    "red_flash": {"name": "红色闪烁", "colors": [RED, BLACK], "counts": FLASH, "cycles": 9, "frequency": 14.4},
    "red_equal": {"name": "红色均等", "colors": [RED, BLACK], "counts": EQUAL, "cycles": 9, "frequency": 14.4},
    "red_on": {"name": "红色常亮", "colors": [RED], "counts": ALWAYS_ON, "cycles": 9, "frequency": 14.4},
    "blue_flash": {"name": "蓝色闪烁", "colors": [BLUE, BLACK], "counts": FLASH, "cycles": 9, "frequency": 14.4},
    "blue_equal": {"name": "蓝色均等", "colors": [BLUE, BLACK], "counts": EQUAL, "cycles": 9, "frequency": 14.4},
    "blue_on": {"name": "蓝色常亮", "colors": [BLUE], "counts": ALWAYS_ON, "cycles": 9, "frequency": 14.4},
    "green_flash": {"name": "绿色闪烁", "colors": [GREEN, BLACK], "counts": FLASH, "cycles": 9, "frequency": 14.4},
    "green_equal": {"name": "绿色均等", "colors": [GREEN, BLACK], "counts": EQUAL, "cycles": 9, "frequency": 14.4},
    "green_on": {"name": "绿色常亮", "colors": [GREEN], "counts": ALWAYS_ON, "cycles": 9, "frequency": 14.4},
    "purple_flash": {"name": "紫色闪烁", "colors": [PURPLE, BLACK], "counts": FLASH, "cycles": 9, "frequency": 14.4},
    "purple_equal": {"name": "紫色均等", "colors": [PURPLE, BLACK], "counts": EQUAL, "cycles": 9, "frequency": 14.4},
    "purple_on": {"name": "紫色常亮", "colors": [PURPLE], "counts": ALWAYS_ON, "cycles": 9, "frequency": 14.4},
    "gray_flash": {"name": "灰色闪烁", "colors": [GRAY, BLACK], "counts": FLASH, "cycles": 9, "frequency": 14.4},
    "gray_equal": {"name": "灰色均等", "colors": [GRAY, BLACK], "counts": EQUAL, "cycles": 9, "frequency": 14.4},
    "gray_on": {"name": "灰色常亮", "colors": [GRAY], "counts": ALWAYS_ON, "cycles": 9, "frequency": 14.4},
    "off": {"name": "关闭", "colors": [BLACK], "counts": ALWAYS_ON, "cycles": 3, "frequency": 14.4}
}

# 模式编号映射
PATTERN_IDS = {
    "red_flash": 1, "red_equal": 2, "red_on": 3,
    "blue_flash": 4, "blue_equal": 5, "blue_on": 6,
    "green_flash": 7, "green_equal": 8, "green_on": 9,
    "purple_flash": 10, "purple_equal": 11, "purple_on": 12,
    "gray_flash": 13, "gray_equal": 14, "gray_on": 15,
    "off": 0,
}
ID_TO_PATTERN = {v: k for k, v in PATTERN_IDS.items()}



def resolve_pattern_key(inp: str):
    """支持模式名字或数字ID，返回 (pattern_key, pattern_id)"""
    s = inp.strip().lower()
    if s.isdigit() or (s.startswith("-") and s[1:].isdigit()):
        pid = int(s)
        if pid not in ID_TO_PATTERN:
            valid_ids = sorted(ID_TO_PATTERN.keys())
            raise SystemExit(f"未知模式ID: {pid}，可选ID: {valid_ids}")
        key = ID_TO_PATTERN[pid]
        return key, pid
    else:
        if s not in LED_PATTERNS:
            choices = ", ".join([f"{name}({PATTERN_IDS.get(name,'?')})" for name in LED_PATTERNS.keys()])
            raise SystemExit(f"未知模式: {inp}，可选: {choices}")
        return s, PATTERN_IDS.get(s, -1)


class LEDController:
    def __init__(self, mqtt_client, robot_id, colors, counts):
        self.mqtt_client = mqtt_client
        self.robot_id = robot_id
        self.colors = colors
        self.counts = counts
        self.current_color_idx = 0
        self.current_count = 0

    def update_color(self):
        self.current_count += 1
        if self.current_count >= self.counts[self.current_color_idx]:
            self.current_color_idx = (self.current_color_idx + 1) % len(self.colors)
            self.current_count = 0

    def publish(self):
        color = self.colors[self.current_color_idx]
        self._send_led("ledup", color, 14)
        self._send_led("leddown", color, 30)

    def _send_led(self, cmd_type, color, brightness):
        msg = {
            "cmd_type": cmd_type,
            "args_length": 6,
            "args": {"0": color, "1": brightness, "2": color, "3": brightness, "4": color, "5": brightness},
        }
        topic = f"/VSWARM{self.robot_id}_robot/cmd"
        self.mqtt_client.publish(topic, json.dumps(msg).encode("utf-8"), qos=1)

    def turn_off(self):
        self._send_led("ledup", BLACK, 14)
        self._send_led("leddown", BLACK, 30)


class MQTTClient:
    def __init__(self, broker, port, client_id):
        self.broker = os.environ.get("REMOTE_SERVER", broker)
        self.client = mqtt_client.Client(client_id)
        self.client.on_connect = lambda c, u, f, rc: print("MQTT连接成功" if rc == 0 else f"MQTT连接失败: {rc}")
        self.client.connect(self.broker, port, 60)
        self.client.loop_start()

    def publish(self, topic, msg, qos=1):
        self.client.publish(topic, msg, qos=qos)


def run_led_control(robot_id, colors, counts, cycles, frequency, broker_ip, port):
    if not rospy.core.is_initialized():
        rospy.init_node("led_controller")
    
    mqtt = MQTTClient(broker_ip, port, f"LED_{robot_id}")
    controller = LEDController(mqtt, robot_id, colors, counts)
    
    rate = rospy.Rate(frequency)
    total_steps = cycles * sum(counts)
    
    for _ in range(total_steps):
        if rospy.is_shutdown():
            break
        controller.update_color()
        controller.publish()
        rate.sleep()
    
    controller.turn_off()
    print("LED控制完成")


def main():
    parser = argparse.ArgumentParser(description="单小车LED控制器")
    parser.add_argument("pattern", type=str, nargs='?', default="6", help="灯语模式名称或数字ID，例如：green_flash 或 1（off=0）")
    parser.add_argument("robot_id", type=int, nargs='?', default=15, help="要控制的小车ID")
    parser.add_argument("--cycles", type=int, default=None, help="循环次数（默认使用模式内置值）")
    parser.add_argument("--frequency", type=int, default=None, help="频率Hz（默认使用模式内置值）")
    parser.add_argument("--broker", type=str, default="10.0.2.66", help="MQTT Broker 地址")
    parser.add_argument("--port", type=int, default=1883, help="MQTT 端口")

    args = parser.parse_args()

    # 解析模式
    key, pid = resolve_pattern_key(args.pattern)
    cfg = LED_PATTERNS[key]

    cycles = args.cycles if args.cycles is not None else cfg["cycles"]
    frequency = args.frequency if args.frequency is not None else cfg["frequency"]

    # 显示信息
    print(f"单小车LED控制器")
    print("=" * 50)
    print(f"模式ID: {pid} | 模式名称: {cfg['name']}")
    print(f"控制小车: {args.robot_id}")
    print(f"循环次数: {cycles} | 频率: {frequency}Hz")
    print(f"⏱️ 预计持续时间: {cycles * sum(cfg['counts']) / frequency:.1f}秒")
    print("=" * 50)

    run_led_control(
        robot_id=args.robot_id,
        colors=cfg["colors"],
        counts=cfg["counts"],
        cycles=cycles,
        frequency=frequency,
        broker_ip=args.broker,
        port=args.port,
    )


if __name__ == "__main__":
    main()

