# light_driver.py 修复说明

## 问题原因
旧的 `light_driver.py` 使用了错误的 MQTT 消息格式：
- ❌ 旧格式：发送 RGBA 列表 `[r, g, b, 1.0]`（0-1范围的浮点数）
- ✅ 新格式：直接发送十六进制颜色值（如 `0x3f0000`）

## 主要改动

### 1. 删除了颜色转换函数
```python
# 删除了这个函数（不需要）
def _convert_color(self, hex_int):
    r = ((hex_int >> 16) & 0xff) / 256.0
    g = ((hex_int >> 8) & 0xff) / 256.0
    b = (hex_int & 0xff) / 256.0
    return [r, g, b, 1.0]
```

### 2. 简化了消息发送
```python
# 新的实现：直接发送十六进制颜色值
def _send_led(self, cmd_type, color, brightness):
    msg = {
        "cmd_type": cmd_type,
        "args_length": 6,
        "args": {"0": color, "1": brightness, "2": color, "3": brightness, "4": color, "5": brightness},
    }
    self.client.publish(topic, json.dumps(msg).encode("utf-8"), qos=1)
```

### 3. 其他优化
- 添加了 `REMOTE_SERVER` 环境变量支持
- 简化了连接状态处理
- 改进了线程停止逻辑

## 使用方法
```python
from light_driver import LightDriver

# 初始化
light = LightDriver(robot_id=15, broker_ip="10.0.2.66")

# 切换模式
light.set_cmd("SEARCH")   # 紫色慢闪
light.set_cmd("FOUND")    # 蓝色快闪
light.set_cmd("ARRIVED")  # 绿色常亮
light.set_cmd("OFF")      # 关闭

# 停止
light.stop()
```

## 测试
运行测试脚本：
```bash
python test_light_standalone.py
```

