# 兼容性指南 - 架构解耦后的数据格式变更

## ⚠️ 重要提示

架构解耦后，`vision_pub.py` 发送的数据格式已变更！

---

## 📊 数据格式对比

### ❌ 旧格式（解耦前）
```json
{
  "distance": 1.5,
  "azimuth": 45.0,
  "bearing_body": 225.0,
  "track_id": 3,
  "cam_idx": 4,
  "command": "APPROACH",        // ❌ 已移除
  "description": "靠近",         // ❌ 已移除
  "class_id": 2,
  "timestamp": 1234567890.0
}
```

### ✅ 新格式（解耦后）
```json
{
  "distance": 1.5,
  "azimuth": 45.0,
  "bearing_body": 225.0,
  "track_id": 3,
  "cam_idx": 4,
  "pattern": "2200",            // ✅ 新增：原始 Pattern
  "class_id": 2,
  "timestamp": 1234567890.0
}
```

---

## 🔧 需要更新的文件

### ✅ 已更新
- [x] `vision_pub.py` - 发送新格式
- [x] `control_sub.py` - 接收新格式并翻译
- [x] `test_vision_pub.py` - 测试脚本已更新

### ⚠️ 可能需要更新的文件
如果你有其他订阅 `vision_pub.py` 的脚本，需要手动更新：

```python
# ❌ 旧代码
command = data.get('command', 'IDLE')
description = data.get('description', '未知')

# ✅ 新代码
pattern = data.get('pattern', 'IDLE')

# 如果需要翻译成 Command，使用映射表：
PATTERN_TO_COMMAND = {
    '220': 'FORWARD', '330': 'LEFT', '110': 'RIGHT', 
    '550': 'REVERSE', '440': 'STOP',
    '2200': 'APPROACH', '1100': 'RETREAT', 
    '4400': 'S_SHAPE', '5500': 'CIRCLE',
    # ... 其他映射
}
command = PATTERN_TO_COMMAND.get(pattern, 'IDLE')
```

---

## 🧪 测试兼容性

### 测试 1: 使用更新后的 test_vision_pub.py

**终端 1 - 启动 Vision 端**
```bash
python3 vision_pub.py
```

**终端 2 - 运行测试脚本**
```bash
python3 test_vision_pub.py
```

**预期输出**：
```
📥 [Cam4] Received Pattern: '2200' | Dist=1.50m | Bearing=45.0° | TrackID=3
```

### 测试 2: 使用 control_sub.py（完整决策链）

**终端 1 - 启动 Vision 端**
```bash
python3 vision_pub.py
```

**终端 2 - 启动 Control 端**
```bash
python3 control_sub.py
```

**预期输出**：
```
Vision 端:
  🎥 [Cam4] Sent Pattern: '2200' | Dist=1.50m | Bearing=45.0° | TrackID=3

Control 端:
  🧠 Decision: Pattern '2200' -> Action 'APPROACH' (靠近)
  ➡️ APPROACH: 误差1.00m | 方向45.0° | vx=0.289, vy=0.289
```

---

## 📝 迁移检查清单

如果你有自定义的订阅者脚本，请按以下步骤检查：

- [ ] 检查是否使用了 `data.get('command')`
- [ ] 检查是否使用了 `data.get('description')`
- [ ] 将 `data.get('command')` 替换为 `data.get('pattern')`
- [ ] 如果需要 Command，添加 `PATTERN_TO_COMMAND` 映射表
- [ ] 测试新脚本是否能正常接收数据

---

## 🔍 快速诊断

### 问题：订阅者收不到数据

**检查 1：ZMQ 连接**
```python
# 确保订阅正确的 topic
socket.setsockopt_string(zmq.SUBSCRIBE, "perception")
```

**检查 2：数据解析**
```python
# 打印原始数据，查看实际格式
message = socket.recv_string()
print(f"Raw message: {message}")
topic, data_str = message.split(' ', 1)
data = json.loads(data_str)
print(f"Parsed data: {data}")
```

### 问题：找不到 'command' 字段

**原因**：使用了旧代码

**解决方案**：
```python
# ❌ 旧代码会报错
command = data['command']  # KeyError: 'command'

# ✅ 新代码
pattern = data.get('pattern', 'IDLE')
```

---

## 📚 相关文档

- [ARCHITECTURE_REFACTORING.md](ARCHITECTURE_REFACTORING.md) - 架构解耦详细说明
- [QUICK_REFERENCE.md](QUICK_REFERENCE.md) - 快速参考指南
- [test_architecture.py](test_architecture.py) - 单元测试脚本

---

## 💡 最佳实践

### 1. 向后兼容的订阅者（可选）

如果你想让订阅者同时支持新旧格式：

```python
# 兼容新旧格式
pattern = data.get('pattern')
command = data.get('command')

if pattern:
    # 新格式：自己翻译
    PATTERN_TO_COMMAND = {...}
    command = PATTERN_TO_COMMAND.get(pattern, 'IDLE')
    print(f"📥 Received Pattern: '{pattern}' -> Command: '{command}'")
elif command:
    # 旧格式：直接使用
    print(f"📥 Received Command: '{command}'")
else:
    print("⚠️ Unknown format")
```

### 2. 日志输出建议

```python
# Vision 端日志（显示 Pattern）
print(f"🎥 [Cam{cam_idx}] Sent Pattern: '{pattern}' | ...")

# Control 端日志（显示决策过程）
print(f"🧠 Decision: Pattern '{pattern}' -> Action '{command}' | ...")

# 测试脚本日志（显示接收到的数据）
print(f"📥 [Cam{cam_idx}] Received Pattern: '{pattern}' | ...")
```

---

## ✅ 验证完成

- [x] `test_vision_pub.py` 已更新，可以正常接收新格式数据
- [x] `control_sub.py` 已更新，可以翻译 Pattern -> Command
- [x] 提供了兼容性检查清单
- [x] 提供了快速诊断方法

**结论**：所有相关文件已更新，系统可以正常运行！

