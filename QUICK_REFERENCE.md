# 快速参考指南 - 架构解耦

## 🎯 核心变更总结

### Vision 端 (vision_pub.py)
```python
# ❌ 旧代码
command = self.recognize_pattern(cam_idx, track_id)  # 返回 'APPROACH'
detection_data = {
    'command': command,
    'description': self.ACTION_DESCRIPTIONS.get(command, '未知')
}

# ✅ 新代码
pattern = self.recognize_pattern(cam_idx, track_id)  # 返回 '2200'
detection_data = {
    'pattern': pattern  # 只发送原始 Pattern
}
```

### Control 端 (control_sub.py)
```python
# ❌ 旧代码
command = data.get('command', 'IDLE')  # 直接使用 Vision 端的翻译结果

# ✅ 新代码
pattern = data.get('pattern', 'IDLE')  # 提取原始 Pattern
command = self.PATTERN_TO_COMMAND.get(pattern, 'IDLE')  # 自己翻译
print(f"🧠 Decision: Pattern '{pattern}' -> Action '{command}'")
```

---

## 📊 数据流对比

### 旧数据流
```
Vision: 检测 -> 识别 Pattern '2200' -> 翻译成 'APPROACH' -> 发送 'APPROACH'
                                        ↑ 翻译在这里
Control: 接收 'APPROACH' -> 执行
```

### 新数据流
```
Vision: 检测 -> 识别 Pattern '2200' -> 发送 '2200'
Control: 接收 '2200' -> 翻译成 'APPROACH' -> 执行
                        ↑ 翻译在这里
```

---

## 🔍 关键代码位置

| 文件 | 行号 | 变更内容 |
|------|------|----------|
| `vision_pub.py` | 104-106 | 移除 `PATTERN_TO_COMMAND` 和 `ACTION_DESCRIPTIONS` |
| `vision_pub.py` | 185-186 | `recognize_pattern` 返回原始 Pattern |
| `vision_pub.py` | 282-297 | 数据包包含 `'pattern'` 而非 `'command'` |
| `vision_pub.py` | 369-376 | 日志显示 Pattern 而非 Command |
| `control_sub.py` | 52-80 | 添加 `PATTERN_TO_COMMAND` 映射表 |
| `control_sub.py` | 348-366 | Pattern -> Command 翻译逻辑 |

---

## 🧪 测试方法

### 1. 单元测试
```bash
python3 test_architecture.py
```

### 2. 集成测试

**终端 1 - 启动 Vision 端**
```bash
python3 vision_pub.py
```
预期输出：
```
🎥 [Cam4] Sent Pattern: '2200' | Dist=1.50m | Bearing=45.0° | TrackID=3
```

**终端 2 - 启动 Control 端**
```bash
python3 control_sub.py
```
预期输出：
```
🧠 Decision: Pattern '2200' -> Action 'APPROACH' (靠近)
➡️ APPROACH: 误差1.00m | 方向45.0° | vx=0.289, vy=0.289
```

---

## 📋 Pattern 映射表

| Pattern | Command | 描述 | 类型 |
|---------|---------|------|------|
| `220` | `FORWARD` | 前进 | 基本 (3位) |
| `330` | `LEFT` | 左移 | 基本 (3位) |
| `110` | `RIGHT` | 右移 | 基本 (3位) |
| `550` | `REVERSE` | 后退 | 基本 (3位) |
| `440` | `STOP` | 停止 | 基本 (3位) |
| `2200` | `APPROACH` | 靠近 | 高级 (4位) |
| `1100` | `RETREAT` | 远离 | 高级 (4位) |
| `4400` | `S_SHAPE` | S形轨迹 | 高级 (4位) |
| `5500` | `CIRCLE` | 圆形轨迹 | 高级 (4位) |
| `1111` | `FORWARD` | 前进 | 连续 (4位) |
| `2222` | `LEFT` | 左移 | 连续 (4位) |
| `3333` | `RIGHT` | 右移 | 连续 (4位) |
| `4444` | `STOP` | 停止 | 连续 (4位) |
| `5555` | `REVERSE` | 后退 | 连续 (4位) |

---

## 🛠️ 故障排查

### 问题 1: Control 端收不到数据
**检查**：
- Vision 端是否正常发送？查看日志是否有 `Sent Pattern: 'xxx'`
- ZMQ 地址是否一致？默认 `tcp://localhost:5555`

### 问题 2: Pattern 无法识别
**检查**：
- Control 端日志是否显示 `Pattern 'xxx' -> Action 'IDLE'`？
- 检查 `PATTERN_TO_COMMAND` 是否包含该 Pattern

### 问题 3: 机器人不执行动作
**检查**：
- Control 端是否启用了实际控制？`ENABLE_CONTROL = True`
- 机器人控制模块是否加载成功？查看启动日志

---

## 📚 相关文档

- [ARCHITECTURE_REFACTORING.md](ARCHITECTURE_REFACTORING.md) - 详细架构文档
- [test_architecture.py](test_architecture.py) - 单元测试脚本
- [vision_pub.py](vision_pub.py) - Vision 端源码
- [control_sub.py](control_sub.py) - Control 端源码

