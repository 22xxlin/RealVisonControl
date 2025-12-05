# 架构解耦前后对比

## 📊 数据流对比

### ❌ 解耦前（耦合架构）

```
┌─────────────────────────────────────┐
│         vision_pub.py               │
│                                     │
│  1. 摄像头采集                      │
│  2. YOLO 检测                       │
│  3. 灯语识别 -> Pattern '2200'     │
│  4. 翻译指令 -> Command 'APPROACH' │ ⚠️ 职责过多
│  5. 发送 {'command': 'APPROACH'}   │
└──────────────┬──────────────────────┘
               │
               ├─────────────────┬─────────────────┐
               ↓                 ↓                 ↓
    ┌──────────────────┐  ┌──────────────┐  ┌──────────────┐
    │ control_sub.py   │  │test_vision_  │  │ 其他订阅者   │
    │                  │  │pub.py        │  │              │
    │ 接收 'APPROACH'  │  │接收 'APPROACH'│  │接收 'APPROACH'│
    │ 执行控制         │  │显示日志      │  │处理数据      │
    └──────────────────┘  └──────────────┘  └──────────────┘
```

**问题**：
- ❌ Vision 端承担了"感知"和"决策"两个职责
- ❌ 修改映射表需要重启 Vision 端（影响所有摄像头）
- ❌ 无法独立测试感知和决策模块
- ❌ 耦合度高，扩展性差

---

### ✅ 解耦后（分层架构）

```
┌─────────────────────────────────────┐
│         vision_pub.py               │  ✅ 纯感知层
│                                     │
│  1. 摄像头采集                      │
│  2. YOLO 检测                       │
│  3. 灯语识别 -> Pattern '2200'     │
│  4. 发送 {'pattern': '2200'}       │  ✅ 只负责感知
└──────────────┬──────────────────────┘
               │ 原始 Pattern
               ├─────────────────┬─────────────────┐
               ↓                 ↓                 ↓
    ┌──────────────────┐  ┌──────────────┐  ┌──────────────┐
    │ control_sub.py   │  │test_vision_  │  │ 其他订阅者   │
    │                  │  │pub.py        │  │              │
    │ 接收 '2200'      │  │接收 '2200'   │  │接收 '2200'   │
    │ 翻译 'APPROACH'  │  │显示 Pattern  │  │自定义处理    │
    │ 执行控制         │  │              │  │              │
    └──────────────────┘  └──────────────┘  └──────────────┘
         ↑ 决策层
```

**优势**：
- ✅ Vision 端只负责感知，职责单一
- ✅ Control 端负责决策，可独立修改映射表
- ✅ 可独立测试各模块
- ✅ 解耦清晰，扩展性强

---

## 📝 代码对比

### vision_pub.py

#### ❌ 解耦前
```python
# 第 100-112 行：包含映射表
self.PATTERN_TO_COMMAND = {
    '220': 'FORWARD', '2200': 'APPROACH', ...
}
self.ACTION_DESCRIPTIONS = {
    'FORWARD': '前进', 'APPROACH': '靠近', ...
}

# 第 188 行：翻译指令
return self.PATTERN_TO_COMMAND.get(pattern, 'IDLE')

# 第 288-299 行：发送 Command
detection_data = {
    'command': command,           # 已翻译的指令
    'description': '靠近',        # 中文描述
    ...
}
```

#### ✅ 解耦后
```python
# 第 104-106 行：移除映射表
# 【架构解耦】移除指令映射表
# Vision 端只负责输出原始 Pattern

# 第 185-186 行：返回原始 Pattern
return pattern  # 直接返回 '2200'

# 第 282-297 行：发送 Pattern
detection_data = {
    'pattern': pattern,           # 原始 Pattern
    ...                           # 移除 command 和 description
}
```

---

### control_sub.py

#### ❌ 解耦前
```python
# 第 344 行：直接使用 Vision 端的翻译结果
command = data.get('command', 'IDLE')

# 第 353-367 行：执行控制
if command == 'APPROACH':
    self.execute_approach_step(target_info)
elif command == 'RETREAT':
    self.execute_retreat_step(target_info)
...
```

#### ✅ 解耦后
```python
# 第 52-80 行：添加映射表
self.PATTERN_TO_COMMAND = {
    '220': 'FORWARD', '2200': 'APPROACH', ...
}
self.ACTION_DESCRIPTIONS = {
    'FORWARD': '前进', 'APPROACH': '靠近', ...
}

# 第 348-363 行：翻译并执行
pattern = data.get('pattern', 'IDLE')  # 提取 Pattern
command = self.PATTERN_TO_COMMAND.get(pattern, 'IDLE')  # 翻译

# 决策日志
if command != 'IDLE':
    print(f"🧠 Decision: Pattern '{pattern}' -> Action '{command}'")

# 执行控制
if command == 'APPROACH':
    self.execute_approach_step(target_info)
...
```

---

### test_vision_pub.py

#### ❌ 解耦前
```python
# 第 48-56 行
cmd = data.get('command', 'IDLE')
if cmd != 'IDLE':
    print(f"📥 [Cam{cam_idx}] Received: {cmd} | ...")
```

#### ✅ 解耦后
```python
# 第 53-61 行
pattern = data.get('pattern', 'IDLE')
if pattern != 'IDLE':
    print(f"📥 [Cam{cam_idx}] Received Pattern: '{pattern}' | ...")
```

---

## 🎯 日志输出对比

### ❌ 解耦前

**Vision 端**：
```
🎥 [Cam4] Sent: APPROACH | Dist=1.50m | Bearing=45.0° | TrackID=3
```

**Control 端**：
```
➡️ APPROACH: 误差1.00m | 方向45.0° | vx=0.289, vy=0.289
```

**test_vision_pub.py**：
```
📥 [Cam4] Received: APPROACH | Dist=1.50m | Bearing=45.0° | TrackID=3
```

---

### ✅ 解耦后

**Vision 端**：
```
🎥 [Cam4] Sent Pattern: '2200' | Dist=1.50m | Bearing=45.0° | TrackID=3
```

**Control 端**：
```
🧠 Decision: Pattern '2200' -> Action 'APPROACH' (靠近)
➡️ APPROACH: 误差1.00m | 方向45.0° | vx=0.289, vy=0.289
```

**test_vision_pub.py**：
```
📥 [Cam4] Received Pattern: '2200' | Dist=1.50m | Bearing=45.0° | TrackID=3
```

---

## 📊 性能对比

| 指标 | 解耦前 | 解耦后 | 说明 |
|------|--------|--------|------|
| Vision 端 CPU | 高 | 低 | 移除了映射表查询 |
| Control 端 CPU | 低 | 略高 | 增加了映射表查询 |
| 总体性能 | 相同 | 相同 | 只是职责转移 |
| 可维护性 | 低 | 高 | 职责清晰 |
| 可扩展性 | 低 | 高 | 易于修改映射 |
| 可测试性 | 低 | 高 | 可独立测试 |

---

## ✅ 总结

| 方面 | 解耦前 | 解耦后 |
|------|--------|--------|
| **职责划分** | ❌ 混乱 | ✅ 清晰 |
| **数据格式** | `{'command': 'APPROACH'}` | `{'pattern': '2200'}` |
| **决策位置** | Vision 端 | Control 端 |
| **可维护性** | ❌ 低 | ✅ 高 |
| **可扩展性** | ❌ 低 | ✅ 高 |
| **可测试性** | ❌ 低 | ✅ 高 |

**结论**：架构解耦成功，系统更加清晰、灵活、易维护！

