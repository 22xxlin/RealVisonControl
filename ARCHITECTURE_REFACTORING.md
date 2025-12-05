# 架构解耦文档 - Vision & Control 分离

## 📋 背景 (Context)

原架构中，`vision_pub.py` 承担了过多职责：
- ✅ 视觉感知（YOLO 检测、灯语识别）
- ❌ 指令翻译（Pattern -> Command 映射）

这违反了单一职责原则，导致耦合度过高。

---

## 🎯 目标 (Objective)

实现架构解耦，明确职责边界：

| 模块 | 职责 | 输入 | 输出 |
|------|------|------|------|
| **Vision 端** | 纯感知层 | 摄像头图像 | 原始 Pattern（如 `'2200'`, `'110'`, `'IDLE'`） |
| **Control 端** | 决策层 | 原始 Pattern | 控制指令（如 `'APPROACH'`, `'FORWARD'`） |

---

## 🔧 实施方案 (Implementation)

### Step 1: 修改 `vision_pub.py`（纯感知层）

#### 变更 1.1：移除指令映射表
```python
# ❌ 删除（第 100-112 行）
self.PATTERN_TO_COMMAND = {...}
self.ACTION_DESCRIPTIONS = {...}

# ✅ 替换为
# 【架构解耦】移除指令映射表
# 原本的 PATTERN_TO_COMMAND 和 ACTION_DESCRIPTIONS 已迁移到 control_sub.py
# Vision 端只负责输出原始 Pattern，不进行指令翻译
```

#### 变更 1.2：修改 `recognize_pattern` 返回值
```python
# ❌ 原逻辑（第 188 行）
return self.PATTERN_TO_COMMAND.get(pattern, 'IDLE')

# ✅ 新逻辑
return pattern  # 直接返回原始 Pattern（如 '2200', '110'）
```

#### 变更 1.3：修改数据包格式
```python
# ❌ 原数据包（第 288-299 行）
detection_data = {
    'command': command,           # 已翻译的指令
    'description': '靠近',        # 中文描述
    ...
}

# ✅ 新数据包
detection_data = {
    'pattern': pattern,           # 原始 Pattern（如 '2200'）
    ...                           # 移除 command 和 description
}
```

#### 变更 1.4：更新日志输出
```python
# ❌ 原日志
print(f"🎥 [Cam{cam_idx}] Sent: {cmd} | ...")

# ✅ 新日志
print(f"🎥 [Cam{cam_idx}] Sent Pattern: '{pattern}' | ...")
```

---

### Step 2: 更新 `control_sub.py`（决策层）

#### 变更 2.1：添加映射表到 `__init__`
```python
# ✅ 新增（第 52-80 行）
self.PATTERN_TO_COMMAND = {
    # 基本运动模式（3位）
    '220': 'FORWARD',   # 红红黑 -> 前进
    '330': 'LEFT',      # 绿绿黑 -> 左移
    '110': 'RIGHT',     # 蓝蓝黑 -> 右移
    '550': 'REVERSE',   # 黄黄黑 -> 后退
    '440': 'STOP',      # 紫紫黑 -> 停止
    
    # 高级运动模式（4位）
    '2200': 'APPROACH', # 红红黑黑 -> 靠近
    '1100': 'RETREAT',  # 蓝蓝黑黑 -> 远离
    '4400': 'S_SHAPE',  # 紫紫黑黑 -> S形轨迹
    '5500': 'CIRCLE',   # 黄黄黑黑 -> 圆形轨迹
    
    # 连续模式（4位）
    '1111': 'FORWARD',  # 蓝蓝蓝蓝 -> 前进
    '2222': 'LEFT',     # 红红红红 -> 左移
    '3333': 'RIGHT',    # 绿绿绿绿 -> 右移
    '4444': 'STOP',     # 紫紫紫紫 -> 停止
    '5555': 'REVERSE',  # 黄黄黄黄 -> 后退
}

self.ACTION_DESCRIPTIONS = {
    'FORWARD': '前进', 'LEFT': '左移', 'RIGHT': '右移', 'STOP': '停止',
    'REVERSE': '后退', 'APPROACH': '靠近', 'RETREAT': '远离', 
    'S_SHAPE': 'S形', 'CIRCLE': '圆形', 'IDLE': '待机'
}
```

#### 变更 2.2：修改主循环逻辑
```python
# ❌ 原逻辑（第 344 行）
command = data.get('command', 'IDLE')

# ✅ 新逻辑（第 348-363 行）
# 1. 提取原始 Pattern
pattern = data.get('pattern', 'IDLE')

# 2. 决策：Pattern -> Command 翻译
command = self.PATTERN_TO_COMMAND.get(pattern, 'IDLE')

# 3. 决策日志
if command != 'IDLE':
    description = self.ACTION_DESCRIPTIONS.get(command, '未知')
    print(f"🧠 Decision: Pattern '{pattern}' -> Action '{command}' ({description})")

# 4. 执行控制
if command == 'APPROACH':
    self.execute_approach_step(target_info)
...
```

---

## ✅ 验证清单 (Checklist)

- [x] `vision_pub.py` 不再包含 `PATTERN_TO_COMMAND` 和 `ACTION_DESCRIPTIONS`
- [x] `vision_pub.py` 的 `recognize_pattern` 返回原始 Pattern
- [x] `vision_pub.py` 发送的 JSON 包含 `'pattern'` 字段（不含 `'command'`）
- [x] `control_sub.py` 包含完整的 `PATTERN_TO_COMMAND` 映射表
- [x] `control_sub.py` 从 `data['pattern']` 提取数据并翻译
- [x] `control_sub.py` 打印决策日志：`Pattern 'xxx' -> Action 'yyy'`

---

## 🚀 运行测试

### 启动 Vision 端
```bash
python3 vision_pub.py
```
**预期输出**：
```
🎥 [Cam4] Sent Pattern: '2200' | Dist=1.50m | Bearing=45.0° | TrackID=3
```

### 启动 Control 端
```bash
python3 control_sub.py
```
**预期输出**：
```
🧠 Decision: Pattern '2200' -> Action 'APPROACH' (靠近)
➡️ APPROACH: 误差1.00m | 方向45.0° | vx=0.289, vy=0.289
```

---

## 📊 架构对比

### 原架构（耦合）
```
┌─────────────────┐
│  vision_pub.py  │
│                 │
│  1. 检测        │
│  2. 识别 Pattern│
│  3. 翻译 Command│ ❌ 职责过多
│  4. 发送 Command│
└─────────────────┘
         ↓
┌─────────────────┐
│ control_sub.py  │
│                 │
│  接收 Command   │
│  执行控制       │
└─────────────────┘
```

### 新架构（解耦）
```
┌─────────────────┐
│  vision_pub.py  │  ✅ 纯感知层
│                 │
│  1. 检测        │
│  2. 识别 Pattern│
│  3. 发送 Pattern│
└─────────────────┘
         ↓ pattern='2200'
┌─────────────────┐
│ control_sub.py  │  ✅ 决策层
│                 │
│  1. 接收 Pattern│
│  2. 翻译 Command│
│  3. 执行控制    │
└─────────────────┘
```

---

## 🎓 设计原则

1. **单一职责原则 (SRP)**：每个模块只负责一件事
2. **关注点分离 (SoC)**：感知与决策分离
3. **可扩展性**：修改映射表只需改 `control_sub.py`
4. **可测试性**：可独立测试 Vision 和 Control 模块

