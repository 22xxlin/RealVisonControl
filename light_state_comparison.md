# Follower 灯光状态转换逻辑对比

## 📊 两个文件的灯光状态流程对比

### transport_node.py (参考实现) ✅

```
初始化 (第 208 行)
  ↓
light.set_cmd("LOCK_LEFT" / "LOCK_RIGHT")  # 绿/蓝常亮
  ↓
WAIT_FORM 状态 (第 256-260 行)
  ↓ (检测到 PURPLE SOLID)
light.set_cmd("FOLLOWER_PUSH")  # 变灰常亮
  ↓
ARMED → RUN → DONE
```

**特点**:
- ✅ Follower 启动时就是常亮（假设已手动就位）
- ✅ 看到 GO 信号立即变灰
- ✅ 每个状态只设置一次灯光

---

### combined_transport_node.py (修复前) ⚠️

```
初始化 (第 341 行)
  ↓
light.set_cmd("OFF")  # 关灯
  ↓
DOCKING 状态 (第 387-394 行)
  ├─ ADJUSTING → BID_LEFT/BID_RIGHT (绿闪/蓝闪)
  └─ LOCKED → LOCK_LEFT/LOCK_RIGHT (绿常亮/蓝常亮) ✅
  ↓
WAIT_GO_SIGNAL 状态 (第 411-414 行 - 修复前)
  ↓ (每次循环都重复设置)
light.set_cmd("LOCK_LEFT" / "LOCK_RIGHT")  # ❌ 浪费资源
  ↓ (检测到 PURPLE SOLID)
light.set_cmd("FOLLOWER_PUSH")  # 变灰常亮 ✅
  ↓
ARMED → RUN → DONE
```

**问题**:
- ❌ WAIT_GO_SIGNAL 状态每次循环都重复设置灯光（30Hz）
- ❌ 浪费 MQTT 带宽

---

### combined_transport_node.py (修复后) ✅

```
初始化 (第 341 行)
  ↓
light.set_cmd("OFF")  # 关灯
  ↓
DOCKING 状态 (第 387-394 行)
  ├─ ADJUSTING → BID_LEFT/BID_RIGHT (绿闪/蓝闪)
  └─ LOCKED → LOCK_LEFT/LOCK_RIGHT (绿常亮/蓝常亮) ✅
  ↓
WAIT_GO_SIGNAL 状态 (第 411-419 行 - 修复后)
  ↓ (不再重复设置，只等待信号)
  ↓ (检测到 PURPLE SOLID)
light.set_cmd("FOLLOWER_PUSH")  # 变灰常亮 ✅
  ↓
ARMED → RUN → DONE
```

**改进**:
- ✅ 灯光只在状态转换时设置一次
- ✅ 节省 MQTT 带宽
- ✅ 逻辑更清晰

---

## 🎯 完整灯光协议流程

### Follower 视角

| 阶段 | 状态 | 灯光命令 | 颜色+模式 | 含义 |
|------|------|----------|-----------|------|
| 1. 搜索 | DOCKING (ADJUSTING) | SEARCH | 紫闪 | 正在寻找球和Leader |
| 2. 竞标 | DOCKING (ADJUSTING) | BID_LEFT/BID_RIGHT | 绿闪/蓝闪 | 正在调整到目标位置 |
| 3. 锁定 | DOCKING (LOCKED) | LOCK_LEFT/LOCK_RIGHT | 绿常亮/蓝常亮 | 已就位，等待GO信号 |
| 4. 确认 | WAIT_GO_SIGNAL → ARMED | FOLLOWER_PUSH | 灰常亮 | 收到GO，准备搬运 |
| 5. 搬运 | RUN | FOLLOWER_PUSH | 灰常亮 | 正在搬运 |
| 6. 完成 | DONE | FOLLOWER_PUSH | 灰常亮 | 搬运完成 |

### Leader 视角

| 阶段 | 状态 | 灯光命令 | 颜色+模式 | 含义 |
|------|------|----------|-----------|------|
| 1. 等待 | WAIT_FORM | LEADER_WAIT | 红常亮 | 等待队友就位 |
| 2. 预热 | PREWARM | SEARCH | 紫闪 | 队友已就位，预热中 |
| 3. 发令 | WAIT_GO_LOCAL | LEADER_GO | 紫常亮 | 发出GO信号 |
| 4. 搬运 | RUN | LEADER_GO | 紫常亮 | 正在搬运 |
| 5. 完成 | DONE | LEADER_GO | 紫常亮 | 搬运完成 |

---

## 🔧 修复内容

### 修改位置：combined_transport_node.py 第 411-421 行

**修复前**:
```python
elif state == "WAIT_GO_SIGNAL":
    # 维持灯光
    if my_side == "left": light.set_cmd("LOCK_LEFT")   # ❌ 每次循环都调用
    else: light.set_cmd("LOCK_RIGHT")
    
    if watcher.stable_pattern(PURPLE, "SOLID", 3, 0.4):
        state = "ARMED"
        light.set_cmd("FOLLOWER_PUSH")
```

**修复后**:
```python
elif state == "WAIT_GO_SIGNAL":
    # 灯光已在 LOCKED 时设置，这里不需要重复设置
    # 只检测 GO 信号
    if watcher.stable_pattern(PURPLE, "SOLID", 3, 0.4):
        state = "ARMED"
        light.set_cmd("FOLLOWER_PUSH")  # 变灰常亮
```

---

## ✅ 验证清单

- [x] Follower 锁定后显示绿/蓝常亮
- [x] Leader 能正确识别 Follower 就位（检查 SOLID 模式）
- [x] Follower 收到 GO 信号后变灰常亮
- [x] 灯光只在状态转换时设置，不重复调用
- [x] 逻辑与 transport_node.py 一致

---

## 📝 总结

**问题**: combined_transport_node.py 在 WAIT_GO_SIGNAL 状态每次循环都重复设置灯光。

**影响**: 浪费 MQTT 带宽，但不影响功能正确性。

**修复**: 移除重复的 `light.set_cmd()` 调用，灯光已在 LOCKED 状态设置。

**结果**: 逻辑更清晰，性能更优，与 transport_node.py 保持一致。

