# 批量发送重构说明 (Batch Sending Refactor)

## 修改概述

将视觉发布节点从 **"逐个发送"** 模式重构为 **"批量打包发送"** 模式，减少 ZMQ 通信开销，避免缓冲区溢出。

---

## 1. vision_pub.py 修改

### 修改位置
- **文件**: `vision_pub.py`
- **行数**: 319-352 (原 319-335)
- **函数**: `VisionPublisher.run()` 中的发送阶段

### 修改内容

#### 原逻辑 (逐个发送)
```python
for obj in final_objects:
    topic = "perception"
    pub_data = {
        'cam_idx': obj['cam_idx'],
        'class_id': obj['class_id'],
        'pattern': obj['pattern'],
        'distance': obj['distance'],
        'bearing_body': obj['bearing_body']
    }
    self.socket.send_string(f"{topic} {json.dumps(pub_data)}")
```

#### 新逻辑 (批量发送)
```python
# 构造包含所有物体的大字典 (Payload)
topic = "perception"
timestamp = time.time()

# 构建物体列表
objects_list = []
for obj in final_objects:
    obj_data = {
        'cam_idx': obj['cam_idx'],
        'class_id': obj['class_id'],
        'pattern': obj['pattern'],
        'distance': obj['distance'],
        'bearing_body': obj['bearing_body']
    }
    objects_list.append(obj_data)

# 构造完整的 Payload
payload = {
    'timestamp': timestamp,
    'count': len(final_objects),
    'objects': objects_list
}

# 只发送一次
self.socket.send_string(f"{topic} {json.dumps(payload)}")
```

### Payload 数据结构
```json
{
    "timestamp": 1234567890.123,
    "count": 2,
    "objects": [
        {
            "cam_idx": 0,
            "class_id": 6,
            "pattern": "SOLID",
            "distance": 1.25,
            "bearing_body": 180.5
        },
        {
            "cam_idx": 2,
            "class_id": 2,
            "pattern": "FLASH",
            "distance": 2.30,
            "bearing_body": 270.0
        }
    ]
}
```

---

## 2. combined_transport_node.py 修改

### 修改位置
- **文件**: `combined_transport_node.py`
- **行数**: 396-422 (原 396-407)
- **函数**: `main()` 中的数据接收循环

### 修改内容

#### 原逻辑 (接收单个物体)
```python
batch = []
while True:
    try:
        s = sock.recv_string(flags=zmq.NOBLOCK)
        _, payload = s.split(" ", 1)
        batch.append(json.loads(payload))
    except zmq.Again:
        break
```

#### 新逻辑 (Drop to Newest 策略 - 关键修复)
```python
batch = []
latest_payload = None

# 循环读空 ZMQ 缓冲区，但只保留最后一次收到的包
while True:
    try:
        s = sock.recv_string(flags=zmq.NOBLOCK)
        _, payload_str = s.split(" ", 1)
        parsed_json = json.loads(payload_str)

        # 兼容性处理
        if 'objects' in parsed_json:
            latest_payload = parsed_json['objects']  # 新版打包格式
        else:
            latest_payload = [parsed_json]  # 旧版格式包装成 list

    except zmq.Again:
        break
    except Exception as e:
        pass

# 如果这一轮循环收到了数据，使用最新的一帧
if latest_payload is not None:
    batch = latest_payload
```

### ⚠️ 关键修复：防止数据积压导致状态误判

#### 问题描述
原先的 `batch.extend()` 逻辑存在严重隐患：
- 如果 ZMQ 缓冲区积压了 3 帧数据 (T1, T2, T3)
- 代码会一次性把 3 帧的所有物体塞进同一个 `batch`
- `EventWatcher` 会在同一微秒内记录这 3 次检测
- **后果**: 欺骗 `stable_pattern` 逻辑，导致机器人在不该启动时启动

#### 解决方案：Drop to Newest
- **策略**: 只保留最新一帧，丢弃过期数据
- **实现**: 使用 `latest_payload` 变量，循环结束时只保留最后一次解析的数据
- **优势**:
  - 避免积压数据污染状态机
  - 保证实时性 (旧帧已过期，无需处理)
  - 防止 `stable_pattern` 被瞬间满足

### 兼容性说明
- **新格式**: 包含 `objects` 字段，解包为物体列表
- **旧格式**: 单个物体字典，包装成 `[parsed_json]`
- **无缝切换**: 接收端可同时处理新旧两种格式

---

## 3. 优势分析

### 性能提升
1. **减少系统调用**: 从 N 次 `send_string` 减少到 1 次
2. **降低网络开销**: 减少 ZMQ 消息头开销
3. **避免缓冲区溢出**: 单次发送大数据包，而非多次小包

### 数据完整性
1. **原子性**: 同一帧的所有物体在同一个消息中
2. **时间戳**: 添加统一时间戳，便于同步
3. **计数器**: `count` 字段可用于校验数据完整性

### 实时性保证 (关键)
1. **Drop to Newest**: 只处理最新帧，丢弃过期数据
2. **防止状态污染**: 避免积压数据欺骗 `stable_pattern` 逻辑
3. **时序正确性**: 保证 `EventWatcher` 记录的时间戳是真实的接收时刻

### 可维护性
1. **清晰的数据结构**: Payload 结构一目了然
2. **向后兼容**: 接收端支持新旧格式
3. **易于扩展**: 可在 Payload 中添加更多元数据

---

## 4. 实时系统设计原则

### Drop to Newest 策略详解

#### 为什么需要？
在实时控制系统中，**旧数据比错误数据更危险**：
- 机器人的决策基于"当前状态"
- 如果处理 100ms 前的数据，机器人可能已经移动到新位置
- 积压的多帧数据会导致状态机误判（如 `stable_pattern` 瞬间满足条件）

#### 实现细节
```python
latest_payload = None
while True:
    try:
        data = sock.recv_string(flags=zmq.NOBLOCK)
        latest_payload = parse(data)  # 不断覆盖
    except zmq.Again:
        break

# 循环结束后，latest_payload 就是最新的一帧
if latest_payload is not None:
    batch = latest_payload
```

#### 对比其他策略
| 策略 | 优点 | 缺点 | 适用场景 |
|------|------|------|----------|
| **Drop to Newest** | 保证实时性 | 丢失历史数据 | 实时控制 (本项目) |
| Accumulate All | 不丢数据 | 可能处理过期数据 | 离线分析 |
| FIFO Queue | 顺序处理 | 延迟累积 | 非实时任务 |

#### 实际案例
假设 ZMQ 缓冲区积压了 3 帧：
```
T1: [球@180°, 车@90°]
T2: [球@175°, 车@85°]
T3: [球@170°, 车@80°]  ← 最新
```

- **旧逻辑 (extend)**: `batch = [球@180°, 车@90°, 球@175°, 车@85°, 球@170°, 车@80°]` (6个物体)
  - `EventWatcher` 会在同一微秒记录 3 次"球"检测
  - `stable_pattern(BALL, "SOLID", 3, 0.4)` 瞬间满足 ❌

- **新逻辑 (latest)**: `batch = [球@170°, 车@80°]` (2个物体)
  - 只处理最新帧，T1/T2 被丢弃
  - `stable_pattern` 需要真实的 3 次检测才满足 ✅

---

## 5. 测试建议

### 功能测试
- [ ] 单物体场景: 验证 count=1 的情况
- [ ] 多物体场景: 验证 count>1 的批量发送
- [ ] 空帧场景: 验证 count=0 时的行为

### 性能测试
- [ ] 对比发送延迟: 旧版 vs 新版
- [ ] 监控 ZMQ 缓冲区使用率
- [ ] 高负载测试 (多相机 + 多物体)

### 实时性测试 (关键)
- [ ] **积压测试**: 人为制造延迟，验证 Drop to Newest 是否生效
  ```python
  # 在接收端添加临时延迟
  time.sleep(0.2)  # 模拟卡顿
  # 观察是否只处理最新帧
  ```
- [ ] **状态机测试**: 验证 `stable_pattern` 不会被积压数据欺骗
  - 快速移动物体，观察是否需要真实的 3 次检测
  - 不应该因为积压而瞬间满足条件
- [ ] **时间戳验证**: 打印 `EventWatcher` 记录的时间戳，确认间隔合理

### 兼容性测试
- [ ] 新发送端 + 新接收端
- [ ] 旧发送端 + 新接收端 (向后兼容)

---

## 6. 注意事项

1. **JSON 序列化开销**: 批量发送会增加单次 JSON 序列化的大小，但总体开销仍低于多次发送
2. **消息大小限制**: 确保 ZMQ 配置支持较大的消息 (默认通常足够)
3. **时间戳精度**: 使用 `time.time()` 提供秒级浮点时间戳
4. **Drop to Newest 的代价**: 会丢失中间帧，但对实时控制系统这是正确的选择
5. **EventWatcher 时间戳**: 使用接收端的 `mono()` 而非发送端的 `timestamp`，避免多机时间不同步

---

## 7. 回滚方案

如需回滚到旧版本，只需恢复以下代码段：

### vision_pub.py (319-335 行)
```python
for obj in final_objects:
    topic = "perception"
    pub_data = {
        'cam_idx': obj['cam_idx'],
        'class_id': obj['class_id'],
        'pattern': obj['pattern'],
        'distance': obj['distance'],
        'bearing_body': obj['bearing_body']
    }
    self.socket.send_string(f"{topic} {json.dumps(pub_data)}")
```

### combined_transport_node.py (396-407 行)
```python
batch = []
while True:
    try:
        s = sock.recv_string(flags=zmq.NOBLOCK)
        _, payload = s.split(" ", 1)
        batch.append(json.loads(payload))
    except zmq.Again:
        break
    except Exception:
        pass
watcher.ingest(batch)
```

---

**重构完成时间**: 2025-01-XX
**版本**: V3.3 (Batch Sending + Drop to Newest)
**关键修复**: 防止数据积压导致状态机误判

