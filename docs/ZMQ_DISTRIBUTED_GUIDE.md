# ZeroMQ 分布式架构使用指南

## 📋 概述

这是一个基于 ZeroMQ 的真正多进程分布式架构，彻底解耦了视觉感知和运动控制。

### 架构对比

| 特性 | 原架构 (threading) | 新架构 (ZeroMQ) |
|------|-------------------|----------------|
| 进程模型 | 单进程多线程 | 多进程分布式 |
| 通信方式 | 共享内存 (RobotState) | ZeroMQ PUB-SUB |
| 终端输出 | 混在一起 | 完全独立 |
| 可扩展性 | 受限于单机 | 可跨网络部署 |
| 调试难度 | 较高（日志混杂） | 低（独立日志） |

---

## 🚀 快速开始

### 1. 安装依赖

```bash
pip install pyzmq
```

### 2. 启动视觉发布者（终端1）

```bash
cd /home/nvidia/Downloads/Ros/pseudo_ros_architecture
python3 vision_pub.py
```

**预期输出：**
```
============================================================
🎥 视觉发布者 (ZeroMQ PUB)
============================================================
📡 发布地址: tcp://*:5555
📹 摄像头: [4]
🤖 模型路径: /home/nvidia/Downloads/Ros/real_ws/vision/best.pt
============================================================
按 Ctrl+C 停止

✅ 视觉发布者初始化完成 - ZMQ 绑定到 tcp://*:5555
✅ 摄像头 4 初始化成功
✅ 摄像头 4 模型加载成功
🚀 启动视觉发布者 - 摄像头 4
🎥 [Cam4] Sent: APPROACH | Dist=1.23m | Bearing=5.2° | TrackID=1
```

### 3. 启动控制订阅者（终端2）

```bash
cd /home/nvidia/Downloads/Ros/pseudo_ros_architecture
python3 control_sub.py
```

**预期输出：**
```
============================================================
🤖 控制订阅者 (ZeroMQ SUB)
============================================================
📡 订阅地址: tcp://localhost:5555
🆔 机器人ID: 8
🎛️ 控制模式: 实际控制
⏱️ 超时保护: 1000ms
============================================================
按 Ctrl+C 停止

✅ 机器人控制模块加载成功
🤖 机器人控制器初始化成功 - ID: 8
✅ 控制订阅者初始化完成 - 订阅 tcp://localhost:5555
🚀 启动控制订阅者
➡️ APPROACH: 误差0.73m | 方向5.2° | vx=0.242, vy=0.022
```

---

## ⚙️ 配置参数

### vision_pub.py

在文件末尾的 `if __name__ == "__main__":` 部分修改：

```python
MODEL_PATH = "/path/to/your/best.pt"  # YOLO 模型路径
CAMERA_INDICES = [4]  # 摄像头索引：[0]=后, [2]=右, [4]=前, [6]=左
ZMQ_PORT = 5555  # ZeroMQ 端口
```

### control_sub.py

在文件末尾的 `if __name__ == "__main__":` 部分修改：

```python
ROBOT_ID = 8  # 机器人ID
ENABLE_CONTROL = True  # False = Mock模式（仅打印日志）
ZMQ_ADDRESS = "tcp://localhost:5555"  # 订阅地址
TIMEOUT_MS = 1000  # 超时保护（毫秒）
```

---

## 🔧 高级用法

### 跨网络部署

如果视觉和控制在不同机器上：

**机器A（视觉）：**
```python
# vision_pub.py
ZMQ_PORT = 5555  # 绑定到所有网卡
```

**机器B（控制）：**
```python
# control_sub.py
ZMQ_ADDRESS = "tcp://192.168.1.100:5555"  # 机器A的IP
```

### Mock 模式测试

如果没有机器人硬件，可以启用 Mock 模式：

```python
# control_sub.py
ENABLE_CONTROL = False
```

此时控制节点只会打印日志，不会发送实际控制指令。

### 多摄像头支持

当前版本为简化实现，仅支持单摄像头。如需多摄像头，可以：

1. **方案1**：启动多个 vision_pub.py 实例，使用不同端口
2. **方案2**：修改代码使用 multiprocessing 模块

---

## 📊 消息格式

### ZeroMQ 消息结构

**Topic:** `perception`

**Payload (JSON):**
```json
{
  "distance": 1.23,
  "azimuth": 5.2,
  "bearing_body": 5.2,
  "track_id": 1,
  "cam_idx": 4,
  "command": "APPROACH",
  "description": "靠近",
  "class_id": 2,
  "timestamp": 1234567890.123
}
```

### 支持的指令

| 指令 | 描述 | 控制逻辑 |
|------|------|---------|
| `APPROACH` | 靠近 | 全向运动，目标距离0.5m |
| `RETREAT` | 远离 | 反向运动，目标距离2.0m |
| `S_SHAPE` | S形运动 | 正弦波轨迹，持续20s |
| `CIRCLE` | 圆形运动 | 圆周轨迹，持续8s |
| `FORWARD` | 前进 | 基本运动，持续4s |
| `LEFT` | 左移 | 基本运动，持续4s |
| `RIGHT` | 右移 | 基本运动，持续4s |
| `REVERSE` | 后退 | 基本运动，持续4s |
| `STOP` | 停止 | 立即停止 |
| `IDLE` | 待机 | 无动作 |

---

## 🛡️ 安全特性

### 超时保护

控制订阅者内置超时保护机制：

- 如果超过 **1.0 秒** 没有收到 ZeroMQ 消息
- 自动执行 `safety_stop()`
- 防止失控

### 非阻塞控制

所有运动控制函数都是 **非阻塞** 的：

- `execute_approach_step()`: 每帧计算一次速度
- `execute_s_shape_step()`: 基于时间生成轨迹
- 不会卡住主循环

---

## 🐛 故障排查

### 问题1：控制节点收不到消息

**症状：**
```
⚠️ 超时 1.02s，执行安全停止
```

**解决方案：**
1. 检查视觉节点是否正常运行
2. 检查 ZeroMQ 地址是否匹配
3. 检查防火墙设置

### 问题2：YOLO 模型加载失败

**症状：**
```
❌ 摄像头 4 模型加载失败: ...
```

**解决方案：**
1. 检查 `MODEL_PATH` 是否正确
2. 确保模型文件存在
3. 检查 ultralytics 版本

### 问题3：摄像头初始化失败

**症状：**
```
❌ 摄像头 4 初始化失败，退出
```

**解决方案：**
1. 检查摄像头是否被占用
2. 尝试其他索引：`[0]`, `[2]`, `[6]`
3. 运行 `ls /dev/video*` 查看可用设备

---

## 📝 与原架构的差异

### 已移除的文件

- ❌ `main_system.py` - 不再需要主入口
- ❌ `robot_shared_state.py` - 不再需要共享内存

### 核心改动

1. **通信方式**：
   - 原：`robot_state.update_perception()` → `robot_state.get_latest_state()`
   - 新：`socket.send_string()` → `socket.recv_string()`

2. **控制逻辑**：
   - 原：阻塞式 `while` 循环（如 `execute_approach` 内部循环）
   - 新：非阻塞式单步执行（每次收到消息计算一次）

3. **进程模型**：
   - 原：`threading.Thread`
   - 新：独立 Python 进程

---

## 🎯 下一步优化建议

1. **添加心跳机制**：定期发送心跳包，检测连接状态
2. **支持多摄像头**：使用 multiprocessing 并行处理
3. **添加日志系统**：使用 logging 模块替代 print
4. **性能监控**：添加 FPS、延迟统计
5. **配置文件**：使用 YAML/JSON 管理参数

---

## 📞 联系方式

如有问题，请检查：
- ZeroMQ 文档：https://zeromq.org/
- PyZMQ 文档：https://pyzmq.readthedocs.io/

---

**祝你使用愉快！🎉**

