# vision_pub.py 重构总结

## 🎯 重构目标
将 `vision_pub.py` 从单摄像头模式重构为支持 **4个摄像头并行工作** 的多线程架构。

## 🏗️ 架构设计：生产者-消费者模型

### 原有问题
- `run()` 方法直接调用 `camera_worker()`，导致主线程被占满
- 只能运行单个摄像头，无法并行处理多个摄像头
- ZMQ Socket 在子线程中操作（不安全）

### 新架构
```
┌─────────────────────────────────────────────────────────┐
│                    主线程 (消费者)                        │
│  - 启动 4 个生产者线程                                    │
│  - while True: data = queue.get()                       │
│  - 统一负责 ZMQ 发送 (socket.send_string)                │
│  - 统一负责日志打印 (print)                               │
└─────────────────────────────────────────────────────────┘
                           ↑
                           │ queue.get()
                           │
              ┌────────────┴────────────┐
              │  queue.Queue(maxsize=100) │
              │    (线程安全队列)          │
              └────────────┬────────────┘
                           │ queue.put_nowait()
                           ↓
┌──────────────┬──────────────┬──────────────┬──────────────┐
│  生产者线程1  │  生产者线程2  │  生产者线程3  │  生产者线程4  │
│  Camera 0    │  Camera 2    │  Camera 4    │  Camera 6    │
│  (后方 180°) │  (右侧 -90°) │  (前方 0°)   │  (左侧 90°)  │
│              │              │              │              │
│  - 读图      │  - 读图      │  - 读图      │  - 读图      │
│  - YOLO检测  │  - YOLO检测  │  - YOLO检测  │  - YOLO检测  │
│  - 灯语识别  │  - 灯语识别  │  - 灯语识别  │  - 灯语识别  │
│  - 放入队列  │  - 放入队列  │  - 放入队列  │  - 放入队列  │
└──────────────┴──────────────┴──────────────┴──────────────┘
```

## 📝 主要修改

### 1. 新增导入
```python
import queue
import threading
```

### 2. `__init__` 方法
```python
# 线程安全队列（生产者-消费者模型）
self.queue = queue.Queue(maxsize=100)
```

### 3. `camera_worker` 方法（生产者）
**修改前：**
- 直接调用 `self.socket.send_string()` 发送 ZMQ 消息
- 直接使用 `print()` 打印日志

**修改后：**
- 不再直接操作 `self.socket`（ZMQ Socket 不是线程安全的）
- 不再直接 `print()`
- 所有数据通过 `self.queue.put_nowait(data)` 放入队列
- 如果队列满了（`queue.Full`），直接丢弃该帧数据
- 数据格式：
  ```python
  {
      'type': 'detection',  # 或 'log'
      'distance': float,
      'azimuth': float,
      'bearing_body': float,
      'track_id': int,
      'cam_idx': int,
      'command': str,
      'description': str,
      'class_id': int,
      'timestamp': float
  }
  ```

### 4. `run` 方法（消费者）
**修改前：**
- 只支持单摄像头：`self.camera_worker(self.camera_indices[0])`

**修改后：**
- 使用 `threading.Thread` 启动 4 个生产者线程
- 主线程进入 `while True` 循环
- 不断执行 `data = self.queue.get()` 从队列取数据
- 根据 `data['type']` 处理不同类型的消息：
  - `'log'`：直接打印日志
  - `'detection'`：通过 ZMQ 发送检测数据
- 主线程统一负责 ZMQ 发送和日志打印

### 5. 删除的方法
- `publish_detection()` 方法已删除（功能整合到 `run()` 方法中）

## ✅ 优势

1. **线程安全**：ZMQ Socket 只在主线程操作，避免多线程竞争
2. **并行处理**：4个摄像头同时工作，充分利用多核 CPU
3. **解耦设计**：生产者和消费者分离，职责清晰
4. **队列缓冲**：maxsize=100 的队列可以缓冲突发数据
5. **丢帧策略**：队列满时自动丢弃，避免阻塞生产者线程
6. **易于扩展**：可以轻松增加或减少摄像头数量

## 🚀 使用方式

```python
# 默认配置：4个摄像头并行
CAMERA_INDICES = [0, 2, 4, 6]

publisher = VisionPublisher(
    model_path=MODEL_PATH,
    camera_indices=CAMERA_INDICES,
    zmq_port=5555
)

publisher.run()
```

## 🔧 技术细节

- **线程模型**：使用 `threading.Thread`，设置 `daemon=True`
- **队列大小**：`maxsize=100`（可根据实际情况调整）
- **丢帧策略**：`queue.put_nowait()` + `try-except queue.Full`
- **线程命名**：`name=f"Camera-{cam_idx}"` 便于调试
- **优雅退出**：`KeyboardInterrupt` 捕获，`thread.join(timeout=5.0)` 等待子线程

## 📊 性能考虑

- **为什么用 threading 而不是 multiprocessing？**
  - ZMQ + CUDA 在多进程 fork 下会有问题
  - 多线程配合 ZMQ 在主线程发送是最佳实践
  - Python GIL 对 I/O 密集型任务（摄像头读取）影响较小
  - YOLO 推理主要在 GPU 上，不受 GIL 限制

## 🎉 完成状态

✅ 支持 4 个摄像头并行工作  
✅ 生产者-消费者模型  
✅ 线程安全的 ZMQ 发送  
✅ 队列缓冲和丢帧策略  
✅ 保留原有的 YOLO 检测和灯语识别逻辑  
✅ 代码结构清晰，易于维护

