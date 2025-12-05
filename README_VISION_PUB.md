# vision_pub.py - 多摄像头并行视觉发布者

## 📖 概述

`vision_pub.py` 是一个基于 ZeroMQ 的视觉检测发布者，支持 **4个摄像头同时并行工作**，采用 **生产者-消费者模型** 实现高效的多线程架构。

## 🏗️ 架构特点

### 生产者-消费者模型

- **4个生产者线程**：每个摄像头独立运行，负责图像采集、YOLO检测、灯语识别
- **1个消费者主线程**：统一负责 ZMQ 消息发送和日志打印
- **线程安全队列**：`queue.Queue(maxsize=100)` 作为生产者和消费者之间的缓冲

### 为什么使用 threading 而不是 multiprocessing？

1. **ZMQ Socket 线程安全**：ZMQ Socket 不是线程安全的，必须在单一线程中操作
2. **CUDA 兼容性**：多进程 fork 模式下，CUDA 和 ZMQ 可能出现问题
3. **GIL 影响小**：摄像头读取是 I/O 密集型，YOLO 推理在 GPU 上，都不受 Python GIL 限制
4. **资源共享**：线程间共享内存（如 detection_buffer），无需进程间通信

## 🚀 快速开始

### 1. 安装依赖

```bash
pip install opencv-python numpy pyzmq ultralytics
```

### 2. 运行发布者

```bash
python3 vision_pub.py
```

### 3. 测试订阅者

```bash
python3 test_vision_pub.py
```

## 📋 配置说明

### 摄像头配置

```python
CAMERA_INDICES = [0, 2, 4, 6]  # 4个摄像头
# Cam0: 后方 (180°)
# Cam2: 右侧 (-90°)
# Cam4: 前方 (0°)
# Cam6: 左侧 (90°)
```

### 模型路径

```python
MODEL_PATH = "/path/to/your/best.engine"  # YOLO 模型路径
```

### ZMQ 端口

```python
ZMQ_PORT = 5555  # ZeroMQ 发布端口
```

## 📊 数据格式

### 发布的消息格式

```json
{
    "distance": 2.5,
    "azimuth": 15.3,
    "bearing_body": 195.3,
    "track_id": 42,
    "cam_idx": 4,
    "command": "FORWARD",
    "description": "前进",
    "class_id": 2,
    "timestamp": 1234567890.123
}
```

### 字段说明

- `distance`: 目标距离（米）
- `azimuth`: 相机坐标系方位角（度）
- `bearing_body`: 机体坐标系方位角（度）
- `track_id`: 目标跟踪 ID
- `cam_idx`: 摄像头索引
- `command`: 识别的灯语命令
- `description`: 命令描述（中文）
- `class_id`: YOLO 检测类别 ID
- `timestamp`: 时间戳

## 🔧 核心实现

### 生产者线程（camera_worker）

```python
def camera_worker(self, cam_idx):
    # 1. 加载 YOLO 模型
    model = YOLO(self.model_path)
    
    # 2. 初始化摄像头
    cap = self.initialize_camera(cam_idx)
    
    # 3. 循环读取和检测
    while True:
        ret, frame = cap.read()
        results = model.track(frame, ...)
        
        # 4. 将检测数据放入队列（不直接发送）
        self.queue.put_nowait(detection_data)
```

### 消费者主线程（run）

```python
def run(self):
    # 1. 启动所有生产者线程
    for cam_idx in self.camera_indices:
        thread = threading.Thread(target=self.camera_worker, args=(cam_idx,))
        thread.start()
    
    # 2. 主线程消费队列数据
    while True:
        data = self.queue.get()
        
        # 3. 统一发送 ZMQ 消息
        if data['type'] == 'detection':
            self.socket.send_string(f"perception {json.dumps(data)}")
```

## ⚙️ 性能优化

### 队列管理

- **队列大小**: `maxsize=100`，可根据实际情况调整
- **丢帧策略**: 使用 `put_nowait()`，队列满时自动丢弃数据，避免阻塞生产者

### 摄像头优化

- **缓冲区大小**: `CAP_PROP_BUFFERSIZE = 1`，减少延迟
- **编码格式**: `FOURCC = MJPG`，提高读取速度

### YOLO 优化

- **置信度阈值**: `conf=0.55`
- **IOU 阈值**: `iou=0.6`
- **图像大小**: `imgsz=(480, 640)`

## 🐛 故障排查

### 摄像头无法打开

```bash
# 检查摄像头设备
ls /dev/video*

# 测试摄像头
v4l2-ctl --list-devices
```

### ZMQ 端口被占用

```bash
# 查看端口占用
sudo netstat -tulpn | grep 5555

# 修改端口
ZMQ_PORT = 5556  # 使用其他端口
```

### CUDA 内存不足

- 减少同时运行的摄像头数量
- 降低 YOLO 模型输入尺寸
- 使用更小的 YOLO 模型（如 YOLOv8n）

## 📈 监控和调试

### 查看线程状态

```python
import threading
print(threading.enumerate())  # 查看所有活动线程
```

### 查看队列状态

```python
print(f"队列大小: {self.queue.qsize()}")
print(f"队列是否满: {self.queue.full()}")
```

## 🎯 最佳实践

1. **优雅退出**: 使用 `Ctrl+C` 触发 `KeyboardInterrupt`，等待所有线程结束
2. **资源清理**: 确保调用 `cleanup()` 方法关闭 ZMQ 资源
3. **日志记录**: 通过队列统一打印日志，避免多线程日志混乱
4. **错误处理**: 每个线程独立捕获异常，不影响其他线程

## 📚 相关文件

- `vision_pub.py`: 主程序
- `test_vision_pub.py`: 测试订阅者
- `REFACTORING_SUMMARY.md`: 重构详细说明

## 🤝 贡献

欢迎提交 Issue 和 Pull Request！

## 📄 许可证

MIT License

