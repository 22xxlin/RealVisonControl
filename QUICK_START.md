# vision_pub.py 快速上手指南

## ✅ 重构完成

`vision_pub.py` 已成功重构为支持 **4个摄像头并行工作** 的多线程架构！

## 🎯 核心改动

### 1. 新增导入
```python
import queue      # 线程安全队列
import threading  # 多线程支持
```

### 2. 初始化队列
```python
self.queue = queue.Queue(maxsize=100)  # 在 __init__ 中
```

### 3. 生产者线程（camera_worker）
- ❌ 不再直接调用 `self.socket.send_string()`
- ❌ 不再直接使用 `print()`
- ✅ 使用 `self.queue.put_nowait(data)` 放入队列
- ✅ 队列满时自动丢弃数据（避免阻塞）

### 4. 消费者主线程（run）
- ✅ 启动 4 个生产者线程
- ✅ 主线程循环执行 `data = self.queue.get()`
- ✅ 统一负责 ZMQ 发送和日志打印

## 🚀 运行方式

### 启动发布者（4个摄像头）
```bash
cd /home/nvidia/Downloads/Ros/pseudo_ros_architecture
python3 vision_pub.py
```

### 测试订阅者
```bash
python3 test_vision_pub.py
```

## 📊 预期输出

### 发布者启动信息
```
============================================================
🎥 视觉发布者 (ZeroMQ PUB) - 多摄像头并行模式
============================================================
📡 发布地址: tcp://*:5555
📹 摄像头: [0, 2, 4, 6]
   - Cam0: 后方 (180°)
   - Cam2: 右侧 (-90°)
   - Cam4: 前方 (0°)
   - Cam6: 左侧 (90°)
🤖 模型路径: /home/nvidia/Downloads/Ros/0821Car3/weights/best.engine
🔄 架构: 生产者-消费者模型 (4个生产者线程 + 1个消费者主线程)
============================================================
按 Ctrl+C 停止

✅ 视觉发布者初始化完成 - ZMQ 绑定到 tcp://*:5555
🚀 启动视觉发布者 - 多摄像头并行模式
📹 摄像头列表: [0, 2, 4, 6]
✅ 摄像头 0 线程已启动
✅ 摄像头 2 线程已启动
✅ 摄像头 4 线程已启动
✅ 摄像头 6 线程已启动

📡 主线程开始消费队列数据...

✅ 摄像头 0 模型加载成功
✅ 摄像头 2 模型加载成功
✅ 摄像头 4 模型加载成功
✅ 摄像头 6 模型加载成功
✅ 摄像头 0 初始化成功
✅ 摄像头 2 初始化成功
✅ 摄像头 4 初始化成功
✅ 摄像头 6 初始化成功
🎥 [Cam4] Sent: FORWARD | Dist=2.50m | Bearing=5.3° | TrackID=1
🎥 [Cam0] Sent: LEFT | Dist=3.20m | Bearing=185.7° | TrackID=2
...
```

## 🔍 关键代码片段

### 生产者：放入队列
```python
# camera_worker 方法中
detection_data = {
    'type': 'detection',
    'distance': float(distance),
    'bearing_body': float(bearing_body),
    'cam_idx': int(cam_idx),
    'command': command,
    # ...
}

try:
    self.queue.put_nowait(detection_data)  # 非阻塞放入
except queue.Full:
    pass  # 队列满了，丢弃该帧
```

### 消费者：从队列取出并发送
```python
# run 方法中
while True:
    data = self.queue.get()  # 阻塞等待
    
    if data.get('type') == 'detection':
        # 移除 type 字段
        detection_data = {k: v for k, v in data.items() if k != 'type'}
        message = json.dumps(detection_data)
        self.socket.send_string(f"perception {message}")  # 主线程发送
        
        # 主线程打印日志
        if detection_data['command'] != 'IDLE':
            print(f"🎥 [Cam{cam_idx}] Sent: {cmd} | ...")
    
    self.queue.task_done()
```

## ⚡ 性能特点

- **并行处理**: 4个摄像头同时工作，充分利用多核 CPU
- **线程安全**: ZMQ Socket 只在主线程操作，避免竞争
- **队列缓冲**: 100个消息的缓冲区，平滑突发流量
- **自动丢帧**: 队列满时丢弃数据，保证实时性

## 🛠️ 故障排查

### 问题1: 摄像头初始化失败
```bash
# 检查摄像头设备
ls /dev/video*

# 如果某个摄像头不存在，修改 CAMERA_INDICES
CAMERA_INDICES = [4]  # 只使用前置摄像头
```

### 问题2: 模型加载失败
```bash
# 检查模型路径
ls -lh /home/nvidia/Downloads/Ros/0821Car3/weights/best.engine

# 修改为正确路径
MODEL_PATH = "/path/to/your/model.engine"
```

### 问题3: ZMQ 端口被占用
```bash
# 查看端口占用
sudo netstat -tulpn | grep 5555

# 修改端口
ZMQ_PORT = 5556
```

## 📚 相关文档

- `README_VISION_PUB.md`: 完整文档
- `REFACTORING_SUMMARY.md`: 重构详细说明
- `test_vision_pub.py`: 测试订阅者脚本

## 🎉 完成清单

- ✅ 支持 4 个摄像头并行工作
- ✅ 生产者-消费者模型
- ✅ 线程安全的 ZMQ 发送
- ✅ 队列缓冲和丢帧策略
- ✅ 保留原有 YOLO 检测逻辑
- ✅ 保留原有灯语识别逻辑
- ✅ 代码结构清晰易维护

## 🚦 下一步

1. 运行 `python3 vision_pub.py` 测试多摄像头
2. 运行 `python3 test_vision_pub.py` 验证数据接收
3. 根据实际情况调整队列大小和摄像头参数
4. 集成到完整的 ROS 架构中

---

**重构完成！** 🎊 现在可以同时使用 4 个摄像头进行 360° 全方位检测了！

