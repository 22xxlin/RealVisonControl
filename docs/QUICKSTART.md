# 🚀 快速开始指南

## 📋 前置要求

### 1. 硬件要求
- ✅ USB 摄像头（支持索引 0, 2, 4, 6）
- ✅ 全向移动机器人（可选，Mock 模式不需要）

### 2. 软件依赖
```bash
# Python 3.8+
python3 --version

# 安装依赖
pip3 install opencv-python numpy ultralytics
```

### 3. YOLO 模型
- 模型路径：`/home/nvidia/Downloads/0821Car3/weights/best.engine`
- 确保模型文件存在

---

## 🎯 三种运行模式

### 模式 1: 演示模式（推荐新手）

不需要摄像头和模型，快速理解架构原理。

```bash
cd /home/nvidia/Downloads/ros/pseudo_ros_architecture
python3 demo.py
```

**预期输出：**
```
📹 [视觉线程] 发布 #1: 距离=3.00m, 角度=90.0°, 指令=APPROACH
🤖 [控制线程] 执行指令: APPROACH | 距离=3.00m | 角度=90.0°
...
```

**学到什么：**
- ✅ 视觉线程如何发布数据
- ✅ 控制线程如何订阅数据
- ✅ 共享状态如何同步
- ✅ 线程安全机制

---

### 模式 2: Mock 模式（有摄像头，无机器人）

使用真实摄像头和 YOLO，但不发送机器人控制指令。

**步骤 1：修改配置**
编辑 `main_system.py`：
```python
system = PseudoROSSystem(
    model_path=model_path,
    robot_id=8,
    enable_control=False,  # 设置为 False
    camera_indices=[0, 2, 4, 6]
)
```

**步骤 2：运行**
```bash
python3 main_system.py
```

**预期输出：**
```
✅ 视觉节点已启动
✅ 控制节点已启动（Mock模式）
📊 [状态] 距离:1.50m | 角度:45.0° | 指令:APPROACH
🛑 [Mock] 安全停止
```

**学到什么：**
- ✅ YOLO 检测是否正常
- ✅ 距离和角度计算是否准确
- ✅ 灯语识别是否正确
- ✅ 不会误触发机器人控制

---

### 模式 3: 完整模式（生产环境）

使用真实摄像头、YOLO 和机器人控制。

**步骤 1：确认机器人控制模块**
```bash
# 检查控制模块路径
ls -l ../real_ws/control/simple_robot_control.py
```

**步骤 2：确认配置**
编辑 `main_system.py`：
```python
system = PseudoROSSystem(
    model_path=model_path,
    robot_id=8,
    enable_control=True,  # 设置为 True
    camera_indices=[0, 2, 4, 6]
)
```

**步骤 3：运行**
```bash
python3 main_system.py
```

**预期输出：**
```
🤖 机器人控制器初始化成功 - ID: 8
✅ 视觉节点已启动
✅ 控制节点已启动
📊 [状态] 距离:1.50m | 角度:45.0° | 指令:APPROACH
➡️ APPROACH: 距离误差0.50m | 方向45.0° | vx=0.150, vy=0.150
```

**安全提示：**
- ⚠️ 确保机器人周围无障碍物
- ⚠️ 准备好紧急停止（Ctrl+C）
- ⚠️ 首次运行建议在空旷区域

---

## 🔧 常见问题排查

### 问题 1: 摄像头打不开
```
❌ 摄像头 0 初始化失败
```

**解决方案：**
```bash
# 检查摄像头设备
ls /dev/video*

# 测试摄像头
python3 -c "import cv2; cap = cv2.VideoCapture(0); print('OK' if cap.isOpened() else 'FAIL')"

# 检查权限
sudo usermod -a -G video $USER
# 重新登录生效
```

---

### 问题 2: 模型加载失败
```
❌ 模型文件不存在: /path/to/model
```

**解决方案：**
```bash
# 检查模型文件
ls -lh /home/nvidia/Downloads/0821Car3/weights/best.engine

# 如果路径不同，修改 main_system.py 中的 model_path
```

---

### 问题 3: 机器人控制模块加载失败
```
⚠️ 机器人控制模块加载失败
```

**解决方案：**
```bash
# 检查控制模块路径
cd /home/nvidia/Downloads/ros
ls real_ws/control/simple_robot_control.py

# 如果不存在，使用 Mock 模式：
# 在 main_system.py 中设置 enable_control=False
```

---

### 问题 4: 数据一直超时
```
⚠️ 目标数据超时 (1.50s)，执行安全停止
```

**可能原因：**
1. YOLO 检测不到目标 → 检查目标是否在视野内
2. 置信度阈值过高 → 调低 `vision_node.py` 中的 `conf=0.55`
3. 摄像头帧率过低 → 检查 USB 带宽

**调试方法：**
```python
# 在 vision_node.py 的 camera_worker 中添加：
print(f"检测到 {len(results[0].boxes)} 个目标")
```

---

## 📊 性能调优

### 1. 调整控制频率
编辑 `main_system.py`：
```python
self.control_node = ControlNode(
    robot_state=self.robot_state,
    robot_id=robot_id,
    enable_control=enable_control,
    control_hz=30.0  # 默认 20.0，提高到 30Hz
)
```

### 2. 调整超时时间
编辑 `control_node.py`：
```python
self.target_timeout = 2.0  # 默认 1.0 秒，增加到 2.0 秒
```

### 3. 调整摄像头分辨率
编辑 `vision_node.py`：
```python
self.frame_width = 320   # 默认 640，降低分辨率提高帧率
self.frame_height = 240  # 默认 480
```

### 4. 选择性使用摄像头
编辑 `main_system.py`：
```python
camera_indices=[0, 4]  # 只使用前后摄像头
```

---

## 🎓 学习路径

### 第一步：理解架构
1. 阅读 `README.md`
2. 运行 `demo.py` 理解通信机制
3. 查看 `ARCHITECTURE_COMPARISON.md` 理解改进点

### 第二步：阅读源码
1. `robot_shared_state.py` - 理解共享状态
2. `vision_node.py` - 理解视觉感知
3. `control_node.py` - 理解控制逻辑
4. `main_system.py` - 理解系统组装

### 第三步：动手实验
1. 修改 `demo.py`，添加新的模拟数据
2. 在 Mock 模式下运行，观察日志
3. 添加自定义指令和控制策略

### 第四步：生产部署
1. 在完整模式下测试
2. 根据实际情况调优参数
3. 添加日志和监控

---

## 📚 进阶主题

### 多目标处理
修改 `robot_shared_state.py`，支持存储多个目标：
```python
self._targets = {}  # {track_id: target_info}
```

### 历史轨迹记录
在 `robot_shared_state.py` 中添加：
```python
self._trajectory = []  # 记录目标运动轨迹
```

### 性能监控
创建监控线程订阅共享状态：
```python
def monitor_thread():
    while True:
        state = robot_state.get_latest_state()
        log_to_file(state)
        time.sleep(1.0)
```

---

## 🆘 获取帮助

### 日志调试
在各模块中添加 `print()` 语句：
```python
print(f"[DEBUG] 当前距离: {distance:.2f}m")
```

### 状态检查
在 `main_system.py` 的 `run_forever()` 中：
```python
# 定期打印详细状态
state = self.robot_state.get_latest_state()
print(f"完整状态: {state}")
```

---

## ✅ 验收清单

运行以下命令，确保系统正常：

- [ ] `python3 demo.py` → 演示成功
- [ ] `python3 main_system.py` (Mock) → 检测正常
- [ ] `python3 main_system.py` (完整) → 控制正常
- [ ] Ctrl+C 优雅退出 → 无错误
- [ ] 超时保护触发 → 自动停止

全部通过即可投入使用！🎉

