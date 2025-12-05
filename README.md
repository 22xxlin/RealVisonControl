# Pseudo ROS 架构 - ZeroMQ 分布式系统

基于 ZeroMQ 的分布式机器人控制系统，视觉和控制在独立进程中运行。

## 快速开始

### 安装依赖
```bash
pip install pyzmq opencv-python ultralytics numpy
```

### 启动系统

**终端1（视觉）：**
```bash
python3 vision_pub.py
```

**终端2（控制）：**
```bash
python3 control_sub.py
```

停止：在两个终端按 `Ctrl+C`

## 架构

```
终端1: vision_pub.py  ──ZMQ PUB──►  终端2: control_sub.py
       (视觉检测)      tcp://5555        (机器人控制)
```

## 配置

### vision_pub.py
```python
CAMERA_INDICES = [4]        # 摄像头索引
MODEL_PATH = "best.pt"      # YOLO模型
ZMQ_PORT = 5555            # ZMQ端口
```

### control_sub.py
```python
ROBOT_ID = 8               # 机器人ID
ENABLE_CONTROL = True      # Mock模式开关
CONTROL_HZ = 20.0         # 控制频率
```

## 支持的指令

| 指令 | 灯语 | 描述 |
|------|------|------|
| APPROACH | 2200 | 靠近 |
| RETREAT | 1100 | 远离 |
| FORWARD | 220 | 前进 |
| LEFT | 330 | 左移 |
| RIGHT | 110 | 右移 |
| STOP | 440 | 停止 |

## 项目结构

```
├── vision_pub.py          # 视觉发布者
├── control_sub.py         # 控制订阅者
├── demo.py                # 演示程序
├── test_zmq_connection.py # 连接测试
├── docs/                  # 详细文档
└── old_version/           # 旧版本（多线程）
```

## 详细文档

- [docs/README_ZMQ.md](docs/README_ZMQ.md) - 详细使用指南
- [docs/ZMQ_DISTRIBUTED_GUIDE.md](docs/ZMQ_DISTRIBUTED_GUIDE.md) - 完整配置说明
- [PROJECT_STRUCTURE.md](PROJECT_STRUCTURE.md) - 项目结构

## 常见问题

**Q: 控制节点显示"超时"**
A: 视觉节点未检测到目标，正常现象

**Q: 机器人不动**
A: 检查 `ENABLE_CONTROL = True`

**Q: 摄像头失败**
A: 运行 `ls /dev/video*` 检查设备，修改 `CAMERA_INDICES`

# RealVisonControl
