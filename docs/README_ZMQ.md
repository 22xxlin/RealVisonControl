# 🚀 ZeroMQ 分布式架构 - 快速入门

## 🎯 一句话总结

**用两个独立的终端窗口运行视觉和控制，彻底解耦终端输出！**

---

## ⚡ 3分钟快速开始

### 1️⃣ 安装依赖
```bash
pip install pyzmq
```

### 2️⃣ 测试连接（推荐首次使用）
```bash
cd /home/nvidia/Downloads/Ros/pseudo_ros_architecture
python3 test_zmq_connection.py
```

### 3️⃣ 启动系统

**终端1（视觉）：**
```bash
python3 vision_pub.py
```

**终端2（控制）：**
```bash
python3 control_sub.py
```

### 4️⃣ 停止系统
在两个终端分别按 `Ctrl+C`

---

## 📁 核心文件

| 文件 | 作用 | 行数 |
|------|------|------|
| `vision_pub.py` | 视觉发布者（替代 VisionNode） | 345 |
| `control_sub.py` | 控制订阅者（替代 ControlNode） | 384 |
| `test_zmq_connection.py` | 连接测试工具 | 130 |
| `quick_start.sh` | 交互式启动脚本 | 90 |

---

## 🔧 常用配置

### 修改摄像头
编辑 `vision_pub.py` 第 332 行：
```python
CAMERA_INDICES = [4]  # 改为 [0], [2], [6]
```

### 修改机器人ID
编辑 `control_sub.py` 第 357 行：
```python
ROBOT_ID = 8  # 改为你的机器人ID
```

### 启用 Mock 模式（无硬件测试）
编辑 `control_sub.py` 第 358 行：
```python
ENABLE_CONTROL = False
```

---

## 📚 详细文档

| 文档 | 内容 |
|------|------|
| [ZMQ_DISTRIBUTED_GUIDE.md](ZMQ_DISTRIBUTED_GUIDE.md) | 完整使用指南、配置说明、故障排查 |
| [MIGRATION_SUMMARY.md](MIGRATION_SUMMARY.md) | 重构总结、架构对比、性能分析 |
| [CHECKLIST.md](CHECKLIST.md) | 测试检查清单、功能验证 |

---

## 🎨 架构对比

### 原架构（threading）
```
main_system.py
├── VisionNode (Thread)
│   └── 更新 robot_state
└── ControlNode (Thread)
    └── 读取 robot_state
```

### 新架构（ZeroMQ）
```
终端1: vision_pub.py (独立进程)
  └── ZMQ PUB → tcp://*:5555

终端2: control_sub.py (独立进程)
  └── ZMQ SUB ← tcp://localhost:5555
```

---

## ✅ 核心优势

| 特性 | 原架构 | 新架构 |
|------|--------|--------|
| 终端输出 | ❌ 混在一起 | ✅ 完全独立 |
| 进程模型 | ❌ 单进程多线程 | ✅ 真正多进程 |
| 可扩展性 | ❌ 受限单机 | ✅ 可跨网络 |
| 调试难度 | ❌ 日志混杂 | ✅ 清晰独立 |
| CPU 利用 | ❌ 单核心 | ✅ 多核并行 |

---

## 🛡️ 安全特性

1. **超时保护**：1秒无消息自动停止
2. **非阻塞控制**：每帧计算一次速度
3. **异常处理**：捕获所有关键异常
4. **优雅退出**：Ctrl+C 正常清理资源

---

## 🐛 常见问题

### Q: 控制节点显示"超时"
**A:** 视觉节点未检测到目标，属于正常现象

### Q: 机器人不动
**A:** 检查是否启用了 Mock 模式（`ENABLE_CONTROL = False`）

### Q: 摄像头初始化失败
**A:** 运行 `ls /dev/video*` 查看可用设备，修改 `CAMERA_INDICES`

---

## 🎯 支持的指令

| 指令 | 描述 | 灯语模式 |
|------|------|---------|
| APPROACH | 靠近目标 | 2200 |
| RETREAT | 远离目标 | 1100 |
| S_SHAPE | S形运动 | 4400 |
| CIRCLE | 圆形运动 | 5500 |
| FORWARD | 前进 | 220 |
| LEFT | 左移 | 330 |
| RIGHT | 右移 | 110 |
| REVERSE | 后退 | 550 |
| STOP | 停止 | 440 |

---

## 📊 消息格式

```json
{
  "distance": 1.23,
  "bearing_body": 45.0,
  "track_id": 1,
  "cam_idx": 4,
  "command": "APPROACH",
  "description": "靠近",
  "timestamp": 1234567890.123
}
```

---

## 🚀 使用快速启动脚本

```bash
./quick_start.sh
```

交互式菜单：
1. 测试 ZeroMQ 连接
2. 启动视觉发布者
3. 启动控制订阅者
4. 查看使用指南
5. 退出

---

## 📞 获取帮助

1. **测试连接**：`python3 test_zmq_connection.py`
2. **查看详细指南**：`cat ZMQ_DISTRIBUTED_GUIDE.md`
3. **检查清单**：`cat CHECKLIST.md`

---

## 🎉 开始使用

```bash
# 方法1：使用快速启动脚本
./quick_start.sh

# 方法2：手动启动
# 终端1
python3 vision_pub.py

# 终端2
python3 control_sub.py
```

**祝你使用愉快！** 🎊

---

**最后更新：** 2024-12-05  
**版本：** 1.0.0  
**作者：** Amazon Q

