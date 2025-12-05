# 🎉 开始使用 ZeroMQ 分布式架构

## ✅ 重构已完成！

你的项目已经成功从**单进程多线程**架构重构为**真正的多进程分布式架构**！

---

## 🚀 立即开始（3步）

### 第1步：检查依赖
```bash
python3 check_dependencies.py
```

### 第2步：测试连接
```bash
python3 test_zmq_connection.py
```

### 第3步：启动系统

**终端1（视觉）：**
```bash
python3 vision_pub.py
```

**终端2（控制）：**
```bash
python3 control_sub.py
```

---

## 📁 重要文件一览

### 🔥 核心文件（必读）
| 文件 | 用途 | 优先级 |
|------|------|--------|
| [README_ZMQ.md](README_ZMQ.md) | **快速入门指南** | ⭐⭐⭐⭐⭐ |
| `vision_pub.py` | 视觉发布者（替代 VisionNode） | ⭐⭐⭐⭐⭐ |
| `control_sub.py` | 控制订阅者（替代 ControlNode） | ⭐⭐⭐⭐⭐ |

### 📚 详细文档
| 文件 | 内容 | 何时阅读 |
|------|------|---------|
| [ZMQ_DISTRIBUTED_GUIDE.md](ZMQ_DISTRIBUTED_GUIDE.md) | 完整使用指南、配置、故障排查 | 需要详细配置时 |
| [MIGRATION_SUMMARY.md](MIGRATION_SUMMARY.md) | 重构总结、架构对比 | 想了解改动细节时 |
| [CHECKLIST.md](CHECKLIST.md) | 测试检查清单 | 验证功能时 |

### 🛠️ 工具脚本
| 文件 | 用途 |
|------|------|
| `test_zmq_connection.py` | 测试 ZeroMQ 连接 |
| `check_dependencies.py` | 检查依赖和环境 |
| `quick_start.sh` | 交互式启动菜单 |

---

## 🎯 核心改进

### ✅ 解决的问题
- ✅ **终端输出混杂** → 现在完全独立
- ✅ **单进程瓶颈** → 真正的多进程并行
- ✅ **调试困难** → 清晰的独立日志
- ✅ **扩展受限** → 可跨网络部署

### 🚀 新增特性
- ✅ **超时保护**：1秒无消息自动停止
- ✅ **非阻塞控制**：每帧计算速度
- ✅ **优雅退出**：Ctrl+C 正常清理
- ✅ **Mock 模式**：无硬件也能测试

---

## 📊 架构对比

### 原架构（已废弃）
```
main_system.py
├── VisionNode (Thread) ──┐
│                         │ 共享内存
└── ControlNode (Thread) ─┘ (robot_state)

❌ 终端输出混在一起
❌ 单进程受限
```

### 新架构（当前）
```
终端1: vision_pub.py (独立进程)
  └── ZMQ PUB → tcp://*:5555
                    ↓
终端2: control_sub.py (独立进程)
  └── ZMQ SUB ← tcp://localhost:5555

✅ 终端输出完全独立
✅ 真正多进程并行
```

---

## 🔧 常用配置

### 修改摄像头
```bash
# 编辑 vision_pub.py 第 332 行
CAMERA_INDICES = [4]  # 改为 [0], [2], [6]
```

### 修改机器人ID
```bash
# 编辑 control_sub.py 第 357 行
ROBOT_ID = 8  # 改为你的机器人ID
```

### 启用 Mock 模式
```bash
# 编辑 control_sub.py 第 358 行
ENABLE_CONTROL = False  # 无硬件测试
```

---

## 🎬 使用场景

### 场景1：开发调试
```bash
# 终端1：视觉节点
python3 vision_pub.py

# 终端2：控制节点（Mock模式）
# 编辑 control_sub.py: ENABLE_CONTROL = False
python3 control_sub.py
```

### 场景2：实际运行
```bash
# 终端1：视觉节点
python3 vision_pub.py

# 终端2：控制节点（实际控制）
# 编辑 control_sub.py: ENABLE_CONTROL = True
python3 control_sub.py
```

### 场景3：跨网络部署
```bash
# 机器A（视觉）
python3 vision_pub.py  # 绑定到 tcp://*:5555

# 机器B（控制）
# 编辑 control_sub.py: ZMQ_ADDRESS = "tcp://192.168.1.100:5555"
python3 control_sub.py
```

---

## 🐛 遇到问题？

### 快速诊断
```bash
# 1. 检查依赖
python3 check_dependencies.py

# 2. 测试连接
python3 test_zmq_connection.py

# 3. 查看日志
# 终端输出会显示详细的错误信息
```

### 常见问题
| 问题 | 解决方案 |
|------|---------|
| 控制节点显示"超时" | 正常现象，视觉节点未检测到目标 |
| 机器人不动 | 检查是否启用了 Mock 模式 |
| 摄像头初始化失败 | 运行 `ls /dev/video*` 查看可用设备 |

详细故障排查请查看 [ZMQ_DISTRIBUTED_GUIDE.md](ZMQ_DISTRIBUTED_GUIDE.md)

---

## 📞 获取帮助

1. **快速入门** → [README_ZMQ.md](README_ZMQ.md)
2. **详细指南** → [ZMQ_DISTRIBUTED_GUIDE.md](ZMQ_DISTRIBUTED_GUIDE.md)
3. **测试清单** → [CHECKLIST.md](CHECKLIST.md)
4. **重构总结** → [MIGRATION_SUMMARY.md](MIGRATION_SUMMARY.md)

---

## 🎉 开始你的旅程

```bash
# 使用交互式菜单（推荐）
./quick_start.sh

# 或者手动启动
python3 vision_pub.py    # 终端1
python3 control_sub.py   # 终端2
```

---

## 📈 下一步优化

完成基本测试后，你可以：
- [ ] 调整控制参数（速度、距离阈值）
- [ ] 添加更多摄像头支持
- [ ] 实现跨网络部署
- [ ] 添加性能监控
- [ ] 实现日志系统

---

**祝你使用愉快！** 🎊

如果这个重构对你有帮助，别忘了给项目点个星！⭐

---

**最后更新：** 2024-12-05  
**版本：** 1.0.0  
**架构：** ZeroMQ PUB-SUB 分布式

