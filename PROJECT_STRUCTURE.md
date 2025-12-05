# 项目结构说明

## 根目录文件（当前版本 - ZMQ实现）
- `demo.py` - 主演示程序
- `vision_pub.py` - 视觉节点（发布者）
- `control_sub.py` - 控制节点（订阅者）
- `check_dependencies.py` - 依赖检查工具
- `test_modules.py` - 模块测试
- `test_zmq_connection.py` - ZMQ连接测试
- `quick_start.sh` - 快速启动脚本
- `README.md` - 项目主文档

## 文件夹
- `docs/` - 所有项目文档
  - 架构对比、图表、检查清单、最终报告等
- `old_version/` - 旧版本代码（共享内存实现）
  - control_node.py
  - vision_node.py
  - robot_shared_state.py
  - main_system.py

## 快速开始
运行当前版本：
```bash
python demo.py
```

查看旧版本：
```bash
cd old_version/
python main_system.py
```

