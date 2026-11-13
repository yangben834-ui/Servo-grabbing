# H3001_grasp

基于 **H3001 六轴机械臂** 的 RGB-D 视觉抓取系统：YOLO 分割 + 深度点云估计瓶类目标的
6D 抓取位姿，经 ROS 2 状态机与 CAN 桥驱动机械臂完成「识别 → 跟随 → 抓取 → 放置」全流程。

## 演示

真机抓取演示视频：[▶ demo.mp4](demo.mp4)

## 抓取链路

```text
Orbbec / RealSense 相机 (RGB-D)
        │
        ▼
wutishibiezitaijiance-master    YOLO 分割 + 点云 PCA + 桌面法向 → 6D 抓取位姿
        │ 手眼变换 → base_link，写 JSON
        ▼
bottle_grasp_executor           抓取状态机（IDLE → 悬停跟随 → 下降 → 抓取 → 放置 → 回位）
        │  /control/move_j | move_p | move_l
        ▼
h3001_bridge                    CAN ↔ ROS 2 桥接
        │  SocketCAN can0 @ 1Mbps
        ▼
H3001 六轴机械臂  +  Gloria-M 夹爪
```

## 目录结构

| 目录 | 说明 |
|---|---|
| `wutishibiezitaijiance-master/` | 感知：YOLO 分割 + 抓取位姿估计（含 PT/RKNN 模型与 ROS 节点） |
| `bottle_grasp_executor/` | 抓取执行状态机（ROS 2 包） |
| `h3001_bridge/` | CAN ↔ ROS 2 桥（ROS 2 包） |
| `h3001_msgs/` | 机械臂状态反馈消息（ROS 2 包） |
| `hand_eye_calibrate/` | 手眼标定工具（eye-in-hand / eye-to-hand） |
| `Gloria-M-SDK/` | Gloria-M 夹爪 SDK |
| `h3001_console.py` / `h3001_tuner.py` | 图形控制台 / 调参界面 |
| `SETUP_NEW_MACHINE.md` | 新机器部署清单 |

## 硬件与软件

- **机械臂**：H3001 六轴，CAN 接口（gs_usb USB-CAN，`can0` @ 1Mbps）
- **相机**：Orbbec / RealSense RGB-D（USB 直连）
- **夹爪**：Gloria-M（串口 `/dev/ttyACM0`）
- **平台**：Ubuntu 22.04 + ROS 2 Humble；感知侧 conda（Python 3.12），ROS 侧系统 Python

## 快速开始

```bash
# 1. 构建 ROS 工作空间
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to h3001_bridge bottle_grasp_executor
source install/setup.bash

# 2. 起 can0
sudo ip link set can0 up type can bitrate 1000000

# 3. 一键控制台（依次拉起 桥接 → 感知 → 执行器）
python3 h3001_console.py
```

> 详细的分步启动、状态机说明与实测踩坑记录见
> [`bottle_grasp_executor/README_H3001_grasp.md`](bottle_grasp_executor/README_H3001_grasp.md)。

## 文档

- [运行与踩坑](bottle_grasp_executor/README_H3001_grasp.md) — 四个终端启动顺序、状态机、实测问题
- [参数配置](bottle_grasp_executor/README_参数配置.md) — 抓取参数速查
- [感知](wutishibiezitaijiance-master/README.md) — YOLO 分割 + 抓取位姿估计算法
- [手眼标定](hand_eye_calibrate/README.md) — 标定工具与流程
- [新机器部署](SETUP_NEW_MACHINE.md) — 换机器部署清单
