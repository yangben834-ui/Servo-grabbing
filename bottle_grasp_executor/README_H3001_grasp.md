# H3001 六轴机械臂抓取链路

本文档描述 H3001 六轴机械臂的 RGB-D 抓取实验：环境配置、四个终端的启动顺序、
触发与人工确认方式、关键参数位置，以及实测踩过的坑和当前未解决的问题。

只写手工分步命令，不提供一键脚本 —— 每个终端要单独看日志，串成一个脚本会丢掉
中间的输出。

---

## 0. 链路总览

```text
Orbbec 相机 (USB, 直连 SDK)
        │  RGB-D
        ▼
test_yolo_realsense_v3f.py          YOLO 分割 + 点云 → 抓取点
        │  手眼变换 (H3003 外参) → base_link
        │  写 JSON (原子替换)
        ▼
/tmp/bottle_grasp_candidates.json
        │  轮询
        ▼
bottle_grasp_executor               状态机:
        │                           IDLE → GRIPPER(张开) → WAIT_TARGET
        │                                 → PRE_GRASP → TRACK_WAIT → DESCEND
        │                                 → GRASP_WAIT → GRIPPER(闭合命令)
        │                                 → LIFT → PLACE_MOVE
        │                                 → PLACE_WAIT → GRIPPER(张开命令)
        │                                 → HOME → IDLE
        │                           TRACK_WAIT = 悬停跟随 + 下降前确认
        │                                        (auto_grasp_stable_time>0 时
        │                                         到位后物体位置连续 3s 不变
        │                                         就自动降; Enter 可提前)
        │                           GRASP_WAIT = 等确认「现在夹」
        │                                        (grasp_confirm_required=false
        │                                         时不等, dwell 一到就夹)
        │                           GRIPPER    = 夹爪动作 (非运动状态,
        │                                        gripper_enabled=false 时整段跳过)
        │                           PLACE_* = 放到 place_pose_reported_mm_deg
        │                                     (place_enabled=false 时整段跳过)
        │                                     PLACE_MOVE = **直线直达**那个位姿本身
        │                                                  (判到位, 无中间点)
        │                           PLACE_WAIT = 等确认「现在放」
        │                                        (place_confirm_required=false
        │                                         时不等, dwell 一到就放)
        │                           HOME = 回待机位 (成功/失败都走,
        │                                  一条绝对指令交给固件规划,
        │                                  期间拒绝新触发)
        │  /control/move_j (sensor_msgs/JointState, **弧度**, 按名字对) 关节空间 (MOVE_J)
        │  /control/move_p (geometry_msgs/PoseStamped, base_link)      点到点 (MOVE_P)
        │  /control/move_l (同上, 同一套单位)                          直线 (MOVE_L)
        │    ↑ **优先走 move_j**: 节点自己求逆解 (位姿 -> 关节角), 绕开固件那条
        │      「收位姿、逐点解逆解」的路 (§5.15)。解不出来就回落到下面两条 ——
        │      回落时只有 DESCEND 走 move_l, 其余走 move_p (§5.11)
        ▼
h3001_bridge                        ROS 话题 ↔ CAN 帧
        │  SocketCAN can0 @ 1Mbps
        ▼
H3001 六轴机械臂 (MCU)
        │  /feedback/tcp_pose, /feedback/arm_status
        └──────────► 回到 executor 做到位判定

executor ──spawn──► scripts/gripper_grip_hold.py    Gloria-M 夹爪
        │            --mode open | close          串口 /dev/ttyACM0
        │            stdout: [start]/[stop]/[abort] 等排障日志
        │            stdin : 一行 OPEN = 松开 (只在保持时才用)
        └──────────► **一次性命令**: 起一个进程跑一条命令, 等它退出就往下走
                     (gripper_enabled=false 时不起)
                     `gripper_hold_torque>0` 时闭合那一条**不退出**: 留在原地持续
                     发 MIT 力矩帧顶着夹持力, 到放置时由节点写 OPEN 让它松手
```

三个进程各自独立启动、独立退出。**感知不经过 ROS 相机节点**，executor 也不订阅
相机话题，它只认 JSON 文件或候选话题。夹爪子进程由 executor 自己起、自己收，
只在 `gripper_enabled=true` 时存在。

---

## 1. 环境

### 1.1 硬件

| 项 | 值 |
|---|---|
| 机械臂 | H3001 六轴，CAN 接口，MCU 固件 |
| CAN 适配器 | gs_usb USB-CAN，接口名 `can0`，波特率 `1000000` |
| 相机 | Orbbec RGB-D，USB 直连（不经 ROS 相机节点） |
| 计算平台 | amd64 主机（RK3588 板端另见感知包 README） |

### 1.2 软件

| 项 | 值 |
|---|---|
| ROS 2 | Humble（`/opt/ros/humble`） |
| 工作空间 | `/home/qin/zdl_fz` |
| Python（感知） | `/home/qin/miniforge3/bin/python3` |
| Orbbec SDK | `/opt/OrbbecSDK_v2.9.3` |
| H3001 SDK | `src/SDK_linux`（`sdk_linux.py`）；旧版 `src/sdk_can/h3001sdk`（`h3001_sdk.py`）仅作回退 |
| 手眼标定 | `/home/qin/hand_eye_calibrate/H3003/hand_eye_extrinsics.yaml` |
| YOLO 模型 | `src/wutishibiezitaijiance-master/best_train_multi.pt` |

感知侧实测可用的包版本（在该环境下导入全部正常）：

```text
numpy 2.5.3 / cv2 5.0.0 / torch 2.14.0+cu130 / pyorbbecsdk 2.1.2
```

### 1.3 涉及的功能包

| 包 | 路径 | 职责 |
|---|---|---|
| `h3001_bridge` | `src/h3001_bridge` | CAN ↔ ROS 2 桥接，发布反馈、接收 `/control/*` |
| `bottle_grasp_executor` | `src/bottle_grasp_executor` | 抓取状态机，输出笛卡尔目标点 |
| 感知 demo | `src/wutishibiezitaijiance-master` | YOLO + 点云 → 基座系抓取点 JSON |

感知包的算法细节、参数和 ROS 感知节点用法见
[感知包 README](../wutishibiezitaijiance-master/README.md)，本文不重复。

> 本文里的 `src/...` 路径都是**工作空间根** `/home/qin/zdl_fz` 的相对路径。

---

## 2. 一次性配置

### 2.1 CAN 适配器

每台机器重启后都要做一次（`can0` 不会自动 up）：

```bash
sudo ip link set can0 down 2>/dev/null
sudo ip link set can0 up type can bitrate 1000000
```

确认：

```bash
ip -details link show can0
```

这里的接口名和波特率必须与 `src/h3001_bridge/config/h3001_bridge.yaml` 里的
`can_channel` / `can_bitrate` 一致。

### 2.2 构建工作空间

```bash
cd /home/qin/zdl_fz
conda deactivate
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to h3001_bridge bottle_grasp_executor
```

两个包都是纯 Python 包，`--symlink-install` 之后改 `.py` 不用重新 build，但改
`config/*.yaml` 和 `launch/*.py` 之外的新增文件仍需重新 build。

### 2.3 ROS 环境

每个终端都要先 source：

```bash
source /opt/ros/humble/setup.bash
source /home/qin/zdl_fz/install/setup.bash
```

`h3001_bridge` 不用设 `H3001_SDK_PATH` —— 代码里的候选路径第一顺位就是当前在用的
`src/SDK_linux`（模块名 `sdk_linux`，2026-09-16 起；旧版是 `src/sdk_can/h3001sdk` 的
`h3001_sdk`，仍留在候选表里作回退）。只有在 SDK 换了位置时才需要：

```bash
export H3001_SDK_PATH=/home/qin/zdl_fz/src/SDK_linux
```

该环境变量的优先级**高于**所有硬编码候选路径。注意 `sdk_linux` 与 `h3001_sdk` 在本链路用到的
接口上一致，但 `send_target_pose` / `get_state` 的位姿语义不同，**不要**指望配错路径时会有人报错 ——
换 SDK 后先确认启动日志里解析到的是哪个文件。

---

## 3. 实验运行步骤

四个终端，顺序不能颠倒：先桥接（确认 CAN 通了），再感知（确认抓取点算出来了），
最后执行器。

> **不想开四个终端的话**：工作区根目录有个图形控制台 `h3001_console.py`
> （2026-09-28 加）—— 一个窗口按顺序把这三条起起来、显示每条的就绪与否，还能点
> 「触发抓取」（调 `~/execute_grasp`）。它顺带替你做 §3.0 那一步（清厂商 web server，
> 那东西和桥抢 can0）。用法见它自己的 docstring。**它不替代急停按钮** —— 面板上的
> 「软停」只是不再下发新指令，机械臂会把当前那一条走完。
>
> 控制台上还有个「调参」按钮，打开 `h3001_tuner.py`（2026-09-29 加）—— 抓取参数的
> 图形界面：起始位姿、放置位姿、side/top 抓取姿态、抓取高度与行程、dwell、跟随参数、
> 开关、夹爪力，改完写回 `bottle_grasp_executor/config/bottle_grasp_executor.yaml`。
> **它只改值，注释一个字节都不动**（注释是这个文件的资产），保存前留备份、写完回读校验。
> **它不下发**：改完要重启执行器才生效 —— 所以真正跑之前记得回来点「起/停 执行器」。
> 详见 [`README_参数配置.md`](README_参数配置.md) 开头那张「改参数有两条路」。

### 3.0 前置：停掉厂商 web server

**真机前必须做。** 厂商的 `h3001_web_server_linux.py` 和 `h3001_bridge` 会抢同一条
`can0` 总线，两边都收不全帧：

```bash
pkill -f h3001_web_server_linux.py
```

确认没有残留：

```bash
pgrep -af h3001_web_server_linux.py
```

### 3.1 终端 A —— 桥接（CAN ↔ ROS）

```bash
source /opt/ros/humble/setup.bash
source /home/qin/zdl_fz/install/setup.bash
ros2 launch h3001_bridge h3001_bridge.launch.py
```

可选参数：

- `speed_percent:=5` —— MCU 运动速度百分比。注意 **launch 参数的默认值（现在是 10）
  会覆盖 yaml 里的同名参数**（launch 的参数字典排在 yaml 之后），所以只改 yaml 是没用的；
  想更快/更慢都从这里给。启动行会打一行 `速度=N%`，跑之前先确认它。
  历史上这里的默认值是 `1`，比注释建议的 5~10 低一个数量级 —— 分段下降时每段 1cm
  要走很久，到位判定反而更容易误判成「够不到」而反复重发指令（见 §5.8）；
  现在下降只有一条指令，速度太低会表现为「`descend_dwell` 到点时还没走完」（§5.11）。
- `auto_enable:=false` —— 只读反馈，不使能电机，用于单纯验证 CAN 通信。
- `can_channel:=can0` —— 换接口名。
- `invert_rx_ry:=true` —— 打开固件 rx/ry 取反修正（见 §5.6）。**当前默认是 `false`**，
  即不做取反；只有在确认某台臂需要取反、且同步改了 `side_orientation_xyzw` 之后才打开。

启动后确认反馈在动：

```bash
ros2 topic echo /feedback/tcp_pose --once
ros2 topic echo /feedback/arm_status --once
```

### 3.2 终端 B —— 感知（YOLO → 位姿 JSON）

```bash
cd /home/qin/zdl_fz/src/wutishibiezitaijiance-master
LD_LIBRARY_PATH=/opt/OrbbecSDK_v2.9.3/lib /home/qin/miniforge3/bin/python3 \
  test_yolo_realsense_v3f.py --camera orbbec \
  --json-out /tmp/bottle_grasp_candidates.json \
  --no-point-cloud \--cylinder-axis \
  -tracker ocsort --offset 0 0 0 
```

要点：

- `--hand-eye` 可以省 —— 默认就是 `/home/qin/hand_eye_calibrate/H3003`，与 executor
  配置指向同一次标定。两边必须同源，否则算出的基座系坐标不一致。
- `--offset` 是**基座坐标系**下的固定偏置（米），叠在转换后的抓取点上。上面这组
  是实测调出来的。
- `--no-point-cloud` 关掉 Open3D 点云窗口；无图形界面时加 `--headless`。
- **不要启动 `orbbec_camera` ROS 节点。** 感知 demo 的 `OrbbecCamera` 直接调
  Orbbec SDK 抓 USB 相机，不走 ROS 话题；再起一个 ROS 相机节点只会两个进程抢同一
  个设备。
- 加 `--cylinder-axis` 会拟合瓶半径并把抓取点从可见表面质心移到瓶轴上，Φ60mm 的瓶
  会**横向移动约 19mm**（相机系 Z 方向、朝向瓶心）。上面那条命令默认**没有**开这个
  开关，即抓取点仍在旧点位。要开就加在 `--camera orbbec` 后面：

  ```bash
  LD_LIBRARY_PATH=/opt/OrbbecSDK_v2.9.3/lib /home/qin/miniforge3/bin/python3 \
    test_yolo_realsense_v3f.py --camera orbbec --cylinder-axis \
    --json-out /tmp/bottle_grasp_candidates.json \
    --no-point-cloud --offset 0 0 0
  ```

  开启后终端每帧会多打一个 `r=`（实测应落在 29~30mm，与瓶径 Φ58~60mm 对得上）；
  拟合失败会自动退回表面质心并打警告，不会中断。**点位变了，`--offset` 和手眼标定
  都需要重新核对再上真机**，详见感知包 README 的 5.8 节。

### 3.3 终端 C —— 执行器

先跑 dry_run（默认就是，不下发机械臂命令）：

```bash
source /opt/ros/humble/setup.bash
source /home/qin/zdl_fz/install/setup.bash
ros2 launch bottle_grasp_executor bottle_grasp_executor.launch.py
```

**核对日志里这五行**，确认夹爪点确实落在瓶子上，再上真机：

```text
待机位: 上报原点=[...] ... ← 启动就打印一次; 上报原点必须等于 yaml 里
                             standby_pose_reported_mm_deg 的 X/Y/Z 换成米
                             (默认 0.078 / 0 / 0.29)。对不上就是参数或换算错了
抓取点(夹爪, base)=...     ← 应该就是感知给的瓶身位置
夹爪偏移=...               ← 模长应等于 |gripper_offset_tool_xyz| (本机 0.08 m, 2026-09-21 起)
上报原点=...               ← 真正下发给机械臂的点, = 抓取点 - 夹爪偏移
预抓取(夹爪)=...           ← 抓取点抬高 pre_grasp_offset_z (现在 = 12cm, 2026-09-21 起)
```

这一行末尾会报下降的行程与落点（2026-09-17 起只有这一条路径，见 §5.11）：

```text
... 预抓取(夹爪)=[0.3  0.   0.22] m, 下降 12.0cm 到抓取点(夹爪)=[0.3  0.   0.1 ] m
```

dry_run 下跑完一次任务的尾部应该是这样（成功与失败都是这个形状）：

```text
状态: LIFT -> HOME
回待机位 (抓取任务完成): 一条绝对指令, 夹爪目标 [...] m, 起点 [...], 直线 N cm; 路径由固件规划
[dry_run] move_p 目标: 夹爪=[...], 上报原点=[0.078 0. 0.29], 姿态=[...]
HOME [dry_run] 模拟到位 (1/1)
已回到待机位 (夹爪 [...] m), 任务结束
状态: HOME -> IDLE
```

回位**只有一条指令**（2026-09-21 起不再拆段），所以是 `1/1`；真机上里程超过
`max_command_distance` 时，那一行结尾还会多一句 `(超过单条指令上限 50 cm, 回位段豁免)`
—— 是说明，不是错误（§5.7 第 8 条）。

然后是：

```bash
ros2 launch bottle_grasp_executor bottle_grasp_executor.launch.py dry_run:=false
```

### 3.4 触发

在任意终端：

```bash
ros2 service call /bottle_grasp_executor/execute_grasp std_srvs/srv/Trigger
```

状态机随后自动走 `WAIT_TARGET → PRE_GRASP → TRACK_WAIT`，然后**停下**——停多久取决于
`auto_grasp_stable_time`：现场 yaml 默认 `3.0`，即机械臂到物体上方、物体坐标连续 3 秒
不变就**自己下降**（§5.10）；设成 `0` 则退回「停下等人按 Enter」。

**这个服务同时是「循环模式」的开关（2026-09-21 起）**：调一次 = 启动当前任务**并开启
循环**，任务跑完、回到待机位之后自动开始下一轮（不用人再调）；**再调一次 = 关闭循环**。
关闭的语义是「**跑完当前这一次再停**」—— 那一刻手里可能夹着瓶子、可能正在下降，所以
那一下**不会打断机械臂**，只是不再有下一次自动触发：

| 何时调 | `success` | `message` |
| --- | --- | --- |
| 空闲，循环关着 | `true` | `抓取任务已启动; 循环模式已开启 (回到待机位后等着抓下一个, 视野里一直没有目标也会一直等; 只有任务真失败或再调用一次本服务才关闭)` |
| 循环开着，任务进行中 | `true` | `循环模式已关闭: 本次任务 (<STATE>) 照常跑完并回到待机位, 之后不再自动触发` |
| 循环开着，已经空闲 | `true` | `循环模式已关闭 (当前空闲, 不会再有下一次自动触发)` |
| 循环**已关**，任务还在跑 | `false` | `任务进行中 (<STATE>), 拒绝新请求` |
| 关掉放置（见下）时调 | `true` | `抓取任务已启动 (循环模式**未开启**: place_enabled=false, 原因见日志) ...` |

**回到待机位是「等」，不是「做完」**：跑完一轮、或者**跟丢目标**回到待机位之后，循环就
停在待机位等相机的新坐标 —— 机械臂不动、一条指令都不发，**等多久都不会自动收工**，
来了新目标才自动接下一轮（日志：`循环模式: 已回待机位, 在这里等相机的新目标 ...`，
之后每 10 s 一行 `还在等相机的新目标 ...`）。“视野里暂时没有瓶子”是正常工况，不是故障。

判据是**观察时刻**而不是「手里有没有候选」：只有**回到待机位之后**新收到的候选才算数，
上一轮那个瓶子（已经被放到相机视野之外）的坐标会被丢掉。为什么必须这样，见 §5.14。

循环**会自动停**的只有「**真失败**」这一条路，一次就停、不重试（2026-09-21 那几次固件
IK 拒收都是同一类系统性故障，继续循环只会一遍遍重跑整个流程再失败）：IK 拒收、**反馈
故障**、夹爪失败、回位判不到位…… 所有真失败都汇到同一条 `_fail`，它顺手把循环关掉
（日志里紧跟一行 `循环模式已自动关闭 (这一次任务失败)`）。

**「视野里没有目标」不算真失败**（2026-09-21 改）—— 下列两条走 `_fail(..., keep_loop=True)`，
只回待机位接着等，循环保持开启（日志：`循环模式保持开启: 这一次只是没等到目标 ...`）：

- 起手后 30 s 内没有有效候选（`等待目标超时 (30s 无有效候选)`）—— 这条超时保留，它只
  负责「别在 `WAIT_TARGET` 里干等」，回待机位之后与它无关，**没有新增参数**；
- `TRACK_WAIT` 悬停跟随期间跟丢（`TRACK_TARGET_LOST`：候选超过 `max_result_age`
  没更新，或一个候选都没收到）。

停掉循环的另一条路是人工再调一次服务；回位判不到位那条异常出口也会停（位置本来就不
确定，不能接着抓，见 §5.14）。

循环**开启的前提**是 `place_enabled=true`（唯一的硬闸）：关掉放置时每一轮只是把瓶子
在原地放下再抓起来 —— 同一个瓶子无限循环。这一条不满足时任务照常跑，但**只跑这一次**，
`message` 里会说明原因。另外三项只**警告不拦**，日志里各有对应 WARN：
`grasp_confirm_required` / `place_confirm_required` 开着 = 循环退化成「一次一确认」；
`return_to_standby_on_finish=false` = 循环不在待机位起步。

代码里一共三个确认点，但**后两个各有一个开关，现场 yaml 里默认都关掉了**
（`grasp_confirm_required` / `place_confirm_required`，见 §4）；第一个（`TRACK_WAIT`）
自 2026-09-22 起也能自动（`auto_grasp_stable_time`，见 §5.10），现场 yaml 里默认开着。
所以现场跑起来**整条链没有人工确认**（要退回人工只需把那个参数设成 `0`）：

| 停在哪 | 确认什么 | 确认后 | 关掉/自动时 |
| --- | --- | --- | --- |
| `TRACK_WAIT` | 位置对了，现在下降 | 进 `DESCEND`，一条指令下降 12cm 到抓取点（§5.11） | `auto_grasp_stable_time`（**这一步没有独立的 `*_confirm_required` 开关**，就靠这个参数退回人工）：`0` = 停下等人按 Enter；`>0`（现场 `3.0`）= 机械臂停稳后物体位置连续不变够久就**自动下降**，Enter / `grasp_confirm` 退化成「提前下降」的越权通道 |
| `GRASP_WAIT` | **现在夹** | `gripper_enabled=true`：进 `GRIPPER` 发一条闭合命令，**命令跑完才** `LIFT` 抬升 9cm；`false`：直接 `LIFT`（旧语义「夹爪已经夹住了，现在抬起」，见 §5.13） | `grasp_confirm_required=false`：不等 Enter，`descend_dwell` 一到就做上面同一件事 |
| `PLACE_WAIT` | **现在放** | `gripper_enabled=true`：进 `GRIPPER` 发一条张开命令，**命令跑完才** `HOME`；`false`：直接 `HOME`（旧语义「物体确实放下了」，见 §5.13） | `place_confirm_required=false`：不等 Enter，`place_descend_dwell` 一到就做同一件事 |

> **关掉这两个确认的代价（务必先读）**：`DESCEND` 是**盲降**，只发指令不判是否到位
> （放置段 `PLACE_MOVE` 不盲走，它判到位，见 §5.12）。确认关掉之后，闭合/张开的时刻就
> 完全由 `descend_dwell` / `place_descend_dwell` 决定 —— 机械臂还没走完就夹的话，
> 夹爪会在**半空合上**（夹个空）。真机上出现这种情况，**先调大对应的 dwell**，
> 而不是把开关改回 `true`。另外「手还伸在夹爪里也来得及退开」这句话只在确认打开时成立。
>
> **`TRACK_WAIT` 自动下降后这条链上就没有人了**：落点准不准只剩 `descend_dwell` 的盲等
> 兜底。判据本身（物体坐标连续 3s 不变）无法区分「瓶子自己不动」和「人手扶着它不动」，
> 也区分不了「感知还在发但输出冻住了」；后一类由 `auto_grasp_stable_time` 自带的新鲜度门
> 挡住（上游停发就作废窗口，见 §5.10），前一类只能靠人在场。循环模式开启时会额外打一条
> warn 提示这件事。

> **Enter 的含义在 `gripper_enabled=true` 时变了**：`GRASP_WAIT` / `PLACE_WAIT` 的
> Enter 不再是「我已经手动做完了」，而是「**节点你现在去做**」。所以那两次确认之前
> **不要**自己去动夹爪，也不要在另一个终端跑厂商的 04/05 —— 串口只有一个，会打架。
> 开关关掉时这两处根本不等人，等于节点自己按点接管。

`LIFT` 之后去哪由 `place_enabled` 决定：

- `false`：直接 `HOME` 回待机位 —— 状态序列与加放置功能之前完全一样。回位下发的就是
  `standby_pose_reported_mm_deg` 那 6 个数（**一条**绝对指令，路径交给固件，
  2026-09-21 起，见 §5.7 第 8 条）；**夹爪保持闭合**（张开那一步只存在于
  `PLACE_WAIT`），物体被带回待机位，要放手只能人工打开；
- `true`（**yaml 当前 `true`**）：`PLACE_MOVE` —— **直线直达** `place_pose_reported_mm_deg`
  那个位姿本身（位置 + 姿态一起走，判到位，**没有「正上方」中间点了**，2026-09-18 起）
  → `PLACE_WAIT` 停 `place_descend_dwell` 秒 / 等第三次确认放下 → `HOME`。
  （三次确认的现状：第一次 `TRACK_WAIT` 由 `auto_grasp_stable_time` 自动接管、后两次
  被两个开关关掉 —— 现场跑起来一次都不用按，见上面那张表和 §5.10。）

而夹爪由谁操作，由 `gripper_enabled` 决定（两件事互相独立）：

- `false`：节点完全不碰夹爪，`GRASP_WAIT` 之前你自己在另一个终端跑 04 夹紧，
  `PLACE_WAIT` 之前自己跑 05 张开 —— 注意此时若两个确认开关都关掉，节点会**不等人**
  直接往下走（抬升 / 回位），你得自己算准 dwell 的时间；
- `true`（**当前默认**）：节点接管夹爪，`GRASP_WAIT` 的 Enter 触发闭合、
  `PLACE_WAIT` 的 Enter 触发张开，并且**每轮开始前会先张开一次**（从确定状态出发）。
  每次动作都是「起一个进程、跑一条命令、等它退出」，**判成败只看退出码** ——
  脚本 stdout 上写什么都不影响流程，**唯一的例外是闭合那行 `[holding]`**（见下）。详见 §5.13。
  抬升/搬运期间夹不夹得住由 `gripper_hold_torque` 决定：**0 = 不保持**（闭合后失能
  退出，靠机构自锁，与 2026-09-18~09-28 的行为相同）；**> 0 = 持续保持**（闭合那个
  进程不退出，切 MIT 一直发力到放置时被写 OPEN 松开）。现场值是 1.0。

确认方式二选一（三次共用同一套）：

- 在**终端 C** 里按 Enter（节点会去打开 `/dev/tty`，`ros2 launch` 的管道 stdin
  按键到不了节点时也能生效）；
- 或者发话题：

```bash
ros2 topic pub --once /bottle_grasp_executor/grasp_confirm std_msgs/msg/Bool '{data: true}'
```

**`TRACK_WAIT` 会跟随移动中的目标**：悬停在目标上方 `pre_grasp_offset_z` 处之后，
瓶子被推动或移动时机械臂会跟着挪，直到你按下第一次确认 —— **或者，现场默认根本不
等你按**：机械臂到位后物体位置连续 `auto_grasp_stable_time`（3.0 s）不变就自己下降
（`auto_grasp_stable_time=0` 退回「等你按」，见 §5.10）。跟随的节奏由 `track_*`
一组参数决定（**每 1 s 读一次最新坐标，超过死区 10 mm 才发**，见 §5.10）；把它按住
不动时，机械臂也不会动 —— 死区就是防真机抖动用的。

跟随期间感知停了（候选超过 `max_result_age` 没更新）就**判失败并回待机位**（循环开着时
只是回待机位接着等下一个，循环不关 —— 见上）。若想在这种情况下继续抓，按确认下降：
执行器会打一行警告，并从当前悬停点**原地垂直**下降 —— 不会拿一个可能早就失效的坐标去抓。

确认下降时用的是**目标当时的位置**：瓶子挪到哪就抓到哪。

**回位期间不接受新任务**：`HOME` 是一段真实运动（一条绝对指令，到位判定与 `motion_timeout`
超时照走；超时按旧的段数放大，最多 4 倍，见 §5.9），
这期间再调 `execute_grasp` 会得到：

```text
success=False, message='正在回待机位, 拒绝新请求'
```

**但循环开着时例外**：这一调只用来关闭循环，返回 `success=True`（见上面那张表）。
若回位自己失败（够不到待机位、超时、反馈丢失），会打一行
`回位未完成, 机械臂停在非待机位` 并把状态放回 `IDLE` —— 这时机械臂停在半路，确认
位置后再重新触发即可（不会锁死）；循环在这种情况下也会**自动关掉**（位置本来就不
确定，不能接着抓，见 §5.14）。

单发（不开循环）时，回到 `IDLE` 之后才能调下一次；循环模式下不用管，节点自己接：回到
待机位就在那儿等相机的新坐标，等到了就抓，一直等。

若日志出现 `Enter 确认不可用` 的告警，说明当前没有可打开的终端，改用上面的话题
方式即可。

### 3.5 收尾

三个终端各自 Ctrl-C。真机跑完若不打算继续，建议把电机下使能或断电；`can0` 保持
up 不影响下次运行。

---

## 4. 关键参数速查

> **想改下面这些参数，先看 `h3001_tuner.py`**（工作区根，控制台的「调参」按钮，2026-09-29 加）：
> 位姿、抓取姿态、抓取高度/行程、dwell、跟随参数、开关、夹爪力这些常动的它都有，
> 带实时换算预览（改了 RPY 立刻能看到等效四元数与夹爪点）与跨字段校验，
> 而且**只改值不动注释**、保存前留备份。
> 它不下发，改完要重启执行器。表里其余那些（话题名、IK 阈值、工作空间）它没收，还是手改。

| 参数 | 文件 | 说明 |
|---|---|---|
| `dry_run` | `src/bottle_grasp_executor/config/bottle_grasp_executor.yaml` | 默认 `true`，只打印不下发 |
| `candidates_source` | 同上 | `file` = 读独立 demo 的 JSON；`topic` = 订阅 ROS 感知节点 |
| `candidates_json_path` | 同上 | 默认 `/tmp/bottle_grasp_candidates.json`，必须与终端 B 的 `--json-out` 一致 |
| `hand_eye_yaml_path` | 同上 | 必须与终端 B 的 `--hand-eye` 是同一次标定 |
| `pre_grasp_offset_z` / `grasp_offset_z` | 同上 | 悬停点 / 最终抓取点相对抓取中心的高度。**两者之差就是下降行程**（2026-09-17 起为 9cm，2026-09-21 起 `pre_grasp_offset_z` 抬到 0.12，行程就是 **12cm**） |
| `gripper_offset_tool_xyz` | 同上 | tool 系下「上报原点 → 夹爪」的平移（m），本机实测 `[0.0, 0.0, 0.08]`（模长 8cm；2026-09-21 起，此前 0.06/6cm）。节点内部按夹爪系工作，只在收/发两个边界换算（见 §5.6）。**代码默认值与 yaml 同值** —— 它是一条物理事实，不是部署开关（**注意**：历史记录里有一次 60 mm 的实测，与现在 yaml 的 0.08 不一致，尚未查清 —— 见 `README_参数配置.md` §10 那段） |
| `orientation_source` | 同上 | `fixed` = 用配置里的固定姿态；`current_tcp` = 保持触发瞬间姿态 |
| `side_orientation_xyzw` | 同上 | `fixed` + `side` 模式用的四元数，已从 H3001 实机读出 |
| `track_enabled` | 同上 | 悬停后是否进入 `TRACK_WAIT`（跟随 + 等确认下降）。置 `false` 退回旧的两阶段流程（见 §5.10） |
| `top_skip_track` | 同上 | **只对倒瓶（`top` 模式）生效**：那一模式从正上方抓，机械臂一到瓶子正上方相机就看不着它，跟随必然判跟丢 —— 开着它则悬停到位直接下降，下降点用锁定目标那一刻的坐标。`side` 不受影响（见 §5.10 末尾）。yaml 现场 `true`、代码默认 `false` |
| `track_deadband` / `track_min_command_interval` | 同上 | 跟随的死区（默认 0.010）/ **跟随周期**（yaml 现场调成 1.0s，即「每 1s 读一次坐标」）。两个都逐次读取，可 `ros2 param set` 边跑边调，见 §5.10 |
| `track_settle_time` / `track_stable_epsilon` / `track_max_lag` | 同上 | 跟随的停稳门与它的例外通道（落后上限），见 §5.10 |
| `auto_grasp_stable_time` | 同上 | `TRACK_WAIT` 那次确认是否**也自动化**（2026-09-22）：机械臂已到物体上方、且物体位置连续这么久没变，就自己下降抓取。**yaml 现场 `3.0`，`0` = 关闭退回「等人按 Enter」**。计时**从「机械臂到位」起算**（不是从「物体停下」），否则物体骤停而机械臂还在横移时会连横移带 12cm 下降并成一条指令 → 半空闭合。逐次读取，可 `ros2 param set` 边跑边调，见 §5.10 |
| `auto_grasp_stable_epsilon` | 同上 | 上面那个判据的容差（m，yaml `0.010`）。**别调小**：实测静止瓶子的 `position_base` 峰峰值就有 3~4 mm，拿 `track_stable_epsilon`（0.5 mm）来判**一次都不会触发**。与 `target_stability_tolerance` 同值同义，见 §5.10 |
| `max_result_age` | 同上 | 候选过期阈值（yaml 现场值 6.0s；代码默认值 2026-09-21 起由 5.0s 对齐到 6.0s）：感知停了多久算「跟丢」。**必须大于 `track_min_command_interval`**。它也管 `WAIT_TARGET` 的候选新鲜度，见 §5.10 |
| `place_enabled` | 同上 | 抓住并抬升后是否**先放到** `place_pose_reported_mm_deg` 再回待机位。**yaml 当前 `true`**（2026-09-18：抬升后**直线直达**那个坐标，见 §5.12）；置 `true` 时位姿必须同时填好，否则 `execute_grasp` 会被拒（见 §5.12） |
| `place_pose_reported_mm_deg` | 同上 | 放置**位姿**（位置 + 姿态）`[X,Y,Z(mm), RX,RY,RZ(度)]`，**上报末端原点系** —— 与 `standby_pose_reported_mm_deg` 同一个约定，从厂商 GUI 直接抄。6 个数都要带小数点写成**浮点**，否则 yaml 解析成整数数组、与参数声明的小数数组类型对不上；**只在加载期读一次，改它要重启节点**。`LIFT` 之后机械臂**直线走到这一组位置 + 姿态本身**（2026-09-18 起没有中间点），放置段**不沿用抓取姿态**（见 §5.12） |
| `place_descend_dwell` | 同上 | 抵达放置点（判到位）之后**再停多久**才进放下那一步（默认 0.5s，2026-09-21 起由 6.0s 调下来）。与 `descend_dwell` 分开：那个从「下降指令发出」起算，这个从「判到位」起算。**名字是旧流程（「下降 dwell」）留下的**，含义已随之改成「停稳 dwell」；逐次读取（见 §5.12） |
| `gripper_enabled` | 同上 | 节点是否**接管夹爪**（每轮先张开、`GRASP_WAIT` 的 Enter = 现在夹、`PLACE_WAIT` 的 Enter = 现在放）。**yaml 当前 `true`**；**只在加载期读一次，改它要重启节点**（见 §5.13） |
| `gripper_port` | 同上 | 夹爪的串口 CAN 适配器（默认 `/dev/ttyACM0`）。**与机械臂的 `can0` 不是同一个口**。真机上加载期会查它在不在；`dry_run` 下不查 |
| `gripper_velocity` | 同上 | 开/闭共用的速度（rad/s，默认 2.0）。取自原先人工跑的那两条命令的 `--velocity 2` |
| `gripper_resistance_torque` | 同上 | 阻力阈值（N·m，默认 0.3）：力矩超过「空载 + 这个值」就停下。**它只决定「闭合到哪里停」，不是成功判据** —— 一路合到底（没遇到阻力）也照样往下走，脚本只在 stdout 上说明（见 §5.13） |
| `gripper_hard_torque_limit` | 同上 | 硬上限（N·m，默认 2.0）：碰到就立即停止并判失败。**不要为了「夹得更紧」调大它** —— 抬起期间夹多紧由 `gripper_hold_torque` 决定 |
| `gripper_script_timeout` | 同上 | 传给脚本的 `--timeout`（s，默认 25）。脚本内部一次闭合/张开的墙钟上限 |
| `gripper_timeout` | 同上 | **节点侧**的上限（s，默认 40，2026-09-28 由 30 提高）。**必须大于 `gripper_script_timeout`**（`test_gripper.py` 盯着这条），否则节点会在脚本还没来得及报告原因时就把它判死。40 是照着闭合那一条命令的脚本侧预算定的：反馈等待 0.5 + 空载采样 ≤2 + 闭合 25 + 切 MIT 模式 ≤0.6 ≈ 28.1s |
| `gripper_hold_torque` | 同上 | 闭合之后**是否持续保持力矩**（N·m，**代码默认 0.0 = 关**，yaml 现场值 **1.0**）。0 = 闭合完脚本就失能退出，抬升/搬运靠机构自锁（与 2026-09-18~09-28 逐字相同）；> 0 = 闭合那个进程**不退出**，切 MIT 持续发帧保持这个力矩到放置时被写 `OPEN` 松开。**必须小于 `gripper_hard_torque_limit`**（加载期会拒）。与 `gripper_resistance_torque` 无关：后者只决定「在哪停下」。⚠ 1.0 N·m 折算约 55~100 N 指尖力，可能把瓶子捏变形，见 §5.13 |
| `gripper_hold_timeout` | 同上 | 保持的最长时间（s，默认 300）：**最后兜底**，只在没人叫停也没人管时生效。到点脚本失能放手并打 `[warn]`；机械臂若还悬空物体会掉 |
| `gripper_hold_kd` | 同上 | 保持帧的 MIT 阻尼项（N·m·s/rad，默认 0.5，范围 [0, 5]）。位置刚度 `kp` 固定 0（纯力矩控制），这一版没开位置环 |
| `descend_dwell` | 同上 | 下降指令发出后等多久进 `GRASP_WAIT`（默认 2.0s，2026-09-21 起由 6.0s 调下来）。**不是到位判据**，到点就走；逐次读取，可直接 `ros2 param set`（见 §5.11） |
| `grasp_confirm_required` | 同上 | `GRASP_WAIT` 是否**等人工按 Enter** 才闭合夹爪。**代码默认 `true`，yaml 现场设成 `false`** —— 关掉它之后现场整条链就只剩 `TRACK_WAIT` 那一次确认（2026-09-22 起那一次也由 `auto_grasp_stable_time` 接管了，见 §5.10）。置 `false` 后闭合时刻完全由 `descend_dwell` 决定（盲降不判到位），**夹空了先调大 `descend_dwell` 而不是改回 `true`**（见 §5.11） |
| `place_confirm_required` | 同上 | `PLACE_WAIT` 是否**等人工按 Enter** 才张开夹爪。**代码默认 `true`，yaml 现场设成 `false`**，与上一行对称，对应要调的是 `place_descend_dwell`（见 §5.12） |
| `endpoint_correction_enabled` 及 `max_correction_*` | 同上 | 末端下垂的迭代修正（见 §5.4） |
| `return_to_standby_on_finish` | 同上 | 任务结束/失败后是否回待机位。**yaml 里 `true`，代码默认 `false`**（`ros2 run` 起节点时不会突然多一段运动）。只改这里就够，launch 故意不给这个参数 |
| `standby_pose_reported_mm_deg` | 同上 | 待机位，**固件/GUI 格式** `[X,Y,Z(mm), RX,RY,RZ(度)]`，上报末端原点系。6 个数必须全写成浮点（见 §5.7）。**只在加载期读一次，改它要重启节点** |
| `standby_tolerance` / `standby_settle_time` | 同上 | 回位的到位容差（默认 0.015）/ 停稳时间（0.30）。**回位是「一条绝对待机位」**（2026-09-21 起不拆段，§5.7 第 8 条）：到位就是拿这两个值判的，而到位超时按旧段数放大（最多 4×`motion_timeout`，见 §5.9）。逐次读取，可直接 `ros2 param set` 调试 |
| `return_to_standby_on_error` / `blind_home_dwell` | 同上 | 机械臂报故障 / 反馈断了时**也**回位（默认 `true`，走「盲走」：发一次绝对待机位、不判到位）；盲走等反馈恢复的上限（默认 8s）。见 §5.9 |
| `invert_rx_ry` | `src/h3001_bridge/config/h3001_bridge.yaml` / launch 参数 | 固件 RPY 约定修正，**当前默认 `false`**（见 §5.6）。改默认值必须改 launch，改 yaml 无效 |
| `speed_percent` | launch 参数 / `src/h3001_bridge/config/h3001_bridge.yaml` | 全局速度百分比（`/control/move_p`、`/control/move_l` 与 `/control/move_j` 共用 —— 三条话题同一个值，**改它下降/抬升的速度一起变**）。launch 参数默认 **10**，与 yaml 一致；**launch 的参数字典排在 yaml 之后，会覆盖 yaml**，所以调速度只能从命令行给或改 launch。只在启动时读一次，改它要重启 bridge。注意 `move_l` 的速度字段范围是 **0~120**（`move_p` 走 0~100 那条钳位），含义见 `_resolve_trajectory_step_percent` |
| `auto_enable` | 同上 | 启动时是否使能电机 |

---

## 5. 实测踩坑点

### 5.1 感知必须用「替换形式」设 `LD_LIBRARY_PATH`

`pyorbbecsdk2 2.1.2` 的扩展模块需要 Orbbec SDK 2.9.x 的符号
（`ob_application_config_set_struct`），而系统 ROS 那套是 2.8.6。直接
`python3 test_yolo...` 会 `ImportError`。

必须写 `LD_LIBRARY_PATH=/opt/OrbbecSDK_v2.9.3/lib`（`=` 替换），**不要用
`:=` 前置保留 ROS 路径** —— 那样会拉进系统 `libstdc++.so.6`，conda 的 `cv2` 又要
`CXXABI_1.3.15`，直接崩。替换形式下 numpy / cv2 / torch / pyorbbecsdk 导入全部正常。

### 5.2 不要同时启 Orbbec 的 ROS 相机节点

见 §3.2：两个进程抢同一个 USB 设备。

### 5.3 下降不再有「容差 vs 步长」这回事（2026-09-17）

分段下降时这里记的是「容差必须小于步长」：`descend_step_tolerance` 若接近
`descend_step_z`，「每步等到位」会退化成「每步只等 `settle_time`」，从上一个路点
起步时距离就是 10mm，机械臂只要动 2mm 就满足 8mm 容差，settle 立即启动放行，
两个目标重叠，最终停在目标下方并锁死（`motion_timeout` 30s 也等不到，因为那是
运动学意义上的新稳态）。

**分段下降连同这条约束一起删掉了**（§5.11）：`DESCEND` 现在只有一条指令、不做到位
判定，所以「容差 / 步长 / settle / 停稳 / 限速门」这些参数一个都不剩。`position_tolerance`
仍然管 `PRE_GRASP` / `LIFT` / `HOME`，与下降无关。

### 5.4 末端到不了目标点

E2PROM 重力参数（m/Gx/Gy/Gz）六轴全为 0，MCU 没有重力前馈，末端在重力方向够不到
目标，且自报 `motion_state=0`（已停稳）。偏差可复现，所以用 `endpoint_correction_*`
迭代补偿。根治办法是补齐 E2PROM 重力参数。

**迭代修正作用于所有判到位的运动状态**，即 `PRE_GRASP` / `LIFT` / `HOME` /
`PLACE_MOVE`（§5.12；2026-09-18 之前放置是盲降，那时不在此列）。唯一不看反馈的是
`DESCEND`：它是一条指令（§5.11），所以下降那一 12cm **不做任何修正**，落点比抓取点
低 3~55mm（该姿态下的下垂量）是**预期行为** —— 侧抓瓶子无影响；若将来要求精确落在
抓取点，用 `grasp_offset_z` 补一个正值，而不是把下降改回「等到位」。

### 5.5 `ros2 launch` 会吃掉终端 Enter

节点的 stdin 是管道，按键到不了。节点会主动打开 `/dev/tty` 兜底；实在拿不到时用
`ros2 topic pub` 发 `grasp_confirm`。

现场 yaml 关掉了后两个确认（`grasp_confirm_required` / `place_confirm_required`，
见 §4），**`TRACK_WAIT` 那一次 2026-09-22 起默认也自动了**（`auto_grasp_stable_time`，
见 §5.10），所以按现场配置跑**已经没有任何一步需要 Enter** —— 这条现在只影响
「想手动提前下降」和「把 `auto_grasp_stable_time` 设回 0」这两种情形。
`ros2 launch` 下要手动确认，仍然走 `ros2 topic pub`。

### 5.6 两个坐标约定问题，改在不同的层（2026-09-15）

这台机械臂有两个坐标约定问题，它们是**两件不同的事**，分别落在不同的层：

**(1) `rx`、`ry` 需要取反 —— 改在 bridge**

H3001 固件上报的 RPY 与 ROS/松灵约定不一致，直接按 ZYX 转四元数得到的姿态方向相反。
放在 `h3001_bridge` 里做，**反馈和指令两个方向都要改**：

| 位置 | 改动 |
|---|---|
| `_publish_feedback()` | 读到固件 RPY 后、转四元数前，`rx, ry` 取反 |
| `_pose_to_list()` | 在固件域 `rx, ry` 取反后再发给 MCU |

为什么必须成对：`orientation_source=current_tcp` 会把反馈姿态**原样回发**当指令。
bridge 里 RPY→四元数和四元数→RPY 本来是互逆的，这个回环才精确；只改反馈侧的话，
回发的角就变成 ROS 约定，固件会理解成反向姿态，机械臂转错。

开关是 `invert_rx_ry`，**2026-09-15 起默认 `false`**（不做取反），下游零改动。
当天先开后关，原因是它与 `side_orientation_xyzw` 不自洽：那个四元数是 2026-09-14 从
**未取反**的 bridge 读出的，仍属固件约定；打开取反后会被再取反一次下发，实测使机械臂
偏离记录姿态 **14.37°**（记录 `-2.918/6.569/-170.19` → 下发 `2.918/-6.569/-170.19`）。
关闭后下发的固件角恰好等于记录值，那个四元数不用动。

> 要重新打开取反，必须同时把 executor 的 `side_orientation_xyzw` 换成 x、y 取反后的值
> `[-0.05489, -0.03022, -0.99425, 0.08679]`，否则会重现上面那 14°。

> 另一个容易踩的点：改默认值必须改 **launch** 的 `default_value`，只改 yaml **无效**——
> launch 的参数字典排在 yaml 之后，会覆盖 yaml（已实测：yaml 写 `false` 仍读到 `True`）。
> 和 `speed_percent` 是同一个坑。

> 顺带一个结论：**「rx、ry 取反、rz 不变」精确等价于 `R_ros = Rz(π)·R_fw·Rz(π)ᵀ`**
> （2000 组随机角验证，误差 2.5e-16）。所以这不是凑出来的经验补丁，就是「固件基座系
> 相对 ROS 基座系绕 Z 差了 180°」。

**(2) 夹爪不在上报末端原点上 —— 改在 executor**

上报的末端原点不在夹爪上，离夹爪 8cm（tool 系 `[0.0, 0.0, 0.08]`，2026-09-21 起）。executor 内部**一律用
夹爪系**，换算只发生在两个边界：

```text
收 /feedback/tcp_pose:  夹爪 = 上报 + R @ offset      (_reported_to_gripper)
发 /control/move_p:     上报 = 夹爪 - R @ offset      (_gripper_to_reported)
发 /control/move_l:     同上, 同一个换算, 同一个出口       (2026-09-24 起只有下降走这条)
```

于是路点、到位判定、残差、日志看到的都是夹爪点，可以直接和感知给的抓取点比较，
配置里 `pre_grasp_offset_z` 之类的语义也才和注释一致。

**顺序不能反**：offset 要用旋转矩阵，而旋转矩阵来自姿态。如果取反留在 executor 而
没放到 bridge，`R @ offset` 用的就是错姿态 —— 8cm 的力臂会把姿态误差放大成位置
误差，所以取反必须在 offset 之前、且在唯一的入口处（bridge）做掉。

填零即退化为恒等变换，新机械臂没有这个问题时把 `gripper_offset_tool_xyz` 置零即可，
代码不用删。

### 5.7 待机位为什么存固件 RPY，而不是四元数（2026-09-15）

任务结束（成功或失败）回待机位是 `HOME` 阶段，目标来自
`standby_pose_reported_mm_deg`，格式是**固件/GUI 的那 6 个数**，不是节点内部用的四元数。
理由和注意事项：

1. **来源即格式。** 这 6 个数是从厂商 GUI 上直接读的，抄进 yaml 就行。任何一步手工
   换算（尤其 `2.4, 4.2, -180` 这种角度转四元数）都是抄错的机会。
2. **换算放在加载期做一次**，`夹爪 = 上报 + R @ offset`，与 §5.6 的边界换算同一套。
   因为换算依赖 `gripper_offset_tool_xyz`，而那个偏移是临时方案（§6.4）——存源头格式，
   将来换臂把偏移置零时，待机位不用重算。
3. **姿态不需要换算**：纯平移不改变姿态，夹爪系与上报系共用同一个四元数。
4. **RPY → 四元数走 ZYX**，与 `h3001_bridge` 同式，`invert_rx_ry=false` 时固件 RPY
   可以直接用。反方向（四元数 → RPY）在 `|ry|≈90°` 才退化，存 RPY 没有这个问题。
   **但如果哪天 `invert_rx_ry` 打开，这个位姿必须和 `side_orientation_xyzw` 一起重算**
   （取反不是恒等变换，本姿态实测最大元素差 0.146）。
5. **±180 不要「修」。** 这组 RPY 转成四元数再转回 RPY 得到的是 `(2.4, 4.2, +180)`
   而不是 `-180` —— `Rz(180)` 与 `Rz(-180)` 是同一个旋转，`atan2` 对 ±180 本来就有歧义。
6. **6 个数必须都带小数点**（`78.0` 而不是 `78`）。`rcl_yaml_param_parser` 要求一个序列里
   元素类型一致并按序列推断数组类型，实测（2026-09-15）：
   - 全整数 → `InvalidParameterTypeException: ... of type 'INTEGER_ARRAY', expecting type 'DOUBLE_ARRAY'`
   - 混合（`78.0, 0`）→ `Sequence should be of same type. Value type 'integer' do not belong`

   两种都是**节点起不来**，不是静默出错。
7. **只在加载期读一次**，`ros2 param set` 改了不生效（位置换算、四元数、工作空间校验
   都是一次性的），改它要重启节点。
8. **回位是「一条绝对指令」，不再拆段**（2026-09-21 改）：失败可能发生在工作空间任何位置，
   返程只发**一次**待机位本身（姿态恒为待机姿态），**路径由固件规划**，到没到仍按
   `standby_tolerance` + `standby_settle_time` 判。原来这里是在直线上的插中间点、保证每段
   不超过 `max_command_distance`，删掉它的原因是一个被真机推翻的推理：插出来的中间点
   **都带着待机姿态**，而固件在**固定姿态**下的 IK 可达域**不是凸的** —— 2026-09-21 真机
   回位就死在这样一个中间点上（上报系 `[0.1348, -0.1889, 0.2147]`，固件
   `ik_status=2 (超出工作空间)`），而起点与待机位两个端点各自都是好的。所以那一段没有
   真正的安全网可丢：路点全在直线上，段长之和恒等于总位移，`max_command_distance` 从来
   没限制过回位的总行程（§5.12 有这套推理的完整对照）。上限因此跟着豁免，超限只打一行
   WARN；到位超时按老算式还账（倍数最多 4，见 §5.9）。
9. **回位时夹爪上可能还夹着瓶子**（成功的情况），所以待机位要选一个余量充足的姿态 ——
   现在整段路径由固件规划（第 8 条），「沿途」怎么绕是固件的事，但待机位本身必须是加载期
   校验过、且固件真收得下的姿态（`ik_status` 是按位姿逐个判的，见 §5.12）。

### 5.8 下降为什么改成了「一条指令」（2026-09-16 → 09-17）

**现象（2026-09-16）**：跑到抓取上方后下降阶段**一直抖动**，日志里一堆
`DESCEND 路点 N/9 停稳后仍差 X mm ... 第 N/4 次迭代`，**调整误差很大**。

**病因**：`DESCEND` 把悬停点→抓取点按步长拆成 8 段（共 9 个路点），每段都要「等到位」
才发下一段。而这台机末端在重力方向固定够不到目标（§5.4），偏差随姿态在 3~55mm 间变化。
只要某段的偏差超过容差，那一段就掉进 `_try_endpoint_correction`：

```
停稳判定 → residual = goal - pose → _cmd_offset += residual → 重发 move_p(goal + _cmd_offset)
```

最多重发 4 次。**8~9 个路点、每段最多 4 次「量残差→改指令→重发」叠加起来，就是肉眼看到的
持续抖动**；修正量还会把夹爪**往抓取点下方压**，下降末端就是瓶子和桌面，本身也是撞压风险。

**当时的修法**：加了「停稳即到位」兜底通道 + 限速门 + 末段专用修正上限，让中间段不再触发
迭代修正。台阶是清楚了，但每段仍要等 settle（0.35s）+ 过限速门（0.5s），8~9 段叠加起来
一次下降要十几秒，而且每段都在重新启停 —— 慢和抖只解决了一半。

**2026-09-17 的修法**：把整套分段判据删掉，`DESCEND` 只发**一条**指令（见 §5.11）。
「降到哪一点」不再由节点自己确认 —— 当时的判据是 `GRASP_WAIT` 里人看一眼夹爪再按
Enter。**注意 2026-09-18 起这一步现场默认也不等了**（`grasp_confirm_required=false`），
于是「降到哪一点」在现场配置下**没有任何判据**，只剩 `descend_dwell` 这个定时 ——
所以调它的方向只能是调大，见 §5.11。删掉的参数（`descend_step_z` / `descend_step_tolerance` /
`descend_settle_time` / `descend_settle_judge` / `descend_stable_*` / `descend_min_*` /
`descend_correction_final_only` / `descend_max_correction`）在 yaml 里一个都没留 ——
留着会让人以为还能调回去，而代码里已经没有任何分支读它们。

**速度默认值也顺手修了**：`h3001_bridge` 的 launch 里 `speed_percent` 默认值是 **1**（比注释
自己建议的 5~10 低一个数量级），而 launch 参数会覆盖 yaml 里的 10。1% 下下降 12cm 要走很久，
`descend_dwell` 就得跟着给大。现已改成 **10**（与 yaml 一致），README 里原来写的「默认 5」
也是过时的。

### 5.9 出错也要回位：「不可控就不回位」是个坏判据（2026-09-16）

`return_to_standby_on_finish` 原先有个缺口：`_should_return_to_standby()` 里写着「机械臂报故障 /
反馈断了 → 不回位，反正发了也白发」。真机证明这个理由不成立：

```
[ERROR] 任务失败: 运动超时 (PRE_GRASP, 路点 1/1, 耗时 30.0s, ... 偏差 151.1 mm)
[INFO]  回待机位 (任务失败: ...): 夹爪目标 [0.078 0. 0.29] m, 起点 [...], 分 3 段
[WARN]  HOME 累计修正将达 138.9 mm, 超过上限 80 mm, 停止迭代
[ERROR] 任务失败: TCP_FEEDBACK_STALE: /feedback/tcp_pose 超时或缺失 (状态 HOME)
[ERROR] 回位未完成, 机械臂停在非待机位; 状态回 IDLE, 请人工确认位置后再触发
```

（上面这块是 2026-09-16 的原文，当时回位还拆段，所以有「分 3 段」这样的字样；
现在同一行的前半句是 `回待机位 (…): 一条绝对指令, 夹爪目标 … m, 起点 …`——见 §5.7 第 8 条。
这段日志要记的是「回位途中反馈断了会发生什么」，与拆不拆段无关。）

回位走到一半 `h3001_bridge` 挂了（事后 `ros2 topic info /feedback/tcp_pose` 是 **0 个发布者**），
回位当场放弃，机械臂就留在抓取点上方。可回位是一条**绝对**位姿指令 —— 固件收到就能自己走过去，
不需要反馈。**把机械臂丢在原地，比盲发一条指令危险得多。**

现在的行为：

| 情况 | 行为 |
|---|---|
| 任务失败，机械臂可控 | **一条**绝对待机位（2026-09-21 起不再拆段，路径交给固件）+ 正常到位判定（`standby_tolerance` / `standby_settle_time`，超时按老段数放大） |
| 任务失败，机械臂报故障 / 反馈断了 | **盲走**：不拆段（起点取自反馈，不可信），直接发**一次**绝对待机位，不判到位 |
| 回位途中才变得不可控 | 中途转盲走，重发一次绝对待机位（`_blind_home_fallback`） |
| 盲走期间反馈恢复 | 自动交回正常判据，仍会打出 `已回到待机位` |
| 盲走等满 `blind_home_dwell` 仍没恢复 | 结束回位，并明确报 `**无法确认是否到位**`，不假装成功 |

盲走**只去待机位这一个点**，而它是加载期校验过的安全点，所以既不插值也不用
`max_command_distance`（那个上限防的是「目标本身有问题」，不是防待机位）——
**正常回位现在与盲走是同一条形状**（2026-09-21 起也不再拆段，§5.7 第 8 条），
差别只剩「判不判到位」。

**正常回位的到位超时按旧的段数放大**（同一处改动带来的账）。合并成一条指令之前，拆出来的
N 个路点**每个都各有一个 `motion_timeout`**（默认 30s），对角 1.3 m 的回位就是 4×30s；
合并后只剩一个 30s，远距离回位会被误判成「没收敛」。所以超时按老算式算出的段数给
（`budget = max(max_command_distance - 2×|gripper_offset|, 0.02)`，
`N = ceil(距离 / budget)`），**倍数截断在上限 4** 并打一行 WARN
（`max_command_distance` 被调小时这个算式会给出几十倍，那是把「机械臂整个卡住」伪装成
正常等待，真超了要让人自己判断）。倍数**一离开 HOME 就复位成 1**，不会漏给后面的运动
阶段。代价是「指令发了但机械臂一步没动」这个盲区也跟着变长，所以节点会在「早该起步了」
的时候自己喊一声（`指令已发出 Ns, 机械臂仍未起步`），别干等到超时才去查。

```bash
# 退回旧行为: 不可控就不回位, 直接回 IDLE
ros2 param set /bottle_grasp_executor return_to_standby_on_error false
# 盲走等反馈恢复的上限 (s), 逐次读取
ros2 param set /bottle_grasp_executor blind_home_dwell 8.0
```

**前提**：盲走的前提是「指令还发得出去」。**bridge 死了 → `/control/move_p` 没有订阅者 →
这条指令谁也收不到**，日志只能说明「指令已下发」，不等于「机械臂动了」。所以看到
`盲走回位结束 ... 无法确认是否到位` 时，先查 bridge 还在不在：

```bash
ros2 topic info /feedback/tcp_pose -v   # 发布者应有 h3001_bridge
ros2 topic info /control/move_p -v      # 订阅者应有 h3001_bridge
ros2 topic info /control/move_l -v      # 同上 —— 只有下降走这条 (2026-09-24 起)
```

**回归测试**：`test_return_to_standby.py`。覆盖「故障 / 反馈断也要回位」、
`return_to_standby_on_error=false` 能否退回旧行为、盲走只发一次且不拆段、等满 dwell 后报
「未能确认」而不是「已回到待机位」、反馈恢复后交回正常判据、回位途中转盲走；以及
2026-09-21 合并那一条带来的四件事：「超限的回位也是一条绝对指令（并打一行 WARN）」
「超时预算按老算式还账」「倍数截断在 4」「倍数不漏出 HOME」，外加
「两个拆段 builder 都已删掉」。

```bash
cd ~/zdl_fz && source /opt/ros/humble/setup.bash && source install/setup.bash
cd src/bottle_grasp_executor && /usr/bin/python3 -m unittest -v test_return_to_standby
```

### 5.10 悬停跟随 + 下降前确认（2026-09-16；2026-09-22 起确认可自动化）

**需求**：抓取原本是「先到物体上方，再下降」两阶段，中间没有停顿；物体一旦被移动，
只能重新触发。现在在悬停和下降之间插了一个 `TRACK_WAIT`：

```
PRE_GRASP (悬停到位) → TRACK_WAIT (跟随移动的目标 + 下降前确认) → DESCEND → GRASP_WAIT
```

关掉它就退回旧流程，不用回滚代码：

```bash
ros2 param set /bottle_grasp_executor track_enabled false
```

**跟随是一条伺服回路，不是路点队列**。`TRACK_WAIT` 不走 `_tick_motion()`：它没有路点、
也不等某一条指令到位，而是每 20 Hz（`_tick` 的定时器）看一次最新候选，需要时就把悬停点
重新下发到目标当前位置。三道门依次放行：

| 门 | 参数 | 作用 |
| --- | --- | --- |
| 死区 | `track_deadband` 0.010 | 目标相对**上一条已下发的指令**移动超过 10 mm 才重发 |
| 周期 | `track_min_command_interval` 1.0 | **跟随周期**：每 1 s 最多一条指令，也就是「每 1 s 读一次坐标」 |
| 停稳 | `track_settle_time` 0.30 / `track_stable_epsilon` 0.0005 | 上一条还没走完就先不发 |
| 例外 | `track_max_lag` 0.050 | 落后超过 50 mm 时**无视**停稳门直接重发 |

**这是离散跟随，不是连续伺服**（2026-09-17）。每一跳都把悬停点更新到「目标现在在哪」，
所以不会累积滞后；跳与跳之间机械臂是静止的，看起来像一步一步挪过去。跟随速度的下限由
死区与周期共同决定：10 mm / 1.0 s ≈ 10 mm/s。跟得不够勤就调小周期，出现小抖就调大周期
或死区 —— 两个都是逐次读取的参数，可以 `ros2 param set` 边跑边调：

```bash
ros2 param set /bottle_grasp_executor track_min_command_interval 2.0
ros2 param set /bottle_grasp_executor track_deadband 0.02
```

**过期阈值必须大于跟随周期**：`max_result_age`（6.0 s，2026-09-21 起；此前 5.0 s）配 1.0 s 的周期，等于容忍感知
连漏 4 拍（感知本身约 10 Hz 出结果）。把周期调到接近或超过它，就会出现「还没读到
下一次坐标就先判失败」。这条由 `test_track_wait.py` 的 `ConfigConsistencyTest` 盯着
（它只钉「周期 > 0 且 < `max_result_age`」，不钉具体秒数 —— 周期是现场可调的）。

**死区基准刻意不是「机械臂实际位置」**。这台机末端在重力方向固定够不到目标（§5.4，
偏差 3~55 mm），拿实际位置当基准会变成「永远差着那一截」，于是每个 tick 都重发一次，
真机表现就是抖。同理，`track_max_lag` 量的是「机械臂距**上一条指令**还有多远」，
不是「距新目标多远」—— 后者里含着那个固定的下垂量，会让例外通道恒开，等于把停稳门取消。

**停稳门防的是和 §5.8 里同一个病**：两个目标重叠下发（当年分段下降 wp3 那次抖动）。周期在
1 s 这一档上它仍然长于任何一次几厘米横移的耗时，所以这道门正常情况下每拍都放行，退化成了
「机械臂慢到一个周期还没走完就别叠指令」的兜底。它的基线由 `_follow_target` 开头**逐拍**
更新，不能等三道门都放行才更新 —— 否则每发完一条指令下一拍必然被判「没停稳」，周期会被
吃成两倍（`test_cadence_is_one_command_per_period` 就是钉这个的）。

#### 跟丢、过期、跳变这三种情况分别怎么办

- **目标被别的物体顶掉**：`_select_and_store()` 在锁定 `track_id` 之后只认这一个 id。
  少了这一步，视野里另一只质量更高的瓶子会把候选顶掉，机械臂就跟到别的物体上、
  甚至把夹爪开到那上面去下降。
- **感知停了 / 目标被场景遮住**（2026-09-17 改）：候选带收到时刻，超过 `max_result_age`
  （6 s，2026-09-21 起；此前 5 s）没有更新就**判任务失败并回待机位**（`TRACK_TARGET_LOST`）。
  以前是「原地悬停等人（不判失败）」，但机械臂此时停在一个已经不知道还对不对的位置上，
  继续等没有意义，
  而人也不一定盯着终端。日志里能直接看到：
  `任务失败: TRACK_TARGET_LOST: 候选已 7.0s 没有更新 (阈值 6.0s, 跟随周期 2.0s)`。
  **想在这种情况下还是抓下去**：按 Enter 确认下降（确认在判失败之前处理），执行器会打一行
  `候选已过期 X s, 从当前悬停点原地垂直下降`，横向一动不动 —— 绝不拿过期坐标去抓。
  **这条路只有人能走**：自动下降自带新鲜度门（见下面「这次确认也能自动化」），
  候选过期时它压根不触发，不会替人做主拿失效坐标去抓。
  候选还没到阈值时，2 s 一行的提示里会提前预警「再没有新坐标就判失败」。
- **同一个 `track_id` 的坐标突然跳一大截**（圆柱拟合翻面之类）：由 `max_command_distance`
  拦下（默认 0.5 m 单条指令位移上限），判任务失败并回待机位。瓶子不会那么动。
  注意这道上限**对「目标是 yaml 常量」的那几段不生效**：放置段（2026-09-20）与回位段
  （2026-09-21）—— 放置点与待机位都在 yaml 里、不是感知，距离远是正常的，那两段一条
  `move_p` 直接发出去（§5.12 / §5.7 第 8 条）。豁免只在这一处调用点上按状态给，
  `_check_distance` 本身照旧拦；跟随与抓取段因此仍然靠它挡感知跳变。

#### 这次确认也能自动化（2026-09-22）

现场 yaml 把 `grasp_confirm_required` / `place_confirm_required` 都关掉之后，整条链上
**只剩 `TRACK_WAIT` 这一次人工确认**（§3.4 那张确认点表）。这一步同样可以交给节点：**机械臂到达物体
上方之后，物体坐标连续 `auto_grasp_stable_time`（yaml `3.0`）秒不变，就自动下降抓取**。

```bash
ros2 param set /bottle_grasp_executor auto_grasp_stable_time 3.0   # 开（现场默认）
ros2 param set /bottle_grasp_executor auto_grasp_stable_time 0.0   # 关，退回等人按 Enter
```

逐次读取，**改完当拍生效**，不用重启。`0` 时连计时窗口都不留（残余锚点会被清掉），
与 2026-09-22 之前的行为逐字一致；Enter 与 `grasp_confirm` 话题**在两种模式下都始终有效**，
自动模式开着时它们退化成「不等满 3s，现在就降」的越权通道。

**计时起点是「机械臂到位」而不是「物体停下」**，这条是整个设计的根：

> 物体骤停时机械臂可能还在横移。若从「物体停下」起算，3s 数满会立刻下降，而 `_confirm_descend`
> 把**横移和整段 12cm 下降合成一条指令**（见下面「确认之后」），那 3s 里机械臂可能才走了一半
> 路 —— 指令还在跑，`descend_dwell`（2.0s，且 `grasp_confirm_required=false` 时是唯一的
> 「夹」的判据）就到期了，夹爪会在**半空**合上。把起点挂到到位之后，这个风险不存在，
> 也不需要再加一道独立的安全门。

四道门，缺一不可（判据在 `_track_object_settled` / `_auto_grasp_ready`）：

| 门 | 参数 | 不放行时 |
| --- | --- | --- |
| 功能开关 | `auto_grasp_stable_time` > 0 | 退回人工确认，提示`自动下降已关` |
| **机械臂到位** | `_track_arm_settled`：`track_settle_time` 0.30 / `track_stable_epsilon` 0.0005 | 提示`等机械臂到位并停稳`；**`_tcp_pose` 为 None 也算没过** |
| 候选新鲜 | `track_min_command_interval`（1.0s）内有新坐标 | 窗口作废，提示`候选已 X s 未更新, 窗口作废` |
| 位置不变 | `auto_grasp_stable_epsilon` 0.010 | 重设锚点，提示`物体位置还在变` |

**容差是 10 mm，不能照搬 `track_stable_epsilon`（0.5 mm）**。实测一个静止瓶子的
`position_base`，112 帧 / 13.9 s：标准差 [0.6, 0.7, 0.8] mm，峰峰值 [3.1, 3.1, 4.2] mm。
拿 0.5 mm 量级去判「没变」，滚动 3s 窗口**一次都不会通过**：

| 容差 | 0.5 mm | 2.0 mm | 5 mm | 10 mm |
| --- | --- | --- | --- | --- |
| 3s 窗口通过率 | 0/73 | 0/73 | 73/73 | 73/73 |

10 mm 有约 2 倍裕量，且与 `WAIT_TARGET` 判「目标已稳定」用的 `target_stability_tolerance`
同值同义。**把 `auto_grasp_stable_epsilon` 调到 5 mm 以下，这个功能就会静默失效** ——
现象是「一直悬停、永远不降」，日志里那行`物体位置还在变`会一直刷。

**锚点只在「判为变了」的时候重设，不是每拍都赋值**。写成每拍赋值等于只比较相邻两帧，
每拍都动 1 mm 的**慢速漂移**会因为「两帧之差没超容差」被一路误判成「一直没变」，
计时器永远不会归零。

**为什么自动路径必须自己带一道新鲜度门**：`max_result_age`（yaml 4.0 / 代码默认 6.0）
**大于** `auto_grasp_stable_time`（3.0），拦不住「上游整个不发了」。上游一停，`_latest_candidate`
就冻在最后一个值上、`received_at` 越来越旧，而**一个冻住的值恰好满足「位置没变」**——
3s 后自动下降，抓的是一个没人再看过的坐标。所以这里按「一个跟随周期」判新鲜度，上游停发
超过一个周期就作废窗口。这同时也意味着**自动路径不会走 `_confirm_descend` 的「候选过期 →
原地垂直下降」那条分支**：那条分支是留给人的越权通道（见上一节），自动化去用它就变成
「拿失效坐标的垂直投影去抓」。

**到位门在 `dry_run` 下整道跳过**：干跑可能根本收不到 `/feedback/tcp_pose`（`_tcp_pose`
恒为 None），拿它当门会让这个功能在干跑里永远不触发。这与 `_follow_target` 把停稳门包在
`not dry_run` 里是同一个惯例。

触发时直接调 `_confirm_descend("自动: 物体位置连续不变")`，与 Enter / `grasp_confirm`
**落在同一个终点函数**上，下游 `DESCEND` / `_tick_descend` / `GRASP_WAIT` 零差异；
不一样的是**不查 `_confirm_received` 标志**，所以人按 Enter 永远赢（确认那一拍在
`_follow_target` 之前就被处理并 `return` 了，走不到自动判断）。

日志样貌：悬停期间每 2s 一行提示会带上进度，卡在哪道门一眼能看出来：

```text
悬停跟随中 (目标累计移动 3.2 mm, 候选 0.3s 前更新 (周期 1.0s)); 自动下降: 物体位置已稳定
1.4/3.0s; 按 Enter 确认下降抓取, 或发布 /bottle_grasp_executor/grasp_confirm '{data: true}'
```

计满时那一拍直接打（不会先打一行马上失效的「按 Enter 确认下降抓取」）：

```
自动确认下降 (机械臂已到位, 物体位置连续 3.0s 未变, 阈值 10.0 mm)
收到下降确认 [自动: 物体位置连续不变] (来源: 自动下降): 下降起点(夹爪)=...
```

进 `TRACK_WAIT` 时那行日志也会跟着变（`auto_grasp_stable_time > 0` 时说明已全自动、
Enter 仍可提前）。**开启循环模式**且 `auto_grasp_stable_time > 0` 且
`grasp_confirm_required = false`（后两个确认里，真正决定「链上还有没有人」的是夹爪那一次）
时，启动循环会额外打一条 WARN（`_loop_warnings`）：`自动下降已开 + 夹爪确认已关: 整条链
**全程没有人工确认** —— 目标位置只要稳住就会自己往下抓 …`。

> **落点准不准只剩 `descend_dwell` 的盲等兜底**：这道判据区分不了「瓶子自己不动」和
> 「人手扶着它不动」，人对准的过程本身可能是稳的。要人在场把关，把
> `auto_grasp_stable_time` 设回 `0`。

#### 确认之后

确认时用**目标当时的位置**重算下降起终点（瓶子挪到哪就抓到哪）。目标刚动过而机械臂
还没跟上时，那段横移和 12cm 下降合并成一条绝对位姿指令一次走完（下降只有一条指令，
见 §5.11）。到的还是同一个抓取点（`move_p` 是绝对目标），只是不再先单独对齐一次。

**三个确认点共用一条话题** `/bottle_grasp_executor/grasp_confirm`：在 `TRACK_WAIT` 收到
就确认下降，在 `GRASP_WAIT` 收到就确认「现在夹」，在 `PLACE_WAIT` 收到就确认「现在放」
（§5.12）。每次进入这三个状态都会清掉先前收到的杂散 Enter。
后两个状态现在默认**不等**这个确认（`grasp_confirm_required` /
`place_confirm_required` 现场为 `false`，见 §4），进状态第一拍就直接往下做 ——
此时发到这条话题上的消息不会有任何效果，但也**不会串到下一次 `TRACK_WAIT`**：
`_transition()` 进这三个状态时会把两个标志一起清掉。

**回归测试**：`test_track_wait.py`（跑法同 §5.11）。覆盖进入 `TRACK_WAIT` 的两条路径、
三道门各自的「放行 / 不放行」、**周期真的是一拍一条指令而不是被停稳门吃成两倍**、
锁定 id 之外的目标被忽略、候选过期（含从未收到候选）判失败、
确认时的新鲜/过期两条下降路径、三次确认的按键互不串门，以及运行期间的故障仍会回待机位。
2026-09-22 起还覆盖自动下降（`AutoDescendTest`）：计满自动降、没计满不降、**物体慢速爬行
永不触发（锚点语义，专门钉「别把锚点写成逐拍赋值」）**、目标移动后窗口重开、**机械臂没停稳
就不降**、候选过期不降、Enter / 话题手动越权优先、`auto_grasp_stable_time=0` 退回人工且
清掉残留窗口、每次发跟随指令与进出 `TRACK_WAIT` 都重开窗口、`dry_run` 下没有 TCP 反馈
也能降；`ConfigConsistencyTest` 另加一条钉住 `0.005 <= eps <= track_deadband` 且
`stable_time > 0`。这几条都做过变异验证（去掉到位门 / 逐拍重设锚点 / 去掉新鲜度门
三种改法各自被对应用例抓红），不是写完就绿的摆设。
它同时校验新参数在节点默认值与 `config/*.yaml` 里取值一致 —— 两边不一致会导致
`ros2 run` 起的节点和 `ros2 launch` 起的节点行为不同 —— 并断言
`max_result_age > track_min_command_interval`。

```bash
cd ~/zdl_fz/src/bottle_grasp_executor
/usr/bin/python3 -m unittest -v test_track_wait
```

**已知边界**：`track_id` 来自上游的 IoU 跟踪器（阈值 0.3）。目标移动过快、被完全遮挡或
离开视野后再回来，都可能拿到一个新的 id —— 新 id 不再匹配锁定的目标，候选就此停更，
超过 `max_result_age` 后按「跟丢」判失败并回待机位，需要重新触发一次抓取。
要更长的跟随后续得换跟踪器（如 ByteTrack），不在本节点内。

**倒瓶（`top` 模式）是这个边界里「必然发生」的那一种**（2026-09-29）：那一模式是工具
朝下、从**正上方**抓，机械臂一挪到瓶子正上方，手眼相机就看不着瓶子了 —— 现场表现就是
「到达瓶子上方就识别不到」：候选停更 → 判 `TRACK_TARGET_LOST` → 回待机位，一轮都抓不
成。而 `top` 的悬停高度本来就只有 1 cm（`top_pre_grasp_offset_z`），悬停点与抓取点之间
几乎没有横向差别，跟随在这里没有东西可跟。所以 `top_skip_track: true` 时 **`top` 模式
不进 `TRACK_WAIT`**：悬停到位直接下降（`_skip_track_for_mode`），下降点用**锁定目标那
一刻**的坐标（`WAIT_TARGET` 里过了 1 s 稳定窗口的那一份，而不是「下降前再读一次候选」
—— 那一读正好读在相机被自己挡住的时候）。`side` 从侧面抓、相机看得见瓶子，跟随照旧。
代价与 `track_enabled=false` 相同：`top` 这一轮的落点**无人确认**。

### 5.11 下降：一条指令走完 12cm（2026-09-17；09-21 起行程由 9cm 改为 12cm）

**2026-09-24：这一条指令改走 `move_l`（笛卡尔直线），不再走 `move_p`。** 现场反馈
是「机械臂运动到物体上方后，发 `move_p` 让它下降，路径规划有些不对」—— `MOVE_P` 是
固件**自己的点到点规划**，中间怎么走由它定，于是夹爪不是垂直落下去的。`MOVE_L` 是
笛卡尔直线插补，工具沿一条直线走，这正是下降/抬升要的语义。

- ~~同一个改动也覆盖**抬升（`LIFT`）**~~ —— **2026-09-28 抬升退回 `move_p`，名单只剩下降**：
  ```python
  _LINEAR_MOTION_STATES = (_STATE_DESCEND,)
  ```
  原来加 LIFT 的理由是「夹着瓶子走点到点，瓶子可能斜着扫过去蹭到台面」，但 L 这条路对
  抬升**根本走不通**：09-28 一轮 7 次抓取里 5 次抬升被固件当场拒收 —— 桥接日志里每次都在
  抬升 `move_l` 之后 ~1.8s 出现 `IK 状态变化: status=5`（`5 = 关节单步过大`，码表见
  `h3001_web/protocol_ui.js`），机械臂一步没动，30s 后报运动超时；两次没有这一行的都
  正常抬到位（误差 1.9 / 2.3 mm）。**同一个位置 3 秒前刚用 `move_p` 悬停到过、姿态相同**，
  所以不是够不着，是 L 的逐点插补在抓取点那个「伸得最开」的构型上关节单步超限。
  退回 P 的代价还是原来那条：瓶子可能斜着扫过去蹭到台面。现场若真见到这个现象，**别把
  LIFT 塞回名单**（它换来的是「根本抬不起来」），改用「先一条短 `move_p` 起步、再走 L」
  的两步写法 —— 那正是为了躲开起步那一步的关节单步超限。
- **`PRE_GRASP` / `LIFT` / `HOME` / `PLACE_MOVE` / 跟随悬停都是 `move_p`**（只要求终点，走什么
  形状无所谓），桥上的 `/control/move_p` 与 `/control/move_j` 也一字未动。
- 下发出口还是 `_send_move_p` 一个：它按**当前状态**查 `_motion_is_linear()` 选话题，
  所以加减线性段只改上面那一行，**没有任何调用点写死模式**。被固件拒收后重发用的是
  `_last_command_linear`（那条指令当初的模式），**不是**当时的当前状态 —— 判决是异步
  结案的，结案时状态可能已经推进了。
- **别指望它解决 IK 拒收**：P→L 只改变中间怎么走，**不改终点**。终点本身被固件判
  `ik_status=2` 的话换成 L 照样被拒；而且 L 还**多一种失败模式**：P 只要求终点可解，
  L 要求整条线段上每一点都可解 —— 两端都可解、中间不可解时，P 能走、L 会被拒。
  **2026-09-28 的抬升就是这个形态的实例**（`status=5 关节单步过大`，见上面那段）。
- 入口时序与 P **故意不同**：`move_p` 是「先设模式与速度、再发位姿」（理由见桥里那段
  注释），`move_l` 走的是厂商 SDK 的 `send_linear_pose`，即**先发位姿、再切 L 模式** ——
  也就是厂商 web UI「发送直线目标（MOVE_L）」那条已知可用的路。这个选择没有被真机证实，
  判据是 `/feedback/arm_status` 的 `mode_feedback`：下降过程中应为 **2**（`2=L`，
  码表见 `h3001_web/protocol_ui.js`）。**若仍是 0，说明 L 没生效**，那时把桥的
  `_on_move_l` 换成 mode-first 再测同一目标。

**改动**：`DESCEND` 不再分段，只发**一条**指令从悬停点直接到抓取点，
发出后等 `descend_dwell`（默认 2.0s，2026-09-21 起；此前 6.0s）就进 `GRASP_WAIT` ——
**不判是否到达指定位置、不迭代末端修正、不判停稳、不分段**。分段下降的那套代码和参数已经删掉了（§5.8），
没有任何开关能调回去，要改只能改代码或 `git revert`。

同时把悬停高度抬到 9cm（`pre_grasp_offset_z: 0.08 → 0.09`），所以这一条指令的
Z 行程是 9cm，落点仍是感知给出的抓取中心（`grasp_offset_z` 保持 0）。
**2026-09-21 又把悬停高度从 0.09 抬到 0.12（`pre_grasp_offset_z: 0.09 → 0.12`，代码默认值
向 yaml 现场值对齐），之后现场继续抬到 **0.20**（yaml 现值，代码默认值仍是 0.12）——
**所以现在的 Z 行程是 20cm，上面这些 9cm / 12cm 都是当时的数，别拿它当现在的行程算。**
没变的是它**只有一条**指令，行程恒等于悬停高度。
若想让末端比抓取中心多压 1cm，改的是 `grasp_offset_z: -0.01`，不是 `pre_grasp_offset_z`。

> ⚠ **行程 20cm + `descend_dwell: 2.0` 这个组合没有实测过。** 那条 dwell 是照 9cm
> 定的量（2026-09-21 从 6.0 降下来），行程翻倍时没跟着重测，而 2026-09-24 改走 L 之后
> 轨迹的耗时曲线又变了。第一次真机验证下降时**必须**盯下面那条日志的「实际」栏：
> 闭合之前还在明显移动就是 dwell 给少了，先调大它再说别的。

**为什么**：§5.8 那套分段判据是为了「降到哪一点」而生的，代价是每段都要等停稳
（0.35s）+ 过限速门（0.50s），一段落点还掉进迭代修正，8~9 段叠加起来又慢又抖。
而**降到哪一点并不需要节点自己确认**：`GRASP_WAIT` 本来就要求人看一眼夹爪位置再按
Enter，那才是真正的判据。所以把「走到哪」交给人和下面那条日志，节点只负责把指令
发出去、给足走完的时间。

> **2026-09-18 起：现场默认连那次 Enter 也不等了**（`grasp_confirm_required=false`，
> 见 §4）。于是这条下降在现场配置下**既没有到位判据、也没有人看**，唯一的把关就是
> `descend_dwell` 这个定时。上面「交给人和日志」这句话只在开关打开时成立。
> 判断 dwell 够不够，只能看下面那条日志的「实际」栏在闭合前是否已经不再移动。
> 要恢复人工把关：`ros2 param set /bottle_grasp_executor grasp_confirm_required true`
> （逐次读取，不用重启）。

**代价（必须知道）**：

- 日志不再能证明「机械臂确实降到位了」。唯一证据是这一行里的实际位置：
  ```
  DESCEND 下降指令已发出 2.0s (不判到位): 目标 [0.3 0.  0.1] (夹爪, base), 实际 [...] (距目标 X mm)
  ```
  若「实际」那一栏还在明显移动，说明 `descend_dwell` 给少了，调大它
  （逐次读取，`ros2 param set /bottle_grasp_executor descend_dwell 4.0` —— 比现在的默认 2.0
  大一档，用来验证是不是给少了）。
  **20cm 在 `speed_percent=10` 下的实际耗时到 2026-09-24 仍然没有实测记录**；默认值从 6.0
  降到 2.0 是照 9cm 定的量，不是算出来的，所以「够不够」只能看那行日志的「实际」栏
  —— 闭合前还在明显移动就是给少了。**2026-09-24 起下降走 L（笛卡尔直线），耗时曲线与
  当时的 P 不是一回事，这笔账要重新算。**
- 重力下垂（§5.4，实测 3~55mm）不再被迭代修掉，落点就是「这条指令停在哪算哪」。
  侧抓姿态下实测下垂约 3mm，需要补就在 `grasp_offset_z` 上补。
- 机械臂完全不动（被挡住 / IK 无解）时不会再失败报错 —— 分段下降有
  `descend_min_travel` 挡着这种「空跑完也算成功」，现在没有。
  确认开着时**人眼看一眼即可**；关掉后（现场默认）连这一眼也没有，只有
  上面那条日志的「距目标 X mm」能事后说明问题 —— 距目标还差一大截就闭合，
  结果就是夹空。

节点启动行会自报这条路径，跑之前先看一眼：

```text
下降: 一条指令走完 (直接下降到抓取点, Z 下降 12.0cm, 发出后等 2.0s 即自动闭合夹爪 (无需确认),
      **不判是否到位、不迭代末端修正**); 落点由 TRACK_WAIT 的自动下降兜底 (全程没有人看过就往下抓)
```

（这两句随确认开关变：`grasp_confirm_required=true` 时是「即进夹爪确认, **不判是否到位…**);
落点由 GRASP_WAIT 的人工确认兜底」；`grasp_confirm_required=false` 且
`auto_grasp_stable_time=0` 时落点那半句退回「落点由 TRACK_WAIT 那一次人工确认兜底」；
`track_enabled=false`（没有 `TRACK_WAIT` 可言）时是「落点无人确认」——见 §5.10。）

**回归测试**：`test_descend_judge.py`，覆盖「整段只有一条指令」「dwell 才放行」
「纹丝不动也照走且不产生修正量」「下垂不补偿」「不受 motion_timeout / position_tolerance
影响」「反馈不可信时不下发」「分段下降的辅助函数已删除」。
跑法：

```bash
cd ~/zdl_fz && source /opt/ros/humble/setup.bash && source install/setup.bash
cd src/bottle_grasp_executor && /usr/bin/python3 -m unittest -v test_descend_judge
```

### 5.12 放置：直线直达放置位姿（判到位）+ 停稳后第三次确认（2026-09-18）

> **2026-09-18 改动：`PLACE_MOVE` 改成「直线直达放置位姿本身」。**
>
> 原来这里是两段：`PLACE_MOVE` 先**水平**平移到放置点正上方（保持当前实际高度），
> 再 `PLACE_DESCEND` **盲目垂直下降**到放置点。现在只剩一段 —— **直接发
> `place_pose_reported_mm_deg` 那一组位置 + 姿态，让机械臂走到它**，判到位，然后放下。
> `PLACE_DESCEND` 这个状态已从代码里删掉（常量、tick 处理、`_advance_after_motion`
> 的分支、`_should_return_to_standby` 白名单里都没有了）。
>
> 为什么这样是对的：用户的要求本来就是「**到达指定位置就释放**」，释放点不一定在台面上
> （§5.12 下面那条「放置点比当前夹爪还高」正是为这种情况写的）。旧的盲降段隐含了
> 「放置点一定在台面、下降一定被台面挡住」这个假设，一旦释放点在空中的高处，盲降就会
> 按 `place_descend_dwell` 定时往下冲过头。直达 + 判到位之后，「走到哪」由反馈说了算，
> 不再依赖那个假设。
>
> 代价：既然判到位，这一段就会走 `_tick_motion` 的到位判据与**迭代末端修正**（§5.4）——
> 包括那条「先看见机械臂动过才承认停稳」的哨兵（见下面「换放置姿态时怎么看对不对」）。
> 释放点若真落在台面上，机械臂压住之后残差会一直在，它会按 `max_correction_retries`
> 的次数反复「量残差 → 改指令 → 重发」，直到累计修正量顶到 `max_correction_distance`
> （8 cm）或次数用完为止 —— 拆段删掉之后，能封住它的只剩这两个上限，比原来的盲降慢一点。

需求（`place_enabled=true` 时）：抓住物体抬升之后，**先把它送到指定位置释放，再回工作位**，
而不是直接带回待机位。

```text
GRASP_WAIT ──夹──> LIFT ──> PLACE_MOVE ──> PLACE_WAIT ──放──> HOME
                            (直线直达      (第三次,默认不等) (现存待机位)
                             放置位姿,判到位)
```

`PLACE_WAIT` 那一步的「放」默认**不等人工确认**（`place_confirm_required=false`，
2026-09-18 起，见 §4）：进状态后停满 `place_descend_dwell` 秒就张开夹爪，张开成功后
自动回待机位，整条链不再在这里停。开关打开时才是原行为 —— 停下来等人看一眼、
按 Enter 再放。

`place_enabled: false` 时这一整段跳过，`LIFT` 之后直接 `HOME` —— 状态序列与加这段之前
一字不差（回位本身 2026-09-21 改成了「一条绝对指令交给固件规划」，见 §5.7 第 8 条）。

**填这组坐标之前必须知道的一件事：`ik_status` 是逐次判决的。** 机械臂够不够得到某个
位姿，由固件在收到那条指令时判，不由节点算 —— `h3001_bridge` 日志里
`IK 状态变化: status=2` 是**拒收**（码表见下面那张表），`status=0` 才是收下。同一个位姿在
真机上出现过一次收下、一次拒收（与当时的关节构型有关），所以**别去「算」一个坐标，要用手把
机械臂挪到释放点、抄 GUI 上那 6 个数**。历史上那两组坐标（`[130.3, -100.0, 194.1, 84.4, 0.2, 90.0]` 与
`[222.0, -290.0, 258.1, 84.4, 0.2, 30.0]`）都是这么被判掉的：`PLACE_MOVE` 原地不动、
跑满 `motion_timeout` 报 `运动超时`，而待机位 `[100.0, 0.0, 290.0, 84.4, 0.2, 90.0]`
一直是好的。

**`status=2` 不能单独当成「刚发的这条被拒了」**（2026-09-21 修）：固件的 `0x5A9` 是
~50Hz 的**周期性心跳**，上面带的始终是「最近一次判决」这个**粘住不散**的值 —— 实测一个
目标被拒收之后两分钟，它还在报 2。所以节点判「拒收」要两条独立证据同时成立：
**固件说非 0，且从指令发出到现在机械臂一步没动**。少任何一条都不判（只看前者会拿旧判决
误伤新指令；只看后者会把「被挡住推不动」抢过来）。

固件码表（`ik_status` 非 0 时的含义，抄自 SDK 的厂商网页 `protocol_ui.js`）：

| 码 | 含义 | 现场先动什么 |
| --- | --- | --- |
| `1` | 参数无效 | 那 6 个数本身有问题（单位/NaN），看是不是按米填了 |
| `2` | 超出工作空间 | **这个码不等于「位置出了 `workspace_*` 盒子」** —— 盒子在界内也照样可能报它（固定姿态下固件 IK 的可达域不是凸的，§5.7 第 8 条）。先问「这个位姿本身在这个朝向下可达吗」：位置：`check_place_pose.py` 换一组 X/Y/Z（先看 §5.12 的工作空间那段）；姿态：换 RX/RY/RZ |
| `3` | 超过关节限位 | 姿态优先：改 RX/RY/RZ，位置其次 |
| `4` | 轨迹缓冲区不足 | 上一条还没跑完就来了新的；`motion_timeout`/重发节奏的问题 |
| `5` | 关节单步过大 | 一步跨太远 —— 旧实现靠 `max_command_distance` 自动拆成两段，**拆段 2026-09-21 已删**（§5.12），节点不再自己缩步 |
| `6` | 奇异姿态 | 姿态（手腕转到奇异点了），改 RX/RY/RZ |
| `7` | 逆解校验失败 | 一般跟着 2/3/6 一起出现，按最前面那个非 0 的原因处理 |

码表里没有的码**照样算拒收**（固件可能比这张表新）—— 判据是「非 0」，这张表只负责把
原因译成人话。


**yaml 里当前那组 `[214.0, -306.0, 154.1, 95.4, 0.2, 26.0]` 2026-09-21 10:17 也被判过
`status=2`**：桥接日志里 MCU 当场拒收，`PLACE_MOVE` 一步没走、跑满超时报 413mm 偏差，
紧接着 `HOME` 也失败（`回位未完成`）。位置够不够得到**不要在这里推断** —— IK 是固件判
的，URDF 和脑算都算不出来；能看的就是「从待机位过来要同时转多少度手腕」，那行日志
写着 64.9°，是当时的头号怀疑对象。

**要换一组数，别再用整个任务去试**（抓一瓶 + 抬起来 + 赌一次，一次几分钟）。只发放置
那一条指令的脚本：

```bash
cd <bottle_grasp_executor 包目录>          # scripts/ 就在这一层下面
/usr/bin/python3 scripts/check_place_pose.py --dry-run      # 只换算并打印, 一条都不发
/usr/bin/python3 scripts/check_place_pose.py                # 拿 yaml 里那组试一次
/usr/bin/python3 scripts/check_place_pose.py 214.0 -306.0 190.0 84.4 0.2 30.0
```

它不抓瓶子、不起执行节点、不碰感知，直接订阅 `/feedback/arm_status` 把固件的判决念出来
（`固件 IK 拒收 (ik_status=2, ...)` 或者「固件没有拒收」），再看这 1 秒实际移动了多少、
停在离目标多远。**判 IK 收没收下本来就该看它**，不用去翻桥接日志。发了指令机械臂就会
真的动 —— 它会先打印要发的那 6 个数并要求按 Enter 确认（`--yes` 跳过）。

**两个状态的判据**：

| 状态 | 动作 | 判到位？ | 为什么 |
| --- | --- | --- | --- |
| `PLACE_MOVE` | **直线**走到 `place_pose_reported_mm_deg`（位置 + 姿态一起，走完就停） | **是** | 自由空间运动，下垂必须靠迭代末端修正补（§5.4）。判到位之后「到没到」由反馈说了算，不依赖「释放点在台面上」这个假设 |
| `PLACE_WAIT` | 停满 `place_descend_dwell` 再放（默认不等确认，见上） | —— | 判到位只说明「在容差里」，不说明「停稳了」。这一停是为了别在还在晃的时候松手；与 `GRASP_WAIT` 同一个套路，只是那边等的是人、这边等的是时间 |

**三个刻意的选择**：

- **位置和姿态一起走**：整段只有**一个路点** —— 放置位姿本身（原先「按直线在夹爪系里
  插值」的拆段已经删掉，连同 `_build_segmented_waypoints` 一起，见下面
  「一条 `move_p`，不拆段」），姿态就是参数里那组 RX/RY/RZ —— 进 `PLACE_MOVE` 时那行日志
  末尾的「全程同时转腕 N°」就是这个总转角。
- **放置姿态整个来自参数**（`place_pose_reported_mm_deg` 的 RX/RY/RZ，加载期算好）。
  2026-09-18 之前这里冻结的是**进 `PLACE_MOVE` 时的实际 TCP 姿态** ——
  于是放置姿态被抓取姿态绑死，抓取用的那个腕部朝向在放置点未必解得出 IK（真机上就出现过
  `PLACE_MOVE` 原地不动、最后 `运动超时`）。改成参数之后位置和姿态都是用户指定的完整
  位姿，与抓取段彻底解耦；顺带也不会再把腕部的重力下垂读成「新的姿态」。
- **一条 `move_p`，不拆段**（2026-09-20 改，用户要求「运动规划全靠机械臂本身」）。
  这一段**不再**按 `max_command_distance` 在直线上插中间点：路点队列就是一个——
  放置位姿本身——路径怎么走全交给固件规划。为此 `_tick_motion` 对 `PLACE_MOVE`
  关掉了 `_check_distance` 的距离上限（`enforce_max=False`）。

  为什么可以豁免：0.50 m 那个上限**防的是感知跳变**（同一个 `track_id` 的坐标突然
  跳一大截，§5.10），而放置点是 yaml 里的**常量、不是感知**，距离远是正常的 —— 本机
  这组放置坐标离抓取区域就有半米量级，正好是旧实现会拆段的那种。豁免**只给「目标是
  yaml 常量」的段**（2026-09-20 给放置段、2026-09-21 给回位段，见 §5.7 第 8 条）：
  「无 TCP 反馈就拒绝发送」照旧生效，跟随 / 抓取（`PRE_GRASP` / `LIFT`）段的超限照旧
  判失败（`test_the_limit_is_waived_for_configured_goals_only` 盯着这条）。

  工作空间检查仍然在**发指令之前**做，而且不拆段也只需查终点（起终点之间的点由固件
  规划，节点管不到）。但**别拿「盒子是凸的」当成「不拆段也安全」的理由** —— 这是
  2026-09-21 把回位拆段删掉的直接原因：`workspace_*` 确实是个凸盒子，可它只证明
  **节点自己那道盒子检查**在起点与终点都合法时会对直线上的每个中间点放行，对
  **固件 IK 的可达域**什么也说明不了 —— 固定姿态下的可达域根本不是凸的。回位当年正是
  照「凸盒子 ⇒ 中间点安全」这个推理在直线上插点的，结果死在第一个中间点上
  （`ik_status=2 超出工作空间`，两个端点各自都够得到，§5.7 第 8 条有完整证据）。
  所以现在的分工是：**路径中间怎么走交给固件**，节点只保证「发出去的那个目标点」在盒子
  里、在加载期被校验过；至于固件收不收得下，只有固件逐次判决说了算（见上面 `ik_status`
  那一段），节点算不出来。

**坐标约定**（最容易填错的地方）：

| 参数 | 坐标系 | 单位 | 含义 |
| --- | --- | --- | --- |
| `place_pose_reported_mm_deg` | 上报末端原点系 | **毫米 + 度** | **位置 + 姿态**（6 个数） |
| `standby_pose_reported_mm_deg` | 同左 | **毫米 + 度** | **位置 + 姿态**（6 个数） |

**两个参数现在是同一个约定**，都是从厂商 GUI 直接抄那 6 个数、不用自己换算。节点在加载期
一次性换算成夹爪系（位置 + 四元数），换算只发生在这里和 `/feedback/tcp_pose`、
`/control/move_p` 两个边界（见 §6.4）。

> 2026-09-18 之前 `place_pose_*` 是 **3 个数、夹爪中心、`base_link` 系、米** —— 与待机位
> 不是一回事，从 GUI 读到的值必须减掉 `R @ gripper_offset_tool_xyz`（本机 0.08 m，2026-09-21 起）。
> 那个约定已废弃；旧值 `[0.190, -0.100, 0.200]` 换算过来是
> `[130.3, -100.0, 194.1, 84.4, 0.2, 90.0]`（姿态顺带换成待机位那组 RPY）。
> 差这一截的旧症状是机械臂停在瓶子旁边 6~8 cm 处。

**填错位姿会在两个时机被拦下**：

1. **加载期**（节点启动时，不 raise）：`place_enabled=true` 而 `place_pose_reported_mm_deg`
   不是 6 个有限数、**上报原点**离基座原点不到 5 cm（默认占位值全零、或位置按米填都是
   这种）、或夹爪点/上报原点任一超出 `workspace_*` —— 打一行 error 并禁用放置功能；
   判据用上报原点而不是换算后的夹爪点：后者 = 上报 + `R @ offset`，即使上报归零也还有
   0.08 m 的偏移，永远够不到那个阈值。
2. **触发时**：上面任一情况都会让 `execute_grasp` **被拒绝**：

```text
success=False, message='place_enabled=true 但 place_pose_reported_mm_deg 无效
                        (见启动日志), 拒绝启动; 修好位姿或把 place_enabled 置 false'
```

刻意不在抓到一半才失败 —— 那时手里已经捏着瓶子，处置起来麻烦得多。

**运行期还会再查一次工作空间**：加载期只能查夹爪点，而 `_check_workspace_pair` 要同时查
夹爪点与它对应的上报原点（两者相距 `|gripper_offset|`），且 `workspace_*` 可以被
`ros2 param set` 改窄。任一越界就打 `WORKSPACE_REJECTED` 并走失败处置（回位）——
**进 `PLACE_MOVE` 之前**，也就是机械臂还没往那边动。

**放置点比抬升点还高**时只打一行 warn（不失败）：这一段成了「上升」。**不是错** —— 释放点
完全由 `place_pose_reported_mm_deg` 决定，机械臂走到那一点松手，那里不一定在台面上（往货架/
高台上放、或者只是要求「送到这个位置释放」都算）。这行 warn 存在的唯一理由是另一个可能：
从 GUI 抄 z 时抄错一位，于是「本该往下放」悄悄变成往上抬 —— 只有那种情况下才去改参数。
**现在这一段的日志措辞是「直线 N cm」，不再有「下降」字样**，因为它可能是在上升。

**`PLACE_WAIT` 的提示里带实际位置与残差**，但那只是给人看的，**不参与判断**
（放置段是判到位的，走到没走到看上面 `PLACE_MOVE 到位` 那行）：

```text
已到放置点 (第三次确认, 位置与姿态已判到位): 夹爪目标 [0.4 -0.3 0.2] m,
      实际 [0.4 -0.3 0.215] (距目标 15.0 mm); 请张开放下物体, 然后在本终端按 Enter
      确认已放置 (确认后回待机位), 或发布 .../grasp_confirm '{data: true}'
```

`place_confirm_required=false` 时同一行变成
`已到放置点并停稳 0.5s (place_confirm_required=false, 不等人工确认): 张开夹爪`（这行的
秒数就是 `place_descend_dwell`，2026-09-21 起默认 0.5s），**残差那一段就不打了** ——
这一行也就是关掉确认之后唯一还能看出「走没走到」的地方，
所以它留着；只是没人会停下来看。

残差长期不为零是正常的（就是那个下垂量）。要判断「机械臂到底走到哪了」看这一行；若显示
还在明显移动，把 `place_descend_dwell` 调大。

**换放置姿态时怎么看对不对**（2026-09-18 新增两条日志）：

- 进 `PLACE_MOVE` 时那行末尾带 **`全程同时转腕 N°`** —— 抬升姿态到放置姿态要转多少。
  超过 90° 会另打一条 `WARN`（多半是 RX/RY/RZ 抄错了一行）。取不到当前 TCP 姿态时打
  `未知 (无当前姿态)`，不参与判断。直线距离超过 `max_command_distance` 时，同一行末尾
  会注明 `(超过单条指令上限 0.50 m, 放置段豁免)` —— 是说明，不是错误。
- `PLACE_MOVE` **原地不动**、最后 `运动超时`：优先怀疑这组姿态机械臂解不出来（IK 无解 /
  关节限位）。**换一组 RPY 再试**，不要去动 `motion_timeout` 或容差 —— 判断「动没动」
  看超时那行的实际位置：偏差 ≈ 整段步长 = 没动过；偏差明显小于步长 = 动过但推不动。
- 顺带：`_try_endpoint_correction` 现在要求**先看见机械臂动过**才承认「停稳」。没有这一下，
  指令刚发出、机械臂还没起步的那一拍会被当成「停稳但够不到」，而站在起点上算出来的残差
  就是整段位移 —— 指令当场被加成两倍距离（2026-09-18 真机日志里 24.4 cm 的横移被算成
  244 mm 修正量，那次靠 `max_correction_distance` 挡住了，短于 8 cm 的横移挡不住）。

**首次上真机的建议顺序**：先 `place_enabled=false` 跑一遍确认老流程没变；再把机械臂
**用手挪到释放点、抄 GUI 上那 6 个数**填进 `place_pose_reported_mm_deg`（别去算 ——
`ik_status` 是逐次判决的，见上面那条），开着 `place_confirm_required` 跑一遍，确认日志里
`移到放置点 (一条 move_p, 无中间点)` 那行的距离与「全程同时转腕 N°」都合理、机械臂真的
停在那个位姿上；位置定下来再关掉确认。

**回归测试**：`test_place.py`，覆盖「关掉时直接回位」「直线直达放置位姿本身（位移在
那一条的目标就是放置点）」「再远也是一条 move_p（不拆段）」「距离上限的豁免只在调用点
按状态给（放置段 / 回位段这类常量目标），`_check_distance` 本身照拦」
「放置姿态来自参数、腕部倾斜也不跟」「判到位才进
`PLACE_WAIT`」「到位前不许发放下命令」「dwell 才放行」「进 `PLACE_WAIT` 时旧确认被清掉」
「确认后回待机位」「待机位不可用也不崩」「越界 → 失败 → 回位」「加载期校验」（含占位值
被拒、按米填被拒、3 个数的旧写法被拒）「上报系 → 夹爪系的换算与 6 个数一致」与参数一致性；
`PlaceArrivalTest` 专门盯「放置段判到位而不是到点就走」；`PlaceWaitAutoConfirmTest` 盯
`place_confirm_required=false` 时「停满 dwell 就走到 IDLE / 走到 HOME」这条路。
`PlaceMoveCorrectionTest` 盯上面那条停稳检测的哨兵：「没起步」不许读成「停稳」、
动过之后迭代照旧、重发时不许把哨兵清零（用 3 cm 短位移复现 —— `max_correction_distance`
挡不住的那种；那组坐标是**纯水平** 3 cm，因为修正是累加的，3 次之后不能顶到 8 cm 上限）。
跑法：

```bash
cd ~/zdl_fz/src/bottle_grasp_executor && /usr/bin/python3 -m unittest -v test_place
```

> **跑测试之前先把 `h3001_bridge` 停掉（2026-09-18 16:49 真机事故）**：这几个用例都是拿
> `dry_run=False` 跑的（要验的正是真机那条运动逻辑），而 `_send_move_p` 在 `dry_run=False`
> 分支里会**真的 publish** 到 `/control/move_p`。同一个 DDS 域里只要有活着的 `h3001_bridge`，
> 跑一遍 `test_place` 就等于**把机械臂真的开动一遍**：日志里那串 `[400,-300,450]` /
> `[400,-300,200]` / `[400,-300,420]` 是放置用例的指令，`[500,-300,450]` 是
> `test_long_horizontal_leg_is_split` 拆出来的第一段，最后那几条
> `[78.0, 0.0, 290.0, 2.4, 4.2, -180.0]` 是回位 —— 用的是**代码默认待机位**，与 yaml 现场那组
> `[100.0, 0.0, 290.0, 84.4, 0.2, 90.0]` 差着一整个腕部朝向，所以现场看到的是「什么都没干，
> 机械臂突然转过去」。
>
> 现在六个测试模块（`test_place` / `test_gripper` / `test_descend_judge` / `test_track_wait` /
> `test_return_to_standby` / `test_auto_repeat`）的 `setUp` 都装了 `_NullPublisher`，
> `_move_p_pub` 与 `_move_l_pub` 都被换成只记录、
> 不发布的桩，所以**照旧可以随便跑**（`dry_run=False` 的用例仍然验得到发送路径上的副作用，
> 只是消息出不了本进程）。**新加文件时记得照抄这两行**（两条话题都要桩 ——
> 下降走 `move_l`，只桩掉 `move_p` 的话，跑 `test_descend_judge` 就会把下降指令
> 真发到桥上），否则同样的事故会再来一次。
> `test_auto_repeat` 还多一层：它测的循环模式**自己**就会发指令（接下一轮），所以那里
> `_NullPublisher` 不是「防一手」而是唯一一道闸。
>
> `MoveModeRoutingTest`（同在 `test_place.py`）钉的就是「哪一段走哪条话题」这条契约：
> 名单恰好是 `DESCEND`（2026-09-28 起 `LIFT` 已退回 `move_p`）、每个状态都路由到预期
> 的话题、被拒后重发**保持原模式**。
>
> 要临时确认「到底有没有发出去」，可以另起一个 `/control/move_p` 订阅者数一下 —— 跑完整套
> 应当是 **0 条**。

#### 2026-09-30：倒瓶抓完之后那次大摆 —— 把回正折进抬升

**现象**（`~/.ros/log/python3_44942_*.log`，抓一个倒着的瓶子）：

```text
夹爪已闭合, 直接去放置 (top 模式, 跳过抬升)
移到放置点 (一条 move_p, 无中间点): 起点(夹爪)=[0.2418 -0.0288 0.0677] ->
        放置点(夹爪)=[0.2165 -0.2825 0.3016] m, 直线 34.6 cm, 全程同时转腕 144.4°
[WARN] 放置姿态与当前 TCP 姿态差 144.4° ... 确认 place_pose_reported_mm_deg 的 RX/RY/RZ 是否抄错
[WARN] 这一段走 move_j 不成 (没收敛 (残差 3.19mm/16.47°)) —— 回落 move_p
[WARN] PLACE_MOVE 累计修正将达 529.1 mm, 超过上限 80 mm, 停止迭代
[ERROR] 任务失败: ROBOT_FAULT err_status=512
        -> 盲走回位, 反馈 8.0s 未恢复, 机械臂可能仍在运动
```

**根因**（三条叠在一起，缺一条都不会出事）：

1. 倒瓶走的是 `top` 模式，抓取姿态来自**感知 live**（`_uses_live_orientation` 只认 top），
   工具 Z 竖直朝下（实测 `rx≈179.2°`）—— 此时 **J6 已经 172.9°，离 CAD 上限 ±180° 只剩
   7.1°**；而放置姿态的工具轴接近水平。两者夹角 144.4°，**J6 非转 146° 不可**。
2. `top_skip_lift: true`（09-28 用户要求）让它**不抬升**，放置段的起点是**离台面 6.7cm
   的抓取点** —— 夹着瓶子贴在台面高度起步。
3. 于是这一条指令要同时走 34.6cm 位移 + 146° 转腕。`_solve_joint_target` 解不出来
   （残差 3.19mm/16.47°）→ 回落 `move_p` → **由固件逐点解逆解规划这次大摆** → 撞电机错误。

**离线复现**（只算不发，直接调 `ik/L4002-0100.urdf` + `ik/L4002-0100-cad-limits.csv`，
`restarts=0` 与节点一致）：对照解 `[-6.8 -83.1 47. -0.6 -51.4 172.9]` 与日志逐位相同；
现流程的残差 `3.19mm/16.47°` 与日志**完全一致** —— 说明模型可信，可以在上面试着改。

**改法：抬升时顺带回正姿态**（`lift_align_threshold_deg` / `lift_align_segments`）。
抬升那一条指令的目标姿态从「当前 TCP 姿态」改成 **side 抓取姿态**（`side_orientation_xyzw`），
于是到达放置段时手臂已经在 side 姿态上。判据是**夹角**：当前姿态与 side 姿态差不超过
`lift_align_threshold_deg`（默认 90°，与上面那条 warn 同源）就不回正 —— side 模式本来就
取自那个参数、差值≈0，**那一段逐字不变**。

回正**按关节直线拆成 N 段**下发（`lift_align_segments`，默认 3）。拆是必须的：目标构型与
当前构型的单轴差有 146°，一条发出去会被守门 ③ 判「换构型、会被原样执行成大摆」拒掉 ——
而那正是这次事故的来路。抽查过的数（现场那一组）：

| 段 | 单轴差 | CAD 余量 | 夹爪点高度 | 侧向漂移 |
|---|---|---|---|---|
| 1 | 49.9° | 33.8° | 9.5cm | 41mm |
| 2 | 49.9° | 20.5° | 12.7cm | 43mm |
| 3 | 49.9° | 7.3° | 16.7cm | 0mm |

每段都在**完整的三道守门**下解出来（60° / 5°），高度单调上升（起点 6.7cm），最后一段由
FK 复核落在「抬升目标 + side 姿态」上。**只有那一次不出门的规划**（求目标构型）把守门 ③
收窄成只查肩肘 J1/J2/J3 —— 腕部那 146° 就是回正本身，不是解错分支。

配套还改了 `_skip_lift_for_mode`：**「需要回正时不许跳过抬升」** —— 回正折在抬升里，没有
抬升就没有它，而且那一下要转大半个腕，贴着台面做太危险（`top_skip_lift` 还开着时也会被
这条拦下并打一行 warn）。`top_skip_lift` 的 yaml 现场值同时改回了 `false`。

> ⚠ **还没做完**：回正之后那一段放置，从回正后的构型去放置位姿时差的是**腕部**
> J4 63.2° / J6 55.3°（肩肘只有 29.2°，余量 35.1°）—— 比 60° 的门多 3°，所以那一段
> **仍然会回落 move_p**。两条备选都已量过：给腕部单独一个门槛（J4/J5/J6 设 70°、肩肘仍
> 60°，09-28 那次 J4 跳 117.6° 的事故仍拦得住），或把放置段也按关节直线拆成 2 段
> （每段 31.6°、余量 20.5°，守门一个字不动）。**2026-09-30 决定先按现状上真机，看回正
> 这一步本身对不对再定。**

**现场怎么判这次改动有没有生效**：

- LIFT 段应当**全部**走 `move_j`（不再出现「这一段走 move_j 不成」）；
- `_enter_place` 那句「全程同时转腕」应当从 **144.4° 降到 ~54°**（只剩 side 姿态到放置姿态
  的那点差）—— 这句现在既是日志、也是判据：回正生效时它不该再触发那条 >90° 的 warn；
- 日志里会出现 `开始抬升 (顺带回正姿态 175.3°, 拆 3 段 ...)` 与
  `回正规划: 腕部单轴差 134° ... 肩肘 40°, 余量 7.3°`。

---

### 5.13 夹爪接入：为什么不能直接用厂商的 04 / 05（2026-09-17）

**原来的做法**：夹爪完全靠人工 —— 在另一个终端跑 Gloria-M SDK 的两个 demo。

```bash
cd /home/qin/Gloria-M-SDK/Gloria-M-SDK
# 关闭夹爪
python -u demos/04_pv_resistance_stop.py --port /dev/ttyACM0 --velocity 2 \
       --resistance-torque 0.3 --hard-torque-limit 2 --timeout 25
# 打开夹爪
python demos/05_pv_open_home.py --port /dev/ttyACM0 --velocity 2
```

**直接照搬会造出一个「提着空气的机械臂」。** `04` 在 `finally:` 里**无条件**
`motor.disable()`（`05` 同），而 `disable()` 发 CAN `0xFD`、电机**立刻失能**：

> `MotorAPI.disable` 的 docstring：*De-energize the motor — it goes limp immediately.
> … the gripper will open or close under gravity / spring force.*

SDK 的中文 README 也写明了后果：**「检测到阻力后默认立即失能，因此不会继续保持夹持力，
物体可能松开。这一行为优先保证机械安全。」** 而且 `04` 里**没有**任何能关掉这个行为的
开关：目标到达、阻力、超时、Ctrl+C 四条路径全部汇到同一个 `finally`。

所以「`GRASP_WAIT` 时跑 04 → 抬升」= 夹爪在抬升**之前**就已经松开，瓶子留在台面上。

#### 落地形态：新脚本 + 子进程（2026-09-18 改成一次性命令；09-28 闭合可以常驻）

新文件 [`scripts/gripper_grip_hold.py`](scripts/gripper_grip_hold.py)，**一个进程跑一条
命令，跑完就退出**（`--hold-torque > 0` 时闭合那一条例外，见最后一行）：

| 模式 | 做什么 |
| --- | --- |
| `--mode open` | 张开 → 失能 → 退出（等价于 `05`） |
| `--mode close` | 抄 `04` 的闭合 + 阻力检测 → 在 stdout 上说明为什么停 → 失能 → 退出 |
| `--mode close --hold-torque T` | 同上，但停在阻力点上**不退出**：切 MIT 持续发帧保持 T N·m，直到 stdin 收到一行 `OPEN`（节点在放置时写）、`--hold-timeout` 到点、stdin EOF、或收到信号 |

**为什么是子进程而不是节点直连 SDK**（`/usr/bin/python3` 其实是能 `import
gloria_m_sdk` 的，所以技术上可行）：

- **串口争用**：节点自己不碰 `/dev/ttyACM0`，闭合/张开两条命令各自开一次口、用完就关；
- **能脱离 ROS 单独在台架上跑**，bring-up 时不需要起 ROS 就能调阻力阈值；
- 脚本崩了不会拖垮 20 Hz 的状态机。

**代价**：节点要管子进程生命周期（但也只有「起、轮询、回收」三件事）。

#### 2026-09-18：为什么从「保持进程」改成「发完就不管」

**事故**：真机上闭合时夹爪**已经夹住瓶子了**，脚本却在 0.43 s 时撞到硬扭矩上限，
按当时的实现把它判成 `[abort]` 并退出码 2，节点于是判定「闭合没有做」，整趟任务作废。

```text
[abort] hard torque limit triggered: -2.160 N·m
        (resistance threshold +0.321, debounce 0.12s, elapsed 0.43s)
```

根因是**扭矩冲过阻力阈值比 0.12 s 的消抖窗口还快** —— 夹到了硬东西（瓶盖 / 台面）
或者夹的位置不对。用户的要求是：**「不需要检查夹爪状态，只需要发送关闭夹爪和打开
夹爪命令就行」**，于是改成：

- **撞硬扭矩上限不再算错误**：按「已夹住」停下，只在 stderr 打一行 `[warn]` 说明；
- **节点不再解析脚本 stdout**：`GRIPPED` / `NO_OBJECT` / `RELEASED` 这几个标记连同
  解析它们的代码一起删掉了，判成败**只看退出码**；
- **不再有「保持进程」**：闭合命令跑完进程就退出，抬升期间夹持力靠机构自锁。
  （这一条 2026-09-28 又被加回来了 —— 见下面那一节。上面这三条里只有第二条还成立：
  节点仍然不解析脚本 stdout，唯一例外是 `[holding]`。）

> **⚠ 自锁从来没有实测过。** SDK 文档说失能后机构 *goes limp… open or close under
> gravity / spring force*，也就是**可反驱、断电不保持**。所以「闭合进程退出后瓶子还
> 夹得住」没有被验证过 —— 掉瓶就是它挡不住。
>
> 反过来说，`--mode close`（不带 `--hold-torque`）**本身就是那个自锁实验**：夹一个
> 瓶子跑一次，然后手动拉一拉，掉不掉立见分晓。

#### 2026-09-28：把「保持」加回来，这次是 MIT 力矩

09-18 之后抬升/搬运期间**没有任何东西在维持夹持力**，全靠上面那个没实测过的自锁。
09-28 把它按用户的选择加了回来，机制是 **MIT 恒力矩**（不是当年那版位置环）：

- 力**只能由主机持续发帧维持**：固件有看门狗（SDK `client.py`：*continue to hold its
  last commanded position until the firmware watchdog times out and cuts power*），
  链路一静默就切电。所以「发一条让 MCU 自己顶着」做不到 —— 必须常驻进程。
  demo 03 每 2ms 一帧、`sdk_linux` 的 clamp 每 10ms 一帧，脚本取 **10ms**。
- 顺序不能改：先用 PV 位置环顶在停下那个点上（盖住切模式那几十毫秒）→ `set_mode(MIT)`
  → **再 enable 一次** → 发第一帧 → **然后才打 `[holding]`**。
- **`[holding]` 是协议，不是日志**：保持进程不退出，节点只有这一个「夹住了、力已经
  加上」的信号。它是「stdout 只给人看」这条老规矩的**唯一例外**，且只在 `CLOSE` 阶段
  生效（张开类阶段绝不认它 —— 缓冲区里可能还留着闭合那趟的那一行）。
- 节点侧三件配套的事，缺一不可：
  1. **每拍收 stdout**（与状态无关，在 `_tick` 最前面）。不收的话 64 KiB 管道按
     100 Hz 打日志十几秒就满，子进程的 `print` 一阻塞，保持帧跟着断，看门狗切电 ——
     **瓶子在搬运途中掉**，正是这个功能要防的事故；
  2. **搬运途中盯着它**（`_gripper_hold_problem`，`LIFT` / `PLACE_MOVE` / **`PLACE_WAIT`**）。`PLACE_WAIT` 不能漏：`place_confirm_required=true` 时人可能站几分钟，
     这期间进程死掉没人发现的话，等 Enter 一来节点会回落到「另外起一个 open 进程」、
     那个进程正常张开、节点报「放置完成」—— 瓶子早掉了却报成功；
  3. **松开 = 给同一个进程写一行 `OPEN`**，不另起进程（串口只有一个，抢的一条会
     `[abort]` 退 2）。进程已经死了 / 写不进去时回落到「另起一个 `--mode open`」。
- **失败与退出「立刻松开」**（2026-09-28 用户定）：失败路径、`destroy_node`、
  SIGTERM 都让脚本**失能放手并退出**。注意这是「失能即 limp」而不是把手指主动打开 ——
  **机械臂若悬空，物体会掉**。这是有意选的，别当 bug 追。
- 「一路合到底」（夹了个空）时**不保持**：两指之间没东西，没什么可保持的，照旧失能
  退出、照常抬升。
- `gripper_hold_torque: 0`（代码默认）时**行为与改动前逐字相同** —— 这是整件事的回退
  路径，`test_gripper.py` 里 `CloseAtGraspTest` / `ReleaseAtPlaceTest` /
  `JobLifecycleTest` 那些老用例一条都没改，钉的就是它。

历史包袱记在这里，免得再走一遍：当年的实现也是「闭合那个进程一直活着，直到放置时节点
往它的 stdin 写一行 `OPEN`」，但它重发的是 `send_pos_vel(position=q_contact, velocity=0)`
（位置环），而且额外踩过一个 bytes/str 的坑（`stream.write("OPEN\n")` 抛 `TypeError`，
整条链只在最后张开那一步崩）。**这一版 stdin 全程走 `os.read`/`os.write` 的字节面**，
`RealPipeTest` 里有一条真管道的往返用例钉着它。

#### 节点侧的动作语义

| 时机 | 节点做什么 |
| --- | --- |
| `execute_grasp` | 先 `GRIPPER(张开)`，退出码 0 才 `WAIT_TARGET` —— 每轮从确定状态出发 |
| `TRACK_WAIT` 确认 | 不变：下降。差别只在**谁发起**：Enter / `grasp_confirm`，或 `auto_grasp_stable_time` 到点（§5.10）。两种入口都走同一个 `_confirm_descend`，下游零差异 |
| `GRASP_WAIT` | 起一个 `--mode close` 进程；**命令跑完就抬升**。`gripper_hold_torque=0` 时判据是退出码 0、脚本 stdout 说什么都不算；`> 0` 时判据多一条 —— 见到 `[holding]` 且进程还活着也算完成（那时它不退出）。超时、退出码非 0（闭合根本没做，见 `[abort]` 那行）、或**见到标记但进程随即退出** → 判失败。默认（`grasp_confirm_required=false`）不等 Enter，`descend_dwell` 到点就做 |
| `PLACE_WAIT` | `gripper_hold_torque=0` 时起一个**新的** `--mode open` 进程（不是去命令闭合那个，它早退了）；`> 0` 时**复用**那个还在保持的进程（给它写一行 `OPEN`），它死了才回落到新起一个。跑完就回待机位。默认（`place_confirm_required=false`）不等 Enter，`place_descend_dwell` 到点就做 |

夹爪步骤期间机械臂**一动不动**（`GRIPPER` 是个非运动状态），所以确认打开时 Enter 之后
手还伸在夹爪里也来得及退开。**两个确认默认都关掉，这一段就没有「人退开」的时机了** ——
手不要在夹爪行程里，闭合/张开由 dwell 定时触发。

**`dry_run=true` 时一行命令都不发**（只打一行 `[dry_run] 夹爪命令: …`），
与「dry_run 不碰硬件」的既有规矩一致，所以无硬件也能验证整条链。

#### 台架自测（上真机前先做）

```bash
cd /home/qin/zdl_fz/src/bottle_grasp_executor
/usr/bin/python3 -u scripts/gripper_grip_hold.py --mode close \
    --sdk-src /home/qin/Gloria-M-SDK/Gloria-M-SDK/src \
    --port /dev/ttyACM0 --velocity 2 --resistance-torque 0.3 \
    --hard-torque-limit 2 --timeout 25
```

1. 夹一个瓶子 → 命令跑完、进程退出，**然后用手拉一拉夹爪**：拉得掉就说明机构不自锁，
   抬升会掉瓶（见上面那个 ⚠）——不过现在现场开着 `gripper_hold_torque: 1.0`，抬升
   期间不靠自锁，这条只在「把保持关掉对照」时才有意义；
2. 空夹爪跑一次 → stdout 里会说「一路合到底」，脚本层面标出「没夹到东西」。不报就说明
   `--resistance-torque` 对这台机构太钝 —— 不影响流程能不能跑，只影响日志里看不看得出
   「这次夹空了」。注意这一条**不会进保持**（没夹到东西没什么可保持的）；
3. 再跑一次 `--mode open` → 应张开、失能、退出 0；
4. 撞硬扭矩上限时应当看到 `[warn]`（不是 `[abort]`），退出码仍是 0 —— 这正是 09-18
   那次事故的修复点。

**保持那一段（2026-09-28 加）要单独验，命令在脚本 docstring 里**：

```bash
( sleep 20; echo OPEN ) | /usr/bin/python3 -u scripts/gripper_grip_hold.py \
    --mode close --hold-torque 1.0 --hold-timeout 120 \
    --sdk-src /home/qin/Gloria-M-SDK/Gloria-M-SDK/src \
    --port /dev/ttyACM0 --velocity 2 --resistance-torque 1.0 \
    --hard-torque-limit 2 --timeout 25
```

看四件事：① `[holding]` 那行里 `tau_fb` 是否接近 `-tau_cmd`（**不接近就说明力没加上**：
模式没切成功，或切完没 enable）；② 那行里的 `rate` 是否远大于 1/看门狗超时；③ 期间用手
拉夹爪，应该比不带保持时**明显更抗拉**；④ 20 秒后应张开、失能、退 0。再 `kill -TERM`
一次 —— 应当**失能放手**后退出（不是把手指主动打开）。
**重点看从 `[stop] resistance detected` 到 `[holding]` 之间**：那是切模式的窗口，瓶子
若在这一下被松开，说明切模式期间力断了 —— 退路是把保持从 MIT 换成 PV 位置环
（`send_pos_vel(q_contact, 0.0)` 持续发帧），只动保持循环那一段，节点侧协议不变。

脚本的失败路径是干净的一行，不吐 traceback：

```text
$ /usr/bin/python3 -u scripts/gripper_grip_hold.py --mode close --port /dev/null
[abort] Cannot open port '/dev/null': Could not configure port: (25, 'Inappropriate ioctl for device')
$ echo $?
2
```

#### 上真机的顺序

1. `gripper_enabled: false` 跑一遍，确认老流程**一字未变**；
2. `dry_run` 起节点 + `gripper_enabled:=true`，确认启动日志有「夹爪: 开 (…」那一行、
   且全流程零 spawn；
3. 改成 `true` 上真机，**先空夹爪**触发一次 —— 能看到脚本在 stdout 上说明「一路合到
   底」，然后照常抬升（节点不判这个）。这一步只为确认「闭合这条命令真的发出去了」；
4. 再放瓶子，跑完整条链：悬停 → 物体不动 3s 自动下降 → 自动闭合 → 自动抬升 → 搬运
   → 自动张开 → 回位（**整条链已经没有任何一步需要 Enter**：`TRACK_WAIT` 那一次由
   `auto_grasp_stable_time` 接管，另外两处由两个确认开关关掉。想临时把每一步都握回手里，
   把 `auto_grasp_stable_time` 设成 0、两个确认开关设成 `true` 即可）。**抬升的过程中
   盯住瓶子** —— 这一段是上面那个 ⚠ 唯一会暴露的地方。
   `gripper_hold_torque > 0` 时再补两个动作：**搬运途中手动 `kill -9` 那个保持进程**
   → 节点应在一拍内（50ms）报 ERROR 并回位（这是 `_gripper_hold_problem` 唯一的现场
   验证）；**再 Ctrl+C 一次** → 夹爪应立即失能放手。收工后 `pgrep -f
   gripper_grip_hold.py` 应为空。
5. 最后做一次**对照**：`gripper_hold_torque:=0.0` 重跑同一条链，日志与状态序列应与
   改动前逐字一致 —— 这是「默认值不改现有行为」的唯一证据。

**串口只有一个**：`gripper_enabled=true` 时不要再在另一个终端跑 04/05，也不要同时开
厂商的 web server，否则脚本会以 `[abort] Cannot open port…` 退出（节点会把它报成
「夹爪动作失败」并回待机位）。

**回归测试**：`test_gripper.py`（57 条）。大部分用例不 spawn 真进程
（`_spawn_gripper_job` 被换成假 job），覆盖「关着时一次都不 spawn」「每轮先张开」
「Enter 只发起、命令跑完前机械臂不动」「脚本 stdout 说什么都不影响判断（含 09-18 那次
的 `[warn]` 撞硬扭矩上限）」「退出码非 0 才判失败」「超时判失败」「失败后不再补发指令」
「张开是**新起一个**进程、不是复用闭合那个」「`dry_run` 零 spawn」「`GRIPPER` 在回位
白名单里」「退出时放脚本自己跑完、不杀」与参数一致性；
`HoldingTest`（15 条）只钉 `gripper_hold_torque > 0` 那半边：`[holding]` 推进且进程留着、
标记到了但进程已死要判失败、张开类阶段不认这个标记、LIFT/PLACE_MOVE/**PLACE_WAIT** 途中
进程死掉立刻判失败（而在 HOME 等状态不判）、释放时复用同一个进程 / 进程已死或写不进去时
回落新起一个、抬升途中的 `_tick` 也会收 stdout（管道憋死那条）、`dry_run` 不留
保持状态、`place_enabled=false + 保持开着` 开工被拒、`destroy_node` 会写 OPEN 让它松开；
`RealPipeTest` 里另有两条真管道的 stdin 往返（含「进程已退出时写不进去要返回 False、
不能抛异常」）；

`RealPipeTest`（3 条）另起**真进程 + 真管道**，专盯假 job 盖不住的那几样：半行不阻塞、
没有换行的最后一行能交出来、跑完的命令能读到退出码。跑法：

```bash
cd ~/zdl_fz/src/bottle_grasp_executor && /usr/bin/python3 -m unittest -v test_gripper
```

---

### 5.14 循环模式：不打断在跑的这一次、只在真失败时停、回来只认新目标（2026-09-21）

`~/execute_grasp` 现在同时是循环模式的开关（用法和停止条件见 §3.4）。这里记五条
**看起来可以偷懒、但都不行**的设计点。

#### 1. 「关闭」为什么不能打断正在跑的那一次

调它关闭的瞬间，机械臂可能正夹着瓶子、正把瓶子按在放置点上、或者正盲降往下走
（`DESCEND` 不判到位，见 §5.11）。那一刻拧断状态机 = 停在半空、掉瓶，或者夹着东西
直接回待机位。所以关闭只清一个标志位（`_loop_armed`），**任务照常跑完**，回到待机位
之后没有下一次自动触发而已。想立刻停，用 §3.5 的办法（Ctrl-C / 下使能），不要指望
这个服务 —— 响应 `message` 里也会写清楚「本次任务照常跑完」。

#### 2. 为什么只有「真失败」才算失败，「没目标」不算

2026-09-21 那几次 `ik_status=2 (超出工作空间)` 拒收（§6.3）不是随机故障：同一条位姿
原样重发一次还是被拒。循环如果在这种状态下继续，就是**一遍遍重复整个抓取流程再
失败** —— 每轮几十秒、确认点要人按、夹爪每轮开合，而且最后一轮还会把瓶子留在待机位。
更糟的是**放置阶段以后**的失败（回位、夹爪）：那时手里可能还夹着瓶子，自动接着抓会
把瓶子带到下一个目标上。这类失败一律收工、等人看过日志（修好、或换个位置）再调一次
`~/execute_grasp` 重新开启。

反过来，「视野里没有目标」**完全不是故障**：下一个瓶子还没摆上来、或者刚才那个瓶子
被推走了，都是正常工况。用户 2026-09-21 的要求原话是「机械臂回到工作位置后继续接受
物体识别到的坐标，然后抓取」—— 这时收工等于把自动化断在最常见的一次停顿上，人还得
回来再按一次。

实现上分两条路，判据只有 `_fail` 的一个参数：

- `_fail(reason)`（默认）：关循环 + 一行 `循环模式已自动关闭 (这一次任务失败)`。
  **所有**真失败都走这里 —— IK 拒收 / 无反馈 / 运动超时 / 夹爪失败 / 内部错误；
- `_fail(reason, keep_loop=True)`：**只给三条「没等到目标」用** —— `_tick_wait_target`
  的 30 s 超时，以及 `_follow_target` 里两条 `TRACK_TARGET_LOST`（候选超过
  `max_result_age` 没更新 / 一个候选都没收到）。循环保持开启，回待机位接着等，日志是
  `循环模式保持开启: 这一次只是没等到目标 ...`。

「回位判不到位、放弃回位」那条异常出口（`_tick_blind_home`，§5.9）仍然**硬关**循环 ——
机械臂位置本来就是不确定的，不能接着抓。

**不要**把这件事放到 HOME→IDLE 那个共同出口上（`_advance_after_motion`）：成功和失败
**都**经过它，在那儿关会把成功的循环一并关死（失败时 `_fail` 已经先关了）。

#### 3. 回来之后为什么按「观察时刻」认目标，而不是「有没有候选」

`_tick_wait_target` 拿的是 `self._latest_candidate`，而它**不查候选年龄**；
`_latest_candidate` 在任务结束时也不会被清掉（`_reset_target_lock` 只清
`_locked_track_id` 与 `_target_history`）。单发流程下没人踩到，是因为每轮都得人重新
按一次服务。循环一旦自动化，第 2 轮会在进入 `WAIT_TARGET` 后**立刻锁住上一轮那个瓶子
的坐标** —— 那个位置现在是空的（瓶子已经放到相机视野外了），1.0 s 稳定窗口一过就去
抓空位：机械臂对着空台面下降、闭合、抬升，然后回位。同一件事还有第二个后果：
`_tick_wait_target` 的 30 s 超时写在 `latest is None` 那个分支里 —— 候选永远不为
`None`，**这条超时永远不会触发**。

修法是给 `_latest_candidate` 加一个**时间戳守卫**，而不是简单地「起手清一次」：

- 回到 IDLE 的第一拍记下 `_loop_guard_at = time.monotonic()`（`_tick_loop_wait`），
  之后只有 `received_at` **晚于**它的候选才算真目标，旧的顺手清掉；
- 为什么用时间戳而不是「起手时一律清空」：清空是**一次性的动作**，只能保证「起手那一
  瞬间手里是空的」；守卫是一个**持续成立的判据**，任何来路不明的旧候选都进不来。而且
  起手那一拍之后新到的候选**不用再等一轮**（真实感知 10 Hz 连续发布，正常情况下一拍
  之内就有新的），清空则会白等一个稳定窗口；
- 守卫在 `_begin_task` 里清掉（任务一起步就不再是「在待机位等目标」了）—— 放这里而不是
  循环起手那一步，是因为**人工触发**也可能从「等过目标的 IDLE」起步，留着一个旧的守卫
  只会让**更旧**的候选混进来；
- `_tick_wait_target` 一行没动，单发流程逐字节不变。

回归测试在 `test_auto_repeat.py` 的 `LoopCandidateFreshnessTest`：第 1 轮正常跑完之后
旧候选还在手里，第 2 轮连 tick 20 次都不许锁、不许发指令；反过来，回来之后**新**看到的
目标会被立刻接受（`test_new_candidate_after_return_home_is_accepted` 端到端断言抓的是
新坐标）。

#### 4. 为什么循环这一段放在 `_tick` 的 handler 派发**之前**

`_tick` 每 50 ms 一拍，HOME 的到位判定在 `_tick_motion` 里做。把起手放在派发之后，
就会出现「同一拍里刚判完到位、立刻发下一条指令」—— 机械臂还没在待机位站稳。放在派发
之前，回位到位的那一拍结束时状态还是 `HOME`（不会起步），**下一拍**（50 ms 后）才进
「等目标」，指令更是要等到有新目标之后的那一拍才发。`test_auto_repeat.py` 的
`test_home_arrival_tick_only_returns_to_idle` 钉的就是这个间隔。

附带的一条：**等目标这一段没有任何超时**。`_tick_loop_wait` 只是空转 + 每 10 s 一行
日志，不发指令、不改状态、不判失败 —— 所以「一直在等」和「卡死了」在日志上必须能分开，
那行 `还在等相机的新目标 ...` 就是给人看的（`test_standby_wait_never_gives_up` 连 tick
200 拍，钉住不离开 IDLE、不发指令、不关循环）。

#### 5. 为什么唯一的硬闸是 `place_enabled`

关掉放置时，`LIFT` 之后直接 `HOME`，而且**夹爪保持闭合**（§3.4）—— 瓶子被夹着回到
待机位，下一轮起手第一件事是张开夹爪（`_begin_task`），瓶子就掉在待机位下方，然后
又被抓起来。同一个瓶子无限循环，和「刚放下的瓶子离开视野、相机看到的是下一个」这个
前提直接冲突。所以只有配了放置才谈得上循环：不满足时任务照常跑，但**只跑这一次**。

另外三条只影响「要不要人盯着」和「从哪起步」，不该挡住循环，所以只打 WARN ——
`grasp_confirm_required` / `place_confirm_required` 开着 = 循环退化成一次一确认；
`return_to_standby_on_finish=false` 或待机位校验没过 = 循环不在待机位起步，而是从
上一轮结束的位置（抬升点 / 放置点）直接去找下一个目标。

---

### 5.15 上位机求逆解（IK）+ 关节下发：查到哪一步了（2026-09-28）

**动机**：固件的 IK 判决挡过好几次事 —— `ik_status=2 (超出工作空间)`、
`5 (关节单步过大)`（09-28 那次抬升失败就是它）。一个自然的想法是**在 PC 侧自己求
逆解**，再把关节角通过 `/control/move_j` 下发。零件其实都现成：桥早就订阅了这条话题
（`send_position_targets()` + `MoveMode.J`），`scripts/move_joints.py` 早就是那个
「按下发 + 打印每轴差值 + 等 Enter」的工具，`/feedback/joint_states` 报当前关节角。

**pinocchio 也不需要 conda**：`ik_h3001.py` 用的那份 pinocchio 4.0.0 就在 ROS Humble
的 3.10 里（`/opt/ros/humble/lib/python3.10/site-packages/pinocchio`），所以
`/usr/bin/python3` 直接 `import pinocchio` 就能跑，不需要 `conda activate base`，也不需要
`env -u PYTHONPATH`（那条是为了绕开 3.13 的 conda 环境串到 ROS 的 site-packages）。

#### 前提一：**上报的关节角**与 URDF 差 J2/J3/J5 三个符号

`ik_h3001.py calibrate` 在本机的输出（纯读，`connect()` 只收不发）：

```text
MCU 关节角 (deg) : [  -3.021,  13.199,  -0.455,  11.780,  -11.962,  -1.463]
MCU 位姿         : [  62.063,   1.382, 172.044,  84.124,   79.923,   83.619]
URDF 正解        : [  92.620,  -8.831, 186.893,  95.873,   79.924,   90.337]
位置偏差 35.476 mm / 姿态偏差 13.534 deg
```

把 64 种关节符号组合全试一遍，**只有一个吻合**：

```text
[1, -1, -1, 1, -1, 1]  ->  位置 0.00 mm, 姿态 0.00 deg
```

厂商 web UI 里那张 `WEB_JOINT_DIRECTION = [1,-1,-1,1,-1,1]` 是同一件事（显示时乘上去），
而它的 `WEB_JOINT_COMMAND_DIRECTION = [1,1,1,1,1,1]` 是原样下发。由此：

| 约定 | 是谁 | 换算 |
| --- | --- | --- |
| **物理 / URDF 约定** | 模型里的关节角，也是 IK 解出来的东西 | `q_物理 = [1,-1,-1,1,-1,1] · q_上报` |
| **话题约定** | `/feedback/joint_states` 报的、`/control/move_j` 吃的，**同一个约定** | `q_上报 = [1,-1,-1,1,-1,1] · q_物理` |

⇒ **要下发的 = `[1,-1,-1,1,-1,1] · (IK 解出来的 q_deg)`**。

**判据（2026-09-28 真机实测，别再推）** —— 两步，都不依赖约定假设：

1. 发 `[.. J2=−19.4 ..]` 出去，回报**仍然是 `J2=−19.4`**（不是 `+19.4`），而同一帧里
   J6 照常走到 90° —— 命令确实执行了，只是话题约定与 URDF 相反；
2. **物理动作验证**：`move_joints.py 0 0 0 0 0 0`（J2/J3/J5 都是 0，两种约定给同一个
   结果，所以这一步**无论约定如何都安全**）把臂收到零位，再发
   `0.000 -3.349 8.790 -0.003 -5.441 0.003` —— 约定对 → 末端**往上** 40mm；约定反 →
   **往下** 40mm。**实测：往上。**

> ⚠ 我（Claude）在这条上**一度推反了**：先由「FK(上报角 × 符号) == 上报位姿」推断
> 「指令吃 URDF 约定、反馈是反的」，并据此说「IK 结果原样下发」。上面的实测否掉了它 ——
> 根因是当时手边那条「IK 原样发能到目标位姿」的证据是在**接近零位**的姿势上做的
> （零位附近两种约定重合，分辨不出来）。教训：**涉及符号约定的结论，必须在关节角
> 明显不为零的构型上验**。

#### 前提二：有些直线在几何上做不到 —— 手腕限位墙

实测「保持姿态、从抓取点上方 10cm 垂直下降到抓取点」这条直线：

```text
CAD 限位:  joint_3 [-170.2, 0.0]      joint_4 [-100.3, +100.3]
z=349.5   J4 = +85.6   (距上限 14.7°)
z=343.7   J4 = +98.1   (距上限  2.2°)
z=341.7   连续解不存在 —— J4 需要 > +100.3
          -> 求解器被迫翻腕: J4 +98.1 -> -77.7 (167°)
```

32 次起点和 200 次起点结果完全一样，所以**这是几何上被迫的，不是求解器的毛病**。
固件对同一条直线的判决就是 `ik_status=5 (关节单步过大)` —— 09-28 那次抬升失败的
根因在这里，不在夹持力、也不在通信。

**顺带解释了两件旧事**：① 当初「发 move_p 让它下降，路径规划有些不对」很可能就是这个
被迫的翻腕（`DESCEND` 不判到位，所以这次翻腕在日志里没有痕迹）；② 改成 `move_l` 之后
仍然解决不了 —— 换直线插补不改变「连续解不存在」这个事实。对策只有换姿态、或者把翻腕
挪到空载时去做。

#### 工具

```bash
cd <bottle_grasp_executor 包目录>
# 单点: 位姿 -> 关节角 + 限位余量
/usr/bin/python3 scripts/ik_plan.py pose 421.2 -48.9 257.0 166.4 79.2 164.0

# 直线: 逐点求逆解, 检查相邻点是不是同一条构型
/usr/bin/python3 scripts/ik_plan.py line 421.2 -48.9 358.7 166.4 79.2 164.0 --delta 0 0 -0.101

# 只读: 验「上报关节角 vs URDF 正解」(需要 bridge 在跑)
/usr/bin/python3 scripts/ik_plan.py check
```

**只算不发** —— 不发任何运动指令、不碰 CAN（`check` 只订阅 ROS 话题）。它把
「相邻路点的最大关节差」「距限位的余量」「在第几点被迫换构型」打成一张表，判定可执行时
直接把 `move_joints.py` 的命令行打出来供逐个粘贴。位姿一律用**上报原点**那一套
（与 yaml / GUI / `/feedback/tcp_pose` 同源）；节点内部的「夹爪点」与它差一个工具偏移
（`gripper_offset_tool_xyz`，代码 0.08m / yaml 0.18m），喂错了就是几十上百毫米的偏差。

注意 `check` 订阅反馈要用 **BEST_EFFORT** —— 桥的 `sensor_qos` 是 BEST_EFFORT，而 rclpy
订阅默认 RELIABLE，两者不兼容、一条都收不到（症状是「等了 5 秒什么也没来」，很容易
误判成「bridge 没在跑」）。

#### 节点接上了（2026-09-28 当天下午）

**六个运动段全部优先走 `move_j`**：位姿 → 节点自己求逆解 → `/control/move_j`。
解不出来（或过不了下面三道守门）就**回落**到 `move_p` / `move_l` —— 所以「求逆解失败」
不是新的失败模式。总开关 `ik_motion_enabled`，置 false 就回到 09-28 之前的行为。

| 落点 | 说明 |
| --- | --- |
| 依赖 | `ik_h3001.py` + URDF + CAD 限位 CSV **原样 vendored** 进 `ik/`（见 `setup.py` 的注释）。URDF **不带 21MB 的 STL**（`buildModelFromUrdf` 不加载网格，实测过）；CSV 改成了 ASCII 名 |
| 种子 | `/feedback/joint_states`（新订阅，**必须用 sensor QoS** —— 桥是 BEST_EFFORT）|
| 出口 | 仍是 `_send_move_p` 一个，内部多一条分支。**没有另开方法** —— 否则几十条靠 `self.sent` 的用例会整体失明（`self.sent == []` 那类会**假绿**）|
| 求解 | 每条指令现算一次（**不是每拍**），`restarts=0` 实测 3.9ms（最坏 13ms），50ms 一拍装得下 |
| 到位判据 | **没动** —— 继续用 TCP 位姿。拿关节角判会永远不收敛（差三个符号）|
| 记账 | 多一个 `_last_command_joint`（None = 笛卡尔），`_active_motion_timeout` 按它切到 `move_j_timeout` |

**三道守门**（接过来的就是固件原先替我们做的那三道判决；最危险的不是解不出来，是
**解出来了但解错了**，回落 move_p 对后两条毫无用处）：

1. **收敛**（对应 `ik_status=2 超出工作空间`）；
2. **CAD 限位余量 ≥ `ik_min_margin_deg`** —— 注意**求解时就把 CAD 限位喂进了** `ik()` 的
   bounds。用默认的 URDF 限位会松很多（J2/J3 ±180、J5 ±100.3 vs 真值 J2 [0,179.9]、
   J3 [−170.2,0]、J5 ±69.9 —— **J5 松 30°**），只事后检查的话「J5 解到 90°」会带着
   10.3° 余量被放行；
3. **与当前构型的单轴差 ≤ `ik_max_joint_jump_deg`**（对应 `ik_status=5 关节单步过大`）——
   本地求解会在解断裂处跳到另一条分支，而那种解**收敛、误差 0、余量还有几十度**
   （实测 J4 跳 117.6° 时余量 34.7°），只有这条抓得到。move_j 是关节插值，这条跳变会被
   **原样执行成一次实打实的关节大摆**。

外加一条**运行期自检**（1s 一次，只在 IDLE 跑）：`URDF 正解(反馈关节角 × 符号)` 的位置
与上报位姿比，差 > 2mm 就**永久降级回 move_p** 并报错。它一次覆盖「URDF 版本不对 /
符号约定错 / 末端 frame 错」三种致命配置错误 —— 这三种都会让 IK 解出**另一个位姿**的解。

#### 还没做的

- **真机验 `move_j` 的失败语义**（最高优先）：固件对 J 模式不跑逆解，所以 `ik_status`
  对 J 没有可信判决 —— J 指令**故意不立那道案**（立案只会误判：那个值是粘住的）。
  代价是「J 指令没被执行」只能等 `motion_timeout`，没有 0.8s 的快速通道。
  要用 `move_joints.py` 小步逼近发一条**略超 CAD 限位**的 J 指令，看
  `/feedback/arm_status` 的 `ik_status` / `ik_joint_limit_fault` / `motor_error`
  有没有反应 —— 有反应才谈得上把快速通道加回来。
- **速度/耗时标定**：同一位姿 `move_j` vs `move_p` 各走 3 次量墙钟，再定
  `move_j_timeout` 与回位的倍数公式（`_home_time_multiplier_for` 是按**笛卡尔距离**
  估的，对「腕部转 180° 但末端几乎没动」会算出 1 段 → 假超时）。
- **灰度顺序**：`ik_motion_states` 先只留 `["HOME"]`，稳了再加 `PRE_GRASP` →
  `TRACK_WAIT` → `PLACE_MOVE`。真机第一轮盯 **DESCEND**（它是唯一为「垂直落到瓶子上」
  而设的直线段，改成关节插值后末端会走一条 16.4mm 的弧）。
- 真机上发过关节指令跑**完整一条链**的闭环（`/control/move_j` 这条通道 09-28 才第一次上真机）。

---

## 6. 当前未解决的问题

### 6.1 手眼标定后 X / Y 方向相反

按松灵机械臂的手眼标定流程标定完，X 与 Y 坐标与实际方向相反。

- 可能原因：坐标系定义与松灵不同。
- 处理：目前按何工要求，`rx` 与 `ry` 位置取反，标定结果大致一样，但**外参误差有点大**。

### 6.2 标定外参误差大

按松灵流程标定后外参误差很大。

- 可能原因：还在确认。
- 处理：尚未处理。

### 6.3 机械臂运动规划问题

- 现象：节点在两点之间自己插中间点，「算得对」不等于「固件做得出来」——`workspace_*`
  是个凸盒子，而固件 IK 在**固定姿态**下的可达域不是凸的（§5.12 那段）。2026-09-21
  真机回位就死在这样一个中间点上（`ik_status=2 超出工作空间`，而起点和待机位两个端点
  各自都够得到）。
- 现状（2026-09-21）：路径规划整个交给固件 —— 放置段（2026-09-20）与回位段都不再由节点
  造中间点，一个目标一条绝对指令。剩下的缺口是节点**没法事先知道**固件收不收某个位姿，
  只能等固件的逐次判决（§5.12 的 `ik_status` 那一段）；换位姿时要落地验证，用
  `scripts/check_place_pose.py` 单独试那一条，不要拿整个任务去赌（§5.12）。
- 处理：（待补充）

### 6.4 夹爪偏移与 rx/ry 取反都是临时方案

2026-09-15 起按实测值落地，两处都写成**开关/参数**，不写死在逻辑里：

| 问题 | 落地位置 | 新机械臂修好后的处理 |
|---|---|---|
| 上报 RPY 的 rx/ry 需取反 | `h3001_bridge`，开关 `invert_rx_ry`（当前 `false`） | 先确认是否真的需要；要打开就得同步改 `side_orientation_xyzw`（§5.6） |
| 上报原点离夹爪 8cm | executor，参数 `gripper_offset_tool_xyz`（现值 `[0.0, 0.0, 0.08]`，2026-09-21 起） | 置 `[0,0,0]`（退化为恒等） |
| 待机位的坐标约定 | executor，参数 `standby_pose_reported_mm_deg`（上报末端原点系） | 不用改 —— 存的是源头格式，偏移置零后自动仍然正确。只有 `invert_rx_ry` 变化时才需按 GUI 重读一次（§5.7） |
| 放置位姿若从厂商 GUI 读 | executor，参数 `place_pose_reported_mm_deg`（上报末端原点系） | **与上一行相同**（2026-09-18 起统一）：从 GUI 直接抄那 6 个数，不用自己换算。偏移置零后也仍然正确 —— 存的是源头格式（§5.12） |

机械臂换新版本后先确认这两条是否还存在，再决定要不要关。

**上真机前必须用 dry_run 核对 §3.3 那几行日志**。历史上按「沿夹爪轴延长 13cm」实现
时（`+0.13` 再改 `-0.13`），末端原点被推到 `[0.3736, 0.0442, 0.1149]`，而瓶子在
`[0.2463, 0.0222, 0.1000]` —— 夹爪越过瓶子 12.7cm、指向远方。符号和数值都得看日志。

**已知副作用**：到位判定用**上报姿态**算 `R`，下发用**指令姿态**。两者不一致时
8cm 力臂会放大姿态误差（最坏转轴下 3° ≈ 4.2 mm = 0.08 × sin 3°，与 `position_tolerance` 同量级）。
若 `PRE_GRASP` / `LIFT` / `HOME` 开始频繁报差几毫米不放行，先查是不是姿态没跟到位，
而不是直接放宽容差。（`DESCEND` 不做到位判定，见 §5.11，不受这条影响。）

### 6.5 抓取姿态卡在手腕限位墙上（2026-09-28 新发现，未处理）

现场用的那组抓取姿态（上报原点 `[421.2, -48.9, 257.0]`，姿态 `[166.4, 79.2, 164.0]`，
工具轴与竖直方向夹角 **79.5°**）配上「保持姿态垂直下降/抬升」这条直线，**在几何上做不到**：

- 悬停点 `z=358.7` 用 6 组不同种子 + 64 次重启求解，**只有一个逆解**（J4=+66.9）；
- 抓取点 `z=257.0` **只有另一个分支的解**（J4=−16.9）；
- ⇒ **两个端点分处两条唯一的分支**，任何连接它们的轨迹都必须中途翻腕。固件的 `move_l`
  要么拒收（09-28 那次抬升的 `ik_status=5 关节单步过大`），要么硬翻过去（下降那次 ——
  `DESCEND` 不判到位，所以日志里没有痕迹）。这也解释了最初「下降路径不是直线」的抱怨。

**换自转（RZ）救不了**：扫了 360°（每 20°），RZ 264°~64° 这一大段**干脆无解**，
其余全都有 87°~186° 的被迫换构型。

**换倾角可以救**（工具轴与竖直方向的夹角；`RZ` 不影响倾角）：

| 倾角 | 下降线 | 最小限位余量 |
| --- | --- | --- |
| 13.6° | **悬停点无解** | — |
| 20.1° / 32.7° | 连续 | **−23.4° / −1.5°**（越过 CAD 机械限位，实际不可执行）|
| 40° / 46.6° / 50° | 连续 | 10.4° / 19.5° / **24.3°** |
| **60.9°（ry 79.2→60.0，其余不变）** | 连续（相邻差 8.1°）| **40.1°** ← 余量最大 |
| **79.5°（现值）** | **跳 159°** | 12.9° |
| 89~105° | 连续 | 20~36° |

⇒ **现值 79.5° 正好落在坏带（70~90°）里**；可用的是 **45~65°**（斜向下抓）与
**89~105°**（水平/略仰）。倾角越接近正上方朝下（<35°）越糟：要么无解，要么解到机械限位之外。

⚠ **换倾角的前提**：倾角变了，夹爪的**闭合方向**也跟着变；夹瓶子要求两指**横向**合上。
上面这些都是固定 `RZ=164.0` 扫出来的，闭合方向对不对**从模型里看不出来**（夹爪不在
URDF 里）—— 现场装夹后要用 `ik_plan.py line` 带着新的 roll 重扫一遍。

**结论：这不是软件问题，是抓取姿态标定的问题。** 在姿态定下来之前，把上位机 IK 接进
节点也是同一条几何，解决不了。节点目前行为未变（`ik_plan.py` 只算不发）。

---

## 7. 故障排查

| 现象 | 可能原因 | 处理 |
|---|---|---|
| `h3001_bridge` 收不到反馈 | `can0` 没 up / 被 web server 占用 | §2.1 + §3.0 |
| 反馈时断时续 | 两个进程抢 `can0` | `pkill -f h3001_web_server_linux.py` |
| `无法导入 sdk_linux` | SDK 路径变了 / 该目录没有 `sdk_linux.py` | `export H3001_SDK_PATH=...`（§2.3） |
| 感知 `ImportError` | `LD_LIBRARY_PATH` 写法不对 | 用替换形式（§5.1） |
| 感知打不开相机 | 有别的进程占用设备 | 关掉 `orbbec_camera` 等节点（§5.2） |
| executor 一直 `WAIT_TARGET` | JSON 没生成 / 路径不一致 | 核对 `candidates_json_path` 与 `--json-out`；看 JSON 的时效（`max_result_age`） |
| 抓取点整体偏移 | 感知与 executor 用了不同标定 | 两边 `--hand-eye` 与 `hand_eye_yaml_path` 必须同源 |
| `DESCEND 下降指令已发出` 那行的「实际」显示还在动 | `descend_dwell` 小于 12cm 的实际走完时间 | 调大它（逐次读取，§5.11）。下降**本来就不判到位**，节点不会因此报错 |
| 下降末端总是比抓取点低几毫米 | 正常：重力下垂（§5.4），下降不做迭代修正 | 要补就在 `grasp_offset_z` 上补一个正值，别去改 `pre_grasp_offset_z`（那是行程） |
| 机械臂在下降时完全没动，却不报错 | 下降不看反馈：被挡住 / IK 无解都不会失败 | 看那行日志的「实际」；确认现场（是不是撞住了、目标点是否可达）后重新触发 |
| 下降整段很慢 | bridge 的 `speed_percent` 太低 | 启动行看 `速度=N%`；launch 参数覆盖 yaml，改 launch 或从命令行给（§3.1） |
| 下降太快 / 没停稳就进 `GRASP_WAIT` | `descend_dwell` 给少了 | 调大它（逐次读取，§5.11） |
| 夹爪位置整体差 8cm 量级 | `gripper_offset_tool_xyz` 符号或单位错 | 看 dry_run 日志的「夹爪偏移」模长是否 = 0.08 m（§3.3） |
| 姿态方向相反 / 转错方向 | `invert_rx_ry` 两侧没成对生效 | 确认反馈与指令用同一个开关（§5.6） |
| 抓取时姿态偏了十几度 | `invert_rx_ry` 与 `side_orientation_xyzw` 约定不一致 | 两者必须同约定：取反关则四元数用现值的；取反开则四元数 x、y 取反（§5.6） |
| `motion_timeout` 超时 | 目标不可达 / 被挡住 | 看日志的目标点是否在工作空间内（`workspace_*`）；**在界内也不代表固件收得下** —— 那种情况会先打 `固件 IK 拒收`（见那一行），别只看这个盒子（§5.12） |
| `WORKSPACE_REJECTED 上报原点 ...` | 夹爪在界内但上报原点越界（或反之） | 夹爪与上报原点都要在界内，见 §5.6；调 `workspace_*` |
| `TRACK_WAIT` 按 Enter 没反应 | stdin 是管道 | 用 `ros2 topic pub`（§5.5） |
| 跑到 `GRASP_WAIT` / `PLACE_WAIT` 不等我按 Enter 就自己动了 | 正常：现场 yaml 把这两个确认关掉了（`grasp_confirm_required` / `place_confirm_required` = `false`） | 想恢复人工把关就 `ros2 param set /bottle_grasp_executor grasp_confirm_required true`（逐次读取，不用重启），或改 yaml 后重启 |
| 夹爪在半空就合上了（夹空 / 没夹到瓶子） | 两个确认关掉后，闭合时刻只由 `descend_dwell` 决定，而下降**不判是否到位** | 先看那行 `DESCEND 下降指令已发出 N s (不判到位)` 的「实际」栏：还在动就是给少了。**调大 `descend_dwell`**（§5.11），不要改回 `grasp_confirm_required=true`——那会连带把整条链又变回逐步确认 |
| 悬停在目标上方不动，目标动了也不跟 | 目标移动没超过死区 / 距上一条指令不到一个周期 / `track_id` 换了 | 看 `悬停跟随中 (...)` 那行提示（带「候选 X s 前更新 / 周期 Y s」）；死区是 `track_deadband`，周期是 `track_min_command_interval`（§5.10） |
| 跟随跟不上被推动的瓶子 | 跟随周期太长（现在 1s 一跳，最小跟随速度 = 死区/周期 ≈ 10mm/s） | 调小 `track_min_command_interval` 或调小死区；这是离散跟随的固有代价（§5.10） |
| 跟随期间机械臂小幅抖动 | 死区或停稳门太小 | 调大 `track_deadband` / `track_settle_time`，或调大周期；别去调 `track_max_lag`（§5.10） |
| `任务失败: TRACK_TARGET_LOST ...` | 跟随期间候选超过 `max_result_age` 没更新（感知停了/目标被长期遮挡） | 先查感知为什么停（`ros2 topic hz /bottle_grasp_perception/candidates`）。想在这一刻还是抓下去，可以在判失败前按 Enter 走原地垂直下降；嫌太敏感就把 `max_result_age` 调大（§5.10） |
| 跟随刚进入就判 `TRACK_TARGET_LOST` | `track_min_command_interval` 被调得 ≥ `max_result_age` | 让 `max_result_age` 明显大于周期（现在 6.0 / 1.0），否则还没读到下一次坐标就先判失败（§5.10） |
| 悬停时跟丢了目标 | 上游 IoU 跟踪器换了 `track_id`（移动过快/被遮挡/出视野） | 判失败回待机位，重新触发一次抓取；需要更长跟随得换跟踪器（§5.10） |
| 确认下降后夹爪先横着挪一段再往下 | 目标刚动过而机械臂还没跟上 | 正常：横移和 12cm 下降合并成一条绝对位姿指令一次走完（§5.10 / §5.11） |
| `候选已过期 X s, 从当前悬停点原地垂直下降` | 确认时感知已经停了 | 安全行为，不是错误；先查感知为什么停了（§5.10） |
| **没按 Enter 机械臂自己下降了** | 正常：`auto_grasp_stable_time` 默认 `3.0`，悬停到位后物体位置连续 3s 不变就自动降（§5.10） | 要人工把关就 `ros2 param set /bottle_grasp_executor auto_grasp_stable_time 0.0`（逐次读取，立即生效，不用重启） |
| 悬停到位了、一直不自动下降 | 看那行 2s 一拍的提示：`等机械臂到位并停稳` / `候选已 X s 未更新, 窗口作废` / `物体位置还在变` / `自动下降已关` | 三种门对应三种修法（§5.10）：机械臂没停稳 → 调 `track_settle_time`；候选不新鲜 → 查感知；位置一直在变 → 查 `auto_grasp_stable_epsilon` 是不是被调到了 5mm 以下 |
| 调 `auto_grasp_stable_epsilon` 调小之后**再也不自动下降** | 静止物体的 `position_base` 峰峰值就有 3~4 mm，容差小于它等于永远判定「还在变」 | 回到 0.010（不要低于 0.005，`ConfigConsistencyTest` 会红）；这条判据本来就不该比感知噪声更细（§5.10） |
| 悬停时人手扶着瓶子、松手瞬间机械臂就往下抓 | 判据区分不了「瓶子自己不动」和「人手扶着不动」 | 设计如此，判据只看「位置是否连续 3s 不变」。要人盯就设 `auto_grasp_stable_time=0`（§5.10） |
| 干跑（`dry_run`）里自动下降会不会不触发 | 不会：干跑可能收不到 `/feedback/tcp_pose`，所以「机械臂到位」这道门在干跑下整道跳过 | 无需处理。反过来说**干跑时它触发得快**（少一道门），别拿干跑的耗时当现场耗时（§5.10） |
| 触发被拒，日志有 `place_pose_reported_mm_deg` / `WORKSPACE_REJECTED` / `需要 6 个有限数` / `离基座原点只有 ...` | `place_enabled=true` 但放置位姿没填好（占位值全零、位置按米填、写成 3 个数的旧约定、或超出 `workspace_*`） | 修 `place_pose_reported_mm_deg`（上报末端原点系 / mm+度，见 §5.12），或先把 `place_enabled` 置 `false` 跑老流程。**只在加载期读一次，改完要重启节点** |
| 放到一半发现压着台面 / 一直往下顶 | `place_pose_reported_mm_deg` 的 z 填低了 | 把 z 调高。放置段现在**判到位、会迭代末端修正**（上限 `max_correction_distance` 8cm），压住之后它会顶着残差迭代到上限为止 —— 慢，但不会像旧盲降那样一路往下冲。`PLACE_WAIT` 那行日志的「实际 vs 目标」残差就是它顶了多少（§5.12）。**调试 z 的时候把 `place_confirm_required` 打开**，不然张开的那一刻没人能拦 |
| 物体被歪着放下去 | 放置姿态被改成了「读当前 TCP 姿态」 | 正常实现下不会发生：整段用一个常量姿态（`place_pose_reported_mm_deg` 的 RX/RY/RZ，加载期算好），`test_place.py` 有专门用例。若真出现，查是不是又按「每段读当前 TCP 姿态」改了 |
| 日志出现 `固件 IK 拒收 (ik_status=2 (超出工作空间))`，机械臂一步没动（`PLACE_MOVE` 或 `HOME`） | **被拒的是那条指令的位姿本身**：这个位置配上这个姿态固件解不出来 —— 要么是节点自己插出来的中间点（回位段 2026-09-21 起已不再插点，§5.7 第 8 条），要么是填错的放置位姿 / 待机位 | **第一步先问「这个位姿本身在那个朝向下可达吗」**，别当成超时或容差问题：`workspace_*` 在界内**不等于**固件收得下（§5.12 那段凸性）。中间点那种已经删掉了；参数那种要用手把机械臂挪过去、按 GUI 抄那 6 个数，再用 `scripts/check_place_pose.py` 单独试（§5.12）。**重发同样的数救不回来** —— 姿态层面的不可达是系统性的，不是随机的，节点留的那次「原样重发一次」只能救「换个关节构型就收下」的情况。**判决要配上「机械臂一步没动」才作数** —— 固件心跳里的非 0 值是粘住的，只看到 `IK 状态变化: status=2` 不等于这次被拒。放置段顺带看那行日志的 `全程同时转腕 N°`（超 90° 会另打一条 WARN） |
| 日志出现 `固件 IK 拒收`，但 `h3001_bridge` 那边一条 `IK 状态变化:` 都没有 | **改过 `H3001ArmStatus.msg` 之后桥没重启**：它按旧字段布局在发，订阅端拿新定义解不出来，静默丢一条不报 | 重启 `h3001_bridge`。判据：`ros2 topic hz /feedback/tcp_pose` 与 `/feedback/joint_states` 照常 100Hz，只有 `/feedback/arm_status` 是 0（消息按字段布局序列化，rmw 只比类型名）。执行节点同理要重启 —— 否则它永远收不到臂状态，**而且不报错** |
| 下降的路径不是直线（或要确认 `move_l` 到底有没有生效） | 桥的 `_on_move_l` 用的是厂商 `send_linear_pose`（**先发位姿、再切 L 模式**），这个入口时序**没有被真机证实**过 —— 读代码判断不了 | 读 `/feedback/arm_status` 的 `mode_feedback`：下降过程中应为 **2**（码表 `2=L`，见 `h3001_web/protocol_ui.js`）。**仍是 0 就是 L 没生效**（实际按 MOVE_P 走的），把桥的 `_on_move_l` 换成 mode-first（照 `_on_move_p`：`enter_can_mode(MoveMode.L)` → `sleep(0.02)` → `send_target_pose`）再测**同一目标**。注意 `move_l` 只有 `DESCEND` 一段用（2026-09-28 起）—— 别拿其它阶段的日志判断它没生效（§5.11） |
| 抬升走 `move_p`，瓶子/夹爪是不是会斜着扫过去蹭到台面 | 会，这是 09-28 退回 P 认下的代价 | **别把 LIFT 塞回 `_LINEAR_MOTION_STATES`** —— 09-28 实测它换来的是「5/7 次抬升被固件 `ik_status=5 关节单步过大` 拒收、根本抬不起来」。真要躲这个扫掠，改用「先一条短 `move_p` 起步、再走 L」的两步写法（§5.11） |
| 下降途中夹爪在半空合上（夹个空） | `descend_dwell`（2.0s）是照 **9cm** 定的量，而现在的行程是 **20cm**（`pre_grasp_offset_z: 0.2`），且 2026-09-24 起下降改走 L、轨迹的耗时曲线也变了 —— **这个组合从没实测过** | 调大 `descend_dwell`（逐次读取：`ros2 param set /bottle_grasp_executor descend_dwell 4.0`），**不要**改回 `grasp_confirm_required: true`。判据是 `DESCEND 下降指令已发出 …` 那行的「实际」栏在闭合前是否还在移动（§5.11） |
| 下降那条指令被 IK 拒收，重发之后模式好像变了 | 重发用的是 `_last_command_linear`（**那条指令当初的模式**），不是当前状态 —— 判决是异步结案的 | 正常情况下不会变：`MoveModeRoutingTest` 钉着「重发保持原模式」。日志里那句 `固件拒收了这一条指令, 原样重发一次 (move_l, ...)` 直接写着模式，对不上就是真 bug（§5.11） |
| 放置那一段也想走直线 | `_LINEAR_MOTION_STATES` 里只有 `DESCEND` —— `PLACE_MOVE` 名字叫「直线直达」但发的是 `move_p`（**点到点**），这是已知的、2026-09-24 明确选择不动的 | 要改就把它加进 `_LINEAR_MOTION_STATES`（唯一开关，调用点不用动），但**先想清楚 L 多一种失败模式**：整条线段逐点可解才走，中间不可解会被拒（§5.11） |
| 机械臂停在瓶子旁边约 8cm 处不动 | 位置填的是**夹爪中心**（旧约定），而参数收的是**上报原点** | `place_pose_reported_mm_deg` 要填 GUI 上那 6 个数（上报原点系）。旧转新：`上报 = 夹爪 - R @ gripper_offset_tool_xyz`；点差 8cm 量级时查 `gripper_offset_tool_xyz` 本身（§5.6） |
| 末端总是差几毫米 | 重力前馈缺失 | §5.4 / §6.4 |
| `正在回待机位, 拒绝新请求` | 正常：`HOME` 还没走完 | 等 `状态: HOME -> IDLE` 再触发。远距离回位的到位超时最多放大到 4×`motion_timeout`，等得久一点是预期的（§5.9） |
| 启动报 `待机位不可用, 回位功能已禁用` | `standby_pose_reported_mm_deg` 长度/单位/工作空间校验没过 | 看同一行的 `WORKSPACE_REJECTED` 详情；抓取不受影响，但不会回位（§5.7） |
| 启动报 `Sequence should be of same type` / `InvalidParameterTypeException` | 那 6 个数没全写成浮点 | 全部加小数点（§5.7 第 6 条） |
| 待机位换算后离基座原点只有几十毫米 | 位置误按米填（`0.078` 而不是 `78`） | 该 WARN 会直接打出来；改成 mm（§5.7） |
| `HOME` 运动超时，但日志显示只差几厘米 | 别的姿态下重力下垂比抓取姿态大 | 先调大 `standby_tolerance`，不要去追位姿（§5.4） |
| 回位日志出现 `回位直线 X cm 超过单条指令上限 50 cm: 回位段豁免这个上限` | 回位里程超过 `max_command_distance`（0.50 m）—— **正常**：待机位是加载期校验过的 yaml 常量、不是感知，与放置段同一个豁免（2026-09-21） | **不是错误**，整段一条 `move_p` 照发（§5.7 第 8 条）。这行 WARN 是故意留的：豁免掉上限等于少了原来「分段顺便看一眼里程」那层兜底，剩的证据只有同一行的「起点」与 `_enter_home` 开头那次反馈检查 —— 起点看着不对就先人工确认机械臂真实位置 |
| `回位未完成, 机械臂停在非待机位` | 回位自己失败了（够不到 / 超时 / 固件拒收待机位那条位姿，见上面 `固件 IK 拒收` 那行） | 机械臂停在半路，确认位置后重新触发；不会锁死。**反馈丢失不该再走到这里**（2026-09-16 起转盲走，§5.9） |
| `盲走回位结束 ... 无法确认是否到位` | 回位时机械臂不可控，指令发出去了但没人确认 | 先查 `ros2 topic info /control/move_p -v` 有没有订阅者（bridge 死没死），再人工确认位置（§5.9） |
| `任务失败: ROBOT_FAULT err_status=N` | 某个电机报了错 | `N` 的位 `i` 表示 J(i+1) 的 `motor_error_code`（bit0-5 是 `motor_fault`）；用厂商 UI「电机状态 → 节点/轴 → 指定轴清错」清（`joint = (node-1)*3+axis`）。清错前先停 bridge 让 web server 独占 `can0` |
| 日志里 `夹爪脚本: [stop] 一路合到底…` | 阻力阈值对当前物体太钝（或两指之间本来就没东西）。**不判失败，照常抬升** | 想夹得更「有感觉」就调小 `gripper_resistance_torque`（默认 0.3）；不调也能跑（§5.13） |
| 日志里 `夹爪脚本: [warn] 扭矩冲到硬上限…` | 消抖窗口还没成立就撞上了硬扭矩上限 —— 多半夹到硬东西或位置不对。**不判失败**，按「已夹住」停下 | 先看夹的位置对不对；这是 2026-09-18 那次事故的修复点（§5.13） |
| `夹爪动作失败: 夹爪脚本异常退出 (退出码 N)` | 闭合/张开根本没做，看上面 `[abort]` 那行 | 退出码 2 基本就是串口被占：厂商 web server / 另一个终端里的 04、05 |
| `夹爪动作失败: … 超过 30s 还没退出` | 脚本卡住或串口发不出帧 | 查串口是否被占：厂商 web server / 另一个终端里的 04、05。日志里往前找 `[abort]` 那行（节点把脚本的 stderr 一起收进来了） |
| `启动夹爪脚本` 之后一行 `[abort] Cannot open port …` | 串口打不开 | 同一条：`/dev/ttyACM0` 只有一个占用者。`ls /dev/ttyACM*` 确认适配器还在（§5.13） |
| `gripper_enabled=true 但夹爪不可用 … 拒绝启动` | 脚本没找到或串口不存在 | 看启动日志里的 `找不到夹爪脚本` / `gripper_port=… 不存在`。新克隆的工作空间要先 `colcon build --symlink-install` 一次，`scripts/` 才会出现在 `install/` 里 |
| `gripper_enabled=false` 改了但行为没变 | 它是**只读一次**的参数 | `ros2 param set` 对它无效，必须重启节点（同 `place_enabled`） |
| 夹爪夹住了但抬升时瓶子掉了 | `gripper_hold_torque=0`：闭合命令跑完脚本就失能退出了，抬升期间没有任何东西在维持夹持力，靠的是**没实测过**的机构自锁（§5.13 那个 ⚠） | 把 `gripper_hold_torque` 开起来（现场是 1.0），抬升/搬运全程就由 MIT 力矩顶着 —— 不要靠调 `resistance_torque` |
| 日志里 `夹爪保持进程已退出 (退出码 N)` | 搬运途中那个常驻进程死了 → 夹持力没了，**瓶子可能已经掉了**。节点立刻判失败并回位 | 现场看一眼瓶子在哪。常见死因：串口被拔 / CAN 抖动、`gripper_hold_timeout` 到点（日志里在那之前会有 `[warn] 保持已到 --hold-timeout`） |
| 日志里 `已向保持中的夹爪进程写 OPEN, 但 Ns 内没退出` | 写进去了但脚本没在超时内退出：卡在切回 PV 模式，或张开本身失败 | 看脚本 stdout 上那几行；`gripper_timeout`（40s）就是这条的预算 |
| 重启节点后第一次夹爪动作 `[abort] Cannot open port` | 上一轮那个**保持进程还活着**，攥着 `/dev/ttyACM0` | `pgrep -f gripper_grip_hold.py` 找到它，`kill` 掉（它会先松开再退；也可以等 `gripper_hold_timeout` 到点）。正常收工不会这样 —— 节点退出时会写 OPEN 并等它退 |
| Ctrl+C 之后夹爪立刻松了、瓶子掉了 | 2026-09-28 定的「失败/退出立刻松开」：`destroy_node` 会写 OPEN，悬空时就是会掉 | 这是**配置行为**不是 bug。要改成「悬空时不松手」得动节点的策略（把 `destroy_node` 那段改成只让脚本自己顶到 `hold_timeout`），不是调参数 |
| `gripper_hold_torque>0 但 place_enabled=false … 拒绝启动` | 没有放置点 = 整条链**没有任何一步会去松开夹爪**，保持进程会顶到 `hold_timeout` 然后在待机位上自己松手掉瓶子 | 补好放置点，或把 `gripper_hold_torque` 置 0（§5.13） |

---

## 8. 相关文档

- 感知包（YOLO / 点云 / 手眼变换 / ROS 感知节点）：[`src/wutishibiezitaijiance-master/README.md`](../wutishibiezitaijiance-master/README.md)
- 桥接节点接口与参数：`src/h3001_bridge/launch/h3001_bridge.launch.py` 头部注释
- 执行器状态机与全部参数说明：`config/bottle_grasp_executor.yaml` 注释
- H3001 SDK（当前）：`src/SDK_linux/sdk_linux.py`
- H3001 SDK（旧版，回退候选）：`src/sdk_can/h3001sdk/h3001_sdk.py`
- 手眼标定工程与标定流程：`/home/qin/hand_eye_calibrate/`
- 项目总目录索引：[`README.md`](../../README.md)
