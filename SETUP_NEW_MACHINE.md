# H3001 瓶子抓取链路 —— 新机器部署清单

## 0.一键启动
source /opt/ros/humble/setup.bash && source install/setup.bash
/usr/bin/python3 h3001_console.py

（控制台上还有个「调参」按钮，打开 `h3001_tuner.py` —— 抓取参数的图形界面，改的是
`bottle_grasp_executor/config/bottle_grasp_executor.yaml`。它**不下发**：改完要在控制台点
「起/停 执行器」重启才生效。见 `bottle_grasp_executor/README_参数配置.md` 开头。）

---

## 1. 这个包里有什么

| 目录 | 是什么 | 必需 |
| --- | --- | --- |
| `bottle_grasp_executor/` | 抓取执行状态机（ROS 2 包） | ✅ |
| `h3001_bridge/` | CAN ↔ ROS 2 桥（ROS 2 包） | ✅ 真机必需 |
| `h3001_msgs/` | 机械臂状态反馈消息（ROS 2 包） | ✅ bridge 依赖 |
| `SDK_linux/` | H3001 CAN SDK（`sdk_linux.py`）+ 厂商 web server | ✅ bridge 靠它通信 |
| `wutishibiezitaijiance-master/` | 感知：YOLO 分割 + 抓取位姿估计 | ✅ |
| `wutishibiezitaijiance-master/ros2_ws/src/bottle_grasp_interfaces/` | 抓取候选 msg/srv（ROS 2 包） | ✅ executor 编译依赖 |
| `Gloria-M-SDK/` | 夹爪 SDK（`gloria_m_sdk`） | ✅ `gripper_enabled: true` 时必需 |
| `hand_eye_calibrate/` | 手眼标定工具 + 标定结果 | ⚠️ 工具要，标定结果见 §7 |
| `requirements-ros.txt` / `requirements-perception.txt` | 依赖清单（本文档同目录） | ✅ |

**三块是分开的，装法各不相同**：ROS 包走 colcon，感知走 conda，夹爪 SDK 只是加进 `sys.path`。

---

## 2. 前置条件

### 硬件

- H3001 六轴臂 + CAN 适配器（开发机是 gs_usb USB-CAN）
- Orbbec 深度相机（USB）
- Gloria-M 夹爪（串口，开发机是 `/dev/ttyACM0`）

### 软件

| 项 | 版本 | 说明 |
| --- | --- | --- |
| Ubuntu | 22.04 | ROS 2 Humble 的对应版本 |
| ROS 2 | **Humble** | `source /opt/ros/humble/setup.bash` |
| 系统 Python | 3.10 | ROS 节点跑在这个上 |
| conda | miniforge/miniconda | 感知跑在独立环境（Python 3.12） |
| OrbbecSDK | **v2.9.3** | 装到 `/opt/OrbbecSDK_v2.9.3/`，供 pyorbbecsdk 用 |

```bash
sudo apt install ros-humble-desktop python3-pip can-utils
```

---

## 3. 目录摆放

建议照搬源机的结构（`src/` 这一层要保留，colcon 扫起来最省事）：

```
~/zdl_fz/
├── src/
│   ├── bottle_grasp_executor/
│   ├── h3001_bridge/
│   ├── h3001_msgs/
│   ├── SDK_linux/
│   └── wutishibiezitaijiance-master/     ← 整个放进来
│       └── ros2_ws/src/
│           ├── bottle_grasp_interfaces/  ← executor 编译时需要
│           └── bottle_grasp_perception/  （当前 candidates_source: file，没在用）
└── (build/ install/ 由 colcon 生成)
```

`bottle_grasp_interfaces` 虽然藏在 `wutishibiezitaijiance-master/ros2_ws/src/` 里，
但 colcon 是从工作空间根**递归**找 `package.xml` 的，所以放在 `src/` 下就能一起编出来。
**不要**把它挪到别处，否则 executor 编译报找不到消息包。

`Gloria-M-SDK/` 和 `hand_eye_calibrate/` 放哪都行，它们不是 ROS 包，只按路径引用。

---

## 4. 装依赖

### 4.1 ROS 侧（系统 Python 3.10）

```bash
/usr/bin/python3 -m pip install -r requirements-ros.txt
```

### 4.2 感知侧（conda，Python 3.12）

```bash
conda create -n grasp python=3.12 -y
conda activate grasp
pip install -r requirements-perception.txt \
  -i https://pypi.tuna.tsinghua.edu.cn/simple
# pyorbbecsdk2 要单独装 —— 不在 PyPI 上, 从 Orbbec 官方拿对应版本/架构的 .whl
pip install pyorbbecsdk2 -i https://pypi.tuna.tsinghua.edu.cn/simple
```

### 4.3 夹爪 SDK 的 python 依赖

```bash
/usr/bin/python3 -m pip install -r Gloria-M-SDK/Gloria-M-SDK/requirements.txt
```

---

## 5. 三个路径要改（代码里写死了源机路径）

新机器如果用户名不是 `qin`，或目录不在 `/home/qin/zdl_fz`，下面三处**全都**要处理：

| # | 位置 | 源机值 | 怎么改 |
| --- | --- | --- | --- |
| 1 | `SDK_linux` 位置 | 环境变量优先级最高 | `export H3001_SDK_PATH=<新路径>/SDK_linux` |
| 2 | `Gloria-M-SDK/src` | 同上 | `export GLORIA_M_SDK_SRC=<新路径>/Gloria-M-SDK/Gloria-M-SDK/src` |
| 3 | 手眼标定 yaml | `bottle_grasp_executor/config/bottle_grasp_executor.yaml:21` | 直接改 yaml 里的 `hand_eye_yaml_path` |

建议把前两个写进 `~/.bashrc`。

> 第 1 项有个坑：`h3001_bridge.py` 的候选路径表里**有多份不同版本的 sdk_linux.py**。
> 配置错了不会报错，会静默加载另一份、`send_target_pose` 语义不同。启动日志第一行会打
> 出实际加载的文件路径，**每次启动都看一眼**。

---

## 6. CAN 与构建

### 6.1 起 can0

```bash
sudo ip link set can0 up type can bitrate 1000000
ip -details link show can0          # 确认 up + bitrate 1000000
```

接口名和波特率必须与 `h3001_bridge/config/h3001_bridge.yaml` 里的
`can_channel` / `can_bitrate` 一致。

### 6.2 构建

```bash
conda deactivate                    # ← 必须! 否则 rosidl 报 No module named 'em'
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to h3001_bridge bottle_grasp_executor
```

> **`conda deactivate` 不能省。** conda 的 Python 会污染 CMake 缓存，报
> `No module named 'em'`；一旦缓存被写坏，光重新 build 不够，要
> `rm -rf build/ install/` 再来。

两个包都是纯 Python，`--symlink-install` 之后改 `.py` 不用重编；但新增文件、
`config/*.yaml`、`launch/*.py` **仍要重新 build**。

---

## 7. ⚠️ 必须在本机重做的部分

**这一节是换机器最容易翻车的地方。**

### 7.1 手眼标定：通常必须重做

`hand_eye_extrinsics.yaml` 描述的是**相机与机械臂基座的相对位姿**。

- 相机和机械臂**一起搬、相对位置没动** → 可以直接用 `hand_eye_calibrate/H3003/` 里那份
- 相机或机械臂**挪过位置、重新固定过** → **必须重标**，拷过去算出的 base_link 坐标会整体偏

重标工具就在 `hand_eye_calibrate/` 里，流程见该目录的 `README.md`。

标完确认两侧指向**同一次标定**（两边必须同源，否则坐标对不上）：

```bash
# executor 侧
grep hand_eye_yaml_path bottle_grasp_executor/config/bottle_grasp_executor.yaml
# 感知侧默认值 (hand_eye_transform.py 的 DEFAULT_HAND_EYE_PATH)
# 命令行可用 --hand-eye 覆盖
```

### 7.2 示教点位：必须重新示教

下面这几组数是**现场示教出来的**，跟这台机器的安装位置强相关，拷过去一定是错的：

| 参数（`config/bottle_grasp_executor.yaml`） | 含义 |
| --- | --- |
| `standby_pose_reported_mm_deg` | 待机位 6 个数 |
| `place_pose_reported_mm_deg` | 放置位姿 6 个数 |
| `side_orientation_xyzw` | 侧抓姿态四元数 |
| 感知的 `--offset x y z` | 基座系固定偏置（米），实测调出来的 |

`gripper_offset_tool_xyz: [0.0, 0.0, 0.06]` **可以**照抄 —— 那是夹爪的几何尺寸，
与安装位置无关。

> `side_orientation_xyzw` 与 bridge 的 `invert_rx_ry` 开关**必须自洽**：
> 那个四元数是从**未取反**的 bridge 读出来的。要打开 `invert_rx_ry`，
> 就得同时把四元数的 x、y 取反，否则姿态会偏十几度。
> 详见 `h3001_bridge/config/h3001_bridge.yaml` 里那段注释。

### 7.3 先跑 dry_run

真机之前**一定**先用 `dry_run:=true`（默认值）跑一遍，核对日志里这五行：

```text
待机位: 上报原点=[...]     ← 必须等于 standby_pose_reported_mm_deg 的前三个数换米
抓取点(夹爪, base)=...     ← 应该是感知给的位置
夹爪偏移=...               ← 模长 = |gripper_offset_tool_xyz| = 0.06 m
上报原点=...               ← 真正下发的点 = 抓取点 - 夹爪偏移
预抓取(夹爪)=...           ← 抓取点抬高 pre_grasp_offset_z
```

---

## 8. 运行顺序（四个终端，不能颠倒）

```bash
# 每个终端都要先 source
source /opt/ros/humble/setup.bash
source install/setup.bash
```
cd SDK_linux
./run_h3001_web_linux.sh 

**A. 桥接**

```bash
export H3001_SDK_PATH=/home/zdl/H3001_grasp/SDK_linux
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch h3001_bridge h3001_bridge.launch.py

```

> `speed_percent` 的 **launch 默认值（10）会覆盖 yaml 里的同名参数**，改 yaml 没用，
> 要从命令行给。启动行会打 `速度=N%`，跑之前先确认。
> 想只读反馈不使能电机：加 `auto_enable:=false`。

**B. 感知**

```bash
cd wutishibiezitaijiance-master
conda activate grasp
LD_LIBRARY_PATH=/opt/OrbbecSDK_v2.9.3/lib /home/zdl/miniconda3/envs/grasp/bin/python \
  test_yolo_realsense_v3f.py --camera orbbec \
  --json-out /tmp/bottle_grasp_candidates.json \
  --no-point-cloud \--cylinder-axis \
  --tracker ocsort --offset 0 0 -0.02 
```

> **不要启动 `orbbec_camera` ROS 节点。** 感知脚本直接调 Orbbec SDK 抓 USB 设备，
> 再起一个 ROS 相机节点会两个进程抢同一个相机。

**C. 执行器**

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
conda activate grasp
ros2 launch bottle_grasp_executor bottle_grasp_executor.launch.py dry_run:=false # 真机
```

**D. 触发**

```bash
ros2 service call /bottle_grasp_executor/execute_grasp std_srvs/srv/Trigger
```

之后状态机自动走 `WAIT_TARGET → PRE_GRASP → TRACK_WAIT`，**停下等人工确认**
（现场配置下整条链只有这一次确认：`grasp_confirm_required` / `place_confirm_required`
都关掉了）。

---

## 9. 排错速查

| 现象 | 原因 | 处理 |
| --- | --- | --- |
| `No module named 'em'` | conda 污染了 CMake 缓存 | `conda deactivate` + `rm -rf build/ install/` 重来 |
| `无法导入 sdk_linux` | 路径不对 | `export H3001_SDK_PATH=...`；看启动日志确认加载的是哪份 |
| bridge 收不全帧 | 厂商 web server 还在跑 | `pkill -f h3001_web_server_linux.py` |
| `import pyorbbecsdk` 失败 | 缺 Orbbec 系统库 | 确认 `/opt/OrbbecSDK_v2.9.3/lib` 在 `LD_LIBRARY_PATH` 里 |
| 夹爪不动 | 串口被占 / 路径不对 | 检查 `/dev/ttyACM0`；确认 `GLORIA_M_SDK_SRC` |
| 夹爪**在半空合上**（夹个空） | 下降还没走完就到点了 | **调大 `descend_dwell`**，不是把 `grasp_confirm_required` 改回 true |
| 机械臂到位后姿态偏十几度 | `invert_rx_ry` 与 `side_orientation_xyzw` 不自洽 | 见 §7.2 的说明 |
| 坐标整体偏 | 手眼标定没重做 | 见 §7.1 |

---

## 10. 没打进包里的东西

以下**不在** `H3001_grasp/` 里，需要在新机器单独准备：

| 项 | 说明 |
| --- | --- |
| OrbbecSDK 系统库 | 装 v2.9.3 到 `/opt/`，没法打包 |
| conda 环境本身 | 照 §4.2 重建 |
| `pyorbbecsdk2` 的 whl | Orbbec 官方仓库下载，不在 PyPI |
| ROS 2 Humble | apt 装 |
| can0 / gs_usb 内核支持 | 见 §6.1 |

另外，源机上另有一份**排除区** `/home/qin/H3001_grasp_excluded/`，装着从包里拿出去、
但源机仍保留的东西（历史标定、旧相机 ROS 包、`.git` 等）。新机器**不需要**它们。

---

## 附：本包体积变化

349M → **67M**（移出 136M 的 `.git`、15M 的标定压缩包、其他机器的历史标定、
colcon 生成物、`__pycache__`）。清单里的 7 个目录内容与源机逐字节一致，未做任何代码改动。
