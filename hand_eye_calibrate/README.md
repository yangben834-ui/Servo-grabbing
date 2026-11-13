# hand_eye_calibrate

## 一键启动
python3 hand_eye_calib_ui.py


该项目可以进行机器人的手眼标定，支持常见的 eye-in-hand、eye-to-hand 以及 head-camera 采集场景。机械臂位姿使用的是末端法兰位姿，有需求就要添加 TCP 位姿和 offset

- `hand_eye_calibrate.py`：计算脚本，可根据采集的数据计算相机内参与手眼转换关系矩阵，并将结果保存为 YAML 文件。
- `robot_arm.py`：机械臂适配层。统一 **Piper / H3001 的连接、单位换算**和位姿合理性自检。
- `h3001_sdk.py`：H3001 的 CAN/AGX 协议 SDK（厂商侧，未改动）。
- `check_calib_consistency.py`：采集数据自洽性自检，标定前先跑。
- `data_collect.py` / `data_collect_orbbec_ros.py`：RealSense / Orbbec 采集脚本。
- `validate_calibration.py`、`verify_target_flange.py`：标定结果检验（见文末状态）。

## 切换机械臂

支持 **H3001** 和 **Piper（松灵 AgileX）** 两种，通过 `--robot` 参数选择，
**不需要改代码**。默认 `h3001`。

```bash
# H3001（默认）
python3 data_collect.py --mode eye_to_hand --data-dir ./my_calibration

# 换回松灵 Piper
python3 data_collect.py --robot piper --mode eye_to_hand --data-dir ./my_calibration
```

`data_collect_orbbec_ros.py` 同样支持 `--robot` / `--no-can-fd`。

> **不要用 `git checkout` 来切换机械臂。** 工作区的 `create_robot()` 重构是未提交改动，
> `git checkout` 会退回到更早的扁平写法，把这些改动一起冲掉。用 `--robot` 切换。

## 机械臂：H3001

### 单位与角度约定（重要）

`h3001_sdk.py` 的 `get_flange_pose()` 返回的是 **毫米和度**，而
`hand_eye_calibrate.py` 要求 **米和弧度**。换算统一由 `robot_arm.py` 完成：

| 量 | SDK 输出 | 换算后 | 依据 |
|---|---|---|---|
| x, y, z | mm | m（`×1e-3`） | 协议原始单位 0.001mm |
| rx, ry, rz | 度 | 弧度（`×DEG2RAD`） | 协议原始单位 0.001 度 |

欧拉角约定为 `R = Rz(yaw)·Ry(pitch)·Rx(roll)`，与 `hand_eye_calibrate.py:55` 一致，
不需要做任何顺序调整。

> 依据来源：`pyAgxArm` 解析同一套 AGX 协议的同一批反馈帧
> （`pyAgxArm/protocols/can_protocol/drivers/piper/default/parser.py:180-205`）。
> **这是从协议族推断的，第一次接新臂时必须按下面的预检流程实测确认。**

### 1. 连接前预检（务必先做）

```bash
cd /home/qin/hand_eye_calibrate
python3 -c "
import robot_arm as ra
r = ra.create_robot('h3001', fd=True)     # 经典 CAN 适配器改成 fd=False
print('原始反馈:', r.get_flange_pose())
print('换算后  :', ra.describe_pose(ra.read_flange_pose(r)))
r.close()"
```

把"换算后"那行（米 + 度）跟示教器读数对照。**对上了再采集。**

`read_flange_pose()` 内置了合理性检查：臂展不在 0.02~2.0 m，或角度超过
±π / pitch 超过 ±π/2，都会直接报错而不是静默采出废数据。

### 2. CAN 配置

默认沿用 `h3001_sdk` 的 CAN-FD 配置（1 Mbps / 5 Mbps）。若现场是经典 CAN
适配器（如原来 Piper 那套）：

```bash
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 up
```

并把 `create_robot(fd=False)`。连不上时先试这个。

### 3. 采集后先自检

**采 5 张就先跑一次**，均值 < 2° 再继续采满：

```bash
/usr/bin/python3 check_calib_consistency.py <数据目录>
```

> 注意：该脚本只从**欧拉角列**算相对旋转角，**不校验位置列**。位置被改坏它看不出来。

## 相机采集

### RealSense

```bash
python3 data_collect.py --mode eye_to_hand --data-dir ./collect_data_test
python3 data_collect.py --mode eye_in_hand --data-dir ./collect_data_test
```

### Orbbec（ROS2）

先启动相机节点：

```bash
source /opt/ros/humble/setup.bash
source ~/orbbec_ws/install/setup.bash
ros2 launch orbbec_camera gemini_330_series.launch.py \
  depth_registration:=true \
  align_target_stream:=COLOR \
  enable_point_cloud:=false
```

再启动采集脚本：

```bash
source /opt/ros/humble/setup.bash
cd hand_eye_calibrate
/usr/bin/python3 data_collect_orbbec_ros.py \
  --mode eye_to_hand \
  --data-dir ./H3003
```
# 事后任何时候审计
/usr/bin/python3 calib_drift.py ./H3003
## 手眼标定

```bash
conda deactivate
source /opt/ros/humble/setup.bash
cd hand_eye_calibrate
/usr/bin/python3 hand_eye_calibrate.py \
  --mode eye_to_hand \
  --data-dir ./H3003
```

内外参检验：
/usr/bin/python3
```bash
cd hand_eye_calibrate
python3 validate_calibration.py \
  --calib-dir ./H3003 --validate-dir ./H3003 --mode eye_to_hand
```

## 换机械臂时的注意事项

- **基座系不同不影响标定。** 求解的是 `camera_to_base`，基座系变了只是结果换个坐标系表达
  （`X' = X·T⁻¹`），喂进采集到的位姿直接标即可，**不需要对 `poses.txt` 做任何手工变换**。
- 只有在**复用旧机械臂的 `hand_eye_extrinsics.yaml`** 时才需要转换，且必须是完整刚体变换：
  若两基座系相差绕 Z 转 180°，则位置取反 x、y **且 `rz += 180°`**（`rx`、`ry` 不变）。
  只改位置不改 `rz`，得到的是混合坐标系，比不改更糟。
- 换臂后旧数据不可比，需重新采集。eye_to_hand 模式下法兰/TCP 的坐标系定义差异会被
  标定板相对法兰的固定变换吸收，但 eye_in_hand 模式会受影响。

## 待适配

- `verify_target_flange.py` 仍基于 `pyAgxArm`（Piper），**在 H3001 上跑会连错驱动**。
  需要用 `robot_arm` 重写其中的 `move_p` / `move_j` / 使能流程后再使用。
- `h3001_sdk.py` 里 `pose` 的 `/1000.0` 解析（`h3001_sdk.py:642-649`）单位有误，
  适配层已在其之上重新换算，未改动 SDK 本身。
