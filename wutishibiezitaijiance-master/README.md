# Bottle RGB-D 抓取姿态估计 Demo

本项目是一个面向瓶类目标（bottle）的 RGB-D 视觉抓取姿态估计原型。当前版本可使用
Intel RealSense 或 Orbbec 相机完成目标实例分割与跟踪，并结合深度图、目标点云 PCA
和桌面法向估计输出相机坐标系下的 6D 抓取位姿。

仓库同时提供独立 Python demo 和 ROS 2 Humble 感知节点。ROS 节点订阅标准 RGB-D
图像与相机内参，发布结构化抓取候选、调试图像、RViz Marker 和可选 TF，并提供读取
最近候选的服务，便于后续集成到 Franka 强化学习系统和真机视觉伺服抓取平台。

## 目录

- [1. 当前功能](#1-当前功能)
- [2. 文件说明](#2-文件说明)
- [3. 算法流程](#3-算法流程)
- [4. 环境与依赖](#4-环境与依赖)
- [5. 运行 Demo](#5-运行-demo)
  - [5.8 把抓取点移到瓶轴上（圆柱拟合）](#58-把抓取点移到瓶轴上圆柱拟合)
- [6. 当前关键参数](#6-当前关键参数)
- [7. ROS 2 感知节点](#7-ros-2-感知节点)
- [8. Franka 与视觉伺服集成要点](#8-franka-与视觉伺服集成要点)
- [9. 建议开发顺序](#9-建议开发顺序)
- [10. 已知限制](#10-已知限制)
- [11. 版本演进记录](#11-版本演进记录)

## 1. 当前功能

已实现：

- 支持 RealSense、Orbbec 两类 RGB-D 相机；
- 输出对齐到彩色图的 BGR 图像和米制深度图；
- 根据 CPU 架构自动选择 PT 或 RKNN 分割模型；
- PT 和 RKNN 共用预处理、NMS、mask 解码、坐标恢复及 IoU 跟踪代码；
- 根据分割 mask 反投影得到目标点云，并过滤深度离群点；
- 根据非目标区域点云，通过 RANSAC 估计桌面法向；
- 使用目标点云 PCA 主轴判断瓶体为立放或平放；
- 可选圆柱拟合：由可见半壳推算瓶半径，把抓取点从表面质心移到瓶轴上（见 5.8）；
- 立放瓶采用侧向抓取，平放瓶采用顶部抓取；
- 输出抓取中心、旋转矩阵、抓取模式及目标跟踪 ID；
- 提供 OpenCV 二维结果显示和可选的 Open3D 点云/夹爪姿态显示；
- 提供不创建任何 GUI 窗口的 headless 模式，适用于 RK3588 板端。

当前算法适用于平行夹爪对单个或多个瓶类目标进行候选抓取姿态估计。输出位姿是几何
候选位姿，尚未包含碰撞检测、可达性判断、逆运动学、轨迹规划和抓取成功率评分。

## 2. 文件说明

- `test_yolo_realsense_v3f.py`
  模块化 demo 入口。负责参数解析、相机循环以及各模块编排。

- `bottle_grasp_pipeline.py`
  单帧 RGB-D 处理管线。封装推理后端与几何处理，并输出结构化的 FrameResult 和
  GraspCandidate；本层不直接访问相机或创建 GUI。

- `yolo_segmentation.py`
  PT/RKNN 统一推理适配层。包含架构识别、模型路径选择、letterbox、置信度过滤、
  NMS、实例 mask 解码、原图坐标恢复和轻量 IoU 跟踪。PT 后端只使用模型原始输出，
  不调用 Ultralytics 自带的检测或分割后处理。

- `grasp_pose_estimator.py`
  独立几何算法层。包含深度反投影、环境点云、桌面 RANSAC、PCA 抓取姿态估计、
  GeometryConfig 和 GraspPose，不依赖相机 SDK、YOLO 或 GUI。

- `grasp_visualization.py`
  可选调试显示层。包含 OpenCV 标注绘制和 Open3D 点云/夹爪显示。

- `test_grasp_pose_estimator.py`
  几何算法离线回归测试，使用合成点云，不需要相机和模型权重。

- `test_yolo_segmentation.py`
  PT/RKNN 统一后处理回归测试，覆盖平台后缀选择、NMS、mask/bbox 对齐、letterbox
  坐标恢复和跟踪 ID 复用。

- `rgbd_camera.py`
  RGB-D 相机统一接口。目前包含 RealSenseCamera、OrbbecCamera 和 create_camera()。
  read() 返回：

  - color：BGR 彩色图，numpy.ndarray，形状为 H x W x 3；
  - depth：与彩色图对齐的深度图，numpy.ndarray，单位为 m；
  - camera.intrinsics：(fx, fy, cx, cy)，对应彩色相机内参。

- `best_train_multi.pt` / `best_train_multi.rknn`
  同一 YOLO 分割模型的 amd64 和 arm64 权重。配置时推荐使用不带后缀的模型基础名
  `best_train_multi`；程序在 amd64/x86_64 上选择 `.pt`，在 arm64/aarch64 上选择
  `.rknn`。两个权重必须放在相同目录并保持基础文件名一致。

## 3. 算法流程

### 3.1 处理流程

```text
RGB-D 相机
    -> 彩色/深度图对齐
    -> 按平台执行 PT/RKNN 原始推理
    -> 统一 NMS、实例 mask 后处理与 IoU 跟踪
    -> 合并目标 mask，提取环境点云
    -> RANSAC 拟合桌面并估计桌面法向
    -> 每个实例 mask 反投影为目标点云
    -> PCA 计算瓶体长轴和几何中心
    -> 判断立放/平放并生成抓取坐标系
    -> 输出抓取位姿和可视化结果
```

### 3.2 抓取坐标系

抓取坐标系当前定义如下：

- X 轴：平行夹爪闭合方向（closing）；
- Y 轴：夹爪手指延伸方向（lateral）；
- Z 轴：夹爪进近方向（approach）；
- 平移：默认为目标点云坐标中值（可见表面质心），单位为 m；开启圆柱轴拟合后
  （`--cylinder-axis` / ROS 参数 `use_cylinder_axis`）改为拟合出的**瓶轴**上的点；
- 旋转矩阵：R = [X Y Z]，三个轴向量按列排列；
- 参考坐标系：相机光学坐标系，通常为 X 向右、Y 向下、Z 向前。

注意：输出的 Z 轴是当前算法定义的“进近方向”。接入具体机械臂末端坐标系时，应根据
夹爪 URDF/标定约定增加一个固定旋转 T_grasp_tool，不能直接假定它等于 Franka 的
tool0 或末端法兰坐标系。

## 4. 环境与依赖

### 4.1 amd64 开发环境

amd64 开发环境：

- Linux；
- Python 3.9 或更高版本；
- NVIDIA GPU + CUDA（可选）；
- RealSense 或 Orbbec RGB-D 相机。

基础 Python 依赖：

```bash
pip install numpy opencv-python ultralytics
```

其中 PT 后端需要 PyTorch 和 Ultralytics。Ultralytics 只负责加载 PT 网络，检测 NMS、
分割 mask 和跟踪均使用本项目的统一后处理。

### 4.2 RK3588 / arm64 板端

- 安装与板端系统及 RKNN 模型兼容的 `rknn_toolkit_lite2`；
- Python 导入名应为 `rknnlite.api`；
- 无显示器环境建议使用 `opencv-python-headless`，不要安装 Open3D；
- 当前 `.rknn` 模型面向 RK3588，模型输入为 640 x 640；
- 板端运行不需要 PyTorch 或 Ultralytics。

板端依赖示意（RKNN wheel 应使用 Rockchip 提供的匹配版本）：

```bash
pip install numpy opencv-python-headless
pip install /path/to/rknn_toolkit_lite2-*.whl
```

### 4.3 可选可视化与相机 SDK

启用三维点云窗口时：

```bash
pip install open3d
```

RealSense：

```bash
pip install pyrealsense2
```

Orbbec：

- 安装 Orbbec SDK 及对应 Python 包 pyorbbecsdk2；
- 程序中的 Python 导入名为 pyorbbecsdk；
- 根据相机型号完成 USB 权限和 udev 规则配置。

实际 CUDA、PyTorch、相机 SDK 的版本应与操作系统、显卡驱动及设备型号匹配。

## 5. 运行 Demo

### 5.1 模型准备与相机选择

先确认两个平台权重位于同一目录：

```text
best_train_multi.pt
best_train_multi.rknn
```

cd /home/qin/zdl_fz/src/wutishibiezitaijiance-master
export LD_LIBRARY_PATH=/home/qin/miniforge3/lib/python3.12/site-packages/pyorbbecsdk:$LD_LIBRARY_PATH
/home/qin/miniforge3/bin/python test_yolo_realsense_v3f.py --camera orbbec --no-point-cloud

使用 Orbbec（默认）：

```bash
python test_yolo_realsense_v3f.py --camera orbbec --no-point-cloud
```

使用 RealSense：

```bash
python test_yolo_realsense_v3f.py --camera realsense
```

### 5.2 显示与 headless 模式

不启动 Open3D 点云窗口：

```bash
python test_yolo_realsense_v3f.py --camera realsense --no-point-cloud
```

RK3588 或其他无桌面环境使用 headless 模式：

```bash
python test_yolo_realsense_v3f.py --camera orbbec \
    --model best_train_multi --headless
```

`--headless` 会禁用 Open3D、`cv2.imshow`、`cv2.waitKey` 和窗口清理调用，但不会关闭
深度点云计算、抓取姿态估计、计时或终端候选输出。使用 Ctrl+C 停止程序。

### 5.3 推理参数

指定模型、置信度和一个或多个类别 ID：

```bash
python test_yolo_realsense_v3f.py --model best_train_multi \
    --confidence 0.25 --image-size 640 --class-id 0 --class-id 1
```

### 5.4 离线测试

运行不依赖相机的算法测试：

```bash
python -m unittest -v test_grasp_pose_estimator.py test_yolo_segmentation.py
```

### 5.5 退出方式与输出字段

图形模式按 ESC 退出；headless 模式使用 Ctrl+C。终端输出示例字段如下：

```text
ID=1 bottle 0.91 XYZ=(0.012,-0.034,0.623)m pose=side \
axis-table=8.5deg normal=(...) r=29.4mm
```

字段含义：

- ID：统一 IoU 跟踪器生成的帧间 ID；
- bottle：类别名称；
- 0.91：分割置信度；
- XYZ：相机光学坐标系下的抓取中心，单位 m；
- pose：side 表示立瓶侧抓，top 表示卧瓶上抓；
- axis-table：瓶体 PCA 长轴与桌面法向的夹角；
- normal：当前抓取坐标系 Z 轴，即进近方向；
- r：拟合出的瓶半径（见 [5.8](#58-把抓取点移到瓶轴上圆柱拟合)），未开启或拟合失败时不打印。

写 JSON 时另外带上两个字段：`position_surface` 始终是可见表面质心，`axis_fit`
记录本次拟合是否成功与诊断量：

```json
"position_surface": [0.012, -0.034, 0.578],
"axis_fit": {"ok": true, "radius": 0.0294, "inlier_ratio": 0.64,
             "arc_deg": 178.2, "reject_reason": ""}
```

`axis_fit.arc_deg` 只作诊断，不作判据：恰好落在内点圆环带里的桌面点会补上方位角
空档，把半壳的 180° 报成 330° 以上。

### 5.6 转换到机械臂基坐标系

Demo 默认用 `--hand-eye` 指向 `/home/qin/hand_eye_calibrate/H3003` 的手眼标定，
额外输出基座坐标系（`base_link`）下的坐标：

```bash
python test_yolo_realsense_v3f.py --camera orbbec --no-point-cloud \
    --hand-eye /home/qin/hand_eye_calibrate/H3003
```

传 `--hand-eye none` 可关闭，只输出相机光学坐标系。标定文件缺失时会打印警告并
退回相机坐标系运行，不中断。终端会追加基座系字段：

```text
ID=1 bottle 0.91 XYZ=(0.012,-0.034,0.623)m pose=side axis-table=8.5deg \
normal=(...) base=(0.350,0.090,-0.023)m
```

`--json-out` 每个候选新增 `position_base` 和 `orientation_base_xyzw`
（四元数 x/y/z/w），顶层新增 `base_frame` 和 `hand_eye`。
**原有 `position` 仍是相机光学坐标系**，`bottle_grasp_executor` 的既有解析不受影响。

手眼变换由 `hand_eye_transform.py` 的 `HandEyeTransform` 完成，语义为
`p_base = R @ p_cam + t`，与 `agx_visual_servo` 的 C++ `HandEyeTransform`
一致。加载时会校验 `metadata.frame_convention` 必须是 `camera_to_base`，
方向不符直接报错（方向搞反会静默产出完全错误的坐标）。

注意：标定内参是在 848×530 下测得的，而 Demo 默认采集 640×480。两种模式垂直
FOV 相同、共用同一光心，因此**外参在两种分辨率下都有效**，不需要换算内参；
相机内参一律取设备实时上报值。

`bottle_grasp_executor` 的 `hand_eye_yaml_path` 必须与 `--hand-eye` 指向同一次
标定，否则两侧算出的基座系坐标会不一致。

### 5.7 在目标位置上加偏置

用 `--offset X Y Z` 在**基座坐标系**下给抓取点加固定偏置，单位米：

```bash
# 整体抬高 2cm（抓瓶身偏上位置）
python test_yolo_realsense_v3f.py --camera orbbec --no-point-cloud \
    --offset 0 0 0.02

# X 方向后退 5cm，Y 方向平移 1cm
python test_yolo_realsense_v3f.py --camera orbbec --no-point-cloud \
    --offset -0.05 0.01 0
```

偏置**在转换到基座系之后叠加**（`p_base = R @ p_cam + t + offset`），所以：

- 终端 `base=(...)` 和 JSON 的 `position_base` 都**已包含**偏置；
- 相机系 `position` **不受影响**，始终是手眼变换前的原始值；
- 姿态（`orientation_base_xyzw`）不受位置偏置影响；
- 必须配合 `--hand-eye` 生效；手眼关闭时偏置无处可加，会打印提示后忽略。

JSON 顶层新增 `offset_base` 记录当前生效的偏置，便于排查。

注意：`position_base` 只有本 demo 写入，`bottle_grasp_executor` 读的是相机系
`position` 并自己做手眼变换，因此**偏置不会传递到执行器**。要让偏置影响实际
抓取动作，应改执行器的 `grasp_offset_z` / `pre_grasp_offset_z`
（见 [bottle_grasp_executor.yaml](../bottle_grasp_executor/config/bottle_grasp_executor.yaml)）。

### 5.8 把抓取点移到瓶轴上（圆柱拟合）

相机只看得到瓶子朝向自己的那半个圆柱壳，所以默认的抓取点（可见表面质心）必然
贴在一侧瓶壁上，比瓶轴偏向相机约 `2r/π`。Φ60mm 的瓶子约 **19mm**，夹爪闭合中心
因此是偏的。加 `--cylinder-axis` 开启圆柱拟合，把抓取点移到拟合出的瓶轴上：

```bash
LD_LIBRARY_PATH=/opt/OrbbecSDK_v2.9.3/lib /home/qin/miniforge3/bin/python3 \
  test_yolo_realsense_v3f.py --camera orbbec --cylinder-axis \
  --json-out /tmp/bottle_grasp_candidates.json --no-point-cloud
```

算法：取 PCA 长轴，把点投影到垂直于长轴的平面；沿轴向两端各裁掉
`cylinder_axial_trim`（躲开瓶盖/颈肩和瓶桌交界）；在投影平面里做 RANSAC
三点定圆（半径先验 `[cylinder_min_radius, cylinder_max_radius]`），再对最优内点做
Kåsa 代数拟合精修。拟合圆心即瓶轴，半径即瓶半径。**桌面背景靠 RANSAC 排除**，
所以轴心不受表面质心被背景拖拽的影响——这也是不能用“质心 + 半径”去补的原因。

判据（任一不过就退回表面质心并打警告）：

- 半径落在 `[cylinder_min_radius, cylinder_max_radius]` 内；
- 内点率 ≥ `cylinder_min_inlier_ratio`（真机实测：真瓶 62~65%，非瓶目标 20~28%）；
- 拟合圆心必须落在可见面**之后**（远离相机一侧），否则几何上不成立。

实测单帧耗时约 9ms，与桌面 RANSAC 同量级，已含在 `TIMING` 的 `pose=` 分项内。

**注意**：开启后抓取点会横向移动约 19mm（基座系水平面内）。已有的 `--offset` 与
手眼标定都是在旧点位下调的，**首次上真机前必须重新核对**。

## 6. 当前关键参数

几何参数定义在 `GeometryConfig`，ROS 默认值位于 [grasp_pose.yaml](ros2_ws/src/bottle_grasp_perception/config/grasp_pose.yaml)：

- MIN_POINT_COUNT = 100：姿态估计所需的最少目标点数；
- MAX_POINT_COUNT = 6000：参与目标 PCA 的最大点数；
- DEPTH_OUTLIER_THRESHOLD = 0.05：相对中值深度的过滤阈值，单位 m；
- TABLE_DISTANCE_THRESHOLD = 0.008：桌面 RANSAC 内点阈值，单位 m；
- TABLE_MIN_INLIERS = 500：有效桌面模型所需最少内点数；
- TABLE_NORMAL_SMOOTHING = 0.2：桌面法向时序平滑系数；
- UPRIGHT_ANGLE_THRESHOLD_DEG = 25.0：立瓶判断角度阈值；
- USE_CYLINDER_AXIS = False：是否拟合圆柱轴并把抓取点移到瓶轴上（见 5.8）；
- CYLINDER_RANSAC_ITERATIONS = 300：圆柱 RANSAC 迭代次数；
- CYLINDER_MAX_SAMPLE_POINTS = 1500：参与圆柱拟合的最大点数；
- CYLINDER_DISTANCE_THRESHOLD = 0.004：圆的径向内点阈值，单位 m；
- CYLINDER_MIN_INLIER_RATIO = 0.35：内点率下限，低于此值退回表面质心；
- CYLINDER_MIN_RADIUS / CYLINDER_MAX_RADIUS = 0.010 / 0.060：半径先验范围，单位 m；
- CYLINDER_AXIAL_TRIM = 0.25：轴向两端各裁掉的比例；
- confidence = 0.05：统一后处理的检测置信度阈值；
- iou_threshold = 0.45：类别内 NMS IoU 阈值（独立 demo 配置对象参数）；
- model_input_size = 640：PT/RKNN 推理输入边长。
- confidence = 0.05：统一后处理的检测置信度阈值；
- iou_threshold = 0.45：类别内 NMS IoU 阈值（独立 demo 配置对象参数）；
- model_input_size = 640：PT/RKNN 推理输入边长。
- tracker = iou：默认使用轻量 IoU 跟踪。PT 可设为 `bytetrack`、`botsort`、
  `ocsort` 或 Ultralytics tracker YAML 路径；RKNN 指定其他 tracker 时自动回退为 IoU。

这些参数集中在 GeometryConfig 和 InferenceConfig，并由 ROS 参数或 YAML 构造。

## 7. ROS 2 感知节点

第二阶段已按 ROS 2 Humble 完成基础封装。相机仍由 RealSense/Orbbec 厂商 ROS 驱动
负责，本节点不直接访问设备 SDK。工作区结构如下：

```text
ros2_ws/src/
|-- bottle_grasp_interfaces/
|   |-- msg/GraspCandidate.msg
|   |-- msg/GraspCandidatePoints.msg
|   |-- msg/GraspCandidateArray.msg
|   `-- srv/GetGraspCandidates.srv
`-- bottle_grasp_perception/
    |-- bottle_grasp_perception/grasp_pose_node.py
    |-- config/grasp_pose.yaml
    `-- launch/grasp_pose.launch.py
```

### 7.1 构建工作区

构建前请使用与 ROS 2 Humble ABI 一致的 Python 3.10。当前机器的 Conda base 是
Python 3.14，不能用它生成 ROS 接口。可以激活已有 py310 环境（其中已安装
Ultralytics），然后显式指定系统 Python 完成接口构建：

```bash
cd ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --cmake-args \
    -DPython3_EXECUTABLE=/usr/bin/python3 \
    -DPYTHON_EXECUTABLE=/usr/bin/python3 \
    -DPYTHON_INCLUDE_DIR=/usr/include/python3.10 \
    -DPYTHON_LIBRARY=/usr/lib/x86_64-linux-gnu/libpython3.10.so
source install/setup.bash
```

上面的 `libpython3.10.so` 路径是 amd64 示例。RK3588 上应使用板端实际的 aarch64
系统库路径，先执行 `python3-config --configdir` 或查询系统 Python 开发包，不要照搬
`/usr/lib/x86_64-linux-gnu`。

### 7.2 启动节点与模型配置

运行节点时，解释器必须能够导入 ROS 2 Python 包，并能导入当前平台后端：amd64 使用
PyTorch/Ultralytics，RK3588 使用 `rknnlite.api`。本机 amd64 可使用：

```bash
conda activate py310
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
ros2 launch bottle_grasp_perception grasp_pose.launch.py
```

如果在 amd64 使用纯系统 Python，请为系统 Python 安装 PyTorch/Ultralytics，并避免混入
其他 Python ABI 的 ROS 构建缓存。RK3588 则为 ROS 所用的同一个 Python 安装 RKNN Lite。

ROS launch 默认传入不带后缀的 `models/best_train_multi`。安装阶段会将仓库中的 `.pt`
和 `.rknn` 一并复制到包的 `share/models`，运行时按 CPU 架构自动选择。也可以覆盖基础
路径，但同目录下必须存在当前平台对应的后缀文件：

```bash
ros2 launch bottle_grasp_perception grasp_pose.launch.py \
    model_path:=/absolute/path/to/best_train_multi
```

ROS 节点本身不创建 OpenCV/Open3D 窗口，因此天然支持 headless。板端若不需要调试图和
RViz Marker，可在 YAML 中关闭 `publish_debug_image`、`publish_markers`，或运行时设置
相应参数，以减少 CPU 和网络开销。

### 7.3 相机输入与深度对齐

建议订阅：

- /camera/color/image_raw        sensor_msgs/msg/Image
- /camera/depth/image_raw        sensor_msgs/msg/Image（要求已注册到彩色图）
- /camera/depth/camera_info      sensor_msgs/msg/CameraInfo（注册后的内参）

三个输入通过 message_filters 同步，默认使用 50 ms 容差的近似同步，也可以配置为
精确同步。节点接受 16UC1/MONO16 和 32FC1 深度图；16 位深度默认乘 0.001 转换成 m。
彩色图、深度图和 CameraInfo 的尺寸不一致时会拒绝该帧。

Orbbec 驱动通常不会另建 `aligned_depth_to_color` 话题；开启深度注册后仍发布为
`/camera/depth/image_raw`。以 Gemini 330 系列为例，应这样启动相机：

```bash
ros2 launch orbbec_camera gemini_330_series.launch.py \
    depth_registration:=true \
    align_mode:=SW \
    align_target_stream:=COLOR \
    enable_color:=true \
    enable_depth:=true \
    enable_frame_sync:=true
```

请将 launch 文件名替换为实际型号，例如 `gemini2.launch.py`、`femto_bolt.launch.py`。
部分型号默认使用 HW 对齐，可保留该型号 launch 的默认 align_mode，只需确保
depth_registration 为 true 且 align_target_stream 为 COLOR。

启动后检查实际数据而不只是话题名称：

```bash
ros2 topic hz /camera/color/image_raw
ros2 topic hz /camera/depth/image_raw
ros2 topic echo /camera/depth/camera_info --once
```

还应确认彩色图、深度图和 CameraInfo 的 width/height 相同。未注册的原始深度图不能
直接与彩色分割 mask 混用，否则会得到位置偏移的点云和抓取姿态。

### 7.4 输出话题与候选查询

- /bottle_grasp_perception/candidates
  bottle_grasp_interfaces/msg/GraspCandidateArray；
- /bottle_grasp_perception/selected_candidate
  bottle_grasp_interfaces/msg/SelectedCandidate；仅在锁定目标当前可见时发布其 ID、抓取位姿和抓取模式；
- /bottle_grasp_perception/markers
  visualization_msgs/msg/MarkerArray；
- /bottle_grasp_perception/debug_image
  sensor_msgs/msg/Image；
- /bottle_grasp_perception/get_candidates
  bottle_grasp_interfaces/srv/GetGraspCandidates；
- camera optical frame -> grasp_candidate_<track_id> TF（默认关闭）。

查看连续结果：

```bash
ros2 topic echo /bottle_grasp_perception/candidates
```

只查看当前锁定目标的 ID 和抓取位姿：

```bash
ros2 topic echo /bottle_grasp_perception/selected_candidate
```

请求当前锁定目标（track_id = -1）：

```bash
ros2 service call /bottle_grasp_perception/get_candidates \
    bottle_grasp_interfaces/srv/GetGraspCandidates "{track_id: -1}"
```

节点首次选择当前候选中最小的 track_id，并持续锁定；目标连续丢失超过
target_lost_timeout（默认 10 秒）后，才从当前候选中重新选择最小 ID。锁定目标暂时
丢失时服务返回失败，不会返回旧位姿或切换到其他目标。显式指定非负 track_id 时仍按
指定 ID 查询，不受默认目标锁定影响。服务会按 max_result_age 拒绝过期结果。消息时间戳继承彩色图，
frame_id 默认继承彩色图的相机光学坐标系。每个候选包含 track ID、类别、置信度、
Pose 四元数、抓取模式、简单质量分数、长轴角度和参与计算的点数。
### 7.5 候选点云与坐标系约定

GraspCandidateArray.candidates 的元素类型为 GraspCandidatePoints，每个元素的
object_points（geometry_msgs/Point32[]）直接保存 x/y/z 浮点坐标
（float32，单位 m），使用候选 header 的时间戳和坐标系。发布上限由正整数参数
publish_point_count 设置（默认 1024）；超过上限时按原点云索引等间隔采样，
不足时保留全部点，不补齐。point_count 为
实际发布点数。姿态估计和 quality 仍使用采样前的点云。
可在 config/grasp_pose.yaml 中修改 publish_point_count，或运行时设置（下一帧生效）：

```bash
ros2 param set /bottle_grasp_perception publish_point_count 2048
```

该参数只控制发布数量；实际点数还受原始有效点数和算法参数 max_point_count
（默认 6000）限制。服务在收到下一帧后返回新数量的点云。

连续发布和服务查询均包含点云。此接口变更后，发布端和订阅端都需要重新编译并
source 工作空间的 install/setup.bash。

frame_id_override 只能用于纠正相机驱动缺失或错误的光学坐标系标签，它不会执行坐标
变换。转换到 Franka 基座坐标系必须在下一阶段通过带图像时间戳的 tf2 查询完成。

### 7.6 话题与算法参数覆盖

相机话题不一致时可在 launch 中覆盖：

```bash
ros2 launch bottle_grasp_perception grasp_pose.launch.py \
    color_topic:=/your/color/image_raw \
    depth_topic:=/your/registered_depth/image_raw \
    camera_info_topic:=/your/registered_depth/camera_info
```

算法、同步、输出开关和模型参数集中在
[grasp_pose.yaml](ros2_ws/src/bottle_grasp_perception/config/grasp_pose.yaml)。`class_ids` 使用逗号分隔字符串，
例如 "0,1"；空字符串表示不按类别过滤。

模型输入大小可通过 YAML 的 `model_input_size` 设置，或在启动时覆盖：

```bash
ros2 launch bottle_grasp_perception grasp_pose.launch.py model_input_size:=512
```

常见值为 320、416、512、640。较小尺寸通常有利于边缘设备推理速度，但可能降低小目标
和边缘 mask 的精度。该参数只控制 YOLO 内部缩放，不会改变原始 RGB-D 图像的尺寸或
相机内参。

圆柱轴拟合在本节点里由 `use_cylinder_axis` 控制，`grasp_pose.yaml` 中已置 `true`；
同组参数 `cylinder_ransac_iterations` / `cylinder_max_sample_points` /
`cylinder_distance_threshold` / `cylinder_min_inlier_ratio` / `cylinder_min_radius` /
`cylinder_max_radius` / `cylinder_axial_trim` 含义见 5.8 与第 6 节。节点只在
**拟合状态翻转**时打日志（成功时 info、退回表面质心时 warn），不会逐帧刷屏。

注意：ROS 消息（`bottle_grasp_interfaces`）里**没有**半径字段，本节点只把抓取点
`pose.position` 移到轴上；半径仅通过日志和 Demo 的 JSON 暴露。

### 7.7 性能分段计时

节点默认逐帧输出一行 `TIMING` 日志，用于边缘部署性能分析，例如：

```text
TIMING frame=25 callback=94.20ms convert=1.10 \
pipeline=82.30ms yolo=61.20 mask=2.10 env_cloud=1.50 table=10.40 \
objects=6.70 details=[id=3:cloud=1.20/pose=0.30/pack=0.10] \
msg_publish=0.40 markers=0.60 debug=8.90 tf=0.00 candidates=1 env_points=18000
```

字段均为毫秒：

- callback：ROS 收到同步数据到本帧全部输出完成；
- convert：ROS Image 转 NumPy、深度米制转换和输入校验；
- pipeline：核心感知管线总时间；
- yolo：PT/RKNN 推理、统一分割后处理和 IoU 跟踪；
- mask：mask 从模型输出复制、缩放和合并；
- env_cloud：生成桌面环境点云；
- table：RANSAC 桌面拟合和法向平滑；
- objects：所有跟踪实例的点云、PCA 和结果封装总时间；
- details：每个 track ID 的 cloud/pose/pack 细分时间；
- msg_publish、markers、debug、tf：各 ROS 输出阶段时间；debug 进一步细分为
  draw/pack/publish。

相关参数：

```yaml
timing_log_enabled: true
timing_log_every_n: 1
```

日志输出本身会带来少量开销。长期运行可设置每 30 帧打印一次：

```bash
ros2 param set /bottle_grasp_perception timing_log_every_n 30
```

评估纯感知性能时，建议关闭调试图、Marker 和 TF 后对比：

```bash
ros2 param set /bottle_grasp_perception publish_debug_image false
ros2 param set /bottle_grasp_perception publish_markers false
```

独立 demo 可使用以下参数输出核心管线耗时：

```bash
python test_yolo_realsense_v3f.py --camera orbbec --headless --timing
```

## 8. Franka 与视觉伺服集成要点

感知输出首先位于 camera_optical_frame。控制前需要通过手眼标定和 tf2 转换到机器人
基座坐标系：

```text
T_base_grasp = T_base_camera * T_camera_grasp
```

如果相机安装在末端（eye-in-hand），T_base_camera 会随机器人状态变化，必须使用图像
时间戳查询 tf2；如果相机固定在外部（eye-to-hand），仍需完成 base 到 camera 的外参
标定。随后还需应用抓取坐标系到实际工具坐标系的固定变换。

用于真机伺服前至少应增加：

- 位姿有效标志、超时和无目标状态；
- 跟踪 ID 锁定，避免控制过程中切换目标；
- 位姿时序滤波，并处理旋转矩阵/四元数符号连续性；
- 置信度、点数、平面拟合质量和姿态稳定度评分；
- 预抓取位姿（沿进近轴保留安全距离）；
- 工作空间、桌面距离、碰撞和机械臂可达性检查；
- 最大平移/旋转速度以及单周期增量限制；
- 感知丢失时停止或撤退的安全状态机；
- 急停、限位和独立安全监控。

强化学习系统建议接收基座坐标系下、归一化后的目标相对位姿，例如末端到抓取目标的
位置误差和旋转误差，同时保留 perception_valid、confidence、target_age 等状态量。
不要让策略直接依赖可变的 track_id 数值。

## 9. 建议开发顺序

### 9.1 算法解耦（已完成基础版本）

- [x] 将模型加载、单帧推理、点云处理和姿态估计拆成独立类/函数；
- [x] 将模型路径、类别、阈值和点云参数改为配置对象；
- [x] 定义统一的 GraspCandidate 和 FrameResult 数据结构；
- [x] 增加合成点云离线单元测试；
- [ ] 增加真实 RGB-D 帧/数据集回放及期望结果测试。

### 9.2 ROS 感知节点（已完成基础版本）

- [x] 接入标准 Image/CameraInfo；
- [x] 时间同步、ROS Image/NumPy 转换、米制深度转换和坐标系声明；
- [x] 发布候选位姿、调试图像、RViz Marker 和可选 TF；
- [x] 提供读取最近候选的 service；
- [x] 增加 launch 和 YAML 参数；
- [ ] 相机/rosbag 端到端回归测试。

### 9.3 Franka 集成

- 完成手眼标定与 tf2 变换；
- 增加目标锁定、滤波、预抓取位姿和安全检查；
- 先在仿真/回放中验证，再低速接入真机；
- 最后将稳定、带有效性标志的观测接口接入强化学习环境。

## 10. 已知限制

- 自动平台选择目前仅支持 amd64/x86_64 和 arm64/aarch64；
- RKNN 权重与 PT 权重必须使用相同基础文件名，类别名称需与 PT 训练配置保持一致；
- RKNN/NPU 运行依赖板端正确安装与系统匹配的 rknn_toolkit_lite2；
- 独立 demo 仍直接管理相机；ROS 节点使用标准图像话题；
- 桌面法向依赖环境点云和可见桌面区域；
- PCA 主轴对近似对称、遮挡严重或分割不完整的目标可能不稳定；
- 圆柱拟合也依赖 PCA 长轴：如果 mask 泄漏进来的背景在跨度上超过瓶高，长轴会被
  背景带走，此时拟合要么被判据拒绝（安全退回），要么算出错误的轴。合成点云上
  已经复现过这种失败，真机在 `confidence=0.5` 下长轴稳定（帧间约 0.7°），但把
  `confidence` 放得很松时需要重新确认；
- 抓取中心采用目标点云中值，未考虑夹爪指宽、目标直径和抓取接触位置；
  （开启圆柱拟合后抓取中心改到瓶轴，但夹爪指宽与接触位置仍未建模）
- ROS 消息已输出四元数和基础质量分数，但尚未估计姿态协方差；
- 没有手眼标定、tf2 坐标变换及机器人安全控制；
- 图形调试依赖桌面环境；无显示器部署应使用 `--headless`，ROS 节点则关闭不需要的
  debug image/Marker 发布。

## 11. 版本演进记录

- v1：YOLO 实时目标检测 + RealSense；
- v2：加入深度信息和目标三维坐标；
- v3：YOLO 分割 mask + 三维坐标；
- v3a：加入目标跟踪；
- v3b：支持 RealSense/Orbbec 切换；
- v3c：使用 PCA 计算抓取姿态；
- v3d：使用 bottle 分割模型并增加类别/阈值设置；
- v3e：增加点云和抓取姿态三维可视化；
- v3f：使用目标点云 PCA 长轴和桌面法向共同确定抓取姿态。
- v3g：增加 amd64 PT / arm64 RKNN 自动切换、统一分割后处理和 RK3588 headless 模式。
- v3h：可选圆柱拟合，把抓取点从可见表面质心移到瓶轴上（`--cylinder-axis` /
  `use_cylinder_axis`），失败自动退回表面质心并打警告。
