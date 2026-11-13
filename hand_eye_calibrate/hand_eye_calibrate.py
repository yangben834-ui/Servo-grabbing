# coding=utf-8
"""
眼在手上 用采集到的图片信息和机械臂位姿信息计算 相机坐标系相对于机械臂末端坐标系的 旋转矩阵和平移向量
A2^{-1}*A1*X=X*B2*B1^{−1}
"""

import argparse
import os

import cv2
import numpy as np

from datetime import datetime
try:
    import yaml
except ImportError:  # pragma: no cover - fallback for environments without PyYAML
    yaml = None

np.set_printoptions(precision=8, suppress=True)

images_path = "./collect_data_test"  # 手眼标定采集的标定版图片所在路径
arm_pose_file = "./collect_data_test/poses.txt"  # 采集标定板图片时对应的机械臂末端的位姿，从第一行到最后一行需要和采集的标定板的图片顺序对应
default_calibration_mode = "eye_to_hand"  # 默认模式：外置相机，标定板随法兰移动

# 夹爪偏移：从法兰中心到夹爪工具点（标定板安装点）的固定变换
# 格式: [x, y, z, Rx, Ry, Rz]，单位：米、弧度
# 如果夹爪沿法兰 Z 轴方向延伸了 L 米，则设为 [0, 0, L, 0, 0, 0]
# 设为全零表示没有夹爪偏移（法兰=工具点）
gripper_offset = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]


def euler_angles_to_rotation_matrix(rx, ry, rz):
    """将欧拉角(rx, ry, rz)转换为旋转矩阵。

    参数:
        rx: 绕x轴的旋转角度，单位为弧度。
        ry: 绕y轴的旋转角度，单位为弧度。
        rz: 绕z轴的旋转角度，单位为弧度。

    返回:
        3x3旋转矩阵。
    """
    Rx = np.array([[1, 0, 0],
                   [0, np.cos(rx), -np.sin(rx)],
                   [0, np.sin(rx), np.cos(rx)]])

    Ry = np.array([[np.cos(ry), 0, np.sin(ry)],
                   [0, 1, 0],
                   [-np.sin(ry), 0, np.cos(ry)]])

    Rz = np.array([[np.cos(rz), -np.sin(rz), 0],
                   [np.sin(rz), np.cos(rz), 0],
                   [0, 0, 1]])

    R = Rz @ Ry @ Rx
    return R


def pose_to_homogeneous_matrix(pose):
    """将机械臂末端位姿pose转换为旋转矩阵和平移向量。

    参数:
        pose: 长度为6的列表或数组，[x, y, z, Rx, Ry, Rz]，其中Rx,Ry,Rz为欧拉角。

    返回:
        R: 3x3旋转矩阵。
        t: 3x1平移向量。
    """
    x, y, z, rx, ry, rz = pose

    # 角度归一化到 [-pi, pi)
    rx = np.arctan2(np.sin(rx), np.cos(rx))
    ry = np.arctan2(np.sin(ry), np.cos(ry))
    rz = np.arctan2(np.sin(rz), np.cos(rz))

    R = euler_angles_to_rotation_matrix(rx, ry, rz)
    t = np.array([x, y, z]).reshape(3, 1)

    return R, t


def rotation_vector_to_matrix(rvec):
    """将旋转向量转换为旋转矩阵。"""
    R, _ = cv2.Rodrigues(np.asarray(rvec, dtype=np.float64))
    return R


def invert_rt(R, t):
    """求刚体变换(R,t)的逆。"""
    R_inv = R.T
    t_inv = -R_inv @ np.asarray(t, dtype=np.float64).reshape(3, 1)
    return R_inv, t_inv


def build_hand_eye_problem(mode, R_arm, t_arm, rvecs, tvecs, valid_indices):
    """根据标定模式构造 OpenCV calibrateHandEye 需要的绝对位姿输入。

    OpenCV 需要每个采样点的绝对位姿列表：
      - R_gripper2base/t_gripper2base
      - R_target2cam/t_target2cam

    当前机器人接口返回的是法兰在基坐标系下的位姿，即 flange->base。
      - eye_in_hand: 法兰就是 gripper，直接使用 flange->base。
      - eye_to_hand: 相机外置、标定板随法兰动。为得到 camera->base，
        需要把 flange->base 取逆后作为 gripper2base 传给 OpenCV。
    """
    if mode not in {"eye_in_hand", "eye_to_hand"}:
        raise ValueError(f"不支持的标定模式: {mode}")

    if len(valid_indices) == 0:
        raise RuntimeError("没有可用的相机位姿。")

    if len(valid_indices) < 2:
        raise RuntimeError("至少需要两组有效位姿进行手眼标定。")

    R_gripper2base = []
    t_gripper2base = []
    R_target2cam = []
    t_target2cam = []
    skipped_duplicates = 0

    for pose_idx, rvec, tvec in zip(valid_indices, rvecs, tvecs):
        R_flange2base = R_arm[pose_idx]
        t_flange2base = t_arm[pose_idx]
        if mode == "eye_to_hand":
            R_robot, t_robot = invert_rt(R_flange2base, t_flange2base)
        else:
            R_robot, t_robot = R_flange2base, t_flange2base

        R_target = rotation_vector_to_matrix(rvec)
        t_target = np.asarray(tvec, dtype=np.float64).reshape(3, 1)

        if R_gripper2base:
            same_translation = np.linalg.norm(t_robot - t_gripper2base[-1]) < 1e-9
            same_rotation = np.linalg.norm(R_robot - R_gripper2base[-1], ord="fro") < 1e-9
            if same_translation and same_rotation:
                skipped_duplicates += 1
                print(f"警告: 跳过重复机器人位姿，对应图像索引 {pose_idx}。")
                continue

        R_gripper2base.append(R_robot)
        t_gripper2base.append(t_robot)
        R_target2cam.append(R_target)
        t_target2cam.append(t_target)

    if skipped_duplicates:
        print(f"共跳过 {skipped_duplicates} 组重复机器人位姿。")

    if len(R_gripper2base) < 3:
        raise RuntimeError("去除重复位姿后，有效手眼样本少于3组，请重新采集更多不同姿态。")

    return R_gripper2base, t_gripper2base, R_target2cam, t_target2cam


def save_yaml(file_path, data):
    """将字典数据保存为 YAML 文件。若环境中没有 PyYAML，则使用简化的手写 YAML 格式。"""
    os.makedirs(os.path.dirname(file_path), exist_ok=True)
    if yaml is not None:
        with open(file_path, "w", encoding="utf-8") as f:
            yaml.safe_dump(data, f, sort_keys=False)
        return

    with open(file_path, "w", encoding="utf-8") as f:
        for key, value in data.items():
            if isinstance(value, dict):
                f.write(f"{key}:\n")
                for sub_key, sub_value in value.items():
                    if isinstance(sub_value, (list, tuple, np.ndarray)):
                        f.write(f"  {sub_key}:\n")
                        for item in np.asarray(sub_value).tolist():
                            if isinstance(item, (list, tuple)):
                                f.write(f"    - {item}\n")
                            else:
                                f.write(f"    - {item}\n")
                    else:
                        f.write(f"  {sub_key}: {sub_value}\n")
            elif isinstance(value, (list, tuple, np.ndarray)):
                f.write(f"{key}:\n")
                for item in np.asarray(value).tolist():
                    if isinstance(item, (list, tuple)):
                        f.write(f"  - {item}\n")
                    else:
                        f.write(f"  - {item}\n")
            else:
                f.write(f"{key}: {value}\n")


def camera_calibrate(images_path, show_corners=True):
    """对指定路径下的棋盘格图片进行相机标定，获取相机外参。

    参数:
        images_path: 保存标定板图片的文件夹路径，图片命名为0.jpg、1.jpg...等。

    返回:
        mtx: 相机内参矩阵。
        dist: 畸变系数。
        rvecs: 每张成功标定图片对应的旋转向量列表。
        tvecs: 每张成功标定图片对应的平移向量列表。
        valid_indices: 成功检测到棋盘的图片索引列表。
    """
    print("++++++++++开始相机标定++++++++++++++")
    # 角点的个数以及棋盘格间距
    XX = 8  # 标定板的中长度对应的角点的个数
    YY = 6  # 标定板的中宽度对应的角点的个数
    L = 0.025  # 标定板一格的长度  单位为米

    # 设置寻找亚像素角点的参数，采用的停止准则是最大循环次数30和最大误差容限0.001
    criteria = (cv2.TERM_CRITERIA_MAX_ITER | cv2.TERM_CRITERIA_EPS, 30, 0.001)

    # 获取标定板角点的位置
    objp = np.zeros((XX * YY, 3), np.float32)
    objp[:, :2] = np.mgrid[0:XX, 0:YY].T.reshape(-1, 2)  # 将世界坐标系建在标定板上，所有点的Z坐标全部为0，所以只需要赋值x和y
    objp = L * objp

    obj_points = []  # 存储3D点
    img_points = []  # 存储2D点
    valid_indices = []  # 存储成功检测到棋盘角点的图片索引

    def numeric_key(name: str):
        base = os.path.splitext(name)[0]
        if base.isdigit():
            return int(base)
        return name

    image_files = sorted(
        [f for f in os.listdir(images_path) if f.lower().endswith(('.jpg', '.png', '.jpeg'))],
        key=numeric_key,
    )
    if not image_files:
        raise RuntimeError(f"目录 {images_path} 中没有找到图像文件。")

    jump = []
    for i, image_name in enumerate(image_files):
        if i in jump:
            print(f"跳过{i}张图片：{image}")
            continue
        image = os.path.join(images_path, image_name)
        print(f"正在处理第{i}张图片：{image}")

        img = cv2.imread(image)
        if img is None:
            print(f"无法读取图像 {image}，跳过。")
            continue

        print(f"图像大小： {img.shape}")
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        size = gray.shape[::-1]
        ret, corners = cv2.findChessboardCorners(gray, (XX, YY), None)
        if not ret or corners is None:
            print("未检测到棋盘格角点，跳过该图像。")
            continue

        print(f"左上角点：{corners[0, 0]}")
        print(f"右下角点：{corners[-1, -1]}")

        if show_corners:
            cv2.drawChessboardCorners(img, (XX, YY), corners, ret)
            cv2.imshow('Chessboard', img)
            # cv2.waitKey(1500)

        obj_points.append(objp)
        valid_indices.append(i)

        corners2 = cv2.cornerSubPix(gray, corners, (5, 5), (-1, -1), criteria)  # 在原角点的基础上寻找亚像素角点
        if [corners2]:
            img_points.append(corners2)
        else:
            img_points.append(corners)

    N = len(img_points)
    print(f"成功检测到 {N} 张图像的棋盘角点。")

    if N < 1:
        raise RuntimeError("未找到任何有效的棋盘角点图像，无法进行相机标定。")

    # 标定得到图案在相机坐标系下的位姿
    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(obj_points, img_points, size, None, None)

    print("内参矩阵:\n", mtx)  # 内参数矩阵
    print("畸变系数:\n", dist)  # 畸变系数   distortion cofficients = (k_1,k_2,p_1,p_2,k_3)

    print("++++++++++相机标定完成++++++++++++++")

    return mtx, dist, rvecs, tvecs, valid_indices ,size, (XX, YY), L


def process_arm_pose(arm_pose_file, gripper_offset=None):
    """读取机械臂位姿文件，并将每行位姿转换为旋转矩阵和平移向量。

    如果提供了 gripper_offset，则将法兰位姿补偿到工具点（夹爪末端）位姿。
    T_tool = T_flange * T_offset

    参数:
        arm_pose_file: 存储机械臂末端位姿的文本文件路径，每行格式为x,y,z,Rx,Ry,Rz。
        gripper_offset: 可选，长度为6的列表 [x, y, z, Rx, Ry, Rz]，法兰到工具点的偏移。

    返回:
        R_arm: 旋转矩阵列表。
        t_arm: 平移向量列表。
    """

    # 计算夹爪偏移的齐次变换矩阵
    if gripper_offset is not None and np.linalg.norm(gripper_offset) > 1e-9:
        R_offset, t_offset = pose_to_homogeneous_matrix(gripper_offset)
        T_offset = np.eye(4)
        T_offset[:3, :3] = R_offset
        T_offset[:3, 3] = t_offset.flatten()
        apply_offset = True
        print(f"夹爪偏移已启用: {gripper_offset}")
    else:
        apply_offset = False

    R_arm, t_arm = [], []
    with open(arm_pose_file, "r", encoding="utf-8") as f:
        # 读取文件中的所有行
        all_lines = f.readlines()
    for line in all_lines:
        pose = [float(v) for v in line.split(',')]
        R_flange, t_flange = pose_to_homogeneous_matrix(pose=pose)

        if apply_offset:
            # T_tool = T_flange * T_offset
            T_flange = np.eye(4)
            T_flange[:3, :3] = R_flange
            T_flange[:3, 3] = t_flange.flatten()
            T_tool = T_flange @ T_offset
            R_arm.append(T_tool[:3, :3])
            t_arm.append(T_tool[:3, 3].reshape(3, 1))
        else:
            R_arm.append(R_flange)
            t_arm.append(t_flange)
    return R_arm, t_arm


def hand_eye_calibrate(mode, show_corners=True):
    """执行手眼标定流程，计算相机相对于机械臂末端或基座的位姿变换。

    mode:
        "eye_in_hand"：相机安装在机械臂末端，输出相机到机械臂末端变换。
        "eye_to_hand"：相机固定在工作空间外部，输出相机到机械臂基座变换。

    返回:
        R: 旋转矩阵。
        t: 平移向量。
    """
    mtx, dist, rvecs, tvecs, valid_indices, img_size, board_size, square_size = camera_calibrate(images_path, show_corners)
    R_arm, t_arm = process_arm_pose(arm_pose_file=arm_pose_file, gripper_offset=gripper_offset)

    if len(R_arm) != len(t_arm):
        raise RuntimeError("机械臂位姿数量不一致，无法进行手眼标定。")

    if len(valid_indices) == 0:
        raise RuntimeError("没有可用的相机位姿用于手眼标定。")

    if len(valid_indices) > len(R_arm):
        raise RuntimeError("有效图像数量超过机械臂位姿数量，请检查输入数据。")

    R_arm_selected, t_arm_selected, R_target_selected, t_target_selected = build_hand_eye_problem(
        mode,
        R_arm,
        t_arm,
        rvecs,
        tvecs,
        valid_indices,
    )

    if len(R_arm_selected) != len(R_target_selected):
        raise RuntimeError("构造出的手眼标定数据组数量不匹配。")

    if len(R_arm_selected) < 2:
        raise RuntimeError("至少需要两组有效位姿进行手眼标定。")

    R, t = cv2.calibrateHandEye(
        R_arm_selected,
        t_arm_selected,
        R_target_selected,
        t_target_selected,
        cv2.CALIB_HAND_EYE_PARK,
    )
    print("+++++++++++手眼标定完成+++++++++++++++")

    output_dir = images_path
    calib_timestamp = datetime.now().strftime("%Y-%m-%d %H:%M")
    intrinsics_path = os.path.join(output_dir, "camera_intrinsics.yaml")
    intrinsics_data = {
        "camera_name": "hand_camera",           # 根据你的实际情况修改
        "image_width": img_size[0],                 # size 来自 camera_calibrate 中的 gray.shape[::-1]
        "image_height": img_size[1],
        "distortion_model": "plumb_bob",        # OpenCV 默认5参数畸变模型
        "K": np.asarray(mtx).tolist(),
        "D": np.asarray(dist).reshape(-1).tolist(),
        "metadata": {
            "calib_date": calib_timestamp,
            "chessboard_size": [board_size[0], board_size[1]],        # 注意: XX,YY 需从 camera_calibrate 返回或改为全局变量
            "square_size_m": square_size,                 # 同上
            "valid_images_count": len(valid_indices),
            "calibration_mode": mode,
        }
    }

    hand_eye_path = os.path.join(output_dir, "hand_eye_extrinsics.yaml")

    T_cam_to_tool = np.eye(4)
    T_cam_to_tool[:3, :3] = R
    T_cam_to_tool[:3, 3] = t.flatten()

    hand_eye_data = {
        "transform_matrix": T_cam_to_tool.tolist(),  # 4x4 齐次矩阵
        "metadata": {
            "calib_date": calib_timestamp,
            "method": "Tsai-Lenz",
            "frame_convention": "camera_to_tool" if mode == "eye_in_hand" else "camera_to_base",
            "calibration_mode": mode,
            "translation_unit": "meter",              # 关键: 标注单位
            "rotation_unit": "radian",
            "valid_samples_count": len(R_arm_selected)
        }
    }
    
    # 3. 执行保存
    save_yaml(intrinsics_path, intrinsics_data)
    save_yaml(hand_eye_path, hand_eye_data)
    
    print(f"✅ 已保存相机内参到: {intrinsics_path}")
    print(f"✅ 已保存手眼外参到: {hand_eye_path}")

    return R, t


def parse_arguments():
    parser = argparse.ArgumentParser(description="手眼标定工具，支持 eye_in_hand 和 eye_to_hand 模式。")
    parser.add_argument("--mode", choices=["eye_in_hand", "eye_to_hand"], default=default_calibration_mode,
                        help="选择手眼标定模式。")
    parser.add_argument("--data-dir", default=images_path,
                        help="标定图片和poses.txt所在目录。")
    parser.add_argument("--no-show-corners", action="store_true",
                        help="不显示每张图像的棋盘角点检测结果。")
    parser.add_argument("--gripper-offset", nargs=6, type=float, default=None, metavar=("X", "Y", "Z", "RX", "RY", "RZ"),
                        help="夹爪偏移量 [x y z Rx Ry Rz]，单位：米、弧度。"
                             " 例如: --gripper-offset 0 0 0.12 0 0 0 表示夹爪沿Z轴延伸0.12米")
    return parser.parse_args()


if __name__ == "__main__":
    args = parse_arguments()
    images_path = args.data_dir
    arm_pose_file = os.path.join(images_path, "poses.txt")
    if args.gripper_offset is not None:
        gripper_offset = args.gripper_offset
        print(f"使用命令行夹爪偏移: {gripper_offset}")
    R, t = hand_eye_calibrate(args.mode, not args.no_show_corners)

    print("旋转矩阵：")
    print(R)
    print("平移向量：")
    print(t)
