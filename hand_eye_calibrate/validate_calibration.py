#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""验证相机内参和手眼外参的质量。

本脚本读取：
  1) 标定目录中的 camera_intrinsics.yaml
  2) 标定目录中的 hand_eye_extrinsics.yaml
  3) 验证目录中的图片和 poses.txt

输出：
  - 相机内参重投影误差（平均 / 最大）
  - 手眼外参相对运动残差（平均 / 最大）

说明：
  这是工程上很常见的验证方式：不用直接看矩阵数值，而是看它在新样本上的残差是否足够小。
"""

import argparse
import os

import cv2
import numpy as np
import yaml
import math

jump = []
def read_yaml(path: str):
    with open(path, "r", encoding="utf-8") as f:
        return yaml.safe_load(f)


def load_pose_file(path: str):
    """读取 poses.txt，每行 x,y,z,Rx,Ry,Rz，返回 list of 6D pose。"""
    if not os.path.exists(path):
        raise FileNotFoundError(f"未找到位姿文件: {path}")

    poses = []
    with open(path, "r", encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            values = [float(v.strip()) for v in line.split(",")]
            if len(values) != 6:
                raise ValueError(f"姿态行格式错误: {line!r}")
            poses.append(values)

    if not poses:
        raise RuntimeError(f"位姿文件为空: {path}")
    return poses


def euler_to_rotation_matrix(rx: float, ry: float, rz: float) -> np.ndarray:
    rx = float(rx)
    ry = float(ry)
    rz = float(rz)
    Rx = np.array([
        [1, 0, 0],
        [0, math.cos(rx), -math.sin(rx)],
        [0, math.sin(rx), math.cos(rx)],
    ])
    Ry = np.array([
        [math.cos(ry), 0, math.sin(ry)],
        [0, 1, 0],
        [-math.sin(ry), 0, math.cos(ry)],
    ])
    Rz = np.array([
        [math.cos(rz), -math.sin(rz), 0],
        [math.sin(rz), math.cos(rz), 0],
        [0, 0, 1],
    ])
    return Rz @ Ry @ Rx


def pose_to_homogeneous(pose):
    x, y, z, rx, ry, rz = pose
    R = euler_to_rotation_matrix(rx, ry, rz)
    t = np.array([x, y, z], dtype=np.float64).reshape(3, 1)
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = R
    T[:3, 3] = t[:, 0]
    return T


def homogeneous_to_rt(T: np.ndarray):
    R = T[:3, :3]
    t = T[:3, 3].reshape(3, 1)
    return R, t


def detect_board(image_path: str, board_size, square_size: float):
    img = cv2.imread(image_path, cv2.IMREAD_COLOR)
    if img is None:
        raise FileNotFoundError(f"无法读取图像: {image_path}")

    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    pattern_size = tuple(board_size)
    ret, corners = cv2.findChessboardCorners(gray, pattern_size, None)
    if not ret:
        return None, None, img

    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)

    objp = np.zeros((board_size[0] * board_size[1], 3), np.float64)
    objp[:, :2] = np.mgrid[0:board_size[0], 0:board_size[1]].T.reshape(-1, 2)
    objp *= square_size
    return objp, corners, img


def validate_intrinsics(image_dir: str, calib_dir: str, board_size=(8, 6), square_size=0.025):
    intrinsics = read_yaml(os.path.join(calib_dir, "camera_intrinsics.yaml"))
    K = np.array(intrinsics["K"], dtype=np.float64)
    D = np.array(intrinsics["D"], dtype=np.float64).reshape(-1)

    image_files = sorted(
        [f for f in os.listdir(image_dir) if f.lower().endswith((".jpg", ".png", ".jpeg"))],
        key=lambda s: int(os.path.splitext(s)[0]) if os.path.splitext(s)[0].isdigit() else s,
    )
    if not image_files:
        raise RuntimeError(f"验证目录中没有图像: {image_dir}")

    total_errors = []
    image_error_rows = []
    valid_count = 0
    test = -1
    for fname in image_files:
        test +=1 
        if test in jump:
            print(f"跳过{test}张图片：{test}")
            
            continue
        image_path = os.path.join(image_dir, fname)
        objp, corners, _ = detect_board(image_path, board_size, square_size)
        if objp is None or corners is None:
            continue

        valid_count += 1
        # solvePnP uses board points in object frame, returns camera pose relative to object
        success, rvec, tvec = cv2.solvePnP(objp, corners, K, D, flags=cv2.SOLVEPNP_ITERATIVE)
        if not success:
            continue

        projected, _ = cv2.projectPoints(objp, rvec, tvec, K, D)
        projected = projected.reshape(-1, 2)
        corners_2d = corners.reshape(-1, 2)
        diff = projected - corners_2d
        err = np.linalg.norm(diff, axis=1)
        total_errors.extend(err.tolist())
        image_error_rows.append((fname, float(err.mean()), float(err.max()), float(np.sqrt(np.mean(err ** 2)))))

    if valid_count == 0:
        raise RuntimeError("没有图像能够正确检测到棋盘角点。")

    errs = np.asarray(total_errors, dtype=np.float64)
    print("[内参验证] 有效图像数:", valid_count)
    print("[内参验证] 平均重投影误差 (px):", float(errs.mean()))
    print("[内参验证] 最大重投影误差 (px):", float(errs.max()))
    print("[内参验证] RMS 重投影误差 (px):", float(np.sqrt(np.mean(errs ** 2))))
    print("[内参验证] 误差最大的图像 Top5:")
    for fname, mean_px, max_px, rms_px in sorted(image_error_rows, key=lambda row: row[2], reverse=True)[:5]:
        print(f"  {fname}: mean={mean_px:.4f}px, max={max_px:.4f}px, rms={rms_px:.4f}px")
    return {
        "valid_count": valid_count,
        "mean_px": float(errs.mean()),
        "max_px": float(errs.max()),
        "rms_px": float(np.sqrt(np.mean(errs ** 2))),
        "image_errors": image_error_rows,
    }


def validate_extrinsics(image_dir: str, calib_dir: str, pose_path: str, mode="eye_to_hand", board_size=(8, 6), square_size=0.025):
    extrinsics = read_yaml(os.path.join(calib_dir, "hand_eye_extrinsics.yaml"))
    transform_matrix = np.array(extrinsics["transform_matrix"], dtype=np.float64)
    X = transform_matrix

    poses = load_pose_file(pose_path)
    image_files = sorted(
        [f for f in os.listdir(image_dir) if f.lower().endswith((".jpg", ".png", ".jpeg"))],
        key=lambda s: int(os.path.splitext(s)[0]) if os.path.splitext(s)[0].isdigit() else s,
    )

    intrinsics = read_yaml(os.path.join(calib_dir, "camera_intrinsics.yaml"))
    K = np.array(intrinsics["K"], dtype=np.float64)
    D = np.array(intrinsics["D"], dtype=np.float64).reshape(-1)

    valid_samples = []
    for idx, fname in enumerate(image_files):
        if idx in jump:
            print(f"跳过{idx}张图片")
            continue        
        if idx >= len(poses):
            break
        image_path = os.path.join(image_dir, fname)
        objp, corners, _ = detect_board(image_path, board_size, square_size)
        if objp is None or corners is None:
            continue
        success, rvec, tvec = cv2.solvePnP(objp, corners, K, D, flags=cv2.SOLVEPNP_ITERATIVE)
        if not success:
            continue
        R, _ = cv2.Rodrigues(rvec)
        T_target_camera = np.eye(4)
        T_target_camera[:3, :3] = R
        T_target_camera[:3, 3] = tvec.reshape(3)
        valid_samples.append((idx, pose_to_homogeneous(poses[idx]), T_target_camera))

    if len(valid_samples) < 2:
        raise RuntimeError("检测到的有效图像太少，无法做手眼外参残差验证。")

    if mode == "eye_to_hand":
        # X is camera->base. The board is fixed on the flange, so
        # flange->board should be constant across all valid samples.
        flange_to_board = []
        for _, T_flange_base, T_target_camera in valid_samples:
            T_base_camera = X
            T_flange_board = np.linalg.inv(T_flange_base) @ T_base_camera @ T_target_camera
            flange_to_board.append(T_flange_board)

        translations = np.asarray([T[:3, 3] for T in flange_to_board], dtype=np.float64)
        mean_translation = translations.mean(axis=0)
        trans_residuals = np.linalg.norm(translations - mean_translation, axis=1)

        mean_R = flange_to_board[0][:3, :3]
        rot_residuals = [
            np.linalg.norm(T[:3, :3] - mean_R, ord="fro")
            for T in flange_to_board
        ]
        rot_residuals = np.asarray(rot_residuals, dtype=np.float64)
    else:
        # X is camera->flange. The board is fixed in the robot base/world,
        # so base->board should be constant across all valid samples.
        base_to_board = []
        for _, T_flange_base, T_target_camera in valid_samples:
            T_base_board = T_flange_base @ X @ T_target_camera
            base_to_board.append(T_base_board)

        translations = np.asarray([T[:3, 3] for T in base_to_board], dtype=np.float64)
        mean_translation = translations.mean(axis=0)
        trans_residuals = np.linalg.norm(translations - mean_translation, axis=1)

        mean_R = base_to_board[0][:3, :3]
        rot_residuals = [
            np.linalg.norm(T[:3, :3] - mean_R, ord="fro")
            for T in base_to_board
        ]
        rot_residuals = np.asarray(rot_residuals, dtype=np.float64)

    mean_rot = rot_residuals.mean()
    mean_trans = trans_residuals.mean()
    max_rot = rot_residuals.max()
    max_trans = trans_residuals.max()

    print("[外参验证] 有效样本数:", len(valid_samples))
    print("[外参验证] 平均旋转一致性残差 (Frobenius):", float(mean_rot))
    print("[外参验证] 最大旋转一致性残差 (Frobenius):", float(max_rot))
    print("[外参验证] 平均平移一致性残差 (m):", float(mean_trans))
    print("[外参验证] 最大平移一致性残差 (m):", float(max_trans))
    residual_rows = [
        (sample_idx, float(rot_err), float(trans_err))
        for (sample_idx, _, _), rot_err, trans_err in zip(valid_samples, rot_residuals, trans_residuals)
    ]
    print("[外参验证] 平移残差最大的样本 Top5:")
    for sample_idx, rot_err, trans_err in sorted(residual_rows, key=lambda row: row[2], reverse=True)[:5]:
        print(f"  image/pose index {sample_idx}: trans={trans_err:.6f}m, rot={rot_err:.6f}")

    return {
        "valid_count": len(valid_samples),
        "mean_rot_residual": float(mean_rot),
        "max_rot_residual": float(max_rot),
        "mean_trans_residual": float(mean_trans),
        "max_trans_residual": float(max_trans),
        "residual_rows": residual_rows,
    }


def main():
    parser = argparse.ArgumentParser(description="验证相机内参和手眼外参")
    parser.add_argument("--calib-dir", default="./collect_data_test", help="保存内外参 YAML 的目录")
    parser.add_argument("--validate-dir", default="./yanzen", help="验证图像和 poses.txt 所在目录")
    parser.add_argument("--mode", default="eye_to_hand", choices=["eye_in_hand", "eye_to_hand"], help="标定模式")
    parser.add_argument("--board-cols", type=int, default=8, help="棋盘列数")
    parser.add_argument("--board-rows", type=int, default=6, help="棋盘行数")
    parser.add_argument("--square-size", type=float, default=0.025, help="棋盘格边长，单位米")
    args = parser.parse_args()

    board_size = (args.board_cols, args.board_rows)
    pose_path = os.path.join(args.validate_dir, "poses.txt")

    # print("==== 验证配置 ====")
    # print("calib_dir:", args.calib_dir)
    # print("validate_dir:", args.validate_dir)
    # print("mode:", args.mode)
    # print("board_size:", board_size)
    # print("square_size:", args.square_size)
    # print()

    intrinsic_result = validate_intrinsics(args.validate_dir, args.calib_dir, board_size, args.square_size)
    print("\n")
    extrinsic_result = validate_extrinsics(args.validate_dir, args.calib_dir, pose_path, args.mode, board_size, args.square_size)

    print("\n==== 验证结论 ====")
    print("内参平均重投影误差:", round(intrinsic_result["mean_px"], 4), "px")
    print("内参最大重投影误差:", round(intrinsic_result["max_px"], 4), "px")
    print("\n")
    print("外参平均相对平移残差:", round(extrinsic_result["mean_trans_residual"], 6))
    print("外参最大相对平移残差:", round(extrinsic_result["max_trans_residual"], 6))

    intrinsics_ok = intrinsic_result["mean_px"] < 1.0 and intrinsic_result["max_px"] < 5.0
    extrinsics_ok = extrinsic_result["mean_trans_residual"] < 0.02 and extrinsic_result["max_trans_residual"] < 0.05

    if intrinsics_ok and extrinsics_ok:
        print("判定：当前标定结果在该验证集上表现良好。")
    else:
        print("判定：当前标定结果尚需进一步检查，建议重新采集更稳定的棋盘图像或优化标定流程。")
        if not intrinsics_ok:
            print("原因：内参重投影误差偏大，可能存在角点检测不准、模糊、棋盘参数错误或图像质量问题。")
        if not extrinsics_ok:
            print("原因：外参一致性残差偏大，尤其最大平移残差会直接导致实际定位偏差。")


if __name__ == "__main__":
    main()
