# coding=utf-8
"""
手眼标定数据自洽性检查器（在求解手眼外参之前先用它验证采集质量）。

核心思想：
    标定板刚性安装在机械臂法兰上、相机固定不动的前提下，任意两次位姿之间：
      法兰的相对旋转角度（由 poses.txt 报出的 flange 位姿算出）
      与 标定板在相机里的相对旋转角度（由图片 + 内参 solvePnP 算出）
    必须相等（旋转角与坐标系无关）。若两者明显不一致，说明这组数据
    无法被任何一个刚性相机外参 X 拟合，手眼标定结果必然不稳定/差距大。

用法（务必用 /usr/bin/python3，即带 calibrateHandEye 的 opencv 4.5.4 环境）：
    /usr/bin/python3 check_calib_consistency.py my_calibration
    /usr/bin/python3 check_calib_consistency.py <数据目录> [cols rows square_m]

输出：
    平均/中位/p90/最大 的角度差（单位：度）。
    参考判据：老相机采集（realsense / 1280x720）约 0.7~1.2°；
    若均值 > 3° 或 p90 > 6°，这组数据自洽性差，不应直接拿去标定。
"""

import sys
import os
import glob

import cv2
import numpy as np


def euler_angles_to_rotation_matrix(rx, ry, rz):
    Rx = np.array([[1, 0, 0],
                   [0, np.cos(rx), -np.sin(rx)],
                   [0, np.sin(rx), np.cos(rx)]])
    Ry = np.array([[np.cos(ry), 0, np.sin(ry)],
                   [0, 1, 0],
                   [-np.sin(ry), 0, np.cos(ry)]])
    Rz = np.array([[np.cos(rz), -np.sin(rz), 0],
                   [np.sin(rz), np.cos(rz), 0],
                   [0, 0, 1]])
    return Rz @ Ry @ Rx


def rot_angle_deg(R):
    return float(np.degrees(np.arccos(np.clip((np.trace(R) - 1) / 2, -1.0, 1.0))))


def main(data_dir, XX=8, YY=6, L=0.025):
    image_files = sorted(
        glob.glob(os.path.join(data_dir, "*.jpg"))
        + glob.glob(os.path.join(data_dir, "*.png")),
        key=lambda p: int(os.path.splitext(os.path.basename(p))[0]),
    )
    pose_file = os.path.join(data_dir, "poses.txt")
    if not image_files or not os.path.exists(pose_file):
        print(f"目录 {data_dir} 下缺少 0.jpg... 或 poses.txt")
        sys.exit(1)

    with open(pose_file, encoding="utf-8") as f:
        poses = [[float(v) for v in ln.strip().split(",")] for ln in f if ln.strip()]
    print(f"图片 {len(image_files)} 张, 位姿 {len(poses)} 行")

    criteria = (cv2.TERM_CRITERIA_MAX_ITER | cv2.TERM_CRITERIA_EPS, 30, 0.001)
    objp = np.zeros((XX * YY, 3), np.float32)
    objp[:, :2] = np.mgrid[0:XX, 0:YY].T.reshape(-1, 2) * L

    imgpts = []
    ids = []
    for f in image_files:
        idx = int(os.path.splitext(os.path.basename(f))[0])
        gray = cv2.cvtColor(cv2.imread(f), cv2.COLOR_BGR2GRAY)
        ret, corners = cv2.findChessboardCorners(gray, (XX, YY), None)
        if not ret or corners is None:
            print(f"  图片 {idx}: 棋盘检测失败, 跳过")
            continue
        c2 = cv2.cornerSubPix(gray, corners, (5, 5), (-1, -1), criteria)
        imgpts.append(np.asarray(c2).reshape(-1, 2))
        ids.append(idx)

    n = len(ids)
    if n < 3:
        print("有效图片 < 3, 无法评估")
        sys.exit(1)
    H, W = gray.shape
    _, mtx, dist, _, _ = cv2.calibrateCamera(
        [objp] * n, imgpts, (W, H), None, None
    )
    print(f"有效图片 {n} 张, 相机内参标定 RMS = {cv2.calibrateCamera([objp]*n, imgpts, (W,H), None, None)[0]:.4f} px")
    print(f"  K diag = {np.round(np.diag(mtx), 1)}, D = {np.round(dist.ravel(), 4)}")

    # 每张图: flange 旋转 与 board(相机) 旋转
    R_flange = {}
    R_board = {}
    for k, idx in enumerate(ids):
        if idx >= len(poses):
            continue
        x, y, z, rx, ry, rz = poses[idx]
        R_flange[idx] = euler_angles_to_rotation_matrix(rx, ry, rz)
        rvec, tvec = cv2.solvePnP(objp.astype(np.float32),
                                  imgpts[k].astype(np.float32), mtx, dist)[1:]
        R_board[idx], _ = cv2.Rodrigues(rvec)

    idxs = sorted(set(R_flange) & set(R_board))
    if len(idxs) < 3:
        print("图片与位姿索引对不齐, 无法评估")
        sys.exit(1)

    diffs = []
    per_sample = {i: [] for i in idxs}
    for a in range(len(idxs)):
        for b in range(a + 1, len(idxs)):
            i, j = idxs[a], idxs[b]
            aF = rot_angle_deg(R_flange[j] @ R_flange[i].T)
            aB = rot_angle_deg(R_board[j] @ R_board[i].T)
            d = abs(aF - aB)
            diffs.append(d)
            per_sample[i].append(d)
            per_sample[j].append(d)

    d = np.array(diffs)
    print("\n相对旋转角 |法兰-标定板| 差异(度): "
          f"mean={d.mean():.2f}  median={np.median(d):.2f}  "
          f"p90={np.percentile(d, 90):.2f}  max={d.max():.2f}")

    print("\n按单张图平均差异排序 (越高越可能是有问题的样本):")
    for i, v in sorted(per_sample.items(), key=lambda kv: -np.mean(kv[1])):
        print(f"  图片 {i:>3}: {np.mean(v):6.2f} deg")

    if d.mean() < 2.0:
        verdict = "数据自洽 (建议阈值 mean<2°)"
    elif d.mean() < 5.0:
        verdict = "数据勉强可用, 建议剔除高差异样本或重新采集"
    else:
        verdict = "数据不自洽: 该组图片无法被单个刚性相机外参拟合, 结果必然不稳定 —— 需排查采集环节"
    print(f"\n结论: {verdict}")


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(1)
    args = sys.argv[1:]
    cols = int(args[1]) if len(args) > 1 else 8
    rows = int(args[2]) if len(args) > 2 else 6
    sq = float(args[3]) if len(args) > 3 else 0.025
    main(args[0], cols, rows, sq)
