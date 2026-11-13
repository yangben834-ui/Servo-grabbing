#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""采集过程中的"标定板是否还在原位"实时检测。

判据（与坐标系无关，不需要知道相机外参）
--------------------------------------
标定板刚性固定在法兰上、相机固定不动时，取任意两张图 i、j：

    法兰的相对转角 dF_ij = angle(R_fj · R_fiᵀ)
    板在相机系里的相对转角 dB_ij = angle(R_bj · R_biᵀ)

这两个角必须相等 —— 它们描述的是同一个物理转动，只是分别在基座系和相机系里看。
坐标系、基座系定义、法兰/TCP 定义都影响不了它。

H3003 实测（32 张，2026-10-08）
    同一段内：0.1 ~ 2.3°（好数据）
    板被翻到支架另一面后：27 ~ 133°
区分度极大，所以阈值取 2°（与 check_calib_consistency.py 一致的判据）。

**局限（重要）**：这个判据只看旋转。标定板如果发生**纯平移**滑动而不转，
本模块看不出来 —— 那要靠标定后的 validate_calibration.py 的外参残差来发现。
原理同 check_calib_consistency.py 的说明。

用法
----
    # 审计一个已有数据目录：逐张报告，并指出从第几张开始出问题
    /usr/bin/python3 calib_drift.py H3003
    /usr/bin/python3 calib_drift.py H3003 --min-rot-deg 15 --tol 2.0

    # 采集脚本内部按张调用（见 data_collect_orbbec_ros.py --consistency-gate）
    from calib_drift import DriftMonitor
    mon = DriftMonitor.from_data_dir(data_dir)
    v = mon.check(new_sample_index)      # 返回 verdict
"""

import argparse
import math
import os
import sys

import cv2
import numpy as np
import yaml

# 判据默认值
DEFAULT_TOL_DEG = 2.0        # 单对"法兰转角-板转角"之差的上限
DEFAULT_BAD_DEG = 5.0        # 超过它直接判"板动过"
DEFAULT_MIN_ROT_DEG = 15.0   # 相对转角小于它的一对没有信息量，不计入
DEFAULT_BOARD = (8, 6)
DEFAULT_SQUARE_M = 0.025


# --------------------------------------------------------------------------
# 基础几何（与 hand_eye_calibrate.py 同一约定 R = Rz·Ry·Rx）
# --------------------------------------------------------------------------
def euler_to_rotation(rx, ry, rz):
    Rx = np.array([[1, 0, 0],
                   [0, math.cos(rx), -math.sin(rx)],
                   [0, math.sin(rx), math.cos(rx)]])
    Ry = np.array([[math.cos(ry), 0, math.sin(ry)],
                   [0, 1, 0],
                   [-math.sin(ry), 0, math.cos(ry)]])
    Rz = np.array([[math.cos(rz), -math.sin(rz), 0],
                   [math.sin(rz), math.cos(rz), 0],
                   [0, 0, 1]])
    return Rz @ Ry @ Rx


def rotation_angle_deg(R):
    return float(np.degrees(np.arccos(np.clip((np.trace(R) - 1.0) / 2.0, -1.0, 1.0))))


def detect_board_rotation(gray, K, dist, board_size=DEFAULT_BOARD,
                          square_m=DEFAULT_SQUARE_M, criteria=None):
    """检测棋盘并返回该板在相机系下的旋转矩阵；检测失败返回 None。"""
    if criteria is None:
        criteria = (cv2.TERM_CRITERIA_MAX_ITER | cv2.TERM_CRITERIA_EPS, 30, 0.001)
    cols, rows = board_size
    ret, corners = cv2.findChessboardCorners(gray, (cols, rows), None)
    if not ret or corners is None:
        return None
    corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)

    objp = np.zeros((cols * rows, 3), np.float32)
    objp[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2) * square_m
    ok, rvec, _ = cv2.solvePnP(objp, corners, K, dist)
    if not ok:
        return None
    R, _ = cv2.Rodrigues(rvec)
    return R


# --------------------------------------------------------------------------
# 数据读取
# --------------------------------------------------------------------------
def image_names_in_order(data_dir):
    """按数字大小排序的图片名列表（与 hand_eye_calibrate / validate 的规则一致）。"""
    names = [f for f in os.listdir(data_dir)
             if f.lower().endswith((".jpg", ".jpeg", ".png"))]

    def key(name):
        base = os.path.splitext(name)[0]
        return (0, int(base)) if base.isdigit() else (1, base)

    return sorted(names, key=key)


def load_poses(path):
    poses = []
    with open(path, "r", encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            values = [float(v) for v in line.split(",")]
            if len(values) != 6:
                raise ValueError(f"位姿行应为 6 个数: {line!r}")
            poses.append(values)
    return poses


def load_intrinsics(data_dir, explicit=None):
    """优先用标定得到的内参，其次用 ROS CameraInfo 存的 color_camera_info.yaml。

    采集过程中 camera_intrinsics.yaml 还不存在（它是标定后才生成的），
    所以这条路是给"边采边查"用的。
    """
    cands = []
    if explicit:
        cands.append(explicit)
    cands += [os.path.join(data_dir, "camera_intrinsics.yaml"),
              os.path.join(data_dir, "color_camera_info.yaml")]
    for path in cands:
        if not os.path.exists(path):
            continue
        with open(path, "r", encoding="utf-8") as f:
            data = yaml.safe_load(f)
        if not isinstance(data, dict) or "K" not in data:
            continue
        K = np.asarray(data["K"], dtype=np.float64).reshape(3, 3)
        D = np.asarray(data.get("D", [0.0] * 5), dtype=np.float64).reshape(-1)
        return K, D, path
    raise FileNotFoundError(
        f"在 {data_dir} 里找不到内参（camera_intrinsics.yaml 或 color_camera_info.yaml）。\n"
        "采集阶段可以先启动相机节点让它把 color_camera_info.yaml 写出来，"
        "或用 --intrinsics 指定。"
    )


def pair_stats(R_flange_new, R_board_new, R_flange_ref, R_board_ref,
               tol_deg=DEFAULT_TOL_DEG, min_rot_deg=DEFAULT_MIN_ROT_DEG):
    """返回 (n_pairs, diffs 列表)。只统计相对转角够大的配对。"""
    diffs = []
    for Rf, Rb in zip(R_flange_ref, R_board_ref):
        dF = rotation_angle_deg(R_flange_new @ Rf.T)
        if dF < min_rot_deg:
            continue
        dB = rotation_angle_deg(R_board_new @ Rb.T)
        diffs.append(abs(dF - dB))
    return len(diffs), diffs


def verdict_from_diffs(sample_index, diffs,
                       tol_deg=DEFAULT_TOL_DEG, bad_deg=DEFAULT_BAD_DEG):
    """把一组配对差异判成结论。"""
    if len(diffs) == 0:
        return {"index": sample_index, "status": "no_evidence", "n_pairs": 0,
                "median": None, "max": None, "ok_ratio": None,
                "text": "证据不足：还没有与它相对转角 ≥ 15° 的历史样本，无法判断"}
    arr = np.asarray(diffs, dtype=np.float64)
    median = float(np.median(arr))
    mx = float(arr.max())
    ok_ratio = float((arr < tol_deg).mean())

    if median <= tol_deg and mx <= bad_deg:
        status, text = "ok", "与历史样本一致"
    elif median >= bad_deg or ok_ratio < 0.5:
        status = "moved"
        text = ("** 标定板相对法兰动过（或被重新装夹）**"
                f" —— 与新样本配对的中位差异 {median:.1f}°、最大 {mx:.1f}°，"
                f"仅 {ok_ratio * 100:.0f}% 的配对在 {tol_deg:.1f}° 以内")
    else:
        status = "suspect"
        text = (f"可疑：中位差异 {median:.1f}°、最大 {mx:.1f}°"
                f"（{ok_ratio * 100:.0f}% 在 {tol_deg:.1f}° 以内）")
    return {"index": sample_index, "status": status, "n_pairs": len(arr),
            "median": median, "max": mx, "ok_ratio": ok_ratio, "text": text}


# --------------------------------------------------------------------------
# 实时监测：每采一张就查一次
# --------------------------------------------------------------------------
class DriftMonitor:
    """增量式的"板还在原位吗"检测器。

    只保留"当前这一段"的参考样本：一旦判定板动过，就以新样本重新开一段，
    这样不会因为一次误判把后面全部拖黑，而且能明确告诉你边界在哪一张。
    """

    def __init__(self, K, dist, board_size=DEFAULT_BOARD, square_m=DEFAULT_SQUARE_M,
                 tol_deg=DEFAULT_TOL_DEG, bad_deg=DEFAULT_BAD_DEG,
                 min_rot_deg=DEFAULT_MIN_ROT_DEG):
        self.K = K
        self.dist = dist
        self.board_size = board_size
        self.square_m = square_m
        self.tol_deg = tol_deg
        self.bad_deg = bad_deg
        self.min_rot_deg = min_rot_deg
        self.segment_start = None          # 当前段从第几张开始
        self.ref_flange = []               # 当前段的参考位姿
        self.ref_board = []
        self.history = []                  # 每次 check 的 verdict

    @classmethod
    def from_data_dir(cls, data_dir, intrinsics=None, **kwargs):
        K, dist, _ = load_intrinsics(data_dir, intrinsics)
        return cls(K, dist, **kwargs)

    def check_rotation(self, index, R_flange, R_board, *, reset_on_move=True):
        """喂入一张已经算好旋转的样本，返回 verdict。"""
        n, diffs = pair_stats(R_flange, R_board, self.ref_flange, self.ref_board,
                              tol_deg=self.tol_deg, min_rot_deg=self.min_rot_deg)
        v = verdict_from_diffs(index, diffs, self.tol_deg, self.bad_deg)
        v["segment_start"] = self.segment_start
        self.history.append(v)

        if v["status"] == "moved":
            if reset_on_move:
                # 以它为起点重开一段，后续按新安装状态比对
                self.segment_start = index
                self.ref_flange = [R_flange]
                self.ref_board = [R_board]
        else:
            if self.segment_start is None:
                self.segment_start = index
            self.ref_flange.append(R_flange)
            self.ref_board.append(R_board)
        return v

    def save_state(self):
        """快照当前状态，供 restore_state 回滚。

        用法：采一张 → save_state() → check_image() → 若决定丢弃这张，
        restore_state()，这样被判为"板动过"的这张就不会污染参考集
        （check_rotation 判 moved 时会**替换**参考集、以它为新段起点，
        所以这里必须整份拷下来，不能用长度截断来回滚）。
        """
        return (self.segment_start, list(self.ref_flange), list(self.ref_board),
                len(self.history))

    def restore_state(self, state):
        """回滚到 save_state() 那一刻（丢弃最后一次 check 的影响）。"""
        seg_start, ref_flange, ref_board, n_hist = state
        self.segment_start = seg_start
        self.ref_flange = list(ref_flange)
        self.ref_board = list(ref_board)
        del self.history[n_hist:]

    def warm_up(self, data_dir, verbose=False):
        """把目录里已有的样本先喂一遍（用于"继续采集"时接上当前的段）。

        不做任何输出（verbose=False 时），只用来恢复"当前段"的状态，
        否则续采的第一张会被当成段起点，丢掉与已有数据的比对。
        返回已喂入的张数。
        """
        names = image_names_in_order(data_dir)
        pose_path = os.path.join(data_dir, "poses.txt")
        if not os.path.exists(pose_path):
            return 0
        poses = load_poses(pose_path)

        # 图片数与位姿行数必须一致：hand_eye_calibrate / validate_calibration /
        # check_calib_consistency 都是按"图片序号 == 位姿行号"来配对的，
        # 差一行就会让后面所有样本静默错配（而且不报错）。
        if len(names) != len(poses):
            print(f"⚠ 数据目录本身已经错位：图片 {len(names)} 张，但 poses.txt 只有 "
                  f"{len(poses)} 行。两者必须一一对应，否则标定会静默用错位姿。")
            print(f"  请先修好再采（多半是某次异常中断只写了图或只写了位姿）：{data_dir}")

        n = 0
        for i, name in enumerate(names):
            if i >= len(poses):
                break
            v = self.check_image(i, os.path.join(data_dir, name), poses[i])
            if verbose and v["status"] == "moved":
                print(f"  [续采预热] 图 {i} 判为板动过，已在此重开一段")
            n += 1
        return n

    def segment_pair_diffs(self):
        """当前段内所有两两配对的"法兰转角-板转角"之差（度）。"""
        diffs = []
        for a in range(len(self.ref_flange)):
            for b in range(a + 1, len(self.ref_flange)):
                dF = rotation_angle_deg(self.ref_flange[b] @ self.ref_flange[a].T)
                if dF < self.min_rot_deg:
                    continue
                dB = rotation_angle_deg(self.ref_board[b] @ self.ref_board[a].T)
                diffs.append(abs(dF - dB))
        return diffs

    def segment_summary(self):
        """当前段的统计，数值口径与 check_calib_consistency.py 一致（段内全部配对）。

        采集过程中相机内参还没标定，而 check_calib_consistency.py 每次调用都会
        用 calibrateCamera 重算内参，所以这里用增量方式给出同一个量，避免反复重算。
        """
        arr = np.asarray(self.segment_pair_diffs(), dtype=np.float64)
        if arr.size == 0:
            return {"n": 0, "mean": None, "median": None, "p90": None, "max": None}
        return {"n": int(arr.size), "mean": float(arr.mean()),
                "median": float(np.median(arr)),
                "p90": float(np.percentile(arr, 90)), "max": float(arr.max())}

    def check_image(self, index, image_path, pose6):
        """读图 + 检测 + 判定的便捷入口。检测失败返回 status='no_board'。"""
        img = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)
        if img is None:
            return {"index": index, "status": "no_image", "text": f"读不到 {image_path}"}
        R_board = detect_board_rotation(img, self.K, self.dist,
                                        self.board_size, self.square_m)
        if R_board is None:
            return {"index": index, "status": "no_board", "text": "棋盘未检出"}
        R_flange = euler_to_rotation(pose6[3], pose6[4], pose6[5])
        return self.check_rotation(index, R_flange, R_board)


# --------------------------------------------------------------------------
# 审计一个已有目录
# --------------------------------------------------------------------------
def audit(data_dir, intrinsics=None, board_size=DEFAULT_BOARD,
          square_m=DEFAULT_SQUARE_M, tol_deg=DEFAULT_TOL_DEG,
          bad_deg=DEFAULT_BAD_DEG, min_rot_deg=DEFAULT_MIN_ROT_DEG,
          quiet=False):
    """逐张检查，返回 {"verdicts": [...], "boundaries": [...], "n_ok": n}。"""
    K, dist, src = load_intrinsics(data_dir, intrinsics)
    names = image_names_in_order(data_dir)
    pose_path = os.path.join(data_dir, "poses.txt")
    poses = load_poses(pose_path)

    if not quiet:
        print(f"目录: {data_dir}")
        print(f"内参来源: {src}")
        print(f"图片 {len(names)} 张, 位姿 {len(poses)} 行")
        if len(names) != len(poses):
            print(f"⚠ 图片数与位姿行数不一致 —— 配对本身可能已经错位，先解决这个")
        print()

    mon = DriftMonitor(K, dist, board_size, square_m, tol_deg, bad_deg, min_rot_deg)
    verdicts = []
    boundaries = []

    for i, name in enumerate(names):
        if i >= len(poses):
            break
        v = mon.check_image(i, os.path.join(data_dir, name), poses[i])
        v["image"] = name
        verdicts.append(v)
        if not quiet:
            if v["status"] == "no_board":
                print(f"  图 {i:3d}: 棋盘未检出")
            elif v["status"] == "no_evidence":
                print(f"  图 {i:3d}: {v['text']}")
            else:
                seg = v.get("segment_start")
                seg_txt = f" [第{seg}张起]" if v["status"] == "moved" and seg == i else ""
                print(f"  图 {i:3d}: n={v['n_pairs']:3d} "
                      f"中位={v['median']:6.2f}° 最大={v['max']:7.2f}° "
                      f"达标={v['ok_ratio'] * 100:5.1f}%  {v['status']}{seg_txt}")
                if v["status"] == "moved":
                    boundaries.append(i)
                    print(f"        {v['text']}")

    n_ok = sum(1 for v in verdicts if v["status"] == "ok")
    if not quiet:
        print()
        if boundaries:
            print(f"结论: 检测到 {len(boundaries)} 处标定板相对法兰发生变化，"
                  f"发生在图 {boundaries}。")
            print(f"      这些位置把数据切成了 {len(boundaries) + 1} 段互不兼容的集合，"
                  "**必须分成若干组分别标定，或重采**。")
            print("      直接混在一起解，外参会完全不可信（H3003 实测偏了 331 mm / 154°）。")
        else:
            print(f"结论: 全部 {len(verdicts)} 张自洽，未发现标定板位移。")
    return {"verdicts": verdicts, "boundaries": boundaries, "n_ok": n_ok}


def parse_arguments(argv=None):
    p = argparse.ArgumentParser(
        description="检测采集过程中标定板是否相对法兰移动过（只看旋转，与坐标系无关）。")
    p.add_argument("data_dir", help="包含图片和 poses.txt 的目录")
    p.add_argument("--intrinsics", default=None,
                   help="内参 yaml 路径；默认优先 data_dir/camera_intrinsics.yaml，"
                        "否则用 color_camera_info.yaml")
    p.add_argument("--board-cols", type=int, default=DEFAULT_BOARD[0])
    p.add_argument("--board-rows", type=int, default=DEFAULT_BOARD[1])
    p.add_argument("--square-size", type=float, default=DEFAULT_SQUARE_M,
                   help="棋盘格边长，单位米")
    p.add_argument("--tol", type=float, default=DEFAULT_TOL_DEG,
                   help="单对差异的上限（度），默认 2.0")
    p.add_argument("--bad", type=float, default=DEFAULT_BAD_DEG,
                   help="中位差异超过它就判'板动过'（度），默认 5.0")
    p.add_argument("--min-rot-deg", type=float, default=DEFAULT_MIN_ROT_DEG,
                   help="相对转角小于它的配对不计入（没有信息量），默认 15")
    return p.parse_args(argv)


def main(argv=None):
    args = parse_arguments(argv)
    if not os.path.isdir(args.data_dir):
        print(f"目录不存在: {args.data_dir}")
        return 1
    result = audit(args.data_dir, args.intrinsics,
                   (args.board_cols, args.board_rows), args.square_size,
                   args.tol, args.bad, args.min_rot_deg)
    return 1 if result["boundaries"] else 0


if __name__ == "__main__":
    sys.exit(main())
