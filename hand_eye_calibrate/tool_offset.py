#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""法兰位姿 <-> 工具点（夹爪尖端）位姿的换算。

为什么需要它
------------
机械臂上报和接受的都是**法兰**位姿，但真正要到达的点（夹爪尖端、吸盘、
焊枪尖）通常在法兰前方一段距离。设这段固定变换为

    T_offset = T_tool→flange      # 工具点在法兰坐标系里的位姿

则两种位姿的互换是

    T_tool→base  = T_flange→base · T_offset          # 法兰 -> 工具点
    T_flange→base = T_tool→base  · T_offset⁻¹        # 工具点 -> 法兰

注意方向别搞反：**法兰→工具 = 右乘 T_offset**。搞反的代价是 2×偏移量
（12cm 变 24cm），而且不会报错。

eye_to_hand 标定里这个偏移**不需要**加
--------------------------------------
标定板随法兰动时，`hand_eye_calibrate.py` 解出的 T_cam→base 与工具点
定义无关：把法兰位姿换成工具点位姿，未知量"板在工具点上的位置"会等量
反向补偿，乘积 T_法兰→base · K 不变。实测（H3003，49 张图）：

    eye_to_hand: --gripper-offset 加 12cm / 50cm / 换轴  ->  Δt = 0.0000 mm
    eye_in_hand: 同样加 12cm                              ->  Δt = 120.0000 mm

所以 12cm 要加在**使用**标定结果的那一步（把目标位姿转成下发给机械臂的
法兰位姿），不是加在标定输入里。本模块就是那一步。

位姿格式与项目其余部分一致：[x, y, z, rx, ry, rz]，单位米、弧度，
欧拉角约定 R = Rz(rz)·Ry(ry)·Rx(rx)（同 hand_eye_calibrate.py:55）。
"""

import argparse
import math
import sys

import numpy as np

# 与 hand_eye_calibrate.py 保持一致：|ry| 超过 pi/2 就不是 ZYX 能唯一表示的范围了
_GIMBAL_TOL = 1e-9


# --------------------------------------------------------------------------
# 欧拉角 <-> 旋转矩阵（ZYX，与 hand_eye_calibrate.py 同约定）
# --------------------------------------------------------------------------
def euler_to_rotation(rx, ry, rz):
    """[rx, ry, rz] -> 3x3 旋转矩阵，约定 R = Rz(rz)·Ry(ry)·Rx(rx)。"""
    cx, sx = math.cos(rx), math.sin(rx)
    cy, sy = math.cos(ry), math.sin(ry)
    cz, sz = math.cos(rz), math.sin(rz)
    return np.array([
        [cz * cy, cz * sy * sx - sz * cx, cz * sy * cx + sz * sx],
        [sz * cy, sz * sy * sx + cz * cx, sz * sy * cx - cz * sx],
        [-sy,     cy * sx,                cy * cx],
    ])


def rotation_to_euler(R):
    """3x3 旋转矩阵 -> [rx, ry, rz]，euler_to_rotation 的逆。

    万向锁（|ry| = pi/2）时 rz 取 0，把自由度全部记到 rx 上。
    项目的 check_pose_plausible 要求 |ry| <= pi/2，正常数据走不到这里。
    """
    R = np.asarray(R, dtype=np.float64)
    sy = -R[2, 0]
    if abs(sy) >= 1.0 - _GIMBAL_TOL:
        # cos(ry) == 0：只有 rx ± rz 有意义，约定 rz = 0
        ry = math.copysign(math.pi / 2, sy)
        rz = 0.0
        if sy < 0:
            rx = math.atan2(-R[0, 1], -R[0, 2])
        else:
            rx = math.atan2(R[0, 1], R[0, 2])
        return [rx, ry, rz]
    ry = math.asin(sy)
    rx = math.atan2(R[2, 1], R[2, 2])
    rz = math.atan2(R[1, 0], R[0, 0])
    return [rx, ry, rz]


def pose_to_matrix(pose):
    """[x, y, z, rx, ry, rz] -> 4x4 齐次矩阵。"""
    values = [float(v) for v in pose]
    if len(values) != 6:
        raise ValueError(f"位姿应为 6 个元素，实际得到 {len(values)} 个。")
    T = np.eye(4)
    T[:3, :3] = euler_to_rotation(values[3], values[4], values[5])
    T[:3, 3] = values[:3]
    return T


def matrix_to_pose(T):
    """4x4 齐次矩阵 -> [x, y, z, rx, ry, rz]。"""
    T = np.asarray(T, dtype=np.float64)
    if T.shape != (4, 4):
        raise ValueError(f"应为 4x4 矩阵，实际 {T.shape}。")
    xyz = T[:3, 3]
    rx, ry, rz = rotation_to_euler(T[:3, :3])
    return [float(xyz[0]), float(xyz[1]), float(xyz[2]), rx, ry, rz]


def invert_matrix(T):
    """刚体变换求逆（用 R^T，避免 np.linalg.inv 的数值误差）。"""
    T = np.asarray(T, dtype=np.float64)
    R, t = T[:3, :3], T[:3, 3]
    out = np.eye(4)
    out[:3, :3] = R.T
    out[:3, 3] = -R.T @ t
    return out


# --------------------------------------------------------------------------
# 换算本体
# --------------------------------------------------------------------------
def flange_to_tool(pose, offset):
    """法兰位姿 -> 工具点位姿：T_tool = T_flange · T_offset。"""
    return matrix_to_pose(pose_to_matrix(pose) @ pose_to_matrix(offset))


def tool_to_flange(pose, offset):
    """工具点位姿 -> 法兰位姿：T_flange = T_tool · T_offset⁻¹。

    这才是下发 move_p 之前要用的那一步。
    """
    return matrix_to_pose(pose_to_matrix(pose) @ invert_matrix(pose_to_matrix(offset)))


def axis_offset(length, axis="z"):
    """沿法兰某个轴延长 length 米的偏移，返回 [x, y, z, rx, ry, rz]。

    夹爪一般是沿法兰 +Z 伸出，即 axis_offset(0.12) == [0, 0, 0.12, 0, 0, 0]。
    """
    offset = [0.0] * 6
    index = {"x": 0, "y": 1, "z": 2}[axis.lower()]
    offset[index] = float(length)
    return offset


# --------------------------------------------------------------------------
# CLI
# --------------------------------------------------------------------------
def _read_poses(path):
    poses = []
    with open(path, encoding="utf-8") as f:
        for lineno, line in enumerate(f, start=1):
            if not line.strip():
                continue
            values = [float(v) for v in line.replace(";", ",").split(",")]
            if len(values) != 6:
                raise ValueError(f"{path} 第 {lineno} 行应为 6 列，实际 {len(values)} 列。")
            poses.append(values)
    return poses


def main():
    parser = argparse.ArgumentParser(
        description="法兰位姿 <-> 工具点（夹爪尖端）位姿换算。",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="例:\n"
               "  # 把 H3003 的法兰位姿都转成夹爪尖端位姿（沿法兰 Z 伸出 12cm）\n"
               "  python3 tool_offset.py H3003/poses.txt --offset 0 0 0.12 0 0 0 --to-tool\n"
               "  # 反方向：目标点在夹爪尖端，求该下发给机械臂的法兰位姿\n"
               "  python3 tool_offset.py target.txt --offset 0 0 0.12 0 0 0 --to-flange\n",
    )
    parser.add_argument("pose_file", help="输入位姿文件，每行 x,y,z,rx,ry,rz（米、弧度）")
    parser.add_argument("--offset", nargs=6, type=float, metavar=("X", "Y", "Z", "RX", "RY", "RZ"),
                        help="工具点在法兰坐标系里的位姿，例如沿法兰 Z 伸出 12cm 就是 0 0 0.12 0 0 0")
    direction = parser.add_mutually_exclusive_group(required=True)
    direction.add_argument("--to-tool", action="store_true",
                           help="法兰 -> 工具点：T_tool = T_flange · T_offset")
    direction.add_argument("--to-flange", action="store_true",
                           help="工具点 -> 法兰：T_flange = T_tool · T_offset⁻¹（下发 move_p 前用这个）")
    parser.add_argument("-o", "--out", default=None, help="输出文件；默认 <输入名>.tool.txt / .flange.txt")
    parser.add_argument("--in-place", action="store_true", help="覆盖输入文件（默认另存）")
    parser.add_argument("--demo-naive", action="store_true",
                        help="额外打印「直接在基座系里加 z」这种常见错法的偏差，用来确认自己没写错")
    args = parser.parse_args()

    if args.offset is None:
        parser.error("必须给出 --offset；沿法兰 Z 伸出 12cm 写 --offset 0 0 0.12 0 0 0")

    try:
        poses = _read_poses(args.pose_file)
    except (OSError, ValueError) as exc:
        print(f"读取失败: {exc}", file=sys.stderr)
        return 1

    convert = flange_to_tool if args.to_tool else tool_to_flange
    suffix = ".tool.txt" if args.to_tool else ".flange.txt"
    out_path = args.pose_file if args.in_place else (args.out or args.pose_file + suffix)
    if args.in_place and args.out:
        parser.error("--in-place 与 -o 不能同时使用。")

    converted = [convert(p, args.offset) for p in poses]

    with open(out_path, "w", encoding="utf-8") as f:
        for pose in converted:
            f.write(",".join(repr(v) for v in pose) + "\n")

    direction_name = "法兰 -> 工具点" if args.to_tool else "工具点 -> 法兰"
    print(f"方向: {direction_name}")
    print(f"偏移: {args.offset}")
    print(f"已写入: {out_path}（{len(converted)} 行）")

    # 自检：换回去应当逐位还原
    back = [(tool_to_flange if args.to_tool else flange_to_tool)(p, args.offset) for p in converted]
    worst = max(np.linalg.norm(np.array(a) - np.array(b)) for a, b in zip(poses, back))
    print(f"往返自检最大误差: {worst:.3e}")

    print("\n前两行对照:")
    for pose, new in list(zip(poses, converted))[:2]:
        print(f"  入: [{', '.join(f'{v:+.6f}' for v in pose)}]")
        print(f"  出: [{', '.join(f'{v:+.6f}' for v in new)}]")

    if args.demo_naive:
        # 常见错法：不管姿态，直接在基座系里把 z 加 12cm。
        # 只有法兰 Z 轴恰好和基座 Z 轴平行时才对，其余姿态全错。
        dz = args.offset[2]
        diffs = [np.linalg.norm(np.array(convert(p, args.offset)) - (np.array(p) + np.array([0, 0, dz, 0, 0, 0])))
                 for p in poses]
        print(f"\n[对比] 「直接在基座系加 z={dz}」的错法："
              f"平均偏 {np.mean(diffs) * 1000:.1f} mm，最大偏 {np.max(diffs) * 1000:.1f} mm")
        print("       它只在法兰 Z 与基座 Z 平行时才正确，别用。")
    return 0


if __name__ == "__main__":
    sys.exit(main())
