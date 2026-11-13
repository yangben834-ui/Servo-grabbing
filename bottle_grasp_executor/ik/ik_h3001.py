#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# ─────────────────────────────────────────────────────────────────────────────
# **VENDORED 副本 —— 不要在本仓库里单独改它。**
#
# 来源: /home/zdl/pinocchio-devel/SDK_linux/ik_h3001.py
# 拷贝日期: 2026-09-28
# 更新方式: 从上游**整文件覆盖拷贝**, 保持逐字一致 (这样 diff 一眼能看出上游有没有改动)。
#
# 为什么拷进来: 执行节点要用它的 `load_model` / `ik` / `cad_limits` /
# `forward_kinematics`, 而上游在工作区外的开发目录里, 部署时不能依赖那条路径。
# (同一个理由见 setup.py 里 data_files 那段注释。)
#
# 配套的两个文件 (同目录, 也是拷来的):
#   L4002-0100.urdf            —— **只拷 urdf, 不要拷 21MB 的 STL**:
#                                 pin.buildModelFromUrdf 只读 XML, 不加载网格 (2026-09-28 实测)
#   L4002-0100-cad-limits.csv  —— CAD 真机械限位。原名带中文/括号/空格, 拷过来时改成
#                                 ASCII, 免得在无中文 locale 的机器上 colcon 拷付出问题
#
# ⚠ 节点必须用 `ik(..., limits=(cad_lo, cad_hi))` 把 **CAD 限位喂进求解器**, 不能用
#   `ik()` 的默认 (URDF 限位: J2/J3 ±180、J5 ±100.3 —— 比真限位松, J5 松 30°)。
#   理由与实测见 README §5.15 与节点里 `_load_ik_config` / `_solve_joint_target` 的注释。
# ─────────────────────────────────────────────────────────────────────────────
"""H3001 (L4002-0100) 六轴机械臂逆运动学：末端位姿 -> 关节角。

基于 pinocchio + urdf/L4002-0100.urdf，用带关节限位约束的阻尼最小二乘
(scipy least_squares, TRF) 求解，多起点重启避免肘部翻转。

位姿约定与 SDK 的 ``ArmState.pose`` 一致：
    [X mm, Y mm, Z mm, RX deg, RY deg, RZ deg]，XYZ 为 URDF ZYX-RPY 欧拉角。

常用命令（在 SDK_linux 目录下运行）：
    python ik_h3001.py fk 0 0 0 0 0 0                 # 正解：关节角(deg) -> 位姿
    python ik_h3001.py ik 76 0 181 90 0 90            # 逆解：位姿 -> 关节角(deg)
    python ik_h3001.py ik 200 0 300 180 0 90 --seed 0 0 0 0 0 0
    python ik_h3001.py calibrate --channel can0       # 对比 MCU 位姿与 URDF 正解

注意：本目录的 pinocchio 在 conda base (Python 3.13)。若 shell 里设置了 ROS
的 PYTHONPATH，会串到 /opt/ros/humble 的 Python 3.10 版本而导入失败，此时用
    env -u PYTHONPATH python ik_h3001.py ...
"""

from __future__ import annotations

import argparse
import csv
import math
import sys
from pathlib import Path

import numpy as np
import pinocchio as pin
from scipy.optimize import least_squares

HERE = Path(__file__).resolve().parent
DEFAULT_URDF = HERE / "urdf" / "L4002-0100.urdf"
DEFAULT_CSV = HERE / "urdf" / "L4002-0100-六轴行星臂总成(带手爪)__URDF.csv"
EE_FRAME = "link_6"

POS_SCALE = 1000.0  # m  -> mm
ROT_SCALE = 180.0 / math.pi  # rad -> deg

EULER_CONVENTIONS = ("zyx", "xyz", "yxz", "yzx", "zxy", "xzy")


# --------------------------------------------------------------------------- #
# 模型与位姿转换
# --------------------------------------------------------------------------- #
def load_model(urdf: Path | str = DEFAULT_URDF, ee_frame: str = EE_FRAME):
    """返回 (model, data, frame_id)。frame_id 为末端 (默认 link_6) 的 frame 索引。"""
    model = pin.buildModelFromUrdf(str(urdf))
    if model.nq != 6:
        raise ValueError(f"期望 6 自由度模型，实际 nq={model.nq}")
    fid = model.getFrameId(ee_frame)
    if fid >= len(model.frames):
        names = [f.name for f in model.frames]
        raise ValueError(f"URDF 中找不到 frame {ee_frame!r}，可选: {names}")
    return model, model.createData(), fid


def pose_to_se3(pose, euler: str = "zyx", tool=None) -> pin.SE3:
    """位姿 [x mm, y mm, z mm, rx deg, ry deg, rz deg] -> SE3（米）。

    tool 为可选的末端固定变换 [x mm, y mm, z mm, rx deg, ry deg, rz deg]，
    即位姿描述的是 "link_6 再叠加 tool" 的坐标系。
    """
    x, y, z, rx, ry, rz = (float(v) for v in pose)
    R = pin.rpy.rpyToMatrix(np.radians(_as_zyx([rx, ry, rz], euler)))
    M = pin.SE3(R, np.array([x, y, z]) / POS_SCALE)
    if tool is not None:
        M = M * pose_to_se3(tool, euler="zyx")
    return M


def se3_to_pose(M: pin.SE3, euler: str = "zyx", tool=None):
    """SE3 -> [x mm, y mm, z mm, rx deg, ry deg, rz deg]。"""
    if tool is not None:
        M = M * pose_to_se3(tool, euler="zyx").inverse()
    rpy = np.degrees(pin.rpy.matrixToRpy(M.rotation))
    return np.concatenate([M.translation * POS_SCALE, _from_zyx(rpy, euler)])


def _as_zyx(rpy, euler: str):
    """把任意欧拉序的角度统一成 ZYX-RPY 三元组（供 rpyToMatrix 使用）。"""
    if euler == "zyx":
        return rpy
    from scipy.spatial.transform import Rotation

    return Rotation.from_euler(euler.upper(), rpy, degrees=True).as_euler("ZYX", degrees=True)


def _from_zyx(rpy, euler: str):
    """ZYX-RPY 三元组转换成指定欧拉序的角度。"""
    if euler == "zyx":
        return rpy
    from scipy.spatial.transform import Rotation

    return Rotation.from_euler("ZYX", rpy, degrees=True).as_euler(euler.upper(), degrees=True)


# --------------------------------------------------------------------------- #
# 正解
# --------------------------------------------------------------------------- #
def forward_kinematics(model, data, fid, q_deg, euler: str = "zyx", tool=None):
    """关节角 (deg, J1~J6) -> 位姿 [mm, mm, mm, deg, deg, deg]。"""
    pin.forwardKinematics(model, data, np.radians(np.asarray(q_deg, dtype=float)))
    pin.updateFramePlacements(model, data)
    return se3_to_pose(data.oMf[fid], euler=euler, tool=tool)


# --------------------------------------------------------------------------- #
# 关节限位
# --------------------------------------------------------------------------- #
def urdf_limits(model):
    """URDF 中写的限位（J2/J3 偏宽松，仅用于求解边界）。"""
    return model.lowerPositionLimit.copy(), model.upperPositionLimit.copy()


def cad_limits(csv_path: Path | str = DEFAULT_CSV, model=None):
    """从 CAD 导出的 CSV 读取真实机械限位（弧度），按 URDF 关节顺序排列。"""
    rows = {}
    with open(csv_path, newline="", encoding="utf-8-sig") as stream:
        for row in csv.DictReader(stream):
            name = (row.get("Joint Name") or "").strip()
            if not name:
                continue
            try:
                rows[name] = (float(row["Limit Lower"]), float(row["Limit Upper"]))
            except (KeyError, TypeError, ValueError):
                continue
    order = [n for n in model.names[1:]] if model is not None else list(rows)
    lo = np.array([rows.get(n, (-math.pi, math.pi))[0] for n in order])
    hi = np.array([rows.get(n, (-math.pi, math.pi))[1] for n in order])
    return lo, hi, order


# --------------------------------------------------------------------------- #
# 逆解
# --------------------------------------------------------------------------- #
def ik(
    model,
    data,
    fid,
    target_pose,
    *,
    seed_deg=None,
    euler: str = "zyx",
    tool=None,
    limits=None,
    tol_mm: float = 0.1,
    tol_deg: float = 0.1,
    restarts: int = 32,
    jitter_deg: float = 25.0,
    max_nfev: int = 200,
    random_state: int = 0,
):
    """末端位姿 -> 关节角。

    target_pose: [x mm, y mm, z mm, rx deg, ry deg, rz deg]
    seed_deg:    初值（通常是 arm.get_state().joint_deg），决定选中哪个构型
    limits:      (lower_rad, upper_rad)，默认用 URDF 限位；可用 cad_limits() 收紧
    restarts:    除初值外额外尝试的随机起点个数

    返回 dict: ok / q_deg / pos_err_mm / rot_err_deg / iters / cost
    """
    target = pose_to_se3(target_pose, euler=euler, tool=tool)
    lo, hi = limits if limits is not None else urdf_limits(model)
    lo = np.asarray(lo, dtype=float)
    hi = np.asarray(hi, dtype=float)
    # 允许 ±360° 的关节（URDF 里的 ±pi 附近）向内收一点，避免退化解
    lo = np.maximum(lo, -2 * math.pi)
    hi = np.minimum(hi, 2 * math.pi)

    def residual(q):
        pin.forwardKinematics(model, data, q)
        pin.updateFramePlacements(model, data)
        err = pin.log6(data.oMf[fid].actInv(target)).vector
        return np.concatenate([err[:3] * POS_SCALE, err[3:] * ROT_SCALE])

    def solve(q0):
        q0 = np.clip(np.asarray(q0, dtype=float), lo, hi)
        result = least_squares(
            residual,
            q0,
            bounds=(lo, hi),
            xtol=1e-12,
            ftol=1e-12,
            gtol=1e-12,
            max_nfev=max_nfev,
            diff_step=1e-6,
        )
        cost = float(np.linalg.norm(residual(result.x)))
        return result, cost

    rng = np.random.default_rng(random_state)
    seed = np.asarray(seed_deg, dtype=float) if seed_deg is not None else None
    if seed is not None:
        seed = np.radians(seed)
        seed = np.clip(seed, lo, hi)

    candidates = []
    starts = []
    if seed is not None:
        starts.append(seed)
        for _ in range(max(1, restarts // 2)):  # 初值附近微调：保持构型不变
            starts.append(np.clip(seed + np.radians(rng.uniform(-jitter_deg, jitter_deg, model.nq)), lo, hi))
    for _ in range(restarts - len(starts) + 1):  # 全空间随机：寻找其它可行构型
        starts.append(rng.uniform(lo, hi))

    for q0 in starts:
        result, cost = solve(q0)
        err = residual(result.x)
        pos_err = float(np.linalg.norm(err[:3]))
        rot_err = float(np.linalg.norm(err[3:]))
        candidates.append(
            {
                "ok": pos_err <= tol_mm and rot_err <= tol_deg,
                "q_rad": result.x.copy(),
                "pos_err_mm": pos_err,
                "rot_err_deg": rot_err,
                "nfev": int(result.nfev),
                "cost": cost,
            }
        )

    # 收敛的解一律等价，取离初值最近的构型（避免无谓的肘部/腕部翻转）；
    # 都不收敛时才比末端误差。
    def rank(item):
        if item["ok"]:
            dist = float(np.linalg.norm(item["q_rad"] - seed)) if seed is not None else 0.0
            return (0, dist)
        return (1, item["cost"])

    best = min(candidates, key=rank)
    best["q_deg"] = np.degrees(best.pop("q_rad"))
    best["n_solutions"] = sum(1 for c in candidates if c["ok"])
    return best


def check_limits(q_deg, lo_rad, hi_rad, names=None, tol_deg: float = 0.5):
    """返回超出限位的关节描述列表（CAD 真实限位检查）。"""
    q = np.radians(np.asarray(q_deg, dtype=float))
    tol = math.radians(tol_deg)
    out = []
    for i, (value, low, high) in enumerate(zip(q, lo_rad, hi_rad)):
        if value < low - tol or value > high + tol:
            label = names[i] if names else f"J{i + 1}"
            out.append(
                f"{label}={math.degrees(value):+.1f}° 超出 "
                f"[{math.degrees(low):+.1f}°, {math.degrees(high):+.1f}°]"
            )
    return out


# --------------------------------------------------------------------------- #
# 命令行
# --------------------------------------------------------------------------- #
def _fmt_pose(pose):
    return "[" + ", ".join(f"{v:10.4f}" for v in pose) + "]"


def _fmt_joints(q_deg):
    return "[" + ", ".join(f"{v:8.3f}" for v in q_deg) + "]"


def cmd_fk(args):
    model, data, fid = load_model(args.urdf, args.frame)
    pose = forward_kinematics(model, data, fid, args.joints, euler=args.euler, tool=args.tool)
    print("关节角 (deg):", _fmt_joints(args.joints))
    print("末端位姿 [X Y Z RX RY RZ] (mm, deg):", _fmt_pose(pose))
    return 0


def cmd_ik(args):
    model, data, fid = load_model(args.urdf, args.frame)
    if args.limits == "cad":
        lo, hi, names = cad_limits(args.csv, model)
    else:
        lo, hi = urdf_limits(model)
        names = [n for n in model.names[1:]]

    result = ik(
        model,
        data,
        fid,
        args.pose,
        seed_deg=args.seed,
        euler=args.euler,
        tool=args.tool,
        limits=(lo, hi),
        tol_mm=args.tol_mm,
        tol_deg=args.tol_deg,
        restarts=args.restarts,
    )

    print("目标位姿      :", _fmt_pose(args.pose))
    print(f"求解器限位    : {args.limits} ({', '.join(f'{math.degrees(l):.1f}..{math.degrees(h):.1f}' for l, h in zip(lo, hi))})")
    print("逆解关节角(deg):", _fmt_joints(result["q_deg"]))
    print(f"末端误差      : {result['pos_err_mm']:.4f} mm, {result['rot_err_deg']:.4f} deg")
    print(f"迭代/可行解数 : {result['nfev']} / {result['n_solutions']}")
    if not result["ok"]:
        print("⚠ 未收敛到容差内：目标可能不可达、超出限位或位于奇异点附近。")

    bad = check_limits(result["q_deg"], *cad_limits(args.csv, model)[:2], names=names)
    if bad:
        print("⚠ 超出 CAD 机械限位（URDF 限位偏宽松）:")
        for line in bad:
            print("   ", line)

    # 用正解回代，确认闭环
    verify = forward_kinematics(model, data, fid, result["q_deg"], euler=args.euler, tool=args.tool)
    print("正解回代      :", _fmt_pose(verify))
    if result["ok"]:
        print("\n可直接下发:")
        print("    arm.send_position_targets([%.3f, %.3f, %.3f, %.3f, %.3f, %.3f])" % tuple(result["q_deg"]))
    return 0 if result["ok"] else 1


def cmd_calibrate(args):
    """连接机械臂，比较 MCU 上报位姿与 URDF 正解，标定坐标系/零位约定。"""
    sys.path.insert(0, str(HERE))
    try:
        from sdk_linux import H3001Arm
    except ImportError as exc:
        print(f"无法导入 sdk_linux（{exc}）。请先 pip install -r requirements.txt", file=sys.stderr)
        return 1

    model, data, fid = load_model(args.urdf, args.frame)
    arm = H3001Arm(interface=args.interface, channel=args.channel, bitrate=args.bitrate, fd=False)
    arm.connect(start_listener=True, wait_feedback=True, timeout=args.timeout)
    try:
        state = arm.get_state()
        q_mcu = np.array(state.joint_deg)
        pose_mcu = np.array(state.pose)
        pose_fk = forward_kinematics(model, data, fid, q_mcu, euler=args.euler, tool=args.tool)

        print("MCU 关节角 (deg) :", _fmt_joints(q_mcu))
        print("MCU 位姿         :", _fmt_pose(pose_mcu))
        print("URDF 正解        :", _fmt_pose(pose_fk))
        dpose = pose_fk - pose_mcu
        print("差值 (正解-MCU)  :", _fmt_pose(dpose))
        print(f"位置偏差         : {np.linalg.norm(dpose[:3]):.3f} mm")
        print(f"姿态偏差         : {np.linalg.norm(dpose[3:]):.3f} deg")

        if np.linalg.norm(dpose[:3]) > 2.0:
            print("\n→ 位置差 > 2 mm：URDF 与 MCU 的零位或连杆参数不一致，")
            print("  请把机械臂摆到关节全零（可用 return_zero()）后重测。")
        if np.linalg.norm(dpose[3:]) > 2.0:
            print("\n→ 姿态差 > 2°：欧拉角约定或末端坐标系不同。")
            print("  试试 --euler xyz/yxz/yzx/zxy/xzy，或用 --tool 给出末端固定偏置：")
            print("  例如只差绕末端 Z 轴的固定角，可用 --tool 0 0 0 0 0 <角度>")
    finally:
        arm.close()
    return 0


def main(argv=None):
    parser = argparse.ArgumentParser(
        description="H3001 六轴机械臂逆运动学 (URDF + pinocchio)",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument("--urdf", type=Path, default=DEFAULT_URDF, help="URDF 路径")
    parser.add_argument("--frame", default=EE_FRAME, help=f"末端 frame 名，默认 {EE_FRAME}")
    parser.add_argument("--euler", default="zyx", choices=EULER_CONVENTIONS, help="RPY 欧拉序，默认 zyx")
    parser.add_argument(
        "--tool",
        type=float,
        nargs=6,
        default=None,
        metavar=("TX", "TY", "TZ", "TRX", "TRY", "TRZ"),
        help="末端固定变换 (mm/deg)，用于对齐工具/TCP 坐标系",
    )
    parser.add_argument("--csv", type=Path, default=DEFAULT_CSV, help="CAD 导出的限位 CSV")
    sub = parser.add_subparsers(dest="command", required=True)

    p_fk = sub.add_parser("fk", help="正解：关节角(deg) -> 位姿")
    p_fk.add_argument("joints", type=float, nargs=6, metavar="Q")

    p_ik = sub.add_parser("ik", help="逆解：位姿 -> 关节角(deg)")
    p_ik.add_argument("pose", type=float, nargs=6, metavar="P", help="X Y Z RX RY RZ (mm, deg)")
    p_ik.add_argument("--seed", type=float, nargs=6, metavar="Q", help="关节角初值(deg)，通常取当前反馈")
    p_ik.add_argument("--limits", choices=("urdf", "cad"), default="urdf", help="求解限位来源，默认 urdf")
    p_ik.add_argument("--restarts", type=int, default=32, help="额外随机起点个数")
    p_ik.add_argument("--tol-mm", type=float, default=0.1)
    p_ik.add_argument("--tol-deg", type=float, default=0.1)

    p_cal = sub.add_parser("calibrate", help="对比 MCU 位姿与 URDF 正解")
    p_cal.add_argument("--interface", default="socketcan")
    p_cal.add_argument("--channel", default="can0")
    p_cal.add_argument("--bitrate", type=int, default=1_000_000)
    p_cal.add_argument("--timeout", type=float, default=2.0)

    args = parser.parse_args(argv)
    if args.command == "fk":
        return cmd_fk(args)
    if args.command == "ik":
        return cmd_ik(args)
    if args.command == "calibrate":
        return cmd_calibrate(args)
    parser.print_help()
    return 2


if __name__ == "__main__":
    sys.exit(main())
