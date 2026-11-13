#!/usr/bin/env python3
"""上位机求逆解 + 轨迹规划检查: **只算不发**, 把「能不能这么走」先看清楚。

用法
------
    # 1) 单点: 给一个**上报原点位姿**, 求逆解并检查限位
    /usr/bin/python3 scripts/ik_plan.py pose 421.2 -48.9 257.0 166.4 79.2 164.0

    # 2) 直线: 从起点沿一个方向走一段, 逐点求逆解并**检查分支连续性**
    /usr/bin/python3 scripts/ik_plan.py line 421.2 -48.9 358.7 166.4 79.2 164.0 \
        --delta 0 0 -0.101

    # 3) 只读地验关节约定 (不碰机械臂, 只读 ROS 话题; 需要 bridge 在跑)
    /usr/bin/python3 scripts/ik_plan.py check

    # 4) 指定 IK 脚本位置 / 种子
    /usr/bin/python3 scripts/ik_plan.py pose ... --ik-src /path/to/SDK_linux \
        --seed 0 -13 0 12 12 -1

这个脚本**一条指令都不发**, 也不碰 CAN。它只做三件事: 求逆解、查机械限位、
检查相邻路点是不是同一条构型。真正下发用 scripts/move_joints.py (它自带
打印每轴差值 + 等 Enter 的闸门), 本脚本会把可以直接粘贴的命令行打出来。

单位与坐标系
--------------
位姿一律是**上报原点**的 mm/deg (`[X, Y, Z, RX, RY, RZ]`, ZYX-RPY) ——
和 yaml 里的 `place_pose_reported_mm_deg` / `standby_pose_reported_mm_deg`、
厂商 GUI、以及 `/feedback/tcp_pose` 是同一套。**喂进来的必须是这一套**:
执行节点内部的「夹爪点」与它差一个工具偏移 (gripper_offset_tool_xyz, 代码默认
0.08m / yaml 0.18m), 喂错了就是几十上百毫米的偏差。

关节角一律是**度**。

关节约定的那三个符号 (重要)
------------------------------
实测: `URDF 正解( [1,-1,-1,1,-1,1] · 上报关节角 ) == 上报位姿` (0.00mm/0.00°)。
也就是说固件**上报**的关节角与 URDF 的关节约定差 J2/J3/J5 三个符号。厂商 web UI
里那张 `WEB_JOINT_DIRECTION = [1,-1,-1,1,-1,1]` 是同一件事 (显示时乘上去)。

由此有两条**方向相反**的结论, 别搞混:

  - **物理/URDF 约定** (`q_物理`): 模型的关节角, 也是 IK 解出来的东西。
    `q_物理 = JOINT_SIGN · q_上报`.
  - **话题约定** (`q_上报`): `/feedback/joint_states` 报的是它, `/control/move_j`
    吃的也是它。**两者是同一个约定** —— 2026-09-28 真机实测: 发
    `[.. J2=-19.4 ..]` 之后回报仍是 `J2=-19.4` (不是 `+19.4`), 而同一帧里 J6 照常
    走到 90° (命令确实执行了)。厂商 web UI 的 `WEB_JOINT_COMMAND_DIRECTION` 是
    `[1,1,1,1,1,1]` (原样下发), 与这条一致。

⇒ **要下发的 = `JOINT_SIGN · (IK 解出来的 q_deg)`**。本脚本的表打的是 IK 的原值
(列标题写着 URDF 约定), 而「可直接下发」那段打的是**乘过 JOINT_SIGN 的**, 别混。

**最终判据 (2026-09-28 真机, 物理动作)** —— 两步都不依赖约定假设:

  1. `move_joints.py 0 0 0 0 0 0` —— J2/J3/J5 都是 0, **两种约定给的是同一个结果**,
     所以这一步无论约定如何都安全, 用来把臂收到零位 `[76.3, 0, 181.2] mm`;
  2. `move_joints.py 0.000 -3.349 8.790 -0.003 -5.441 0.003` —— 约定对 → 末端**往上**
     40mm 到 `[76.3, 0, 221.2]`; 约定反 → **往下** 40mm 到 `[75.0, 0, 140.9]`。
     两种都在零位附近、都不会撞到东西, 而方向一眼可辨。**实测: 往上。**

  一条命令就能区分、且两种结果都安全 —— 换机械臂/换固件后重做这两步即可。

⚠ 我 (Claude) 一度推断反了 (以为指令吃 URDF 约定, 说「原样下发」)。根因是当时那条
「IK 原样发能到目标位姿」的证据是在**接近零位**的姿势上做的 —— 零位附近两种约定重合,
分辨不出来。**涉及符号约定的结论, 必须在关节角明显不为零的构型上验。**

为什么要检查「分支连续性」
----------------------------
逆解只保证「位姿对」, 不保证「和上一点是同一条构型」。实测一条垂直下降的直线
(抓取点上方 10cm → 抓取点, 姿态不变): 连续解会在 z≈342mm 处撞上 joint_4 的机械
上限 (+100.3°), 之后**没有任何连续解**, 求解器只能翻腕 (J4 从 +98 跳到 -78, 167°)。
固件对这条直线的判决就是 `ik_status=5 (关节单步过大)` —— 2026-09-28 那次抬升失败
正是它。本脚本把这件事变成一张能看的表: 相邻路点的最大关节差、距限位的余量、
以及在第几点发生了被迫换构型。

参数
------
    --ik-src    ik_h3001.py 所在目录 (默认 $H3001_IK_SRC, 否则试几个常见位置)
    --urdf      URDF 路径 (默认从脚本位置往上找 urdf/L4002-0100.urdf)
    --seed      首点的关节角初值 (deg, 6 个)。不给就**从 /feedback/joint_states 读
                当前构型**再乘 JOINT_SIGN (推荐 —— 种子决定挑哪条分支)
    --step      直线取点间距 (m, 默认 0.01)
    --margin    要求的最小限位余量 (deg, 默认 5.0)
    --jump-warn 相邻点关节差超过它就算「换构型」(deg, 默认 20.0)
    --restarts  IK 多起点个数 (默认 32; 求解器会在收敛的解里挑离种子最近的)

退出码: 0 全部可执行 / 1 有问题 (不可达、越限、被迫换构型)
"""

from __future__ import annotations

import argparse
import importlib.util
import math
import os
import sys
import unicodedata
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
PKG_ROOT = HERE.parent
WS_ROOT = PKG_ROOT.parent

# 上报关节角 -> URDF 关节角。见模块 docstring 那张表。
JOINT_SIGN = np.array([1.0, -1.0, -1.0, 1.0, -1.0, 1.0])

URDF_NAME = "L4002-0100.urdf"
IK_MODULE_NAME = "ik_h3001.py"


# ── 找依赖 ──────────────────────────────────────────────────


def _resolve_ik_src(explicit: str | None) -> Path:
    """找 ik_h3001.py 所在目录。返回目录, 找不到就抛 FileNotFoundError。"""
    candidates = []
    if explicit:
        candidates.append(Path(explicit))
    env = os.environ.get("H3001_IK_SRC")
    if env:
        candidates.append(Path(env))
    # 本机实测的位置 (pinocchio 在 ROS Humble 的 3.10 里就有, 不需要 conda)
    candidates.append(Path("/home/zdl/pinocchio-devel/SDK_linux"))
    for path in candidates:
        if (path / IK_MODULE_NAME).is_file():
            return path
    raise FileNotFoundError(
        f"找不到 {IK_MODULE_NAME}; 用 --ik-src 指定它所在目录 "
        f"(或设环境变量 H3001_IK_SRC)。试过: {[str(c) for c in candidates]}"
    )


def _resolve_urdf(explicit: str | None, ik_src: Path) -> Path:
    """找 URDF。优先命令行, 然后工作区自己的 urdf/, 最后 IK 脚本旁边那份。"""
    candidates = []
    if explicit:
        candidates.append(Path(explicit))
    candidates.append(WS_ROOT / "urdf" / URDF_NAME)
    candidates.append(ik_src / "urdf" / URDF_NAME)
    for path in candidates:
        if path.is_file():
            return path
    raise FileNotFoundError(
        f"找不到 {URDF_NAME}; 用 --urdf 指定。试过: {[str(c) for c in candidates]}"
    )


def _load_ik_module(ik_src: Path):
    """按路径 import ik_h3001 (它不是包, 只能这么加载)。"""
    spec = importlib.util.spec_from_file_location("ik_h3001", ik_src / IK_MODULE_NAME)
    if spec is None or spec.loader is None:
        raise ImportError(f"无法加载 {ik_src / IK_MODULE_NAME}")
    module = importlib.util.module_from_spec(spec)
    sys.modules["ik_h3001"] = module
    spec.loader.exec_module(module)
    return module


def _cad_csv(urdf: Path) -> Path | None:
    """CAD 限位 CSV: 与 URDF 同目录的那一个。**两种命名都认**。

    上游 CAD 导出的叫「…__URDF.csv」(中文包名那一套), 而 vendored 进
    `bottle_grasp_executor/ik/` 之后改成了 ASCII 的 `L4002-0100-cad-limits.csv`。
    只认前一种时, `--urdf ik/L4002-0100.urdf` 会**静默**退回 URDF 限位 (只打一行
    「没找到 CAD CSV, 限位偏松」)—— 位置解一样、余量却按松的那一套算, 2026-09-30
    排查放置段时就被它误导过一次。

    括号里的优先级不是随意的: 同名目录里两份都在时, 先认上游那一种 (它是 CAD 原样
    导出, ASCII 那份是它的抄本)。
    """
    for pattern in ("*URDF.csv", "*cad-limits.csv"):
        for path in sorted(urdf.parent.glob(pattern)):
            return path
    return None


# ── 读当前构型 (只读 ROS, 不做任何运动) ─────────────────────


def _sensor_qos():
    """与 h3001_bridge 反馈发布器**匹配**的 QoS。

    桥用的是 BEST_EFFORT (见 h3001_bridge.py 里的 sensor_qos), 而 rclpy 订阅默认是
    RELIABLE —— 两者**不兼容**, 订阅者一条都收不到。症状是「等了 5 秒什么也没来」,
    很容易误判成「bridge 没在跑」。
    """
    from rclpy.qos import (  # noqa: PLC0415
        DurabilityPolicy,
        HistoryPolicy,
        QoSProfile,
        ReliabilityPolicy,
    )

    return QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
        history=HistoryPolicy.KEEP_LAST,
        depth=5,
    )


def _current_seed_deg(timeout_s: float = 5.0):
    """从 /feedback/joint_states 读当前关节角, 换成 URDF 约定返回 (deg)。

    读不到返回 None (调用方退回全零并打 warn)。**只订阅, 不发布**。
    """
    try:
        import rclpy  # noqa: PLC0415 - 只有这一条路需要 ROS
        from rclpy.node import Node
        from sensor_msgs.msg import JointState
    except Exception as exc:  # noqa: BLE001 - 没 source 过 ROS 就走不到这里
        print(f"[warn] 读不到 ROS ({exc}); 种子退回全零", file=sys.stderr)
        return None

    import time as _time

    rclpy.init()
    node = Node("ik_plan_seed_probe")
    holder = {"msg": None}
    node.create_subscription(
        JointState, "/feedback/joint_states", lambda m: holder.update(msg=m), _sensor_qos()
    )
    deadline = _time.monotonic() + timeout_s
    try:
        while holder["msg"] is None and _time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.destroy_node()
        rclpy.shutdown()

    msg = holder["msg"]
    if msg is None:
        print(
            f"[warn] {timeout_s:.0f}s 内没收到 /feedback/joint_states (bridge 没在跑?); "
            "种子退回全零",
            file=sys.stderr,
        )
        return None
    return np.degrees(np.asarray(msg.position[:6], dtype=float))


def _reported_to_urdf(q_reported_deg):
    """上报关节角 -> URDF 关节角 (见模块 docstring)。"""
    return JOINT_SIGN * np.asarray(q_reported_deg, dtype=float)


# ── 求解与检查 ──────────────────────────────────────────────


class Planner:
    """把 ik_h3001 的求解 + CAD 限位检查包成「一条轨迹」的用法。"""

    def __init__(self, ik, urdf: Path, csv: Path | None, margin_deg: float):
        self.ik = ik
        self.model, self.data, self.fid = ik.load_model(urdf, ik.EE_FRAME)
        if csv is not None:
            self.lo, self.hi, self.names = ik.cad_limits(csv, self.model)
            self.limit_source = f"CAD ({csv.name})"
        else:
            self.lo, self.hi = ik.urdf_limits(self.model)
            self.names = list(self.model.names[1:])
            self.limit_source = "URDF (没找到 CAD CSV, 限位偏松)"
        self.margin_deg = float(margin_deg)

    def solve(self, pose, seed_deg):
        return self.ik.ik(
            self.model, self.data, self.fid, list(pose), seed_deg=np.asarray(seed_deg)
        )

    def margins_deg(self, q_deg):
        """每个关节距最近一侧限位的余量 (deg)。"""
        q = np.asarray(q_deg, dtype=float)
        return np.minimum(q - np.degrees(self.lo), np.degrees(self.hi) - q)


def _fmt_q(q_deg):
    return " ".join(f"{v:+7.2f}" for v in q_deg)


def _width(text: str) -> int:
    """字符串在终端里占几列 (CJK 算 2 列)。表头里有中文, 不这么算就对不齐。"""
    return sum(2 if unicodedata.east_asian_width(ch) in "WF" else 1 for ch in text)


def _pad(text: str, width: int, right: bool = False) -> str:
    padding = " " * max(0, width - _width(text))
    return padding + text if right else text + padding


def _walk(planner, poses, seed_deg, jump_warn, restarts, margin):
    """逐点求解, 返回行列表 + 统计。种子用上一点的解 (求解器会挑离它最近的解)。"""
    rows = []
    prev = np.asarray(seed_deg, dtype=float)
    worst_jump = 0.0
    jumped_at = []
    for index, pose in enumerate(poses):
        result = planner.solve(pose, prev)
        q = np.asarray(result["q_deg"], dtype=float)
        margins = planner.margins_deg(q)
        min_margin = float(margins.min())
        jump = float(np.abs(q - prev).max()) if index > 0 else 0.0
        if index > 0:
            worst_jump = max(worst_jump, jump)
            if jump > jump_warn:
                which = int(np.argmax(np.abs(q - prev)))
                jumped_at.append((index, which, float(prev[which]), float(q[which]), jump))
        rows.append(
            {
                "pose": pose,
                "ok": bool(result["ok"]),
                "pos_err": float(result["pos_err_mm"]),
                "rot_err": float(result["rot_err_deg"]),
                "q": q,
                "jump": jump,
                "min_margin": min_margin,
                "margin_ok": min_margin >= margin,
            }
        )
        prev = q
    return rows, worst_jump, jumped_at


def _report(rows, worst_jump, jumped_at, jump_warn, margin, limit_source, unit_label):
    """打印那张表 + 结论。返回退出码。"""
    print(f"\n限位来源: {limit_source}; 要求余量 ≥ {margin:.1f}°; "
          f"相邻差 > {jump_warn:.0f}° 记为「换构型」")
    print()
    print(
        f"{_pad(unit_label, 7, True)}  {_pad('位姿误差', 16, True)}  "
        f"{_pad('关节角 J1..J6 (deg, URDF 约定)', 52)}  {_pad('相邻差', 7, True)}  "
        f"{_pad('余量', 7, True)}  结论"
    )
    problems = []
    for index, row in enumerate(rows):
        flags = []
        if not row["ok"]:
            flags.append("未收敛")
            problems.append(f"第 {index + 1} 点逆解没收敛 ({row['pos_err']:.2f}mm/"
                            f"{row['rot_err']:.2f}°)")
        if not row["margin_ok"]:
            flags.append("余量不足")
            problems.append(
                f"第 {index + 1} 点距限位只剩 {row['min_margin']:.1f}°"
            )
        if index > 0 and row["jump"] > jump_warn:
            flags.append("换构型")
        label = rows[index].get("label", str(index + 1))
        print(
            f"{_pad(label, 7, True)}  "
            f"{row['pos_err']:7.2f}mm/{row['rot_err']:5.2f}°  "
            f"{_fmt_q(row['q'])}  {row['jump']:6.2f}°  {row['min_margin']:6.2f}°  "
            f"{'/'.join(flags) if flags else 'ok'}"
        )

    print()
    if jumped_at:
        # 只把**第一次**跳变解释成「被迫换构型」: 后面那些大差值多半是换过去之后的
        # 回稳 (构型变了, 新分支的关节角离老分支本来就远), 同样不能当连续运动,
        # 但成因不同, 混在一起说会把人带偏。
        index, which, before, after, jump = jumped_at[0]
        print("⚠ **中途被迫换构型**: 这条直线不是一条连续的关节运动。")
        print(
            f"   首次在第 {index + 1} 点: J{which + 1} 从 {before:+.1f}° 跳到 "
            f"{after:+.1f}° ({jump:.0f}°)。多半是某一轴撞到了机械限位, 连续解不存在了"
            f" —— 固件对同一条直线的判决就是 ik_status=5 (关节单步过大)。"
        )
        if len(jumped_at) > 1:
            rest = ", ".join(
                f"第 {i + 1} 点 {j:.0f}°" for i, _w, _b, _a, j in jumped_at[1:]
            )
            print(f"   之后还有 {rest} —— 那是换过去之后的回稳 (新分支本来就离老分支"
                  f"远), 不是又撞了一次限位。想看清跳变发生在哪, 把 --step 调小重跑。")
        print("   对策: ① 换个抓取姿态 (让手腕远离限位); ② 把翻腕这一步挪到**空载**"
              "时痛快做完, 再走剩下的直线; ③ 下降不要走直线, 接受一条绕开的路径。")
    if problems:
        for line in problems:
            print(f"⚠ {line}")
    print()

    if not problems and not jumped_at:
        print("✅ 全程同一条构型、限位余量充足、每点位姿误差在容差内 —— 可以逐点下发。")
        print()
        print("下发用 scripts/move_joints.py (它自带等 Enter 的闸门)。"
              "**逐条按顺序跑**, 每条走完再跑下一条:")
        print()
        print("(下面是**乘过 JOINT_SIGN** 的, 也就是真正要发的值 —— 话题上的约定"
              "与 IK 的 URDF 约定差那三个符号, 见 check)")
        print()
        for index, row in enumerate(rows):
            print(
                "    /usr/bin/python3 scripts/move_joints.py "
                + " ".join(f"{v:.3f}" for v in JOINT_SIGN * row["q"])
                + f"    # 第 {index + 1}/{len(rows)} 点"
            )
            if index == 0 or row["jump"] > jump_warn:
                print("      ↑ 走完看到位了再发下一条"
                      if index == 0 else "      ↑ 这是换构型后的第一个点, 更要盯着")
        return 0

    print("❌ 这条轨迹**不要**照原样下发。")
    return 1


# ── 子命令 ──────────────────────────────────────────────────


def _add_source_options(parser):
    """三个子命令都要的: 去哪儿找 IK 脚本与模型。"""
    parser.add_argument("--ik-src", default=None,
                        help=f"{IK_MODULE_NAME} 所在目录 (默认 $H3001_IK_SRC)")
    parser.add_argument("--urdf", default=None, help=f"URDF 路径 (默认 {URDF_NAME})")


def _add_plan_options(parser):
    """只有真正求解的两个子命令要的: 种子与两条阈值。"""
    parser.add_argument("--seed", type=float, nargs=6, default=None,
                        metavar=("Q1", "Q2", "Q3", "Q4", "Q5", "Q6"),
                        help="首点关节角初值 (deg, **URDF 约定**); 不给就从 "
                             "/feedback/joint_states 读当前构型再乘 JOINT_SIGN")
    parser.add_argument("--margin", type=float, default=5.0,
                        help="要求的最小限位余量 (deg, 默认 5)")
    parser.add_argument("--jump-warn", type=float, default=20.0,
                        help="相邻点关节差超过它就算换构型 (deg, 默认 20)")


def _make_planner(args):
    ik_src = _resolve_ik_src(args.ik_src)
    urdf = _resolve_urdf(args.urdf, ik_src)
    ik = _load_ik_module(ik_src)
    planner = Planner(ik, urdf, _cad_csv(urdf), args.margin)
    print(f"IK 脚本 : {ik_src / IK_MODULE_NAME}")
    print(f"URDF    : {urdf}")
    return planner


def _initial_seed(args, planner):
    """首点的种子。给了 --seed 就用它; 否则读当前构型并换到 URDF 约定。"""
    if args.seed is not None:
        seed = np.asarray(args.seed, dtype=float)
        print(f"种子    : 命令行给出 {_fmt_q(seed)} deg (URDF 约定)")
        return seed
    reported = _current_seed_deg()
    if reported is None:
        print("种子    : 全零 (读不到当前构型)")
        return np.zeros(6)
    seed = _reported_to_urdf(reported)
    print(f"当前上报关节角 : {_fmt_q(reported)} deg")
    print(f"种子 (乘 JOINT_SIGN): {_fmt_q(seed)} deg (URDF 约定)")
    return seed


def cmd_pose(args):
    planner = _make_planner(args)
    seed = _initial_seed(args, planner)
    rows, _, _ = _walk(planner, [args.pose], seed, args.jump_warn,
                       args.restarts, args.margin)
    print()
    return _report(rows, 0.0, [], args.jump_warn, args.margin,
                   planner.limit_source, "位姿(mm)")


def cmd_line(args):
    planner = _make_planner(args)
    seed = _initial_seed(args, planner)
    delta = np.asarray(args.delta, dtype=float)
    length = float(np.linalg.norm(delta))
    if length <= 0.0:
        raise SystemExit("--delta 不能是零向量")
    step = float(args.step)
    count = max(2, int(math.ceil(length / step)) + 1)
    start = np.asarray(args.pose, dtype=float)
    poses = []
    for index in range(count):
        pose = start.copy()
        pose[:3] = start[:3] + delta * (index / (count - 1)) * 1000.0  # m -> mm
        poses.append(pose)
    print(f"直线    : 起点 {np.round(start[:3], 1)} → 走 {np.round(delta * 1000.0, 1)} mm, "
          f"{count} 点 (每 {np.round(length / (count - 1) * 1000.0, 1)} mm)")
    print(f"姿态固定: {np.round(start[3:], 2)} deg")
    rows, worst, jumped = _walk(planner, poses, seed, args.jump_warn,
                                args.restarts, args.margin)
    for index, pose in enumerate(poses):
        rows[index]["label"] = f"{pose[2]:.1f}"
    print()
    code = _report(rows, worst, jumped, args.jump_warn, args.margin,
                   planner.limit_source, "z(mm)")
    if worst:
        print(f"(全程相邻点最大关节差 {worst:.2f}°)")
    return code


def cmd_check(args):
    """只读: 用当前这一帧反馈验证「上报关节角」与「URDF 正解」的关系。

    **不碰机械臂、不发任何指令**。判据是位置 (3 个数), 避开姿态约定那一层。
    """
    ik_src = _resolve_ik_src(args.ik_src)
    urdf = _resolve_urdf(args.urdf, ik_src)
    ik = _load_ik_module(ik_src)
    model, data, fid = ik.load_model(urdf, ik.EE_FRAME)

    try:
        import rclpy  # noqa: PLC0415
        from rclpy.node import Node
        from geometry_msgs.msg import PoseStamped
        from sensor_msgs.msg import JointState
    except Exception as exc:  # noqa: BLE001
        print(f"[abort] 需要 ROS 环境 (source /opt/ros/humble/setup.bash): {exc}",
              file=sys.stderr)
        return 2

    import time as _time

    rclpy.init()
    node = Node("ik_plan_check")
    box = {"joints": None, "pose": None}
    qos = _sensor_qos()
    node.create_subscription(
        JointState, "/feedback/joint_states", lambda m: box.update(joints=m), qos
    )
    node.create_subscription(
        PoseStamped, "/feedback/tcp_pose", lambda m: box.update(pose=m), qos
    )
    deadline = _time.monotonic() + 5.0
    try:
        while (box["joints"] is None or box["pose"] is None) and _time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.destroy_node()
        rclpy.shutdown()

    if box["joints"] is None or box["pose"] is None:
        print("[abort] 5s 内没同时收到 /feedback/joint_states 与 /feedback/tcp_pose; "
              "h3001_bridge 在跑吗 (ros2 topic hz /feedback/tcp_pose)", file=sys.stderr)
        return 2

    q_rep = np.degrees(np.asarray(box["joints"].position[:6], dtype=float))
    p = box["pose"].pose.position
    reported_mm = np.array([p.x, p.y, p.z]) * 1000.0

    def fk_pose(q_deg):
        return np.asarray(ik.forward_kinematics(model, data, fid, q_deg), dtype=float)

    pose_as_is = fk_pose(q_rep)                  # 把上报角直接喂给 URDF
    pose_flipped = fk_pose(_reported_to_urdf(q_rep))  # 上报角 × JOINT_SIGN
    as_is, flipped = pose_as_is[:3], pose_flipped[:3]

    print("=== 当前这一帧 (只读, 没有发任何指令) ===")
    print(f"上报关节角 (deg)          : {_fmt_q(q_rep)}")
    print(f"上报位姿 (上报原点, mm)   : {np.round(reported_mm, 2)}")
    print()
    print(f"URDF 正解(上报角)         : {np.round(as_is, 2)}   "
          f"差 {np.linalg.norm(as_is - reported_mm):7.2f} mm")
    print(f"URDF 正解(上报角 ×JOINT_SIGN): {np.round(flipped, 2)}   "
          f"差 {np.linalg.norm(flipped - reported_mm):7.2f} mm")
    print()
    if np.linalg.norm(flipped - reported_mm) < 1.0 <= np.linalg.norm(as_is - reported_mm):
        print("→ 反馈报的关节角与 URDF 差 J2/J3/J5 三个负号:")
        print("      q_物理(URDF) = JOINT_SIGN · q_上报")
        print("  而**话题上的值(含指令)用的是上报那一套** —— 2026-09-28 真机实测:")
        print("      发 [.. J2=-19.4 ..] 之后回报仍是 J2=-19.4 (不是 +19.4),")
        print("      同一帧里 J6 照常走到 90° —— 命令执行了, 只是约定与 URDF 相反。")
        print("  ⇒ **要下发的 = JOINT_SIGN · (IK 解出来的 q_deg)**。")
        print()
        # 「原地不动」是**第一条该发**的指令: 判据是「一动不动」, 不需要任何反馈、
        # 不需要量, 站着看就行。约定若与我推的相反, 它会走到 F(上报角) 那个位姿,
        # 与现在差多少 = 上面那两个数之差 —— 幅度可控, 而且再发一次乘过 SIGNS 的
        # 值就能回到原处 (自证)。
        stay = np.asarray(q_rep, dtype=float)   # 话题上的约定: 发回原值 = 原地不动
        motion_mm = float(np.linalg.norm(pose_as_is[:3] - pose_flipped[:3]))
        turn_deg = float(np.linalg.norm(pose_as_is[3:] - pose_flipped[3:]))
        print()
        print("当前位姿 (上报原点 mm/deg; 由上报关节角 × JOINT_SIGN 正解得到):")
        print(f"    {np.round(pose_flipped, 2)}")
        print("    想让机械臂从当前位置往上走一段, 就把上面第 3 个数加上距离再喂给 "
              "`ik_plan.py pose`。")
        print()
        # 「原地不动」这条测试只在**约定万一反了也不会走出危险距离**时才建议:
        # 判据是「一动不动」, 不需要反馈、不需要读数, 站着看就行。但约定若反了它会
        # 走到 F(上报角) 那个姿势 —— 那个姿势可能又低又远 (实测撞见过 z=63mm 的),
        # 所以先算出来, 大了就不提这条建议。
        if motion_mm <= 50.0 and turn_deg <= 30.0:
            print("**可以做的第一条指令 (原地不动测试)** —— 判据是「一动不动」:")
            print(f"    /usr/bin/python3 scripts/move_joints.py "
                  f"{' '.join(f'{v:.3f}' for v in stay)}")
            print(f"    就是上面那 6 个上报值原样发回去。约定对 → **一动不动**;")
            print(f"    万一反了 → 走到 F(上报角), 与现在差 {motion_mm:.1f} mm / "
                  f"{turn_deg:.1f}°, 幅度不大也回得来。")
        else:
            print("**⚠ 现在这个姿势不适合做「原地不动」测试**:")
            print(f"    万一约定是反的, 它会走到 F(上报角) = "
                  f"{np.round(pose_as_is[:3], 1)} mm, 与现在差 {motion_mm:.1f} mm / "
                  f"姿态 {turn_deg:.1f}° —— 那个位置可能又低又靠近底座, 会撞台面。")
            print("    **不依赖约定的收尾姿势**: 发 `0 0 0 0 0 0` —— J2/J3/J5 都是 0, "
                  "两种约定给的是**同一个结果**, 所以这一步两种约定下都安全, 可以先把"
                  "臂收到零位再说。")
    elif np.linalg.norm(as_is - reported_mm) < 1.0:
        print("→ 这一次上报角与 URDF 正解直接吻合 (可能是全部接近零位的构型)。")
        print("  请把机械臂摆到一个**关节角大一点**的姿势上再跑一次 —— 零位附近两种"
              "约定几乎重合, 分辨不出来。")
    else:
        print("→ 两种都对不上: 说明还有别的差异 (零位偏置 / URDF 版本 / 工具坐标系)。")
        print("  别急着下发 IK 结果, 先用 --urdf 指定正确的模型再查。")
    return 0


def main(argv=None):
    parser = argparse.ArgumentParser(
        description="上位机求逆解 + 轨迹检查 (只算不发)",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    sub = parser.add_subparsers(dest="command", required=True)

    p_pose = sub.add_parser("pose", help="单点: 位姿 -> 关节角 + 限位检查")
    p_pose.add_argument("pose", type=float, nargs=6, metavar=("X", "Y", "Z", "RX", "RY", "RZ"),
                        help="上报原点位姿 (mm/deg)")
    p_pose.add_argument("--restarts", type=int, default=32)
    _add_source_options(p_pose)
    _add_plan_options(p_pose)

    p_line = sub.add_parser("line", help="直线: 逐点求逆解 + 分支连续性检查")
    p_line.add_argument("pose", type=float, nargs=6, metavar=("X", "Y", "Z", "RX", "RY", "RZ"),
                        help="起点位姿 (上报原点, mm/deg)")
    p_line.add_argument("--delta", type=float, nargs=3, required=True, metavar=("DX", "DY", "DZ"),
                        help="位移 (m), 姿态保持不变; 例: 下降 10cm 填 0 0 -0.10")
    p_line.add_argument("--step", type=float, default=0.01, help="取点间距 (m, 默认 0.01)")
    p_line.add_argument("--restarts", type=int, default=32)
    _add_source_options(p_line)
    _add_plan_options(p_line)

    p_check = sub.add_parser("check", help="只读: 验「上报关节角 vs URDF 正解」")
    _add_source_options(p_check)

    args = parser.parse_args(argv)
    if args.command == "pose":
        return cmd_pose(args)
    if args.command == "line":
        return cmd_line(args)
    if args.command == "check":
        return cmd_check(args)
    parser.print_help()
    return 2


if __name__ == "__main__":
    raise SystemExit(main())
