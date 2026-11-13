"""机械臂适配层：统一 Piper 与 H3001 两种机械臂的接口。

采集/验证脚本只依赖本模块导出的 ``create_robot`` / ``read_flange_pose``，
换机械臂时用 ``--robot piper`` 或 ``--robot h3001`` 选择，不需要改代码。

两种机械臂的差异
----------------
                 Piper (pyAgxArm)              H3001 (h3001_sdk)
连接             AgxArmFactory.create_arm()    H3001Arm(...)
读位姿           get_flange_pose().msg         get_flange_pose() 返回 list
返回单位         米 + 弧度                      毫米 + 度 → 需换算

两边的欧拉角约定都是 R = Rz(yaw)*Ry(pitch)*Rx(roll)，与
hand_eye_calibrate.py:55 一致，不需要做任何顺序调整。
"""

from __future__ import annotations

import math
import os
import sys
import time
from platform import system

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

DEG2RAD = math.pi / 180.0

ROBOT_KINDS = ("piper", "h3001")

# 位姿自检用的合理范围。臂展不可能超出这个区间，欧拉角也不可能超出 ±pi，
# 单位换算一旦搞错（差 1000 倍 / 度弧度混淆）会立刻被这几条拦下来。
_MIN_REACH_M = 0.02
_MAX_REACH_M = 2.0
_MAX_ANGLE_RAD = math.pi + 1e-6
_MAX_PITCH_RAD = math.pi / 2 + 1e-6


def default_can_endpoint():
    """按平台给出默认的 CAN 接口类型与通道名。"""
    platform_name = system()
    if platform_name == "Linux":
        return "socketcan", "can0"
    if platform_name == "Windows":
        return "agx_cando", "0"
    if platform_name == "Darwin":
        return "slcan", "/dev/ttyACM0"
    raise RuntimeError(f"不支持的平台: {platform_name}")


# --------------------------------------------------------------------------
# Piper（松灵 AgileX）
# --------------------------------------------------------------------------
def _create_piper(interface, channel):
    from pyAgxArm import AgxArmFactory, ArmModel, PiperFW, create_agx_arm_config

    config = create_agx_arm_config(
        robot=ArmModel.PIPER,
        firmeware_version=PiperFW.V183,
        interface=interface,
        channel=channel,
    )
    robot = AgxArmFactory.create_arm(config)
    robot.connect()
    return robot


def _piper_flange_pose(robot, retries):
    """pyAgxArm 已直接返回米和弧度，不需要换算。"""
    message = None
    for _ in range(retries):
        message = robot.get_flange_pose()
        if message is not None and len(message.msg) == 6:
            return [float(v) for v in message.msg]
        time.sleep(0.1)
    raise RuntimeError(f"无法读取 Piper 法兰位姿 get_flange_pose()，最后一次返回: {message}")


# --------------------------------------------------------------------------
# H3001
# --------------------------------------------------------------------------
def _create_h3001(interface, channel, fd):
    from h3001_sdk import H3001Arm

    robot = H3001Arm(interface=interface, channel=channel, fd=fd)
    robot.connect()
    return robot


def h3001_pose_to_m_rad(sdk_pose):
    """把 h3001_sdk 的 pose（毫米 + 度）换算成 [x, y, z, rx, ry, rz]（米 + 弧度）。

    依据：pyAgxArm 解析同一套 AGX 协议的同一批反馈帧
    （0x2A2/0x2A3/0x2A4），见
    pyAgxArm/protocols/can_protocol/drivers/piper/default/parser.py:180-205：

        X/Y/Z     raw * 1e-6            # 原始单位 0.001mm
        RX/RY/RZ  raw * 1e-3 * DEG2RAD  # 原始单位 0.001 度

    而 h3001_sdk.py:642-649 对六个数统一做了 ``/ 1000.0``，于是位置得到的是
    毫米、姿态得到的是度。这里把 SDK 的输出再换算一次即可：
        位置  × 1e-3      （毫米 → 米）
        姿态  × DEG2RAD   （度   → 弧度）
    """
    values = [float(v) for v in sdk_pose]
    if len(values) != 6:
        raise ValueError(f"位姿应为 6 个元素，实际得到 {len(values)} 个。")
    x, y, z, rx, ry, rz = values
    return [x * 1e-3, y * 1e-3, z * 1e-3, rx * DEG2RAD, ry * DEG2RAD, rz * DEG2RAD]


def m_rad_to_h3001_pose(pose):
    """h3001_pose_to_m_rad 的逆运算。仅用于直接下发 SDK 原始刻度的接口。"""
    values = [float(v) for v in pose]
    if len(values) != 6:
        raise ValueError(f"位姿应为 6 个元素，实际得到 {len(values)} 个。")
    x, y, z, rx, ry, rz = values
    return [x * 1e3, y * 1e3, z * 1e3, rx / DEG2RAD, ry / DEG2RAD, rz / DEG2RAD]


def _h3001_flange_pose(robot, retries):
    last = None
    for _ in range(retries):
        last = robot.get_flange_pose()
        if last is not None and len(last) == 6:
            return h3001_pose_to_m_rad(last)
        time.sleep(0.1)
    raise RuntimeError(f"无法读取 H3001 法兰位姿 get_flange_pose()，最后一次返回: {last}")


# --------------------------------------------------------------------------
# 对外接口
# --------------------------------------------------------------------------
def create_robot(kind="h3001", *, fd=True, interface=None, channel=None):
    """连接机械臂。``kind`` 取 ``"piper"`` 或 ``"h3001"``。

    ``fd`` 只对 H3001 有效：默认沿用 h3001_sdk 的 CAN-FD 配置
    （1 Mbps / 5 Mbps）；经典 CAN 适配器传 ``fd=False``。
    """
    if interface is None or channel is None:
        interface, channel = default_can_endpoint()
    if kind == "piper":
        return _create_piper(interface, channel)
    if kind == "h3001":
        return _create_h3001(interface, channel, fd)
    raise ValueError(f"未知的机械臂类型: {kind!r}，可选 {ROBOT_KINDS}")


def disconnect_robot(robot):
    """断开机械臂连接，释放 CAN 资源。

    两个 SDK 的方法名不一样，别直接调：
        Piper (pyAgxArm):  robot.disconnect()
        H3001 (h3001_sdk): robot.close()
    """
    if robot is None:
        return
    if _is_h3001(robot):
        robot.close()
    else:
        robot.disconnect()


def read_flange_pose(robot, retries=30, *, verbose=False):
    """读取法兰位姿，返回 [x, y, z, rx, ry, rz]，单位统一为米和弧度。

    读取后做合理性检查，单位搞错时直接报错，而不是静默采出废数据。
    """
    if robot is None:
        raise RuntimeError("机械臂未连接。")

    if _is_h3001(robot):
        pose = _h3001_flange_pose(robot, retries)
    else:
        pose = _piper_flange_pose(robot, retries)

    if verbose:
        print(f"  {describe_pose(pose)}")
    check_pose_plausible(pose)
    return pose


def read_settled_flange_pose(robot, retries=30, *, samples=5, gap_s=0.05,
                             tol_m=0.0005, tol_deg=0.15, timeout_s=5.0):
    """读法兰位姿，但要求连续几次读数彼此一致 —— 也就是**机械臂真的停稳了**。

    为什么需要这个（H3001 尤其）
    ---------------------------
    `h3001_sdk` 的 `state.pose` 是**三条 CAN 反馈帧拼起来**的
    （`h3001_sdk.py:639-648`）：

        0x2A2 -> pose[0], pose[1]   (X, Y)
        0x2A3 -> pose[2], pose[3]   (Z, RX)
        0x2A4 -> pose[4], pose[5]   (RY, RZ)

    机械臂还在动的时候，这三帧来自**不同时刻**，拼出来的位姿内部就是错位的。
    H3003 实测：有一张图与它记录的位姿差 **17°**，而臂停稳的样本只差 0.6°。
    这种误差不会报错，会直接污染手眼标定，所以必须在**采之前**拦住。

    返回 [x, y, z, rx, ry, rz]（米 + 弧度，取窗口均值）。若在 timeout_s 内
    始终不收敛则抛 RuntimeError —— 调用方应当放弃这一张，而不是照收。
    """
    if robot is None:
        raise RuntimeError("机械臂未连接。")

    deadline = time.monotonic() + timeout_s
    window = []
    while True:
        window.append(read_flange_pose(robot, retries))
        if len(window) > samples:
            window.pop(0)
        if len(window) == samples:
            pos_spread_m, rot_spread_deg = _pose_spread(window)
            if pos_spread_m <= tol_m and rot_spread_deg <= tol_deg:
                return _mean_pose(window)
        if time.monotonic() >= deadline:
            pos_spread_m, rot_spread_deg = _pose_spread(window[-samples:])
            raise RuntimeError(
                f"机械臂没有停稳：{timeout_s:g} 秒内 {samples} 次读数的散布仍有 "
                f"{pos_spread_m * 1000:.2f} mm / {rot_spread_deg:.2f}° "
                f"（容差 {tol_m * 1000:.2f} mm / {tol_deg:.2f}°）。"
                "请等机械臂完全静止后再采；这一张不要用。"
            )
        time.sleep(gap_s)


def _pose_spread(window):
    """一组位姿读数的散布：返回 (位置最大偏差 m, 角度最大偏差 deg)。

    角度按 ±pi 环绕处理，否则 rz 在 ±180° 附近跳变会被误判成"没停稳"。
    """
    arr = np.asarray(window, dtype=np.float64)
    pos_spread_m = float(np.abs(arr[:, :3] - arr[0, :3]).max())
    d = (arr[:, 3:] - arr[0, 3:] + math.pi) % (2.0 * math.pi) - math.pi
    return pos_spread_m, float(np.degrees(np.abs(d).max()))


def _mean_pose(window):
    """窗口均值；角度带环绕处理。"""
    arr = np.asarray(window, dtype=np.float64)
    d = (arr[:, 3:] - arr[0, 3:] + math.pi) % (2.0 * math.pi) - math.pi
    angles = arr[0, 3:] + d.mean(axis=0)
    return [float(v) for v in arr[:, :3].mean(axis=0)] + [float(v) for v in angles]


def _is_h3001(robot):
    return type(robot).__name__ == "H3001Arm"


def describe_pose(pose):
    """把位姿格式化成便于和示教器对照的一行。"""
    x, y, z, rx, ry, rz = pose
    return (
        f"xyz=({x:.4f}, {y:.4f}, {z:.4f}) m  "
        f"rpy=({math.degrees(rx):.3f}, {math.degrees(ry):.3f}, {math.degrees(rz):.3f}) deg"
    )


def check_pose_plausible(pose, label="位姿"):
    """检查位姿是否落在物理上可能的范围内，否则抛错并给出诊断。

    这是防止单位搞错的安全网：位置差 1000 倍、或姿态是度而不是弧度，
    都会在这里被拦下，而不是等到标定结果崩掉才发现。
    """
    x, y, z, rx, ry, rz = pose
    reach = math.sqrt(x * x + y * y + z * z)
    problems = []
    if not (_MIN_REACH_M <= reach <= _MAX_REACH_M):
        problems.append(
            f"臂展 {reach:.6g} m 超出合理区间 [{_MIN_REACH_M}, {_MAX_REACH_M}] m"
        )
    for name, value in (("rx", rx), ("ry", ry), ("rz", rz)):
        if abs(value) > _MAX_ANGLE_RAD:
            problems.append(f"{name}={value:.6g} rad 超出 ±pi")
    if abs(ry) > _MAX_PITCH_RAD:
        problems.append(f"ry={ry:.6g} rad 超出 ±pi/2（roll-pitch-yaw 的 pitch 上限）")
    if problems:
        raise RuntimeError(
            f"{label} 单位疑似有误，请核对后重采：\n  " + "\n  ".join(problems)
        )


def wait_for_arm_ready(robot, timeout=5.0):
    """等待机械臂开始上报反馈（仅 H3001 有明确的反馈计数）。"""
    if not _is_h3001(robot):
        return True
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if robot.get_state().feedback_count > 0:
            return True
        time.sleep(0.05)
    return False
