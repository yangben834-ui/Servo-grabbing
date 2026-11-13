#!/usr/bin/env python3
"""放置位姿单点验证: 发**一条** move_p, 看固件收不收, 看机械臂动不动。

为什么要有它 (2026-09-21 10:17 真机)
--------------------------------------
那次放置失败, 桥接日志里 MCU 当场回了 `IK 状态变化: status=2` —— 目标被**拒收**,
机械臂一步没走; 而执行节点只看得见「没动」, 等满 motion_timeout(30s) 才报一个
413mm 的偏差。想再试一组数, 当时的办法只能是把整个任务重跑一遍: 抓一瓶、抬起来、
再赌一次放置位姿 —— 一次几分钟, 而且每次都要重新夹一次瓶子。

这个脚本把最小可验证单元摘出来: **只发放置那一条指令**, 不抓瓶子、不动感知、
不起执行节点。它的目的不是「把瓶子放好」, 是回答一个二值问题
「这 6 个数, 固件收不收」—— 收下了再让机械臂走, 走完打印实际位移。

它和真机跑的**是同一组数、同一条话题、同一个固件判决**: 节点里放置位姿走的是
reported -> 夹爪 -> reported 的恒等往返 (见 `_load_place_pose` / `_send_move_p`),
所以这里直接把参数那 6 个数原样发出去, 与节点发出去的字节级一致。夹爪偏移
(`gripper_offset_tool_xyz`) 只影响下面**打印**的那个「夹爪点」, 不影响发出去的指令。

用法
------
    # 1) 只换算并打印, 一条指令都不发 (先看数对不对, 最安全)
    /usr/bin/python3 bottle_grasp_executor/scripts/check_place_pose.py --dry-run

    # 2) 用 yaml 里的 place_pose_reported_mm_deg 试一次
    /usr/bin/python3 bottle_grasp_executor/scripts/check_place_pose.py

    # 3) 现场改一组数试试 (GUI / h3001_bridge 日志上那 6 个数: mm + 度)
    /usr/bin/python3 bottle_grasp_executor/scripts/check_place_pose.py 214.0 -306.0 154.1 95.4 0.2 26.0

    # 4) 跳过 Enter 确认 (脚本化, 自己确认过再这么用)
    /usr/bin/python3 bottle_grasp_executor/scripts/check_place_pose.py 244.0 -300.0 250.0 95.4 0.2 26.0 --yes
    /usr/bin/python3 bottle_grasp_executor/scripts/check_place_pose.py 100.0 0.0 290.0 180.4 40.2 180.0 --yes
**跑之前先确认**:
  - h3001_bridge 在跑 (`ros2 topic hz /feedback/tcp_pose` 有输出), 否则发出去的
    指令没人接, 脚本会报「没收到反馈」;
  - h3001_bridge 是**重启过的** —— 见下面「改过消息定义要重启桥」;
  - **执行节点没在跑任务** —— 它自己也往 /control/move_p 发, 两边的指令会互相穿插;
  - 机械臂周围没人, 而且**别夹着瓶子时跑** (它会把机械臂开到目标点)。

改过 h3001_msgs 之后要重启桥 (2026-09-21 实测)
--------------------------------------------------
`H3001ArmStatus` 加了 `ik_*` 四个字段之后, 10:13 起着的那个桥进程**照常在发**, 但它
发的是旧字段布局 —— 而消息是按字段布局序列化的, 订阅端拿了新定义去解旧布局, 一条都
解不出来, 而且**不报错**(rmw 层只比类型名, 不比字段数)。现场症状很好认: `tcp_pose` /
`joint_states` 照常 100Hz, 只有改过字段的那个话题是 0。这个脚本把这一条单独报出来
(「位姿有、臂状态没有」), 免得被当成「桥没起」白折腾。

所以改过 .msg 之后: `colcon build` 完**重启 h3001_bridge**; 执行节点同理
(它按新布局订阅, 桥不重启就永远收不到臂状态, 而且不会有任何报错)。

退出码: 0 = 动了且到位误差在 --tolerance 内 / 1 = 拒收或一步没动 / 2 = 环境不对
(没反馈、参数错、只有 5 个数之类)

单独跑, 不需要起执行节点:

    /usr/bin/python3 scripts/check_place_pose.py --dry-run
"""

from __future__ import annotations

import argparse
import math
import os
import sys
import time

import numpy as np

try:
    import yaml
except ImportError:                                    # pragma: no cover
    yaml = None

import rclpy
from geometry_msgs.msg import PoseStamped
from h3001_msgs.msg import H3001ArmStatus
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

# 与 bottle_grasp_executor_node 里的同名函数同式 (invert_rx_ry=false 时固件 RPY 可以
# 直接按 ZYX 转, README §5.6)。这里**故意另写一份**: 这个脚本的用处之一就是拿一条
# 独立路径去核节点算出来的数, 复用它的实现等于让两边一起错也看不出来。
DEFAULT_TOPICS = {
    "move_p_topic": "/control/move_p",
    "tcp_pose_topic": "/feedback/tcp_pose",
    "arm_status_topic": "/feedback/arm_status",
    "robot_base_frame": "base_link",
}

# 固件 ik_status 码表 (抄自 SDK_linux/h3001_web/protocol_ui.js 的 ikNames)。
# 只用来把拒收原因译成人话; 判据是「非 0 即拒收」, 表里没有的码照样算拒收。
IK_STATUS_NAMES = {
    1: "参数无效",
    2: "超出工作空间",
    3: "超过关节限位",
    4: "轨迹缓冲区不足",
    5: "关节单步过大",
    6: "奇异姿态",
    7: "逆解校验失败",
}


def ik_status_text(code):
    """`ik_status=2 (超出工作空间)` —— 认不出来的码只写数字。"""
    name = IK_STATUS_NAMES.get(int(code))
    if name is None:
        return f"ik_status={int(code)}"
    return f"ik_status={int(code)} ({name})"


def say(text=""):
    """脚本的 stdout 是给人看的排障信息, 不做机器解析 (同 gripper_grip_hold.py)。"""
    print(text, flush=True)


# ── 与节点同式的换算 ──────────────────────────────────────


def rpy_to_quaternion(rx_deg, ry_deg, rz_deg):
    """ZYX 顺序 RPY(度) -> (x, y, z, w)。"""
    cr, sr = math.cos(math.radians(rx_deg) * 0.5), math.sin(math.radians(rx_deg) * 0.5)
    cp, sp = math.cos(math.radians(ry_deg) * 0.5), math.sin(math.radians(ry_deg) * 0.5)
    cy, sy = math.cos(math.radians(rz_deg) * 0.5), math.sin(math.radians(rz_deg) * 0.5)
    quaternion = np.asarray(
        [
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * sp * sy,
        ],
        dtype=np.float64,
    )
    norm = float(np.linalg.norm(quaternion))
    if norm < 1e-12:
        raise ValueError("四元数模长为零 —— RX/RY/RZ 填错了?")
    quaternion /= norm
    if quaternion[3] < 0.0:
        # 规范到 w >= 0: q 与 -q 是同一个旋转, 但这样打印出来的数与文档一致。
        quaternion = -quaternion
    return quaternion


def quaternion_to_matrix(quaternion):
    """(x, y, z, w) 单位四元数 -> tool 到 base 的 3x3 旋转矩阵 (与节点同式)。"""
    x, y, z, w = quaternion
    return np.asarray(
        (
            (1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)),
            (2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)),
            (2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)),
        ),
        dtype=np.float64,
    )


def reported_to_gripper(reported_m, quaternion, offset_tool):
    """上报末端原点 -> 夹爪点 (夹爪 = 上报 + R @ offset, 见节点 _reported_to_gripper)。

    **只用来打印**。发出去的永远是调用者给的那 6 个数 (见模块 docstring)。
    """
    return reported_m + quaternion_to_matrix(quaternion) @ np.asarray(
        offset_tool, dtype=np.float64
    )


def angle_between(quaternion_a, quaternion_b):
    """两个姿态之间的夹角 (度), 取 [0, 180]。"""
    dot = abs(float(np.dot(quaternion_a, quaternion_b)))
    return math.degrees(2.0 * math.acos(min(1.0, dot)))


# ── 配置 ──────────────────────────────────────────────────


def default_config_path():
    """先用安装目录里的 yaml, 装之前退回源码树里那份。"""
    try:
        from ament_index_python.packages import get_package_share_directory

        installed = os.path.join(
            get_package_share_directory("bottle_grasp_executor"),
            "config",
            "bottle_grasp_executor.yaml",
        )
        if os.path.exists(installed):
            return installed
    except Exception:                                  # pragma: no cover
        pass
    return os.path.join(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
        "config",
        "bottle_grasp_executor.yaml",
    )


def load_params(path):
    """读 yaml 里 ros__parameters 那一层。缺文件/缺键都不是错误 —— 用默认值。"""
    if yaml is None:
        say(f"[warn] 没装 PyYAML, 跳过 {path}, 一律用默认值")
        return {}
    if not os.path.exists(path):
        say(f"[warn] 找不到参数文件 {path} —— 一律用默认值")
        return {}
    with open(path, "r", encoding="utf-8") as handle:
        document = yaml.safe_load(handle) or {}
    return (document.get("bottle_grasp_executor") or {}).get("ros__parameters") or {}


class Probe(Node):
    """只订阅: 上报末端位姿 + 臂状态 (含固件 IK 判决)。一条也不发布地收集反馈。"""

    def __init__(self, topics, report):
        super().__init__("check_place_pose")
        self._report = report
        self.latest_pose = None        # (时钟秒, 位置 np(3) m, 四元数 tuple)
        self.latest_status = None      # (时钟秒, H3001ArmStatus)
        self._last_ik = None
        # qos_profile_sensor_data (BEST_EFFORT): h3001_bridge 那两个反馈发布器用的就是
        # BEST_EFFORT/VOLATILE/depth 5, 订阅端**必须**也是 BEST_EFFORT —— 拿默认的
        # RELIABLE 订阅会一条都收不到 (rclpy 只在启动时打一行 incompatible QoS 的
        # warn, 然后就静默收空)。执行节点订阅这两个话题用的也是这个 profile。
        self.create_subscription(
            PoseStamped, topics["tcp_pose_topic"], self._on_pose, qos_profile_sensor_data
        )
        self.create_subscription(
            H3001ArmStatus,
            topics["arm_status_topic"],
            self._on_status,
            qos_profile_sensor_data,
        )

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_pose(self, msg):
        self.latest_pose = (
            self.now(),
            np.asarray(
                (msg.pose.position.x, msg.pose.position.y, msg.pose.position.z),
                dtype=np.float64,
            ),
            (
                msg.pose.orientation.x,
                msg.pose.orientation.y,
                msg.pose.orientation.z,
                msg.pose.orientation.w,
            ),
        )

    def _on_status(self, msg):
        self.latest_status = (self.now(), msg)
        # 判决只在**变化**时打印 (固件就是这么上报的, 桥接日志里那行
        # `IK 状态变化: status=` 同源), 否则 100Hz 会刷屏。
        # 只打 ik_status 和 motion_status: ik_status_valid / ik_joint_limit_fault /
        # ik_pose_pending 在真机上**恒为 0** (SDK 的 get_state() 漏拷这几个字段,
        # sdk_linux.py:816), 打出来只会让人以为「固件没上报判决」。
        verdict = (int(msg.ik_status), int(msg.motion_status))
        if verdict != self._last_ik:
            self._last_ik = verdict
            self._report(
                f"    [反馈] IK 状态变化: {ik_status_text(verdict[0])}, "
                f"motion_status={verdict[1]}"
            )

    def spin_for(self, seconds):
        """转给定时长; 返回期间收到的新反馈帧数。"""
        deadline = time.monotonic() + seconds
        frames = 0
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.02)
            frames += 1
        return frames


# ── 主流程 ────────────────────────────────────────────────


def parse_args(argv):
    parser = argparse.ArgumentParser(
        description="发一条 move_p 到放置位姿, 看固件收不收、机械臂动不动",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument(
        "pose",
        nargs="*",
        type=float,
        help="上报末端原点系那 6 个数 [X,Y,Z(mm), RX,RY,RZ(度)]; "
        "不填就用 yaml 里的 place_pose_reported_mm_deg",
    )
    parser.add_argument("--config", default=None, help="参数文件 (默认用安装目录里那份)")
    parser.add_argument(
        "--dry-run", action="store_true",
        help="只换算并打印, 一条指令都不发 (机械臂不会动)",
    )
    parser.add_argument("--yes", action="store_true", help="跳过 Enter 确认")
    parser.add_argument("--grace", type=float, default=None,
                        help="发出后等多久再读判决 (s, 默认取 yaml 的 ik_verdict_grace)")
    parser.add_argument("--monitor", type=float, default=2.0,
                        help="读取判决之后再看多久运动 (s)")
    parser.add_argument("--tolerance", type=float, default=0.005,
                        help="到位误差容差 (m), 默认 5mm")
    return parser.parse_args(argv)


def main(argv=None):
    args = parse_args(sys.argv[1:] if argv is None else argv)
    config_path = args.config or default_config_path()
    params = load_params(config_path)
    topics = {key: params.get(key, value) for key, value in DEFAULT_TOPICS.items()}

    # ── 目标那 6 个数 ──
    if args.pose:
        if len(args.pose) != 6:
            say(f"[error] 要 6 个数 [X,Y,Z(mm), RX,RY,RZ(度)], 收到 {len(args.pose)} 个")
            return 2
        raw = [float(value) for value in args.pose]
        source = "命令行"
    else:
        raw = params.get("place_pose_reported_mm_deg")
        if not raw:
            say(
                "[error] 命令行没给数, yaml 里也没有 place_pose_reported_mm_deg —— "
                "两个至少给一个"
            )
            return 2
        raw = [float(value) for value in raw]
        if len(raw) != 6:
            say(f"[error] yaml 里的 place_pose_reported_mm_deg 不是 6 个数: {raw}")
            return 2
        source = "yaml 的 place_pose_reported_mm_deg"

    offset = [float(value) for value in params.get("gripper_offset_tool_xyz", [0.0] * 3)]
    if len(offset) != 3:
        say(f"[error] gripper_offset_tool_xyz 需要 3 个数: {offset}")
        return 2

    reported = np.asarray(raw[:3], dtype=np.float64) * 0.001
    quaternion = rpy_to_quaternion(*raw[3:])
    gripper = reported_to_gripper(reported, quaternion, offset)

    say(f"参数文件: {config_path}")
    say(f"目标 (来自 {source}): 上报原点={np.round(reported, 4)} m, "
        f"RPY={np.round(np.asarray(raw[3:]), 2)} 度")
    say(f"    姿态四元数={np.round(quaternion, 6)}")
    say(f"    夹爪点={np.round(gripper, 4)} m "
        f"(= 上报 + R @ {np.round(np.asarray(offset), 4)}, 只影响这一行打印)")

    # 与待机位的姿态夹角。2026-09-21 那次放置被拒收时, 「从待机位过来要转六十多度
    # 手腕」是当时的头号怀疑对象 (现场那组数算出来约 65 度), 所以这里顺手打出来。
    standby = params.get("standby_pose_reported_mm_deg")
    if standby and len(standby) == 6:
        standby_quaternion = rpy_to_quaternion(*[float(v) for v in standby[3:]])
        say(f"    与待机位姿态夹角: "
            f"{angle_between(quaternion, standby_quaternion):.1f} 度")

    # 单位自检: 位置按 mm 解释。按米填 (0.214 而不是 214) 会让上报原点变成
    # 0.000214 m, **贴着基座原点** —— 工作空间那条拦不住这种错 (z 下限是 -0.10),
    # 只有这一条能。
    if float(np.linalg.norm(reported)) < 0.05:
        say(f"[warn] 上报原点离基座原点只有 {np.linalg.norm(reported) * 1000.0:.1f} mm "
            "—— 位置是不是误按**米**填了 (应为 mm)?")

    limits = {
        axis: (params.get(f"workspace_{axis}_min"), params.get(f"workspace_{axis}_max"))
        for axis in ("x", "y", "z")
    }
    if all(low is not None and high is not None for low, high in limits.values()):
        # 夹爪点与上报原点**都要**查: 两者差 |offset| (本机 80mm), 一个在界内另一个
        # 仍可能出去 —— 同节点里 _check_workspace_pair 的理由。
        for label, point in (("上报原点", reported), ("夹爪点", gripper)):
            outside = []
            for index, axis in enumerate("xyz"):
                low, high = limits[axis]
                if not low <= point[index] <= high:
                    outside.append(
                        f"{axis}={point[index]:+.3f} 不在 [{low:+.2f}, {high:+.2f}]"
                    )
            if outside:
                say(f"[warn] {label} 超出 workspace 参数界 ({'; '.join(outside)}) "
                    "—— 执行节点会**拒绝下发**这个点 (这个脚本不替它拦)")

    if args.dry_run:
        say("")
        say("[dry-run] 只做了上面的换算, 一条指令都没发。去掉 --dry-run 才会真的发。")
        return 0

    # ── 起节点, 等反馈 ──
    rclpy.init()
    probe = Probe(topics, say)
    try:
        say("")
        say(f"等反馈 ({topics['tcp_pose_topic']} / {topics['arm_status_topic']}) ...")
        probe.spin_for(2.0)
        if probe.latest_pose is None:
            say(f"[error] 2 秒内没收到 {topics['tcp_pose_topic']} —— h3001_bridge 起了吗? "
                "试 `ros2 topic hz /feedback/tcp_pose`")
            return 2
        if probe.latest_status is None:
            # 位姿有、臂状态没有 —— 这**不是**「桥没起」, 单看这一条很容易白折腾。
            # 2026-09-21 实测: H3001ArmStatus 加了 ik_* 四个字段之后, 10:13 起着的那个
            # 桥进程还在按**旧字段布局**发, 而消息是按字段布局序列化的 —— 按新定义编出来
            # 的订阅端一条都解不出来, 而且**不报错**(rmw 层只看类型名, 不看字段数)。
            # 现场判据: tcp_pose / joint_states 照常 100Hz, 只有改过字段的那个话题是 0。
            say(f"[error] {topics['tcp_pose_topic']} 正常 (收得到位姿), 但 "
                f"{topics['arm_status_topic']} 一条都没收到 —— 判 IK 判决必须有它。")
            say("    最可能的原因: **h3001_msgs 加过字段之后 h3001_bridge 没重启**。")
            say("    消息按字段布局序列化, 老进程发的是旧布局 (字段数不同), 新定义编出来的")
            say("    订阅端收到也解不出来 —— 静默丢弃, 一条都不报。重启桥即可:")
            say("        Ctrl+C 掉 h3001_bridge 那个终端, 再重新 ros2 launch h3001_bridge ...")
            return 2
        _, start_position, _ = probe.latest_pose
        say(f"    当前上报原点={np.round(start_position, 4)} m, "
            f"到目标 {float(np.linalg.norm(reported - start_position)) * 1000.0:.1f} mm")
        _, status = probe.latest_status
        say(f"    当前臂状态: err_status={int(status.err_status)} "
            f"joint_angle_limit={[bool(v) for v in status.joint_angle_limit]} "
            f"motion_status={int(status.motion_status)}")
        if int(status.err_status) != 0 or any(status.joint_angle_limit):
            say("[warn] 机械臂现在就有故障/限位 —— 先把上面这些处理掉再试位姿")

        # ── 确认 ──
        say("")
        say("即将下发这一条指令 (机械臂会真的动):")
        say(f"    {np.round(reported, 4)} m + 姿态 {np.round(quaternion, 4)} "
            f"-> {topics['move_p_topic']}")
        if not args.yes:
            say("按 Enter 发送, Ctrl+C 放弃 ...")
            try:
                input()
            except (EOFError, KeyboardInterrupt):
                say("放弃。")
                return 0

        message = PoseStamped()
        message.header.stamp = probe.get_clock().now().to_msg()
        message.header.frame_id = topics["robot_base_frame"]
        message.pose.position.x = float(reported[0])
        message.pose.position.y = float(reported[1])
        message.pose.position.z = float(reported[2])
        message.pose.orientation.x = float(quaternion[0])
        message.pose.orientation.y = float(quaternion[1])
        message.pose.orientation.z = float(quaternion[2])
        message.pose.orientation.w = float(quaternion[3])

        # depth=1 + 默认 RELIABLE: 与执行节点那个发布器逐字一致 (它也是 create_publisher
        # (PoseStamped, topic, 1)); 桥接那边按 depth 1 订阅。发布器**不用** sensor_qos ——
        # 那是反馈话题的方向, 指令这边两边都是 RELIABLE。
        publisher = probe.create_publisher(PoseStamped, topics["move_p_topic"], 1)
        # 发之前先把「判决之前的那些帧」看一遍, 免得把上一条的旧判决当成这一条的
        # (节点里靠时间戳分, 这里靠这一行先清屏)。
        say("")
        say(f"已发送: {np.round(reported, 4)} m (帧 {topics['move_p_topic']})")
        sent_at = probe.now()
        message.header.stamp = probe.get_clock().now().to_msg()
        publisher.publish(message)

        grace = args.grace
        if grace is None:
            grace = float(params.get("ik_verdict_grace", 0.8))
        probe.spin_for(grace)

        # ── 判决 ──
        stamp, status = probe.latest_status
        fresh = stamp >= sent_at
        verdict = int(status.ik_status)
        # **不依赖 ik_status_valid**: 那个字段在真机上恒为 0 (SDK 的 get_state() 建
        # ArmState 副本时漏拷了它, sdk_linux.py:816) —— 2026-09-21 10:48 就是因为它
        # 白等了 30s。判据和执行节点保持一致: 固件说非 0 **且**机械臂一步没动。
        say("")
        if not fresh:
            say("[warn] 发出之后没收到新的臂状态帧 —— 下面的判决不算数, "
                "先确认桥接还在发 /feedback/arm_status")
        elif verdict:
            say(f"固件最近一次判决: {ik_status_text(verdict)} = 拒收。")
            say("    但这个值是 0x5A9 心跳里**粘住**的 (它是 ~50Hz 周期帧, 带的始终是")
            say("    「最近一次判决」, 拒收之后两分钟它还是这个数) —— 可能属于更早的")
            say("    某条指令, **单看它不算数**。作不作数看下面「动没动」。")
        else:
            say("固件最近一次判决: ik_status=0 (收下了)。")

        # ── 看运动 ──
        say("")
        say(f"看接下来 {args.monitor:.1f} 秒的运动 ...")
        probe.spin_for(args.monitor)
        _, final_position, _ = probe.latest_pose
        travelled = float(np.linalg.norm(final_position - start_position))
        residual = float(np.linalg.norm(reported - final_position))
        moved = travelled * 1000.0 >= 1.0
        say("")
        say(f"结果: 实际移动 {travelled * 1000.0:.1f} mm, "
            f"停在 {np.round(final_position, 4)} m, "
            f"距目标 {residual * 1000.0:.1f} mm")

        if not moved and fresh and verdict:
            say("")
            say(f"结论: **固件 IK 拒收** ({ik_status_text(verdict)}) —— "
                "目标没被接受, 机械臂一步没动。")
            say("    IK 是**逐次判决**的: 同一组数在不同关节构型下结果可能不同, "
                "所以从别的方向走近、或者改一改 RX/RY/RZ, 都可能就解出来了。")
            return 1
        if moved and fresh and verdict:
            say("(机械臂动过了 -> 这次的目标固件是收下了的; 上面那个非 0 值属于更早的"
                "某条指令, 与这次无关)")
        if not moved:
            say("一步没动, 而且固件没报拒收 —— 那要么是机械臂被挡住/没使能, "
                "要么状态帧没跟上 (见上面的 [warn])")
            return 1
        if residual <= args.tolerance:
            say("到位。这组数可用 —— 抄进 yaml 的 place_pose_reported_mm_deg 即可。")
            return 0
        say(f"动了但没到 (差 {residual * 1000.0:.1f} mm > 容差 "
            f"{args.tolerance * 1000.0:.1f} mm)。若它停在某个位置不动了, 看是不是"
            "被挡住、或者固件自己规划到别的地方去了。")
        return 1
    finally:
        probe.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
