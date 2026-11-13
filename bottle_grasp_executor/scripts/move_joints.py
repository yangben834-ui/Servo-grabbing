#!/usr/bin/env python3
"""按**关节角**下发一条 J 模式指令 (/control/move_j), 然后看它走没走完。

用法
------
    # 1) 只想看当前 6 个关节角 (抄一排数当模板, 一条指令都不发)
    /usr/bin/python3 move_joints.py --print-current

    # 2) 只校验 + 打印将要下发什么, 不碰 ROS 也不发指令 (最安全)
    /usr/bin/python3 move_joints.py 0 -30 45 0 60 0 --dry-run

    # 3) 真的发 (机械臂会动, 发送前会打印每轴差值并等 Enter)
    /usr/bin/python3 move_joints.py 0 -30 45 0 60 0

    # 4) 跳过 Enter (自己确认过再这么用), 只允许单轴不超过 45 度
    cd bottle_grasp_executor/scripts
    /usr/bin/python3 move_joints.py 0 -30 45 0 60 0 --yes --max-delta 45

单位: 命令行收**度**, 上总线前换算成**弧度**
------------------------------------------
`_JOINT_NAMES` / 反馈 / 指令这几条话题在 ROS 侧一律是**弧度**
(`/feedback/joint_states` 与 `/control/move_j` 都是 JointState.rad, 桥接里
`_joints_to_deg` 再乘回 180/pi 给固件)。但人在这台机上手边能看到的角度都是**度**:
厂商 GUI、标定脚本、yaml 里 `standby_pose_reported_mm_deg` 那套 RPY 全是度。
所以这个脚本的**入参是度**, 并且在打印和日志里同时给出弧度 —— 差 57 倍的那个错误
(把度当弧度, 或者反过来) 在这里是灾难性的, 所以两种都写出来让人对得上。

为什么用 move_j 而不是 move_p / move_l
--------------------------------------
J 模式是**关节空间**的点到点: 六个轴各自从当前角走到目标角, 不经过笛卡尔直线, 也就
**没有 IK 解不解得出来的问题** —— 不会出现 `ik_status=2 (超出工作空间)` 那种拒收
(现场遇到过的那种: 位姿在参数界内、固件却说不可达)。手边想「让机械臂摆到某个姿势」
时, 关节角是比 XYZ+RPY 更直接、更可控的输入。

代价是**末端的路径形状由关节插值决定**, 可能划出一道弧线 (不是直线), 而且不是你能预先
算出来的那种 —— 周围有障碍、或者夹着东西的时候, 先用低速、少量地试。固件对关节目标
同样有判决 (超限位是 `ik_status=3`, 单步过大是 `ik_status=5`), 所以「没动」的排查
和 move_p 一样看 `/feedback/arm_status`。

**跑之前先确认**:
  - 终端 source 过工作区 (`source /opt/ros/humble/setup.bash` +
    `source install/setup.bash`) —— 这个脚本 import 了 `h3001_msgs`, 连 `--dry-run`
    也要 (它只是不连 ROS, 不是不需要 ROS 环境);
  - h3001_bridge 在跑 (`ros2 topic hz /feedback/joint_states` 有输出), 否则指令没人接;
  - **执行节点没在跑任务** —— 它自己也在发运动指令 (move_p / move_l), 两边的指令会
    互相穿插, 谁的都走不利索;
  - 机械臂周围没人, 而且**别夹着瓶子时跑**;
  - 速度不是这个脚本能定的: `/control/move_j` 用的是桥的 `speed_percent` (launch 参数,
    默认 10%), 想更慢要改 launch 并**重启桥** (README_参数配置.md §speed_percent)。

退出码: 0 = 到位 (每轴残差都在 --tolerance 内且停稳) / 1 = 没动、没到位或超时 /
2 = 环境或参数不对 (没反馈、关节角超限位、机械臂有故障、数不是 6 个之类)
"""

from __future__ import annotations

import argparse
import math
import sys
import time

import numpy as np

import rclpy
from h3001_msgs.msg import H3001ArmStatus
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState

# 与 h3001_bridge 里的同名量逐字一致 (那里是 _JOINT_NAMES / _DEG_TO_RAD / _RAD_TO_DEG)。
# 这两条话题都要**按名字**对, 不能按数组下标 —— 桥接反馈里确实填了 name, 但真出现
# 顺序不同的时候, 按顺序读会把 6 个轴整体错位, 那是能撞坏机械臂的错误。
JOINT_NAMES = [f"joint{i}" for i in range(1, 7)]
RAD_TO_DEG = 180.0 / math.pi
DEG_TO_RAD = math.pi / 180.0

DEFAULT_TOPICS = {
    "move_j_topic": "/control/move_j",
    "joint_states_topic": "/feedback/joint_states",
    "arm_status_topic": "/feedback/arm_status",
}

# 固件/URDF 的关节限位: ±180°。桥接判「碰到限位了」的余量也是 2°
# (h3001_bridge.py: `abs(state.joint_deg[i]) > (180.0 - 2.0)`) —— 这里用同一个数先拦一道,
# 免得把一条必然被固件以 ik_status=3 (超过关节限位) 拒收的指令发出去。
JOINT_LIMIT_DEG = 180.0
LIMIT_MARGIN_DEG = 2.0

# 「指令发出去了但一步没动」的判据: STALL_AFTER 秒后位移还不到 STALL_EPS 度就提醒一次
# (执行节点里 stall_detect_time / stall_detect_epsilon 是同一套思路)。
STALL_AFTER_S = 3.0
STALL_EPS_DEG = 0.2

# 单轴差值超过它就在确认前单独提一句 —— 这类数多半是抄错了一位, 不是真想这么动。
BIG_DELTA_DEG = 90.0


def say(text=""):
    """脚本的 stdout 是给人看的排障信息, 不做机器解析 (同 check_place_pose.py)。"""
    print(text, flush=True)


def angles_text(values_deg):
    """一排可直接复制回命令行的数 —— 与小数的位数保持一致。"""
    return " ".join(f"{value:.1f}" for value in values_deg)


def pairs_text(values_deg):
    """`j1=0.0 (0.000rad)` 这种逐轴对照, 用来一眼看出度/弧度有没有搞反。"""
    return "  ".join(
        f"j{index}={value:+.1f}deg({value * DEG_TO_RAD:+.3f}rad)"
        for index, value in enumerate(values_deg, start=1)
    )


class JointsNode(Node):
    """只订阅反馈 + 一个发布器: 收集关节角与臂状态, 需要时才发那一条指令。"""

    def __init__(self, topics, report):
        super().__init__("move_joints")
        self._report = report
        self._move_j_topic = topics["move_j_topic"]
        self.latest_joints = None      # (时钟秒, {名字: 弧度})
        self.latest_status = None      # (时钟秒, H3001ArmStatus)
        self._last_err = None
        # 发布器**在这里就建**, 不在 send() 里临时建: DDS 的发现是异步的, 建完立刻
        # publish 有可能赶在「订阅端匹配上」之前 —— 那样这一条指令就石沉大海, 而脚本
        # 只会报「一步没动」。放在这里, 后面那 2 秒等反馈的 spin 顺便把发现做完。
        # depth=1 + 默认 RELIABLE: 与桥接那边 create_subscription(JointState, ..., 1)
        # 逐字一致。发布器**不用** sensor_qos —— 那是反馈话题的方向, 指令这边两边都是
        # RELIABLE。
        self._move_j_pub = self.create_publisher(JointState, self._move_j_topic, 1)
        # qos_profile_sensor_data (BEST_EFFORT): h3001_bridge 这两个反馈发布器用的是
        # BEST_EFFORT/VOLATILE/depth 5, 订阅端**必须**也是 BEST_EFFORT —— 拿默认的
        # RELIABLE 订阅会一条都收不到 (rclpy 只在启动时打一行 incompatible QoS 的 warn,
        # 然后就静默收空)。执行节点与 check_place_pose.py 用的也是这个 profile。
        self.create_subscription(
            JointState,
            topics["joint_states_topic"],
            self._on_joints,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            H3001ArmStatus,
            topics["arm_status_topic"],
            self._on_status,
            qos_profile_sensor_data,
        )

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_joints(self, msg):
        self.latest_joints = (self.now(), dict(zip(msg.name, msg.position)))

    def _on_status(self, msg):
        self.latest_status = (self.now(), msg)
        # 只在故障位**变化**时打印 (100Hz 会刷屏)。限位位不在这里打 —— 它是「碰到了」
        # 的瞬时状态, 由 current_deg() 的调用方按需看, 见 main 里的前置检查。
        err = int(msg.err_status)
        if err != self._last_err:
            self._last_err = err
            self._report(f"    [反馈] err_status={err}")

    def current_deg(self):
        """当前 6 个关节角 (度, 按 JOINT_NAMES 顺序); 反馈还没到就返回 None。"""
        if self.latest_joints is None:
            return None
        _, positions = self.latest_joints
        if not all(name in positions for name in JOINT_NAMES):
            return None
        return np.asarray(
            [positions[name] * RAD_TO_DEG for name in JOINT_NAMES], dtype=np.float64
        )

    def spin_for(self, seconds):
        """转给定时长 (只收反馈, 不发任何东西)。"""
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.02)

    def send(self, target_deg):
        """把一条关节目标 (度) 发到 /control/move_j (线上是弧度)。"""
        message = JointState()
        message.header.stamp = self.get_clock().now().to_msg()
        message.name = list(JOINT_NAMES)
        message.position = [float(value) * DEG_TO_RAD for value in target_deg]
        self._move_j_pub.publish(message)


def normalize_angle_tokens(argv):
    """把「连逗号一起粘贴」的关节角拆成独立 token。

    关节角最常见的写法是 `-51.3, 59.4, ...`, 从文档 / 厂商 UI / 上一行输出里复制过来时
    逗号会一起带进来: 整串加引号粘 → 1 个 token; 不加引号按空格粘 → 6 个 token 每个带个尾逗号。
    两种都得收, 否则用户看到的是 `invalid float value: '59.385,'` 这种与意图无关的报错。

    只在「整段每个逗号分片都能解析成 float」时才拆 —— 所以 `/control/move_j` 这种
    带下划线的话题名、`--yes` 这种开关都不会被误伤 (它们要么没逗号, 要么分片解析不过)。
    """
    out = []
    for token in argv:
        if "," not in token:
            out.append(token)
            continue
        pieces = [piece.strip() for piece in token.split(",")]
        pieces = [piece for piece in pieces if piece]
        if all(_is_float(piece) for piece in pieces):
            out.extend(pieces)
        else:
            out.append(token)
    return out


def _is_float(text):
    try:
        float(text)
    except ValueError:
        return False
    return True


def parse_args(argv):
    if argv is None:
        argv = sys.argv[1:]
    parser = argparse.ArgumentParser(
        description="按关节角下发一条 J 模式指令 (/control/move_j)",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=(
            "例: %(prog)s 0 -30 45 0 60 0        # 6 个角, 单位**度**\n"
            "     %(prog)s '0, -30, 45, 0, 60, 0'   # 带逗号粘也行"
        ),
    )
    parser.add_argument(
        "angles",
        nargs="*",
        type=float,
        help=f"6 个关节角 (**度**, 顺序 {', '.join(JOINT_NAMES)}; 可带逗号)",
    )
    parser.add_argument(
        "--print-current",
        action="store_true",
        help="只打印当前 6 个关节角 (可直接复制的那一排), 一条指令都不发",
    )
    parser.add_argument(
        "--dry-run", action="store_true", help="只校验并打印将要下发什么, 不碰 ROS"
    )
    parser.add_argument("--yes", action="store_true", help="跳过 Enter 确认")
    parser.add_argument(
        "--tolerance",
        type=float,
        default=1.0,
        help="判到位的单轴残差上限 (度, 默认 1.0)",
    )
    parser.add_argument(
        "--settle",
        type=float,
        default=0.5,
        help="进容差后还要稳多久才算到位 (秒, 默认 0.5)",
    )
    parser.add_argument(
        "--timeout", type=float, default=30.0, help="等它走完的上限 (秒, 默认 30)"
    )
    parser.add_argument(
        "--max-delta",
        type=float,
        default=0.0,
        help="单轴位移上限 (度); 超过就**拒发**。0 = 不拦 (默认, 仍然会警告)",
    )
    parser.add_argument(
        "--move-j-topic", default=DEFAULT_TOPICS["move_j_topic"], help="指令话题"
    )
    parser.add_argument(
        "--joint-states-topic",
        default=DEFAULT_TOPICS["joint_states_topic"],
        help="关节反馈话题",
    )
    parser.add_argument(
        "--arm-status-topic",
        default=DEFAULT_TOPICS["arm_status_topic"],
        help="臂状态话题 (查故障)",
    )
    return parser.parse_args(normalize_angle_tokens(argv))


def limit_problems(target_deg):
    """超限位 / 非有限的检查; 返回 (给人看的问题清单, 其中有没有真超限位)。

    两个分开返回是因为**收尾那句提示只对超限位成立** —— `ik_status=3` 是固件对「超了
    关节限位」的判决, 拿它去解释「数不是有限值」是误导。
    """
    problems = []
    limit_violation = False
    for index, value in enumerate(target_deg, start=1):
        if not math.isfinite(value):
            problems.append(f"j{index} 不是有限数 ({value})")
            continue
        if abs(value) > JOINT_LIMIT_DEG - LIMIT_MARGIN_DEG:
            limit_violation = True
            problems.append(
                f"j{index}={value:+.1f}deg 超出限位 (固件/URDF 是 ±{JOINT_LIMIT_DEG:.0f}deg, "
                f"这里按 ±{JOINT_LIMIT_DEG - LIMIT_MARGIN_DEG:.0f}deg 拦)"
            )
    return problems, limit_violation


def main(argv=None):
    args = parse_args(argv if argv is not None else sys.argv[1:])
    topics = {
        "move_j_topic": args.move_j_topic,
        "joint_states_topic": args.joint_states_topic,
        "arm_status_topic": args.arm_status_topic,
    }

    if args.print_current:
        return print_current(topics)

    if len(args.angles) != 6:
        say(f"[error] 要给 6 个关节角, 实际给了 {len(args.angles)} 个")
        say(f"    顺序: {', '.join(JOINT_NAMES)}  (单位: 度)")
        say("    想看当前值当模板就加 --print-current")
        return 2
    target_deg = np.asarray(args.angles, dtype=np.float64)

    say(f"目标关节角 (度): {angles_text(target_deg)}")
    say(f"            {pairs_text(target_deg)}")
    problems, limit_violation = limit_problems(target_deg)
    if problems:
        say("")
        for problem in problems:
            say(f"[error] {problem}")
        if limit_violation:
            say("    超限位的指令固件会以 ik_status=3 拒收; 先改数, 不要靠试。")
        return 2

    if args.dry_run:
        say("")
        say("[dry-run] 校验通过, 但一条指令都没发 (也没连 ROS)。去掉 --dry-run 才会真的发。")
        say("    真发之前会先打印「当前 vs 目标」的逐轴差值, 再等你按 Enter。")
        return 0

    # ── 起节点, 等反馈 ──
    rclpy.init()
    node = JointsNode(topics, say)
    try:
        say("")
        say(f"等反馈 ({topics['joint_states_topic']}) ...")
        # 反馈是 100Hz, 1 秒足够 —— 给 2 秒是因为桥刚起时 CAN 那条线还在握手。
        node.spin_for(2.0)
        current_deg = node.current_deg()
        if current_deg is None:
            say(f"[error] 2 秒内没收到 {topics['joint_states_topic']} —— h3001_bridge 起了吗?")
            say("    试 `ros2 topic hz /feedback/joint_states`; 报了 100Hz 却收不到,")
            say("    就是桥没重启 (改过消息定义之后必须重启, 见 README 那条)。")
            return 2
        if node.latest_status is None:
            say(f"[error] 关节反馈正常, 但 {topics['arm_status_topic']} 一条都没收到 ——")
            say("    查不了故障位。最可能的原因: **h3001_msgs 加过字段之后桥没重启** ——")
            say("    消息按字段布局序列化, 老进程发旧布局, 新定义编出来的订阅端静默丢一条")
            say("    都不报。重启桥即可。")
            return 2

        delta_deg = target_deg - current_deg
        say("")
        say(f"当前关节角 (度): {angles_text(current_deg)}")
        say(f"目标关节角 (度): {angles_text(target_deg)}")
        say(f"逐轴位移 (度):   {angles_text(delta_deg)}")
        say(f"                 {pairs_text(delta_deg)}")
        say(f"最大单轴位移:    {np.max(np.abs(delta_deg)):.1f}deg")
        big = np.where(np.abs(delta_deg) > BIG_DELTA_DEG)[0]
        if len(big):
            say("")
            say(f"[warn] j{', j'.join(str(i + 1) for i in big)} 一动就是 "
                f"{np.max(np.abs(delta_deg)):.0f}deg 以上 —— 先确认这组数不是抄错了一位")
        if args.max_delta > 0.0 and np.max(np.abs(delta_deg)) > args.max_delta:
            say("")
            say(f"[error] 最大单轴位移 {np.max(np.abs(delta_deg)):.1f}deg 超过 --max-delta "
                f"{args.max_delta:.1f}deg, 拒发")
            return 2

        _, status = node.latest_status
        if int(status.err_status) != 0:
            # 有电机故障时固件不会好好执行运动指令, 而且故障位自己不会消 —— 让它先清。
            say("")
            say(f"[error] 机械臂报故障: err_status={int(status.err_status)} —— "
                "先用厂商 UI「电机状态 → 节点/轴 → 指定轴清错」清掉再跑")
            say("    (位 i 对应 J(i+1) 的 motor_error_code, 见 README 排障表那行)")
            return 2
        if any(status.joint_angle_limit):
            say("")
            say(f"[warn] 现在就有轴在限位附近: {[bool(v) for v in status.joint_angle_limit]} "
                "—— 再往同一个方向走会被固件拒收 (ik_status=3)")
        say("")
        say(f"    [反馈] 现在: err_status={int(status.err_status)}, "
            f"motion_status={int(status.motion_status)}")

        # ── 确认 ──
        say("")
        say("即将下发这一条指令 (机械臂会真的动, J 模式是关节空间点到点):")
        say(f"    {angles_text(target_deg)} deg -> {topics['move_j_topic']}")
        if not args.yes:
            say("按 Enter 发送, Ctrl+C 放弃 ...")
            try:
                input()
            except (EOFError, KeyboardInterrupt):
                say("放弃。")
                return 0

        sent_at = time.monotonic()   # 单调时钟: 下面那句「发出多久了」不能用 ROS 时钟比
        node.send(target_deg)
        say("")
        say(f"已发送 (帧 {topics['move_j_topic']}), 等它走完 "
            f"(上限 {args.timeout:.0f}s, 容差 {args.tolerance:.1f}deg) ...")

        # ── 盯着走 ──
        start_deg = current_deg
        deadline = time.monotonic() + args.timeout
        settled_since = None
        stalled_warned = False
        arrived = False
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.02)
            current = node.current_deg()
            if current is None:
                continue
            residual = np.abs(current - target_deg)
            if residual.max() <= args.tolerance:
                if settled_since is None:
                    settled_since = time.monotonic()
                elif time.monotonic() - settled_since >= args.settle:
                    arrived = True
                    break
            else:
                settled_since = None
            # 发出去了但一步没动: 与执行节点同一套判据 (stall_detect_*), 只提醒一次,
            # 不打断 —— 慢慢起步也是可能的。
            if (
                not stalled_warned
                and time.monotonic() - sent_at > STALL_AFTER_S
                and np.max(np.abs(current - start_deg)) < STALL_EPS_DEG
            ):
                stalled_warned = True
                say("")
                say(f"[warn] 指令已发出 {STALL_AFTER_S:.0f}s, 六个轴一个都没动 "
                    f"(位移 < {STALL_EPS_DEG}deg) —— 查 /feedback/arm_status 的 "
                    "ik_status: 3=超过关节限位, 5=关节单步过大")

        final_deg = node.current_deg()
        if final_deg is None:
            say("[error] 中途收不到关节反馈了 —— 桥还在吗?")
            return 1
        residual_deg = final_deg - target_deg
        travelled = np.max(np.abs(final_deg - start_deg))
        say("")
        say(f"结果: 停止在 {angles_text(final_deg)} deg")
        say(f"      逐轴残差 {angles_text(residual_deg)} deg "
            f"(最大 {np.max(np.abs(residual_deg)):.2f}deg)")
        say(f"      相对起点最大单轴位移 {travelled:.1f}deg")
        if node.latest_status is not None:
            _, status = node.latest_status
            say(f"      err_status={int(status.err_status)} "
                f"motion_status={int(status.motion_status)}")

        if arrived:
            say("")
            say("到位。这排数可以用:")
            say(f"    {angles_text(final_deg)}")
            return 0
        say("")
        if travelled < STALL_EPS_DEG:
            say("结论: **一步没动** —— 命令没被执行。看 /feedback/arm_status 的 ik_status")
            say("    (3=超过关节限位, 5=关节单步过大), 以及机械臂是不是没使能/被挡住。")
        else:
            say(f"结论: 动了但没进容差 ({np.max(np.abs(residual_deg)):.2f}deg > "
                f"{args.tolerance:.1f}deg), 或者没在 {args.settle:.1f}s 内停稳。")
            say("    重力下垂会让它停在略低的位置; 要等更久就加大 --timeout,")
            say("    要放宽判据就加大 --tolerance。")
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


def print_current(topics):
    """只读: 打印当前关节角 (可直接抄回命令行), 一条指令都不发。"""
    rclpy.init()
    node = JointsNode(topics, say)
    try:
        node.spin_for(2.0)
        current_deg = node.current_deg()
        if current_deg is None:
            say(f"[error] 2 秒内没收到 {topics['joint_states_topic']} —— h3001_bridge 起了吗?")
            return 2
        say(f"当前关节角 (度): {angles_text(current_deg)}")
        say(f"            {pairs_text(current_deg)}")
        if node.latest_status is not None:
            _, status = node.latest_status
            say(f"err_status={int(status.err_status)} "
                f"joint_angle_limit={[bool(v) for v in status.joint_angle_limit]}")
        say("")
        say("复制这一排就是一条命令:")
        say(f"    /usr/bin/python3 {sys.argv[0]} {angles_text(current_deg)} --dry-run")
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
