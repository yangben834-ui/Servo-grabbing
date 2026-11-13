#!/usr/bin/env python3
"""H3001 机械臂 CAN ↔ ROS 2 桥接节点。

将 H3001 Python SDK 的 CAN 通信封装为 ROS 话题接口，
使上游视觉伺服代码无需修改即可驱动 H3001 机械臂。

发布话题 (反馈):
  /feedback/tcp_pose       (geometry_msgs/PoseStamped)  末端位姿, base_link 系, 米/弧度
  /feedback/joint_states   (sensor_msgs/JointState)      关节角, 弧度
  /feedback/arm_status     (h3001_msgs/H3001ArmStatus)   臂状态

订阅话题 (控制):
  /control/move_p          (geometry_msgs/PoseStamped)   笛卡尔位姿目标 (MOVE_P, 点到点), 米/弧度
  /control/move_l          (geometry_msgs/PoseStamped)   笛卡尔位姿目标 (MOVE_L, 直线插补), 米/弧度
  /control/move_j          (sensor_msgs/JointState)       关节目标, 弧度
  /control/joint_states    (sensor_msgs/JointState)       KDL 伺服关节命令, 弧度

move_p 与 move_l 收的是同一种消息、同一套单位, 只有固件模式不同 —— 下降 / 抬升这类
需要走直线的段用 move_l, 其余用 move_p。两者的入口顺序不同, 理由见各自回调的注释。
"""

import math
import sys
import os
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from h3001_msgs.msg import H3001ArmStatus

# 将 H3001 SDK 路径加入 sys.path。
# 优先级: 环境变量 H3001_SDK_PATH > 工作区内的最新 Linux SDK > 历史候选路径。
# (2026-09-16 修正: 原来是正序 insert(0, p), 后插入的排到最前, 实际优先级与注释
#  正好相反 —— 硬编码的历史路径排在 H3001_SDK_PATH 前面, 环境变量形同虚设,
#  换了 SDK 也生效不了。)
_SDK_SEARCH_PATHS = [
    os.environ.get("H3001_SDK_PATH", ""),
    # **工作区里那份** (2026-09-28 加): 上面那句注释一直写着「工作区内的最新 Linux
    # SDK」, 但这一项以前**根本不存在** —— 除环境变量外四条全是老机器的硬编码路径。
    # 结果是换一台机器 (或换一个工作区目录) 之后, 不导出 H3001_SDK_PATH 就必然
    # `RuntimeError: 无法导入 sdk_linux`, 而报错里列出的候选目录全是老路径, 看着
    # 像「SDK 丢了」。按 cwd 找是因为工作区的标准跑法就是 `cd <工作区> && ros2 launch`,
    # 模块级求值那一下拿到的就是工作区根。
    os.path.join(os.getcwd(), "SDK_linux"),
    "/home/qin/zdl_fz/src/SDK_linux",
    "/home/qin/zdl_fz/src/sdk_can/h3001sdk",
    os.path.expanduser("~/下载/sdk_can/h3001sdk"),
    os.path.expanduser("~/zdl_fz/src/h3001_bridge/sdk"),
]
# 先按优先级去重, 再倒序 insert(0): 边遍历边 insert 是不行的 —— 若环境变量与
# 后面某个候选指向同一个目录, 处理到后一项时会被 "已在 sys.path" 的判断跳过,
# 那一次 insert 就不发生, 结果环境变量反而排在硬编码路径后面 (实测踩到过)。
# 每个目录都先摘掉已有的同路径再插到最前, 这样顺序与 _SDK_SEARCH_PATHS 一致,
# 不受外部 PYTHONPATH 里已有条目的位置影响。
_ordered_sdk_paths = []
for _p in _SDK_SEARCH_PATHS:
    if _p and os.path.isdir(_p) and _p not in _ordered_sdk_paths:
        _ordered_sdk_paths.append(_p)
for _p in reversed(_ordered_sdk_paths):
    sys.path[:] = [item for item in sys.path if item != _p]
    sys.path.insert(0, _p)

try:
    # 模块名是 sdk_linux (2026-09-16 起), 不是旧版的 h3001_sdk。
    # 两者在本节点用到的接口上完全一致, 但**不要**写成 "先 sdk_linux 失败再退回
    # h3001_sdk" 的二选一 —— 路径配错时静默用回旧 SDK 正是上一版最难查的坑。
    from sdk_linux import H3001Arm, MoveMode
except (ImportError, SystemExit) as exc:
    # sdk_linux.py 在缺少 python-can 时是 raise SystemExit, 不是 ImportError;
    # 不接住的话节点会被一个无上下文的 SystemExit 打断。
    raise RuntimeError(
        "无法导入 sdk_linux (在该目录下未找到 sdk_linux.py)。"
        "已尝试的路径: "
        + ", ".join(repr(p) for p in _SDK_SEARCH_PATHS if p)
        + "。请用环境变量 H3001_SDK_PATH 指向 SDK 目录, 例如 "
        "export H3001_SDK_PATH=/home/qin/zdl_fz/src/SDK_linux"
    ) from exc

# 实际加载到的 SDK 文件, 启动时打一行日志。
# 为什么值得专门记一笔: 候选目录里有**不止一份** sdk_linux.py (2026-09-16 核查:
# src/SDK_linux 是 09-15 版, src/sdk_can/h3001sdk 里还有一份 09-02 版)。
# 也就是说路径一旦配偏, import 依然成功、只是加载到旧修订 —— 没有报错、没有
# 任何异常, 只有行为不一样。日志里有这个路径, 才可能在排查时看出这种事。
_SDK_FILE = getattr(sys.modules.get("sdk_linux"), "__file__", "<未知>")

_JOINT_NAMES = [f"joint{i}" for i in range(1, 7)]
_DEG_TO_RAD = math.pi / 180.0
_RAD_TO_DEG = 180.0 / math.pi
_MM_TO_M = 0.001
_M_TO_MM = 1000.0


def _pose_to_list(pose: PoseStamped, invert_rx_ry: bool = False) -> list:
    """将 ROS PoseStamped (m/rad) 转为 H3001 SDK 列表 [X,Y,Z,RX,RY,RZ] (mm/deg)。

    invert_rx_ry 见 _publish_feedback 的说明; 指令方向与反馈方向必须用同一个开关,
    否则 current_tcp 模式下「反馈原样回发」会转成相反姿态。
    """
    x = pose.pose.position.x * _M_TO_MM
    y = pose.pose.position.y * _M_TO_MM
    z = pose.pose.position.z * _M_TO_MM
    # 从四元数提取 RPY (简化: 假设上游发送的 orientation 可转换为 RPY)
    q = pose.pose.orientation
    # 使用 ZYX 顺序欧拉角提取
    sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z)
    cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
    rx = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (q.w * q.y - q.z * q.x)
    sinp = max(-1.0, min(1.0, sinp))
    ry = math.asin(sinp)

    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    rz = math.atan2(siny_cosp, cosy_cosp)

    if invert_rx_ry:
        # ROS 约定 -> 固件约定。取反是自逆的, 所以与反馈方向是同一个操作。
        rx, ry = -rx, -ry
    return [x, y, z, rx * _RAD_TO_DEG, ry * _RAD_TO_DEG, rz * _RAD_TO_DEG]


def _pose_command_text(target: list, target_proto: list) -> str:
    """move_p / move_l 日志里共用的那一串: mm/deg 值 + 实际发出去的固件整数。

    固件值是跟 [start] 那类日志对齐用的排障信息 —— 单位是 0.001mm/0.001deg,
    和 target 的倍数关系一眼能对上, 所以两个都打。
    """
    return (
        f"[{target[0]:.1f}, {target[1]:.1f}, {target[2]:.1f}, "
        f"{target[3]:.1f}, {target[4]:.1f}, {target[5]:.1f}] mm/deg "
        f"(固件值 [{target_proto[0]}, {target_proto[1]}, {target_proto[2]}, "
        f"{target_proto[3]}, {target_proto[4]}, {target_proto[5]}])"
    )


def _joints_to_deg(msg: JointState) -> list:
    """将 JointState (rad) 按 _JOINT_NAMES 顺序转为 deg 列表。"""
    name_to_pos = dict(zip(msg.name, msg.position))
    result = []
    for jname in _JOINT_NAMES:
        if jname not in name_to_pos:
            raise ValueError(f"缺少关节 {jname}")
        result.append(name_to_pos[jname] * _RAD_TO_DEG)
    return result


class H3001Bridge(Node):
    """CAN ↔ ROS 桥接节点。"""

    def __init__(self):
        super().__init__("h3001_bridge")

        # ── 参数 ──
        self.declare_parameter("can_interface", "socketcan")
        self.declare_parameter("can_channel", "can0")
        self.declare_parameter("can_bitrate", 1_000_000)
        self.declare_parameter("feedback_rate", 100.0)
        self.declare_parameter("robot_base_frame", "base_link")
        self.declare_parameter("auto_enable", True)
        # MCU 运动速度百分比 (0~100)。测试阶段建议 10，确认无误后再逐步提高。
        self.declare_parameter("speed_percent", 10)
        # 固件 RPY 约定修正, 详见 _publish_feedback。2026-09-15 起默认关闭,
        # 原因见 config/h3001_bridge.yaml 的注释 (与 side_orientation_xyzw 不自洽)。
        # 这里只是兜底默认 (ros2 run 不经 launch/yaml 时才生效)。
        self.declare_parameter("invert_rx_ry", False)

        can_interface = self.get_parameter("can_interface").value
        can_channel = self.get_parameter("can_channel").value
        can_bitrate = self.get_parameter("can_bitrate").value
        feedback_rate = self.get_parameter("feedback_rate").value
        self._base_frame = self.get_parameter("robot_base_frame").value
        auto_enable = self.get_parameter("auto_enable").value
        self._speed_percent = int(self.get_parameter("speed_percent").value)
        self._invert_rx_ry = bool(self.get_parameter("invert_rx_ry").value)

        # ── SDK 连接 ──
        # 先报加载到的是哪一份 SDK: 候选目录里存在多个 sdk_linux.py 修订版,
        # 路径配偏也能 import 成功 (见模块顶部 _SDK_FILE 的说明)。
        self.get_logger().info(f"H3001 SDK: {_SDK_FILE}")
        self._arm = H3001Arm(
            interface=can_interface,
            channel=str(can_channel),
            bitrate=int(can_bitrate),
        )
        self._lock = threading.RLock()
        self._arm.connect(wait_feedback=True, timeout=3.0)
        self.get_logger().info(
            f"H3001 CAN 已连接 interface={can_interface} channel={can_channel}"
        )

        if auto_enable:
            self._arm.enable_all_motors()
            self.get_logger().info("六轴电机已使能")

        # ── 反馈发布器 ──
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        self._tcp_pub = self.create_publisher(PoseStamped, "/feedback/tcp_pose", sensor_qos)
        self._joint_pub = self.create_publisher(JointState, "/feedback/joint_states", sensor_qos)
        self._status_pub = self.create_publisher(H3001ArmStatus, "/feedback/arm_status", sensor_qos)

        # ── 控制订阅 ──
        self.create_subscription(PoseStamped, "/control/move_p", self._on_move_p, 1)
        self.create_subscription(PoseStamped, "/control/move_l", self._on_move_l, 1)
        self.create_subscription(JointState, "/control/move_j", self._on_move_j, 1)
        self.create_subscription(JointState, "/control/joint_states", self._on_joint_command, 1)

        # ── 反馈定时线程 ──
        self._feedback_timer = self.create_timer(
            1.0 / feedback_rate, self._publish_feedback
        )

        # ── 上一次关节命令时间 (用于模式切换防抖) ──
        self._last_joint_mode = None  # "P" / "L" / "J"

        # ── 上一次 IK 诊断快照 (只在变化时打印, 见 _publish_feedback) ──
        self._last_ik_snapshot = None

        self.get_logger().info(
            f"H3001 桥接节点就绪: 反馈频率={feedback_rate}Hz, "
            f"base_frame={self._base_frame}, 速度={self._speed_percent}%, "
            f"invert_rx_ry={self._invert_rx_ry}"
        )
        if self._invert_rx_ry:
            self.get_logger().warn(
                "已启用 rx/ry 取反 (H3001 固件 RPY 约定修正, 临时方案)。"
                "注意: 此时 executor 的 side_orientation_xyzw 也必须换成 x、y 取反后的值, "
                "否则会偏离记录姿态约 14 度 (详见 config/h3001_bridge.yaml 注释)"
            )

    # ── 反馈发布 ──────────────────────────────────────────────

    def _publish_feedback(self):
        """定时读取 SDK 状态并发布为 ROS 话题。"""
        with self._lock:
            try:
                state = self._arm.get_state()
            except Exception as exc:
                self.get_logger().warn(f"读取 H3001 状态失败: {exc}")
                return

        # IK / 运动诊断: 值变化时打印一次。
        # SDK 单独反馈了 MCU 的 IK 校验结果。「MCU 收下目标却不执行」在 ROS 侧原本完全
        # 看不见: 2026-09-14 下降卡死就是这样 —— 命令正常转发、机械臂不动、最后只报运动
        # 超时, 无法区分「夹爪被挡住推不动」和「IK 无解 / 关节限位」。
        # 2026-09-21 起这 4 个值同时进了 H3001ArmStatus.msg (ik_status 等), 执行节点
        # 可以据此立刻判失败; 这里保留这行日志, 是因为它是**逐次变化**的原始记录,
        # 排查时一眼能看出「哪一条 move_p 被拒了」—— 消息里只有最新值。
        ik_snapshot = (
            state.ik_status,
            state.ik_status_valid,
            state.ik_validation_ok,
            state.ik_motion_state,
            state.ik_pose_pending,
            state.ik_joint_limit_fault,
        )
        if ik_snapshot != self._last_ik_snapshot:
            self._last_ik_snapshot = ik_snapshot
            self.get_logger().info(
                f"IK 状态变化: status={state.ik_status} "
                f"valid={state.ik_status_valid} "
                f"validation_ok={state.ik_validation_ok} "
                f"motion={state.ik_motion_state} "
                f"pose_pending={state.ik_pose_pending} "
                f"limit_fault={state.ik_joint_limit_fault}"
            )

        now_msg = self.get_clock().now().to_msg()

        # 1) TCP 位姿
        pose_msg = PoseStamped()
        pose_msg.header.stamp = now_msg
        pose_msg.header.frame_id = self._base_frame
        # state.pose = [X_mm, Y_mm, Z_mm, RX_deg, RY_deg, RZ_deg]
        pose_msg.pose.position.x = state.pose[0] * _MM_TO_M
        pose_msg.pose.position.y = state.pose[1] * _MM_TO_M
        pose_msg.pose.position.z = state.pose[2] * _MM_TO_M
        # RPY → 四元数
        rx = state.pose[3] * _DEG_TO_RAD
        ry = state.pose[4] * _DEG_TO_RAD
        rz = state.pose[5] * _DEG_TO_RAD
        if self._invert_rx_ry:
            # ── 固件约定修正 (临时方案) ──
            # H3001 固件上报的 RPY 与 ROS/松灵约定不一致: 直接按 ZYX 转四元数得到的
            # 姿态方向相反, 所以这里对 rx/ry 取反。
            # 取反不是近似: 它精确等价于对旋转矩阵做 Rz(pi) 共轭,
            #   R_ros = Rz(pi) @ R_fw @ Rz(pi)^T
            # 也就是「固件基座系相对 ROS 基座系绕 Z 差了 180 度」。
            #
            # 必须与 _pose_to_list 用同一个开关成对生效 —— 只改一侧会让
            # orientation_source=current_tcp 的「反馈姿态原样回发」转成反向姿态。
            #
            # 2026-09-15 按何工要求加入。新固件若修正了此约定, 把 invert_rx_ry
            # 置 false 即可, 下游无需改动。
            rx, ry = -rx, -ry
        q = _rpy_to_quaternion(rx, ry, rz)
        pose_msg.pose.orientation.x = q[0]
        pose_msg.pose.orientation.y = q[1]
        pose_msg.pose.orientation.z = q[2]
        pose_msg.pose.orientation.w = q[3]
        self._tcp_pub.publish(pose_msg)

        # 2) 关节状态
        joint_msg = JointState()
        joint_msg.header.stamp = now_msg
        joint_msg.name = list(_JOINT_NAMES)
        joint_msg.position = [deg * _DEG_TO_RAD for deg in state.joint_deg]
        joint_msg.velocity = [0.0] * 6
        joint_msg.effort = [0.0] * 6
        self._joint_pub.publish(joint_msg)

        # 3) 臂状态
        status_msg = H3001ArmStatus()
        # H3001 control_mode: 0=位置模式(正常待机)
        # 上游代码期望 ctrl_mode==1 表示就绪，做映射。
        status_msg.ctrl_mode = 1 if state.control_mode == 0 else state.control_mode
        status_msg.arm_status = state.arm_state
        status_msg.mode_feedback = state.move_mode
        status_msg.teach_status = state.teach_state
        status_msg.motion_status = state.motion_state
        status_msg.trajectory_num = state.trajectory_index
        # MCU 的 IK 判决 (2026-09-21)。原样透传, 一个值都不加工 —— 判据在消费侧
        # (执行节点 _ik_rejection_problem): ik_status 非 0 且 ik_status_valid 为 1
        # 才算「拒收」, valid=0 时 SDK 的默认 0 分不清「收下」和「没收到过」。
        status_msg.ik_status = int(state.ik_status) & 0xFF
        status_msg.ik_status_valid = int(state.ik_status_valid) & 0xFF
        status_msg.ik_joint_limit_fault = int(state.ik_joint_limit_fault) & 0xFF
        status_msg.ik_pose_pending = int(state.ik_pose_pending) & 0xFF
        # 综合错误: 任何电机故障 → err_status
        err = 0
        for i in range(6):
            if state.motor_fault[i]:
                err |= (1 << i)
            if state.motor_error_code[i]:
                err |= (1 << (i + 8))
        status_msg.err_status = err
        # 关节限位: 从 joint_deg 与 URDF 限位 (±180°) 比较
        limit_margin = 2.0  # deg
        status_msg.joint_angle_limit = [
            abs(state.joint_deg[i]) > (180.0 - limit_margin) for i in range(6)
        ]
        # 通信状态: joint_comm_fault 按位解析
        status_msg.communication_status_joint = [
            bool(state.joint_comm_fault & (1 << i)) for i in range(6)
        ]
        self._status_pub.publish(status_msg)

    # ── 控制回调 ──────────────────────────────────────────────

    def _pose_command_proto(self, msg: PoseStamped, label: str):
        """PoseStamped -> (mm/deg 列表, 固件整数列表); 转换失败返回 None 并记 error。

        共有这一段是因为 move_p / move_l 收的是同一种消息、同一套单位, 差别只在固件
        模式和入口顺序 —— 单位换算不该有两份。

        单位约定 (与官方 web server / test_ik_move.py 一致):
          get_pose()/反馈 为 mm/deg；send_target_pose 接收 0.001mm/0.001deg
          整数 (即 mm/deg × 1000)，SDK 内部仅 round 不换算。
        """
        try:
            target = _pose_to_list(msg, invert_rx_ry=self._invert_rx_ry)
        except Exception as exc:
            self.get_logger().error(f"{label} 四元数转换失败: {exc}")
            return None
        return target, [round(v * 1000.0) for v in target]

    def _on_move_p(self, msg: PoseStamped):
        """处理 /control/move_p 笛卡尔位姿命令 (MOVE_P, 点到点)。"""
        converted = self._pose_command_proto(msg, "move_p")
        if converted is None:
            return
        target, target_proto = converted
        with self._lock:
            try:
                # 顺序不能反: 先设模式与速度, 再发位姿。
                # MCU 收到完整位姿就立即按「当时的速度」起步 —— 若把 enter_can_mode 放在
                # 位姿之后 (SDK 的 send_linear_pose / send_position_targets 就是这么写的),
                # 速度设置要 50ms 后才到, 对这一次运动已经无效。结果就是 speed_percent
                # 调到 5 机械臂依然按上一次的速度跑, 看起来「速度设置不起作用」。
                # SDK 的 send_circular_poses 是 mode-first 的写法, 注释也写明 MCU 会用收到的
                # 第一个完整位姿立即触发点到点运动。
                self._arm.enter_can_mode(move_mode=MoveMode.P, speed_percent=self._speed_percent)
                time.sleep(0.02)
                self._arm.send_target_pose(target_proto)
                self._last_joint_mode = "P"
                self.get_logger().info(
                    f"move_p 已发送: {_pose_command_text(target, target_proto)}"
                )
            except Exception as exc:
                self.get_logger().error(f"move_p 发送失败: {exc}")

    def _on_move_l(self, msg: PoseStamped):
        """处理 /control/move_l 笛卡尔位姿命令 (MOVE_L, 笛卡尔直线)。

        为什么要单独一条话题: MOVE_P 是固件自己的点到点规划, 中间怎么走由它定 ——
        下降 / 抬升这种要求工具沿一条直线走的段, 光靠 P 做不到。

        **入口顺序故意与 move_p 不同**: 这里调 SDK 自己的 send_linear_pose
        (先发位姿、再切 L 模式), 也就是厂商 web UI「发送直线目标(MOVE_L)」那条已知
        可用的路 (SDK_linux/h3001_web_server_linux.py 的 linear_move 分支)。
        没有照 move_p 改成 mode-first, 是因为那样多一种失败模式: 若 MCU 以「模式切换」
        作为 L 的触发点, mode-first 会立刻朝**上一条遗留目标**起一条 L 轨迹 —— 一次
        没人预期的运动。pose-first 最坏只是退化成点到点, 不会乱走。

        ⚠ 这个选择**没有被真机证实**, 判据是 /feedback/arm_status 的 mode_feedback:
        下降过程中它应为 2 (MOVE_L, 码表见 h3001_web/protocol_ui.js)。若仍是 0,
        说明 L 没生效 (可能是「收到完整位姿就起步」那套时序在 L 上也成立, 见
        _on_move_p 里那段顺序说明) —— 那时把下面换成 mode-first 再测同一目标。
        """
        converted = self._pose_command_proto(msg, "move_l")
        if converted is None:
            return
        target, target_proto = converted
        with self._lock:
            try:
                self._arm.send_linear_pose(
                    target_proto, speed_percent=self._speed_percent
                )
                # 这条记账别漏: _on_joint_command 靠它判断「上次是不是 J 模式」,
                # 不置 "L" 的话下一次伺服命令不会重切模式。
                self._last_joint_mode = "L"
                self.get_logger().info(
                    f"move_l 已发送: {_pose_command_text(target, target_proto)}"
                )
            except Exception as exc:
                self.get_logger().error(f"move_l 发送失败: {exc}")

    def _on_move_j(self, msg: JointState):
        """处理 /control/move_j 关节命令。"""
        try:
            target_deg = _joints_to_deg(msg)
        except Exception as exc:
            self.get_logger().error(f"move_j 关节转换失败: {exc}")
            return
        with self._lock:
            try:
                # 同 _on_move_p: 必须先设模式与速度。send_position_targets 内部是
                # 「先发关节目标再切 J 模式」, 那一次的速度同样来不及生效, 所以这里
                # 先补一次 enter_can_mode (内部那次变成冗余, 幂等无害)。
                self._arm.enter_can_mode(move_mode=MoveMode.J, speed_percent=self._speed_percent)
                time.sleep(0.02)
                self._arm.send_position_targets(target_deg, speed_percent=self._speed_percent)
                self._last_joint_mode = "J"
                self.get_logger().info(
                    f"move_j 已发送: {[round(d, 2) for d in target_deg]} deg"
                )
            except Exception as exc:
                self.get_logger().error(f"move_j 发送失败: {exc}")

    def _on_joint_command(self, msg: JointState):
        """处理 /control/joint_states KDL 伺服关节命令。

        使用 send_direct_position 直接发送目标，不经过 MCU 轨迹规划。
        """
        try:
            target_deg = _joints_to_deg(msg)
        except Exception as exc:
            self.get_logger().error(f"joint_command 转换失败: {exc}")
            return
        with self._lock:
            try:
                # 如果上次不是 J 模式，先切换一次
                if self._last_joint_mode != "J":
                    self._arm.enter_can_mode(move_mode=MoveMode.J, speed_percent=self._speed_percent)
                    self._last_joint_mode = "J"
                self._arm.send_direct_position(target_deg)
            except Exception as exc:
                self.get_logger().error(f"joint_command 发送失败: {exc}")

    # ── 关闭 ──────────────────────────────────────────────

    def destroy_node(self):
        self.get_logger().info("正在关闭 H3001 桥接节点...")
        with self._lock:
            try:
                self._arm.disable_all_motors()
                self.get_logger().info("六轴电机已失能")
            except Exception as exc:
                self.get_logger().warn(f"电机失能失败: {exc}")
            try:
                self._arm.close()
                self.get_logger().info("CAN 连接已关闭")
            except Exception as exc:
                self.get_logger().warn(f"CAN 关闭失败: {exc}")
        super().destroy_node()


def _rpy_to_quaternion(roll: float, pitch: float, yaw: float) -> tuple:
    """ZYX 顺序 RPY → 四元数 (x, y, z, w)。"""
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    w = cr * cp * cy + sr * sp * sy
    return (x, y, z, w)


def main(args=None):
    rclpy.init(args=args)
    node = H3001Bridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
