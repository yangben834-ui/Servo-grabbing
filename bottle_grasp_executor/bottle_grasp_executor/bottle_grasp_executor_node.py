#!/usr/bin/env python3
"""瓶子抓取执行节点。

订阅 bottle_grasp_perception 发布的抓取候选 (相机光学坐标系 6D 位姿),
经手眼外参变换到 base_link, 按 side/top 模式选择固定 TCP 姿态,
通过 /control/move_p (点到点 MOVE_P) 与 /control/move_l (直线 MOVE_L) 驱动
H3001 机械臂完成:
预抓取 → 视觉下降 → 人工确认下降 → 抬升 → 移到放置点 → 放下 → 回待机位。

两条话题收的是同一种 PoseStamped、同一套单位, 差别只在桥下发的固件模式; 走哪条由
状态决定 (只有 DESCEND 走 move_l, 见 _LINEAR_MOTION_STATES)。

**2026-09-28 起运动指令优先走关节空间**: 位姿 -> 节点自己求逆解 -> /control/move_j。
理由与实测见 _IK_MOTION_STATES 和 _solve_joint_target 的注释 (固件的逐点逆解会在现场
那组抓取姿态上断裂, 09-28 抬升被拒收 `ik_status=5` 就是这么来的)。解不出来就**回落**
到上面那两条笛卡尔话题 —— 所以「求逆解失败」永远不是新的失败模式。总开关
`ik_motion_enabled`, 加载期校验不过 (pinocchio / URDF / CAD 限位缺一个) 会自动关掉。

接口:
  服务 ~/execute_grasp (std_srvs/Trigger)  循环模式的开关: 调一次 = 启动抓取任务并开启
                                          循环 (回到待机位后自动接着抓下一个),
                                          再调一次 = 关闭循环
  话题 ~/grasp_confirm (std_msgs/Bool)     人工确认 (三个确认点共用)
  终端 Enter                               等效于 grasp_confirm
  发布 /control/move_p | move_l (PoseStamped)  笛卡尔(点到点 / 直线), 回落时用
  发布 /control/move_j (JointState, 弧度)      关节空间, 优先用 (2026-09-28)
  订阅 /feedback/joint_states (JointState)     当前关节角 —— 求逆解的种子

循环模式 (2026-09-21): 原来这个服务是一次任务一次触发, 想抓下一个必须人再调一次。
现在调一次就**一直触发** —— 每一轮跑完、机械臂回到待机位后, 自动开始下一轮。
关闭有两条路: ①再调一次 ~/execute_grasp (不打断正在跑的那一次, 只在「回到待机位、
准备下一次」那一刻生效 —— 机械臂手里可能夹着瓶子, 中途拧断有掉瓶/撞压风险);
②让某一轮任务**真失败** (故障往往是系统性的, 继续循环只会一遍遍重跑整个流程再失败
—— 2026-09-21 三次固件 IK 拒收都是同一类)。

「视野里没有目标」**不算真失败** (2026-09-21 改): 跟丢目标、或者回到待机位后下一个
瓶子还没摆上来, 都只是回待机位接着等 (等的时候机械臂不动, 每 10s 一行「还在等」),
等多久都行, 不自动收工 —— 用户要的是「回到工作位置后继续接受物体识别到的坐标」。
判据是**观察时刻**: 只有回来之后新收到的候选才开下一轮, 上一轮那个用过的候选不会
被重复抓 (见 _tick_loop_wait)。开启循环要求 place_enabled=true: 关掉放置时每一轮只是
把瓶子在原地放下再抓起来, 同一个瓶子会无限循环。详见 README §5.14。

状态机:
  IDLE -> GRIPPER(张开) -> WAIT_TARGET -> PRE_GRASP -> TRACK_WAIT -> DESCEND
       -> GRASP_WAIT -> GRIPPER(闭合命令) -> LIFT -> PLACE_MOVE
       -> PLACE_WAIT -> GRIPPER(张开命令) -> HOME -> IDLE
  任意阶段机器人故障/反馈超时/工作空间拒绝/运动超时/夹爪动作失败 -> 任务结束:
    机械臂仍可控时先进 HOME (回用户设定的待机位, 到位判定照走; 超时按老拆段算式
    放大成 motion_timeout × 段数, 见 _active_motion_timeout),
    不可控时直接回 IDLE。HOME 期间不接受新的抓取任务 (循环开着时这一调只用来关闭循环)。

PRE_GRASP 在目标正上方 pre_grasp_offset_z 处悬停;
TRACK_WAIT 在悬停点跟随移动中的目标 (死区 + 限速, 见 track_* 参数), 并等人工确认;
DESCEND 从确认后的位置垂直下降到抓取点, 全程**一条指令**: 直接下降
pre_grasp_offset_z - grasp_offset_z (12cm: 代码默认与 yaml 现场值一致, 见下面参数注释),
发出后等 descend_dwell 秒就进
GRASP_WAIT —— 不判是否到位、不迭代末端修正、不分段;
LIFT 抬升 lift_distance;
PLACE_MOVE 把物体从抬升点**一条 move_p 送**到放置位姿 place_pose_reported_mm_deg 本身
(2026-09-18 改: 原来是「先平移到放置点正上方、再垂直下降」两段, 现在直接发那个坐标;
2026-09-20 又去掉拆段: 不再按 max_command_distance 插中间点, 路径全交给固件规划),
判到位; 到达后停 place_descend_dwell 秒进 PLACE_WAIT 放下;
放置位姿是**完整位姿**, 位置与姿态都来自参数, 与抓取姿态无关;
HOME 回到 standby_pose_reported_mm_deg 指定的待机位 (成功与失败都走,
可用 return_to_standby_on_finish 关闭): **一条绝对待机位指令**,
2026-09-21 起不再按 max_command_distance 拆段 (拆出来的中间点配上固定姿态, 固件 IK
解不出来 —— 见 _home_time_multiplier_for), 路径全交给固件规划。
place_enabled=false 时整段跳过, 退回 LIFT -> HOME 的旧流程。

TRACK_WAIT / GRASP_WAIT / PLACE_WAIT 是三个独立的人工确认点:
  TRACK_WAIT = 「目标已经跟到了, 现在下降」—— 确认后下降;
  GRASP_WAIT = 「现在夹」—— 确认后**节点去闭合夹爪**, 闭合成功才抬升;
  PLACE_WAIT = 「现在放」—— 确认后**节点去张开夹爪**, 张开成功才回待机位。
track_enabled=false 时跳过 TRACK_WAIT, 退回 PRE_GRASP -> DESCEND 的旧两阶段流程。

top_skip_track=true (2026-09-29) 时**只有倒瓶 (top) 模式**跳过 TRACK_WAIT: top 是工具
朝下、从正上方抓, 机械臂一挪到瓶子正上方手眼相机就看不着它了, 候选不再更新 ->
_follow_target 判 TRACK_TARGET_LOST 回待机位, 一轮都抓不成。所以 top 悬停到位直接
下降, 下降点用锁定目标那一刻的坐标 (见 _skip_track_for_mode)。side 不受影响。

后两个确认点各有开关可以关掉 (现场默认关):
  grasp_confirm_required=false -> GRASP_WAIT 不等 Enter, descend_dwell 一到就闭合;
  place_confirm_required=false -> PLACE_WAIT 不等 Enter, 到达放置点后停满
  place_descend_dwell 就张开。
两个都关掉时整条链**只剩 TRACK_WAIT 一次确认**, 之后夹、抬、搬、放、回位全自动
(回位仍由 return_to_standby_on_finish 控制)。代价见 _declare_parameters 里那两处注释:
没有「看一眼再动手」这一步了, 闭合/张开的时刻完全由两个 dwell 决定。

**夹爪**: gripper_enabled=true 时节点接管夹爪, 每次动作 spawn
scripts/gripper_grip_hold.py 一个子进程 —— **节点只发「关闭夹爪」「打开夹爪」两条
命令, 不看夹爪状态、不解析脚本输出** (2026-09-18 用户明确要求: 真机上夹爪明明已经
夹住了, 却因为脚本把「撞硬扭矩上限」判成失败而整趟任务作废)。脚本 stdout 上的
`[stop]` / `[warn]` 只进日志, 判成败只看退出码。

**唯一的例外是 `[holding]` 那一行** (2026-09-28): `gripper_hold_torque > 0` 时
闭合之后那个进程**不退出**, 留在原地持续发 MIT 力矩帧顶着夹持力 (固件有看门狗,
不发帧就切电, 所以力只能由主机持续发帧维持)。它不退出, 就没有退出码可等 ——
节点靠 `[holding]` 知道「力已经建立」, 抬升/搬运全程保持, 到放置时往它的 stdin
写一行 `OPEN` 让它松手。默认 `gripper_hold_torque: 0.0` = 关: 进程跑完就失能退出,
夹持力靠机构自锁 (老行为, 一个字节都没变)。详见脚本 docstring 与 README §5.13。
false 时完全不碰夹爪, 确认点的语义退回旧版
(「夹爪已经夹住」/「物体已经放下」, 夹爪由人工在另一个终端跑厂商的 04/05 操作) ——
**注意 false + 两个确认开关都关掉时, 节点既不动夹爪也不等人**, 到点直接抬升/回位,
启动时会打一行 warn 说明 (要人工操作就把对应确认开关打开)。
为什么不能直接用 04/05: 它们在 finally 里无条件 disable(), 电一断夹爪就松,
抬升时手里其实是空的 —— 详见 scripts/gripper_grip_hold.py 与 README §5.13。

抓取姿态由 orientation_source 决定: current_tcp (默认) 保持任务开始时 TCP
的当前姿态只改位置, fixed 则使用配置里标定好的四元数。
"""

import csv
import importlib.util
import json
import math
import os
import select
import subprocess
import sys
import threading
import time

import numpy as np
import rclpy
import yaml
from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from bottle_grasp_interfaces.msg import GraspCandidateArray, SelectedCandidate
from geometry_msgs.msg import PoseStamped
from h3001_msgs.msg import H3001ArmStatus
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from std_srvs.srv import Trigger

# 六轴名字, **与 h3001_bridge 的 _JOINT_NAMES 逐字一致**。桥的 `_joints_to_deg` 是
# 按名字查表取关节角的 (缺一个就 raise), 所以下发 JointState 的 name 必须对得上 ——
# 按下标发会把六个轴整体错位, 那是能撞坏机械臂的错误 (桥那边同一句注释)。
_JOINT_NAMES = [f"joint{i}" for i in range(1, 7)]

# 固件 ik_status 码表 (0 除外), 抄自 SDK_linux/h3001_web/protocol_ui.js 的 ikNames。
# 只用来把拒收原因写进日志 —— 判据是「非 0 即拒收」, 不认识的值照样拒收, 不许因为
# 码表里没有就当它不是问题 (固件可能比这份表新)。
_IK_STATUS_NAMES = {
    1: " (参数无效)",
    2: " (超出工作空间)",
    3: " (超过关节限位)",
    4: " (轨迹缓冲区不足)",
    5: " (关节单步过大)",
    6: " (奇异姿态)",
    7: " (逆解校验失败)",
}


def _ik_status_text(verdict):
    """`ik_status=2 (超出工作空间)` —— 认不出来的码就只写数字。"""
    return f"ik_status={verdict}{_IK_STATUS_NAMES.get(verdict, '')}"


class _GripperJob:
    """一次 `scripts/gripper_grip_hold.py` 子进程 (一条一次性夹爪命令)。

    节点对它的全部要求就是**非阻塞**: 它被 20Hz 的状态机回调轮询, 任何一次阻塞读
    都会把 /feedback 订阅和 execute_grasp 服务一起冻住 (执行器是单线程的)。所以
    stdout 走一条自己建的 `os.pipe` + 非阻塞读, 不碰 subprocess 的缓冲包装 ——
    select 说「可读」只代表有字节, 半行会让 readline 阻塞。

    脚本的 stdout **只在一种情况下参与判断**: 闭合之后那行 `[holding]` —— 它表示
    「夹持力已经建立」, 也是保持进程不退出时唯一的「这一步做完了」信号 (见
    _gripper_outcome)。其余的输出仍然只是排障日志: 读它是为了把 `[abort]` /
    `[warn]` 那些原因行转进 ROS 日志, 免得真机出事时还得回头手动复现一次。

    **stdin 是命令通道**(2026-09-28 加回): 保持中的进程靠 stdin 上的一行 `OPEN`
    才会松手。只有 `write_line` 一个入口, 它绝不抛异常 —— 它在计时器回调里被调。

    它同时是测试的接缝: 单元测试注入一个只有 poll / read_lines / write_line /
    kill / mode 的假对象, 完全不 spawn 真进程、不碰串口。

    mode 只是记着这一趟跑的是哪条命令 (open / close), 给日志和测试看; 节点自己不
    拿它做判断 —— 判断只看 poll() 的退出码 (以及上面那行标记)。
    """

    def __init__(self, process, mode, stdout_fd=None, stdin_fd=None):
        self.process = process
        self.mode = mode
        self.exit_code = None
        self._stdout_fd = stdout_fd
        self._stdin_fd = stdin_fd
        self._pending = b""     # 还没凑成一整行的字节
        self._eof = stdout_fd is None

    def poll(self):
        self.exit_code = self.process.poll()
        return self.exit_code

    def write_line(self, text):
        """往子进程 stdin 写一行, 返回是否写成功。**绝不抛异常**。

        失败的两种情形都要当「这次没送达」处理, 由调用方回落到「另外起一个进程」:
        - 进程已经退出 → EPIPE (BrokenPipeError);
        - 管道满了 (对端没在读) → BlockingIOError (写端是非阻塞的)。
        它在 20Hz 的计时器回调里被调用, 抛出去会直接拖垮节点。
        """
        if self._stdin_fd is None:
            return False
        try:
            os.write(self._stdin_fd, (text + "\n").encode("utf-8"))
        except (BrokenPipeError, BlockingIOError, InterruptedError, OSError, ValueError):
            return False
        return True

    def read_lines(self):
        """取走当前**已经读到**的整行 (str 列表); 一行都没有就返回 []。

        半行留着等下一拍 —— 调用方是状态机回调, 不能等。进程退出时最后那行没有
        换行也照样交出去, 否则脚本被信号打断时的 `[abort] ...` 会永远卡在缓冲里。
        """
        if self._stdout_fd is not None:
            while True:
                try:
                    chunk = os.read(self._stdout_fd, 4096)
                except BlockingIOError:
                    break
                except OSError:
                    break
                if not chunk:
                    self._eof = True
                    break
                self._pending += chunk
        lines = []
        while b"\n" in self._pending:
            raw, self._pending = self._pending.split(b"\n", 1)
            lines.append(raw.decode("utf-8", "replace").strip())
        if self._eof and self._pending:
            lines.append(self._pending.decode("utf-8", "replace").strip())
            self._pending = b""
        return lines

    def kill(self):
        """SIGTERM 掉进程 (不关管道 —— 让它把收尾的 print 走完)。"""
        if self.process.poll() is None:
            try:
                self.process.terminate()
            except OSError:
                pass

    def close(self):
        """放掉 stdout/stdin 两条管道。只在这个进程已经不会再用它们的时候调。"""
        for name in ("_stdout_fd", "_stdin_fd"):
            fd = getattr(self, name)
            if fd is None:
                continue
            try:
                os.close(fd)
            except OSError:
                pass
            setattr(self, name, None)


def spawn_gripper_process(argv, mode):
    """按 _GripperJob 的要求起一个进程; 起不来就抛 OSError。

    三个不显眼但必须的细节:
      - stdout 走自己建的 os.pipe + 非阻塞读, 不用 stdout=PIPE —— 见 _GripperJob。
      - **不给 `text=True`**: 读侧是 os.read + 按 b"\\n" 切字节, 两边都按字节来。
      - stdin 同理另建一条 pipe, 写端**非阻塞**: 保持进程不读的时候 write 会卡住,
        而调用点在计时器回调里。

    stdin 是「闭合保持中的进程被写成 OPEN 才松手」那套协议 (2026-09-28 加回, 理由
    见 _GripperJob 与 README §5.13)。脚本保持中时会盯着它; 一次性命令 (open,
    或者 hold-torque=0 的 close) 根本不读它, 写了也无害。

    放在模块级而不是节点方法里, 是为了让单元测试能用**同一条**管道装配去起一个替身
    脚本: 照抄一遍装配的话, 抄漏的地方 (漏 set_blocking / 漏关写端) 测试永远看不见。
    """
    read_fd, write_fd = os.pipe()
    os.set_blocking(read_fd, False)
    cmd_read_fd, cmd_write_fd = os.pipe()
    os.set_blocking(cmd_write_fd, False)
    try:
        process = subprocess.Popen(
            argv,
            stdin=cmd_read_fd,
            stdout=write_fd,
            stderr=subprocess.STDOUT,   # 脚本的 [abort] 原因在 stderr 上, 不能丢
        )
    except OSError:
        for fd in (read_fd, write_fd, cmd_read_fd, cmd_write_fd):
            os.close(fd)
        raise
    # 父进程不写 stdout 那一端。**必须关**: 留着的话读端永远等不到 EOF, 进程退出后
    # 最后那行 (没有换行) 也读不出来。stdin 的读端同理要关 (它在子进程手里)。
    os.close(write_fd)
    os.close(cmd_read_fd)
    return _GripperJob(process, mode, stdout_fd=read_fd, stdin_fd=cmd_write_fd)


class BottleGraspExecutor(Node):
    """感知候选 → base_link 目标 → H3001 move_p / move_l 的抓取执行状态机。"""

    _STATE_IDLE = "IDLE"
    _STATE_WAIT_TARGET = "WAIT_TARGET"
    _STATE_PRE_GRASP = "PRE_GRASP"
    _STATE_TRACK_WAIT = "TRACK_WAIT"   # 悬停跟随移动目标, 等人工确认下降
    _STATE_DESCEND = "DESCEND"
    _STATE_GRASP_WAIT = "GRASP_WAIT"
    _STATE_LIFT = "LIFT"
    _STATE_PLACE_MOVE = "PLACE_MOVE"        # 抬升后直线走到放置位姿本身 (判到位)
    _STATE_PLACE_WAIT = "PLACE_WAIT"        # 到达放置点后放下物体 (默认不等确认)
    _STATE_HOME = "HOME"        # 任务结束 (成功或失败) 后回待机位, 一段真实运动
    # 夹爪动作: 一个**非运动**状态, 由三个地方进入, 靠 _gripper_stage
    # 区分这一趟要做什么 (见 _gripper_plan):
    #   任务开始时 / GRASP_WAIT 之后 / PLACE_WAIT 之后
    # 它不产生任何 move_p —— 机械臂在这些时刻是不动的, 动的是夹爪。
    _STATE_GRIPPER = "GRIPPER"

    # 需要走**笛卡尔直线** (MOVE_L) 而不只是「到得了就行」(MOVE_P) 的运动状态。
    # 唯一的下发出口 _send_move_p 每次都问 _motion_is_linear(), 所以加减阶段只改这里。
    #
    # 为什么只剩下降: P 是固件自己的点到点规划, 中间怎么走由它定 —— 下降时若中途兜一个
    # 弧, 夹爪就不是垂直落到瓶子上, 这是 2026-09-24 把它改成 L 的理由。其余的段
    # (PRE_GRASP / LIFT / HOME / PLACE_MOVE / TRACK_WAIT) 只要求终点, 走什么形状无所谓。
    #
    # 抬升 (LIFT) 于 2026-09-28 从 L **退回 P**: 它被固件的 L 规划器反复拒收
    # (`ik_status=5 关节单步过大` —— 抬升那一段的起点是抓取点, 构型伸得最开, 逐点插补
    # 时关节单步超限), 30s 超时收场; 同一个位置 3 秒前刚用 move_p 悬停到过。
    # 退回 P 的代价: 固件可能让夹着的瓶子斜着扫过去蹭到台面 —— 现场若见到这个现象,
    # 别再把 LIFT 塞回这张名单 (它换来的是「根本抬不起来」), 改用「先一条短 move_p
    # 起步、再走 L」那种两步写法。判据与实测见 README §5.11。
    _LINEAR_MOTION_STATES = (_STATE_DESCEND,)

    # **优先走关节空间 (MOVE_J) 的运动状态** (2026-09-28)。
    #
    # 与上面那张表**语义不同、不互斥**: _LINEAR_MOTION_STATES 是「J 走不成、回落到
    # 笛卡尔时, 哪一段用 MOVE_L」; 这一张是「哪一段优先走 J」。DESCEND 同时在两张表里,
    # 这是对的 —— 降也走 J, 回退时仍走 L (「夹爪垂直落到瓶子上」那个要求在回退路径上
    # 依然成立)。
    #
    # 为什么整体改走 J: 固件那条路是「收笛卡尔位姿、逐点解逆解」, 现场那组抓取姿态
    # 配上保持姿态的垂直升降会让 J4 一路爬到机械上限然后解断裂 —— 09-28 抬升被拒收
    # `ik_status=5` 就是这么来的。而**关节插值不逐点解逆解**, 走的是同一条连续关节
    # 路径 (实测 J4 单调 +66.9→-16.9、末端只偏离直线 16.4mm), 直接绕开了那一整类故障。
    #
    # 代价: 这条路上固件原先替我们做的**三道判决**(可达性 / 关节限位 / 关节单步)
    # 现在全归我们 —— 所以 _solve_joint_target 里那三道守门缺一不可, 见那里的注释。
    _IK_MOTION_STATES = (
        _STATE_PRE_GRASP,
        _STATE_TRACK_WAIT,
        _STATE_DESCEND,
        _STATE_LIFT,
        _STATE_PLACE_MOVE,
        _STATE_HOME,
    )

    # GRIPPER 状态的三个进入点 (唯一写 _gripper_stage 的地方是 _enter_gripper)。
    # 每个进入点都是**起一个新进程、跑一条一次性命令、等它退出**, 三者只差
    # 「跑哪条命令」和「跑完去哪」。
    _GRIPPER_STAGE_OPEN = "open_start"    # 每轮开始前先张开, 保证从确定状态开始
    _GRIPPER_STAGE_CLOSE = "close"        # GRASP_WAIT 之后: 闭合
    _GRIPPER_STAGE_RELEASE = "release"    # PLACE_WAIT 之后: 张开
    # 三个阶段的「脚本模式 / 日志说明」。键就是上面的 stage, 在一起才看不出歧义。
    # OPEN 与 RELEASE 跑的是同一条命令 (都是张开), 分开只是因为跑完的去向不同。
    _GRIPPER_PLANS = {
        _GRIPPER_STAGE_OPEN: ("open", "张开"),
        _GRIPPER_STAGE_CLOSE: ("close", "闭合"),
        _GRIPPER_STAGE_RELEASE: ("open", "张开"),
    }
    # 命令跑完之后日志里那句话。与 _GRIPPER_PLANS 一一对应, 分开写是因为
    # 「跑哪条命令」和「跑完怎么报」不是一回事 (OPEN 和 RELEASE 命令相同、说法相同,
    # 但 CLOSE 用「已闭合」才不会被误读成张开)。
    _GRIPPER_DONE_LABELS = {
        _GRIPPER_STAGE_OPEN: "已张开",
        _GRIPPER_STAGE_CLOSE: "已闭合",
        _GRIPPER_STAGE_RELEASE: "已张开",
    }

    def __init__(self):
        super().__init__("bottle_grasp_executor")
        self._declare_parameters()

        # ── 手眼矩阵 ──
        self._T_base_camera = self._load_hand_eye_matrix()

        # ── 夹爪偏移 (临时方案, 详见 _reported_to_gripper) ──
        self._gripper_offset = np.asarray(
            self.get_parameter("gripper_offset_tool_xyz").value, dtype=np.float64
        )
        if self._gripper_offset.shape != (3,):
            raise RuntimeError(
                f"gripper_offset_tool_xyz 需要 3 个数, 实际 {self._gripper_offset.shape}"
            )

        # ── 待机位 (回位目标) ──
        # 参数是固件/GUI 格式的 6 个数, 在这里一次性换算成内部的夹爪系。
        # 换算依赖 self._gripper_offset, 所以必须放在上面那段之后。
        self._return_pending = False        # 本次任务还能不能回位 (防重入, 见 _should_return_to_standby)
        self._last_command_position = None  # 最后下发的夹爪路点 (dry_run 下当回位起点)
        # 最后那条指令走的是不是直线 (L)。_retry_rejected_command 靠它把被拒的指令
        # **按原模式**重发 —— 不能重发时去查当前状态: 状态可能已经变了。
        self._last_command_linear = False
        self._standby_ready = False         # 加载期校验通过才置 True
        self._standby_position = None       # 夹爪系 (base_link, m)
        self._standby_orientation = None    # (x, y, z, w)
        self._home_blind = False            # 本次回位是「盲走」(机械臂不可控, 不判到位)
        self._blind_started = None          # 盲走下发时刻, 用于 blind_home_dwell
        self._load_standby_pose()

        # ── 放置点 ──
        # 与待机位一样只在这里读一次 (换算/校验都是一次性的, 改它必须重启节点)。
        self._place_ready = False           # 加载期校验通过才置 True
        self._place_position = None         # 夹爪系 (base_link, m), 直接就是路点
        self._place_orientation = None      # (x, y, z, w), 与位置一样来自参数
        self._load_place_pose()

        # ── 夹爪 ──
        # 与放置点同样只在启动时解析一次 (脚本路径与串口都不会中途变)。
        self._gripper_ready = False         # 加载期校验通过才置 True
        self._gripper_script = None         # scripts/gripper_grip_hold.py 的绝对路径
        self._gripper_job = None            # 当前 _GripperJob; 关闭后可能一直活着 (保持中)
        self._gripper_stage = None          # 这一趟做什么, 见 _GRIPPER_PLANS
        self._gripper_started_at = None     # 这一趟的起点 (None = 还没起进程), gripper_timeout 用
        # 手里那个进程是不是**正在保持** (闭合后没退出, 在持续发力)。它决定两件事:
        # 张开时是「给它写 OPEN」还是「另外起一个进程」; 以及搬运途中它死了要不要判失败。
        # 只有 CLOSE 阶段看到 [holding] 标记才会置 True (见 _gripper_outcome)。
        self._gripper_holding = False
        # 这一趟的进程是**复用**回来的 (给保持进程写了 OPEN) 还是新起的 —— 只影响
        # 超时文案: 复用时那句「还没退出」说的其实是「张开还没做完」。
        self._gripper_adopted = False
        self._load_gripper_config()

        # ── 运行状态 ──
        self._state = self._STATE_IDLE
        self._state_enter_time = 0.0
        # 运动超时按「路点」而不是按「状态」计时 —— 按路点才是「这个路点我试了多久」。
        # 2026-09-21 起每个运动阶段都是**一个路点** (HOME 也不再拆段, 见 _enter_home),
        # 所以这条现在等于按状态计时; HOME 例外地用 _home_time_multiplier 把时间预算
        # 放大回拆段年代的总量 (见 _active_motion_timeout)。
        # (DESCEND 不在这里: 它只有一条指令且不等反馈, 见 _tick_descend。)
        self._waypoint_enter_time = 0.0
        self._goal_sent = False
        self._goal_position = None          # 当前 MoveP 目标 (夹爪系, base_link, m)
        self._tolerance_start = None        # 进入到位容差的起始时刻
        self._waypoints = []                # 当前运动阶段的路点队列 (只装位置)
        self._waypoint_index = 0            # 已发出的路点下标
        # 与 _waypoints 一一对应的姿态队列, **只有「抬升时顺带回正」会用它** —— 那条
        # 路径上每个路点的姿态都不相同 (关节直线采样正解出来的), 拿一个状态级常量姿态
        # 下发会把中间路点指到别处。None = 本阶段所有路点共用 _active_orientation()。
        self._waypoint_orientations = None
        # HOME 的到位超时倍数 (1 = 不放大)。只在回位期间 > 1, 离开 HOME 就清 — 见
        # _home_time_multiplier_for 与 _active_motion_timeout。
        self._home_time_multiplier = 1
        # 「指令发出去了但机械臂一步没动」只提示一次的哨兵 (见 _tick_motion)。
        self._no_start_warned = False
        # ── DESCEND 的状态 ──
        # 下降只有一条指令, 所以只需要记「什么时候发的」, 用来等 descend_dwell。
        self._descend_sent_time = 0.0

        # ── TRACK_WAIT 悬停跟随的状态 ──
        # 与下降一样, 这些状态都是「针对某一条已下发的指令」的, 换段/离开跟随就清零。
        self._track_command = None          # 上一条悬停跟随指令 (夹爪系, base_link, m)
        self._track_last_command_time = 0.0  # 上一条跟随指令的时刻 (周期门用)
        # 停稳检测: 锚点 (上一次**判为变化**时的 TCP 位置) / 锚点被重设的时刻。
        # 是锚点不是「上一帧」: 基准只在违规的那一拍重设, 所以累计漂移超过 epsilon
        # 就重开窗口。写成每拍都赋值等于只比相邻两帧, 慢速漂移 (每拍都小于 epsilon)
        # 会被误判成「一直没变」。逐拍调用 (见 _follow_target 开头), 不是「等门放行才更新」。
        self._track_stable_pose = None
        self._track_stable_since = None
        # 全自动下降: 物体坐标的锚点 / 锚点被重设的时刻, 判据同构。量的是**物体坐标**
        # 而不是机械臂位姿, 且只在机械臂已到位之后才起数 (见 _track_object_settled)。
        self._auto_grasp_ref = None
        self._auto_grasp_since = None
        self._auto_grasp_block = ""         # 自动下降当前卡在哪道门 (只用于提示)
        self._track_initial_position = None  # 进入跟随时目标在哪 (只用于日志/提示)
        self._track_mode_warned = False     # grasp_mode 中途变化只提示一次

        # ── 迭代末端修正 (末端下垂补偿) ──
        # 叠加在标称路点上的指令修正量: 实发指令 = 标称路点 + _cmd_offset。
        # 残差按 base_link 系累加 (下发时也是 base_link 系, 姿态不变), 详见参数注释。
        self._cmd_offset = np.zeros(3)
        self._retry_count = 0
        self._settle_pose = None            # 停稳检测: 上一帧 TCP 位置
        self._settle_since = None           # 停稳检测: 位置最后一次变化的时刻
        # 「这条指令到底执行了没有」的哨兵: 见 _try_endpoint_correction 开头。
        self._stall_reference = None        # 发指令那一刻的 TCP 位置
        self._stall_armed = False           # 看见机械臂动过 -> 停稳判定才生效
        self._retry_exhausted = False
        self._sent_orientation = None       # 本次路点实际下发的姿态 (迭代重发要用同一个)
        self._prompt_time = 0.0             # GRASP_WAIT 提示打印计时
        self._enter_pressed = False         # 终端 Enter 确认标志
        self._confirm_received = False      # topic 确认标志

        # ── 目标锁定 ──
        self._latest_candidate = None
        self._locked_track_id = None
        self._target_history = []           # [(stamp, base_xyz)]
        self._grasp_center_base = None
        self._grasp_mode = None

        # ── 抓取目标 ──
        self._pre_grasp_target = None
        self._final_grasp_target = None
        self._lift_target = None
        self._goal_orientation = None       # 当前 MoveP 目标四元数 (x,y,z,w)

        # ── 反馈缓存 ──
        # _tcp_pose 是 **夹爪** 位姿 (已从上报末端原点换算), 节点内部一律用夹爪系:
        # 路点、到位判定、残差、日志都是夹爪点, 只有收发两个边界做换算。
        self._tcp_pose = None               # (stamp_s, 夹爪位置 np3)
        self._tcp_orientation = None        # (x, y, z, w), 夹爪系与上报系同姿态
        self._arm_status = None             # (stamp_s, msg)

        # ── 上位机求逆解 (2026-09-28) ──
        # 运动指令优先走关节空间 (见 _IK_MOTION_STATES): 位姿 -> IK -> /control/move_j。
        # 三道守门在 _solve_joint_target 里; 解不出来就**回落**到 move_p / move_l。
        self._joint_deg = None              # (stamp_s, np6) 当前关节角 **话题约定**(弧度→度)
        self._ik_ready = False              # 加载期校验通过才置 True (pinocchio/URDF/CSV 齐)
        self._ik_module = None              # vendored 的 ik_h3001 模块对象
        self._ik_model = None               # (model, data, fid)
        self._ik_cad_lo = None              # CAD 真机械限位 (rad), 喂给求解器当 bounds
        self._ik_cad_hi = None
        self._ik_names = None               # 6 个关节名 (日志与自检用)
        # LIFT 的**种子**(悬停那一段的解, **URDF 约定**), 不是目标: LIFT 从抓取构型
        # 出发自己解会卡在关节限位上 (实测残差 0.16mm、余量 0°), 而用悬停解当起点就
        # 解得出 —— 因为抬升就是「回到下降之前那个点」。
        self._lift_seed_joint_deg = None
        self._last_solved_urdf_deg = None   # 最近一次成功求解的 URDF 约定解 (给上面当种子)
        self._last_command_joint = None     # 最近一条指令的关节目标 (话题约定, 度); None = 笛卡尔
        self._ik_watch_joint = None         # J 指令「没动」判据的基准 (话题约定, 度)
        self._ik_selfcheck_counter = 0      # 运行期自检的节拍 (见 _ik_selfcheck)
        # **必须在上面这些字段声明之后调** —— 它会把 _ik_ready 置 True, 而声明里写的是
        # False。放在声明之前调就会被这一行覆盖掉 (2026-09-28 踩过: 启动日志说「开」,
        # 实际一路在回落 move_p)。
        self._load_ik_config()

        # ── 固件 IK 判决 (2026-09-21) ──
        # 每次 _send_move_p 都重新「立案」, 由 _ik_rejection_problem 在 tick 里结案。
        # 判「这一条指令被固件拒收」要**两条独立证据同时成立**, 缺一不可:
        #   1) ik_status 非 0 —— 固件说它解不出逆解;
        #   2) 从指令发出到现在, 机械臂**一步没动** (_ik_watch_pose 对得上)。
        # 只有 1) 不够: 0x5A9 那帧是 ~50Hz 的**周期性心跳**, 带的是「最近一次判决」
        # 这个**粘住不散**的值 (2026-09-21 实测: 放置点被拒收两分钟后它还是 2)。
        # 也就是说 ik_status=2 只说明「固件最近拒收过什么」, 不说明是这一条; 「没动」
        # 才是本条指令特有的证据。反过来只有 2) 也不够 —— 被挡住推不动的一步没动
        # 是另一条通道 (stall / motion_timeout) 的事, 两条不许互相抢。
        # _ik_watch_since 是指令发出时刻 (节点时钟秒), 用来挑「指令之后才发布」的
        # 状态快照 (同一条理由: 不拿旧判决判新指令)。
        self._ik_watch_since = None         # 时钟秒; None = 没有待判的指令
        self._ik_watch_done = True          # 这一条指令的判决已经结案
        self._ik_watch_pose = None          # 发指令那一刻的夹爪位置 (判「没动」的基准)
        self._ik_retry_used = False         # 这一条指令的「原样重发」额度用掉了没有

        # ── 循环模式 (2026-09-21) ──
        # 「本次任务跑完后要不要自动再触发一次」的唯一来源。~/execute_grasp 改成了
        # **开关**: 调一次 = 启动任务 + 置 True, 再调一次 = 置 False。
        # 置 False 的三条路: ①人工关闭 (再调一次服务); ②**真**失败 (_fail 且
        # keep_loop=False: 固件拒收 / 反馈故障 / 夹爪失败 / 回位失败……);
        # ③回位判不到位、放弃回位那条异常出口 (_tick_blind_home)。
        # 「视野里没有目标」**不算**真失败 (2026-09-21 用户要求): 跟丢目标、
        # 或回来之后下一个瓶子还没摆上来, 都只是回待机位接着等新坐标 —— 那些路径
        # 走 _fail(..., keep_loop=True), 循环保持开启。真失败才收工 (2026-09-21
        # 那几次固件 IK 拒收是同一类系统性故障, 继续循环只会一遍遍重跑整个流程再失败)。
        self._loop_armed = False
        # 循环模式下「等新目标」的判据起点 (单调钟)。None = 没在等 (正在跑 / 没武装)。
        # 回来之后只有**观察时刻晚于它**的候选才算新目标 —— 见 _tick_loop_wait。
        self._loop_guard_at = None
        self._loop_wait_log_at = 0.0        # 等目标时每 10s 提醒一次的节流

        # ── ROS 接口 ──
        self._candidates_file_mtime = None
        self._logged_offset_base = None     # 已提示过的 demo --offset
        if self.get_parameter("candidates_source").value == "topic":
            self._candidates_sub = self.create_subscription(
                SelectedCandidate,
                self.get_parameter("candidates_topic").value,
                self._on_selected_candidate,
                10,
            )
        else:
            self._candidates_sub = None
            self.get_logger().info(
                "候选来源: JSON 文件 "
                f"{self.get_parameter('candidates_json_path').value} "
                "(demo 需加 --json-out 参数)"
            )
        self._tcp_sub = self.create_subscription(
            PoseStamped,
            self.get_parameter("tcp_pose_topic").value,
            self._on_tcp_pose,
            qos_profile_sensor_data,
        )
        self._status_sub = self.create_subscription(
            H3001ArmStatus,
            self.get_parameter("arm_status_topic").value,
            self._on_arm_status,
            qos_profile_sensor_data,
        )
        self._confirm_sub = self.create_subscription(
            Bool,
            self.get_parameter("grasp_confirm_topic").value,
            self._on_grasp_confirm,
            1,
        )
        # 当前关节角 (2026-09-28): 求逆解的**种子**。走 J 的每一段都要拿它当起点 ——
        # 「从当前构型出发解得出解」正是我们要的语义 (解不出就回落 move_p)。
        # **QoS 必须用 sensor 那一套**: 桥的反馈发布器是 BEST_EFFORT, 默认的 RELIABLE
        # 订阅者跟它不兼容, 一条都收不到 (症状是「一直没有种子」, 很容易误判成桥没在跑)。
        self._joint_sub = self.create_subscription(
            JointState,
            self.get_parameter("joint_states_topic").value,
            self._on_joint_states,
            qos_profile_sensor_data,
        )
        # 三个运动出口。前两条收 PoseStamped (base_link 米 + 四元数), 按 _motion_is_linear()
        # 二选一; 第三条收 JointState (**弧度**, 按名字对), 见 _send_move_p。
        self._move_p_pub = self.create_publisher(
            PoseStamped, self.get_parameter("move_p_topic").value, 1
        )
        self._move_l_pub = self.create_publisher(
            PoseStamped, self.get_parameter("move_l_topic").value, 1
        )
        self._move_j_pub = self.create_publisher(
            JointState, self.get_parameter("move_j_topic").value, 1
        )
        self._execute_srv = self.create_service(
            Trigger, "~/execute_grasp", self._on_execute
        )

        # ── 终端 Enter 读取线程 ──
        self._shutdown_event = threading.Event()
        threading.Thread(target=self._stdin_reader, daemon=True).start()

        # ── 状态机主循环 ──
        self.create_timer(1.0 / 20.0, self._tick)

        self.get_logger().info(
            f"bottle_grasp_executor 就绪 (dry_run={self.get_parameter('dry_run').value}), "
            f"调用服务 /bottle_grasp_executor/execute_grasp 触发抓取"
        )
        # 循环模式自报, 与下面几行「下降 / 悬停 / 放置」同一套约定: 启动时把这条路径
        # 说清楚, 跑之前先看一眼。place_enabled 那道硬闸在这里也报一次 —— 现场 yaml
        # 若把它关了, 开启循环会被拒, 而拒的原因必须一眼能看见。
        self.get_logger().info(
            "触发: ~/execute_grasp 调一次 = 启动抓取并**开启循环** "
            "(回到待机位后自动抓下一个), 再调一次 = 关闭循环 (不打断正在跑的那一次); "
            "循环的停止条件 = 一次任务**真失败** (固件拒收 / 无反馈 / 夹爪失败 / "
            "回位不到位) 或人工再调一次服务 —— 跟丢目标、视野里一直没瓶子都只是回"
            "待机位接着等, 不会自动收工; "
            "循环要求 place_enabled=true (当前 "
            f"{self.get_parameter('place_enabled').value})"
        )
        # 后两个确认点的开关, 下面几行汇总都要用
        grasp_confirm = self.get_parameter("grasp_confirm_required").value
        place_confirm = self.get_parameter("place_confirm_required").value
        auto_hold = float(self.get_parameter("auto_grasp_stable_time").value)
        # track_enabled=false 时整段 TRACK_WAIT 都不存在, 自动下降自然也谈不上
        auto_on = self.get_parameter("track_enabled").value and auto_hold > 0.0
        self.get_logger().info(
            "下降: 一条指令走完 (直接下降到抓取点, "
            f"Z 下降 "
            f"{(self.get_parameter('pre_grasp_offset_z').value - self.get_parameter('grasp_offset_z').value) * 100.0:.1f}cm"
            + (
                # 倒瓶 (top) 单独设了悬停高度就把两个都报出来 —— 否则现场看着
                # 「10cm」而实际 top 走的是另一套, 又得回头查
                f" (top 模式 "
                f"{(self.get_parameter('top_pre_grasp_offset_z').value - self.get_parameter('grasp_offset_z').value) * 100.0:.1f}cm)"
                if float(self.get_parameter("top_pre_grasp_offset_z").value) > 0.0
                else ""
            )
            + ", "
            f"发出后等 {self.get_parameter('descend_dwell').value:.1f}s 即"
            + ("进夹爪确认, " if grasp_confirm else "自动闭合夹爪 (无需确认), ")
            + "**不判是否到位、不迭代末端修正**); "
            + (
                "落点由 GRASP_WAIT 的人工确认兜底"
                if grasp_confirm
                else (
                    "落点由 TRACK_WAIT 的自动下降兜底 (全程没有人看过就往下抓)"
                    if auto_on
                    else (
                        "落点由 TRACK_WAIT 那一次人工确认兜底"
                        if self.get_parameter("track_enabled").value
                        else "落点无人确认 (track_enabled=false 没有 TRACK_WAIT, "
                        "且 grasp_confirm_required=false)"
                    )
                )
            )
        )
        if self.get_parameter("track_enabled").value:
            track_how = (
                f"机械臂停稳后物体位置连续 {auto_hold:.1f}s 不变即自动下降, "
                "Enter 可提前下降"
                if auto_on
                else "Enter 确认下降"
            )
            self.get_logger().info(
                "悬停跟随: 开 (悬停后跟随移动目标, "
                f"{track_how}) "
                f"跟随周期 "
                f"{self.get_parameter('track_min_command_interval').value:.2f}s "
                f"(每周期读一次最新坐标, 超过死区 "
                f"{self.get_parameter('track_deadband').value * 1000.0:.0f}mm 才发), "
                f"停稳 {self.get_parameter('track_settle_time').value:.2f}s, "
                f"落后超过 {self.get_parameter('track_max_lag').value * 1000.0:.0f}mm "
                f"时不等停稳直接重发; 候选超过 "
                f"{self.get_parameter('max_result_age').value:.1f}s 没更新则判失败回待机位"
                + (
                    # 倒瓶 (top) 是个例外, 而且是个**必须自报**的例外: 那一模式的现场
                    # 病根就是「一到瓶子正上方就看不见」, 若这里仍说「跟随移动目标」,
                    # 日志会和实际行为对不上。
                    "; **倒瓶 (top) 例外: 不跟随, 悬停到位直接下降** (top_skip_track=true)"
                    if self.get_parameter("top_skip_track").value
                    else ""
                )
            )
        else:
            self.get_logger().info(
                "悬停跟随: 关 (悬停到位后直接下降, "
                + ("需人工确认的只有夹爪闭合那一次" if grasp_confirm
                   else "且 grasp_confirm_required=false, 下降后也不等人)")
            )
        if self._place_ready:
            self.get_logger().info(
                f"放置: 开 (抬升后**一条 move_p 直达**放置位姿 -> 停 "
                f"{self.get_parameter('place_descend_dwell').value:.1f}s -> "
                + ("Enter 确认已放置" if place_confirm else "自动张开夹爪 (无需确认)")
                + " -> 回待机位) "
                f"放置点(夹爪)={np.round(self._place_position, 4)} m, "
                "位置与姿态都用参数里的那一组 (不沿用抓取姿态, 也没有中间点, "
                "不按距离拆段 —— 路径由固件规划)"
            )
        else:
            reason = (
                "place_enabled=false"
                if not self.get_parameter("place_enabled").value
                else "place_pose_reported_mm_deg 无效 (见上面的 error)"
            )
            self.get_logger().info(
                f"放置: 关 ({reason}; 抬升后直接回待机位)"
            )
        if self._gripper_ready:
            # 「在哪张开」取决于放置那一段开不开: _GRIPPER_STAGE_RELEASE 唯一的入口是
            # PLACE_WAIT, 所以 place_enabled=false 时**没有张开这一步** —— 夹爪一直闭着,
            # 物体被带回待机位。这段日志原来一律写「PLACE_WAIT 自动张开」, 在放置关掉之后
            # 就成了假话 (2026-09-18 改回 false 时发现的)。
            if self._place_ready:
                release = (
                    "PLACE_WAIT 的 Enter 改成「现在放」"
                    if place_confirm
                    else "PLACE_WAIT 自动张开 (place_confirm_required=false)"
                ) + ", 张开后自动回位"
            else:
                release = (
                    "抬升后**一直闭着**不发张开指令 (place_enabled=false, 没有 "
                    "PLACE_WAIT 那一步), 物体被带回待机位, 要放手只能人工打开"
                )
            if float(self.get_parameter("gripper_hold_torque").value) > 0.0:
                held = (
                    f"闭合后**持续保持 "
                    f"{float(self.get_parameter('gripper_hold_torque').value):g} N·m** "
                    f"(MIT 发帧, 最长 "
                    f"{float(self.get_parameter('gripper_hold_timeout').value):.0f}s; "
                    "搬运途中保持进程一死即判失败)"
                )
            else:
                held = (
                    "闭合命令跑完就失能退出 (**不保持夹持力** —— 抬升/搬运期间靠机构"
                    "自锁; 要一直有力就把 gripper_hold_torque 开起来)"
                )
            self.get_logger().info(
                "夹爪: 开 (每轮开始先张开 -> "
                + (
                    "GRASP_WAIT 的 Enter 改成「现在夹」"
                    if grasp_confirm
                    else "GRASP_WAIT 自动闭合 (grasp_confirm_required=false)"
                )
                + ", "
                + held
                + " -> "
                + release
                + "); "
                f"端口={self.get_parameter('gripper_port').value}, "
                f"脚本超时 {self.get_parameter('gripper_script_timeout').value:.0f}s / "
                f"节点超时 {self.get_parameter('gripper_timeout').value:.0f}s, "
                "不判夹到没有 (只看命令跑没跑完)"
            )
        else:
            reason = (
                "gripper_enabled=false"
                if not self.get_parameter("gripper_enabled").value
                else "夹爪端口/脚本不可用 (见上面的 error)"
            )
            self.get_logger().info(
                f"夹爪: 关 ({reason}; 夹爪由人工在另一个终端操作)"
            )
            if not (grasp_confirm and place_confirm):
                # 夹爪不由节点接管时, 「确认」原本是「我已经手动夹好/放好了」。关掉确认
                # 之后节点既不动夹爪、也不等人, 到点直接抬升 / 直接回位 —— 必须说清楚,
                # 否则会被读成「节点会自己夹」。
                self.get_logger().warn(
                    "夹爪不由节点接管, 且确认已关掉 "
                    f"(grasp_confirm_required={grasp_confirm}, "
                    f"place_confirm_required={place_confirm}): "
                    "节点到点会直接抬升/回位, 不会自己夹、也不会等人手动夹好 —— "
                    "要人工操作夹爪就把对应的确认开关打开"
                )

    # ── 参数 ──────────────────────────────────────────────

    def _declare_parameters(self):
        defaults = {
            "dry_run": True,
            # 输入: candidates_source = topic (ROS 感知节点) 或 file (独立 demo --json-out)
            "candidates_source": "file",
            "candidates_json_path": "/tmp/bottle_grasp_candidates.json",
            # 输入
            "candidates_topic": "/bottle_grasp_perception/selected_candidate",
            "tcp_pose_topic": "/feedback/tcp_pose",
            "arm_status_topic": "/feedback/arm_status",
            "move_p_topic": "/control/move_p",
            # 只有下降走这条 (MOVE_L, 笛卡尔直线; 抬升 2026-09-28 退回 move_p)。
            # 桥上是一条独立话题, 与 move_p 收同一种 PoseStamped、同一套单位,
            # 只有固件模式不同 —— 见桥的 _on_move_l。
            "move_l_topic": "/control/move_l",
            # 关节角指令 (**弧度**, JointState, 按名字对)。运动指令优先走这条, 见
            # _IK_MOTION_STATES 与 _solve_joint_target。
            "move_j_topic": "/control/move_j",
            # 当前关节角 (弧度, JointState) —— IK 的种子。
            "joint_states_topic": "/feedback/joint_states",
            "grasp_confirm_topic": "/bottle_grasp_executor/grasp_confirm",
            "robot_base_frame": "base_link",
            # 手眼标定
            "hand_eye_yaml_path": os.path.expanduser(
                "~/hand_eye_calibrate/H3003/hand_eye_extrinsics.yaml"
            ),
            # 候选筛选
            "min_confidence": 0.30,
            "min_quality": 0.05,
            # 候选过期阈值 (s): 收到的候选比这还旧就当「没有候选」。
            # 两个用处: WAIT_TARGET 里判断候选还算不算新鲜; TRACK_WAIT 里判断
            # 感知是不是停了 (停了就判失败, 见 _follow_target)。
            # **必须大于 track_min_command_interval**, 否则跟随还没到下一次读数
            # 就先判失败 (由 test_track_wait 的 ConfigConsistencyTest 盯着)。
            # 具体秒数在真机上边跑边定 (2026-09-17 现场调成 5.0, 2026-09-21 现场是
            # 6.0 —— 配合 1.0s 的周期, 容得下连续漏 5 拍); 这里跟着 **yaml 走**
            # (2026-09-21 对齐成 6.0), 不让 ros2 run 与 ros2 launch 起出两种行为。
            # ConfigConsistencyTest 盯着这一条: 改 yaml 就得同步改这里。
            "max_result_age": 6.0,
            # 目标稳定
            "stable_duration": 1.0,
            "min_stable_samples": 6,
            "target_stability_tolerance": 0.010,
            # 抓取几何 (base_link, m)
            # 悬停点高度, 同时也是单步下降的下降距离 (再减 grasp_offset_z):
            # 0.12 = 悬停在抓取中心上方 12cm, 下降 12cm 落到感知给出的抓取中心。
            # 2026-09-21 起**代码默认值与 yaml 现场值一致** (都是 12cm; 之前这里是
            # 9cm, 现场是 12cm —— 那种不一致意味着 ros2 run 不带 params-file 跑出来
            # 的下降行程比真机短 3cm)。下降没有到位判据, 行程变长必须同步调大
            # descend_dwell (现场: 12cm / 2.0s), 否则会在半空闭合夹爪。
            "pre_grasp_offset_z": 0.12,
            # **倒瓶 (top 模式) 单独一个悬停高度** (2026-09-28)。<= 0 = 不单独设,
            # 沿用上面那个 —— 参数声明要具体类型, 所以用负数当「没设」的哨兵。
            #
            # 为什么 top 模式需要它: 那一模式是**工具朝下**抓, 末端原点(法兰)在夹爪
            # **上方一整个工具偏移** (gripper_offset_tool_xyz, 现场 0.18m)。所以悬停
            # 抬高 1cm, 法兰就高 1cm —— 同一组数 side 下够得着、top 下可能直接出界
            # (2026-09-28 实测: 悬停 0.20 时法兰报到 532mm, 超出可达区, 固件与上位机
            # 都拒收)。它同时也是那一段的**下降距离** (见上面注释), 所以 top 模式改它
            # 等于同时改 top 的下降行程 —— descend_dwell 要跟着核。
            "top_pre_grasp_offset_z": -1.0,
            # **倒瓶 (top 模式) 抓完不抬升, 直接去放置** (2026-09-28, 用户要求)。
            #
            # 为什么只给 top: 那一模式是**工具朝下**抓, 夹爪本来就停在物体正上方 ——
            # 先抬 10cm 再横move到放置点, 对「把物体放到某处」没有任何帮助, 反而多走
            # 一段、多经过一个可能解不出来的位姿; 直接进放置, 起点就是抓取点。
            # (side 模式不动: 它是从**侧面**抓的, 先把物体提离台面是必要的。)
            #
            # false (默认) = 与改动前逐字相同: 三种模式都先 LIFT 再放置。
            "top_skip_lift": False,
            # **倒瓶 (top) 不跟踪: 悬停到位直接下降抓取** (2026-09-29, 用户要求)。
            #
            # 为什么只给 top: 那一模式是工具朝下、从**正上方**抓, 机械臂一挪到瓶子
            # 正上方, 手眼相机就看不着瓶子了 —— 现场表现就是「到达瓶子上方就识别不
            # 到」: 候选不再更新, 于是 _follow_target 按 max_result_age 判
            # TRACK_TARGET_LOST 回待机位, 一轮都抓不成 (2026-09-29 现场)。
            # 而 top 的悬停高度本来就只有一个很小的量 (top_pre_grasp_offset_z, 现场
            # 1cm), 悬停点与抓取点之间几乎没有横向差别 —— 跟随在这里没有东西可跟。
            # (side 不动: 从**侧面**抓, 相机看得见瓶子, 跟随仍是「瓶子挪到哪就抓到哪」
            # 的唯一手段。)
            #
            # 打开后的行为: PRE_GRASP 悬停到位**直接进 DESCEND** —— 不进 TRACK_WAIT,
            # 因此既不等 Enter, 也不等 auto_grasp_stable_time。下降点用**锁定目标那
            # 一刻**算好的坐标 (_tick_wait_target 里过了稳定窗口那一份), 见
            # _skip_track_for_mode。代价与 track_enabled=false 相同: 这一轮的落点
            # **无人确认** (那一段注释里的取舍在这里逐条成立)。
            # false (默认) = 与改动前逐字相同: top 也进 TRACK_WAIT。
            "top_skip_track": False,
            "grasp_offset_z": 0.0,
            "lift_distance": 0.10,
            # ── 抬升时顺带回正姿态 (2026-09-30) ──
            # 倒瓶 (top) 的抓取姿态来自感知 live、工具朝下, 与放置姿态差 144°; 在桌面
            # 高度一条指令同时走长位移 + 这次转腕, IK 解不出来 -> 回落 move_p -> 固件
            # 规划成大摆 -> 电机报错收场 (2026-09-30 现场, 见 _begin_lift)。
            # 这两项把回正折进抬升: 当前姿态与 side 抓取姿态差超过阈值时, 抬升那一条
            # 指令的目标姿态改成 side 姿态 (side 模式本来就等于它, 差值≈0, 行为逐字不变)。
            "lift_align_threshold_deg": 90.0,
            # 回正按**关节直线**拆成几段下发。为什么拆: 目标构型与当前构型的单轴差有
            # 146° (J6), 一条指令发出去会被守门 ③「换构型大摆」拒掉; 拆成整段 ≤60° 的
            # 几条 move_j, 每段都过门, 而固件对 J 本来就是关节插值 —— 拆与不拆走的是同
            # 一条关节路径。
            "lift_align_segments": 3,
            # ── 放置 (2026-09-17, 2026-09-18 改成完整位姿 + 直达) ──
            # 抬手后把物体送到 place_pose_reported_mm_deg 再回待机位。两段:
            #   PLACE_MOVE 从抬升点**直线直达**放置位姿本身, 判到位 (迭代末端修正照走);
            #   PLACE_WAIT 到达后停 place_descend_dwell 秒放下物体, 然后回待机位。
            # 2026-09-18 之前是「先平移到放置点正上方(高度=抬升后的实际高度)、再垂直
            # 盲降」两段; 用户要求「直接发送 place_pose_reported_mm_deg 这个坐标让机械臂
            # 到达」, 那个中间点被去掉, PLACE_DESCEND 状态随之删除。
            # 置 false 则整段跳过, 退回 LIFT -> HOME 的旧流程 (与 return_to_standby_on_finish
            # 一样, 代码默认关、yaml 里开)。
            "place_enabled": False,
            # 放置位姿: **上报末端原点系** 的 [X,Y,Z (mm), RX,RY,RZ (度)], 就是
            # /feedback/tcp_pose 和厂商 GUI 上的那 6 个数 —— 与
            # standby_pose_reported_mm_deg 完全同一套约定, 填法也一样。
            #
            # 位置与姿态都在加载期一次性换算成夹爪系 (_load_place_pose):
            #     夹爪 = 上报 + R @ gripper_offset_tool_xyz      姿态不用换算
            # 放置段**不再沿用抓取时的姿态**: 整段都用这里的 RX/RY/RZ (2026-09-18 用户
            # 明确要求, 换成一个机械臂解得出、够得到的朝向)。
            #
            # 默认全零是**占位值**: 位置 [0,0,0] 会让上报原点落在基座原点附近, 加载期判
            # 无效并禁用放置 (_load_place_pose), 所以忘了填不会变成「往底座里扎」, 而是
            # 任务被拒绝启动。姿态全零 = 单位四元数, 但位置既然无效就轮不到它。
            # 仅加载期读取, 改它要重启节点。
            "place_pose_reported_mm_deg": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            # 到达放置点后停多久才放下物体 (提示确认, 或 place_confirm_required=false
            # 时直接张开) (s)。
            # **名字是旧流程留下的** (2026-09-18 之前这里是「盲降指令发出后等多久」),
            # 现在没有下降那一段了, 它等的是「判到位之后机械臂真的停稳」—— 作用一样:
            # 别在还在晃的时候松手。所以名字不改 (改了要动 yaml/文档/测试四五个地方),
            # 但**语义看这里**。
            # 与 descend_dwell 分开的理由也变了: 抓取下降是盲降 (只看时间), 放置这里是
            # 判到位后再停一下, 两件事不该共用一个参数。
            # 2026-09-21 起**代码默认值与 yaml 现场值一致** (都是 0.5): 之前这里是
            # 6.0 而现场是 0.5 —— 那种不一致意味着 ros2 run 起出来的节点会多等 5.5 秒
            # 才松手。PLACE_WAIT 进状态时已经判过到位 (含 settle_time 的停稳), 这 0.5
            # 秒是「别在还在晃的时候松手」的余量, 真机上 10:48 那次就是 0.5 秒放下的。
            "place_descend_dwell": 0.5,
            # PLACE_WAIT 要不要等人工按 Enter 才张开夹爪。与 grasp_confirm_required
            # 完全对称 (那处的注释里写了代价, 这里不重复):
            #   false -> 到点停满 place_descend_dwell 就自动张开, 张开成功后自动回待机位
            #            (回位本身仍由 return_to_standby_on_finish 控制);
            #   true (默认) -> 退回原行为, 由人确认物体确实放下了再按 Enter。
            # 真机上出现「放得太早/放歪」时先调大 place_descend_dwell, 而不是改 true。
            "place_confirm_required": True,
            # 「上报原点到抓取点」在 tool 坐标系下的偏移 (m)。
            # 下发前换算 origin = 抓取点 - R @ offset; 填零则 origin == 抓取点。
            # 2026-09-14 曾定为 **零** (当时的假设是「上报的末端位姿本身就落在抓取点
            # 上」); 2026-09-17 实测推翻了这个假设 —— 上报原点不在夹爪上, 而在 tool 系
            # +z 方向上一个几厘米级的距离上 (当时的量级 ~6cm; 2026-09-21 起与 yaml 一起
            # 定为 8cm, 现场那几趟就是这样抓起来的)。
            # 现值与 config/bottle_grasp_executor.yaml **一致** (2026-09-21 对齐):
            # 那边是这套参数的源头, 改这里也要同步改那边, 否则 ros2 run 不带
            # params-file 跑出来的换算和真机不一样 —— 差一个偏移量 = 每个抓取目标
            # 整体平移那么多, 而瓶子粗细可能正好容得下这个误差, 所以**抓起来了不等于
            # 这个值是对的**; 有疑问就重新标定, 别拿日志里那行「模长 N mm」当依据
            # (那是参数的回显, 不是测量)。
            # 填零是「换新机械臂后没有这个问题」时的退路。
            "gripper_offset_tool_xyz": [0.0, 0.0, 0.08],
            # ── 悬停跟随 + 下降前确认 (2026-09-16) ──
            # 到达悬停点后不立刻下降, 而是先进 TRACK_WAIT: 目标动了就跟着动,
            # 同时等人工在终端确认「现在下降」。确认后才进 DESCEND。
            # 置 false 则跳过 TRACK_WAIT, 退回 PRE_GRASP -> DESCEND 的旧流程 ——
            # 这条回退路径是「新功能在真机上有问题」时的第一选择, 所以保留。
            "track_enabled": True,
            # 死区: 目标相对**上一条已下发的悬停指令**移动超过它才重发。
            # 不拿「机械臂实际位置」当基准, 是因为这台机末端有 3~55mm 的固定重力
            # 下垂 (见下面迭代末端修正), 用实际位置做基准会变成永远追不上、一直重发。
            "track_deadband": 0.010,
            # 跟随周期 (s): 两条跟随指令的最小间隔, 「每 N 秒读一次最新坐标」里的 N。
            # 这是**离散跟随**而不是连续伺服 —— 每一跳都把悬停点更新到「目标现在在哪」,
            # 所以不会累积滞后; 跳与跳之间机械臂是静止的。
            # 与 track_deadband 一起决定最小跟随速度: 10mm / 1.0s ≈ 10mm/s。
            # 逐次读取, 真机上可以 ros2 param set 边跑边调 (调小跟得更勤, 代价是启停更频繁;
            # 2026-09-17 现场从 2.0 调到 1.0, 同步把 max_result_age 放宽到 5.0)。
            "track_min_command_interval": 1.0,
            # 上一条指令还没走完就先不发新目标 —— 否则两个目标重叠, 真机表现就是
            # 「抖动」, 与 2026-09-14 下降 wp3 那次是同一个病。
            #   track_settle_time / track_stable_epsilon: 停稳判定 (位置变化小于
            #   epsilon 并保持 settle_time 就算走完了)。
            #   track_max_lag: 例外通道。目标比机械臂跑得快时不能傻等, 落后超过它
            #   就无视「上一条走完」直接重发 —— 跟得上比不抖动更重要。
            #   注意 lag 量的是「距上一条指令还有多远」而不是「距新目标多远」:
            #   后者里含着固定下垂量, 会让这条通道永远打开, 等于取消停稳门。
            "track_settle_time": 0.30,
            "track_stable_epsilon": 0.0005,
            "track_max_lag": 0.050,
            # ── 悬停之后也全自动 (2026-09-22) ──
            # 把 TRACK_WAIT 这一次人工确认也自动化: 机械臂已经跟到物体上方、且物体
            # 位置连续保持 auto_grasp_stable_time 秒没变, 就自动下降抓取。
            #   0 -> 关闭本功能, 退回「等人按 Enter / grasp_confirm」的旧行为。
            # Enter 与 grasp_confirm 话题始终有效, 随时可以手动越权。
            # **计时的起点是「机械臂到位」而不是「物体停下」**: 物体骤停时机械臂可能
            # 还在横移, 那时若就开始数秒, 数满会立刻下降, 把长距离横移和整段下降并成
            # 一条指令, 而 descend_dwell 可能在机械臂到位前就走完 -> 半空闭合夹爪。
            "auto_grasp_stable_time": 3.0,
            # 判「物体位置没变」的容差 (m)。**不能调小**: 实测静止瓶子的 position_base
            # 标准差 [0.6, 0.7, 0.8]mm、峰峰值 [3.1, 3.1, 4.2]mm (112 帧 / 13.9s),
            # 拿 0.5mm 量级 (track_stable_epsilon) 来判会**一次都不触发** —— 滚动 3s
            # 窗口在 eps=2.0mm 时 0/73 通过, 到 5mm 才 73/73。0.010 有约 2 倍裕量,
            # 且与 WAIT_TARGET 判「目标已稳定」用的 target_stability_tolerance 同值。
            "auto_grasp_stable_epsilon": 0.010,
            # ── 下降: 一条指令走完 (2026-09-17) ──
            # DESCEND 只发**一条**指令 (2026-09-24 起走 move_l 笛卡尔直线), 从悬停点
            # 直接下降到抓取点
            # (Z 下降 pre_grasp_offset_z - grasp_offset_z, 代码默认 0.12 / yaml 现场
            # 0.20), 发出后等 descend_dwell 秒就进 GRASP_WAIT —— **不判是否到达指定
            # 位置、不迭代末端修正、不分段**。
            # ⚠ 行程 0.20 配 descend_dwell 默认 2.0 这个组合**没有实测过** (dwell 是
            # 照 0.12 定的量), 而改走 L 之后耗时曲线也不一样 —— 真机第一次跑务必看
            # 那条日志的「实际」栏在闭合前是否还在移动。
            # **这两个数是联动的**: 行程改长了 (调 pre_grasp_offset_z 或 grasp_offset_z),
            # descend_dwell 要跟着调大, 否则 dwell 还没到点机械臂就已经走完 (白等),
            # 或者反过来 —— 见下面 grasp_confirm_required 那条注释。
            #
            # 为什么: 这台机末端有固定重力下垂 (见下面迭代末端修正), 「距目标还有多远」
            # 长期不为零。2026-09-16 那套分段下降为了让每一段都能被「到位」判据确认,
            # 要等停稳 (0.35s) + 过限速门 (0.50s), 末段还掉进迭代修正, 8~9 段叠加起来
            # 又慢又抖。而**降到哪一点并不需要节点自己确认**: GRASP_WAIT 本来就要求人看
            # 一眼夹爪再按 Enter, 那才是真正的判据。于是把「走到哪」交给人和日志, 节点
            # 只负责把指令发出去、给足走完的时间。
            #
            # 代价: 日志不再能证明「确实降到位了」, 唯一证据是 _tick_descend 里那行带着
            # 实际位置的日志。另外机械臂完全不动 (被挡住 / IK 无解) 也不会再报失败 ——
            # 看一眼即可。
            # 2026-09-21 起**代码默认值与 yaml 现场值一致** (都是 2.0): 之前这里是 6.0
            # 而现场是 2.0。这一条必须与下降行程一起看 —— 12cm 的盲降给 2.0s, 现场
            # 跑通了; 谁把 pre_grasp_offset_z 调大就必须同步调大这里。
            "descend_dwell": 2.0,
            # GRASP_WAIT 要不要等人工按 Enter 才闭合夹爪。
            #   false (现场用法) -> dwell 一到就自动闭合, 整条链只剩 TRACK_WAIT 那一次
            #                       确认: 人是「确认下降」的那一下, 之后全自动;
            #   true (默认)      -> 退回原行为, 由人看一眼夹爪确实到了抓取点再按 Enter。
            # **关掉它的代价必须说清楚**: 下降不判是否到位 (见上面 descend_dwell),
            # 闭合时刻完全由 descend_dwell 决定 —— 机械臂还没走完就闭合的话, 夹爪会在
            # 半空合上。真机上出现「夹了个空」先调大 descend_dwell, 而不是改成 true。
            # 另外原来那句「手还伸在夹爪里也来得及退开」只在 true 时成立。
            "grasp_confirm_required": True,
            # true 时额外要求固件自报「已到达」(motion_status == 0) 才算到位。
            # 默认 false: 该字段是否稳定上报尚未在真机验证, 硬性依赖它一旦不上报
            # 就会卡到 motion_timeout 而失败。只作用于 PRE_GRASP / LIFT / HOME ——
            # DESCEND 不看反馈, 这个开关管不到它。
            "use_motion_status": False,
            # TCP 姿态来源:
            #   current_tcp = 保持任务开始时 TCP 的当前姿态 (无需为 H3001 标定姿态,
            #                 但要求事先把手腕摆到合适的俯视姿态)
            #   fixed       = 使用下面的 side/top_orientation_xyzw
            "orientation_source": "current_tcp",
            # 固定 TCP 姿态 (x, y, z, w), 按 grasp_mode 切换。
            # side: 2026-09-14 由 H3001 实机 side 抓取姿态直接读取 (arm 静止, 407 帧零漂移)。
            # top : 仍抄自松灵 piper 配置, 对 H3001 几乎肯定不对, 用 top 模式前必须重新标定。
            "side_orientation_xyzw": [
                0.054892131727544674, 0.030223952165843913,
                -0.994253992704515, 0.08678919622287587,
            ],
            "top_orientation_xyzw": [
                -0.0543224092682961, 0.8397357076455230,
                -0.0794954161305229, 0.5343907708508778,
            ],
            # 运动监控
            "position_tolerance": 0.005,
            "settle_time": 0.30,
            "motion_timeout": 30.0,
            # ── 迭代末端修正 ──
            # 这台 H3001 的 E2PROM 重力参数 (m/Gx/Gy/Gz) 六轴全为 0, MCU 没有重力前馈,
            # 位置环只能靠关节稳态误差顶住重力, 于是末端在重力方向固定够不到目标。
            # 2026-09-14 实测: x 差 2 微米、z 差 9.8mm; 偏差随姿态在 3~55mm 之间变化;
            # 状态帧自报 motion_state=0 (已停稳), 30s 也不会自我修正。
            # 偏差是可复现的, 所以用「发指令 -> 量残差 -> 把残差加到指令上重发」迭代掉。
            # 这是权宜之计, 根治是补齐 E2PROM 重力参数 (找厂商要 J1~J6 的 m/Gx/Gy/Gz)。
            "endpoint_correction_enabled": True,
            "max_correction_retries": 4,
            # 修正量上限 (m)。防止病根不是下垂 (例如真的被挡住) 时指令被越推越远。
            "max_correction_distance": 0.08,
            # 停稳判定: 位置连续 stall_detect_time 秒变化不超过 stall_detect_epsilon,
            # 就认为机械臂已经停在新的稳态上, 此时量到的残差才算「够不到的偏差」。
            # 必须明显长于 descend_settle_time, 否则会在机械臂还在走的时候误判停稳。
            "stall_detect_time": 1.0,
            "stall_detect_epsilon": 0.0005,
            # ── 固件 IK 判决 (2026-09-21) ──
            # 开关: 收到 /feedback/arm_status 里 ik_status 非 0 (拒收) 时立刻判失败,
            # 不再等满 motion_timeout。**这是「命令发出去了、机械臂一动不动」的
            # 快速通道**: 2026-09-21 10:17 的放置失败, MCU 当场就回了 status=2,
            # 执行节点却只能等 30s 再报「运动超时 偏差 413mm」, 病根被埋在噪声里。
            "ik_verdict_enabled": True,
            # 指令发出后等多久再读判决 (s)。MCU 要收齐 3 帧位姿 (SDK 的
            # send_target_pose 分 XY / Z+RX / RY+RZ 三帧、帧间 20ms) 才重新判决,
            # 判决再经 CAN 回到节点; 而状态快照 100Hz 发布, 所以 0.8s 足够。
            # 给小的风险: 读到上一条指令的拒收判决, 把一条好指令误判成拒收。
            "ik_verdict_grace": 0.8,
            # 单条运动指令的位移上限 (m), **与走 move_p 还是 move_l 无关** —— 判据看的
            # 是两端距离, 不是路径形状 (LIFT 不管走哪条都受它管)。
            # **防的是「感知给的坐标突然跳一大截」**
            # (同一个 track_id 的圆柱拟合翻面之类, 见 README §5.10), 不是机械臂的
            # 物理极限 —— 轨迹怎么走由固件自己规划。超过就判任务失败并回待机位。
            # **放置段不受它约束** (2026-09-20, _enter_place): 放置点是 yaml 里的
            # 常量、不是感知, 距离远是正常的, 那一段一条 move_p 直接发出去。
            "max_command_distance": 0.50,
            # ── 上位机求逆解 (2026-09-28) ──
            # 现场总闸。false = 全部运动段走今天那套 (move_p / move_l), 与 09-28 之前
            # 逐字相同 —— 出问题时第一件事就是把它关掉。加载期校验不过 (pinocchio /
            # URDF / CAD CSV 缺一个) 也会被自动置 false, 节点照常起得来。
            "ik_motion_enabled": True,
            # 关节角约定: `q_物理(URDF) = ik_joint_sign · q_话题`。
            # 真机两步实测确认 (README §5.15): 发一个 J2 的值出去、看回报是变号还是
            # 变大 —— 变号就说明两套约定不同。换机械臂或换固件后要重测这条。
            "ik_joint_sign": [1.0, -1.0, -1.0, 1.0, -1.0, 1.0],
            # 距 **CAD** 机械限位的最小允许余量 (deg)。注意 CAD 那套比 URDF 紧得多
            # (J5: ±69.9 vs ±100.3), 而 `ik()` 的默认限位是 URDF 那套 —— 所以求解时
            # 就必须把 CAD 限位喂进 bounds, 不能只在这里事后检查。见 _load_ik_config。
            "ik_min_margin_deg": 5.0,
            # 相对**种子**(当前反馈关节角)允许的最大单轴变化 (deg)。这是「换构型」判据:
            # 本地求解会在解断裂处跳到另一条分支, 而那种解**收敛、误差 0、余量还有几十度**
            # (实测 J4 跳 117.6° 时余量 34.7°) —— 余量检查抓不到它。move_j 是关节插值,
            # 这条跳变会被原样执行成一次实打实的关节大摆, 手里可能还夹着瓶子。
            "ik_max_joint_jump_deg": 60.0,
            # 目标姿态的 rx/ry 是否取反 —— **必须与桥的 invert_rx_ry 成对**。
            # 取反是自逆的, 所以两处用同一个值; 不一致时 IK 解的是另一个位姿。
            # 当前桥是 false (见 config/h3001_bridge.yaml), 所以这里也是 false。
            "ik_target_invert_rx_ry": False,
            # 关节反馈比这旧就不做 IK (宁可不做, 也不拿过期构型当种子)。(s)
            "ik_seed_max_age": 0.5,
            # 走 J 的一段用多少秒判超时 (s)。J 的速度语义与 P/L 不同 (enter_can_mode
            # 对 J 的钳位是 0..100, 对 P/L/C 是 0..120), 所以不能直接沿用 motion_timeout
            # 那套按笛卡尔距离估的时间预算 —— 实测标定前先给一个保守值。
            "move_j_timeout": 30.0,
            # vendored 的求逆解三件套 (空 = 用 share/<pkg>/ik/ 里的那份)。见 setup.py。
            "ik_urdf_path": "",
            "ik_cad_limits_csv": "",
            "input_timeout": 2.0,
            # 工作空间 (base_link, m)
            "workspace_x_min": -0.60,
            "workspace_x_max": 0.60,
            "workspace_y_min": -0.60,
            "workspace_y_max": 0.60,
            "workspace_z_min": -0.10,
            "workspace_z_max": 0.60,
            # ── 回到待机位 ──
            # 代码默认关: 用 ros2 run 直接起节点时不会突然多出一段运动。
            # yaml 里是 true (与 gripper_offset_tool_xyz / orientation_source 同一套约定)。
            "return_to_standby_on_finish": False,
            # 待机位, 固件/GUI 格式 [X,Y,Z (mm), RX,RY,RZ (度)], 上报末端原点系。
            # 6 个数**必须都写成浮点** (78.0 而不是 78): rcl_yaml_param_parser 要求
            # 序列元素同类型并按序列推断数组类型, 混合写法会让节点起不来。
            # 仅加载期读取, 改它要重启节点。
            "standby_pose_reported_mm_deg": [78.0, 0.0, 290.0, 2.4, 4.2, -180.0],
            # 回位的到位判定 (m) 与停稳时间 (s)。比 position_tolerance (0.005) 松:
            # 回位只要求「离开工作区、停在待机位附近」, 不是抓取精度; 而重力下垂
            # 在别的姿态下实测 3~55mm, 卡在 5mm 会每次任务都白等一个 motion_timeout。
            "standby_tolerance": 0.015,
            "standby_settle_time": 0.30,
            # 机械臂报故障 / 反馈断了时, 是否**仍然**回位。
            # 回位是一条**绝对**位姿指令, 不依赖反馈也能让固件把机械臂开回待机位;
            # 而「任务失败后把机械臂丢在原地」是更糟的结果 (2026-09-16 16:38:
            # bridge 在回位途中挂掉, 机械臂停在抓取点上方不动, 回位也放弃了)。
            # 这种回位是**盲走**: 不再拆段插值 (起点来自反馈, 不可信), 直接发一次
            # 绝对待机位, 不判到位; 反馈一旦恢复就自动交回正常判据。
            # 置 false 退回旧行为: 不可控就不回位, 直接回 IDLE。
            "return_to_standby_on_error": True,
            # 盲走时「等反馈恢复」的上限 (s)。满了仍没恢复就结束回位, 并明确报出
            # 「无法确认是否到位」—— 不能把没确认的事说成已回到待机位。
            "blind_home_dwell": 8.0,
            # ── 夹爪 (2026-09-17) ──
            # 这一组是**夹爪这个设备**的参数 (Gloria-M, 串口 /dev/ttyACM0), 与上面
            # 的 gripper_offset_tool_xyz 无关 —— 那个是「H3001 上报的末端原点离夹爪
            # 有多远」, 是机械臂侧的事。
            #
            # 节点不直接驱动夹爪, 而是 spawn scripts/gripper_grip_hold.py 跑一条
            # 命令。为什么不 direct 用厂商的 04/05: 那两个 demo 在 finally 里无条件
            # disable(), 电一断夹爪就松, 抬升时手里其实是空的。原因与取舍见
            # README_H3001_grasp.md §5.13。
            #
            # 2026-09-28 起 close 之后**可以一直保持力矩** (见 gripper_hold_torque):
            # 那时进程不再退出, 节点靠它 stdout 上那行 [holding] 推进, 并在放置时
            # 往它的 stdin 写一行 OPEN 让它松手。gripper_hold_torque=0 (代码默认)
            # 时行为与改动前逐字相同 —— 那是一次性命令, 等退出码。
            #
            # 部署开关 (同 place_enabled 的约定): 代码默认关、yaml 里也先关着,
            # 接线与台架实测都过了再改 true。**仅加载期读取**, 改它要重启节点。
            "gripper_enabled": False,
            # 串口 CAN 适配器。**与机械臂的 can0 不是同一个口** (那是 socketcan)。
            "gripper_port": "/dev/ttyACM0",
            # 开/闭共用的速度 (rad/s)。取自用户标定好的 04/05 命令 --velocity 2。
            "gripper_velocity": 2.0,
            # 阻力检测: 夹紧过程中力矩超过「空载力矩 + 这个值」就停, 认为夹到东西了。
            # 0.3 是用户在真机上对着瓶子调出来的 (04 的 --resistance-torque 0.3)。
            # **这是「闭合到哪里停」的判据, 不是成败判据** —— 够不到阈值就一路合到底,
            # 脚本在 [stop] 里说明, 节点照样往下走。
            "gripper_resistance_torque": 0.3,
            # 硬上限: 任何时候力矩碰到它就停下 (04 的 --hard-torque-limit 2)。
            # 这是防夹坏东西/防堵转的那一道, 不要为了「夹得更紧」去调大 —— 抬起期间
            # 夹多紧由 gripper_hold_torque 决定, 这两个值的关系也受它约束
            # (hold_torque 必须 < 硬上限, 加载期会拦)。
            # 撞上它**不算失败**: 2026-09-18 真机上扭矩 0.43s 就冲到 -2.160 (阈值 +0.321,
            # 消抖 0.12s 还没成立), 夹爪其实已经夹住了, 老代码判失败让整趟任务作废 ——
            # 所以现在按「已夹住」停下, 只在 stderr 打一行 warn 说明为什么提前停。
            "gripper_hard_torque_limit": 2.0,
            # 传给脚本的 --timeout (s): 脚本内部一次闭合/张开的墙钟上限。
            "gripper_script_timeout": 25.0,
            # **节点侧**的墙钟上限 (s): 从 spawn 到命令跑完最多等多久。
            # 必须 > gripper_script_timeout, 让脚本先干净退出并自己报告原因
            # (退出码 + stdout 上的 [abort]); 反过来的话节点会在脚本还没来得及
            # 说明原因时就把它判死, 日志里只剩一句「超时」。
            # 2026-09-28 由 30 提到 40: 闭合那一条命令的脚本侧预算是
            # _wait_for_fresh_feedback(0.5) + _measure_idle_torque(≤2) + 闭合(25)
            # + 切 MIT 模式(≤0.6) ≈ 28.1s, 原来只剩 1.9s 余量, 太薄。
            "gripper_timeout": 40.0,
            # ── 保持 (2026-09-28) ──
            # 闭合停下后是否**持续保持力矩**。0 = 关闭 (代码默认, 与改动前逐字相同):
            # 脚本停在阻力点上就失能退出, 抬升/搬运期间夹持力全靠机构自锁。
            # > 0 = 脚本停在阻力点上不退出, 切 MIT 持续发这个力矩 [N·m] 直到节点在
            # 放置时叫它松手 (写一行 OPEN), 或 hold_timeout 到点。
            #
            # **它和 gripper_enabled 一样是部署开关**(代码默认保守、yaml 里开),
            # 所以刻意不进 test_gripper.py 的 NEW_GRIPPER_KEYS 一致性名单。
            #
            # 两个数要知道: ① 它**必须小于** gripper_hard_torque_limit (脚本会拦),
            # ② 它和 gripper_resistance_torque 没关系 —— 后者只决定「在哪停下」,
            # 抬起期间夹得紧不紧由这个值决定。低于停下那一刻测到的扭矩时, 切过去的
            # 一瞬间夹持力会先降一下 (脚本打一行 warn)。
            "gripper_hold_torque": 0.0,
            # 保持的最长时间 (s): 最后兜底。只在这段时间里**没人叫停也没人管**时才生效
            # (节点正常会在放置时写 OPEN, 失败/退出时会直接杀进程), 所以给得宽一点
            # 不影响正常流程 —— 给窄了反而会把「人在 PLACE_WAIT 慢慢确认」截断。
            # 到点脚本失能放手并打一行 warn; 机械臂若还悬空, 物体会掉。
            "gripper_hold_timeout": 300.0,
            # 保持帧的 MIT 阻尼项 [N·m·s/rad], 范围 [0, 5] (12bit 打包, 超了静默截断)。
            # 0.5 抄 demos/03 的 --hold-kd。
            "gripper_hold_kd": 0.5,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

    # ── 手眼矩阵 ──────────────────────────────────────────

    def _load_hand_eye_matrix(self):
        path = self.get_parameter("hand_eye_yaml_path").value
        with open(path) as handle:
            data = yaml.safe_load(handle)
        matrix = np.asarray(data["transform_matrix"], dtype=np.float64)
        if matrix.shape != (4, 4):
            raise RuntimeError(f"手眼矩阵形状错误: {matrix.shape}, 期望 4x4: {path}")
        rotation = matrix[:3, :3]
        deviation = float(np.max(np.abs(rotation @ rotation.T - np.eye(3))))
        if deviation > 1e-6:
            self.get_logger().warn(
                f"手眼旋转矩阵正交性偏差 {deviation:.2e}, 建议重新标定: {path}"
            )
        self.get_logger().info(f"已加载手眼矩阵: {path}")
        return matrix

    # ── 待机位 ────────────────────────────────────────────

    @staticmethod
    def _rpy_to_quaternion(roll_rad, pitch_rad, yaw_rad):
        """ZYX 顺序 RPY (rad) → 四元数 (x, y, z, w)。

        与 h3001_bridge._rpy_to_quaternion 同式。那边是另一个包的模块级私有函数,
        这里保留一份拷贝 —— 两边必须一起改。
        invert_rx_ry=false 时固件 RPY 可以直接按 ZYX 转换 (README §5.6)。
        """
        cr, sr = math.cos(roll_rad * 0.5), math.sin(roll_rad * 0.5)
        cp, sp = math.cos(pitch_rad * 0.5), math.sin(pitch_rad * 0.5)
        cy, sy = math.cos(yaw_rad * 0.5), math.sin(yaw_rad * 0.5)
        return (
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * sp * sy,
        )

    def _load_standby_pose(self):
        """把 standby_pose_reported_mm_deg 换算成夹爪系, 并做一次加载期校验。

        参数是 **上报末端原点系** 的 [X,Y,Z (mm), RX,RY,RZ (度)], 就是 /feedback/tcp_pose
        和厂商 GUI 上的那 6 个数。换算只发生在这里:
            夹爪 = 上报 + R @ offset          (_reported_to_gripper)
        姿态不用换算 —— 纯平移不改变姿态, 两个系共用同一个四元数。

        为什么要存源头格式而不是存换算后的值: 换算依赖 gripper_offset_tool_xyz, 而那个
        偏移是临时方案 (README §6.4), 将来换臂要置零。存源头格式, 那天这里不用重算。

        校验不通过就 _standby_ready = False (关掉回位), **不 raise**: 节点的本职是抓取,
        用户填错一个待机位不该让整个可执行文件起不来; 日志够响, 且抓取功能不受影响。
        """
        raw = [float(v) for v in self.get_parameter("standby_pose_reported_mm_deg").value]
        if len(raw) != 6 or not np.all(np.isfinite(raw)):
            self.get_logger().error(
                f"standby_pose_reported_mm_deg 需要 6 个有限数 "
                f"[X,Y,Z(mm),RX,RY,RZ(度)], 实际 {raw} —— 回位功能已禁用"
            )
            return
        reported = np.asarray(raw[:3], dtype=np.float64) * 0.001
        orientation = self._rpy_to_quaternion(*(math.radians(v) for v in raw[3:]))
        norm = math.sqrt(sum(v * v for v in orientation))
        if norm < 1e-12:
            self.get_logger().error("待机位四元数模长为零 —— 回位功能已禁用")
            return
        orientation = tuple(v / norm for v in orientation)
        if orientation[3] < 0.0:
            # 规范到 w >= 0: q 与 -q 是同一个旋转, 但这样日志里的数和文档里的一致。
            # 这个姿态 w≈0.0008 (接近 180°), 不定这一下会打印出四个分量全反的值。
            orientation = tuple(-v for v in orientation)
        position = self._reported_to_gripper(reported, orientation)

        # 单位自检: 位置按 mm 解释。若有人按米填 (0.078 而不是 78), 上报原点会变成
        # 0.000078 m —— 几乎贴在基座原点上, 而工作空间的 z 下限 (-0.10) 拦不住这种错。
        # 判据必须用 **reported** 而不是换算后的夹爪点: 后者 = reported + R@offset,
        # 即使 reported 归零也还有 0.08 m 的偏移, 永远够不到这个阈值。
        if float(np.linalg.norm(reported)) < 0.05:
            self.get_logger().warn(
                f"待机位的上报原点离基座原点只有 "
                f"{np.linalg.norm(reported) * 1000.0:.1f} mm "
                f"({np.round(reported, 4)} m) —— 检查 standby_pose_reported_mm_deg 的 "
                "位置是否误按米填写 (应为 mm)"
            )
        # 夹爪点与上报原点都要在工作空间内: 两者差 |gripper_offset| (本机 0.08 m),
        # 一个在界内另一个仍可能越界, 同 _check_workspace_pair 的理由。
        for target, label in ((position, "待机位(夹爪)"), (reported, "待机位(上报原点)")):
            problem = self._workspace_problem(target, label)
            if problem is not None:
                self.get_logger().error(
                    f"WORKSPACE_REJECTED {problem} —— 待机位不可用, 回位功能已禁用 "
                    "(抓取功能不受影响)"
                )
                return
        self._standby_position = position
        self._standby_orientation = orientation
        self._standby_ready = True
        # 四元数手写格式化: np.round 对小分量会用科学计数法, 而这一行就是拿来和
        # yaml 注释里的数逐位对照的, 必须好认。
        quaternion_text = "[" + ", ".join(f"{v:+.8f}" for v in orientation) + "]"
        self.get_logger().info(
            f"待机位: 上报原点={np.round(reported, 4)} m (与 GUI/参数一致才是对的), "
            f"夹爪={np.round(position, 4)} m, "
            f"偏移={np.round(position - reported, 4)} m "
            f"(模长 {np.linalg.norm(position - reported) * 1000.0:.1f} mm), "
            f"姿态={quaternion_text}"
        )

    # ── 放置点 ────────────────────────────────────────────

    def _load_place_pose(self):
        """校验 place_pose_reported_mm_deg, 通过则置 _place_ready / _place_orientation。

        参数是 **上报末端原点系** 的 [X,Y,Z (mm), RX,RY,RZ (度)], 与
        standby_pose_reported_mm_deg 同一套约定、同一套换算:
            夹爪 = 上报 + R @ gripper_offset_tool_xyz          (_reported_to_gripper)
        姿态不用换算 —— 纯平移不改变姿态, 两个系共用同一个四元数。

        位置**和姿态**都取自参数: 放置段不再沿用抓取时的姿态 (2026-09-18 用户要求)。
        好处是放置姿态可以按「机械臂解得出来」挑, 而不是被抓取姿态绑死。

        校验不通过就 _place_ready = False (整段放置跳过, 退回 LIFT -> HOME), **不 raise**:
        与待机位同一条理由 —— 用户填错一个坐标不该让整个可执行文件起不来。但这里是
        「用户开着 place_enabled 却填错了」, 所以 _on_execute 会拒绝启动任务, 而不是
        抓完才发现无处可放。
        """
        # 先清空再校验: 这个函数是**唯一**写这三个字段的地方, 清在这里才能保证
        # 「_place_ready 为真 == 当前参数校验通过」。否则改坏参数后再调一次, 旧坐标
        # 会连同 _place_ready 一起留下 (加载期只跑一次, 但测试与将来的热重载会再调)。
        self._place_ready = False
        self._place_position = None
        self._place_orientation = None
        if not self.get_parameter("place_enabled").value:
            self.get_logger().info(
                "放置: 关 (抬升后直接回待机位; 置 place_enabled=true 并填好 "
                "place_pose_reported_mm_deg 即可开启)"
            )
            return
        raw = [float(v) for v in self.get_parameter("place_pose_reported_mm_deg").value]
        if len(raw) != 6 or not np.all(np.isfinite(raw)):
            self.get_logger().error(
                f"place_pose_reported_mm_deg 需要 6 个有限数 "
                f"[X,Y,Z(mm),RX,RY,RZ(度)] (上报末端原点系), 实际 {raw} —— "
                "放置功能已禁用, 任务会被拒绝启动"
            )
            return
        reported = np.asarray(raw[:3], dtype=np.float64) * 0.001
        orientation = self._rpy_to_quaternion(*(math.radians(v) for v in raw[3:]))
        norm = math.sqrt(sum(v * v for v in orientation))
        if norm < 1e-12:
            self.get_logger().error("放置位姿四元数模长为零 —— 放置功能已禁用, 任务会被拒绝启动")
            return
        orientation = tuple(v / norm for v in orientation)
        if orientation[3] < 0.0:
            # 规范到 w >= 0, 与待机位同一个理由: q 与 -q 是同一个旋转, 但这样日志里的
            # 数和文档里的一致。放置姿态常见 -180° (w≈0), 不定这一下会打印出四个分
            # 量全反的值。
            orientation = tuple(-v for v in orientation)
        position = self._reported_to_gripper(reported, orientation)

        # 占位值/单位的兜底: [0,0,0,...] 是参数没填时的默认值, 而换算后的夹爪点 =
        # reported + R@offset 仍有 0.08 m 的偏移, 落在工作空间内 —— 单靠下面的
        # workspace 检查拦不住, 放行就等于让机械臂往自己底座里扎。
        # 判据必须用 **reported** 而不是换算后的夹爪点, 同 _load_standby_pose:
        # 按 mm 填的那种量级错误 (0.19 而不是 190) 也一并被这条拦下。
        if float(np.linalg.norm(reported)) < 0.05:
            self.get_logger().error(
                f"放置点的上报原点离基座原点只有 {np.linalg.norm(reported) * 1000.0:.1f} mm "
                f"({np.round(reported, 4)} m) —— 多半是 place_pose_reported_mm_deg 还没填 "
                "(默认占位值就是全零) 或位置误按米填写 (应为 mm); "
                "放置功能已禁用, 任务会被拒绝启动"
            )
            return
        # 夹爪点与上报原点都要在工作空间内: 两者差 |gripper_offset| (本机 0.08 m),
        # 一个在界内另一个仍可能越界, 同 _check_workspace_pair 的理由。
        for target, label in ((position, "放置点(夹爪)"), (reported, "放置点(上报原点)")):
            problem = self._workspace_problem(target, label)
            if problem is not None:
                self.get_logger().error(
                    f"WORKSPACE_REJECTED {problem} —— 放置功能已禁用, 任务会被拒绝启动"
                )
                return
        self._place_position = position
        self._place_orientation = orientation
        self._place_ready = True
        quaternion_text = "[" + ", ".join(f"{v:+.8f}" for v in orientation) + "]"
        self.get_logger().info(
            f"放置位姿: 上报原点={np.round(reported, 4)} m (与 GUI/参数一致才是对的), "
            f"夹爪={np.round(position, 4)} m (base_link), 姿态={quaternion_text}; "
            "抬升后直线直达此位姿 (判到位), 停 "
            f"{self.get_parameter('place_descend_dwell').value:.1f}s 后放下"
        )

    # ── 夹爪 ──────────────────────────────────────────────

    def _load_gripper_config(self):
        """定位夹爪脚本并做一次加载期校验, 通过则置 _gripper_ready。

        与 place_enabled 同一套约定: 校验不过就关掉功能并 **不 raise** (用户填错一个
        参数不该让整个可执行文件起不来), 但 _on_execute 会拒绝启动任务 —— 拒绝在
        任务开始前, 而不是抓完才发现在手上没法夹。
        """
        if not self.get_parameter("gripper_enabled").value:
            self.get_logger().info(
                "夹爪: 关 (置 gripper_enabled=true 让节点接管夹爪)"
            )
            return
        if not self.get_parameter("dry_run").value:
            # 只在真机上查串口: dry_run 不碰硬件, 台架上没这个口也要能跑完整个流程。
            # 真机上前置地查一次, 是为了把「串口被占用」(最常见: 厂商 web server
            # 还在跑, 或者已经有一个 04 在另一终端开着) 变成启动时就能看见的错。
            port = str(self.get_parameter("gripper_port").value)
            if not os.path.exists(port):
                self.get_logger().error(
                    f"gripper_port={port} 不存在 —— 夹爪功能已禁用, 任务会被拒绝启动; "
                    "检查串口适配器是否插好 (ls /dev/ttyACM*)"
                )
                return
        hold_torque = float(self.get_parameter("gripper_hold_torque").value)
        hard_limit = float(self.get_parameter("gripper_hard_torque_limit").value)
        if hold_torque < 0.0:
            self.get_logger().error(
                f"gripper_hold_torque={hold_torque:g} 不能是负数 —— "
                "夹爪功能已禁用, 任务会被拒绝启动"
            )
            return
        if hold_torque >= hard_limit:
            # 硬上限只写在闭合循环里; 保持比它还大就等于绕过了那道保护
            # (脚本自己也会拦, 但那是运行期才发现的, 不如启动时就说清楚)。
            self.get_logger().error(
                f"gripper_hold_torque={hold_torque:g} 不小于 "
                f"gripper_hard_torque_limit={hard_limit:g} —— 保持期间会一直顶着超过"
                "保护上限的力; 夹爪功能已禁用, 任务会被拒绝启动"
            )
            return
        script = self._resolve_gripper_script()
        if script is None:
            return
        self._gripper_script = script
        self._gripper_ready = True
        self.get_logger().info(f"夹爪脚本: {script}")
        if hold_torque > 0.0:
            self.get_logger().info(
                f"夹爪保持: 开 ({hold_torque:g} N·m, MIT 持续发帧; "
                f"放置时由节点写 OPEN 松开, 最长 "
                f"{float(self.get_parameter('gripper_hold_timeout').value):.0f}s)"
            )
        else:
            self.get_logger().info(
                "夹爪保持: 关 (gripper_hold_torque=0) —— 闭合后失能退出, "
                "抬升/搬运期间靠机构自锁"
            )

    def _resolve_gripper_script(self):
        """找 `scripts/gripper_grip_hold.py` 的绝对路径; 找不到返回 None (已记 error)。

        先按 ROS 包目录找 (install 之后是 share/bottle_grasp_executor/scripts/),
        再退回源码目录 —— 单元测试直接从 src 起节点, 那时包还没装,
        get_package_share_directory 会抛 PackageNotFoundError。
        """
        relative = os.path.join("scripts", "gripper_grip_hold.py")
        candidates = []
        try:
            share = get_package_share_directory("bottle_grasp_executor")
        except PackageNotFoundError:
            share = None
        except Exception as exc:  # noqa: BLE001 - ament 索引出任何问题都该退回源码目录
            self.get_logger().warn(f"查询包目录失败 ({exc}), 改用源码目录找夹爪脚本")
            share = None
        if share:
            candidates.append(os.path.join(share, relative))
        # 本文件在 <包根>/bottle_grasp_executor/ 下, 往上两层就是包根。
        package_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        candidates.append(os.path.join(package_root, relative))
        for path in candidates:
            if os.path.isfile(path):
                return path
        self.get_logger().error(
            f"找不到夹爪脚本 {relative} (找过 {candidates}) —— 夹爪功能已禁用, "
            "任务会被拒绝启动; scripts/ 是后加的目录, 需要先 colcon build --symlink-install "
            "一次才会出现在 install/ 里"
        )
        return None

    # ── 上位机求逆解 (2026-09-28) ──────────────────────────

    def _resolve_ik_file(self, relative, override):
        """找 `ik/` 下的一个文件。空 override = 用默认名; 找不到返回 None (已记 error)。

        与 _resolve_gripper_script 同一条路子: 先 share/<pkg>/ik/, 再源码目录的 ik/。
        """
        if override:
            return override if os.path.isfile(override) else None
        candidates = []
        try:
            share = get_package_share_directory("bottle_grasp_executor")
            candidates.append(os.path.join(share, "ik", relative))
        except Exception:  # noqa: BLE001 - 包没装就走源码目录
            pass
        package_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        candidates.append(os.path.join(package_root, "ik", relative))
        for path in candidates:
            if os.path.isfile(path):
                return path
        self.get_logger().error(f"找不到 {relative} (找过 {candidates})")
        return None

    @staticmethod
    def _cad_csv_joint_names(csv_path):
        """CAD 限位 CSV 里 `Joint Name` 那一列的全部关节名 (set)。

        刻意**不复用** cad_limits 的返回值来判这件事: 它对查不到的关节静默退回
        (-π, π), 返回值里分不出「查到的限位正好是 ±180°」和「根本没查到」——
        2026-09-30 就是栽在这个分不出上 (见 _load_ik_config 里那段说明)。
        这一列才是「解析全没全」的直接证据。

        (本方法名与 `import csv` 不冲突: _load_ik_config 里那个局部变量已经从 `csv`
        改名成 `csv_path`, 免得遮住模块名。)
        """
        names = set()
        with open(csv_path, newline="", encoding="utf-8-sig") as stream:
            for row in csv.DictReader(stream):
                name = (row.get("Joint Name") or "").strip()
                if name:
                    names.add(name)
        return names

    def _load_ik_config(self):
        """加载求逆解三件套 (vendored ik_h3001 + URDF + CAD 限位)。**绝不 raise**。

        与 _load_gripper_config 同一套约定: 校验不过就关掉这项功能, 节点照常起得来 ——
        它是现场唯一能干活的东西, 旧路 (move_p / move_l) 完全可用, 不该因为一个
        可选功能的依赖缺失就起不来。

        **三道守门的第 ① 道在这里落地**: CAD 限位必须解析出来并喂给求解器。`ik()` 的
        默认限位是 URDF 那套, 与 CAD 限位仍有实质差别 —— 典型的是 J2: CAD 写
        [0, +179.9], URDF 却写 ±180 (反方向整段被机械限位禁掉)。只事后检查余量的话,
        「J2 解到 -90°」会带着 90° 余量被放行。
        """
        if not self.get_parameter("ik_motion_enabled").value:
            self.get_logger().info(
                "上位机求逆解: 关 (ik_motion_enabled=false) —— 全部运动段走 "
                "move_p / move_l, 与 2026-09-28 之前一致"
            )
            return
        try:
            import pinocchio  # noqa: F401,PLC0415 - 只在启用时才需要它
        except Exception as exc:  # noqa: BLE001
            self._ik_disable(f"import pinocchio 失败 ({exc})")
            return
        script = self._resolve_ik_file("ik_h3001.py", None)
        urdf = self._resolve_ik_file("L4002-0100.urdf", self.get_parameter("ik_urdf_path").value)
        csv_path = self._resolve_ik_file(
            "L4002-0100-cad-limits.csv", self.get_parameter("ik_cad_limits_csv").value
        )
        if script is None or urdf is None or csv_path is None:
            self._ik_disable("vendored 的 ik_h3001.py / URDF / CAD CSV 有缺")
            return
        try:
            spec = importlib.util.spec_from_file_location("ik_h3001_vendored", script)
            module = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(module)
            model, data, fid = module.load_model(urdf)
            lo, hi, names = module.cad_limits(csv_path, model)
        except Exception as exc:  # noqa: BLE001 - 坏掉的模型不该拖垮节点
            self._ik_disable(f"加载模型失败 ({exc})")
            return
        lo = np.asarray(lo, dtype=float)
        hi = np.asarray(hi, dtype=float)
        try:
            csv_joints = self._cad_csv_joint_names(csv_path)
        except Exception as exc:  # noqa: BLE001 - 同本函数的约定: 绝不 raise
            self._ik_disable(f"读不了 CAD 限位表 {csv_path} ({exc})")
            return
        # 限位表必须**逐关节都真的解析出来**: cad_limits 对查不到的关节名静默退回
        # (-π, π), 那等于把安全边界悄悄换回 URDF 那套, 而且不会报错 —— 必须在这里拦住。
        #
        # 判据只能是「**名字在不在**」, 不能拿数值反推。这里原来写的是
        # `degrees(hi - lo) > 350` —— 拿「跨度接近整圈」当「退回哨兵 (-π, π)」的指纹。
        # 2026-09-30 那次限位数据更新把 joint_6 改成了真值 ±180°, 与哨兵**完全重合**,
        # 于是名字明明对得上也被判成「没解析全」: 求逆解被整个关掉, 全部运动段静默回落
        # move_p / move_l —— 现场表现就是「明明该发 move_j, 桥上却全是 move_p」。
        # 名字是这里唯一能区分「查到了」和「退回了」的东西。
        missing = [n for n in model.names[1:] if n not in csv_joints]
        # 跨度 <= 0 是另一种「行在、值废」: Limit Lower/Upper 为空会让 cad_limits 整行
        # 跳过 (名字因此缺失, 上面那条抓住), 但 0/0 这种能过解析的行只能在这里挡。
        empty_range = np.degrees(hi - lo) <= 0.0
        if len(lo) != 6 or missing or bool(np.any(empty_range)):
            detail = (
                f"CSV 里没有 {missing}"
                if missing
                else f"{int(empty_range.sum())} 组上下限反了或跨度为 0"
            )
            self._ik_disable(
                f"CAD 限位表没解析全 (得到 {len(lo)} 组, {detail}) —— "
                f"检查 {csv_path} 的关节名是否与 URDF 对得上"
            )
            return
        self._ik_module = module
        self._ik_model = (model, data, fid)
        self._ik_cad_lo, self._ik_cad_hi, self._ik_names = lo, hi, list(names)
        self._ik_ready = True
        limits_text = ", ".join(
            f"{n}[{np.degrees(l):.1f},{np.degrees(h):.1f}]"
            for n, l, h in zip(names, lo, hi)
        )
        self.get_logger().info(f"上位机求逆解: 开 ({script})")
        self.get_logger().info(f"  URDF: {urdf}")
        self.get_logger().info(
            f"  CAD 限位 (deg): {limits_text}; 要求余量 ≥ "
            f"{float(self.get_parameter('ik_min_margin_deg').value):.1f}°, "
            f"单轴跳变 ≤ {float(self.get_parameter('ik_max_joint_jump_deg').value):.0f}°"
        )
        self.get_logger().info(
            "  关节约定: q_物理 = ik_joint_sign · q_话题; 目标姿态 invert_rx_ry="
            f"{bool(self.get_parameter('ik_target_invert_rx_ry').value)} "
            "(**必须与桥的 invert_rx_ry 一致**) —— 不一致时 IK 解的是另一个位姿"
        )

    def _ik_disable(self, reason):
        """求逆解不可用: 关掉它并记 error。**不 raise** (节点必须起得来)。"""
        self._ik_ready = False
        self._ik_module = None
        self._ik_model = None
        self.get_logger().error(
            f"{reason} —— 上位机求逆解已禁用, 全部运动段回落 move_p / move_l "
            "(功能仍然完整, 只是走固件的 IK)"
        )

    def _camera_to_base(self, point_camera):
        point = np.asarray(point_camera, dtype=np.float64)
        return (self._T_base_camera[:3, :3] @ point) + self._T_base_camera[:3, 3]

    @staticmethod
    def _grasp_axes_to_tcp(grasp_mode):
        """感知 grasp 坐标系 -> H3001 TCP 的轴重标签 (在物体自身坐标系里右乘)。

        必须是右乘: 这个映射就是「夹爪怎么装在法兰上」的装配常数, 与瓶子摆在哪无关。
        写成 R_obj @ P 之后, 瓶子转 Rz(Δ), 夹爪跟着转 Rz(Δ), 工具 Z 恒竖直向下, 只有
        RZ 跟着瓶子 1:1 走。若改成基座系里的固定姿态 (左乘), 瓶子一转向就跟不上。

        top (躺瓶): TCP Z = -approach, 即沿桌面法向反向压向瓶顶, 是进给轴;
            TCP X = -closing 是两指闭合方向, 垂直瓶身长轴。
            2026-09-22 真机标定: 算出的姿态 RPY(ZYX) ≈ 180/0/54, 与人工摆出的正确抓取
            姿态差 2.8° (GUI 只读到整数) 即吻合; 下一名候选差 87.6°, 唯一解。
            原来的 [:, [2, 0, 1]] 把 approach 放到 TCP X 上, 工具 Z 落到瓶身长轴方向,
            进给轴横躺 90° —— 真机上就是「瓶子躺着的姿态」, 端面压不到瓶口。
        side (立瓶): 还没做真机标定, 维持原来的 [:, [2, 0, 1]]。目前 side 走不到这里
            (见 _uses_live_orientation), 留着它是为了放开 side 时退回原行为而不是报错。
        """
        if str(grasp_mode) == "top":
            P = np.diag([-1.0, 1.0, -1.0])

            # 绕 TCP/工具 Z 轴再转 +90°
            Rz90 = np.array([[0.0, -1.0, 0.0],
                            [1.0,  0.0, 0.0],
                            [0.0,  0.0, 1.0]])

            return P @ Rz90

        return np.eye(3)[:, [2, 0, 1]]

    @staticmethod
    def _uses_live_orientation(grasp_mode):
        """该模式的抓取姿态是否采用感知结果。

        只有 top 做过真机标定 (2026-09-22 实测 RPY(ZYX) ≈ 180/0/54)。side 还没量过,
        先不让 live 姿态盖掉 orientation_source 定的固定姿态 —— 等 side 也摆一次真机,
        把 "side" 加进来即可。
        """
        return str(grasp_mode) == "top"

    def _grasp_orientation_to_tcp(self, quaternion_grasp, grasp_mode, in_camera_frame):
        """grasp 姿态 -> base_link 下的 H3001 TCP 姿态。输入无效时返回 None。

        in_camera_frame=True: 输入在相机系 (话题里的 grasp_pose.orientation), 先乘手眼
            旋转转到基座系。
        in_camera_frame=False: 输入已在 base_link 下 (demo JSON 的
            orientation_base_xyzw), 只做轴重标签 —— 再乘一次 T_base_camera 就等于把
            162.6° 的手眼旋转重复叠上去。
        """
        try:
            quaternion = np.asarray(quaternion_grasp, dtype=np.float64)
        except (TypeError, ValueError):
            return None
        if quaternion.shape != (4,):
            return None
        norm = float(np.linalg.norm(quaternion))
        if norm < 1e-9 or not np.all(np.isfinite(quaternion)):
            return None
        rotation_grasp = self._quaternion_to_matrix(quaternion / norm)
        if in_camera_frame:
            rotation_grasp = self._T_base_camera[:3, :3] @ rotation_grasp
        rotation_base_tcp = rotation_grasp @ self._grasp_axes_to_tcp(grasp_mode)
        return self._matrix_to_quaternion(rotation_base_tcp)

    # ── ROS 回调 ──────────────────────────────────────────

    def _on_selected_candidate(self, msg: SelectedCandidate):
        now = self.get_clock().now().nanoseconds * 1e-9
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if stamp <= 0.0:
            # 无有效时间戳时按接收时刻处理
            stamp = now
        if now - stamp > self.get_parameter("max_result_age").value:
            return

        position_camera = (
            msg.grasp_pose.position.x,
            msg.grasp_pose.position.y,
            msg.grasp_pose.position.z,
        )
        orientation_base = self._grasp_orientation_to_tcp(
            (
                msg.grasp_pose.orientation.x,
                msg.grasp_pose.orientation.y,
                msg.grasp_pose.orientation.z,
                msg.grasp_pose.orientation.w,
            ),
            msg.grasp_mode,
            in_camera_frame=True,
        )
        if orientation_base is None:
            self.get_logger().warn(
                f"忽略 track_id={msg.track_id} 的候选: grasp_pose.orientation 无效"
            )
            return
        self._latest_candidate = {
            "track_id": msg.track_id,
            "grasp_mode": msg.grasp_mode,
            "position_base": self._camera_to_base(position_camera),
            "received_at": time.monotonic(),
        }
        if self._uses_live_orientation(msg.grasp_mode):
            self._latest_candidate["orientation_base"] = orientation_base

    def _on_candidates(self, msg: GraspCandidateArray):
        now = self.get_clock().now().nanoseconds * 1e-9
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if stamp <= 0.0:
            # 无有效时间戳时按接收时刻处理
            stamp = now
        if now - stamp > self.get_parameter("max_result_age").value:
            return
        items = []
        for candidate in msg.candidates:
            item = {
                "track_id": candidate.track_id,
                "grasp_mode": candidate.grasp_mode,
                "confidence": candidate.confidence,
                "quality": candidate.quality,
                "position_camera": (
                    candidate.pose.position.x,
                    candidate.pose.position.y,
                    candidate.pose.position.z,
                ),
            }
            if self._uses_live_orientation(candidate.grasp_mode):
                orientation = self._grasp_orientation_to_tcp(
                    (
                        candidate.pose.orientation.x,
                        candidate.pose.orientation.y,
                        candidate.pose.orientation.z,
                        candidate.pose.orientation.w,
                    ),
                    candidate.grasp_mode,
                    in_camera_frame=True,
                )
                if orientation is not None:
                    item["orientation_base"] = orientation
            items.append(item)
        self._select_and_store(items)

    def _poll_candidates_file(self):
        """轮询 demo 写入的候选 JSON 文件 (仅在 candidates_source=file 时调用)。"""
        path = self.get_parameter("candidates_json_path").value
        try:
            mtime = os.stat(path).st_mtime
        except OSError:
            return
        if mtime == self._candidates_file_mtime:
            return
        self._candidates_file_mtime = mtime
        try:
            with open(path) as handle:
                payload = json.load(handle)
        except (json.JSONDecodeError, OSError):
            return
        try:
            stamp = float(payload.get("stamp", 0.0))
        except (TypeError, ValueError):
            return
        if time.time() - stamp > self.get_parameter("max_result_age").value:
            return
        items = []
        for candidate in payload.get("candidates", []):
            try:
                position = candidate["position"]
                grasp_mode = str(candidate["grasp_mode"])
                item = {
                    "track_id": int(candidate["track_id"]),
                    "grasp_mode": grasp_mode,
                    "confidence": float(candidate["confidence"]),
                    "quality": float(candidate["quality"]),
                    "position_camera": (
                        float(position[0]),
                        float(position[1]),
                        float(position[2]),
                    ),
                }
                # demo 的 orientation_base_xyzw 已经在 base_link 下: 只做轴重标签, 不再
                # 乘手眼旋转。
                if self._uses_live_orientation(grasp_mode):
                    orientation = self._grasp_orientation_to_tcp(
                        candidate.get("orientation_base_xyzw"),
                        grasp_mode,
                        in_camera_frame=False,
                    )
                    if orientation is not None:
                        item["orientation_base"] = orientation
                items.append(item)
            except (KeyError, TypeError, ValueError, IndexError):
                continue
        self._select_and_store(items, self._parse_offset_base(payload))

    def _parse_offset_base(self, payload):
        """读 demo 的 --offset (基座系偏置, m); 缺省或非法时按零处理。

        demo 把 --offset 只加在 position_base 上, 而本节点是拿相机系 position
        自己做手眼变换的, 不读这个字段就等于把偏置吃掉了。
        """
        try:
            offset = np.asarray(
                payload.get("offset_base", (0.0, 0.0, 0.0)), dtype=np.float64
            )
        except (TypeError, ValueError):
            return None
        if offset.shape != (3,) or not np.all(np.isfinite(offset)):
            return None
        return offset if np.any(offset) else None

    def _select_and_store(self, items, offset_base=None):
        """按阈值筛选最佳候选, 手眼变换后存入 _latest_candidate。"""
        # 锁定之后只认锁定的 track_id。少了这一步, TRACK_WAIT 跟随期间视野里
        # 另一只质量更高的瓶子会把 _latest_candidate 顶掉, 机械臂就跟到别的物体上,
        # 甚至把夹爪开到那上面去下降。WAIT_TARGET 阶段不在这里过滤: 那一阶段的
        # 语义是「锁定前先挑一个」, 由下面的分支单独处理。
        if self._locked_track_id is not None and self._state != self._STATE_WAIT_TARGET:
            items = [item for item in items if item["track_id"] == self._locked_track_id]

        best = None
        for candidate in items:
            if candidate["confidence"] < self.get_parameter("min_confidence").value:
                continue
            if candidate["quality"] < self.get_parameter("min_quality").value:
                continue
            if best is None or candidate["quality"] > best["quality"]:
                best = candidate
        if best is None:
            if self._state == self._STATE_WAIT_TARGET and self._locked_track_id is not None:
                self.get_logger().warn("锁定目标丢失 (无满足阈值的候选), 重新等待")
                self._reset_target_lock()
            return

        if (
            self._state == self._STATE_WAIT_TARGET
            and self._locked_track_id is not None
            and best["track_id"] != self._locked_track_id
        ):
            # 锁定期间出现更高质量的其他目标: 保持锁定, 防止执行中切换
            return

        position_base = self._camera_to_base(best["position_camera"])
        if offset_base is not None:
            position_base = position_base + offset_base
            if not np.array_equal(offset_base, self._logged_offset_base):
                self._logged_offset_base = offset_base
                self.get_logger().info(
                    f"已应用 demo 的 --offset (基座系): {np.round(offset_base, 4)} m "
                    f"→ 抓取点 {np.round(position_base, 4)} m"
                )
        self._latest_candidate = {
            "track_id": best["track_id"],
            "grasp_mode": best["grasp_mode"],
            "position_base": position_base,
            # 收到时刻 (单调钟)。max_result_age 只能拦住「上游发出的数据本身很旧」,
            # 拦不住「上游整个不发了」—— 那种情况下 _latest_candidate 会一直停在
            # 最后一个值上。TRACK_WAIT 靠这个时间戳判断目标还新不新鲜 (见
            # _target_age), 从而决定是继续跟随还是原地悬停等人。
            "received_at": time.monotonic(),
        }
        # 只有做过真机标定的 mode 才带姿态 (见 _uses_live_orientation)。带上之后
        # _tick_wait_target 会优先用它; 不带则回落到 orientation_source 的固定姿态,
        # side 的候选就是这样。
        orientation_base = best.get("orientation_base")
        if orientation_base is not None:
            self._latest_candidate["orientation_base"] = orientation_base

    def _on_tcp_pose(self, msg: PoseStamped):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self._tcp_orientation = (
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w,
        )
        # 上报末端位姿 -> 夹爪位姿。旋转矩阵用的是 **上报姿态** (机械臂实际姿态),
        # 不是本次下发的指令姿态 —— 两者不一致时夹爪的真实落点由前者决定。
        self._tcp_pose = (
            stamp,
            self._reported_to_gripper(
                (
                    msg.pose.position.x,
                    msg.pose.position.y,
                    msg.pose.position.z,
                ),
                self._tcp_orientation,
            ),
        )

    def _on_joint_states(self, msg: JointState):
        """当前关节角 —— **IK 的种子**。存的是**话题约定**的度, 不做任何符号换算。

        (与 IK 之间的关系: `q_物理 = ik_joint_sign · q_话题`。换算只发生在
        `_solve_joint_target` 里, 这里保持「桥发什么就存什么」, 免得两个地方各换一次。)
        """
        if len(msg.position) < 6:
            return
        values = np.asarray(msg.position[:6], dtype=float)
        if not np.all(np.isfinite(values)):
            return
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if stamp <= 0.0:
            # 桥不一定填 header.stamp; 那就用本地时钟 (只用于「新不新」的判断)。
            stamp = self.get_clock().now().nanoseconds * 1e-9
        self._joint_deg = (stamp, np.degrees(values))

    def _on_arm_status(self, msg: H3001ArmStatus):
        # H3001ArmStatus 无 header, 用节点时钟记录接收时刻
        self._arm_status = (self.get_clock().now().nanoseconds * 1e-9, msg)

    def _on_grasp_confirm(self, msg: Bool):
        # 三个确认点共用一条话题: 在哪个状态收到就确认哪一件事
        # (TRACK_WAIT = 确认下降, GRASP_WAIT = 现在夹, PLACE_WAIT = 现在放)。
        # 后两个状态在 *_confirm_required=false 时**根本不看这个标志** (进状态第一拍就
        # 往下走), 所以那时发过来的消息是空转; 但它也只落在 _confirm_received 上,
        # 而每个确认状态进出时都会清这个标志, 不会串到下一个确认点。
        if msg.data and self._state in (
            self._STATE_TRACK_WAIT,
            self._STATE_GRASP_WAIT,
            self._STATE_PLACE_WAIT,
        ):
            self._confirm_received = True

    def _on_execute(self, _request, response):
        """~/execute_grasp: **循环模式的开关**, 同时保留「单发」用法。

        2026-09-21 改: 原来是一次任务一次触发, 想抓下一个必须人再调一次。现在
        **调一次 = 启动任务并开启循环** (机械臂回到待机位后自动接着抓下一个),
        **再调一次 = 关闭循环**。

        关闭**永远不打断正在跑的那一次任务** —— 机械臂手里可能夹着瓶子、可能正在
        下降, 中途拧断有掉瓶/撞压风险。关闭只在「回到待机位、准备下一次」那一刻生效,
        响应 message 会说清楚这一点。

        「抓完一个 → 回待机位 → 等下一个」之间**没有任何超时**: 视野里一直没有瓶子
        就一直在待机位等下去 (每 10s 一行「还在等」), 不自动收工。跟丢目标也一样 ——
        那是「这一次没抓到」, 不是机械臂出了故障。只有真失败 (固件拒收 / 无反馈 /
        夹爪失败 / 回位判定失败) 才会自动关掉循环, 见 _fail 的 keep_loop。

        为什么是开关而不是新加一个 service: 见 README §5.14。
        """
        if self._loop_armed:
            # 第二次调用 = 关闭。这一段必须在状态检查**之前**: 任务进行中也要能关,
            # 否则会先被「任务进行中, 拒绝新请求」挡回去, 现场就会以为关不掉。
            self._loop_armed = False
            if self._state == self._STATE_IDLE:
                response.message = "循环模式已关闭 (当前空闲, 不会再有下一次自动触发)"
            else:
                response.message = (
                    f"循环模式已关闭: 本次任务 ({self._state}) 照常跑完并回到待机位, "
                    "之后不再自动触发"
                )
            self.get_logger().warn(response.message)
            response.success = True
            return response

        reason = self._begin_task()
        if reason is not None:
            response.success = False
            response.message = reason
            return response

        # 循环的开启条件。place_enabled 是**硬闸**: 关掉放置时每一轮只是把瓶子在原地
        # 放下再抓起来 —— 同一个瓶子无限循环, 而循环模式成立的前提就是「放下去的东西
        # 离开了相机视野」。其余几条只警告不拦 (见 _loop_warnings)。
        if not self.get_parameter("place_enabled").value:
            self.get_logger().warn(
                "循环模式未开启: place_enabled=false —— 每一轮只会把瓶子在原地放下再"
                "抓起来 (同一个瓶子无限循环)。本次任务照常执行, 但只跑这一次"
            )
            response.success = True
            response.message = (
                "抓取任务已启动 (循环模式**未开启**: place_enabled=false, 原因见日志); "
                "把放置打开后重新调用本服务即可循环"
            )
            return response

        self._loop_armed = True
        for warning in self._loop_warnings():
            self.get_logger().warn(warning)
        response.success = True
        response.message = (
            "抓取任务已启动; 循环模式已开启 (回到待机位后等着抓下一个, 视野里一直"
            "没有目标也会一直等; 只有任务真失败或再调用一次本服务才关闭)"
        )
        return response

    def _loop_warnings(self):
        """循环开着时「不会顺手」的几件事, 开启那一刻提示一次。**只警告, 不拦**。

        都不是「跑起来就不是用户要的那件事」, 所以不设硬闸; 但每一条都会让循环
        变慢或者不在待机位起步, 事先说清楚比事后查日志便宜。
        """
        warnings = []
        if self.get_parameter("grasp_confirm_required").value:
            warnings.append(
                "grasp_confirm_required=true: 每一轮都会在 GRASP_WAIT 停下等 Enter / "
                "grasp_confirm —— 循环会变成「一次一确认」, 不是无人值守"
            )
        if self.get_parameter("place_confirm_required").value:
            warnings.append(
                "place_confirm_required=true: 每一轮都会在 PLACE_WAIT 停下等确认"
            )
        if (
            self.get_parameter("track_enabled").value
            and float(self.get_parameter("auto_grasp_stable_time").value) > 0.0
            and not self.get_parameter("grasp_confirm_required").value
        ):
            # 整个链上最后一个人工确认也被去掉了 —— 这是「无人值守」的代价, 不是
            # 配置错误, 所以只警告。落点准不准此时只剩 descend_dwell 那个盲等兜底。
            warnings.append(
                "自动下降已开 + 夹爪确认已关: 整条链**全程没有人工确认** —— 目标位置"
                "只要稳住就会自己往下抓, 落点准不准只剩 descend_dwell 的盲等兜底。"
                "第一次上真机先用 dry_run 看一眼日志与时长, 或把 auto_grasp_stable_time "
                "调大/设 0 退回人工"
            )
        if not self.get_parameter("return_to_standby_on_finish").value:
            warnings.append(
                "return_to_standby_on_finish=false: 循环不会从待机位起步, 而是从上一轮"
                "结束的位置 (抬升点/放置点) 直接去找下一个目标"
            )
        if not self._standby_ready:
            warnings.append(
                "待机位不可用 (加载期校验没过, 见启动日志): 循环不会回待机位, 同上一轮"
                "结束的位置起步"
            )
        return warnings

    def _begin_task(self):
        """启动一次抓取任务。成功返回 None;**被拒时返回原因串**(调用方装进响应)。

        ~/execute_grasp (人工触发) 与循环起手 (_begin_loop_cycle) 共用这一份 ——
        启动这件事只能有一处定义, 否则两条路迟早走岔。
        """
        if self._state != self._STATE_IDLE:
            if self._state == self._STATE_HOME:
                return "正在回待机位, 拒绝新请求"
            return f"任务进行中 ({self._state}), 拒绝新请求"
        if self.get_parameter("place_enabled").value and not self._place_ready:
            # 明确开了放置却填错坐标: 现在拒绝, 而不是抓完放到一半才失败 —— 那时手里
            # 已经捏着瓶子, 处置起来麻烦得多。原因在启动日志里 (WORKSPACE_REJECTED /
            # 需要 3 个有限数 / 离基座原点太近)。
            return (
                "place_enabled=true 但 place_pose_reported_mm_deg 无效 (见启动日志), 拒绝启动; "
                "修好坐标或把 place_enabled 置 false"
            )
        if self.get_parameter("gripper_enabled").value and not self._gripper_ready:
            # 同上: 明确开了夹爪却定位不到脚本 / 串口不在, 现在拒绝。
            return (
                "gripper_enabled=true 但夹爪不可用 (见启动日志), 拒绝启动; "
                "检查 gripper_port 与脚本路径, 或把 gripper_enabled 置 false"
            )
        if float(self.get_parameter("gripper_hold_torque").value) > 0.0 and not self._place_ready:
            # 保持开着却没有放置点: 整条链上**没有任何一步会去松开夹爪** ——
            # place_enabled=false 时 LIFT 结束直接落 IDLE (_tick_motion 的 next_state),
            # 不走 PLACE_WAIT, 于是保持进程会一直顶到 hold_timeout, 然后在**待机位上、
            # 没人看着的时候**自己失能松手, 瓶子掉下来。比今天「失能后靠机构自锁带回
            # 待机位」更糟 (时间不可预期、位置也不对)。所以现在拒绝, 而不是抓完才发作。
            return (
                "gripper_hold_torque>0 但 place_enabled=false / 放置点无效: 抬升之后"
                "没有任何一步会松开夹爪, 瓶子会在 hold_timeout 之后自己掉在待机位上; "
                "补好放置点或把 gripper_hold_torque 置 0"
            )
        # 本次任务允许回位一次。放在这里而不是 _enter_home 里, 是为了「一次任务
        # 最多回一次位」这件事有一个唯一、好审计的来源。
        self._return_pending = True
        # 上个任务期间收到的杂散 Enter / grasp_confirm 不能顺延到下一次 GRASP_WAIT
        # (回位这段时间更长了, 更可能被误按)。
        self._enter_pressed = False
        self._confirm_received = False
        # 上一轮遗留的 IK 判决同理不许顺延: 留着「待判」会让这一轮第一条指令在发出后
        # 的一瞬间被上一条的拒收判决误伤。
        self._reset_ik_watch()
        # 任务一起步就不再是「在待机位等目标」了, 把等待守卫清掉。放这里 (而不是
        # _begin_loop_cycle) 是因为人工触发那条路也可能从「等过目标的 IDLE」起步:
        # 留着一个旧的守卫时刻只会让**更旧**的候选混进来, 下一轮又是抓空位。
        self._loop_guard_at = None
        if self._gripper_ready:
            # 每轮开始前先张开: 保证这一轮从「确定张开」这个状态出发, 而不是从上一次
            # 任务留下的姿势 (上一次可能夹着东西, 也可能停在半路)。走的是同一个
            # GRIPPER 状态, 只是 stage 不同。
            self.get_logger().info("抓取任务已启动: 先张开夹爪")
            self._enter_gripper(self._GRIPPER_STAGE_OPEN)
        else:
            self._transition(self._STATE_WAIT_TARGET)
        return None

    def _tick_loop_wait(self):
        """循环模式: 回到 IDLE 之后**在待机位等一个新目标**, 等到了才开下一轮。

        由 _tick 在 IDLE + _loop_armed 时每拍调用。这一拍只做「等」, 不发任何指令,
        机械臂停在待机位不动 —— 这是 2026-09-21 用户要的语义: 抓完一个、或跟丢了目标
        回到待机位后, 继续接受相机识别到的坐标, 视野里暂时没有瓶子就一直等下去, 不
        自动收工。

        判据是**观察时刻**, 不是「有没有候选」:
          * 进 IDLE 的头一拍记下 _loop_guard_at = now;
          * 只有 received_at **晚于** _loop_guard_at 的候选才算真目标 —— 上一轮用过的
            那个候选 (瓶子早被搬到相机视野之外) 时间戳比它早, 一律不算; 顺手清掉,
            免得 _tick_wait_target 拿着旧坐标去抓空位 (它只看 _latest_candidate 在不在,
            不查年龄), 也免得它把「视野里一直没有新目标」这条判据顶掉;
          * 等的时候每 10s 打一行「还在等」, 现场能从日志上看出节点是活的、只是在等,
            而不是卡死了。
        """
        now = time.monotonic()
        if self._loop_guard_at is None:
            self._loop_guard_at = now
            self._loop_wait_log_at = now
            self.get_logger().info(
                "循环模式: 已回待机位, 在这里等相机的新目标 (机械臂不动, 不自动收工; "
                "想收工就再调一次 ~/execute_grasp)"
            )
        latest = self._latest_candidate
        if latest is None or latest["received_at"] <= self._loop_guard_at:
            if latest is not None:
                # 上一轮剩下的旧候选: 清掉, 别让它被 WAIT_TARGET 捡去抓空位。
                self._latest_candidate = None
            if now - self._loop_wait_log_at >= 10.0:
                self._loop_wait_log_at = now
                self.get_logger().info("循环模式: 还在等相机的新目标 ...")
            return
        self._begin_loop_cycle()

    def _begin_loop_cycle(self):
        """循环模式: 有新目标了, 在 IDLE 里开启下一轮。由 _tick_loop_wait 调用。

        两件事, 顺序有讲究:
          ① _reset_target_lock() —— 各条结束路径本来就都清过锁, 这里再清一次是为了让
             「循环起手」自包含, 不依赖上一轮是从哪条路走回 IDLE 的;
          ② _begin_task() —— 与人工触发同一份启动逻辑 (守卫时刻也在那里清)。

        这里**不动 _latest_candidate**: 上面那一拍刚确认它是回来之后新观察到的目标,
        清掉就成了「等到了又扔掉」, WAIT_TARGET 又要从头等一轮。
        (旧候选的清零改由 _tick_loop_wait 做, 判据是时间戳, 比「起手时一律清」更准。)

        启动被拒就关掉循环 + 报 ERROR: 那几条都是加载期就该校验出来的条件 (放置坐标
        无效 / 夹爪不可用), 原地重试没有意义。
        """
        self._reset_target_lock()
        self.get_logger().info("循环模式: 收到新目标, 开始下一轮")
        reason = self._begin_task()
        if reason is not None:
            self._loop_armed = False
            self.get_logger().error(
                f"循环模式已关闭: 启动下一轮被拒 ({reason}) —— 修好后重新调用 "
                "~/execute_grasp 开启"
            )

    def _open_confirm_stream(self):
        """返回 (可读流, 是否本函数打开的)。拿不到返回 (None, False)。

        ros2 launch 会把子进程的 stdin 接成管道 (fd0=pipe), 终端按键根本到不了
        节点, 表现为「GRASP_WAIT 时按 Enter 没反应」。所以 stdin 不是 tty 时优先
        直接打开控制终端 /dev/tty; 只有在没有控制终端时才退回 stdin (可能是个
        带数据的管道)。
        """
        stdin = sys.stdin
        if stdin is not None and stdin.isatty():
            return stdin, False
        try:
            return open("/dev/tty", "r"), True
        except OSError:
            pass
        if stdin is not None:
            return stdin, False
        return None, False

    def _stdin_reader(self):
        """后台读取终端 Enter, 在三个确认状态等效人工确认。

        用 select 轮询而不是阻塞 input(): 避免持有 stdin 缓冲锁,
        节点退出时可干净结束线程, 不会在解释器关闭时死锁。
        """
        stream, owned = self._open_confirm_stream()
        hint = (
            "Enter 确认不可用 (stdin 不是终端且打不开 /dev/tty, "
            "ros2 launch 会把子进程 stdin 接成管道)。"
            "到 TRACK_WAIT (确认下降) / GRASP_WAIT (现在夹) / "
            "PLACE_WAIT (现在放) 时请改用: "
            f"ros2 topic pub --once {self.get_parameter('grasp_confirm_topic').value} "
            "std_msgs/msg/Bool '{data: true}'"
        )
        if stream is None:
            self.get_logger().warn(hint)
            return
        try:
            is_tty = stream.isatty()
        except (OSError, ValueError):
            is_tty = False
        if not is_tty:
            self.get_logger().warn(hint)
        try:
            while not self._shutdown_event.is_set():
                try:
                    ready, _, _ = select.select([stream], [], [], 0.2)
                except (OSError, ValueError):
                    return
                if not ready:
                    continue
                line = stream.readline()
                if not line:
                    return  # EOF
                if not line.strip():
                    self._enter_pressed = True
        finally:
            if owned:
                stream.close()

    # ── 状态机 ────────────────────────────────────────────

    def _transition(self, state, waypoints=None, orientations=None):
        """切换状态, 并装载该阶段的路点队列。

        waypoints 为空表示该阶段是「非运动」状态 (IDLE / WAIT_TARGET / GRASP_WAIT)。
        orientations 只在「每个路点姿态都不同」时给 (目前只有抬升时顺带回正那条路,
        见 _begin_lift / _waypoint_orientations); 不给就是本阶段共用一个姿态。
        """
        if state != self._STATE_HOME:
            # 「盲走」标记只在 HOME 期间有效: _enter_home 是先置位再 _transition(HOME),
            # 所以这里只清「离开回位」的那一次。
            self._home_blind = False
            self._blind_started = None
            # 回位的到位超时倍数是还给拆段年代的时间预算, 当然也只属于回位 ——
            # 漏到后面的运动阶段会让那些阶段的超时莫名其妙地长 (最多 4×30s)。
            self._home_time_multiplier = 1
        if state != self._STATE_TRACK_WAIT:
            # 跟随状态同样只属于 TRACK_WAIT。离开时清掉 _track_command 有个必须的
            # 副作用: 下一次任务不会再拿上一次的悬停点当死区基准。
            # 进 TRACK_WAIT 时不清 —— _enter_track_wait 是先置 _track_command
            # 再 _transition, 那时它正是本状态要用的基准。
            self._track_command = None
            self._reset_track_stability()
        if state in (
            self._STATE_TRACK_WAIT,
            self._STATE_GRASP_WAIT,
            self._STATE_PLACE_WAIT,
        ):
            # 三个确认等待状态都从零开始收按键: 上一次确认(甚至上一次任务)留下的
            # Enter 不能顺延成这一次的确认 —— 现在要按三次, 更可能被误按。
            self._enter_pressed = False
            self._confirm_received = False
        self.get_logger().info(f"状态: {self._state} -> {state}")
        self._state = state
        self._state_enter_time = time.monotonic()
        self._waypoint_enter_time = self._state_enter_time
        self._goal_sent = False
        self._tolerance_start = None
        self._prompt_time = 0.0
        self._waypoints = list(waypoints) if waypoints else []
        self._waypoint_index = 0
        self._waypoint_orientations = list(orientations) if orientations else None
        self._reset_endpoint_correction()

    def _reset_endpoint_correction(self):
        """清空迭代末端修正。

        修正量是「针对某一条指令」的, 换了路点就不再有意义 —— 下垂量随姿态变化,
        把上一个路点的修正量带过去会变成过冲。所以每次换路点都从零重算。
        """
        self._cmd_offset = np.zeros(3)
        self._retry_count = 0
        self._settle_pose = None
        self._settle_since = None
        self._stall_reference = None
        self._stall_armed = False
        self._retry_exhausted = False

    def _arm_stall_detection(self):
        """记下发指令那一刻机械臂在哪; 停稳判定要等它**动过**才生效。

        没有这一下, _try_endpoint_correction 会把「指令刚发出、机械臂还没起步」读成
        「已经停稳但够不到」, 然后按残差重发 —— 而站在起点上算出来的残差就是整段位移,
        指令当场被加成两倍距离, 机械臂冲过目标。2026-09-18 真机日志里就是这么把
        24.4 cm 的横移算成 244 mm 修正量的; 那次是 max_correction_distance 挡下了,
        横移短于 8 cm 时挡不住。

        取不到 TCP 反馈时不置位 (reference 为 None): 后面对应的分支会直接放弃修正,
        宁可不动手, 不乱猜。
        """
        self._stall_reference = (
            None if self._tcp_pose is None else np.asarray(self._tcp_pose[1], dtype=float)
        )
        self._stall_armed = False
        # 「这条指令到现在还没起步」的提醒一次就够: 这里是每条新指令的唯一入口
        # (发送路径上都会调), 所以复位放在这儿。
        self._no_start_warned = False

    def _reset_track_stability(self):
        """清空悬停跟随的两条停稳检测 (每发一条跟随指令就重来一遍)。

        两个判据: 机械臂停稳 (`_track_arm_settled`) 与**物体坐标**稳定
        (`_track_object_settled`, 全自动下降用)。这里是它们**唯一**的清点 ——
        `_enter_track_wait` 与 `_transition` (离开 TRACK_WAIT) 都调它, 所以
        「上一轮/上一次悬停留下的窗口不许顺延」这件事不用在别处再补一遍。

        不碰 _track_last_command_time: 它记的是「上一条指令发出去的时刻」, 清成 0 会让
        限速门立刻打开, 正好抵消掉它要防的那件事 (两条目标重叠 -> 抖动)。
        """
        self._track_stable_pose = None
        self._track_stable_since = None
        self._auto_grasp_ref = None
        self._auto_grasp_since = None

    def _commanded_target(self, goal):
        """标称路点 + 迭代修正量 = 真正下发的位姿 (base_link, m)。"""
        return np.asarray(goal, dtype=float) + self._cmd_offset

    def _try_endpoint_correction(self, distance):
        """停稳但仍够不到目标时, 把残差加到指令上重发一次。

        返回 True 表示已经重发, 本轮不要再做别的判断。

        只被 PRE_GRASP / LIFT / HOME 用到: DESCEND 是一条指令且不看反馈, 不走这里。
        """
        now = time.monotonic()
        pose = np.asarray(self._tcp_pose[1], dtype=float)
        epsilon = float(self.get_parameter("stall_detect_epsilon").value)
        if not self._stall_armed:
            # 「停稳」的前提是「动过」—— 否则指令刚发出去、机械臂还在原地待命的那
            # 一拍就会被当成「停稳但够不到」, 而此刻的残差正好是整段位移, 重发出去
            # 就是两倍距离。见 _arm_stall_detection。
            if self._stall_reference is None:
                return False
            if float(np.linalg.norm(pose - self._stall_reference)) <= epsilon:
                return False
            # 第一次看见它真的动了: 停稳计时的零点定在**这里**, 不是指令发出那一刻。
            self._stall_armed = True
            self._settle_pose = pose
            self._settle_since = now
            return False
        if self._settle_pose is None or float(np.linalg.norm(pose - self._settle_pose)) > epsilon:
            # 还在动 (或刚重发完): 记住位置, 重新开始停稳计时
            self._settle_pose = pose
            self._settle_since = now
            return False
        if now - self._settle_since < float(self.get_parameter("stall_detect_time").value):
            return False
        if not self.get_parameter("endpoint_correction_enabled").value or self._retry_exhausted:
            return False

        limit = int(self.get_parameter("max_correction_retries").value)
        if self._retry_count >= limit:
            self._retry_exhausted = True
            self.get_logger().warn(
                f"{self._state} 路点 {self._waypoint_index + 1}/{len(self._waypoints)} "
                f"迭代 {limit} 次后仍差 {distance * 1000.0:.1f} mm, 累计修正 "
                f"{np.round(self._cmd_offset, 4)} m; 不再重发, 交给 motion_timeout 兜底"
            )
            return False

        residual = np.asarray(self._goal_position, dtype=float) - pose
        new_offset = self._cmd_offset + residual
        cap = float(self.get_parameter("max_correction_distance").value)
        if float(np.linalg.norm(new_offset)) > cap:
            # 修正量失控说明病根不是「够不到」而是「被挡住 / IK 无解」,
            # 继续加码只会把指令推得越来越离谱, 这里必须停下并明确报出来。
            self._retry_exhausted = True
            self.get_logger().warn(
                f"{self._state} 累计修正将达 {np.linalg.norm(new_offset) * 1000.0:.1f} mm, "
                f"超过上限 {cap * 1000.0:.0f} mm, 停止迭代 —— 病根多半不是末端下垂"
                f" (例如被挡住或 IK 无解), 请检查现场"
            )
            return False

        self._cmd_offset = new_offset
        self._retry_count += 1
        target = self._commanded_target(self._goal_position)
        self.get_logger().warn(
            f"{self._state} 路点 {self._waypoint_index + 1}/{len(self._waypoints)} "
            f"停稳后仍差 {distance * 1000.0:.1f} mm (实际 {np.round(pose, 4)}), "
            f"第 {self._retry_count}/{limit} 次迭代: 改发 {np.round(target, 4)} "
            f"(累计修正 {np.round(self._cmd_offset, 4)} m)"
        )
        self._send_move_p(target, self._sent_orientation)
        self._settle_pose = None
        self._settle_since = None
        return True

    def _reset_target_lock(self):
        self._locked_track_id = None
        self._target_history = []

    def _fail(self, reason, keep_loop=False):
        """任务失败: 记日志, 然后按 _should_return_to_standby() 决定去 HOME 还是 IDLE。

        这里刻意不改任何一个调用点: 「该不该回位」的判据全都集中到
        _should_return_to_standby() 里了 (开关/可用性/防重入/状态/机器人是否可控)。

        keep_loop=True **只给「视野里没有目标」这一类用** (等待目标超时 / 两条
        TRACK_TARGET_LOST): 那不是机械臂出了问题, 用户要的是「回到待机位继续接受相机
        识别到的坐标」(2026-09-21) —— 循环保持开启, 回待机位接着等, 等多久都行。
        其余全部走默认值 (循环关掉收工): 固件拒收 / 无反馈 / 夹爪失败 / 内部错误那些
        继续循环只会一遍遍重跑同一个坏流程 (2026-09-21 那几次固件 IK 拒收就是同一类
        系统性故障), 而且放置阶段的失败可能夹着瓶子, 不能自动接着干。
        """
        self.get_logger().error(f"任务失败: {reason} (状态 {self._state})")
        if self._loop_armed:
            # 循环模式的开关只在这里 (_fail) 和 _tick_blind_home 两处被自动关掉,
            # 加上人工再调一次服务 —— 所以这两条日志必须说清「循环怎么了、怎么再开」,
            # 否则现场看到的是一串 ERROR, 不知道该不该继续等。
            if keep_loop:
                self.get_logger().info(
                    "循环模式保持开启: 这一次只是没等到目标, 回待机位后继续等新坐标 "
                    "(想收工就再调一次 ~/execute_grasp)"
                )
            else:
                self._loop_armed = False
                self.get_logger().info(
                    "循环模式已自动关闭 (这一次任务失败); 处理完后再调一次 "
                    "~/execute_grasp 可重新开启"
                )
        self._reset_target_lock()
        if self._state == self._STATE_HOME:
            self.get_logger().error(
                "回位未完成, 机械臂停在非待机位; 状态回 IDLE, 请人工确认位置后再触发"
            )
        if self._should_return_to_standby():
            self._enter_home(f"任务失败: {reason}")
            return
        self._transition(self._STATE_IDLE)

    def _finish_task(self, reason):
        """任务**正常**结束 (抓取完成 / 放置完成): 按 _should_return_to_standby() 决定回位还是停在原地。

        与 _fail 用同一套判据 (开关/可用性/防重入/机器人是否可控), 不另立一套 ——
        区别只在日志级别与文案: 这不是失败。

        注意 _should_return_to_standby 里那道 _standby_ready 闸门必须走: 直接调
        _enter_home 会拿 None 当待机位用。
        """
        if self._should_return_to_standby():
            self._enter_home(reason)
            return
        self.get_logger().info(f"{reason} (未回位: 回位开关关闭或待机位不可用)")
        self._reset_target_lock()
        self._transition(self._STATE_IDLE)

    def _should_return_to_standby(self):
        """本次结束是否要回待机位。所有「不回位」的理由都集中在这里。

        _fail 会从 _tick_motion 内部被调用 (超时/距离上限/机器人故障), 所以这里必须
        把「已经在回位」这件事挡住, 否则回位失败会再进一次 HOME, 无限循环。
        """
        if not self.get_parameter("return_to_standby_on_finish").value:
            return False          # 开关关闭
        if not self._standby_ready:
            return False          # 加载期校验没过 (已 error), 姿态/位置不可用
        if not self._return_pending:
            return False          # 本次任务已经回过位 (也是防重入的最后一道闸)
        if self._state not in (
            self._STATE_WAIT_TARGET,
            self._STATE_PRE_GRASP,
            self._STATE_TRACK_WAIT,
            self._STATE_DESCEND,
            self._STATE_GRASP_WAIT,
            self._STATE_LIFT,
            self._STATE_PLACE_MOVE,
            self._STATE_PLACE_WAIT,
            # 夹爪步骤也算「任务进行中」: 在 GRIPPER 里失败 (闭合/张开超时, 或脚本
            # 异常退出 = 命令根本没做) 与别处失败一样要回待机位。
            self._STATE_GRIPPER,
        ):
            return False          # IDLE / HOME: 没有任务在跑, 不回位
        if not self.get_parameter("dry_run").value and self._feedback_problem() is not None:
            # 机械臂当前不可控 (报故障 / 反馈断了)。默认**仍然回位** —— 回位是一条绝对
            # 位姿指令, 不依赖反馈; 把机械臂丢在抓取点上方才是更糟的结果。
            # 这种回位走盲走路径 (见 _enter_home / _tick_blind_home)。
            # return_to_standby_on_error=false 时退回旧行为: 不可控就不回位。
            if not self.get_parameter("return_to_standby_on_error").value:
                return False
        return True

    def _current_gripper_position(self):
        """回位起点: 真机用最新反馈; dry_run 或反馈缺失时退回最后下发的路点。"""
        if not self.get_parameter("dry_run").value and self._tcp_pose is not None:
            return np.asarray(self._tcp_pose[1], dtype=np.float64)
        return self._last_command_position

    def _home_time_multiplier_for(self, distance):
        """回位到位超时要放大的倍数 —— 还给**拆段年代**的时间预算。

        拆段已经删掉了 (2026-09-21, 见 _enter_home), 这个倍数只剩还账这一件事:
        老代码把 start->待机位 拉成直线, 超过 max_command_distance 一半就插中间点,
        **每条路点各给一个 motion_timeout**; 对角 1.3 m 的回位是 4 条路点 = 4×30 s。
        合并成一条绝对指令后只剩 30 s, 远距离回位会被误判成「没收敛」。
        所以这里照旧算那个段数 (算式逐字照抄, 见下), 只把它当**倍数**用 ——
        它不参与任何可行性判断, 只是「这条指令该给多久」。

        ⚠ **那条被删掉的推理是错的, 别再照着它把回位拆回去**:
        老 docstring 写「workspace_* 是凸的盒子 -> 中间路点不会越界」。这对
        **节点自己的工作空间检查**成立, 对**固件 IK 的可达域**不成立 ——
        固定姿态下可达域根本不是凸的。2026-09-21 真机回位就死在这上面: 起点与待机位
        两个端点各自都好好的, 中间点 [0.1348 -0.1889 0.2147] (上报系) 配上待机姿态
        固件解不出来 (ik_status=2 超出工作空间), 而节点自己的凸盒子检查一路放行。
        当天实测: 单发**一条**绝对待机位 (353 mm) 固件收下并规划到位, 误差 0.5 mm ——
        路径交给固件, 比节点在直线上猜中点好。

        上限截断到 4: 别的参数不该把安全超时放大到无界 —— max_command_distance 被调到
        0.05 m 时这个算式会给出 65 倍 (32 分钟), 那不是「给足时间」, 那是把「机械臂整个
        卡住」伪装成正常等待。真超了就 WARN, 让人自己判断。
        """
        max_distance = float(self.get_parameter("max_command_distance").value)
        # 预算里扣掉 2|offset|: 老代码在夹爪系插值而在上报系比较, 两系的 R @ offset
        # 不同, 最大差这么多 (本机 0.16 m)。地板 0.02 m 防除零/除出无穷多段。
        offset_span = 2.0 * float(np.linalg.norm(self._gripper_offset))
        budget = max(max_distance - offset_span, 0.02)
        count = int(math.ceil(distance / budget - 1e-9))
        if count > 4:
            self.get_logger().warn(
                f"回位里程 {distance:.2f} m 按老算式要 {count} 倍到位超时, "
                f"按上限 4 计 (= {4.0 * float(self.get_parameter('motion_timeout').value):.0f}s) "
                f"—— 超出部分不给 (max_command_distance={max_distance:.2f} m 偏小?)"
            )
            return 4
        return max(count, 1)

    def _enter_home(self, reason):
        """进入 HOME: 只发**一条绝对待机位**, 路径交给固件规划, 到位判定交给 _tick_motion。

        2026-09-21 改 (原来是「把 start->待机位 拉成直线, 超过一半上限就插中间点」):
        真机回位死在中间点上 —— 那个点配上**待机姿态**固件 IK 解不出来 (ik_status=2),
        而两个端点各自都好好的, 原地重发同样的数必然还是被拒。凸的 workspace_* 盒子
        证明不了固件 IK 的可达域, 详见 _home_time_multiplier_for 的 docstring。

        所以现在与**放置段**(2026-09-20: 用户要求「运动规划全靠机械臂本身」)和
        **盲走回位**(下面 problem 那一支)是同一套做法: 一条绝对位姿, 路径由固件规划。
        目标永远是加载期校验过的待机位, 且姿态恒为 _standby_orientation, 于是
        _gripper_to_reported(standby_position, standby_orientation) 逐位等于
        _load_standby_pose 校验过的那个 reported —— 不需要运行期工作空间复检。
        _home_blind 现在只表示「判不判到位」, 不再隐含「拆不拆段」(两条路都已是一条)。

        _return_pending 必须在「任何可能 _fail 的调用之前」清掉 —— 本函数之后
        唯一还会 _fail 的地方是 _tick_motion 的 _check_distance 与超时, 那时状态
        已经是 HOME, 被 _should_return_to_standby 挡住, 不会重进。
        """
        self._return_pending = False
        problem = None if self.get_parameter("dry_run").value else self._feedback_problem()
        self._home_blind = problem is not None
        self._blind_started = None
        start = self._current_gripper_position()
        # 起点只用来算时间预算与打日志: 下发的永远是待机位本身 (绝对指令, 与起点无关),
        # 所以反馈离谱也不会把机械臂支到别处去 —— 它影响的是到位判定与迭代修正,
        # 而后者另有 max_correction_distance 与 _stall_armed 两道限制。
        if problem is None and start is not None:
            distance = float(
                np.linalg.norm(
                    np.asarray(self._standby_position, dtype=np.float64)
                    - np.asarray(start, dtype=np.float64)
                )
            )
            self._home_time_multiplier = self._home_time_multiplier_for(distance)
        else:
            # 盲走不判到位 (走 blind_home_dwell), 这个倍数用不上; dry_run/无反馈时
            # 起点可能只是「最后下发过的地方」, 拿它算里程也没意义。
            distance = None
            self._home_time_multiplier = 1
        beyond = ""
        if distance is not None and self._tcp_orientation is not None:
            # 与 _check_distance 同系比较 (上报系才是机械臂真正要走的位移)。
            # 这里只是**提示**: 上限对回位豁免了, 一句 WARN 换掉原来那层「反馈离谱」的
            # 兜底 —— 拆段当年并不限制总行程 (路点全在直线上, 段长之和恒等于总位移),
            # 所以豁免它没丢真正的安全网, 但它确实让人少一眼「这趟要走多远」。
            max_distance = float(self.get_parameter("max_command_distance").value)
            reported_distance = float(
                np.linalg.norm(
                    self._gripper_to_reported(self._standby_position, self._standby_orientation)
                    - self._gripper_to_reported(start, self._tcp_orientation)
                )
            )
            if reported_distance > max_distance:
                beyond = f" (超过单条指令上限 {max_distance * 100.0:.0f} cm, 回位段豁免)"
                self.get_logger().warn(
                    f"回位直线 {reported_distance * 100.0:.1f} cm 超过单条指令上限 "
                    f"{max_distance * 100.0:.0f} cm: 回位段豁免这个上限 (目标是加载期校验过的"
                    f" yaml 常量、不是感知, 和放置段同一个理由 —— 那个上限防的是感知坐标"
                    f"跳变), 整段一条 move_p 交给固件规划。兜底因此少了一层, 只剩「起点来自"
                    f"反馈」这一个旁证: 上面那行的起点值看着不对, 先人工确认机械臂真实位置"
                )
        self.get_logger().info(
            f"回待机位 ({reason}): 一条绝对指令, 夹爪目标 {np.round(self._standby_position, 4)} m, "
            f"起点 {np.round(start, 4) if start is not None else '未知'}"
            f"{f', 直线 {distance * 100.0:.1f} cm' if distance is not None else ''}"
            f"{beyond}; 路径由固件规划"
        )
        if problem is not None:
            self.get_logger().warn(
                f"机械臂当前不可控 ({problem}), 仍按要求回位 —— **盲走**: 只发一次绝对"
                f"待机位, 不判到位 (反馈恢复自动接管, 否则 "
                f"{self.get_parameter('blind_home_dwell').value}s 后结束并报「未能确认」)"
            )
        self._transition(self._STATE_HOME, [self._standby_position])

    def _enter_place(self):
        """进入放置: LIFT 结束后把物体**一条 move_p 送到**放置位姿本身。

        2026-09-18 改 (用户要求「直接发送 place_pose_reported_mm_deg 这个坐标让机械臂
        到达」): 原来这里是「先平移到放置点正上方 (x,y 用放置点、z 用抬升后的实际高度),
        再垂直下降到放置高度」两段, 中间那个点(放置点正上方)**用户从来没要求过** ——
        真机上 IK 拒收的恰恰是它 (status=2, 见 config/*.yaml 里那段注释)。

        2026-09-20 又去掉了**拆段** (用户要求「运动规划全靠机械臂本身」): 这里原来调
        _build_segmented_waypoints 按 max_command_distance 在直线上插中间点, 于是
        「一条 move_p 到 place_pose」在距离超过预算时变成两三条 (该函数 2026-09-21
        已随回位一起删除, 见 _home_time_multiplier_for 的 docstring)。现在整段就是
        **一条指令、一个目标**: 参数里那个位姿本身, 路径怎么走由固件规划 (MoveMode.P
        点到点, 桥接那边发的就是它)。为此 _tick_motion 对 PLACE_MOVE 关掉了
        _check_distance 的上限 —— 那个上限防的是感知跳变, 而放置点是 yaml 常量。

        四个刻意的选择:
          - **姿态用参数里的那一个** (_place_orientation, 加载期算好), 全程用它。
            2026-09-18 之前用的是「进这里时冻结的实际 TCP 姿态」, 于是放置姿态被抓取姿态
            绑死 —— 抓取挑的是侧抓, 那个腕部朝向在放置点未必解得出 IK (真机上就出现过
            PLACE_MOVE 原地不动)。现在位置和姿态都是用户在 yaml 里指定的完整位姿, 与
            抓取段彻底解耦。
            顺带也解决了「每段重新读 _tcp_orientation 会把腕部重力下垂读成新姿态」的
            老问题: 常量姿态压根不会再变。
          - **不拆段**: 见上。工作空间检查仍然做, 而且是在**发指令之前**做。只查终点
            就够 —— workspace_* 是凸的盒子, 起点 (抬升结束的位置) 和终点都在界内时,
            这条直线上的每个点都在界内, 不需要沿路逐点查。
          - **判到位**: 这一段是自由空间运动, 终点是用户指定的释放点, 「到没到」有明确
            答案, 所以走 _tick_motion 的到位判据 + 迭代末端修正 (与 PRE_GRASP / LIFT /
            HOME 同一套)。旧的垂直下降那一段**不判** —— 物体落在台面上以后机械臂就下不去
            了, 判它只会持续往下压; 现在没有那一段了。
          - 起点取不到时退回 _lift_target: 只可能在 LIFT 刚结束时进来, 它必然已设。
        """
        if not self._place_ready:
            # 防御: 正常情况下进不来 (LIFT 只在 _place_ready 时才转过来)。计时器回调里
            # 抛异常会直接拖垮节点, 所以宁可判失败。
            self._fail("放置点未配置或不可用")
            return
        orientation = self._place_orientation
        if orientation is None:
            # 同上: _place_ready 为真时它必然已设, 这里只是防御。
            self._fail("放置阶段没有可用姿态")
            return
        start = self._current_gripper_position()
        if start is None:
            start = self._lift_target
        if start is None:
            self._fail("放置阶段没有可用起点")
            return
        start = np.asarray(start, dtype=np.float64)
        place = np.asarray(self._place_position, dtype=np.float64)
        # **一个路点 = 一条 move_p** (2026-09-20): 不再按 max_command_distance 插中间点。
        waypoints = [place]
        if not self._check_workspace_pair(place, orientation):
            return  # 内部已 _fail (此刻状态还是 LIFT, 在回位白名单里)
        distance = float(np.linalg.norm(place - start))
        # 这一整段还要转腕多少度 (抓取姿态 -> 放置姿态): 这是「机械臂解不解得出」
        # 的第一嫌疑, 原地不动的现场日志里就靠这一行判断。取不到当前姿态时打 None。
        current = self._lift_orientation()
        turn = self._orientation_angle(current, orientation)
        turn_text = "未知 (无当前姿态)" if turn is None else f"{math.degrees(turn):.1f}°"
        max_distance = float(self.get_parameter("max_command_distance").value)
        beyond = " (超过单条指令上限 %.2f m, 放置段豁免)" % max_distance if distance > max_distance else ""
        self.get_logger().info(
            f"移到放置点 (一条 move_p, 无中间点): 起点(夹爪)={np.round(start, 4)} m -> "
            f"放置点(夹爪)={np.round(place, 4)} m, 直线 {distance * 100.0:.1f} cm{beyond}, "
            f"全程同时转腕 {turn_text}; 路径由固件规划"
        )
        if turn is not None and math.degrees(turn) > 90.0:
            # 不是失败: 只是姿态差得远, 多半是 place_pose_reported_mm_deg 的 RX/RY/RZ
            # 抄错了一行。真转过去也合法, 所以只提醒。
            self.get_logger().warn(
                f"放置姿态与当前 TCP 姿态差 {math.degrees(turn):.1f}°, 这一段要同时转 "
                "大半个腕 —— 确认 place_pose_reported_mm_deg 的 RX/RY/RZ 是否抄错"
            )
        self._transition(self._STATE_PLACE_MOVE, waypoints)

    def _blind_home_fallback(self, problem):
        """回位途中机械臂变得不可控: 不半途丢下, 改盲走接着回 (幂等)。

        只在「正常进 HOME, 中途 bridge 挂掉 / 报故障」时走到这里。此时这个路点的到位
        判定全靠反馈, 而反馈已经没了, 再等下去只会 motion_timeout, 所以把判定改成
        「等满 blind_home_dwell 就收工」。

        注意它现在**只剩这一件事**: 队列清成 `[待机位]` 这步在 2026-09-21 之后是空操作
        (正常进 HOME 时队列本来就只有待机位一条, 见 _enter_home), 但保留着 —— 一是
        幂等无害, 二是万一以后 _enter_home 又给队列装多条, 这里就是那道「改发一条绝对
        指令」的兜底。改这条时别把它删掉。
        """
        if self._home_blind:
            return
        self.get_logger().warn(
            f"回位途中机械臂不可控 ({problem}), 转盲走: 改为直接下发一次绝对待机位"
        )
        self._home_blind = True
        self._blind_started = None
        self._waypoints = [self._standby_position]
        self._waypoint_index = 0
        self._waypoint_orientations = None
        self._goal_sent = False
        self._tolerance_start = None
        self._reset_endpoint_correction()

    def _tick_blind_home(self):
        """盲走回位的一次 tick: 只负责「绝对待机位指令发出去了」。

        反馈一旦恢复就交回正常判据 —— 此时 _goal_position 已经是待机位, 直接续上,
        所以恢复后仍会走到「已回到待机位」那句。一直不恢复则等满 blind_home_dwell
        就结束, 并明确报「没能确认到位」, 不假装成功。
        """
        if self._feedback_problem() is None:
            self.get_logger().info("回位反馈已恢复, 改回正常到位判定")
            self._home_blind = False
            self._blind_started = None
            self._waypoint_enter_time = time.monotonic()
            self._tolerance_start = None
            return
        if not self._goal_sent:
            orientation = self._active_orientation()
            self.get_logger().warn(
                f"盲走回位: 下发绝对待机位 夹爪={np.round(self._standby_position, 4)} m, "
                f"上报原点="
                f"{np.round(self._gripper_to_reported(self._standby_position, orientation), 4)} m"
            )
            self._send_move_p(self._standby_position, orientation)
            self._goal_position = self._standby_position
            self._sent_orientation = orientation
            self._goal_sent = True
            self._blind_started = time.monotonic()
            return
        started = self._blind_started if self._blind_started is not None else time.monotonic()
        waited = time.monotonic() - started
        if waited < float(self.get_parameter("blind_home_dwell").value):
            return
        self.get_logger().warn(
            f"盲走回位结束: 已下发待机位指令并等待 {waited:.1f}s, 反馈始终未恢复, "
            f"**无法确认是否到位** —— 机械臂可能仍在运动, 请人工确认后再触发"
        )
        if self._loop_armed:
            # 「回位判不到位」是异常收尾: 位置本来就不确定, 不能接着自动抓下一个。
            self._loop_armed = False
            self.get_logger().warn("循环模式已关闭 (回位没能确认到位)")
        self._reset_target_lock()
        self._transition(self._STATE_IDLE)

    def _tick(self):
        if self._candidates_sub is None:
            self._poll_candidates_file()
        # 夹爪子进程的 stdout **每拍都要收, 与状态无关**: 闭合之后那个进程可能一直
        # 活着保持力矩 (gripper_hold_torque > 0), 而状态机早就推进到 LIFT/PLACE_MOVE
        # 了 —— 那边没人读它的 stdout。管道 64KiB, 按 100Hz 打日志十几秒就满, 一满
        # 子进程的 print 就阻塞, 保持帧跟着断, 固件看门狗切电, **瓶子在搬运途中掉**
        # (正是这个功能要防的事故)。read_lines 非阻塞、每拍最多读 4KiB, 放最前面。
        if self._gripper_job is not None:
            for line in self._gripper_job.read_lines():
                self._note_gripper_line(line)
        # 保持进程死了 = 夹持力没了, 东西可能正在往下掉 —— 这条比其它判决都急,
        # 和 IK 拒收一样放在状态机之前收口。IDLE 下不判 (同 IK 那条的理由)。
        if self._state != self._STATE_IDLE:
            problem = self._gripper_hold_problem()
            if problem is not None:
                self._fail(problem)
                return
        # 求逆解的自检: 只在 IDLE 跑 (那时机械臂静止, 两条反馈必然同源), 1s 一次。
        # 它自己内部按需降级, 不判失败 —— 配置没对齐是「换个下发方式」的事, 不是任务失败。
        self._ik_selfcheck()
        # 固件 IK 拒收: 一个与状态无关的快速失败通道, 必须在状态机之前收口 ——
        # DESCEND / TRACK_WAIT 这些「不看反馈判到位」的状态同样会发运动指令, 机械臂
        # 一动不动时它们原本连个运动超时都没有, 拒收彻底没有痕迹 (README 里
        # 「下降时完全没动却不报错」那一行)。IDLE 下不判: 那时没有任务在跑,
        # 手里的旧判决不该把节点推进 HOME。
        if self._state != self._STATE_IDLE:
            problem = self._ik_rejection_problem()
            if problem is not None:
                # 先原样重发一次 (判决是逐次的, 同样的数常常第二次就被收下, 见
                # _retry_rejected_command 里的真机证据)。重发这一拍**不做别的判断** ——
                # 与 _try_endpoint_correction 同一条约定; 新的案子已经由 _send_move_p
                # 重新立好, 下一次拒收才会走到 _fail (那时额度已用完)。
                if not self._retry_rejected_command():
                    self._fail(problem)
                    return
                return
        # ── 循环模式: 已回待机位 → 在这里等相机的新目标, 等到了才接下一轮 ──
        # 放在 handler 派发**之前**是故意的: HOME 那一拍结束时状态还是 HOME (回位判定
        # 在 _tick_motion 里做), 所以不会同一拍抢着起步 —— 下一拍 (50ms 后) 才起手,
        # 那时机械臂已经在待机位站稳。
        # 「循环要不要关」不在这里管: 真失败由 _fail 关掉 (keep_loop=False), 回位判不
        # 到位由 _tick_blind_home 关掉; 这里只负责在还开着的时候等目标、接下一轮。
        if self._state == self._STATE_IDLE and self._loop_armed:
            self._tick_loop_wait()
            return
        handler = {
            self._STATE_WAIT_TARGET: self._tick_wait_target,
            self._STATE_PRE_GRASP: self._tick_motion,
            self._STATE_TRACK_WAIT: self._tick_track_wait,
            self._STATE_DESCEND: self._tick_descend,
            self._STATE_GRASP_WAIT: self._tick_grasp_wait,
            self._STATE_LIFT: self._tick_motion,
            self._STATE_PLACE_MOVE: self._tick_motion,
            self._STATE_PLACE_WAIT: self._tick_place_wait,
            self._STATE_GRIPPER: self._tick_gripper,
            self._STATE_HOME: self._tick_motion,
        }.get(self._state)
        if handler is not None:
            handler()

    def _tick_wait_target(self):
        latest = self._latest_candidate
        if latest is None:
            if time.monotonic() - self._state_enter_time > 30.0:
                # keep_loop=True: 「视野里没有目标」不是机械臂的故障。循环模式开着时
                # 回待机位接着等 (见 _tick_loop_wait), 单发模式下 _loop_armed 本来就是
                # 假, 这条 keep_loop 不起作用, 行为与以前逐字相同。
                self._fail("等待目标超时 (30s 无有效候选)", keep_loop=True)
            return

        # 锁定 track_id
        if self._locked_track_id is None:
            self._locked_track_id = latest["track_id"]
            self._target_history = []
            self.get_logger().info(f"锁定目标 track_id={self._locked_track_id}")

        if latest["track_id"] != self._locked_track_id:
            return

        position = latest["position_base"]
        self._target_history.append((time.monotonic(), position))
        # 只保留稳定窗口内的样本
        window = self.get_parameter("stable_duration").value
        self._target_history = [
            item for item in self._target_history if time.monotonic() - item[0] <= window
        ]
        if len(self._target_history) < self.get_parameter("min_stable_samples").value:
            return

        deviation = max(
            float(np.linalg.norm(position - item[1]))
            for item in self._target_history
        )
        tolerance = self.get_parameter("target_stability_tolerance").value
        if deviation > tolerance:
            return

        # 目标稳定: 计算抓取位姿
        self._grasp_center_base = position
        self._grasp_mode = latest["grasp_mode"]

        orientation = latest.get("orientation_base")
        if orientation is None:
            orientation = self._resolve_orientation()
        if orientation is None:
            return  # _resolve_orientation 内部已 _fail

        # position 就是抓取点, 也就是「夹爪闭合中心该落在的地方」, 因此路点全部以
        # **夹爪系** 表示, 与 _tcp_pose 同系; 上报末端原点的换算只发生在
        # _on_tcp_pose / _send_move_p 两个边界, 到位判定与残差都不用再管偏移。
        offset_base = self._gripper_offset_in_base(orientation)
        pre_offset = self._pre_grasp_offset_z()
        grasp_offset = self.get_parameter("grasp_offset_z").value
        pre_grasp = position + np.asarray((0.0, 0.0, pre_offset))
        final_grasp = position + np.asarray((0.0, 0.0, grasp_offset))

        for target in (pre_grasp, final_grasp):
            if not self._check_workspace_pair(target, orientation):
                return  # 内部已 _fail

        self.get_logger().info(
            f"目标稳定 (偏差 {deviation * 1000.0:.1f} mm), mode={self._grasp_mode}, "
            f"抓取点(夹爪, base)={np.round(position, 4)} m, "
            f"夹爪偏移={np.round(offset_base, 4)} m, "
            f"上报原点={np.round(self._gripper_to_reported(position, orientation), 4)} m, "
            f"预抓取(夹爪)={np.round(pre_grasp, 4)} m, "
            f"下降 {abs(pre_offset - grasp_offset) * 100.0:.1f}cm 到抓取点(夹爪)"
            f"={np.round(final_grasp, 4)} m"
        )
        self._pre_grasp_target = pre_grasp
        self._final_grasp_target = final_grasp
        self._goal_orientation = orientation
        self._transition(self._STATE_PRE_GRASP, [pre_grasp])

    def _tick_motion(self):
        """PRE_GRASP / LIFT / HOME / PLACE_MOVE 的心跳: 下发路点, 然后**靠反馈判到位**。

        这四个阶段**都走 MOVE_P** (点到点), 只要求终点 —— 唯一的线性段是 DESCEND,
        它不走这里。走 L 还是走 P 由 _send_move_p 按状态查 _LINEAR_MOTION_STATES 决定,
        这里不用写。LIFT 2026-09-28 之前走 L, 被固件拒收后退回 P, 理由见那张名单上的注释。

        DESCEND 不走这里 —— 它只有一条指令且不看反馈, 见 _tick_descend。
        """
        dry_run = self.get_parameter("dry_run").value
        # 每个运动阶段的安全前置检查 (dry_run 不依赖机械臂反馈)
        if not dry_run:
            problem = self._feedback_problem()
            if problem is not None:
                if self._state == self._STATE_HOME:
                    # 回位途中才不可控: 不能把机械臂半路丢下, 转盲走接着回。
                    # 这里刻意不走 _fail —— 那正是 2026-09-16 16:38 「回位未完成,
                    # 机械臂停在非待机位」的来路。
                    self._blind_home_fallback(problem)
                else:
                    self._fail(problem)
                    return

        if self._state == self._STATE_PRE_GRASP:
            # 悬停到位后默认进 TRACK_WAIT (跟随 + 等确认)。两条路跳过它, 都直接进
            # DESCEND (下降目标已由 _tick_wait_target 算好):
            #   track_enabled=false —— 整段跟随都不要 (旧的两阶段流程);
            #   倒瓶 (top) + top_skip_track —— 那一模式相机看不见瓶子, 跟了必判跟丢,
            #   见 _skip_track_for_mode。
            # 这里**不打日志**: 本函数在 PRE_GRASP 期间每拍都跑 (50ms), 判决只在
            # _advance_after_motion 真正转状态那一刻才写 (与"判决只算一次"无关 ——
            # _skip_track_for_mode 是纯谓词, 连调两次结论必然相同)。
            next_state = (
                self._STATE_TRACK_WAIT
                if self._track_wanted()
                else self._STATE_DESCEND
            )
        elif self._state == self._STATE_PLACE_MOVE:
            # 位置与姿态都到位 -> 松开物体。2026-09-18 之前这里还有一段
            # PLACE_DESCEND (从正上方盲降), 现在整段只剩这一条直达的路。
            next_state = self._STATE_PLACE_WAIT
        elif self._state == self._STATE_LIFT and self._place_ready:
            # 抬升到位且配了放置点 -> 去放置; 没配就落到 IDLE 那一支 (走 _finish_task)。
            next_state = self._STATE_PLACE_MOVE
        else:  # LIFT (未配放置) / HOME
            next_state = self._STATE_IDLE

        if not self._waypoints:
            # 防御: 计时器回调里抛异常会直接拖垮节点, 宁可判失败回 IDLE
            self._fail(f"{self._state} 没有可用路点")
            return
        goal = self._waypoints[self._waypoint_index]

        if self._home_blind:
            # 盲走回位: 下面那套全是「靠反馈判到位」, 此刻没有可信反馈可用。
            self._tick_blind_home()
            return

        if not self._goal_sent:
            # 真正下发的是「标称路点 + 迭代修正量」; 修正量为零时就是标称路点本身。
            command = self._commanded_target(goal)
            orientation = self._active_orientation()
            # 距离上限针对机械臂真正要走的位移, 所以在 **上报系** 里比较。
            # **放置段 (2026-09-20) 与回位 (2026-09-21) 都豁免这个上限**: 那个上限防的
            # 是「感知给的坐标突然跳一大截」, 而这两个目标都是 yaml 里的常量、不是感知,
            # 距离远是正常的 —— 它们都改成「一条 move_p 让机械臂自己规划整段」, 不再由
            # 节点在直线上插中间点。工作空间检查在 _enter_place / _load_standby_pose 里
            # 已经做过, 不靠这个兜底。
            # 注意策略在**这个调用点**, 不在 _check_distance 里: 直接调 _check_distance
            # 永远还是会拦 (test_place.py 里那条用例钉的就是这里)。
            if not dry_run and not self._check_distance(
                self._gripper_to_reported(command, orientation),
                enforce_max=self._state not in (self._STATE_PLACE_MOVE, self._STATE_HOME),
            ):
                return
            self._send_move_p(command, orientation)
            # 停稳检测的哨兵: 必须在这一刻取 (指令刚出去, 机械臂刚要起步), 而且**只在
            # 每段的第一条指令上取** —— 迭代重发不走这里, 否则「重发后没动」会被当成
            # 「还没起步」, 迭代就永远卡在原地。
            self._arm_stall_detection()
            self._goal_sent = True
            # 记的是标称路点: 到位判定、日志、残差计算全部以它为准, 修正量单独存。
            # 与 _tcp_pose 同为夹爪系, 两者相减才有意义。
            self._goal_position = goal
            # 迭代重发要用同一个姿态, LIFT 阶段用的是当前 TCP 姿态而不是 _goal_orientation。
            self._sent_orientation = orientation
            return

        if dry_run:
            # dry_run: 无反馈可监控, 停留 settle_time 后直接推进
            if time.monotonic() - self._tolerance_start >= self._active_settle_time():
                self.get_logger().info(
                    f"{self._state} [dry_run] 模拟到位 "
                    f"({self._waypoint_index + 1}/{len(self._waypoints)})"
                )
                self._advance_waypoint(next_state)
            return

        # 监控到位
        elapsed = time.monotonic() - self._waypoint_enter_time
        limit = self._active_motion_timeout()
        if not self._stall_armed and not self._no_start_warned:
            # 「指令发出去了, 机械臂一步没动」以前有个天然的报信人: 30s 的超时。回位
            # 的预算放大到最多 4×30s 之后, 这个盲区跟着变长, 而它**没有别的通道**:
            # IK 拒收要非 0 判决 (固件收了但没执行就不会有), 迭代修正要先动过一小步才会
            # 武装。所以在「早该起步了」的时候自己喊一声, 只喊一次 —— 不是判失败, 只是
            # 让人别干等到超时才去查。
            if elapsed > float(self.get_parameter("stall_detect_time").value) + 1.0:
                self._no_start_warned = True
                # 走 J 的那一段**没有「固件拒收」这条快速通道** —— 固件对 J 模式不跑
                # 逆解, ik_status 不会有可信的判决 (那条通道对 J 只会误判, 所以
                # _send_move_p 里 J 指令干脆不立案)。「没动」在这里只能靠这个提示 +
                # 最终的 motion_timeout, 别去翻 ik_status。
                hint = (
                    "这一条是 move_j —— 固件对 J 模式不跑逆解, ik_status 上不会有判决, "
                    "只看目标关节角对不对 (上一行)、以及关节反馈有没有动"
                    if self._last_command_joint is not None
                    else "被拒收会用更短的窗口先报"
                )
                self.get_logger().warn(
                    f"{self._state} 指令已发出 {elapsed:.1f}s, 机械臂仍未起步 "
                    f"(位移 < {float(self.get_parameter('stall_detect_epsilon').value) * 1000.0:.1f} mm): "
                    f"固件可能没执行这一条 —— 最迟 {limit:.0f}s 判运动超时; "
                    f"看上一行的目标是否合理 (move_p / move_l / move_j 都会打; {hint})"
                )
        if elapsed > limit:
            # 超时只说明「没收敛」, 不说明「没动」, 而这两者的病因完全不同。
            # 把实际停在哪一并打出来, 下次跑一次就能分辨:
            #   停在半路 (偏差比整段步长小, 例如 18mm 目标只差 6mm)
            #       -> 动过但推不动, 多半是夹爪顶住了瓶子
            #   原地没动 (偏差≈整段步长, 例如目标 0.1604 实际 0.1704)
            #       -> MCU 收了目标却没执行, 多半是 IK 无解 / 关节限位
            if self._tcp_pose is None:
                actual = "无 TCP 反馈"
            else:
                actual = (
                    f"{np.round(self._tcp_pose[1], 4)} "
                    f"(偏差 {np.linalg.norm(self._tcp_pose[1] - self._goal_position) * 1000.0:.1f} mm)"
                )
            self._fail(
                f"运动超时 ({self._state}, 路点 {self._waypoint_index + 1}/"
                f"{len(self._waypoints)}, 耗时 {elapsed:.1f}s, 上限 {limit:.0f}s, "
                f"目标 {np.round(self._goal_position, 4)}, "
                f"实际 {actual})"
            )
            return

        if self._tcp_pose is None:
            return
        distance = float(np.linalg.norm(self._tcp_pose[1] - self._goal_position))
        if distance <= self._active_tolerance() and self._motion_reached():
            if self._tolerance_start is None:
                self._tolerance_start = time.monotonic()
                return
            if time.monotonic() - self._tolerance_start < self._active_settle_time():
                return
            self.get_logger().info(
                f"{self._state} 到位 (误差 {distance * 1000.0:.1f} mm, "
                f"路点 {self._waypoint_index + 1}/{len(self._waypoints)})"
            )
            self._advance_waypoint(next_state)
            return

        # 没进容差。要区分两种「没到位」: 还在走 (等着就行), 以及
        # 已经停在新稳态上却够不到 —— 后者才是末端下垂, 才值得迭代修正。
        self._tolerance_start = None
        self._try_endpoint_correction(distance)

    def _advance_waypoint(self, next_state):
        """当前路点到位: 队列没走完就继续下一段, 走完才切换状态。"""
        self._waypoint_index += 1
        if self._waypoint_index < len(self._waypoints):
            self._goal_sent = False
            self._tolerance_start = None
            self._waypoint_enter_time = time.monotonic()
            self._reset_endpoint_correction()
            self.get_logger().info(
                f"{self._state} 推进到路点 {self._waypoint_index + 1}/"
                f"{len(self._waypoints)}"
            )
            return
        self._advance_after_motion(next_state)

    def _advance_after_motion(self, next_state):
        if next_state == self._STATE_TRACK_WAIT:
            self._enter_track_wait()
            return
        if next_state == self._STATE_DESCEND:
            # 悬停到位后进下降。下降没有路点队列 (只有一条指令), 目标用
            # _final_grasp_target —— 由 _tick_wait_target 或 _confirm_descend 算好。
            # 日志只在 PRE_GRASP 走这一支时写 (另一支 _confirm_descend 自己会打),
            # 而且必须写清楚是**哪条路**跳过了 TRACK_WAIT: 现场看到「悬停到位直接
            # 下降」时, 不能把它跟「跟丢判失败回待机位」混起来 —— 后者是要回位收工的。
            # 「有没有跳过」仍然只判一次 (_track_wanted), 下面那个 if/else 只负责
            # 把**原因**写出来, 不另做一次判决。
            if self._state == self._STATE_PRE_GRASP and not self._track_wanted():
                if self._skip_track_for_mode():
                    self.get_logger().info(
                        f"倒瓶 (top) 不跟踪 (top_skip_track=true): 悬停到位直接下降; "
                        f"下降点用锁定目标时的坐标 "
                        f"{np.round(self._final_grasp_target, 4)} m (夹爪, base), "
                        f"本轮落点无人确认"
                    )
                else:
                    # 被 _track_wanted 否掉而 top_skip_track 又不成立, 只剩这一条
                    self.get_logger().info(
                        "track_enabled=false: 悬停到位直接下降, 不进 TRACK_WAIT"
                    )
            self._transition(next_state)
            return
        if next_state == self._STATE_PLACE_MOVE:
            # 抬升到位: 装载直达放置位姿的路点队列 (在 _enter_place 里算)。
            self._enter_place()
            return
        if next_state == self._STATE_IDLE:
            # self._state 此刻还是切换前的状态 (LIFT 或 HOME), 两个分支就靠它区分
            if self._state == self._STATE_HOME:
                self.get_logger().info(
                    f"已回到待机位 (夹爪 {np.round(self._standby_position, 4)} m), 任务结束"
                )
                self._reset_target_lock()
            else:
                self._finish_task("抓取任务完成")
                return
        self._transition(next_state)

    def _descend_plan(self):
        """当前下降阶段的三样东西: (目标, 姿态, 后继状态)。

        现在只剩抓取下降 (DESCEND) 一个用户了 —— 放置那一段 2026-09-18 起改成
        「一条直线直达放置位姿、判到位」(_enter_place), 不再走盲降。
        """
        return self._final_grasp_target, self._goal_orientation, self._STATE_GRASP_WAIT

    def _tick_descend(self):
        """DESCEND 的心跳: 一条指令直接降到目标, 然后等满 dwell 进下一个状态。

        这条指令走 **MOVE_L (笛卡尔直线)** —— DESCEND 在 _LINEAR_MOTION_STATES 里, 由
        _send_move_p 按状态自动选路, 这里不用写。改之前走的是 MOVE_P, 固件自己的点到点
        规划会让夹爪中途兜弧, 不是垂直落到瓶子上。

        刻意**不拿 /feedback/tcp_pose 判是否到达指定位置**: 这台机末端有固定重力下垂
        (见迭代末端修正那一段), 「距目标还有多远」长期不为零, 判它只会变成反复重发/
        迭代修正 —— 2026-09-16 那套分段下降又慢又抖就是这么来的。降得准不准由下游的
        人工确认兜底 (GRASP_WAIT = 确认夹爪确实到了抓取点), 这里只保证「指令发出去了,
        并且给足了走完的时间」。

        (放置那一段 2026-09-18 之前也走这条路, 那时「不能判到位」的理由是物体落到台面上
        以后机械臂就更不下去了, 判到位会为了追那个永远追不上的残差**持续往下压**。现在
        放置改成直达释放点并且判到位, 所以那个理由只对「往台面上压」成立 —— 释放点悬空时
        判到位是对的。)

        代价: 机械臂完全不动 (被挡住 / IK 无解) 也不会再报失败 —— 看一眼日志即可。
        下面的日志会打实际位置, 是给人看「到点那一刻机械臂走到哪了」, **不参与判断**;
        若显示还在明显移动, 把 dwell 调大。
        """
        target, orientation, successor = self._descend_plan()
        if target is None or orientation is None:
            self._fail(f"{self._state} 没有目标点或姿态")
            return

        dry_run = self.get_parameter("dry_run").value
        if not dry_run:
            problem = self._feedback_problem()
            if problem is not None:
                self._fail(problem)
                return

        if not self._goal_sent:
            # 距离上限针对机械臂真正要走的位移, 所以在 **上报系** 里比较
            if not dry_run and not self._check_distance(
                self._gripper_to_reported(target, orientation)
            ):
                return
            self._send_move_p(target, orientation)
            self._goal_sent = True
            self._goal_position = target
            self._sent_orientation = orientation
            self._descend_sent_time = time.monotonic()
            return

        waited = time.monotonic() - self._descend_sent_time
        if waited < float(self.get_parameter("descend_dwell").value):
            return

        if self._tcp_pose is None:
            actual = "无 TCP 反馈"
        else:
            pose = np.asarray(self._tcp_pose[1], dtype=float)
            actual = (
                f"实际 {np.round(pose, 4)} "
                f"(距目标 {np.linalg.norm(pose - target) * 1000.0:.1f} mm)"
            )
        self.get_logger().info(
            f"{self._state} 下降指令已发出 {waited:.1f}s (不判到位): "
            f"目标 {np.round(target, 4)} (夹爪, base), {actual}"
        )
        self._transition(successor)

    def _tick_grasp_wait(self):
        now = time.monotonic()
        required = self.get_parameter("grasp_confirm_required").value
        if required:
            if now - self._prompt_time >= 2.0:
                self._prompt_time = now
                confirm_topic = self.get_parameter("grasp_confirm_topic").value
                # 下降不判到位, 所以到这里的「已下降到抓取点」是**指令层面**的说法,
                # 机械臂实际走到哪由人看一眼确认 —— 提示里必须说清楚, 否则会被读成
                # 「节点已经确认到位了」。
                if self._gripper_ready:
                    action = (
                        "确认夹爪确实到了抓取点后, 按 Enter 让节点**现在闭合夹爪** "
                        "(闭合成功后自动抬升); 之前请先把瓶子摆正在两指之间"
                    )
                else:
                    action = "请闭合夹爪, 然后在本终端按 Enter"
                self.get_logger().info(
                    f"已下降到抓取点 (第二次确认, 下降不判到位, 请确认夹爪确实到了抓取点): "
                    f"{action}, 或发布 {confirm_topic} '{{data: true}}'"
                )
            if not (self._enter_pressed or self._confirm_received):
                return
        else:
            # 不需要人工确认 (grasp_confirm_required=false): 进本状态第一拍就往下走。
            # _transition 会把 _prompt_time 清零, 所以下面那道 2s 闸门在这里必然立即
            # 放行 —— 留着它不是让「自动」慢 2s, 而是当兜底: 动作那条路目前**每一条
            # 出口都会转状态或判失败** (_enter_gripper / _begin_lift / _fail), 万一将来
            # 多出一条「什么也没做就返回」的出口, 这里会 2s 后重试, 而不是静默卡在
            # GRASP_WAIT 不动 —— 人工模式下人可以再按一次 Enter, 自动模式没人可按。
            if now - self._prompt_time < 2.0:
                return
            self._prompt_time = now
            self.get_logger().info(
                "已下降到抓取点 (grasp_confirm_required=false, 不等人工确认): "
                + ("闭合夹爪" if self._gripper_ready else "抬升 (夹爪不由节点接管)")
            )
        self._enter_pressed = False
        self._confirm_received = False
        if self._gripper_ready:
            # Enter 的含义从「我已经手动夹好了」变成「现在夹」。抬升挪到闭合成功
            # 之后 (_advance_after_gripper -> _begin_lift), 所以在夹爪真的合上之前
            # 机械臂不会动 —— 手还伸在夹爪里也来得及退开。
            # 注意这一条**只在 required=true 时成立**: 自动模式下没有「人退开」这一步,
            # 手还伸在夹爪里就会被合上 (代价写在 grasp_confirm_required 的参数注释里)。
            if required:
                self.get_logger().info("收到夹取确认, 闭合夹爪")
            self._enter_gripper(self._GRIPPER_STAGE_CLOSE)
            return
        self._begin_lift()

    def _begin_lift(self):
        """GRASP_WAIT 的确认走完之后抬升。

        原来整段就写在 _tick_grasp_wait 的确认分支里。夹爪接进来之后「确认」触发的是
        闭合, 抬升改由闭合成功回调过来 (_advance_after_gripper), 于是提出来 ——
        gripper_enabled=false 时它仍然是「确认即抬升」那条老路, 两处共用同一份代码。

        _robot_ok() 的闸门跟着抬升走, 而不是跟着确认走: 闭合是夹爪的事, 机械臂报不
        报故障都该夹; 而抬升要读 _tcp_pose[1] 当起点, 反馈不可信就不能抬。

        抬升这一段走 **MOVE_P (2026-09-28 从 L 退回)** —— 走哪条由 _send_move_p 按状态
        查 _LINEAR_MOTION_STATES 决定, 这里不写模式。起点取当前夹爪位置加 lift_distance,
        也就是「从这里往上抬 lift_distance」, 与 P/L 无关, 不用额外处理。
        """
        dry_run = self.get_parameter("dry_run").value
        if not dry_run and not self._robot_ok():
            return
        # 真机: 从当前夹爪位置向上抬升, 避免使用可能过期的视觉目标;
        # dry_run: 从最终抓取点向上抬升。
        base_point = (
            self._tcp_pose[1]
            if not dry_run and self._tcp_pose is not None
            else self._final_grasp_target
        )
        if base_point is None:
            self._fail("抬升阶段没有可用起点")
            return
        self._lift_target = base_point + np.asarray(
            (0.0, 0.0, self.get_parameter("lift_distance").value)
        )
        align = self._lift_align_orientation()
        if not self._check_workspace_pair(self._lift_target, align or self._lift_orientation()):
            return  # 内部已触发失败
        if align is not None:
            plan = self._lift_align_waypoints(align)
            if plan is not None:
                waypoints, orientations = plan
                self.get_logger().info(
                    f"开始抬升 (顺带回正姿态 {math.degrees(self._orientation_angle(self._lift_orientation(), align)):.1f}°, "
                    f"拆 {len(waypoints)} 段: 关节直线采样, 每段各自过三道守门)"
                )
                self._transition(self._STATE_LIFT, waypoints, orientations)
                return
            # plan 为 None: 内部已说明原因并回落到「只抬升、不回正」—— 这是任务能够
            # 继续跑完的那条路, 与其它地方的「回落 move_p」同一个取向。
        self.get_logger().info("开始抬升")
        self._transition(self._STATE_LIFT, [self._lift_target])

    def _lift_align_orientation(self):
        """抬升时顺带回正到的姿态 (四元数); **不需要回正时返回 None**。

        判据只有一条: 当前 (抓取) 姿态与 **side 抓取姿态** 差多少。不超过
        `lift_align_threshold_deg` 就不回正 —— side 模式的抓取姿态本来就取自
        `side_orientation_xyzw`, 差值≈0, 于是那一段逐字保持原样 (仍下发当前 TCP
        姿态, 含重力下垂), 这条改动对 side 零影响。

        为什么回正目标是 side 姿态、不是放置姿态: 放置段本来就是按 side 姿态设计的
        (它与放置姿态差 ~54°, 2026-09-30 实测一条 move_j 走完、到位 1.6mm)。倒瓶
        (top) 的抓取姿态来自感知 live、工具 Z 竖直朝下, 与放置姿态差 144°。离线按
        同一套守门算过: 在抬升后的位置直接回正到**放置姿态**解不出来 (残差 0.46°,
        放开多重起点才收敛但余量只剩 4.8° < 5°), 而回正到 side 姿态余量 33°、之后
        那段放置的解与现场实测成功的解逐位相同。

        **纯谓词, 不 _fail**: `_skip_lift_for_mode` 每轮闭合后都会问它一次, 参数坏掉
        只当「不回正」并记一条 warn, 让任务照旧走老路, 而不是把整轮判失败。
        """
        current = self._lift_orientation()
        if current is None:
            return None
        try:
            raw = [float(v) for v in self.get_parameter("side_orientation_xyzw").value]
        except (TypeError, ValueError):
            raw = []
        norm = math.sqrt(sum(v * v for v in raw)) if len(raw) == 4 else 0.0
        if norm < 1e-9:
            self.get_logger().warn(
                "side_orientation_xyzw 不是有效的四元数 —— 本轮不做姿态回正"
            )
            return None
        target = tuple(v / norm for v in raw)
        turn = self._orientation_angle(current, target)
        if turn is None or math.degrees(turn) <= float(
            self.get_parameter("lift_align_threshold_deg").value
        ):
            return None
        return target

    def _reported_pose_to_quaternion(self, rpy_deg):
        """上报系 ZYX-RPY (度) -> 内部四元数: `_to_reported_pose` 姿态那半边的**逆**。

        回正的路点是正解出来的 (上报系), 而路点队列装的是内部夹爪系位姿, 所以需要
        这一小步。`ik_target_invert_rx_ry` 是自逆的 (取反两次还原), 两处共用同一个
        布尔, 与 `_to_reported_pose` 天然一致 —— 现场这个开关是 false。
        """
        roll, pitch, yaw = (math.radians(float(v)) for v in rpy_deg)
        if self.get_parameter("ik_target_invert_rx_ry").value:
            roll, pitch = -roll, -pitch
        return self._rpy_to_quaternion(roll, pitch, yaw)

    def _lift_align_waypoints(self, align):
        """抬升 + 回正的路点与姿态; 解不出来返回 None (调用方回落「只抬升、不回正」)。

        为什么不直接发「抬升位置 + 回正姿态」这一条: 目标构型与当前构型的单轴差有
        146° (J6), 守门 ③ (`ik_max_joint_jump_deg` = 60°) 会判「换构型, 会被原样执行
        成大摆」拒掉, 回落 move_p —— 而回落正是 2026-09-30 那次大摆的来路。拆段的
        意义在**执行**: 每条 move_j 的单轴差 ≤60°, 三道守门逐段照旧生效; 至于路径
        本身, 固件对 J 本来就是关节插值, 拆与不拆走的是同一条关节路径。

        采样用 URDF 约定的关节角、正解用 vendored 的 `forward_kinematics` (与
        `_ik_selfcheck` 同一套), 再把正解出来的上报系位姿换算回内部夹爪系。

        ⚠ **还没做完的一块 (2026-09-30 决定先不动)**: 回正之后那一段放置, 从回正后的
        构型去放置位姿时差的是**腕部** J4 63.2° / J6 55.3° (肩肘只有 29.2°, 余量 35.1°)
        —— 比 `ik_max_joint_jump_deg` 的 60° 多 3°, 于是那一段仍会回落 move_p (也就是
        固件逐点解逆解那条路)。两条备选都已量过: 给腕部单独一个门槛 (J4/J5/J6 设 70°,
        肩肘仍 60° —— 09-28 那次 J4 跳 117.6° 的事故仍拦得住), 或把放置段也按关节直线
        拆成 2 段 (每段 31.6°、余量 20.5°, 守门一个字不动, 但「放置段恒为一条指令」那条
        契约要改)。**现场先按现状跑, 看回正这一步本身对不对再定。**
        """
        seed = self._joint_seed_deg()
        if seed is None:
            self.get_logger().warn(
                "回正: 没有新鲜的关节反馈当种子 —— 回落「只抬升、不回正」"
            )
            return None
        joint, reason = self._solve_joint_target(
            self._lift_target, align, seed_deg=seed, align=True
        )
        if joint is None:
            self.get_logger().warn(f"回正: 这一段解不成 ({reason}) —— 回落「只抬升、不回正」")
            return None
        target_urdf = np.asarray(self._last_solved_urdf_deg, dtype=float)
        module, (model, data, fid) = self._ik_module, self._ik_model
        segments = max(1, int(self.get_parameter("lift_align_segments").value))
        waypoints, orientations = [], []
        for step in range(1, segments + 1):
            config = seed + (target_urdf - seed) * (step / float(segments))
            try:
                pose = module.forward_kinematics(model, data, fid, config)
            except Exception as exc:  # noqa: BLE001 - 正解失败不该拖垮节点
                self.get_logger().warn(
                    f"回正: 中间路点正解失败 ({exc}) —— 回落「只抬升、不回正」"
                )
                return None
            quaternion = self._reported_pose_to_quaternion(pose[3:])
            waypoints.append(
                self._reported_to_gripper(
                    np.asarray(pose[:3], dtype=float) * 0.001, quaternion
                )
            )
            orientations.append(quaternion)
        # FK 复核 (最后一段就是我们真正要去的那个位姿): URDF 版本 / ik_joint_sign /
        # ee_frame 有一个对不上都会在这里露头 —— 那类解**收敛、误差 0、余量也够**,
        # 三道守门一条都抓不到 (与 _ik_selfcheck 同一个取向)。
        error_mm = float(np.linalg.norm(np.asarray(waypoints[-1]) - self._lift_target) * 1000.0)
        error_deg = math.degrees(self._orientation_angle(orientations[-1], align) or 0.0)
        if error_mm > 2.0 or error_deg > 2.0:
            self.get_logger().error(
                f"回正: FK 复核不过 (位置差 {error_mm:.1f}mm, 姿态差 {error_deg:.1f}°) —— "
                "回落「只抬升、不回正」; 先查 URDF / ik_joint_sign / ee_frame 是否与真机一致"
            )
            return None
        return waypoints, orientations

    def _tick_place_wait(self):
        """PLACE_WAIT 的心跳: 到达放置点后放下物体, 然后回待机位。

        进本状态时位置和姿态**都已经判到位** (PLACE_MOVE 走 _tick_motion, 2026-09-18
        之前那个盲降的 PLACE_DESCEND 已经删掉), 所以
        place_confirm_required=true (原行为) 时「到底放没放好」由人看一眼定 ——
        提示里必须说清楚这一点, 否则会被读成「节点已经确认放好了」。
        现场默认是 false: 不等 Enter, 到点后再停 place_descend_dwell 秒就张开并回位
        (见 _declare_parameters), 此时那一行提示不再打印 (没人会停下来看), 只有动作日志。
        """
        now = time.monotonic()
        required = self.get_parameter("place_confirm_required").value
        if required:
            if now - self._prompt_time >= 2.0:
                self._prompt_time = now
                confirm_topic = self.get_parameter("grasp_confirm_topic").value
                if self._tcp_pose is None:
                    actual = "无 TCP 反馈"
                else:
                    pose = np.asarray(self._tcp_pose[1], dtype=float)
                    actual = (
                        f"实际 {np.round(pose, 4)} "
                        f"(距目标 {np.linalg.norm(pose - self._place_position) * 1000.0:.1f} mm)"
                    )
                if self._gripper_ready:
                    action = (
                        "按 Enter 让节点**现在张开夹爪**放下物体 (张开后自动回待机位)"
                    )
                else:
                    action = "请张开放下物体, 然后在本终端按 Enter 确认已放置 (确认后回待机位)"
                self.get_logger().info(
                    f"已到放置点 (第三次确认, 位置与姿态已判到位): "
                    f"夹爪目标 {np.round(self._place_position, 4)} m, {actual}; "
                    f"{action}, 或发布 {confirm_topic} '{{data: true}}'"
                )
            if not (self._enter_pressed or self._confirm_received):
                return
        else:
            # 不需要人工确认 (place_confirm_required=false): 到点后停 place_descend_dwell
            # 秒再放 —— 这个停留时间是从旧的「盲降 dwell」直接继承下来的 (那时它等的是
            # 「下降指令发出去之后机械臂走完」), 现在等的是「判到位之后机械臂真的停稳」,
            # 作用一样: 别在还在晃的时候松手。
            # 起算点用 _state_enter_time (进本状态那一刻 = 抵达放置点), **不是**
            # _prompt_time —— 后者被 _transition 清零, 拿它算「已经等了多久」会得到一个
            # 巨大的值, 等于这道停留根本不生效。_tick_grasp_wait 那边能用 _prompt_time
            # 是因为它只要「>= 2s 就放行」, 清零正好等于立即放行 (那里本来就要立即走)。
            dwell = float(self.get_parameter("place_descend_dwell").value)
            if now - self._state_enter_time < dwell:
                return
            self._prompt_time = now
            self.get_logger().info(
                f"已到放置点并停稳 {dwell:.1f}s (place_confirm_required=false, "
                "不等人工确认): "
                + ("张开夹爪" if self._gripper_ready else "回待机位 (夹爪不由节点接管)")
            )
        self._enter_pressed = False
        self._confirm_received = False
        if not self._gripper_ready:
            self._finish_task("放置完成")
            return
        # 这里**只**查 dry_run, 不查 _robot_ok() —— 与 _tick_grasp_wait 故意不同:
        # 张开是纯粹的夹爪动作, 机械臂可以完全不动; 而物体已经放好了 (人工确认过,
        # 或 place_confirm_required=false 时按 dwell 到点), 因为机械臂报了个故障就
        # 迟迟不松手, 只会把它一直吊在放置点上方。
        # 反馈真的有问题的话, 紧接着的 _finish_task -> HOME 会走既有的盲走路径。
        if not self.get_parameter("dry_run").value and self._feedback_problem() is not None:
            self.get_logger().warn(
                "机械臂反馈不健康, 但放置已确认 —— 照常张开夹爪, 随后按盲走回位"
            )
        if required:
            self.get_logger().info("收到已放置确认, 张开夹爪")
        self._enter_gripper(self._GRIPPER_STAGE_RELEASE)

    # ── 夹爪 (GRIPPER) ────────────────────────────────────

    def _enter_gripper(self, stage):
        """进入 GRIPPER 状态。三个进入点**只**负责置 stage, 其余全在这里。

        刻意不在这里 spawn: spawn 放在 _tick_gripper 的第一拍, 这样测试只要替换
        _spawn_gripper_job 就能完全接管这一趟, 不必碰 subprocess / 串口。

        **保持中的进程不在这里回收** (2026-09-28): 张开类的阶段可以直接复用那个还在
        发力的进程 —— 给它写一行 OPEN 就是「松开」, 不用另起一个去抢同一个串口。
        判据在 _can_reuse_gripper_job; 不能复用的照旧一律回收。
        """
        if stage not in self._GRIPPER_PLANS:
            # 防御: 计时器回调里抛异常会直接拖垮节点 (同 _enter_place 的理由)。
            self._fail(f"内部错误: 未知的夹爪阶段 {stage!r}")
            return
        if not self._can_reuse_gripper_job(stage):
            self._reap_gripper_job()   # 内部会把 _gripper_job 与 _gripper_holding 清掉
        self._gripper_stage = stage
        self._gripper_started_at = None
        self._gripper_adopted = False
        self._transition(self._STATE_GRIPPER)

    def _can_reuse_gripper_job(self, stage):
        """这一趟能不能直接用手里那个**保持中**的进程 (而不是另起一个)。

        三条都要成立:
          - 目标是「张开」类的阶段 (`_GRIPPER_PLANS` 里 mode == "open"; open_start
            与 release 都是) —— 保持进程收到 OPEN 才松手, 闭合类阶段它帮不上忙;
          - 手里那个确实在保持 (`_gripper_holding`);
          - 它确实还活着 —— 活着的才收得到 OPEN; 已经死了就没什么可复用的。
        """
        if not self._gripper_holding or self._gripper_job is None:
            return False
        if self._GRIPPER_PLANS[stage][0] != "open":
            return False
        return self._gripper_job.poll() is None

    def _gripper_plan(self):
        """这一趟要做什么: (脚本模式, 日志说明)。唯一输入是 _gripper_stage。

        与 _descend_plan / _active_orientation 同一套路 —— 把「按阶段分派」集中到
        一处, 进入点就不必各自记住细节。
        """
        return self._GRIPPER_PLANS[self._gripper_stage]

    def _tick_gripper(self):
        """GRIPPER 状态的一拍: 起进程 / 收输出 / 等它退出。**全程非阻塞**。

        它跑在 20Hz 主循环里 (单线程执行器), 所以这里不能等子进程 —— 一等掉的就是
        /feedback 订阅和 execute_grasp 服务。等待全部交给「每拍 poll 一次 + 比时间戳」。

        判据只有一条: **这一趟做完了没有**。除了退出码, 2026-09-28 起多一个通道 ——
        闭合后的 `[holding]` 标记 (保持进程不退出, 没有退出码可等, 见 _gripper_outcome)。
        其余 stdout (`[stop]` / `[warn]` 之类) 仍然只是排障信息, 不参与判断 ——
        夹到没有、夹得正不正, 由人看。
        """
        if self._gripper_started_at is None:
            self._start_gripper_stage()
            return
        job = self._gripper_job
        if job is not None:
            for line in job.read_lines():
                self._note_gripper_line(line)
        outcome = None if job is None else self._gripper_outcome(job)
        if outcome is None:
            timeout = float(self.get_parameter("gripper_timeout").value)
            waited = time.monotonic() - self._gripper_started_at
            if waited < timeout:
                return
            mode, label = self._gripper_plan()
            if self._gripper_adopted:
                # 复用那条路: 已经写进去了 OPEN, 在等它张开完退出。
                detail = (
                    f"已向保持中的夹爪进程写 OPEN, 但 {timeout:.0f}s 内没退出 "
                    f"(脚本模式 {mode})"
                )
            else:
                detail = (
                    f"{label}超过 {timeout:.0f}s 还没退出 (脚本模式 {mode}); "
                    "检查 gripper_port 是否被占用 (厂商 web server / 另一个 04 还在跑)、"
                    "以及闭合时是不是卡在切换 MIT 模式那一步"
                )
            outcome = ("fail", detail)
        status, detail = outcome
        if status == "ok":
            self._advance_after_gripper(detail)
        else:
            self._fail_gripper(detail)

    def _start_gripper_stage(self):
        """GRIPPER 的第一拍: dry_run 直接放行, 否则起进程。"""
        mode, label = self._gripper_plan()
        if self.get_parameter("dry_run").value:
            # dry_run 的规矩是「不碰硬件」, 夹爪也不破例。日志里留一行, 让人能看出
            # 这一趟本该下什么命令。
            self.get_logger().info(
                f"[dry_run] 夹爪命令: --mode {mode} "
                f"--port {self.get_parameter('gripper_port').value} "
                f"--velocity {float(self.get_parameter('gripper_velocity').value):g}"
                " (不碰硬件, 直接继续)"
            )
            self._advance_after_gripper(f"{label} (dry_run 未实际执行)")
            return
        # 手里有个还在保持的进程, 这一趟又是「张开」—— 直接命令它松手, 不另起进程:
        # 串口只有一个, 另起一个会和它抢, 抢输的那个 [abort] 退 2。
        if self._can_reuse_gripper_job(self._gripper_stage):
            if self._gripper_job.write_line("OPEN"):
                self.get_logger().info(
                    f"{label}: 复用保持中的夹爪进程 (写 OPEN 让它松开), 不另起进程"
                )
                self._gripper_holding = False
                self._gripper_adopted = True
                self._gripper_started_at = time.monotonic()
                return
            # 写不进去 (进程恰好死了 / 管道异常) —— 报到下面那条「另外起一个」的路上。
            self.get_logger().warn(
                f"{label}: 保持中的夹爪进程写不进 OPEN, 改为另起一个进程"
            )
        # 要另起一个了: 先把手里那个收掉。正常路径上它已经是 None (进状态时就回收了);
        # 会留下东西的是这两种: 上面写 OPEN 没送到的那个, 以及**进入了 GRIPPER 之后、
        # 这一拍之前**才死掉的那个 (进状态那一刻它还活着, 所以没被回收)。不收就会漏掉
        # 它的两条管道, 串口也可能还占着。
        self._reap_gripper_job()
        job = self._spawn_gripper_job(mode)
        if job is None:
            return  # 已经 _fail 过
        self._gripper_job = job
        self._gripper_started_at = time.monotonic()

    def _spawn_gripper_job(self, mode):
        """起一个夹爪脚本进程; 起不来就 _fail 并返回 None。

        管道装配在模块级的 spawn_gripper_process 里 (测试要复用同一条路径);
        这里只管拼命令行。命令行里那个 `-u` 不能少 —— 不加的话脚本的 stdout 会
        攒在缓冲区里, 真机排障时日志要等进程退出才刷出来, 正好错过出问题的那一刻。
        """
        argv = [
            sys.executable,   # 跟启动节点的解释器一致, 不硬编码 python3
            "-u",
            self._gripper_script,
            "--mode", mode,
            "--port", str(self.get_parameter("gripper_port").value),
            "--velocity", f"{float(self.get_parameter('gripper_velocity').value):g}",
            "--timeout", f"{float(self.get_parameter('gripper_script_timeout').value):g}",
        ]
        if mode == "close":
            argv += [
                "--resistance-torque",
                f"{float(self.get_parameter('gripper_resistance_torque').value):g}",
                "--hard-torque-limit",
                f"{float(self.get_parameter('gripper_hard_torque_limit').value):g}",
                # 保持 (2026-09-28): 0 = 不传也无所谓, 脚本默认就是 0 (一次性行为)。
                # 传了才能让「老节点 + 新脚本」与「新节点 + 旧脚本」都在日志里看明白 ——
                # 旧脚本不认识这些参数会直接退 2 (argparse), 是安全的失败。
                "--hold-torque",
                f"{float(self.get_parameter('gripper_hold_torque').value):g}",
                "--hold-timeout",
                f"{float(self.get_parameter('gripper_hold_timeout').value):g}",
                "--hold-kd",
                f"{float(self.get_parameter('gripper_hold_kd').value):g}",
            ]
        self.get_logger().info("启动夹爪脚本: " + " ".join(argv))
        try:
            return spawn_gripper_process(argv, mode)
        except OSError as exc:
            self._fail(f"启动夹爪脚本失败: {exc}")
            return None

    def _note_gripper_line(self, line):
        """把脚本 stdout 的一行转成日志, 并认下唯一一行**协议**: `[holding]`。

        除那行之外**不解析任何结论**: 脚本只报告过程 —— `[start]` 把这次用的阈值打
        出来, `[stop]` 说为什么停下, `[abort]` 说为什么没做成。夹到没夹到是人在现场
        看的, 节点不猜 (2026-09-17 用户明确要求)。

        **别再退回白名单**: 原来是 [stop]/[abort]/[warn]/[release]/[selftest] 五个,
        于是 [start] 那行 (带 idle_tau 和 resistance_threshold —— 出问题时唯一能看出
        「阈值到底设在哪」的两个数) 被静默降级到 debug, 真机上出事时日志里正好缺的
        就是它。宁可多打几行。
        """
        if not line:
            return
        # [holding] 是 2026-09-28 起的**协议行**: 它表示「夹持力已经建立」。保持进程
        # 不会退出, 所以它不退出时节点只有这一个「这一步做完了」的信号 (见
        # _gripper_outcome)。
        # **只在 CLOSE 阶段认它**: 张开类阶段绝不能认 —— 缓冲区里可能还留着闭合那趟
        # 的行, 认了就会把「还夹着」读成「已张开」, 然后一路报放置完成。
        if line.startswith("[holding]") and self._gripper_stage == self._GRIPPER_STAGE_CLOSE:
            if not self._gripper_holding:
                self.get_logger().info(f"夹爪脚本: {line}")
            self._gripper_holding = True
            return
        if line.startswith("[") and "]" in line[:16]:
            self.get_logger().info(f"夹爪脚本: {line}")
            return
        self.get_logger().debug(f"夹爪脚本: {line}")

    def _gripper_outcome(self, job):
        """给当前 job 下结论: None = 还没结束, 否则 (\"ok\"/\"fail\", 说明)。

        **判的只是「这条命令跑完了没有」, 不是「夹到没夹到 / 掉没掉」。** 节点不替
        用户判断物体状态: 闭合是闭合、张开是张开, 夹到什么、有没有夹住由人在确认点
        看 (2026-09-17 用户明确要求)。脚本 [stop] 里写的是「一路合到底」还是
        「撞到阻力」, 都只是日志。

        判据按顺序:
          1. 退出码非 0 → 失败 (那时闭合/张开根本没发生, 串口被占 -> [abort] 退 2);
          2. 退出码 0 → 完成 (与改动前一致);
          3. **见过 `[holding]` 且进程还活着** → 完成 —— 保持进程不退出, 这是它唯一
             能报「夹住了、力已经加上」的通道 (2026-09-28);
          4. 见过 `[holding]` 但进程已经死了 → 失败。标记承诺过「力已经加上」, 现在
             进程没了, 夹持力也就没了; 这时推进 LIFT 就是抬一个空手。
        """
        exit_code = job.poll()
        if exit_code is not None:
            if exit_code != 0:
                mode, label = self._gripper_plan()
                return (
                    "fail",
                    f"夹爪脚本异常退出 (退出码 {exit_code}), {label}没做; "
                    "见上面 [abort] 那行",
                )
            if self._gripper_holding:
                return (
                    "fail",
                    "闭合完成、但保持进程随即退出 —— 夹持力没有建立, 不能就这么抬升",
                )
            return "ok", self._GRIPPER_DONE_LABELS[self._gripper_stage]
        if self._gripper_holding:
            return "ok", self._GRIPPER_DONE_LABELS[self._gripper_stage]
        return None

    def _advance_after_gripper(self, detail):
        """夹爪动作成功后去哪 —— 三个阶段的去向只在这里说了算。"""
        stage = self._gripper_stage
        if stage == self._GRIPPER_STAGE_OPEN:
            self.get_logger().info(f"夹爪{detail}, 开始找目标")
            self._transition(self._STATE_WAIT_TARGET)
            return
        if stage == self._GRIPPER_STAGE_CLOSE:
            # 夹持力由**那个还活着的进程**维持 (`gripper_hold_torque > 0` 时), 它一路
            # 跟到放置、由 RELEASE 写 OPEN 让它松手; 保持关着时脚本跑完就退出, 靠机构
            # 自锁挡着 (老行为)。**这里不要碰 _gripper_job** —— 接下来 LIFT/PLACE_MOVE
            # 期间 _tick 会一直收它的行 (管道满了会把它憋死), 并盯着它别死在半路。
            if self._skip_lift_for_mode():
                # 倒瓶 (top): 夹爪本来就在物体正上方, 抬升没有意义 —— 直接去放置,
                # 起点由 _enter_place 取当前 TCP。见 _skip_lift_for_mode 的注释。
                self.get_logger().info(f"夹爪{detail}, 直接去放置 (top 模式, 跳过抬升)")
                self._enter_place()
                return
            self.get_logger().info(f"夹爪{detail}, 开始抬升")
            self._begin_lift()
            return
        self._finish_task(f"放置完成 (夹爪{detail})")

    def _fail_gripper(self, reason):
        """夹爪动作失败: 走既有的失败路径 (判失败并回待机位)。

        **不留活口**: 手里要是还有个保持中的进程 (夹着东西在发力), 这里就把它
        SIGTERM 掉 —— 脚本收到信号会失能、立刻松开并退出 (见脚本的退出语义表),
        串口也随之释放。悬空时这一下会把物体放掉。
        """
        self.get_logger().error(f"夹爪动作失败: {reason}")
        if self._gripper_holding and self._gripper_job is not None:
            self.get_logger().warn(
                "夹爪还在保持中 —— 按配置立刻松开 (失能; 机械臂若悬空, 物体会掉)"
            )
        self._reap_gripper_job()   # 内部只杀还活着的; 已退出的只是关管道
        self._fail(reason)

    def _reap_gripper_job(self):
        """丢掉手里的夹爪进程并放掉它的两条管道。

        正常走到这里时它已经退出了 (一次性命令跑完即退)。两处**真会杀**:
          - 上一趟异常结束 (超时 / spawn 后一直没结论);
          - 还在保持 (闭合之后那个常驻进程) —— 失败路径与节点收尾都走这里。
        杀掉是安全的也是**有意的**: 脚本收到 SIGTERM 会失能 → 立刻松开 → 退出,
        串口随之释放 (不杀的话它还攥着串口, 下一个进程打不开)。
        注意这一次松开是「失能即 limp」而不是把手指打开 —— 悬空时物体会掉。
        """
        job = self._gripper_job
        self._gripper_job = None
        self._gripper_holding = False
        if job is None:
            return
        if job.poll() is None:
            job.kill()
        job.close()

    # ── 悬停跟随 (TRACK_WAIT) ─────────────────────────────

    def _enter_track_wait(self):
        """进入悬停跟随: 以刚到达的悬停点为死区基准, 等确认下降。

        auto_grasp_stable_time > 0 时这里也是「自动下降」计时的起点: 计时不在这一刻
        开始, 而是等机械臂停稳 (见 _track_object_settled) —— 那时它才算到了物体上方。
        """
        # _goal_position 此刻正是 PRE_GRASP 的那个悬停路点。拿它当基准而不是量到的
        # 实际位置, 与 track_deadband 的理由一致: 实际位置里含着固定重力下垂,
        # 用它当基准会让死区被下垂量吃掉, 机械臂一直够不到就一直在追。
        self._track_command = np.asarray(self._goal_position, dtype=float)
        self._track_last_command_time = time.monotonic()
        # 只用于提示: 目标相对**初始抓取中心**移动了多少 (运动量, 比跟踪误差好读)
        self._track_initial_position = (
            None if self._grasp_center_base is None
            else np.asarray(self._grasp_center_base, dtype=float)
        )
        self._track_mode_warned = False
        self._reset_track_stability()
        self._transition(self._STATE_TRACK_WAIT)
        hold = float(self.get_parameter("auto_grasp_stable_time").value)
        if hold > 0.0:
            tail = (
                f"机械臂停稳后**物体位置连续 {hold:.1f}s 不变就自动下降** (容差 "
                f"{self.get_parameter('auto_grasp_stable_epsilon').value * 1000.0:.0f} mm); "
                "期间按 Enter 可提前下降"
            )
        else:
            tail = (
                "确认位置无误后按 Enter 开始下降抓取 (或发布 "
                f"{self.get_parameter('grasp_confirm_topic').value} '{{data: true}}')"
            )
        self.get_logger().info(
            f"已悬停在目标上方 {np.round(self._track_command, 4)} m; "
            f"每 {self.get_parameter('track_min_command_interval').value:.1f}s 读一次最新"
            f"坐标, 目标相对上一条指令移动超过 "
            f"{self.get_parameter('track_deadband').value * 1000.0:.0f} mm 才跟随, "
            f"{tail}"
        )

    def _target_age(self):
        """最新候选距现在多久 (s); 从未收到过返回 None。"""
        if self._latest_candidate is None:
            return None
        return time.monotonic() - self._latest_candidate["received_at"]

    def _tick_track_wait(self):
        """悬停跟随: 目标动了就重发悬停点, 直到确认下降。

        与其它运动状态最大的不同: 这里**没有路点队列**, 也不等某一条指令到位。
        它是一条持续运行的伺服回路 —— 目标动就一直跟, 目标不动就一直悬停。
        结束它的有两条路, 都汇到同一个 _confirm_descend:
          - 人工: Enter / grasp_confirm (永远有效, 等同于越权);
          - 自动: auto_grasp_stable_time > 0 时, 机械臂停稳后物体位置连续不变够久
            (见 _auto_grasp_ready)。
        所以它不走 _tick_motion。
        """
        if not self.get_parameter("dry_run").value:
            problem = self._feedback_problem()
            if problem is not None:
                self._fail(problem)
                return

        now = time.monotonic()
        if self._enter_pressed or self._confirm_received:
            self._enter_pressed = False
            self._confirm_received = False
            self._confirm_descend()
            return

        self._follow_target(now)
        if self._state != self._STATE_TRACK_WAIT:
            return  # 刚刚跟丢判失败并转走了, 不要再打一行「悬停跟随中」

        # 全自动下降: 人工确认那一拍走不到这里(上面已 return), 所以人按 Enter 永远赢。
        # 放在 _follow_target **之后**是必须的 —— 候选过期/从未收到这两种情形要在
        # 那里先判失败转走, 否则这里会拿一份冻住的坐标去降(stale 分支的存在意义是
        # 给人一个越权通道, 自动化去用它就变成「拿失效坐标的垂直投影去抓」)。
        # 放在 _prompt_track_wait 之前, 是为了触发那一拍直接打「自动确认下降」,
        # 而不是先打一行马上失效的「按 Enter 确认下降抓取」。
        if self._auto_grasp_ready(now):
            self.get_logger().info(
                f"自动确认下降 (机械臂已到位, 物体位置连续 "
                f"{now - self._auto_grasp_since:.1f}s 未变, 阈值 "
                f"{self.get_parameter('auto_grasp_stable_epsilon').value * 1000.0:.1f} mm)"
            )
            self._confirm_descend("自动: 物体位置连续不变")
            return
        self._prompt_track_wait(now)

    def _follow_target(self, now):
        """一次跟随判定: 每 track_min_command_interval 秒读一次最新候选, 目标动得
        够多就把悬停点重新下发到它当前位置。

        感知停了 (候选超过 max_result_age 没更新) 就**判失败回待机位**, 不再原地
        悬停等人: 跟丢之后机械臂停在一个已经不知道还对不对的位置上, 继续等没有任何
        意义, 而人也不一定盯着终端。想在这种情况下还是抓下去, 按 Enter —— 确认在
        _follow_target 之前处理, 会从当前悬停点原地垂直下降 (见 _confirm_descend)。

        回待机位之后: 循环模式开着的话接着等新坐标 (这里传 keep_loop=True), 没开就
        按单发语义收工。
        """
        # 停稳判据是**滚动**的: 每一拍都拿当前位姿和上一拍比, 才知道机械臂是什么时候
        # 停下来的。所以必须在门口就更新, 不能等三道门都放行才更新 —— 后者会让基线在
        # 「刚发完指令 / 目标没动」的那些拍上永远学不到新位置, 于是周期被吃成两倍
        # (2s -> 4s), 目标走走停停时这道门还会一直不开。
        settled = self._tcp_pose is not None and self._track_arm_settled(now)

        age = self._target_age()
        if age is None:
            # keep_loop=True: 跟丢目标不是机械臂故障, 循环模式开着就回待机位接着等
            # (2026-09-21 用户要求「回到工作位置后继续接受物体识别到的坐标」)。
            self._fail("TRACK_TARGET_LOST: 悬停跟随期间从未收到候选", keep_loop=True)
            return
        max_age = float(self.get_parameter("max_result_age").value)
        if age > max_age:
            self._fail(
                f"TRACK_TARGET_LOST: 候选已 {age:.1f}s 没有更新 "
                f"(阈值 {max_age:.1f}s, 跟随周期 "
                f"{self.get_parameter('track_min_command_interval').value:.1f}s); "
                "感知可能停了",
                keep_loop=True,
            )
            return

        candidate = np.asarray(self._latest_candidate["position_base"], dtype=float)
        if (
            self._latest_candidate["grasp_mode"] != self._grasp_mode
            and not self._track_mode_warned
        ):
            self._track_mode_warned = True
            self.get_logger().warn(
                f"目标抓取模式由 {self._grasp_mode} 变为 "
                f"{self._latest_candidate['grasp_mode']}; 本次仍按原姿态 "
                f"({self._grasp_mode}) 抓取, 不中途换姿态"
            )

        hover = candidate + np.asarray((0.0, 0.0, self._pre_grasp_offset_z()))
        moved = float(np.linalg.norm(hover - self._track_command))
        if moved < float(self.get_parameter("track_deadband").value):
            return
        if now - self._track_last_command_time < float(
            self.get_parameter("track_min_command_interval").value
        ):
            return
        if not self.get_parameter("dry_run").value:
            if self._tcp_pose is None:
                return
            # lag = 机械臂离「上一条指令」还有多远。刻意不是「离新目标多远」:
            # 后者里含着固定重力下垂 (本机 3~55mm), 会让下面这个例外通道常开,
            # 等于把停稳门取消掉。
            lag = float(
                np.linalg.norm(
                    np.asarray(self._tcp_pose[1], dtype=float) - self._track_command
                )
            )
            if lag < float(self.get_parameter("track_max_lag").value) and not settled:
                # 上一条还没走完。两个目标重叠下发正是下降 wp3 那次抖动的病因,
                # 这里宁可慢一拍, 除非目标跑得比机械臂快 (上面那条例外)。
                return
        if not self._check_workspace_pair(hover, self._goal_orientation):
            return  # 内部已 _fail
        if not self.get_parameter("dry_run").value and not self._check_distance(
            self._gripper_to_reported(hover, self._goal_orientation)
        ):
            # 单条指令的位移上限。跟随时最可能触发它的是「同一个 track_id 的位姿突然
            # 跳了一大截」(例如圆柱拟合翻面), 那不是瓶子在移动, 跟着甩过去很危险。
            return  # 内部已 _fail
        self.get_logger().info(
            f"跟随目标: 悬停点 -> {np.round(hover, 4)} m "
            f"(相对上一条指令 {moved * 1000.0:.1f} mm, "
            f"累计移动 {self._target_travel() * 1000.0:.1f} mm)"
        )
        self._send_move_p(hover, self._goal_orientation)
        self._track_command = hover
        self._track_last_command_time = now
        self._reset_track_stability()

    def _target_travel(self):
        """目标相对初始抓取中心移动了多少 (m); 未知时返回 0。"""
        if self._track_initial_position is None or self._latest_candidate is None:
            return 0.0
        return float(
            np.linalg.norm(
                np.asarray(self._latest_candidate["position_base"], dtype=float)
                - self._track_initial_position
            )
        )

    def _track_arm_settled(self, now):
        """机械臂是否已经停在「上一条跟随指令」上 (与下降的停稳判据同构)。

        这是**滚动**判据: 每次调用都比较「当前位姿」与「上一次调用时的位姿」,
        位置变了就把计时起点重置。所以它必须每一拍都被调用 (见 _follow_target 开头),
        由调用频率而不是由「门是否放行」来决定基线多久更新一次。
        """
        pose = np.asarray(self._tcp_pose[1], dtype=float)
        epsilon = float(self.get_parameter("track_stable_epsilon").value)
        if (
            self._track_stable_pose is None
            or float(np.linalg.norm(pose - self._track_stable_pose)) > epsilon
        ):
            self._track_stable_pose = pose
            self._track_stable_since = now
            return False
        return now - self._track_stable_since >= float(
            self.get_parameter("track_settle_time").value
        )

    def _track_object_settled(self, now):
        """物体坐标是否已连续 auto_grasp_stable_time 秒没变 (锚点式滚动窗口)。

        与 _track_arm_settled 同构, 同样**必须每一拍都被调用**: 锚点只在「变了」的
        那一拍重设。注意与 WAIT_TARGET 里由 stable_duration / _target_history 做的那套
        窗口是**两件事** —— 那个管「进 PRE_GRASP 之前目标稳不稳」, 这个管「进 DESCEND
        之前目标还在不在原地」。

        三重门, 缺一不可:
          1. hold <= 0 -> 关闭 (连锚点都不留, 理由见下);
          2. 候选必须新鲜 (超过一个跟随周期没有新坐标就作废窗口);
          3. 位置相对锚点的偏移不超过 auto_grasp_stable_epsilon。
        """
        hold = float(self.get_parameter("auto_grasp_stable_time").value)
        if hold <= 0.0:
            # 关闭时**连锚点都不留**: 否则关掉期间锚点冻着, 运行中 ros2 param set 打开
            # 时可能「物体位置和几十秒前几乎相同 + 计时起点很旧」-> 打开第一拍就下降。
            self._auto_grasp_ref = None
            self._auto_grasp_since = None
            self._auto_grasp_block = "自动下降已关 (auto_grasp_stable_time=0)"
            return False
        age = self._target_age()
        if age is None or age > float(
            self.get_parameter("track_min_command_interval").value
        ):
            # 上游整个不发了的时候, _latest_candidate 会一直停在最后一个值上
            # (见 _select_and_store 里 received_at 的注释), 「坐标没变」很可能只是
            # 「没人在看」。max_result_age (4.0s) 比 hold (3.0s) **长**, 拦不住它 ——
            # 所以这里自己设一道新鲜度门: 超过一个跟随周期没有新坐标就作废窗口,
            # 从下一次新坐标重新计时。此后 _follow_target 会在 max_result_age 判
            # TRACK_TARGET_LOST 回待机位, 绝不会拿一份冻住的坐标下降。
            self._auto_grasp_ref = None
            self._auto_grasp_since = None
            self._auto_grasp_block = (
                "自动下降: 候选已 "
                f"{'未知' if age is None else f'{age:.1f}s'} 未更新, 窗口作废"
            )
            return False
        position = np.asarray(self._latest_candidate["position_base"], dtype=float)
        epsilon = float(self.get_parameter("auto_grasp_stable_epsilon").value)
        if (
            self._auto_grasp_ref is None
            or float(np.linalg.norm(position - self._auto_grasp_ref)) > epsilon
        ):
            # 只有「变了」才重设锚点 —— 写成每拍都赋值就退化成只比相邻两帧,
            # 慢速漂移 (每拍都小于 epsilon) 会被误判成「一直没变」。
            self._auto_grasp_ref = position
            self._auto_grasp_since = now
            self._auto_grasp_block = "自动下降: 物体位置还在变"
            return False
        held = now - self._auto_grasp_since
        self._auto_grasp_block = (
            f"自动下降: 物体位置已稳定 {min(held, hold):.1f}/{hold:.1f}s"
        )
        return held >= hold

    def _auto_grasp_ready(self, now):
        """可以自动下降了吗: 机械臂已到物体上方 + 物体坐标稳定够久。

        计时的**起点是「机械臂到位」而不是「物体停下」**。物体骤停时机械臂可能还在
        横移, 那时若就开始数秒, 数满会立刻下降, 把长距离横移和整段下降并成一条指令
        (`_confirm_descend` 就是这么合的), 而 descend_dwell (2.0s) 可能在机械臂到位前
        就走完 -> 半空闭合夹爪。挂到到位之后, 横移残差只剩物体在死区内的小挪动
        (<= track_deadband), 对 dwell 的影响可以忽略。

        dry_run 下可能根本收不到 /feedback/tcp_pose (`_tcp_pose` 恒 None), 拿它当门
        会让这个功能在干跑里永远不触发 —— 所以「到位」这道门整体包在 not dry_run 里,
        与 _follow_target 把停稳门包在 not dry_run 里是同一个惯例。
        """
        if float(self.get_parameter("auto_grasp_stable_time").value) <= 0.0:
            self._track_object_settled(now)  # 只为让它清掉残留锚点并写上提示
            return False
        if not self.get_parameter("dry_run").value:
            # 这里会与 _follow_target 在同一拍各调一次 _track_arm_settled。连调两次
            # 是安全的: 同一拍内 _tcp_pose 是同一份缓存, 第二次看到的锚点与第一次写回的
            # 一致, 走同一个 elapsed 分支, 结论相同。
            if self._tcp_pose is None or not self._track_arm_settled(now):
                # 机械臂还在横移(或没有反馈): 窗口作废, 到位后从零起数。
                self._auto_grasp_ref = None
                self._auto_grasp_since = None
                self._auto_grasp_block = "自动下降: 等机械臂到位并停稳"
                return False
        return self._track_object_settled(now)

    def _auto_grasp_hint(self):
        """自动下降当前卡在哪道门, 供 2s 一拍的提示直接打印。

        这是这个功能最主要的可诊断性: 不触发时, 日志要能一眼看出是「自动已关」
        「物体还在动」「机械臂还没到位」还是「上游不发了」, 而不用去猜。
        """
        return self._auto_grasp_block or "自动下降: 正在计时"

    def _prompt_track_wait(self, now):
        """每 2s 提示一次当前跟随状态, 提醒怎么确认。"""
        if now - self._prompt_time < 2.0:
            return
        self._prompt_time = now
        age = self._target_age()
        max_age = float(self.get_parameter("max_result_age").value)
        period = float(self.get_parameter("track_min_command_interval").value)
        if age is None:
            situation = "尚未收到任何候选 (再没有就会判失败)"
        elif age > max_age:
            # 正常情况下走不到这里: _follow_target 在此之前已经判失败转走了。
            situation = f"目标已 {age:.1f}s 没有更新 (阈值 {max_age:.1f}s, 即将判失败)"
        elif age > max_age * 0.6:
            situation = (
                f"目标已 {age:.1f}s 没有更新 (阈值 {max_age:.1f}s), 再没有新坐标就判失败"
                "并回待机位; 此时按 Enter 会从当前悬停点原地垂直下降"
            )
        else:
            # 不报「距上一条跟随指令」: 提示和跟随都是 2s 一拍, 两者相位锁死,
            # 那个数永远显示 0.0。候选的新鲜程度才是这里真正要看的活性指标。
            situation = (
                f"目标累计移动 {self._target_travel() * 1000.0:.1f} mm, "
                f"候选 {age:.1f}s 前更新 (周期 {period:.1f}s)"
            )
        self.get_logger().info(
            f"悬停跟随中 ({situation}); {self._auto_grasp_hint()}; "
            f"按 Enter 确认下降抓取, 或发布 "
            f"{self.get_parameter('grasp_confirm_topic').value} '{{data: true}}'"
        )

    def _confirm_descend(self, origin="人工"):
        """确认下降: 用目标当前位置重算下降起终点, 然后进 DESCEND。

        两条入口: 人工 (Enter / grasp_confirm, origin="人工") 与全自动 (物体位置连续
        不变, 见 _auto_grasp_ready)。**两者走的就是这一个函数**, 所以下游 DESCEND /
        _tick_descend / GRASP_WAIT 全都零改动。origin 只进日志 —— 真机排障时那行
        「收到下降确认」必须能自证来源, 否则分不出是人按的还是自动的。

        两条路径, 差别只在「信不信感知」:
          目标新鲜 -> 用最新候选重算。这正是跟随的意义: 瓶子挪到哪就抓到哪。
          目标过期/没有 -> 从当前悬停点**原地垂直**下降。绝不拿一个可能早就失效的
                          坐标去抓 —— 那正是「跟丢了还硬抓」的危险之处。
        两条路径的终点可能横向偏离机械臂当前位置 (目标刚动过而它还没跟上)。
        下降只有一条绝对位姿指令, 所以这段横移和整段下降 (pre_grasp_offset_z -
        grasp_offset_z) 合并成一条一次走完 —— 目标是绝对点, 到的还是同一个抓取点。
        """
        if not self.get_parameter("dry_run").value and not self._robot_ok():
            return

        age = self._target_age()
        fresh = age is not None and age <= self.get_parameter("max_result_age").value
        pre_offset = float(self._pre_grasp_offset_z())
        grasp_offset = float(self.get_parameter("grasp_offset_z").value)
        if fresh:
            center = np.asarray(self._latest_candidate["position_base"], dtype=float)
            pre_grasp = center + np.asarray((0.0, 0.0, pre_offset))
            final_grasp = center + np.asarray((0.0, 0.0, grasp_offset))
            source = f"按最新候选重算, 候选 {age:.2f}s 前"
        else:
            # 悬停点 -> 抓取点: 高度差就是两个 Z 偏移之差, 横向不动 (纯垂直下降)
            base = (
                self._track_command
                if self._track_command is not None
                else self._pre_grasp_target
            )
            pre_grasp = np.asarray(base, dtype=float)
            final_grasp = pre_grasp - np.asarray((0.0, 0.0, pre_offset - grasp_offset))
            source = (
                f"候选已过期 {age:.1f}s, 从当前悬停点原地垂直下降"
                if age is not None
                else "从未收到候选, 从当前悬停点原地垂直下降"
            )
        pre_grasp = np.asarray(pre_grasp, dtype=float)
        final_grasp = np.asarray(final_grasp, dtype=float)
        for target in (pre_grasp, final_grasp):
            if not self._check_workspace_pair(target, self._goal_orientation):
                return  # 内部已 _fail
        self._pre_grasp_target = pre_grasp
        self._final_grasp_target = final_grasp
        self.get_logger().info(
            f"收到下降确认 [{origin}] ({source}): 下降起点(夹爪)={np.round(pre_grasp, 4)} m, "
            f"抓取点(夹爪)={np.round(final_grasp, 4)} m, "
            f"Z 下降 {(pre_grasp[2] - final_grasp[2]) * 100.0:.1f} cm (一条指令走完)"
        )
        self._transition(self._STATE_DESCEND)

    # ── 安全检查 ──────────────────────────────────────────

    def _robot_ok(self):
        """机器人就绪检查: 反馈新鲜、无故障、控制模式就绪。"""
        problem = self._feedback_problem()
        if problem is not None:
            self._fail(problem)
            return False
        return True

    def _feedback_problem(self):
        """纯谓词: 返回反馈/故障问题描述, 一切正常返回 None。**不触发 _fail**。

        _fail 的守卫 (_should_return_to_standby) 要用它判断「机械臂还可控吗」——
        因此这里绝不能调 _fail, 否则 _fail 会递归。
        检查顺序与错误文案与拆分前逐字一致, 所以 8 个调用点一个都不用改。
        """
        timeout = self.get_parameter("input_timeout").value
        now = self.get_clock().now().nanoseconds * 1e-9
        if self._tcp_pose is None or now - self._tcp_pose[0] > timeout:
            return "TCP_FEEDBACK_STALE: /feedback/tcp_pose 超时或缺失"
        if self._arm_status is None or now - self._arm_status[0] > timeout:
            return "ARM_STATUS_STALE: /feedback/arm_status 超时或缺失"
        _stamp, status = self._arm_status
        if status.err_status != 0:
            return f"ROBOT_FAULT err_status={status.err_status}"
        if status.ctrl_mode != 1:
            return f"控制模式未就绪 ctrl_mode={status.ctrl_mode}"
        if any(status.joint_angle_limit):
            return "ROBOT_FAULT joint_limits"
        if any(status.communication_status_joint):
            return "关节通信故障"
        return None

    def _gripper_hold_problem(self):
        """纯谓词: 保持中的夹爪子进程是不是死了。死了返回描述串, 否则 None。

        **不触发 _fail** (由 _tick 收口), 理由同 _feedback_problem。

        为什么只在 LIFT / PLACE_MOVE / PLACE_WAIT 判:
          - 这三个状态是「手里可能还夹着东西」的全部时段;
          - **PLACE_WAIT 不能漏**: place_confirm_required=true 时人可能站几分钟, 这
            期间进程死掉没人发现, 等 Enter 一来节点会回落到「另外起一个 open 进程」,
            那个进程正常张开、节点报「放置完成」—— 瓶子早掉了却报成功, 是最坏的结局;
          - HOME / IDLE 不判: 那时东西已经放下了 (或者已经判过失败), 一个死掉的保持
            进程只是没回收的残留, 不该再把节点推进一次 HOME。

        取不到结论 (不在保持 / 没有 job) 一律返回 None: 宁可慢, 不误判。
        """
        if not self._gripper_holding or self._gripper_job is None:
            return None
        if self._state not in (
            self._STATE_LIFT,
            self._STATE_PLACE_MOVE,
            self._STATE_PLACE_WAIT,
        ):
            return None
        if self._gripper_job.poll() is None:
            return None
        return (
            f"夹爪保持进程已退出 (退出码 {self._gripper_job.exit_code}) —— "
            f"夹持力已失去, 瓶子可能已经掉落, 请人工确认现场后再继续"
        )

    def _ik_rejection_problem(self):
        """最近一条运动指令是不是被固件 IK 拒收了。拒收返回描述串, 否则 None。

        **与模式无关**: 立案在 _send_move_p 里, move_p / move_l 走的是同一套。固件对 L
        也一样过 IK 判决 (厂商 web UI 里那句 `MOVE_L 规划失败: ${ikText}` 就是它), 只是
        L 多一种被拒的理由 —— 整条线段逐点可解才走, 两端都可解而中间不可解也会被拒。

        纯谓词 (不触发 _fail), 由 _tick 收口成一个判失败。

        为什么要单独有这么一条通道: 「命令发出去了、机械臂一动不动」有两个完全不同的
        病根 —— 被挡住推不动, 和 MCU 压根没接受这个目标 (IK 无解 / 关节限位)。
        在 ik_status 进消息之前, 两者在日志里长得一模一样, 都得等满 motion_timeout
        再报一个巨大的偏差。2026-09-21 10:17 那次放置失败就是后者:
        桥接日志里 MCU 当场回了 `IK 状态变化: status=2`, 而执行节点只能等 30s。

        判据四条, 缺一不可 (**后两条都是 2026-09-21 晚上真机打脸之后加的**, 别删):
          - 已经等过 ik_verdict_grace 秒。MCU 要收齐 send_target_pose 的 3 帧位姿
            (帧间 20ms) 才重新判决, 判决再经 CAN 回来; 等不够就会读到上一条的判决。
          - 手里这帧状态快照是**指令之后**才发布的 (stamp >= _ik_watch_since)。
            本意和上一条一样是「不把上一条的判决算到这一条头上」, 但实测之后它基本是
            **空条款**: 加这条时担心 0x5A9 会一直带着旧判决值, 2026-09-21 晚拿真机量了
            一遍 —— **接受一条指令后 6ms 内 ik_status 就归零** (桥接日志 1789961088.171
            发 `move_p`, 1789961088.177 就 `IK 状态变化: status=0`), 而 grace 是 0.8s,
            所以读到「旧判决」的窗口根本不存在。留着它只是图便宜: 快照比指令还旧时
            不判, 交给 motion_timeout, 免得在无快照的极端时序下误判。
          - ik_status 非 0。这条**不依赖 ik_status_valid** —— 那个字段在真机上恒为 0:
            SDK 的 get_state() 建 ArmState 副本时把它漏掉了 (和 ik_joint_limit_fault /
            ik_pose_pending 一起, SDK_linux/sdk_linux.py:816), 所以「等 valid=1」
            等于永远不判 —— 2026-09-21 10:48 那次拒收就是这么漏掉的,
            桥接日志明明写着 `status=2` 而执行节点照样等满了 30s。
          - 机械臂从指令发出到现在**一步没动** (_moved_since_command)。
            这条才是「这一条指令」的证据: 0x5A9 的 ik_status 是**粘住的**: 被拒之后
            一直停在非 0 (实测拒了就一直是 2, 两分钟后再看还是 2), 只有下一条指令被
            **接受**才清零。所以单看它分不清「刚被拒」和「两分钟前被拒还没散」。

        结案 (置 _ik_watch_done) 之后不再重复判断: 判决是**逐条指令**的, 而 ik_status
        粘着不散, 不结案的话后续每一条新指令刚发出的一瞬间都会被这条旧判决误伤。

        「粘住」这件事和上面那个 6ms 合起来说明: **不需要**再加一个「判决基线比对」
        (记下发送时的 ik_status, 只在它变了才归因到这一条) —— 清零点就在接受那一刻,
        中间没有可误判的窗口。2026-09-21 曾计划加一个 `ik_verdict_stale_grace` 参数,
        被这次实测否掉了, 别再加回来。
        """
        if self._ik_watch_done or self._ik_watch_since is None:
            return None
        if self._arm_status is None:
            return None
        if not self.get_parameter("ik_verdict_enabled").value:
            self._ik_watch_done = True
            return None
        now = self.get_clock().now().nanoseconds * 1e-9
        grace = float(self.get_parameter("ik_verdict_grace").value)
        if now - self._ik_watch_since < grace:
            return None
        stamp, status = self._arm_status
        if stamp < self._ik_watch_since:
            # 还没有指令之后的快照: 继续等, 由 motion_timeout 兜底 (不误判)
            return None
        verdict = int(getattr(status, "ik_status", 0))
        if verdict == 0:
            # 固件说收下了。**不结案也有讲究**: 结案是「判过了」的留痕, 而这里只是
            # 「眼下没问题」, 后面机械臂真不动了还得靠 stall / motion_timeout 说话。
            self._ik_watch_done = True
            return None
        moved = self._moved_since_command()
        if moved is None:
            # 「动没动」这条证据取不到 (没有 TCP 反馈, 或这条指令本来就不需要动)。
            # 取不到证据就不判, 案子留着让 motion_timeout 兜底。
            return None
        # 到这里「动没动」有结论了, 这条指令的判决结案 —— 不管结论落在哪一边。
        # 动过了 = 固件确实收下了这个目标, 它嘴上挂着的非 0 值属于更早的某条指令。
        self._ik_watch_done = True
        if moved:
            return None
        # 上报系的目标比夹爪系更适合写进日志 —— 它才是参数/固件/GUI 的那 6 个数。
        target = self._goal_position if self._goal_position is not None else self._last_command_position
        if target is None or self._sent_orientation is None:
            target_text = "未知"
        else:
            target_text = (
                f"上报原点="
                f"{np.round(self._gripper_to_reported(target, self._sent_orientation), 4)} m"
            )
        retry_note = (
            "; 已经原样重发过一次, 还是被拒 —— 大概率是这组数在这个构型下**系统性**"
            "不可达, 别再原地重试, 换个数或换个中间路点"
            if self._ik_retry_used
            else ""
        )
        return (
            f"固件 IK 拒收 ({_ik_status_text(verdict)}): "
            f"MCU 没接受这个目标, 从指令发出到现在机械臂一步没动{retry_note} —— "
            f"换一组目标位姿 (IK 是逐次判决的: 同一组数在不同关节构型下结果不同, "
            f"2026-09-21 实测同一个放置位姿先被拒、半小时后同样的数又被收下); "
            f"本条目标 {target_text}, 姿态="
            f"{None if self._sent_orientation is None else np.round(self._sent_orientation, 4)}"
        )

    def _retry_rejected_command(self):
        """被固件拒收之后: 把**同一条**指令原样重发一次。返回 True = 已重发, 别判失败。

        ⚠ 走 J 的那一段**到不了这里** (2026-09-28): J 指令在 `_send_move_p` 里不立固件
        IK 判决的案, 所以 `_ik_rejection_problem` 永远不会为它结案。理由见那里 ——
        固件对 J 模式不跑逆解, ik_status 对 J 没有可信判决, 立案只会误判。
        真机上验过「J 指令被拒时固件到底报什么」之后, 这里可能改成「J 被拒后回落
        move_p 重试」; 在那之前 J 段的失败只能是 motion_timeout。

        为什么值得重发 (2026-09-21 真机证据): 判决是**逐次**的, 同样的数在不同关节构型
        下结论可能不同 —— 待机位那条指令 10:16 被拒 (机械臂停在半路), 10:17 **一个数都
        不改**重发, 4.8 秒到位、误差 2.4 mm; 放置点那组数 10:16 被拒、10:48 原样收下。
        而 10:48 的 HOME 第 1 段被拒之后没人重发, 机械臂就搁在释放点了。

        额度: **每条指令一次** (_ik_retry_used)。重发用的是 _last_command_position, 它正是
        这条指令真正下发的那个点 (夹爪系), 所以重发的是同一件事, 不是「另一个目标」。
        重发之后要重新起停稳判定 (_arm_stall_detection): 相当于刚发了一条新指令。

        模式也要一起原样重发 (_last_command_linear): 下降 / 抬升那条被拒的是 L 指令,
        用 P 重发就不是「同一件事」了 —— 路径形状变了, 而且下降中若中途从直线折成
        点到点, 夹爪可能是斜着扎到瓶子上的。这里**不能**问 _motion_is_linear(): 判决
        是异步结案的 (ik_verdict_grace 之后), 结案时状态可能已经推进, 问出来的是
        「现在」该走什么模式, 不是这条指令当初走的是什么。

        取不到「刚发的是什么」时**不猜着重发** (返回 False): 宁可按拒收判失败, 也不乱发
        一条来路不明的指令。
        """
        if self._ik_retry_used:
            return False
        if self._last_command_joint is not None:
            # 防御 (见 docstring): 正常情况下 J 指令不立案、到不了这里。真要有一天给 J
            # 立了案, **也不要**原样重发同一个关节角 —— J 模式没有「逐次重判」这回事,
            # 该换的是下发方式, 不是重复一遍。返回 False 让调用方按拒收判失败。
            self.get_logger().error(
                "内部错误: 走 J 的指令进了原样重发通道 —— J 模式没有逐次重判, "
                "不重发, 按拒收判失败 (请检查 _send_move_p 的立案条件)"
            )
            return False
        command = self._last_command_position
        orientation = self._sent_orientation
        if command is None or orientation is None:
            return False
        target_text = np.round(
            self._gripper_to_reported(np.asarray(command, dtype=float), orientation), 4
        )
        self.get_logger().warning(
            f"固件拒收了这一条指令, 原样重发一次 "
            f"({'move_l' if self._last_command_linear else 'move_p'}, "
            f"上报原点={target_text} m): "
            f"判决是逐次判的, 同样的数换个瞬间常被收下 —— 2026-09-21 实测待机位那条 "
            f"10:16 被拒、10:17 原样重发 4.8s 到位。再被拒就按失败处理"
        )
        self._send_move_p(
            np.asarray(command, dtype=float),
            tuple(orientation),
            is_retry=True,
            linear=self._last_command_linear,
        )
        self._arm_stall_detection()
        return True

    def _moved_since_command(self):
        """本条指令发出到现在, 机械臂动过没有: True / False, 取不到证据时 None。

        「没动」是判固件拒收的**独立证据** (见 _ik_rejection_problem 的判据 4)。
        两种情况下这条证据不成立, 返回 None:
          - 没有 TCP 反馈 (_ik_watch_pose 或 _tcp_pose 为 None) —— 没法比;
          - 这条指令要求的位移本来就不足 stall_detect_epsilon。原地微调的指令「没动」
            是正常的, 拿它当证据会把一条微调指令误判成拒收。
        阈值跟 stall/settle 用同一个 stall_detect_epsilon, 免得现场两处口径不一致。
        """
        if self._tcp_pose is None or self._ik_watch_pose is None:
            return None
        epsilon = float(self.get_parameter("stall_detect_epsilon").value)
        # 用 _last_command_position 而不是 _goal_position: 前者就是**这条**指令下发的那
        # 个点 (_send_move_p 里和 _ik_watch_pose 同一拍记的), 后者是「当前路点的标称
        # 目标」, 换段的那一拍可能还是上一段的值 —— 拿它算位移会算出个假的「不用动」,
        # 于是拒收漏判 (慢 30s), 与「宁可慢也不误判」同向, 但没必要吃这个亏。
        requested = self._last_command_position
        if requested is not None:
            if float(
                np.linalg.norm(np.asarray(requested, dtype=float) - self._ik_watch_pose)
            ) <= epsilon:
                return None
        pose = np.asarray(self._tcp_pose[1], dtype=float)
        return bool(float(np.linalg.norm(pose - self._ik_watch_pose)) > epsilon)

    def _reset_ik_watch(self):
        """清掉待判的 IK 判决 (任务开始/结束时调用)。"""
        self._ik_watch_since = None
        self._ik_watch_done = True
        self._ik_watch_pose = None
        self._ik_retry_used = False
        # 关节那一路也要清: `_last_command_joint` 会决定 `_active_motion_timeout` 用哪套
        # 预算, `_lift_seed_joint_deg` 是上一轮的悬停解 —— 留着它就是拿上一轮的构型
        # 当这一轮的种子 (循环模式下瓶子已经换了), 那是灾难。
        self._last_command_joint = None
        self._ik_watch_joint = None
        self._lift_seed_joint_deg = None

    def _workspace_problem(self, target, label="目标"):
        """纯谓词: 越界返回描述串, 在界内返回 None。**不触发 _fail** (加载期要用)。"""
        if not np.all(np.isfinite(target)):
            return f"{label}坐标含非有限值"
        limits = {
            "x": (self.get_parameter("workspace_x_min").value, self.get_parameter("workspace_x_max").value),
            "y": (self.get_parameter("workspace_y_min").value, self.get_parameter("workspace_y_max").value),
            "z": (self.get_parameter("workspace_z_min").value, self.get_parameter("workspace_z_max").value),
        }
        for axis, (low, high) in limits.items():
            value = target["xyz".index(axis)]
            if not (low <= value <= high):
                return f"{label} {axis}={value:.3f} 超出 [{low}, {high}]"
        return None

    def _check_workspace(self, target, label="目标"):
        problem = self._workspace_problem(target, label)
        if problem is not None:
            self._fail(f"WORKSPACE_REJECTED {problem}")
            return False
        return True

    def _check_workspace_pair(self, gripper_target, quaternion):
        """夹爪目标与它对应的上报原点都要在工作空间内。

        两者相距 |gripper_offset| (本机 0.08 m), 只查一个可能放过另一侧越界。
        失败返回 False (内部已 _fail)。
        """
        if not self._check_workspace(gripper_target, label="夹爪"):
            return False
        origin = self._gripper_to_reported(gripper_target, quaternion)
        return self._check_workspace(origin, label="上报原点")

    def _check_distance(self, reported_target, enforce_max=True):
        """reported_target 是上报末端原点 (与 /control/move_p 同系)。

        比较也在上报系做: 这才是机械臂真正要走的位移。用夹爪点比较会因
        0.08 m 的偏置低估/高估实际行程。

        enforce_max=False 只跳过**距离上限**这一条判断, 「无 TCP 反馈就拒绝发送」
        照旧生效 —— 调用方要的是豁免上半段那个防感知跳变的上限, 不是豁免反馈检查。
        """
        if self._tcp_pose is None or self._tcp_orientation is None:
            self._fail("无 TCP 反馈, 拒绝发送命令")
            return False
        if not enforce_max:
            return True
        current = self._gripper_to_reported(self._tcp_pose[1], self._tcp_orientation)
        distance = float(np.linalg.norm(current - reported_target))
        max_distance = self.get_parameter("max_command_distance").value
        if distance > max_distance:
            self._fail(
                f"单段命令距离 {distance * 1000.0:.0f} mm 超过 "
                f"{max_distance * 1000.0:.0f} mm"
            )
            return False
        return True

    # ── 辅助 ──────────────────────────────────────────────

    def _lift_orientation(self):
        """LIFT 阶段实际下发的姿态 (x, y, z, w)。

        真机用当前 TCP 姿态 (从夹爪当前位置直接抬起, 不用可能过期的目标);
        dry_run 无真实反馈时退回抓取姿态。

        **回正那条路例外**: 那一段的姿态是显式指定的, 最后一个路点就是它。这一条在
        dry_run 下也照实返回 —— `_enter_place` 拿它算「全程同时转腕」, 退回抓取姿态的话
        干跑日志里永远是 144°, 回正到底生效没有就看不出来了。
        """
        if self._waypoint_orientations:
            return self._waypoint_orientations[-1]
        if not self.get_parameter("dry_run").value and self._tcp_orientation is not None:
            return self._tcp_orientation
        return self._goal_orientation

    @staticmethod
    def _orientation_angle(first, second):
        """两个四元数之间的夹角 (弧度, 0..π); 任一为 None 时返回 None。

        用两个旋转之间的相对旋转角: |dot| 已经把 q 与 -q 是同一个旋转这件事算进去了
        (点积取绝对值), 所以不必先规范化 w 的符号。数值略微超出 [-1,1] 时夹一下 ——
        浮点误差会让 acos 拿到 1.0000000000000002 然后返回 nan。
        """
        if first is None or second is None:
            return None
        dot = abs(sum(float(a) * float(b) for a, b in zip(first, second)))
        return math.acos(max(-1.0, min(1.0, dot))) * 2.0

    def _active_orientation(self):
        """本次路点实际下发的姿态。"""
        if self._state == self._STATE_HOME:
            # 待机姿态是用户整个设定的 (含 RX/RY/RZ), 必须原样下发 ——
            # 用 _lift_orientation() 那种「当前 TCP 姿态」会把回位变成纯平移,
            # 到头来停在别的姿态上; 而 _goal_orientation 在 WAIT_TARGET 失败时
            # 还是 None, 会直接让计时器回调抛异常拖垮节点。
            return self._standby_orientation
        if self._state == self._STATE_LIFT:
            # 回正时每个路点姿态都不同 (见 _waypoint_orientations), 得按当前路点取。
            if self._waypoint_orientations:
                index = min(self._waypoint_index, len(self._waypoint_orientations) - 1)
                return self._waypoint_orientations[index]
            return self._lift_orientation()
        if self._state == self._STATE_PLACE_MOVE:
            # 搬运姿态: 参数里指定的那一个 (_place_orientation, 加载期算好), 不是每段
            # 重新读当前 TCP 姿态 —— 理由见 _enter_place。
            return self._place_orientation
        return self._goal_orientation

    def _active_tolerance(self):
        """当前阶段的位置到位容差 (m)。"""
        if self._state == self._STATE_HOME:
            return self.get_parameter("standby_tolerance").value
        return self.get_parameter("position_tolerance").value

    def _active_settle_time(self):
        """当前阶段的到位保持时间 (s)。"""
        if self._state == self._STATE_HOME:
            return self.get_parameter("standby_settle_time").value
        return self.get_parameter("settle_time").value

    def _active_motion_timeout(self):
        """当前这一条指令的到位超时 (s)。

        只有回位会放大: 老代码一条回位拆成 N 段、每段各给一个 motion_timeout, 现在
        合成一条绝对指令, 时间预算得按同一个 N 还回去 (1.3 m 的回位 = 4×30s), 否则
        远距离回位会从「走得慢」变成「没收敛」。倍数由 _home_time_multiplier_for 算,
        并在 _transition 离开 HOME 时清 1 —— 别让回位的预算漏到后面的运动阶段。

        走 J 的那一段用 `move_j_timeout`: J 的速度语义与 P/L 不同 (enter_can_mode 对 J
        的钳位是 0..100, 对 P/L/C 是 0..120), 而且 `_home_time_multiplier_for` 是按
        **笛卡尔距离**估段数的 —— 「腕部要转 180° 但末端几乎没动」时它会算出 1 段,
        只给 30s 就可能假超时。实测标定之前先给一个独立的保守值。
        """
        if self._last_command_joint is not None:
            base = float(self.get_parameter("move_j_timeout").value)
        else:
            base = float(self.get_parameter("motion_timeout").value)
        if self._state == self._STATE_HOME:
            return base * max(int(self._home_time_multiplier), 1)
        return base

    def _motion_reached(self):
        """固件自报的「已到达」(motion_status == 0)。

        仅在 use_motion_status 打开时作为额外判据; 反馈缺失时返回 False 表示
        尚未确认到位, 由 motion_timeout 兜底而不是默默放行。
        """
        if not self.get_parameter("use_motion_status").value:
            return True
        if self._arm_status is None:
            return False
        return self._arm_status[1].motion_status == 0

    def _resolve_orientation(self):
        """确定本次抓取使用的 TCP 姿态 (x, y, z, w); 失败返回 None (内部已 _fail)。"""
        source = self.get_parameter("orientation_source").value
        if source == "current_tcp":
            if self._tcp_orientation is not None:
                return self._tcp_orientation
            if self.get_parameter("dry_run").value:
                # dry_run 不下发机械臂命令, 无 TCP 反馈时退化为占位姿态只为打印
                self.get_logger().warn(
                    "dry_run 下未收到 /feedback/tcp_pose, 暂用 side/top_orientation_xyzw "
                    "仅作占位显示 (不会下发机械臂)"
                )
                return self._orientation_for_mode(self._grasp_mode)
            self._fail(
                "orientation_source=current_tcp, 但尚未收到 /feedback/tcp_pose, "
                "无法确定抓取姿态"
            )
            return None
        if source != "fixed":
            self._fail(f"未知的 orientation_source: {source!r} (可选 current_tcp / fixed)")
            return None
        return self._orientation_for_mode(self._grasp_mode)

    def _skip_lift_for_mode(self):
        """倒瓶 (top) 抓完**不抬升**, 直接去放置 (2026-09-28, 用户要求)。

        为什么单独给 top: 那一模式是工具朝下抓, 夹爪本来就停在物体正上方; 先抬 10cm
        再横着送到放置点, 对「放到某处」没有帮助 —— 多走一段, 还多经过一个可能解不
        出来的位姿。直接进放置时起点就是当前 TCP (_enter_place 就是这么取的)。
        (side 不动: 从侧面抓, 先把物体提离台面是必要的。)

        ⚠ **2026-09-30 现场推翻了「不抬升」这个前提**: 跳过抬升之后, 放置那一条指令
        要从**离台面 6.7cm 的抓取点**同时走 34.6cm 位移 + 144° 转腕, IK 解不出来
        (残差 3.19mm/16.47°) -> 回落 move_p 交给固件规划 -> 大摆 -> 电机报错
        `err_status=512` 收场。抬升恰好是「让后面那条长位移有一个解得出的起点」。

        **需要回正时一律不跳** (回正折在抬升里, 没有抬升就没有它, 而且那 146° 的翻转
        绝不能在贴着台面的高度做) —— 哪怕 `top_skip_lift` 还开着。

        **没配放置点时不跳**: 跳了会变成「夹着瓶子原地进 IDLE」, 比多抬一次糟得多。
        """
        if not self.get_parameter("top_skip_lift").value:
            return False
        if str(self._grasp_mode) != "top":
            return False
        if self._lift_align_orientation() is not None:
            self.get_logger().warn(
                "top_skip_lift=true, 但这一轮需要回正姿态 —— 仍然先抬升 "
                "(回正折在抬升里; 而且那一下要转大半个腕, 贴着台面做太危险)"
            )
            return False
        if not self._place_ready:
            self.get_logger().warn(
                "top_skip_lift=true 但放置点不可用 —— 仍然先抬升 "
                "(不跳: 跳了会夹着瓶子停在原地)"
            )
            return False
        return True

    def _skip_track_for_mode(self):
        """倒瓶 (top) **不跟踪**: 悬停到位直接下降抓取 (2026-09-29, 用户要求)。

        为什么只给 top: 那一模式是工具朝下、从**正上方**抓, 机械臂一挪到瓶子正上方,
        手眼相机就看不着瓶子了 —— 现场表现就是「到达瓶子上方就识别不到」: 候选不再
        更新, _follow_target 按 max_result_age 判 TRACK_TARGET_LOST 回待机位, 一轮都
        抓不成 (2026-09-29 现场)。而 top 的悬停高度本来就只有一个小量
        (top_pre_grasp_offset_z, 现场 1cm), 悬停点与抓取点之间几乎没有横向差别 ——
        跟随在这里没有东西可跟, 下降点用**锁定目标那一刻**的坐标就够了
        (_tick_wait_target 里过完稳定窗口的那一份, 也是全流程唯一被稳定性筛过的坐标)。
        (side 不动: 从**侧面**抓, 相机看得见瓶子, 跟随仍是「瓶子挪到哪就抓到哪」的
        唯一手段。)

        纯谓词, **不打日志**: 它由 _tick_motion 在 PRE_GRASP 期间每拍调用一次
        (50ms 一拍), 日志放在真正转状态的那一处 (_advance_after_motion)。
        需要「要不要进 TRACK_WAIT」的整体判决时用 _track_wanted, 别在各处自己组合。
        """
        return bool(self.get_parameter("top_skip_track").value) and str(
            self._grasp_mode
        ) == "top"

    def _track_wanted(self):
        """悬停到位后要不要进 TRACK_WAIT (两处调用点共用这一份判决)。

        _tick_motion 用它选下一状态, _advance_after_motion 用它决定要不要写「为什么
        没跟」那行日志 —— 判决只此一份, 日志那一处只把**原因**写出来, 不另判一次。
        (分开写迟早会两边打架, 而这里的每一条都直接决定机械臂往哪走。)
        """
        if not self.get_parameter("track_enabled").value:
            return False
        return not self._skip_track_for_mode()

    def _pre_grasp_offset_z(self):
        """悬停高度 (m)。top 模式可以单独给一个 —— 见参数的注释。

        与 _orientation_for_mode 同一个套路: 按 grasp_mode 在「通用值 / 专用值」之间选,
        专用值没设 (<= 0) 就退回通用值 —— 这样不配 top_pre_grasp_offset_z 时行为与
        改动前逐字相同。
        """
        if str(self._grasp_mode) == "top":
            top = float(self.get_parameter("top_pre_grasp_offset_z").value)
            if top > 0.0:
                return top
        return float(self.get_parameter("pre_grasp_offset_z").value)

    def _orientation_for_mode(self, mode):
        key = "top_orientation_xyzw" if mode == "top" else "side_orientation_xyzw"
        quaternion = [float(v) for v in self.get_parameter(key).value]
        norm = math.sqrt(sum(v * v for v in quaternion))
        if norm < 1e-9:
            self._fail(f"{key} 四元数为零")
            # 必须 None: 调用方 (_tick_wait_target) 靠它判断「内部已 _fail, 本 tick
            # 到此为止」, 返回一个可用四元数会让那一 tick 继续跑到 _transition(PRE_GRASP),
            # 把刚判定失败的任务复活, 也会冲掉 HOME 状态。
            return None
        return tuple(v / norm for v in quaternion)

    @staticmethod
    def _quaternion_to_matrix(quaternion):
        """(x, y, z, w) 单位四元数 → tool 到 base 的 3x3 旋转矩阵。"""
        x, y, z, w = quaternion
        return np.asarray(
            (
                (1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)),
                (2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)),
                (2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)),
            ),
            dtype=np.float64,
        )

    @staticmethod
    def _matrix_to_quaternion(rotation):
        """3x3 旋转矩阵 → (x, y, z, w) 单位四元数。"""
        matrix = np.asarray(rotation, dtype=np.float64)
        trace = float(np.trace(matrix))
        if trace > 0.0:
            scale = math.sqrt(trace + 1.0) * 2.0
            quaternion = (
                (matrix[2, 1] - matrix[1, 2]) / scale,
                (matrix[0, 2] - matrix[2, 0]) / scale,
                (matrix[1, 0] - matrix[0, 1]) / scale,
                0.25 * scale,
            )
        else:
            index = int(np.argmax(np.diag(matrix)))
            if index == 0:
                scale = math.sqrt(
                    1.0 + matrix[0, 0] - matrix[1, 1] - matrix[2, 2]
                ) * 2.0
                quaternion = (
                    0.25 * scale,
                    (matrix[0, 1] + matrix[1, 0]) / scale,
                    (matrix[0, 2] + matrix[2, 0]) / scale,
                    (matrix[2, 1] - matrix[1, 2]) / scale,
                )
            elif index == 1:
                scale = math.sqrt(
                    1.0 + matrix[1, 1] - matrix[0, 0] - matrix[2, 2]
                ) * 2.0
                quaternion = (
                    (matrix[0, 1] + matrix[1, 0]) / scale,
                    0.25 * scale,
                    (matrix[1, 2] + matrix[2, 1]) / scale,
                    (matrix[0, 2] - matrix[2, 0]) / scale,
                )
            else:
                scale = math.sqrt(
                    1.0 + matrix[2, 2] - matrix[0, 0] - matrix[1, 1]
                ) * 2.0
                quaternion = (
                    (matrix[0, 2] + matrix[2, 0]) / scale,
                    (matrix[1, 2] + matrix[2, 1]) / scale,
                    0.25 * scale,
                    (matrix[1, 0] - matrix[0, 1]) / scale,
                )
        quaternion = np.asarray(quaternion, dtype=np.float64)
        quaternion /= np.linalg.norm(quaternion)
        if quaternion[3] < 0.0:
            quaternion = -quaternion
        return tuple(float(value) for value in quaternion)

    def _gripper_offset_in_base(self, quaternion):
        """tool 系下的夹爪偏移转到 base_link 系 (3,)。"""
        return self._quaternion_to_matrix(quaternion) @ self._gripper_offset

    def _reported_to_gripper(self, position, quaternion):
        """上报末端原点 -> 夹爪位姿 (都在 base_link 系)。

        ── 临时方案, 2026-09-15 起, 2026-09-17 实测定值 ──
        H3001 上报的末端原点不在夹爪上, 而在夹爪之外一段几厘米级的距离上
        (gripper_offset_tool_xyz, 单位 m, tool 坐标系; 2026-09-21 起默认 [0, 0, 0.08],
        与 yaml 现场值一致)。真机日志里那行「偏移=[..] (模长 N mm)」就是把**当前参数
        值**换到 base 系的结果 —— 它是参数的回显, 不是独立测量, 别拿它当标定依据。
        所以「夹爪落在哪」= 上报位置 + R @ offset。

        纯平移不改变姿态, 因此夹爪系与上报系共用同一个四元数。

        新版本机械臂若不出现这个问题, 把 gripper_offset_tool_xyz 置零即可 ——
        两个换算函数在零偏置下退化为恒等, 这段逻辑不用删。
        注意上游 h3001_bridge 已修正固件的 rx/ry 约定, 这里的四元数可以直接用。
        """
        return np.asarray(position, dtype=np.float64) + self._gripper_offset_in_base(
            quaternion
        )

    def _gripper_to_reported(self, position, quaternion):
        """夹爪位姿 -> 上报末端原点, 即 _reported_to_gripper 的逆变换。"""
        return np.asarray(position, dtype=np.float64) - self._gripper_offset_in_base(
            quaternion
        )

    def _motion_is_linear(self):
        """当前状态该不该走直线 (MOVE_L)。唯一的开关是 _LINEAR_MOTION_STATES。"""
        return self._state in self._LINEAR_MOTION_STATES

    def _to_reported_pose(self, position, quaternion):
        """夹爪目标 -> IK 要的 `[X,Y,Z mm, RX,RY,RZ deg]` (上报原点系, ZYX-RPY)。

        位置用现成的 `_gripper_to_reported`; 姿态要把四元数拆成 ZYX-RPY —— 节点里
        原本只有反向的 `_rpy_to_quaternion`, 这里是它的**精确逆**, 公式照抄
        `h3001_bridge._pose_to_list` (桥两个方向都用这一套, 所以三方一致)。

        `ik_target_invert_rx_ry` 只在桥打开了 `invert_rx_ry` 时才为真 —— 那时桥的
        反馈把 rx/ry 编成了取反的值, 这里要还原回固件 `ArmState.pose` 的约定,
        否则 IK 解的是**另一个位姿**。取反是自逆的, 所以两处用同一个布尔。
        """
        x, y, z = self._gripper_to_reported(position, quaternion)
        qx, qy, qz, qw = (float(v) for v in quaternion)
        roll = math.atan2(
            2.0 * (qw * qx + qy * qz), 1.0 - 2.0 * (qx * qx + qy * qy)
        )
        sin_pitch = max(-1.0, min(1.0, 2.0 * (qw * qy - qz * qx)))
        pitch = math.asin(sin_pitch)
        yaw = math.atan2(
            2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz)
        )
        if self.get_parameter("ik_target_invert_rx_ry").value:
            roll, pitch = -roll, -pitch
        return [x * 1000.0, y * 1000.0, z * 1000.0,
                math.degrees(roll), math.degrees(pitch), math.degrees(yaw)]

    def _joint_seed_deg(self):
        """IK 的种子: 当前关节角换算到 URDF 约定; 没有/太旧就返回 None。"""
        if self._joint_deg is None:
            return None
        stamp, deg = self._joint_deg
        age = self.get_clock().now().nanoseconds * 1e-9 - stamp
        if age > float(self.get_parameter("ik_seed_max_age").value):
            return None
        return np.asarray(
            self.get_parameter("ik_joint_sign").value, dtype=float
        ) * deg

    def _solve_joint_target(self, position, orientation, *, seed_deg=None, align=False):
        """位姿 -> 关节目标 (话题约定, 度)。**过不了任何一道守门就返回 (None, 原因)**。

        这是「把固件原先替我们做的三道判决全接过来」的地方。最危险的不是解不出来
        (那会回落 move_p, 安全), 而是**解出来了但解错了** —— 三道守门里 ②③ 专治它:

          ① 收敛 (`ok`)            —— 对应固件的 `ik_status=2 (超出工作空间)`;
          ② CAD 限位余量 ≥ 阈值    —— 固件那道「关节限位」; 注意求解时 bounds 已经是
                                     CAD 限位 (URDF 那套松 30°), 这一条是"不要贴着限位";
          ③ 与种子的单轴差 ≤ 阈值  —— 对应固件的 `ik_status=5 (关节单步过大)`:
                                     本地求解会在解断裂处跳到另一条分支, 而那种解
                                     **收敛、误差 0、余量还有几十度** (实测 J4 跳 117.6°
                                     时余量 34.7°), 只有这一条抓得到。

        `align=True` **只给「抬升时顺带回正」那一次规划用** (见 _lift_align_waypoints):
        那一步的腕部大角度就是回正本身, ③ 收窄成只守肩肘 (J1/J2/J3)。**下发出去的路点
        一律走完整的三道门** —— 它们按关节直线拆开, 每段单轴差 ≤ 总差 / 段数, 本来就
        过得去; 会放宽的只有这一次不出门的规划。
        """
        if not self._ik_ready:
            return None, "求逆解不可用 (见启动日志)"
        solved = seed_deg if seed_deg is not None else self._joint_seed_deg()
        if solved is None:
            return None, "没有新鲜的关节反馈当种子"
        module, (model, data, fid) = self._ik_module, self._ik_model
        pose = self._to_reported_pose(position, orientation)
        try:
            result = module.ik(
                model, data, fid, pose,
                seed_deg=solved,
                limits=(self._ik_cad_lo, self._ik_cad_hi),
                restarts=0,   # 主循环里写死 0: 实测最坏 13ms, 装得下一拍
            )
        except Exception as exc:  # noqa: BLE001 - 求解器抛异常不该拖垮 tick
            return None, f"求解异常 ({exc})"
        if not result["ok"]:
            return None, (
                f"没收敛 (残差 {result['pos_err_mm']:.2f}mm/"
                f"{result['rot_err_deg']:.2f}°; 多半是这一步跨了构型断裂处)"
            )
        joint = np.asarray(result["q_deg"], dtype=float)
        margin = float(
            np.minimum(joint - np.degrees(self._ik_cad_lo),
                       np.degrees(self._ik_cad_hi) - joint).min()
        )
        if margin < float(self.get_parameter("ik_min_margin_deg").value):
            return None, f"距 CAD 限位只剩 {margin:.1f}°"
        limit = float(self.get_parameter("ik_max_joint_jump_deg").value)
        delta = np.abs(joint - solved)
        if align:
            # 回正这一次规划: 腕部 (J4/J5/J6) 的大角度**就是回正本身** —— 从工具朝下的
            # 倒瓶抓取姿态转到 side 姿态, J6 非转 146° 不可, 拿 60° 去卡它只会把整段
            # 打回 move_p, 也就是 2026-09-30 那次大摆的来路。这里守的是**肩肘 (J1-J3)**:
            # 它们大幅变化才是真的换构型、才会把整条手臂抡起来。收窄用另外两道补:
            # ② CAD 余量照查, ③ 之后在 _lift_align_waypoints 里还有一道 FK 复核。
            jump = float(delta[:3].max())
            if jump > limit:
                return None, f"肩肘单轴差 {jump:.0f}° (换构型, 会被原样执行成大摆)"
            self.get_logger().info(
                f"回正规划: 腕部单轴差 {float(delta[3:].max()):.0f}° (J6 是回正本身, 不按 "
                f"{limit:.0f}° 卡), 肩肘 {jump:.0f}°, 余量 {margin:.1f}°"
            )
        else:
            jump = float(delta.max())
            if jump > limit:
                return None, f"与当前构型单轴差 {jump:.0f}° (换构型, 会被原样执行成大摆)"
        # IK 给的是 URDF 约定, 话题上要的是另一套 (差三个符号)。
        # 顺手记下 URDF 那一份: LIFT 要拿「悬停那一段的解」当种子 (它是 URDF 约定)。
        self._last_solved_urdf_deg = joint
        return np.asarray(
            self.get_parameter("ik_joint_sign").value, dtype=float
        ) * joint, None

    def _ik_selfcheck(self):
        """运行期自检: 拿当前反馈正解一次, 与上报位姿比位置。

        **一次覆盖三种致命配置错误**: URDF 版本不对 / `ik_joint_sign` 方向错 /
        ee_frame 选错。三者都会让 IK 解出「另一个位姿」的解 —— 那正是最危险的情形
        (收敛、误差 0、余量也够, 但机械臂会走去别的地方), 回落 move_p 救不了它。
        发现不一致就**永久降级**回 move_p 并大声报错。

        只在 IDLE 跑: 那两个反馈来自桥的同一帧, 但经两条话题到达节点, 机械臂**在动**
        的时候它们可能不是同一瞬间的 (差几毫米), 那样会误报。IDLE 时机械臂静止,
        两边必然一致 —— 而且节点启动后就停在 IDLE, 第一次任务之前就能查出来。
        """
        if not self._ik_ready or self._state != self._STATE_IDLE:
            return
        if self._joint_deg is None or self._tcp_pose is None or self._tcp_orientation is None:
            return
        self._ik_selfcheck_counter += 1
        if self._ik_selfcheck_counter < 20:      # 20 拍 = 1s
            return
        self._ik_selfcheck_counter = 0
        module, (model, data, fid) = self._ik_module, self._ik_model
        seed = self._joint_seed_deg()
        if seed is None:
            return
        try:
            fk = np.asarray(module.forward_kinematics(model, data, fid, seed), dtype=float)
        except Exception:  # noqa: BLE001 - 自检本身不该抛
            return
        reported = self._gripper_to_reported(self._tcp_pose[1], self._tcp_orientation)
        error_mm = float(np.linalg.norm(fk[:3] - reported * 1000.0))
        if error_mm <= 2.0:
            return
        self._ik_disable(
            f"运行期自检不通过: URDF 正解({np.round(fk[:3], 1)}) 与上报位姿"
            f"({np.round(reported * 1000.0, 1)}) 差 {error_mm:.1f} mm —— 说明 URDF 版本 / "
            f"ik_joint_sign / 末端 frame 有一个对不上; 绝不能拿它求逆解"
        )

    def _ik_motion_for_state(self):
        """这一段该不该走 J。四个条件全成立才行。"""
        if not (self._ik_ready and self.get_parameter("ik_motion_enabled").value):
            return False
        if self._state not in self._IK_MOTION_STATES:
            return False
        # 盲走回位永远走 move_p: 它的前提就是反馈已经不可信, 而 IK 的种子正是反馈
        # (两者出自桥的同一次发布), 拿不可信的东西当种子是最危险的。
        if self._state == self._STATE_HOME and self._home_blind:
            return False
        return True

    def _publish_move_j(self, joint_deg):
        """把关节目标 (话题约定, 度) 发到 /control/move_j。单位是**弧度**。"""
        message = JointState()
        message.header.stamp = self.get_clock().now().to_msg()
        message.name = list(_JOINT_NAMES)
        message.position = [math.radians(float(v)) for v in joint_deg]
        self._move_j_pub.publish(message)

    def _send_move_p(self, position, orientation, is_retry=False, linear=None):
        """position 是 **夹爪** 目标 (base_link, m); 下发前换算成上报末端原点。

        linear 决定走哪条话题 / 固件模式:
          - None (默认): 按**当前状态**问 _motion_is_linear() —— 给所有按状态走的
            调用点用, 这样加减线性段只改 _LINEAR_MOTION_STATES。
          - True / False: 显式指定。只有 _retry_rejected_command 用它 —— 那里要按
            **那条被拒指令**的模式重发, 而此刻的状态可能已经变了。

        两种模式收的都是同一个消息和同一套单位, 所以下面只有「选哪个发布器、日志打哪个
        名字」两处不同。其余的 (IK 判决立案、dry_run 短路) 与模式无关, 一律不分叉。

        名字里的 move_p 是历史遗留 —— 它现在是「运动指令」的唯一出口 (move_p + move_l)。
        改名要动 6 个调用点加六个测试模块里的包装函数, 收益只有好看, 不值。
        """
        if linear is None:
            linear = self._motion_is_linear()
        label = "move_l" if linear else "move_p"
        # 回位起点在 dry_run 下没有反馈可用, 用最后下发的路点代替。
        # 放在 dry_run 分支之前, 两条路径都记。
        self._last_command_position = np.asarray(position, dtype=np.float64)
        self._last_command_linear = bool(linear)
        # ── 优先走关节空间 (2026-09-28) ──
        # **出口仍然只有这一个** (测试与日志都建立在这上面); 这里只是多一种下发方式。
        # 解不出来就一路回落到下面那条笛卡尔的路 —— 而那条路今天完全可用, 所以
        # 「IK 失败」永远不是新的失败模式, 只是少赚一点。
        #
        # 为什么可以**现算**而不缓存: 这个函数是「每条指令调一次」(路点发出、
        # 末端修正迭代、跟随的一条), 不是每拍调 —— 一次 3.9ms 完全无所谓。缓存反而
        # 要处理「修正量改过之后复用旧解」那种空转 (见 _try_endpoint_correction)。
        # `is_retry` 的重发不走这里: 被拒之后要换的是**下发方式**(见
        # _retry_rejected_command), 原样重发同样的关节角没有意义。
        self._last_command_joint = None
        if not is_retry and self._ik_motion_for_state():
            # LIFT 的种子通常用悬停那一段的解 (见 _lift_seed_joint_deg 的注释)。
            # **回正那条路例外**: 它的路点本身就是「当前构型沿关节直线采样」出来的,
            # 种子必须跟着当前构型走 (每段之间只差 ~50°), 拿悬停构型当种子会让最后
            # 一段的单轴差变成整段的总差 146°, 被守门 ③ 拒掉 —— 而 ③ 正是这里唯一
            # 保证「没有跳到别的分支」的东西, 不能为了它放宽。
            seed = (
                self._lift_seed_joint_deg
                if self._state == self._STATE_LIFT
                and self._lift_seed_joint_deg is not None
                and self._waypoint_orientations is None
                else None
            )
            joint, reason = self._solve_joint_target(
                position, orientation, seed_deg=seed
            )
            if joint is not None:
                self._last_command_joint = np.asarray(joint, dtype=float)
                if self._state in (self._STATE_PRE_GRASP, self._STATE_TRACK_WAIT):
                    # 这一条是「去悬停点」的指令 —— 记下它的解给 LIFT 当种子。
                    # (LIFT 从抓取构型出发自己解会卡在关节限位上, 用这个种子解得出。)
                    self._lift_seed_joint_deg = self._last_solved_urdf_deg
                if self.get_parameter("dry_run").value:
                    # dry_run 也要**真的跑一遍 IK** 并把结果打出来 —— 否则这条验证
                    # 白跑 (看日志的人以为在走 J, 其实什么都没算)。
                    self.get_logger().info(
                        f"[dry_run] move_j 目标 (关节角, 度): "
                        f"{np.round(self._last_command_joint, 2)}  "
                        f"(夹爪={np.round(position, 4)} m)"
                    )
                    self._tolerance_start = time.monotonic()
                    self._ik_watch_since = None
                    self._ik_watch_done = True
                    self._ik_watch_pose = None
                    self._ik_watch_joint = None
                    return
                # J 指令**不立固件 IK 判决的案**: 固件对 J 模式不跑逆解, ik_status
                # 会一直是非 0 或一直 0, 那条通道对 J 只会误判 (见 _ik_rejection_problem)。
                self._ik_watch_since = None
                self._ik_watch_done = True
                self._ik_watch_pose = None
                self._ik_watch_joint = (
                    None if self._joint_deg is None else self._joint_deg[1].copy()
                )
                self._publish_move_j(self._last_command_joint)
                self.get_logger().info(
                    f"move_j 已发送 (关节角, 度): "
                    f"{np.round(self._last_command_joint, 2)}  "
                    f"(夹爪={np.round(position, 4)} m, 上报原点="
                    f"{np.round(self._gripper_to_reported(position, orientation), 4)} m)"
                )
                return
            if self._state in (self._STATE_PRE_GRASP, self._STATE_TRACK_WAIT):
                self._lift_seed_joint_deg = None
            self.get_logger().warn(
                f"这一段走 move_j 不成 ({reason}) —— 回落 {label}"
            )
        # 这一条指令的「原样重发」额度: 正常下发先给一次 (被固件拒收时用), **重发本身
        # 不再给** —— 否则被拒一次就会一直重发下去。见 _retry_rejected_command。
        self._ik_retry_used = bool(is_retry)
        reported = self._gripper_to_reported(position, orientation)
        message = PoseStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self.get_parameter("robot_base_frame").value
        message.pose.position.x = float(reported[0])
        message.pose.position.y = float(reported[1])
        message.pose.position.z = float(reported[2])
        message.pose.orientation.x = float(orientation[0])
        message.pose.orientation.y = float(orientation[1])
        message.pose.orientation.z = float(orientation[2])
        message.pose.orientation.w = float(orientation[3])
        if self.get_parameter("dry_run").value:
            self.get_logger().info(
                f"[dry_run] {label} 目标: 夹爪={np.round(position, 4)} m, "
                f"上报原点={np.round(reported, 4)} m, "
                f"姿态={np.round(orientation, 4)}"
            )
            # dry_run 下用固定起点模拟到位计时, _tick_motion 直接推进
            self._tolerance_start = time.monotonic()
            # dry_run 没有固件判决可读, 显式结案 —— 留着上一轮的「待判」状态会让
            # 恢复 dry_run=false 的那次任务拿着旧案卷判失败。
            self._ik_watch_since = None
            self._ik_watch_done = True
            self._ik_watch_pose = None
            return
        # 立案: 这一条指令的 IK 判决由 _ik_rejection_problem 在 ik_verdict_grace 之后结案。
        # 时刻和位置都必须在 publish **之前**取: 判决只可能发生在指令到达之后, 而
        # 「有没有动过」的基准要是 publish 之后那一帧反馈, 手臂刚起步的位移就会被
        # 算进基准里, 拒收反而更容易漏判。
        self._ik_watch_since = self.get_clock().now().nanoseconds * 1e-9
        self._ik_watch_pose = (
            None if self._tcp_pose is None else np.asarray(self._tcp_pose[1], dtype=float)
        )
        self._ik_watch_done = False
        (self._move_l_pub if linear else self._move_p_pub).publish(message)
        self.get_logger().info(
            f"{label} 已发送: 夹爪={np.round(position, 4)} m, "
            f"上报原点={np.round(reported, 4)} m, "
            f"姿态={np.round(orientation, 4)}"
        )


    def destroy_node(self):
        self._shutdown_event.set()
        # 夹爪进程分两种 (2026-09-28):
        #  - **保持中**的常驻进程: 它自己不会退, 一直攥着串口发力。按定下的「退出即
        #    松开」, 这里写一行 OPEN 让它张开再退, 最多等 3s; 还不退就 SIGTERM
        #    (脚本收到信号会失能 → 立刻松开 → 退出)。不这么做的话, 重启节点后第一件
        #    事 (起手张开) 会因为串口被占而 [abort] 退 2。
        #  - 一次性命令: 照旧**不管** —— 它跑完自己会走到 disable 再退出; 杀它反而容易
        #    停在「夹着东西但已断连」的半截状态。节点退出后它被 init 收养。
        job = self._gripper_job
        if job is not None and job.poll() is None:
            if self._gripper_holding:
                self.get_logger().info(
                    "退出时夹爪正在保持 —— 写 OPEN 让它松开 (机械臂若悬空物体会掉, "
                    "这是 gripper_hold_torque 定下的行为)"
                )
                if job.write_line("OPEN"):
                    deadline = time.monotonic() + 3.0
                    while time.monotonic() < deadline and job.poll() is None:
                        time.sleep(0.05)
                if job.poll() is None:
                    # 还没退就走 SIGTERM: 脚本收到信号会失能 → 立刻松开 → 退出,
                    # 串口随之释放 (重启节点时起手张开要它)。
                    self.get_logger().warn(
                        "退出时保持进程仍未退出; SIGTERM 掉它 (会先松开)"
                    )
                    job.kill()
            else:
                # 一次性命令照旧不管: 它跑完自己会 disable 再退出, 最长 gripper_timeout。
                # 杀它反而容易停在「夹着东西但已断连」的半截状态。
                self.get_logger().info(
                    "退出时夹爪脚本仍在跑; 让它自己跑完 (不杀、不等)"
                )
        self._gripper_job = None
        self._gripper_holding = False
        # 上面 set() 的是确认用的 stdin 线程 (_stdin_reader 的 select 周期是 0.2s),
        # 给它一个周期退出, 免得解释器关闭时和它抢 stdin 死锁。
        time.sleep(0.25)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = BottleGraspExecutor()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()