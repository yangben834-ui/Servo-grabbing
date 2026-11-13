#!/usr/bin/env python3
"""bottle_grasp_executor 夹爪接入 (GRIPPER 状态) 的回归测试。

2026-09-17 起节点可以接管 Gloria-M 夹爪, 三次人工确认的语义随之改变:

  execute_grasp        -> GRIPPER(张开)  每轮开始先把夹爪张开到确定状态
  TRACK_WAIT   Enter   -> 下降           (语义不变)
  GRASP_WAIT   Enter   -> GRIPPER(闭合)  -> 命令跑完才 LIFT
  PLACE_WAIT   Enter   -> GRIPPER(张开)  -> 命令跑完才 HOME

2026-09-18 改成**一次性命令** (用户要求「不需要检查夹爪状态, 只需要发送关闭夹爪和
打开夹爪命令」)。真机上那次: 夹爪其实已经夹住了, 脚本却把「0.43s 就撞到硬扭矩上限」
判成失败, 退出码 2, 整趟任务作废。现在三个进入点都是「起一个新进程 -> 跑一条命令
-> 等它退出」, 判据只有退出码, 脚本 stdout 上写什么都不参与判断; 夹持力也不再由
节点维持 (脚本失能退出后靠机构自锁)。

这里钉住的是这套**非阻塞**实现最容易悄悄坏掉的几件事:
  - gripper_enabled=false 时老流程一字不变 (默认值就是 false, 所以这是回归底线);
  - Enter 只**发起**夹爪动作, 命令跑完之前机械臂一动不动;
  - 每趟都必须起**新**进程: 上一趟的进程要先回收, 不能复用;
  - 脚本说什么 (含 [warn] 撞硬扭矩上限) 都不改变成败, 只有退出码算数;
  - dry_run 一行命令都不许发;
  - GRIPPER 期间出事也要回待机位。

不连机械臂、不连 CAN、**不 spawn 真进程**: _spawn_gripper_job 被换成一个返回假
job 的桩, 于是整条状态机可以逐拍驱动。计时全部用「把计时起点往前挪」代替真实 sleep。
(最后那个 RealPipeTest 例外: 它故意起真子进程, 验的是管道装配本身。)

跑法 (在包根目录):
  source /opt/ros/humble/setup.bash && source install/setup.bash
  /usr/bin/python3 -m unittest -v test_gripper
"""

import os
import subprocess
import sys
import time
import unittest
from types import SimpleNamespace

import numpy as np
import rclpy
import yaml

from bottle_grasp_executor.bottle_grasp_executor_node import (
    BottleGraspExecutor,
    spawn_gripper_process,
)

PKG_ROOT = os.path.dirname(os.path.abspath(__file__))

# 夹爪参数里两边必须一致的数值键。**刻意不在里面的两个**:
#   - gripper_enabled: 部署开关, 见 test_enabling_gripper_requires_a_usable_script
#     与 config 注释;
#   - gripper_hold_torque (2026-09-28): 同类 —— 代码默认 0.0 (与改动前逐字相同),
#     现场在 yaml 里开成 1.0。守着它的是 test_hold_torque_stays_below_the_hard_limit。
NEW_GRIPPER_KEYS = (
    "gripper_port",
    "gripper_velocity",
    "gripper_resistance_torque",
    "gripper_hard_torque_limit",
    "gripper_script_timeout",
    "gripper_timeout",
    "gripper_hold_timeout",
    "gripper_hold_kd",
)

GRASP_CENTER = np.asarray((0.30, -0.10, 0.05))
LIFT_START = np.asarray((0.30, -0.10, 0.05))


def _arm_status_msg(motion_status=0):
    """够用的 H3001ArmStatus 替身 (_feedback_problem 与 _ik_rejection_problem 会看这些字段)。

    固件 IK 判决那 4 个字段留默认: ik_status_valid=0 在 _ik_rejection_problem 里
    **一律不判** (SDK 的默认 ik_status 也是 0, 分不清「收下」和「没上报过」), 所以
    这里不填就等于「本次没收到拒收判决」, 用例语义与加字段之前一字不变。
    """
    return SimpleNamespace(
        err_status=0,
        ctrl_mode=1,
        motion_status=motion_status,
        joint_angle_limit=[0] * 6,
        communication_status_joint=[0] * 6,
        ik_status=0,
        ik_status_valid=0,
        ik_joint_limit_fault=0,
        ik_pose_pending=0,
    )


class _FakeJob:
    """_GripperJob 的替身: 不 spawn 进程、不碰串口。

    节点只用到 poll / read_lines / write_line / kill / close 这几样, 加上 mode 与
    exit_code 两个字段, 所以假对象有这些就能把整条状态机驱动起来。

    say() 喂进去的 stdout 行**基本不参与判断** —— 唯一的例外是 `[holding]` 那一行
    (2026-09-28 的保持协议), 见 HoldingTest。用例喂一段脚本输出再断言节点照样只看
    exit_with() 给的那个退出码, 说的是其余那些行 (CloseAtGraspTest 里钉着)。
    """

    def __init__(self, mode):
        self.mode = mode
        self.exit_code = None
        self.killed = False
        self.closed = False
        self.alive = True
        self.written = []          # 节点往 stdin 写过的行
        self.write_fails = False   # 置 True 模拟「写不进去」(管道满 / 对端不读)
        self.exit_on_write = False # 置 True 模拟真脚本: 收到 OPEN 就张开退出
        self._lines = []

    # ── 用例侧: 喂数据 ──
    def say(self, *lines):
        self._lines.extend(lines)

    def exit_with(self, code):
        self.exit_code = code
        self.alive = False

    # ── 节点侧接口 ──
    def poll(self):
        return None if self.alive else self.exit_code

    def read_lines(self):
        lines, self._lines = self._lines, []
        return lines

    def write_line(self, text):
        """真实现的契约是「永不抛异常, 写不进去返回 False」, 假对象照抄。"""
        if self.write_fails or not self.alive:
            return False
        self.written.append(text)
        if self.exit_on_write:
            self.exit_with(0)
        return True

    def kill(self):
        self.killed = True
        self.alive = False

    def close(self):
        self.closed = True


class _NullPublisher:
    """替掉真机的 move_p 发布器: 用例只做记录, **一条指令都不许发出去**。

    2026-09-18 16:49 真机事故: 这些用例是拿 dry_run=False 跑的, 而 `_send_move_p` 在
    dry_run=False 分支里会真的 publish 到 /control/move_p —— h3001_bridge 开着的时候
    跑一遍测试就把机械臂开动了 (详见 test_place.py 里同名类的注释)。
    """

    def __init__(self):
        self.published = []

    def publish(self, message):
        self.published.append(message)


class _GripperTestCase(unittest.TestCase):
    """每个用例一个干净节点; 夹爪脚本被换成假 job, 反馈靠 _feedback() 直接写缓存。"""

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = BottleGraspExecutor()
        # 真机话题隔离: 用例跑起来时 move_p / move_l 一条都不许真发出去 (见 _NullPublisher)。
        self.node._move_p_pub = _NullPublisher()
        self.node._move_l_pub = _NullPublisher()
        # 第三条通路 (move_j, 2026-09-28): 没桩掉的话, 走 J 的用例会**真发**
        # /control/move_j 到桥上 —— 同 2026-09-18 那次事故的来路。
        self.node._move_j_pub = _NullPublisher()
        self.node._gripper_offset = np.zeros(3)
        self.sent = []
        original_send = self.node._send_move_p

        def record(position, orientation, **kwargs):
            self.sent.append(
                (np.asarray(position, dtype=float).copy(), tuple(orientation))
            )
            return original_send(position, orientation, **kwargs)

        self.node._send_move_p = record

        # spawn 的接缝: 换成桩, 完全不碰 subprocess / 串口。
        self.spawned = []

        def fake_spawn(mode):
            job = _FakeJob(mode)
            self.spawned.append(job)
            return job

        self.node._spawn_gripper_job = fake_spawn
        # 真机路径要查串口存在性, 而测试机上没有 ttyACM0。默认 dry_run=true 时
        # 这条检查本来就不跑, 显式把端口指到 /dev/null 是为了让 dry_run=false 的
        # 用例也能走完加载期。
        self.node.set_parameters(
            [rclpy.parameter.Parameter("gripper_port", value="/dev/null")]
        )

    def tearDown(self):
        self.node.destroy_node()

    # ── 工具 ────────────────────────────────────────────────

    def _set(self, **params):
        from rclpy.parameter import Parameter

        results = self.node.set_parameters(
            [Parameter(name, value=value) for name, value in params.items()]
        )
        for (name, _), result in zip(params.items(), results):
            self.assertTrue(result.successful, f"参数 {name} 设置失败: {result.reason}")

    def _feedback(self, position, orientation=(0.0, 0.0, 0.0, 1.0), motion_status=0):
        now = self.node.get_clock().now().nanoseconds * 1e-9
        self.node._tcp_pose = (now, np.asarray(position, dtype=float))
        self.node._tcp_orientation = tuple(orientation)
        self.node._arm_status = (now, _arm_status_msg(motion_status))

    def _age(self, attribute, seconds):
        """模拟时间流逝: 不做真实 sleep, 把计时起点往前挪。"""
        since = getattr(self.node, attribute)
        if since is not None:
            setattr(self.node, attribute, since - seconds)

    def _enable_gripper(self, dry_run=False, **params):
        """打开夹爪并重跑一遍加载期校验。

        默认 dry_run=False —— 夹爪相关的用例基本都在测「真的会去下命令」那条路;
        测 dry_run 的用例自己传 True。
        """
        values = {"gripper_enabled": True, "dry_run": dry_run}
        values.update(params)
        self._set(**values)
        self.node._load_gripper_config()
        self.assertTrue(
            self.node._gripper_ready,
            "测试前提: scripts/gripper_grip_hold.py 应当找得到 (src 或 install)",
        )

    def _enter_grasp_wait(self):
        """把节点摆到 GRASP_WAIT: 下降已发出, 等第二次确认。"""
        self.node._grasp_mode = "side"
        self.node._goal_orientation = (0.0, 0.0, 0.0, 1.0)
        self.node._final_grasp_target = GRASP_CENTER + np.asarray(
            (0.0, 0.0, self.node.get_parameter("grasp_offset_z").value)
        )
        self._feedback(LIFT_START)
        self.node._transition(self.node._STATE_GRASP_WAIT)

    def _enter_place_wait(self):
        """把节点摆到 PLACE_WAIT: 放置位姿已设, 下降已发出, 等第三次确认。"""
        self._set(
            place_enabled=True,
            place_pose_reported_mm_deg=[400.0, -300.0, 200.0, 0.0, 0.0, 0.0],
        )
        self.node._load_place_pose()
        self.assertTrue(self.node._place_ready, "测试前提: 放置点应当校验通过")
        self._feedback(np.asarray((0.40, -0.30, 0.40)))
        self.node._transition(self.node._STATE_PLACE_WAIT)

    def _tick(self, times=1):
        for _ in range(times):
            self.node._tick_gripper()

    def _finish_close(self, exit_code=0, *lines):
        """走完一次闭合: 起进程 -> 喂 stdout -> 让它退出。返回那个假 job。

        用 .mode 断言命令, 而不是断言 stdout —— 现在节点只看退出码。
        """
        self._enable_gripper()
        self._enter_grasp_wait()
        self.node._enter_pressed = True
        self.node._tick_grasp_wait()
        self.node._tick_gripper()                    # 起 close 进程
        job = self.spawned[-1]
        job.say(*lines)
        job.exit_with(exit_code)
        self.node._tick_gripper()
        return job

    def _finish_close_holding(self, **params):
        """走完一次闭合, 停在「保持中」: 状态在 LIFT, 手里那个假 job **还活着**。

        `gripper_hold_torque=1.0` + 喂一行 `[holding]` —— 这两样合起来才是
        「夹住了, 力已经加上」的完整语义 (见 _gripper_outcome)。
        """
        self._enable_gripper(gripper_hold_torque=1.0, **params)
        self._enter_grasp_wait()
        self.node._enter_pressed = True
        self.node._tick_grasp_wait()
        self.node._tick_gripper()                    # 起 close 进程
        job = self.spawned[-1]
        job.say("[holding] tau_cmd=-1.000 kd=0.50 q=+0.217 tau_fb=-0.930 wdt_ms=100")
        self.node._tick_gripper()                    # 认下标记 -> LIFT, 进程留着
        return job


class DisabledTest(_GripperTestCase):
    """gripper_enabled=false (代码与 yaml 的默认状态): 老流程一字不变。"""

    def test_default_is_disabled(self):
        self.assertFalse(self.node.get_parameter("gripper_enabled").value)
        self.assertFalse(self.node._gripper_ready)
        self.assertIsNone(self.node._gripper_script)

    def test_execute_goes_straight_to_wait_target(self):
        response = self.node._on_execute(None, SimpleNamespace())
        self.assertTrue(response.success)
        self.assertEqual(self.node._state, self.node._STATE_WAIT_TARGET)
        self.assertEqual(self.spawned, [], "关着的时候一次都不该 spawn")

    def test_enter_in_grasp_wait_still_lifts_directly(self):
        """老语义: Enter = 「我已经手动夹好了」, 确认后直接抬升。"""
        self.node._grasp_mode = "side"
        self.node._final_grasp_target = np.asarray(GRASP_CENTER, dtype=float)
        self.node._goal_orientation = (0.0, 0.0, 0.0, 1.0)
        self._feedback(LIFT_START)
        self.node._transition(self.node._STATE_GRASP_WAIT)
        self.node._enter_pressed = True
        self.node._tick_grasp_wait()
        self.assertEqual(self.node._state, self.node._STATE_LIFT)
        self.assertEqual(self.spawned, [])

    def test_enter_in_place_wait_still_finishes_directly(self):
        self._enter_place_wait()
        self.node._enter_pressed = True
        self.node._tick_place_wait()
        self.assertNotEqual(self.node._state, self.node._STATE_GRIPPER)
        self.assertEqual(self.spawned, [])

    def test_startup_rejects_a_task_when_enabled_but_unusable(self):
        """开着夹爪却定位不到脚本: 拒绝启动, 而不是抓完才发现手上没法夹。"""
        self._set(gripper_enabled=True)
        self.node._load_gripper_config()
        self.node._gripper_ready = False
        self.node._gripper_script = None
        response = self.node._on_execute(None, SimpleNamespace())
        self.assertFalse(response.success)
        self.assertIn("gripper", response.message)
        self.assertEqual(self.node._state, self.node._STATE_IDLE)


class OpenAtStartTest(_GripperTestCase):
    """每轮开始前先张开: 从「确定张开」这个状态出发, 而不是上一次留下的姿势。"""

    def test_first_state_is_gripper_open(self):
        self._enable_gripper()
        self.node._on_execute(None, SimpleNamespace())
        self.assertEqual(self.node._state, self.node._STATE_GRIPPER)
        self.assertEqual(self.node._gripper_stage, self.node._GRIPPER_STAGE_OPEN)
        self.assertEqual(self.spawned, [], "起进程在下一拍, 不在 _on_execute 里")

    def test_open_then_wait_target(self):
        self._enable_gripper()
        self.node._on_execute(None, SimpleNamespace())
        self.node._tick_gripper()                    # 起进程
        self.assertEqual(len(self.spawned), 1)
        self.assertEqual(self.spawned[0].mode, "open")
        self.spawned[0].exit_with(0)                 # 张开到位, 脚本自己退
        self.node._tick_gripper()
        self.assertEqual(self.node._state, self.node._STATE_WAIT_TARGET)

    def test_open_failure_fails_and_returns_home(self):
        self._set(return_to_standby_on_finish=True)
        self.node._return_pending = True
        self.assertTrue(self.node._standby_ready, "前提: 默认待机位要能通过校验")
        self._enable_gripper()
        self.node._on_execute(None, SimpleNamespace())
        self.node._tick_gripper()
        self.spawned[0].exit_with(2)
        self.node._tick_gripper()
        self.assertEqual(self.node._state, self.node._STATE_HOME)
        self.assertEqual(self.sent, [], "回位指令还没到下发的那一拍")

    def test_nonzero_exit_without_return_goes_idle(self):
        self._set(return_to_standby_on_finish=False)
        self._enable_gripper()
        self.node._on_execute(None, SimpleNamespace())
        self.node._tick_gripper()
        self.spawned[0].exit_with(2)
        self.node._tick_gripper()
        self.assertEqual(self.node._state, self.node._STATE_IDLE)


class CloseAtGraspTest(_GripperTestCase):
    """GRASP_WAIT 的 Enter 从「我已经夹好了」变成「现在夹」。"""

    def test_enter_starts_the_close_without_moving_the_arm(self):
        self._enable_gripper()
        self._enter_grasp_wait()
        self.node._enter_pressed = True
        self.node._tick_grasp_wait()
        self.assertEqual(self.node._state, self.node._STATE_GRIPPER)
        self.assertEqual(self.node._gripper_stage, self.node._GRIPPER_STAGE_CLOSE)
        self.assertEqual(self.sent, [], "闭合跑完之前机械臂一动都不该动")

    def test_a_clean_close_advances_to_lift(self):
        self._enable_gripper()
        self._enter_grasp_wait()
        self.node._enter_pressed = True
        self.node._tick_grasp_wait()
        self.node._tick_gripper()                    # 起 close 进程
        job = self.spawned[0]
        self.assertEqual(job.mode, "close")
        job.exit_with(0)
        self.node._tick_gripper()
        self.assertEqual(self.node._state, self.node._STATE_LIFT)
        lift_distance = float(self.node.get_parameter("lift_distance").value)
        expected = LIFT_START + np.asarray((0.0, 0.0, lift_distance))
        np.testing.assert_allclose(self.node._lift_target, expected, atol=1e-9)

    def test_stdout_does_not_decide_anything(self):
        """脚本 stdout 上写什么都不算数, 只看退出码 —— 2026-09-18 那次真机事故。

        真机上脚本打了 `[warn] 扭矩冲到硬上限 ... 按「已夹住」停下`, 老代码把它当
        失败 (退出码 2) 让整趟任务作废, 而夹爪其实已经夹住了。现在这类行只是日志。
        """
        self._enable_gripper()
        self._enter_grasp_wait()
        self.node._enter_pressed = True
        self.node._tick_grasp_wait()
        self.node._tick_gripper()
        job = self.spawned[0]
        job.say("[warn] 扭矩冲到硬上限 -2.160 N·m, 按「已夹住」停下")
        job.say("[stop] 闭合结束")
        self.node._tick_gripper()                    # 还在跑: 不推进也不失败
        self.assertEqual(self.node._state, self.node._STATE_GRIPPER)
        job.exit_with(0)
        self.node._tick_gripper()
        self.assertEqual(self.node._state, self.node._STATE_LIFT)

    def test_a_close_that_ran_to_the_end_still_lifts(self):
        """一路合到底 (没遇到阻力) 也是「闭合命令跑完了」, 照常抬升。

        节点不判夹到没有 (2026-09-17 用户要求), 夹空的原因靠日志里那行看出来。
        """
        self._finish_close(0, "[stop] 一路合到底, 没遇到阻力")
        self.assertEqual(self.node._state, self.node._STATE_LIFT)

    def _close_in_mode(self, mode, orientation=None, **params):
        """在指定 grasp_mode 下走完一次闭合 (返回闭合后的状态)。

        orientation 给定时覆盖当前 TCP 姿态 —— **必须在 _enter_grasp_wait 之后**:
        它内部会写一帧反馈 (姿态是单位四元数), 先设的会被它冲掉 (回正判据看的正是
        这个姿态)。
        """
        self._enable_gripper(**params)
        self._enter_grasp_wait()
        self.node._grasp_mode = mode          # 必须在上面的 _enter_grasp_wait 之后 (它写 side)
        if orientation is not None:
            self._feedback(LIFT_START, orientation)
        self.node._enter_pressed = True
        self.node._tick_grasp_wait()
        self.node._tick_gripper()             # 起 close 进程
        self.spawned[-1].exit_with(0)
        self.node._tick_gripper()
        return self.node._state

    def test_top_lift_is_not_skipped_when_it_must_align(self):
        """需要回正姿态时**不能**跳过抬升 —— 哪怕 top_skip_lift 还开着 (2026-09-30 现场)。

        那一次真机上跳过了抬升, 放置那条指令就得从**离台面 6.7cm 的抓取点**同时走
        34.6cm 位移 + 144° 转腕: IK 解不出来 (残差 3.19mm/16.47°) -> 回落 move_p 交给
        固件规划 -> 大摆 -> `err_status=512` 收场。回正折在抬升里, 没有抬升就没有它;
        而且那一下要转大半个腕, 贴着台面做太危险。

        姿态用现场实测的那个「工具朝下」四元数: 它与 side 抓取姿态差 176°。
        """
        self._set(place_enabled=True,
                  place_pose_reported_mm_deg=[400.0, -300.0, 200.0, 0.0, 0.0, 0.0])
        self.node._load_place_pose()
        self.assertTrue(self.node._place_ready, "测试前提: 放置点要能通过校验")
        state = self._close_in_mode(
            "top", orientation=(0.9997, 0.0059, 0.0206, 0.0067), top_skip_lift=True
        )
        self.assertEqual(state, self.node._STATE_LIFT)

    def test_top_mode_can_skip_the_lift_when_no_align_is_needed(self):
        """姿态本来就与 side 一致时不回正, 跳过抬升照旧生效 (2026-09-28 的行为)。

        side 模式的抓取姿态就取自 side_orientation_xyzw, 所以 real 上这条就是 side 的
        常态 —— 回正这条改动对它是零影响。
        """
        self._set(place_enabled=True,
                  place_pose_reported_mm_deg=[400.0, -300.0, 200.0, 0.0, 0.0, 0.0])
        self.node._load_place_pose()
        self.assertTrue(self.node._place_ready, "测试前提: 放置点要能通过校验")
        # 直接用参数里那一组当当前姿态 —— 判据就是「当前姿态 vs 这个参数」, 相等即不回正。
        state = self._close_in_mode(
            "top",
            orientation=tuple(self.node.get_parameter("side_orientation_xyzw").value),
            top_skip_lift=True,
        )
        self.assertEqual(state, self.node._STATE_PLACE_MOVE)

    def test_side_mode_still_lifts_with_the_switch_on(self):
        """开关只管 top: side 是从侧面抓的, 必须先把物体提离台面。"""
        self._set(place_enabled=True,
                  place_pose_reported_mm_deg=[400.0, -300.0, 200.0, 0.0, 0.0, 0.0])
        self.node._load_place_pose()
        state = self._close_in_mode("side", top_skip_lift=True)
        self.assertEqual(state, self.node._STATE_LIFT)

    def test_top_mode_still_lifts_when_there_is_no_place_point(self):
        """**没配放置点时不跳** —— 跳了会变成「夹着瓶子原地进 IDLE」, 比多抬一次糟。"""
        state = self._close_in_mode("top", top_skip_lift=True)
        self.assertEqual(state, self.node._STATE_LIFT)

    def test_the_switch_defaults_to_off(self):
        """默认关: 不配它时三种模式的行为与改动前逐字相同。"""
        self.assertEqual(self.node.get_parameter("top_skip_lift").value, False)
        state = self._close_in_mode("top")
        self.assertEqual(state, self.node._STATE_LIFT)

    def test_a_broken_close_still_fails(self):
        """但**命令没跑成**要判失败: 退出码 2 = 脚本 [abort] (典型是串口被占),
        闭合根本没发生, 这时候抬升是没有意义的。"""
        self._set(return_to_standby_on_finish=False)
        self._finish_close(2, "[abort] Cannot open port '/dev/ttyACM0'")
        self.assertEqual(self.node._state, self.node._STATE_IDLE)

    def test_timeout_fails(self):
        self._set(return_to_standby_on_finish=False)
        self._enable_gripper()
        self._enter_grasp_wait()
        self.node._enter_pressed = True
        self.node._tick_grasp_wait()
        self.node._tick_gripper()
        self._age("_gripper_started_at",
                  float(self.node.get_parameter("gripper_timeout").value) + 0.01)
        self.node._tick_gripper()
        self.assertEqual(self.node._state, self.node._STATE_IDLE)

    def test_failure_does_not_try_to_command_the_gripper_again(self):
        """判失败时**不再**补一条「松手」指令: 老设计里那条 _release_gripper_job() 没了。

        现在失败路径只做两件事: 打日志、走 _fail。进程要么已经退出, 要么自己会撞上
        脚本内部的 --timeout 退掉 —— 失败路径上少一个可能失败的分支。
        """
        self._set(return_to_standby_on_finish=False)
        self._finish_close(2)
        self.assertEqual(len(self.spawned), 1, "失败之后不该再起第二个进程")
        self.assertFalse(
            self.spawned[0].killed, "_fail 不杀进程: 让它自己跑完退出"
        )


class ReleaseAtPlaceTest(_GripperTestCase):
    """PLACE_WAIT 的 Enter 从「我已经放好了」变成「现在放」。"""

    def test_release_spawns_a_fresh_process(self):
        """张开是**新起一个进程**, 不是对闭合那个下命令 —— 一次性命令的核心。

        代价写清楚: 闭合进程退出到张开进程接手之间, 夹持力全靠机构自锁。
        """
        first = self._finish_close(0)
        self.assertEqual(self.node._state, self.node._STATE_LIFT)
        self._enter_place_wait()
        self.node._enter_pressed = True
        self.node._tick_place_wait()
        self.assertEqual(self.node._state, self.node._STATE_GRIPPER)
        self.assertEqual(self.node._gripper_stage, self.node._GRIPPER_STAGE_RELEASE)
        self.node._tick_gripper()
        self.assertEqual(len(self.spawned), 2)
        self.assertEqual(self.spawned[1].mode, "open")
        self.assertTrue(first.closed, "上一趟的 stdout 管道必须放掉")

    def test_a_clean_release_then_home(self):
        self._finish_close(0)
        self._set(return_to_standby_on_finish=True)
        self.node._return_pending = True
        self._enter_place_wait()
        self.node._enter_pressed = True
        self.node._tick_place_wait()
        self.node._tick_gripper()
        self.spawned[1].exit_with(0)
        self.node._tick_gripper()
        self.assertEqual(self.node._state, self.node._STATE_HOME)

    def test_a_failed_release_does_not_finish_the_task(self):
        """张开的脚本没跑成 (退出码非 0) 就算失败: 夹着瓶子回待机位不是「放置完成」。"""
        self._finish_close(0)
        self._set(return_to_standby_on_finish=False)
        self._enter_place_wait()
        self.node._enter_pressed = True
        self.node._tick_place_wait()
        self.node._tick_gripper()
        self.spawned[1].exit_with(2)
        self.node._tick_gripper()
        self.assertEqual(self.node._state, self.node._STATE_IDLE)


class AutoConfirmTest(_GripperTestCase):
    """grasp_confirm_required / place_confirm_required = false: 不等 Enter, dwell 一到就动。

    这两处确认是 2026-09-18 应现场要求关掉的 (整条链只留 TRACK_WAIT 那一次确认)。
    这里盯的是「不等确认」**不等于**「跳过动作」—— 夹爪照夹、照张, 只是没人按 Enter。
    反向也要盯 (test_*_still_waits_when_required): 开关打开时必须退回原行为, 否则
    它就成了一个永远自动的假开关。
    """

    def _close_auto(self):
        """与 CloseAtGraspTest 同一条路, 只是闭合那次也不按 Enter。"""
        self._enable_gripper(grasp_confirm_required=False)
        self._enter_grasp_wait()
        self.node._tick_grasp_wait()
        self.node._tick_gripper()
        job = self.spawned[0]
        job.exit_with(0)
        self.node._tick_gripper()
        self.assertEqual(self.node._state, self.node._STATE_LIFT)
        return job

    def test_grasp_wait_closes_without_enter(self):
        self._enable_gripper(grasp_confirm_required=False)
        self._enter_grasp_wait()
        self.node._tick_grasp_wait()
        self.assertEqual(self.node._state, self.node._STATE_GRIPPER)
        self.assertEqual(self.node._gripper_stage, self.node._GRIPPER_STAGE_CLOSE)
        self.assertEqual(self.sent, [], "闭合跑完之前机械臂一动都不该动")

    def test_grasp_wait_still_waits_when_required(self):
        self._enable_gripper()                      # grasp_confirm_required 默认 true
        self._enter_grasp_wait()
        self.node._tick_grasp_wait()
        self.assertEqual(
            self.node._state, self.node._STATE_GRASP_WAIT, "开关打开时没人确认就该一直等"
        )
        self.assertEqual(self.spawned, [], "没人确认就不该起夹爪进程")

    def test_place_wait_releases_without_enter(self):
        self._close_auto()
        self._set(place_confirm_required=False)
        self._enter_place_wait()
        # 自动分支要先停满 place_descend_dwell (起算点是 _state_enter_time, 进
        # PLACE_WAIT 那一刻)。这里只管「进 GRIPPER 之后夹爪那一段」, 停留本身由
        # test_place.PlaceWaitAutoConfirmTest 盯, 所以直接把时间挪过去。
        self._age(
            "_state_enter_time",
            float(self.node.get_parameter("place_descend_dwell").value) + 0.01,
        )
        self.node._tick_place_wait()
        self.assertEqual(self.node._state, self.node._STATE_GRIPPER)
        self.assertEqual(self.node._gripper_stage, self.node._GRIPPER_STAGE_RELEASE)
        self.node._tick_gripper()
        self.assertEqual(len(self.spawned), 2)
        self.assertEqual(self.spawned[1].mode, "open")

    def test_place_wait_still_waits_when_required(self):
        self._close_auto()
        self._enter_place_wait()                    # place_confirm_required 默认 true
        self.node._tick_place_wait()
        self.assertEqual(
            self.node._state, self.node._STATE_PLACE_WAIT, "开关打开时没人确认就该一直等"
        )
        self.assertNotEqual(self.node._state, self.node._STATE_GRIPPER)


class DryRunTest(_GripperTestCase):
    """dry_run 的规矩是「不碰硬件」, 夹爪也不破例。"""

    def test_no_spawn_but_the_flow_still_runs(self):
        self._enable_gripper(dry_run=True)
        self.node._on_execute(None, SimpleNamespace())
        self.assertEqual(self.node._state, self.node._STATE_GRIPPER)
        self.node._tick_gripper()
        self.assertEqual(self.spawned, [], "dry_run 一次都不该 spawn")
        self.assertEqual(self.node._state, self.node._STATE_WAIT_TARGET)

    def test_close_is_skipped_but_lift_still_happens(self):
        self._enable_gripper(dry_run=True)
        self._enter_grasp_wait()
        self.node._enter_pressed = True
        self.node._tick_grasp_wait()
        self.node._tick_gripper()
        self.assertEqual(self.spawned, [])
        self.assertEqual(self.node._state, self.node._STATE_LIFT)

    def test_release_does_not_touch_a_hardware_process(self):
        self._enable_gripper(dry_run=True)
        self._enter_place_wait()
        self.node._enter_pressed = True
        self.node._tick_place_wait()
        self.node._tick_gripper()
        self.assertEqual(self.spawned, [])
        self.assertNotEqual(self.node._state, self.node._STATE_GRIPPER)


class JobLifecycleTest(_GripperTestCase):
    """进程的收尾: 每趟都是新进程, 上一趟不许漏掉, 下一趟不许叠着。"""

    def test_reentering_gripper_reaps_the_previous_job(self):
        self._enable_gripper()
        self.node._on_execute(None, SimpleNamespace())
        self.node._tick_gripper()
        first = self.spawned[0]
        first.exit_with(0)
        self.node._tick_gripper()                       # -> WAIT_TARGET
        self.node._enter_gripper(self.node._GRIPPER_STAGE_OPEN)
        self.assertTrue(first.closed, "上一趟的 stdout 管道必须放掉")

    def test_a_live_job_is_killed_when_reaped(self):
        self._enable_gripper()
        self.node._on_execute(None, SimpleNamespace())
        self.node._tick_gripper()
        first = self.spawned[0]
        self.node._enter_gripper(self.node._GRIPPER_STAGE_OPEN)
        self.assertTrue(first.killed, "还活着的进程要收掉, 否则会一直占着串口")

    def test_only_one_job_is_tracked_at_a_time(self):
        self._enable_gripper()
        self.node._on_execute(None, SimpleNamespace())
        self.node._tick_gripper()
        self.assertIs(self.node._gripper_job, self.spawned[0])
        self.node._enter_gripper(self.node._GRIPPER_STAGE_OPEN)
        self.assertIsNone(self.node._gripper_job, "刚进状态时不该还挂着上一趟")

    def test_the_closed_job_does_not_survive_the_lift(self):
        """闭合进程在抬升时已经退出了, 而且节点不再需要它 —— 它就是一次性的。"""
        job = self._finish_close(0)
        self.assertFalse(job.alive, "闭合命令跑完就该退出, 没有它在后面维持夹持力")
        self.assertEqual(len(self.spawned), 1, "抬升过程中不该再起进程")


class RealPipeTest(unittest.TestCase):
    """拿**真进程 + 真管道**验 _GripperJob 自身。

    上面那些用例注入的是假 job, 而假 job 的 read_lines() 不碰任何真流 —— 于是管道
    装配上的毛病 (少 set_blocking、忘了关写端、半行阻塞、最后一行没有换行) 在单元
    测试里全都看不见, 要到真机上才暴露。

    这里用 spawn_gripper_process 起一个替身脚本 (就是节点用的那条装配路径, 不是照
    抄的), 只验几件假对象验不了的事: 半行不会阻塞、退出时没有换行的最后一行能交出来、
    退出码能通过 poll() 读到, 以及 **stdin 那一行真的能送进去** (2026-09-28 加回
    stdin 之后, 保持进程就是靠它松手的)。
    """

    def _spawn(self, body):
        job = spawn_gripper_process([sys.executable, "-u", "-c", body], "test")
        self.addCleanup(self._reap, job)
        return job

    @staticmethod
    def _reap(job):
        job.kill()
        try:
            job.process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            pass
        job.close()

    def _collect(self, job, want, timeout=5.0):
        """有界地收行, 直到出现 want 或超时; 返回收到的全部行。"""
        deadline = time.monotonic() + timeout
        got = []
        while time.monotonic() < deadline:
            got.extend(job.read_lines())
            if want in got:
                return got
            time.sleep(0.02)
        return got

    def test_a_half_line_does_not_block(self):
        # 脚本的 [stop] 行是分几次写出来的, 半行必须留在缓冲里等下一拍 —— 在 20Hz
        # 回调里等一个换行符会把整个执行器冻住。
        job = self._spawn("import sys, time\nsys.stdout.write('half')\nsys.stdout.flush()\ntime.sleep(30)\n")
        started = time.monotonic()
        self.assertEqual(job.read_lines(), [])
        self.assertLess(time.monotonic() - started, 1.0, "read_lines 不该等换行符")

    def test_the_last_line_without_a_newline_is_delivered(self):
        # 进程被信号打断时最后的 `[abort] ...` 可能没有换行, 交不出来就永远丢在缓冲里。
        job = self._spawn("import sys\nsys.stdout.write('[abort] boom')\nsys.stdout.flush()\n")
        deadline = time.monotonic() + 5.0
        got = []
        while time.monotonic() < deadline:
            got.extend(job.read_lines())
            if got:
                break
            time.sleep(0.02)
        self.assertEqual(got, ["[abort] boom"])

    def test_a_finished_command_reports_its_exit_code(self):
        """跑完的一条命令: 输出能读到, 退出码能读到 —— 节点要的就这两样。"""
        job = self._spawn("import sys\nprint('[stop] 闭合结束')\nsys.exit(0)\n")
        self.assertIn("[stop] 闭合结束", self._collect(job, "[stop] 闭合结束"))
        deadline = time.monotonic() + 5.0
        while job.poll() is None and time.monotonic() < deadline:
            time.sleep(0.02)
        self.assertEqual(job.poll(), 0)

    def test_a_stdin_line_reaches_a_real_child(self):
        """`write_line` 真的能被脚本读到 —— 保持进程就是靠这一行松手的。

        这条专门钉当年那个坑: 父进程按 bytes 写、子进程按 str 比 (或者反过来),
        在假对象上永远看不出来。
        """
        job = self._spawn(
            "import sys\n"
            "print('got', sys.stdin.readline().strip(), flush=True)\n"
        )
        self.assertTrue(job.write_line("OPEN"), "写进去了就该返回 True")
        self.assertIn("got OPEN", self._collect(job, "got OPEN"))

    def test_write_line_after_the_child_exited_is_a_no_op(self):
        """进程已经退出的那种写失败要返回 False, **不能抛异常** —— 调用点在计时器
        回调里, 抛出去直接拖垮节点。"""
        job = self._spawn("import sys\nsys.exit(0)\n")
        deadline = time.monotonic() + 5.0
        while job.poll() is None and time.monotonic() < deadline:
            time.sleep(0.02)
        self.assertIsNotNone(job.poll(), "前提: 子进程要已经退出")
        self.assertFalse(job.write_line("OPEN"))


class HoldingTest(_GripperTestCase):
    """gripper_hold_torque > 0: 闭合后进程**不退出**, 抬升/搬运全程持续保持力矩。

    这一组的契约 (2026-09-28):
      - `[holding]` 那一行是**协议**, 不是日志 —— 保持进程不退出, 节点只有这一个
        「夹住了、力已经加上」的信号 (其余 stdout 仍然只是排障信息);
      - 那个进程跟着走完 LIFT / PLACE_MOVE / PLACE_WAIT, 节点每拍收它的输出
        (不收会把管道写满、把保持帧憋死), 并盯着它别死在半路;
      - 放开它只有一条路: 往它的 stdin 写一行 OPEN。
    默认 `gripper_hold_torque=0.0` 时这一切都不生效, 行为与改动前逐字相同 ——
    上面那些用例 (CloseAtGraspTest / ReleaseAtPlaceTest / JobLifecycleTest) 钉的
    就是那条老路, 它们一条都不用改。
    """

    def _capture_fail(self):
        """把 _fail 的原因记下来 (supervision 判失败时唯一能断言的证据)。"""
        reasons = []
        original = self.node._fail

        def wrapped(reason, *args, **kwargs):
            reasons.append(str(reason))
            return original(reason, *args, **kwargs)

        self.node._fail = wrapped
        return reasons

    # ── 标记本身 ────────────────────────────────────────────

    def test_the_marker_advances_to_lift_and_the_job_stays_alive(self):
        job = self._finish_close_holding()
        self.assertEqual(self.node._state, self.node._STATE_LIFT)
        self.assertTrue(job.alive, "保持进程不该退出 —— 它就是靠活着发帧顶住力的")
        self.assertFalse(job.killed, "保持中不许杀它")
        self.assertFalse(job.closed, "两条管道都要留着: 后面还要靠 stdin 写 OPEN")
        self.assertTrue(self.node._gripper_holding)

    def test_the_marker_is_not_honored_when_the_job_already_died(self):
        """同拍既说了「保持」又已经退出 —— 不能就这么抬升。

        标记承诺的是「力已经加上」; 进程没了, 力也就没了, 这时推进 LIFT 就是抬一个
        空手。宁可判失败让人来看。
        """
        self._enable_gripper(gripper_hold_torque=1.0)
        self._enter_grasp_wait()
        self.node._enter_pressed = True
        self.node._tick_grasp_wait()
        self.node._tick_gripper()
        job = self.spawned[-1]
        job.say("[holding] tau_cmd=-1.000 kd=0.50 q=+0.217 tau_fb=-0.930")
        job.exit_with(0)
        reasons = self._capture_fail()
        self.node._tick_gripper()
        self.assertNotEqual(self.node._state, self.node._STATE_LIFT)
        self.assertTrue(any("保持进程" in r for r in reasons), reasons)

    def test_a_holding_line_is_ignored_outside_the_close_stage(self):
        """张开类阶段绝不能认这个标记: 缓冲区里可能还留着闭合那趟的那一行。"""
        job = self._finish_close_holding()
        self._enter_place_wait()
        self.node._enter_pressed = True
        self.node._tick_place_wait()                 # -> GRIPPER / RELEASE
        job.say("[holding] 这是上一条命令留下的旧标记")
        self.node._tick_gripper()
        self.assertEqual(
            self.node._state, self.node._STATE_GRIPPER,
            "RELEASE 阶段收到 [holding] 不该被当成「已张开」",
        )

    # ── 搬运途中死了 ────────────────────────────────────────

    def test_hold_death_during_lift_fails(self):
        job = self._finish_close_holding()
        reasons = self._capture_fail()
        job.exit_with(0)
        self._feedback(LIFT_START)                   # 别让反馈超时抢了先
        self.node._tick()
        self.assertEqual(self.node._state, self.node._STATE_IDLE)
        self.assertEqual(self.sent, [], "这一拍不该再发任何运动指令")
        self.assertTrue(any("保持进程已退出" in r for r in reasons), reasons)

    def test_hold_death_during_place_move_fails(self):
        job = self._finish_close_holding()
        self.node._transition(self.node._STATE_PLACE_MOVE)
        reasons = self._capture_fail()
        job.exit_with(0)
        self._feedback(LIFT_START)
        self.node._tick()
        self.assertEqual(self.node._state, self.node._STATE_IDLE)
        self.assertTrue(any("保持进程已退出" in r for r in reasons), reasons)

    def test_hold_death_during_place_wait_fails(self):
        """PLACE_WAIT 最容易被漏: place_confirm_required=true 时人可能站几分钟。

        这期间进程死掉没人发现的话, 等 Enter 一来节点会回落到「另外起一个 open
        进程」, 那个进程正常张开、节点报「放置完成」—— 瓶子早掉了却报成功。
        """
        job = self._finish_close_holding()
        self._enter_place_wait()
        reasons = self._capture_fail()
        job.exit_with(0)
        self._feedback(LIFT_START)
        self.node._tick()
        self.assertEqual(self.node._state, self.node._STATE_IDLE)
        self.assertTrue(any("保持进程已退出" in r for r in reasons), reasons)

    def test_hold_death_outside_the_carry_states_is_ignored(self):
        """回待机位途中它死了不该再判一次失败 —— 那时东西早放下了 (或已经失败过)。

        直接问谓词, 不走 _tick: HOME 需要路点队列, 在这儿摆一个空队列只会让状态机
        报另一条无关的错, 把这条用例要钉的东西淹掉。
        """
        job = self._finish_close_holding()
        job.exit_with(0)
        self.node._state = self.node._STATE_LIFT
        self.assertIsNotNone(self.node._gripper_hold_problem(), "前提: LIFT 下要判")
        for state in (
            self.node._STATE_HOME,
            self.node._STATE_IDLE,
            self.node._STATE_WAIT_TARGET,
            self.node._STATE_GRIPPER,
        ):
            with self.subTest(state=state):
                self.node._state = state
                self.assertIsNone(self.node._gripper_hold_problem())

    # ── 松开: 复用那个进程 ──────────────────────────────────

    def test_release_reuses_the_holding_job(self):
        """张开 = 给还在保持的那个进程写一行 OPEN, **不另起进程**。

        串口只有一个: 另起一个会和它抢, 抢输的那个 [abort] 退 2。
        """
        job = self._finish_close_holding()
        job.exit_on_write = True                     # 真脚本: 张开完就退出
        self._enter_place_wait()
        self.node._enter_pressed = True
        self.node._tick_place_wait()
        self.assertEqual(self.node._state, self.node._STATE_GRIPPER)
        self.node._tick_gripper()                    # 第一拍: 写 OPEN
        self.assertEqual(len(self.spawned), 1, "不该另起进程")
        self.assertEqual(job.written, ["OPEN"])
        self.assertFalse(job.closed, "复用的时候不许把管道关掉")
        self.node._tick_gripper()                    # 它退出了 -> 这一趟做完
        self.assertNotEqual(self.node._state, self.node._STATE_GRIPPER)

    def test_the_open_stage_of_the_next_round_reuses_the_holding_job(self):
        """下一轮起手的「先张开」也是同一个动作, 同样复用。"""
        job = self._finish_close_holding()
        job.exit_on_write = True
        self._enter_place_wait()                     # 只为了拿到一个有效放置点
        self.node._transition(self.node._STATE_IDLE)
        self.assertIsNone(self.node._begin_task(), "该正常启动")
        self.assertEqual(self.node._state, self.node._STATE_GRIPPER)
        self.node._tick_gripper()
        self.assertEqual(len(self.spawned), 1, "不该另起进程")
        self.assertEqual(job.written, ["OPEN"])

    def test_release_falls_back_when_the_holding_job_already_died(self):
        job = self._finish_close_holding()
        self._enter_place_wait()
        self.node._enter_pressed = True
        job.exit_with(0)                             # 悄悄死了 (这一拍没走 _tick)
        self.node._tick_place_wait()
        self.node._tick_gripper()
        self.assertEqual(len(self.spawned), 2, "死进程没法复用, 该另起一个")
        self.assertEqual(self.spawned[1].mode, "open")

    def test_release_falls_back_when_the_write_fails(self):
        job = self._finish_close_holding()
        job.write_fails = True                       # 管道异常 / 对端不读
        self._enter_place_wait()
        self.node._enter_pressed = True
        self.node._tick_place_wait()
        self.node._tick_gripper()
        self.assertEqual(len(self.spawned), 2)
        self.assertEqual(self.spawned[1].mode, "open")

    # ── 每拍收行 / dry_run / 开工闸门 ───────────────────────

    def test_lines_are_drained_while_the_arm_is_moving(self):
        """抬升途中没人调 _tick_gripper, 但 stdout 也必须每拍被收走。

        不然 64KiB 管道十几秒就满, 子进程的 print 一阻塞, 保持帧跟着断, 固件看门狗
        切电 —— 瓶子在搬运途中掉, 正是这个功能要防的事故。
        """
        job = self._finish_close_holding()
        self._feedback(LIFT_START)
        job.say("hold t=1.0s q=+0.217 tau=-0.930 rate=98.6Hz")
        self.assertEqual(self.node._state, self.node._STATE_LIFT)
        self.node._tick()
        self.assertEqual(job._lines, [], "LIFT 状态下的 _tick 也要把行收走")

    def test_dry_run_never_starts_holding(self):
        self._enable_gripper(dry_run=True, gripper_hold_torque=1.0)
        self._enter_grasp_wait()
        self.node._enter_pressed = True
        self.node._tick_grasp_wait()
        self.node._tick_gripper()
        self.assertEqual(self.spawned, [], "dry_run 不碰硬件")
        self.assertFalse(self.node._gripper_holding, "dry_run 不该留下保持状态")

    def test_start_is_rejected_when_place_is_disabled(self):
        """保持开着却没有放置点: 抬升之后没有任何一步会松开夹爪。

        place_enabled=false 时 LIFT 结束直接落 IDLE (不走 PLACE_WAIT), 于是保持进程
        会一直顶到 hold_timeout, 然后在待机位上、没人看着的时候自己松手掉瓶子 ——
        比老行为更糟, 所以开工就拒。
        """
        self._enable_gripper(gripper_hold_torque=1.0)
        problem = self.node._begin_task()
        self.assertIsNotNone(problem)
        self.assertIn("gripper_hold_torque", problem)

    def test_destroy_node_releases_a_holding_job(self):
        """Ctrl+C: 写 OPEN 让它松开再退, 而不是把它丢在一边攥着串口。"""
        job = self._finish_close_holding()
        job.exit_on_write = True
        self.node.destroy_node()
        self.assertEqual(job.written, ["OPEN"])
        self.assertIsNone(self.node._gripper_job)


class SafetyTest(_GripperTestCase):
    """夹爪步骤期间出事也要回待机位; dry_run 不算「出事」。"""

    def test_gripper_state_is_in_the_return_to_standby_set(self):
        self._set(return_to_standby_on_finish=True)
        self.node._return_pending = True
        self.assertTrue(self.node._standby_ready, "前提: 默认待机位要能通过校验")
        self.node._transition(self.node._STATE_GRIPPER)
        self.assertTrue(
            self.node._should_return_to_standby(),
            "GRIPPER 必须在允许回位的状态集合里",
        )

    def test_destroy_node_lets_a_running_script_finish(self):
        """Ctrl+C 时正在跑的那条命令**不杀、不等** —— 它自己会 disable 后退出。

        杀进程容易把它停在「夹着东西但已断连」的半截状态, 放它走完反而干净。
        """
        self._enable_gripper()
        self._enter_grasp_wait()
        self.node._enter_pressed = True
        self.node._tick_grasp_wait()
        self.node._tick_gripper()
        job = self.spawned[0]
        self.node.destroy_node()
        self.assertFalse(job.killed, "不该杀掉正在跑的脚本进程")
        self.assertIsNone(self.node._gripper_job, "节点不该还挂着它")

    def test_destroy_node_is_quiet_without_a_job(self):
        self.node.destroy_node()                           # 不抛就算过


class ConfigConsistencyTest(unittest.TestCase):
    """新增的键: 节点默认值必须与 config/bottle_grasp_executor.yaml 一致。"""

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        path = os.path.join(PKG_ROOT, "config", "bottle_grasp_executor.yaml")
        with open(path, "r", encoding="utf-8") as handle:
            cls.yaml_params = yaml.safe_load(handle)["bottle_grasp_executor"][
                "ros__parameters"
            ]
        cls.node = BottleGraspExecutor()
        # 同上: 一致性用例只读参数, 但也不许握着真的发布器。
        cls.node._move_p_pub = _NullPublisher()
        cls.node._move_l_pub = _NullPublisher()
        cls.node._move_j_pub = _NullPublisher()

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def test_new_params_match_yaml(self):
        for key in NEW_GRIPPER_KEYS:
            with self.subTest(param=key):
                self.assertIn(key, self.yaml_params, f"{key} 没写进 yaml")
                default = self.node.get_parameter(key).value
                actual = self.yaml_params[key]
                self.assertIs(type(actual), type(default), f"{key} 类型不一致")
                self.assertEqual(actual, default, f"{key} 两边取值不一致")

    def test_hold_torque_stays_below_the_hard_limit(self):
        """保持力矩必须小于硬上限 —— 硬上限只写在闭合循环里, 保持比它还大就绕过了
        那道保护。脚本自己也会拦 (argparse 之后那几行), 但那是运行期才发现的,
        节点在加载期就该拒。

        这条同时是 gripper_hold_torque 的「守护用例」: 它刻意不进 NEW_GRIPPER_KEYS
        (与 gripper_enabled 同类: 代码默认保守、现场在 yaml 里开), 所以不变量得由
        这一条来守。
        """
        for source, torque, limit in (
            ("代码默认", float(self.node.get_parameter("gripper_hold_torque").value),
             float(self.node.get_parameter("gripper_hard_torque_limit").value)),
            ("yaml 现场值", float(self.yaml_params["gripper_hold_torque"]),
             float(self.yaml_params["gripper_hard_torque_limit"])),
        ):
            with self.subTest(source=source):
                self.assertGreaterEqual(torque, 0.0)
                self.assertLess(torque, limit, f"{source}: 保持力矩不小于硬上限")

    def test_gripper_timeout_is_longer_than_the_script_timeout(self):
        """节点侧的墙钟必须比脚本的长, 否则日志里只剩一句「超时」。

        脚本先自己判超时、干净退出并给出退出码与 [abort] 原因, 节点才有东西可读;
        反过来就是节点在脚本还没来得及报告原因时就把它判死了。
        """
        script = float(self.node.get_parameter("gripper_script_timeout").value)
        node = float(self.node.get_parameter("gripper_timeout").value)
        self.assertGreater(node, script)
        self.assertGreater(float(self.yaml_params["gripper_timeout"]),
                           float(self.yaml_params["gripper_script_timeout"]))

    def test_enabling_gripper_requires_a_usable_script(self):
        """部署开关不能单独成立: 开着就必须真的找得到那个脚本。

        gripper_enabled **刻意不进** NEW_GRIPPER_KEYS —— 它和 place_enabled /
        orientation_source 一样是「代码默认保守、部署时在 yaml 里开」的开关,
        要求两边取值一致等于给自己埋一条假红。守着它的是这条不变量。
        """
        if not self.yaml_params["gripper_enabled"]:
            return
        self.assertIsNotNone(
            self.node._resolve_gripper_script(),
            "yaml 里开着 gripper_enabled, 但 scripts/gripper_grip_hold.py 找不到"
            " —— 需要先 colcon build --symlink-install 一次",
        )
