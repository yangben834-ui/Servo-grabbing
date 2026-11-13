#!/usr/bin/env python3
"""bottle_grasp_executor 悬停跟随 + 下降前确认 (TRACK_WAIT) 的回归测试。

覆盖 2026-09-16 的新流程: 悬停在目标上方后跟随移动中的目标, 人工确认后才下降。
重点不是「跟得上」, 而是几条**不该跟**的边界:
  - 目标没动够 (死区) / 上一条指令还没走完 -> 不发, 免得真机抖动
  - 锁定的 track_id 之外的目标 -> 完全不看, 免得跟到别的瓶子上去
  - 候选过期 -> 保持悬停, 且确认时改成原地垂直下降, 不拿过期坐标去抓
  - 没按确认 -> 一条下降指令都不许有

不连机械臂、不连 CAN: 反馈直接写进内部缓存, 所以可以整段驱动 _tick_track_wait。

跑法 (在包根目录):
  source /opt/ros/humble/setup.bash && source install/setup.bash
  /usr/bin/python3 -m unittest -v test_track_wait
"""

import os
import time
import unittest
from types import SimpleNamespace

import numpy as np
import rclpy
import yaml

from bottle_grasp_executor.bottle_grasp_executor_node import BottleGraspExecutor

PKG_ROOT = os.path.dirname(os.path.abspath(__file__))

# 本次改动新增的参数: 节点默认值必须与 config/*.yaml 一致, 否则
# ros2 run 直接起节点与 ros2 launch 起节点的行为会不一样 (见 ConfigConsistencyTest)。
NEW_TRACK_KEYS = (
    "track_enabled",
    "track_deadband",
    "max_result_age",
    "track_min_command_interval",
    "track_settle_time",
    "track_stable_epsilon",
    "track_max_lag",
    "auto_grasp_stable_time",
    "auto_grasp_stable_epsilon",
)

# 测试里的几何: 抓取中心 0.30 m 高, 悬停点 +0.12 -> 0.42 m, 都在默认工作空间内。
# PRE_OFFSET 必须与节点默认值 (也就是 yaml 里的 pre_grasp_offset_z) 一致,
# 否则这里的期望值全是自说自话 —— 由 ConfigConsistencyTest 盯着。
# 2026-09-21: 与现场 yaml 对齐成 0.12 (之前这里和节点默认都是 0.09, 而现场是 0.12
# → 单步下降距离两边差 3cm, 也就是 ros2 run 不带 params-file 时下降行程短 3cm)。
GRASP_CENTER = np.array([0.20, 0.0, 0.30])
PRE_OFFSET = 0.12


def _arm_status_msg(motion_status=0):
    """够用的 H3001ArmStatus 替身 (_feedback_problem 与 _ik_rejection_problem 会看这些字段)。

    固件 IK 判决那 4 个字段留默认: ik_status_valid=0 在 _ik_rejection_problem 里
    **一律不判**, 所以这里不填就等于「本次没收到拒收判决」, 老用例语义不变。
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


class _TrackTestCase(unittest.TestCase):
    """每个用例一个干净节点; 反馈靠 _feedback() 直接写缓存, 不依赖真机。"""

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
        # 测试里用单位手眼矩阵: 相机系 == base_link 系, 候选坐标可以直接写基座值。
        self.node._T_base_camera = np.eye(4)
        # 夹爪偏移只作用在「上报系 <-> 夹爪系」这个边界上, 与跟随逻辑无关,
        # 置零后日志与断言里的坐标就是同一套数。
        self.node._gripper_offset = np.zeros(3)
        # dry_run 代码默认 True, 而跟随的停稳/距离判据都在真机路径上,
        # 所以测试一律按 dry_run=False 跑。
        self._set(dry_run=False)
        self.sent = []
        original_send = self.node._send_move_p

        def record(position, orientation, **kwargs):
            self.sent.append(np.asarray(position, dtype=float).copy())
            return original_send(position, orientation, **kwargs)

        self.node._send_move_p = record

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

    def _assert_descend_targets(self, pre, final):
        """进 DESCEND 后的目标: 只有一条指令, 所以没有路点队列可查。

        悬停高度 (PRE_OFFSET) 就是下降行程 —— 改它不该让断言变成「测的是另一个
        几何」, 2026-09-17 从 0.08 抬到 0.09 (9cm 下降) 时踩过这个坑。
        """
        self.assertEqual(self.node._waypoints, [], "下降不再分段 (没有路点队列)")
        np.testing.assert_allclose(
            self.node._final_grasp_target, final, atol=1e-9,
            err_msg="下降终点就是抓取点",
        )
        np.testing.assert_allclose(
            self.node._pre_grasp_target, pre, atol=1e-9,
            err_msg="悬停点 = 抓取点 + PRE_OFFSET",
        )

    def _feedback(self, position, motion_status=0):
        """写一帧「刚收到」的 TCP 位姿 + 臂状态 (时间戳取当前 ROS 时钟)。"""
        now = self.node.get_clock().now().nanoseconds * 1e-9
        self.node._tcp_pose = (now, np.asarray(position, dtype=float))
        self.node._tcp_orientation = (0.0, 0.0, 0.0, 1.0)
        self.node._arm_status = (now, _arm_status_msg(motion_status))

    def _candidate(self, position, track_id=1, quality=0.9, confidence=0.9):
        """发布一个候选 (走 _select_and_store, 与真实订阅路径同一入口)。"""
        self.node._select_and_store(
            [
                {
                    "track_id": track_id,
                    "grasp_mode": "side",
                    "confidence": confidence,
                    "quality": quality,
                    "position_camera": tuple(np.asarray(position, dtype=float)),
                }
            ]
        )

    def _age_candidate(self, seconds):
        """把最新候选的收到时刻往前挪 (模拟感知停发)。"""
        self.node._latest_candidate["received_at"] -= seconds

    # 下面三个是「模拟时间流逝」: 不做真实 sleep, 把计时起点往前挪。
    # 起点是 None 表示「当前没有在计时」, 此时无可提前。
    def _age(self, attribute, seconds):
        since = getattr(self.node, attribute)
        if since is not None:
            setattr(self.node, attribute, since - seconds)

    def _age_track_command(self, seconds=None):
        """把「上一条跟随指令」的时刻往前挪, 默认挪过整整一个跟随周期。

        默认值刻意按参数算而不是写死: 周期从 0.30 改成 2.0 之后, 写死 1.0 会让
        「目标动了就跟随」那些用例实际上卡在限速门上, 测的就不是死区/锁定了。
        """
        if seconds is None:
            seconds = float(self.node.get_parameter("track_min_command_interval").value)
        self._age("_track_last_command_time", seconds + 0.01)

    def _settle(self):
        """让「上一条跟随指令已经走完」成立, 不真等 settle_time。

        必须连 _track_stable_pose 一起摆: 停稳判据的第一帧永远返回「没停稳」
        (它要先记住一个位置才能判断位置有没有变), 只把计时起点往前挪是不够的。
        """
        self.node._track_stable_pose = np.asarray(
            self.node._tcp_pose[1], dtype=float
        )
        self.node._track_stable_since = time.monotonic() - (
            float(self.node.get_parameter("track_settle_time").value) + 0.01
        )

    # ── 摆位 ────────────────────────────────────────────────

    def _hover(self, center=GRASP_CENTER, track_id=1, mode="side"):
        """把节点推到「悬停路点刚刚到位, PRE_GRASP 那一拍刚跑完」的状态。

        走的是真实路径 (_tick_motion 判到位 -> _advance_after_motion), 而不是直接调
        _transition, 这样 PRE_GRASP 的去向接线也在被测范围内 —— 悬停之后是进
        TRACK_WAIT 还是直接进 DESCEND, 正是下面那几个用例要测的东西。

        mode 必须是节点内部那一套 ("side" / "top"): _pre_grasp_offset_z 与
        _skip_track_for_mode 都按它分支。默认参数下两种模式的悬停高度同值
        (代码默认 top_pre_grasp_offset_z = -1 = 不单独设), 所以 center + PRE_OFFSET
        对两种模式都成立。
        """
        self.node._locked_track_id = track_id
        self.node._grasp_mode = mode
        self.node._grasp_center_base = np.asarray(center, dtype=float)
        self.node._goal_orientation = (0.0, 0.0, 0.0, 1.0)
        pre = np.asarray(center, dtype=float) + np.array(
            [0.0, 0.0, self.node._pre_grasp_offset_z()]
        )
        self.node._pre_grasp_target = pre
        self.node._final_grasp_target = np.asarray(center, dtype=float)
        self._candidate(center, track_id=track_id)
        self.node._transition(self.node._STATE_PRE_GRASP, [pre])
        self._feedback(pre)
        self.node._goal_position = pre.copy()
        self.node._goal_sent = True
        self.node._tolerance_start = time.monotonic() - 10.0   # 容差已经守住够久
        self.node._tick_motion()
        self.sent.clear()
        return pre

    def _enter_track_wait(self, center=GRASP_CENTER, track_id=1, mode="side"):
        """同上, 但**要求**结果是进了 TRACK_WAIT (调用方按这个前提断言)。"""
        pre = self._hover(center, track_id, mode)
        assert self.node._state == self.node._STATE_TRACK_WAIT, (
            f"没能进入 TRACK_WAIT, 停在 {self.node._state}"
        )
        return pre


class EnterTrackWaitTest(_TrackTestCase):
    """悬停到位之后的去向。"""

    def test_top_mode_can_have_its_own_hover_height(self):
        """倒瓶 (top) 可以单独给一个悬停高度, side 不受影响 (2026-09-28)。

        为什么需要: top 是**工具朝下**抓, 末端原点(法兰)在夹爪上方一整个工具偏移
        (0.18m), 所以悬停抬高 1cm 法兰就高 1cm —— 同一组数 side 够得着、top 可能
        直接出界。不设 (<=0) 时两种模式都用通用值, 与改动前逐字相同。
        """
        self._set(top_pre_grasp_offset_z=-1.0)
        self.node._grasp_mode = "side"
        self.assertEqual(self.node._pre_grasp_offset_z(), PRE_OFFSET)
        self.node._grasp_mode = "top"
        self.assertEqual(
            self.node._pre_grasp_offset_z(), PRE_OFFSET,
            "没单独设时 top 也该用通用值 —— 否则等于偷偷改了现有行为",
        )

        self._set(top_pre_grasp_offset_z=0.05)
        self.node._grasp_mode = "top"
        self.assertEqual(self.node._pre_grasp_offset_z(), 0.05)
        self.node._grasp_mode = "side"
        self.assertEqual(
            self.node._pre_grasp_offset_z(), PRE_OFFSET, "side 不该被 top 的专用值影响"
        )

    def test_pre_grasp_goes_to_track_wait(self):
        pre = self._enter_track_wait()
        self.assertEqual(self.node._state, self.node._STATE_TRACK_WAIT)
        # 进入跟随时不额外发指令: 机械臂已经在悬停点上
        self.assertEqual(len(self.sent), 0)
        np.testing.assert_allclose(self.node._track_command, pre)

    # ── 倒瓶 (top) 不跟踪 (2026-09-29, 用户要求) ──────────────

    def test_top_skip_track_goes_straight_to_descend(self):
        """top 模式: 悬停到位直接下降, 不进 TRACK_WAIT, 也不等任何确认。

        现场病根 (2026-09-29): top 是工具朝下、从**正上方**抓, 机械臂一挪到瓶子
        正上方手眼相机就看不着瓶子了 —— 候选不再更新, _follow_target 判
        TRACK_TARGET_LOST 回待机位, 一轮都抓不成。所以那一模式不跟、直接降。
        """
        self._set(top_skip_track=True)
        center = GRASP_CENTER.copy()
        pre = self._hover(center, mode="top")
        self.assertEqual(
            self.node._state, self.node._STATE_DESCEND,
            "top + top_skip_track 应当直接进 DESCEND",
        )
        # 下降点就是**锁定目标那一刻**算好的抓取点 (WAIT_TARGET 那一份稳定坐标),
        # 不是悬停点、也不是下降前再读一次的候选。
        self._assert_descend_targets(pre, center)
        # 跳过跟随也不该凭空多出一条指令: 下降指令由 _tick_descend 发, 不在这里
        self.assertEqual(len(self.sent), 0)

    def test_top_skip_track_is_off_by_default(self):
        """不开开关时 top 与改动前逐字相同: 照样进 TRACK_WAIT 等确认。"""
        self._enter_track_wait(GRASP_CENTER.copy(), mode="top")
        self.assertEqual(self.node._state, self.node._STATE_TRACK_WAIT)

    def test_top_skip_track_leaves_side_alone(self):
        """side 从侧面抓, 相机看得见瓶子 —— 该跟还得跟。"""
        self._set(top_skip_track=True)
        pre = self._enter_track_wait(GRASP_CENTER.copy(), mode="side")
        self.assertEqual(
            self.node._state, self.node._STATE_TRACK_WAIT,
            "side 不受 top_skip_track 影响",
        )
        self.assertEqual(len(self.sent), 0)
        np.testing.assert_allclose(self.node._track_command, pre)

    def test_track_disabled_falls_back_to_old_two_stage_flow(self):
        """track_enabled=false 必须完全退回旧流程 (悬停到位直接下降)。"""
        self._set(track_enabled=False)
        center = GRASP_CENTER.copy()
        pre = np.asarray(center) + np.array([0.0, 0.0, PRE_OFFSET])
        self.node._locked_track_id = 1
        self.node._grasp_mode = "side"
        self.node._grasp_center_base = center
        self.node._goal_orientation = (0.0, 0.0, 0.0, 1.0)
        self.node._pre_grasp_target = pre
        self.node._final_grasp_target = center          # 旧流程里由 _tick_wait_target 算好
        self.node._transition(self.node._STATE_PRE_GRASP, [pre])
        self._feedback(pre)
        self.node._goal_position = pre.copy()
        self.node._goal_sent = True
        self.node._tolerance_start = time.monotonic() - 10.0
        self.node._tick_motion()
        self.assertEqual(self.node._state, self.node._STATE_DESCEND)
        self._assert_descend_targets(pre, center)

    def test_no_descend_command_before_confirm(self):
        """没按确认之前, 一条下降指令都不许有。"""
        pre = self._enter_track_wait()
        for _ in range(5):
            self.node._tick_track_wait()
        self.assertEqual(self.node._state, self.node._STATE_TRACK_WAIT)
        self.assertEqual(len(self.sent), 0)
        np.testing.assert_allclose(
            self.node._goal_position, pre,
            err_msg="_goal_position 在确认前不该变成抓取点",
        )


class FollowTest(_TrackTestCase):
    """跟随判据: 该死区就死区, 该让路就让路。"""

    def setUp(self):
        super().setUp()
        self.pre = self._enter_track_wait()

    def _move_target(self, delta):
        """目标移动 delta (夹爪系), 并把死区/限速/停稳三道门都摆成可通行。"""
        self._candidate(GRASP_CENTER + np.asarray(delta, dtype=float))
        self._settle()
        self._age_track_command()          # 距上一条指令已经过了整整一个周期

    def test_deadband_holds_the_arm_still(self):
        """目标没动够死区: 不发指令 —— 这是防真机抖动的主要手段。"""
        deadband = float(self.node.get_parameter("track_deadband").value)
        self._move_target([0.0, 0.0, 0.0])
        self._follow()
        self.assertEqual(len(self.sent), 0, "目标没动, 不该有指令")

        # 横向挪半个死区, 仍不发
        self._move_target([deadband * 0.5, 0.0, 0.0])
        self._follow()
        self.assertEqual(len(self.sent), 0, "不足死区不该发指令")

    def test_follow_after_deadband(self):
        deadband = float(self.node.get_parameter("track_deadband").value)
        self._move_target([deadband * 1.5, 0.0, 0.0])
        self._follow()
        self.assertEqual(len(self.sent), 1)
        expected = GRASP_CENTER + np.array([deadband * 1.5, 0.0, PRE_OFFSET])
        np.testing.assert_allclose(self.sent[0], expected, atol=1e-9)
        np.testing.assert_allclose(self.node._track_command, expected, atol=1e-9)

    def test_rate_gate_blocks_a_second_command(self):
        """死区 + 跟随周期是控制频率的硬下限, 刚发完不许再发。"""
        deadband = float(self.node.get_parameter("track_deadband").value)
        self._move_target([deadband * 1.5, 0.0, 0.0])
        self._follow()
        self.assertEqual(len(self.sent), 1)

        # 目标又动了两个死区, 机械臂也停稳了, 但距上一条指令还不到一个周期
        self._candidate(GRASP_CENTER + np.array([deadband * 3.0, 0.0, 0.0]))
        self._settle()
        self._follow()
        self.assertEqual(len(self.sent), 1, "周期内的第二条指令不能发出去")

    def test_settle_gate_blocks_overlapping_commands(self):
        """上一条指令还没走完就发新目标 = 两个目标重叠, 真机表现就是抖动。"""
        deadband = float(self.node.get_parameter("track_deadband").value)
        self._move_target([deadband * 1.5, 0.0, 0.0])
        self._follow()
        self.assertEqual(len(self.sent), 1)

        self._move_target([deadband * 3.0, 0.0, 0.0])
        # 机械臂仍差 20mm 才到上一条指令 (小于 track_max_lag), 且没有停稳
        self._feedback(self.node._track_command - np.array([0.02, 0.0, 0.0]))
        self.node._track_stable_pose = None
        self.node._track_stable_since = None
        self._follow()
        self.assertEqual(len(self.sent), 1, "没停稳就不该重发")

        # 停稳之后放行
        self._settle()
        self._follow()
        self.assertEqual(len(self.sent), 2, "停稳后应该跟上")

    def test_max_lag_escape_hatch(self):
        """目标跑得比机械臂快时不能傻等: 落后超过 track_max_lag 就直接重发。"""
        deadband = float(self.node.get_parameter("track_deadband").value)
        self._move_target([deadband * 1.5, 0.0, 0.0])
        self._follow()
        self.assertEqual(len(self.sent), 1)

        self._move_target([deadband * 3.0, 0.0, 0.0])
        # 距上一条指令 80mm > track_max_lag (50mm), 而且完全没有停稳
        self._feedback(self.node._track_command - np.array([0.08, 0.0, 0.0]))
        self.node._track_stable_pose = None
        self.node._track_stable_since = None
        self._follow()
        self.assertEqual(len(self.sent), 2, "落后太多时必须重发, 跟得上优先")

    def test_other_track_id_is_ignored_while_locked(self):
        """锁定了 track_id=1, 旁边出现一个质量更高的 2 号也不能跟过去。"""
        self._candidate(GRASP_CENTER + np.array([0.15, 0.0, 0.0]), track_id=2, quality=5.0)
        self._age_track_command()          # 限速门放行, 挡住它的只能是「锁定」
        self._settle()
        self._follow()
        self.assertEqual(len(self.sent), 0, "跟到别的物体上是最危险的失败")
        self.assertEqual(self.node._latest_candidate["track_id"], 1)

    def test_stale_candidate_fails_the_task(self):
        """感知停了 -> 判失败回待机位, 不再原地悬停等人 (2026-09-17)。"""
        self._set(return_to_standby_on_finish=False)
        self._move_target([0.10, 0.0, 0.0])
        self._age_candidate(
            float(self.node.get_parameter("max_result_age").value) + 1.0
        )
        self._follow()
        self.assertEqual(len(self.sent), 0, "没有新鲜坐标就不该动")
        self.assertEqual(self.node._state, self.node._STATE_IDLE, "跟丢应当判失败")

    def test_no_candidate_at_all_fails(self):
        """从未收到候选 (感知根本没起) 也是跟丢, 同样判失败。"""
        self._set(return_to_standby_on_finish=False)
        self.node._latest_candidate = None
        self._follow()
        self.assertEqual(len(self.sent), 0)
        self.assertEqual(self.node._state, self.node._STATE_IDLE)

    def test_cadence_is_one_command_per_period(self):
        """周期到了才发第二条 —— 而且**真的**是参数里的那个周期。

        这条同时钉住「停稳判据逐拍滚动」那个修复: 如果停稳门还是等三道门放行才更新
        基线, 发完第一条之后下一拍必然被判「没停稳」, 周期会被吃成两倍
        (1s -> 2s), 这里就会在 period 处只看到 1 条指令。
        """
        period = float(self.node.get_parameter("track_min_command_interval").value)
        deadband = float(self.node.get_parameter("track_deadband").value)
        self._move_target([deadband * 1.5, 0.0, 0.0])
        self._follow()
        self.assertEqual(len(self.sent), 1)
        sent_at = self.node._track_last_command_time

        # 目标又挪了一个半死区, 机械臂以 50ms 的节拍一步步走过去, 然后停在新位置上。
        self._candidate(GRASP_CENTER + np.array([deadband * 3.0, 0.0, 0.0]))
        arrived = self.node._track_command + np.array([deadband * 1.5, 0.0, 0.0])
        for behind in (0.020, 0.010, 0.001, 0.0):    # 每次都还在动 -> 停稳基线跟着更新
            self._feedback(arrived - np.array([behind, 0.0, 0.0]))
            self.node._follow_target(sent_at + 0.05)

        self.node._follow_target(sent_at + period - 0.01)
        self.assertEqual(len(self.sent), 1, f"还没到 {period}s 不该发第二条")
        self.node._follow_target(sent_at + period + 0.01)
        self.assertEqual(len(self.sent), 2, f"到了 {period}s 就该跟上 (不是两倍周期)")

    def test_runaway_target_is_rejected_by_distance_limit(self):
        """同一个 track_id 的位姿突然跳一大截 (拟合翻面) 不是瓶子在动: 判失败回位。"""
        self._set(return_to_standby_on_finish=False)
        self._move_target([0.0, 0.0, 0.0])
        # 同一个 track_id, 但坐标跳了 90cm —— 瓶子不会这么动, 这是感知出错
        self._move_target([0.90, 0.0, 0.0])
        self._follow()
        self.assertEqual(len(self.sent), 0)
        self.assertEqual(self.node._state, self.node._STATE_IDLE, "应当判失败")

    def _follow(self):
        """一次跟随判定。时间参数直接给, 免得依赖真实时钟。"""
        self.node._follow_target(time.monotonic())


class ConfirmDescendTest(_TrackTestCase):
    """确认之后的下降: 从哪开始, 抓到哪。"""

    def setUp(self):
        super().setUp()
        self.pre = self._enter_track_wait()

    def _confirm(self):
        self.node._enter_pressed = True
        self.node._tick_track_wait()

    def test_fresh_candidate_recomputes_the_descend_targets(self):
        """确认时用目标当前位置重算 —— 这正是跟随的意义: 瓶子挪到哪就抓到哪。"""
        moved = np.array([0.06, 0.0, 0.0])
        self._candidate(GRASP_CENTER + moved)
        self._confirm()
        self.assertEqual(self.node._state, self.node._STATE_DESCEND)
        self._assert_descend_targets(GRASP_CENTER + moved + np.array([0.0, 0.0, PRE_OFFSET]),
                                     GRASP_CENTER + moved)

    def test_stale_candidate_descends_vertically_from_the_hover_point(self):
        """候选过期时原地垂直下降: 横向一动不动, 绝不拿过期坐标去抓。"""
        self._age_candidate(
            float(self.node.get_parameter("max_result_age").value) + 1.0
        )
        self._confirm()
        self.assertEqual(self.node._state, self.node._STATE_DESCEND)
        final = self.pre - np.array([0.0, 0.0, PRE_OFFSET])
        self._assert_descend_targets(self.pre, final)
        # 纯垂直: 横坐标与当前悬停点逐位相同
        self.assertAlmostEqual(float(final[0]), float(self.pre[0]), places=12)
        self.assertAlmostEqual(float(final[1]), float(self.pre[1]), places=12)

    def test_no_candidate_at_all_still_descends_vertically(self):
        """从未收到候选 (dry_run 常遇到): 也要能确认下降, 且仍是纯垂直。"""
        self.node._latest_candidate = None
        self._confirm()
        self.assertEqual(self.node._state, self.node._STATE_DESCEND)
        np.testing.assert_allclose(self.node._pre_grasp_target, self.pre, atol=1e-9)

    def test_far_moved_target_is_checked_against_the_workspace(self):
        """目标移到工作空间外时必须拒掉, 而不是硬跟过去。"""
        self._set(return_to_standby_on_finish=False)
        self._candidate(GRASP_CENTER + np.array([0.0, 0.75, 0.0]))   # y=0.75 出界
        self._confirm()
        self.assertEqual(self.node._state, self.node._STATE_IDLE, "应当判失败")

    def test_enter_from_track_wait_does_not_leak_into_grasp_wait(self):
        """两次确认必须各按各的: 第一次的 Enter 不能顺延成第二次的确认。"""
        self._confirm()
        self.assertEqual(self.node._state, self.node._STATE_DESCEND)
        self.assertFalse(self.node._enter_pressed)
        self.node._enter_pressed = True                  # 下降途中被误按
        self.node._transition(self.node._STATE_GRASP_WAIT)
        self.assertFalse(
            self.node._enter_pressed, "进入 GRASP_WAIT 时应当清掉遗留按键"
        )

    def test_confirm_topic_works_in_track_wait(self):
        """话题确认与 Enter 等效 (ros2 launch 下 stdin 不是终端时的唯一通道)。"""
        from std_msgs.msg import Bool

        message = Bool()
        message.data = True
        self.node._on_grasp_confirm(message)
        self.assertTrue(self.node._confirm_received)
        self.node._tick_track_wait()
        self.assertEqual(self.node._state, self.node._STATE_DESCEND)


class AutoDescendTest(_TrackTestCase):
    """全自动下降: 机械臂到位后物体位置连续不变够久, 自己下降抓取 (2026-09-22)。

    守三件事:
      - **计时的起点是「机械臂到位」** —— 物体停住但机械臂还在横移时不许起数;
      - **锚点式窗口** —— 慢速漂移累计超过容差必须重开窗口 (只比相邻两帧就会漏);
      - **新鲜度门** —— 上游停发时「坐标没变」只是「没人在看」, 窗口必须作废。
    """

    def setUp(self):
        super().setUp()
        self.pre = self._enter_track_wait()
        # 机械臂已停在悬停点上 —— 自动下降的计时从这一刻才算开始。
        self._settle()

    def _tick(self):
        """一整拍 TRACK_WAIT (走真实接线, 不是直接调判据)。"""
        self.node._tick_track_wait()

    def _follow(self):
        self.node._follow_target(time.monotonic())

    def _object_stable(self, held=None):
        """摆成「物体位置已经连续不变 held 秒」。

        两个变量都要摆, 理由同 _settle(): 锚点为空的那一帧永远返回「还在变」,
        它得先记住一个位置才能判断位置有没有变。
        """
        if held is None:
            held = float(self.node.get_parameter("auto_grasp_stable_time").value) + 0.01
        self.node._auto_grasp_ref = np.asarray(
            self.node._latest_candidate["position_base"], dtype=float
        )
        self.node._auto_grasp_since = time.monotonic() - held

    def _epsilon(self):
        return float(self.node.get_parameter("auto_grasp_stable_epsilon").value)

    def _assert_still_tracking(self, where):
        """还在 TRACK_WAIT 里 —— 循环驱动时必须逐拍查。

        下降一旦发生, `_transition` 会把 `_track_command` 清成 None, 后面的
        `_tick_track_wait` 就会崩在 `_follow_target` 里, 报成 ERROR 而不是
        「不该下降」。先查状态, 失败信息才指向真正的原因。
        """
        self.assertEqual(
            self.node._state, self.node._STATE_TRACK_WAIT,
            f"{where}: 还在悬停跟随, 不该下降",
        )

    def test_stable_object_auto_descends(self):
        """机械臂已到位 + 物体位置稳够久 -> 自己下降, 全程没碰 Enter。"""
        self._object_stable()
        self._tick()
        self.assertEqual(self.node._state, self.node._STATE_DESCEND)
        self.assertFalse(self.node._enter_pressed, "不该伪造人工按键")
        self.assertFalse(self.node._confirm_received, "也不该借道话题确认标志")
        # 与人工确认走同一条 _confirm_descend: 目标同样是「按最新候选重算」
        self._assert_descend_targets(
            GRASP_CENTER + np.array([0.0, 0.0, PRE_OFFSET]), GRASP_CENTER
        )

    def test_no_descend_before_the_hold_time(self):
        """刚到位的头几拍不许下降 (没有这一条就是「一悬停就往下抓」)。"""
        for index in range(10):
            # 逐拍先查状态: 一旦提前下降, 后面的 _tick_track_wait 会崩在
            # _follow_target 里 (_track_command 已被 _transition 清成 None),
            # 报成 ERROR 反而看不出是这里错了。
            self._assert_still_tracking(index)
            self._tick()
        self.assertEqual(self.node._state, self.node._STATE_TRACK_WAIT)
        self.assertEqual(len(self.sent), 0)

    def test_creeping_object_never_auto_descends(self):
        """慢速漂移: 每拍只挪 0.6 个容差, 但**累计**超过容差就必须重开窗口。

        这条是锚点式判据的规范。若基准写成每拍都重设 (只比相邻两帧), 偏差永远
        只有一步的量, 会被误判成「一直没变」, 于是窗口熬满 3s 就往下抓。
        """
        step = self._epsilon() * 0.6
        self._tick()                                   # 记下锚点
        for index in range(6):
            self._assert_still_tracking(index)
            self._candidate(GRASP_CENTER + np.array([step * (index + 1), 0.0, 0.0]))
            self._age("_auto_grasp_since", 1.1)        # 每拍都让「已稳定」看起来涨
            self._tick()
        self._assert_still_tracking("末")

    def test_moved_object_restarts_the_window(self):
        """物体中途挪超容差: 窗口从零重开, 不下降。"""
        self._object_stable()
        moved = GRASP_CENTER + np.array([self._epsilon() * 1.5, 0.0, 0.0])
        self._candidate(moved)
        self._tick()
        self.assertEqual(self.node._state, self.node._STATE_TRACK_WAIT)
        np.testing.assert_allclose(self.node._auto_grasp_ref, moved, atol=1e-9)

    def test_arm_not_settled_blocks_the_auto_descend(self):
        """**核心用例**: 物体位置稳够久, 但机械臂还在横移 -> 不许下降。

        这时下降会把横移和整段下降并成一条指令, 而 descend_dwell 可能在机械臂
        到位前就走完 -> 半空闭合夹爪。
        """
        self._object_stable()
        self.node._track_stable_pose = None      # 机械臂还在动
        self.node._track_stable_since = None
        self._tick()
        self.assertEqual(self.node._state, self.node._STATE_TRACK_WAIT)
        self.assertIsNone(
            self.node._auto_grasp_ref, "没到位时窗口应当作废, 到位后从零起数"
        )

        # 到位 (停稳) 之后才允许下降
        self._settle()
        self._object_stable()
        self._tick()
        self.assertEqual(self.node._state, self.node._STATE_DESCEND)

    def test_stale_candidate_blocks_the_auto_descend(self):
        """上游停发时「坐标没变」只是「没人在看」—— 窗口必须作废。

        注意 max_result_age (4.0s) 比 auto_grasp_stable_time (3.0s) **长**, 所以
        _follow_target 的过期判失败在这一刻还没到, 这道新鲜度门必须自己存在。
        """
        self._object_stable()
        self._age_candidate(
            float(self.node.get_parameter("track_min_command_interval").value) + 0.1
        )
        self._tick()
        self.assertEqual(self.node._state, self.node._STATE_TRACK_WAIT)
        self.assertIsNone(self.node._auto_grasp_ref)

    def test_manual_enter_still_wins(self):
        """自动条件全满足, 但同一拍人按了 Enter —— 人工优先, 且立刻下降。"""
        self._object_stable()
        self.node._enter_pressed = True
        self._tick()
        self.assertEqual(self.node._state, self.node._STATE_DESCEND)

    def test_manual_confirm_topic_still_wins(self):
        from std_msgs.msg import Bool

        self._object_stable()
        message = Bool()
        message.data = True
        self.node._on_grasp_confirm(message)
        self._tick()
        self.assertEqual(self.node._state, self.node._STATE_DESCEND)

    def test_auto_disabled_falls_back_to_enter(self):
        """auto_grasp_stable_time=0 必须完全退回「等人按 Enter」。"""
        self._set(auto_grasp_stable_time=0.0)
        self._settle()
        for index in range(10):
            self._assert_still_tracking(index)
            self._tick()
        self.assertEqual(len(self.sent), 0)

        self.node._enter_pressed = True
        self._tick()
        self.assertEqual(self.node._state, self.node._STATE_DESCEND)

    def test_disabling_clears_the_window_so_reenabling_starts_fresh(self):
        """运行中关掉再打开: 关闭期间窗口必须被清掉, 打开时从零起数。

        否则可能出现「物体位置和关闭前几乎相同 + 旧的计时起点」-> 打开第一拍就下降,
        也就是参数一改机械臂立刻动。
        """
        self._object_stable()
        self._set(auto_grasp_stable_time=0.0)
        self._tick()
        self.assertIsNone(self.node._auto_grasp_ref, "关掉的那一拍就该清掉窗口")

        self._set(auto_grasp_stable_time=3.0)
        self._tick()
        self.assertEqual(self.node._state, self.node._STATE_TRACK_WAIT)

    def test_follow_command_restarts_the_window(self):
        """发出一条跟随指令 = 目标动了, 窗口必须重开。"""
        self._object_stable()
        deadband = float(self.node.get_parameter("track_deadband").value)
        self._candidate(GRASP_CENTER + np.array([deadband * 1.5, 0.0, 0.0]))
        self._settle()
        self._age_track_command()
        self._follow()
        self.assertEqual(len(self.sent), 1, "前提: 这一拍真的发出了跟随指令")
        self.assertIsNone(self.node._auto_grasp_ref)

    def test_window_is_cleared_on_entering_and_leaving(self):
        """窗口属于 TRACK_WAIT: 进状态与离开状态都必须清 —— 跨轮次不许顺延。"""
        self._object_stable()
        self.node._transition(self.node._STATE_DESCEND)
        self.assertIsNone(self.node._auto_grasp_ref, "离开时应当清")

        self.node._transition(self.node._STATE_IDLE)
        self.node._enter_track_wait()
        self.assertIsNone(self.node._auto_grasp_ref, "进状态时应当清")

    def test_dry_run_without_tcp_feedback_still_descends(self):
        """dry_run 收不到 /feedback/tcp_pose 时, 「到位」无法判 —— 不能因此永不触发。

        与 _follow_target 把停稳门包在 not dry_run 里是同一个惯例。
        """
        self._set(dry_run=True)
        self.node._tcp_pose = None
        self._object_stable()
        self._tick()
        self.assertEqual(self.node._state, self.node._STATE_DESCEND)


class FailureDuringTrackTest(_TrackTestCase):
    """跟随期间出事也要回待机位, 不能把机械臂丢在瓶子上面。"""

    def test_track_wait_is_in_the_return_to_standby_set(self):
        self._enter_track_wait()
        self._set(return_to_standby_on_finish=True, dry_run=False)
        self.assertTrue(self.node._standby_ready, "前提: 默认待机位要能通过校验")
        self.node._return_pending = True
        self.assertTrue(
            self.node._should_return_to_standby(),
            "TRACK_WAIT 必须在允许回位的状态集合里",
        )

    def test_feedback_loss_during_track_fails_the_task(self):
        self._set(return_to_standby_on_finish=False)
        self._enter_track_wait()
        self.node._tcp_pose = None                      # 反馈断了
        self.node._tick_track_wait()
        self.assertEqual(self.node._state, self.node._STATE_IDLE)


class ConfigConsistencyTest(_TrackTestCase):
    """节点默认值 vs config/bottle_grasp_executor.yaml。"""

    @classmethod
    def setUpClass(cls):
        super().setUpClass()
        path = os.path.join(PKG_ROOT, "config", "bottle_grasp_executor.yaml")
        with open(path, "r", encoding="utf-8") as handle:
            cls.yaml_params = yaml.safe_load(handle)["bottle_grasp_executor"][
                "ros__parameters"
            ]

    def test_new_params_match_yaml(self):
        for key in NEW_TRACK_KEYS:
            with self.subTest(param=key):
                self.assertIn(key, self.yaml_params, f"{key} 没写进 yaml")
                default = self.node.get_parameter(key).value
                actual = self.yaml_params[key]
                self.assertIs(type(actual), type(default), f"{key} 类型不一致")
                self.assertEqual(actual, default, f"{key} 两边取值不一致")

    def test_hover_offset_matches_node_and_yaml(self):
        """悬停高度三处必须一致: 本文件的 PRE_OFFSET / 节点默认值 / yaml。

        它同时是单步下降的下降距离 (pre_grasp_offset_z - grasp_offset_z), 所以
        改它等于改真机的下降行程 —— 对不上就该红, 而不是让断言里的 0.39 变成
        「测的是另一个几何」。
        """
        self.assertEqual(PRE_OFFSET, self.node.get_parameter("pre_grasp_offset_z").value)
        self.assertEqual(PRE_OFFSET, self.yaml_params["pre_grasp_offset_z"])
        self.assertEqual(0.0, self.yaml_params["grasp_offset_z"])
        # 悬停点必须仍在默认工作空间 (workspace_z_max) 内
        self.assertLess(
            GRASP_CENTER[2] + PRE_OFFSET, self.yaml_params["workspace_z_max"]
        )

    def test_deadband_is_smaller_than_max_lag(self):
        """死区必须小于「落后上限」, 否则例外通道恒开, 停稳门等于没有。"""
        self.assertLess(
            self.yaml_params["track_deadband"],
            self.yaml_params["track_max_lag"],
        )

    def test_settle_time_and_epsilon_are_usable(self):
        self.assertGreater(self.yaml_params["track_settle_time"], 0.0)
        self.assertGreater(self.yaml_params["track_stable_epsilon"], 0.0)
        self.assertGreater(self.yaml_params["track_min_command_interval"], 0.0)

    def test_auto_grasp_thresholds_are_usable(self):
        """自动下降的两个数都有**实测**依据, 不是随手填的 (2026-09-22)。"""
        # 容差不能小: 静止瓶子的 position_base 峰峰值实测 3.1~4.2mm
        # (112 帧 / 13.9s), 滚动 3s 窗口在 eps=2.0mm 时 0/73 通过, 到 5mm 才 73/73。
        # 拿 track_stable_epsilon (0.5mm) 那种量级来判, 这个功能一次都不会触发。
        self.assertGreaterEqual(self.yaml_params["auto_grasp_stable_epsilon"], 0.005)
        # 也不该比「愿意跟随的死区」还粗 —— 那等于承认「已经动到不想跟了, 还算没动」。
        self.assertLessEqual(
            self.yaml_params["auto_grasp_stable_epsilon"],
            self.yaml_params["track_deadband"],
        )
        self.assertGreater(self.yaml_params["auto_grasp_stable_time"], 0.0)

    def test_cadence_is_sane(self):
        """跟随周期是一个现场可调的数 (2026-09-17 真机调成 1.0s), 不钉死秒数。

        钉死的话每次现场调速都要改测试, 而这条测试真正要守的东西只有两条:
        周期是正的, 且比过期阈值短 (下一条)。原来的「2s 是需求」已经不成立 ——
        需求是「每周期读一次最新坐标」, 周期多长由现场定。
        """
        period = self.yaml_params["track_min_command_interval"]
        self.assertGreater(period, 0.0)
        self.assertLess(period, self.yaml_params["max_result_age"])

    def test_staleness_threshold_exceeds_the_cadence(self):
        """过期阈值必须大于跟随周期, 否则还没到下一次读数就先判失败。"""
        period = self.yaml_params["track_min_command_interval"]
        max_age = self.yaml_params["max_result_age"]
        self.assertGreater(
            max_age, period,
            f"max_result_age ({max_age}) 必须大于 track_min_command_interval "
            f"({period}): 相等或更小时, 跟丢判失败会先于下一次读坐标发生",
        )


if __name__ == "__main__":
    unittest.main()
