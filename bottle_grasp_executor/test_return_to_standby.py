#!/usr/bin/env python3
"""bottle_grasp_executor 「出错也要回位」的回归测试 (2026-09-16)。

覆盖两件事:
  1. 机械臂报故障 / 反馈断了时**仍然**回待机位 (return_to_standby_on_error)。
     旧行为是直接跳过回位、把机械臂丢在原地 —— 2026-09-16 16:38 真机就是这样
     停在抓取点上方的 (回位途中 bridge 挂掉 -> TCP_FEEDBACK_STALE -> 回位放弃)。
  2. 这种回位走的**盲走**路径: 只发一次绝对待机位、**不判到位**;
     反馈恢复就交回正常判据; 等满 blind_home_dwell 就结束, 并明确报「未能确认」。
     (2026-09-21 起「拆段」不再是这两条路的区别 —— 正常回位也不拆段了, 只有到位
     判定不同, 见 test_midway_feedback_loss_switches_to_blind_judgement。)

不连机械臂、不连 CAN: 反馈直接写内部缓存, 计时起点直接往前挪 (逻辑测试,
不是 time.monotonic() 的测试)。

跑法 (在包根目录):
  source /opt/ros/humble/setup.bash && source install/setup.bash
  /usr/bin/python3 -m unittest -v test_return_to_standby
"""

import os
import unittest
from types import SimpleNamespace

import numpy as np
import rclpy
import yaml

from bottle_grasp_executor.bottle_grasp_executor_node import BottleGraspExecutor

PKG_ROOT = os.path.dirname(os.path.abspath(__file__))


def _arm_status_msg(err_status=0, motion_status=0):
    """够用的 H3001ArmStatus 替身 (_feedback_problem 与 _ik_rejection_problem 会看这些字段)。

    固件 IK 判决那 4 个字段留默认: ik_status_valid=0 在 _ik_rejection_problem 里
    **一律不判**, 所以这里不填就等于「本次没收到拒收判决」—— 回位用例要验的正是
    「命令发出去了、机械臂在慢慢挪」, 不该被这条新通道抢走。
    """
    return SimpleNamespace(
        err_status=err_status,
        ctrl_mode=1,
        motion_status=motion_status,
        joint_angle_limit=[0] * 6,
        communication_status_joint=[0] * 6,
        ik_status=0,
        ik_status_valid=0,
        ik_joint_limit_fault=0,
        ik_pose_pending=0,
    )


class _RecordingLogger:
    """把节点日志收进列表, 供断言用 (顺带让测试输出干净)。"""

    def __init__(self):
        self.messages = []

    def info(self, message):
        self.messages.append(("info", str(message)))

    def warn(self, message):
        self.messages.append(("warn", str(message)))

    def error(self, message):
        self.messages.append(("error", str(message)))

    def text(self):
        return "\n".join(message for _, message in self.messages)

    def has(self, needle):
        return any(needle in message for _, message in self.messages)


class _NullPublisher:
    """替掉真机的 move_p 发布器: 用例只做记录, **一条指令都不许发出去**。

    2026-09-18 16:49 真机事故: 这些用例是拿 dry_run=False 跑的, 而 `_send_move_p` 在
    dry_run=False 分支里会真的 publish 到 /control/move_p —— h3001_bridge 开着的时候
    跑一遍测试就把机械臂开动了 (详见 test_place.py 里同名类的注释)。本文件尤其危险:
    回位用的**代码默认待机位**与 yaml 现场那组差着一整个腕部朝向。
    """

    def __init__(self):
        self.published = []

    def publish(self, message):
        self.published.append(message)


class _StandbyReturnTestCase(unittest.TestCase):
    """每个用例一个干净节点; 反馈靠 _feedback() 直接写缓存。"""

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
        # 偏移只作用在「上报系 <-> 夹爪系」边界上, 与回位逻辑无关, 置零便于手算。
        self.node._gripper_offset = np.zeros(3)
        self.log = _RecordingLogger()
        self.node.get_logger = lambda: self.log
        self.sent = []
        original_send = self.node._send_move_p

        def record(position, orientation, **kwargs):
            self.sent.append(np.asarray(position, dtype=float).copy())
            return original_send(position, orientation, **kwargs)

        self.node._send_move_p = record
        # 代码默认是 False (ros2 run 直接起节点时不该突然多一段运动), yaml 里是 true。
        # dry_run 代码默认是 True, 而「机械臂可不可控」只在真机模式下才判 —— 回位这套
        # 逻辑本来就属于真机路径, 所以测试里一律按 dry_run=False 跑。
        self._set(return_to_standby_on_finish=True, dry_run=False)

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

    def _feedback(self, position, err_status=0, motion_status=0):
        """写一帧「刚收到」的反馈 (时间戳取当前 ROS 时钟)。"""
        now = self.node.get_clock().now().nanoseconds * 1e-9
        self.node._tcp_pose = (now, np.asarray(position, dtype=float))
        self.node._tcp_orientation = (0.0, 0.0, 0.0, 1.0)
        self.node._arm_status = (now, _arm_status_msg(err_status, motion_status))

    def _break_feedback(self):
        """模拟 bridge 挂掉: 反馈再也收不到。"""
        self.node._tcp_pose = None
        self.node._arm_status = None

    def _start_task(self, position=(0.30, 0.10, 0.12)):
        """把节点摆到「任务进行中、可回位」的状态 (PRE_GRASP, 一个路点)。"""
        self._feedback(position)
        self.node._return_pending = True
        self.node._transition(
            self.node._STATE_PRE_GRASP, [np.asarray(position, dtype=float)]
        )

    def _age(self, attribute, seconds):
        since = getattr(self.node, attribute)
        if since is not None:
            setattr(self.node, attribute, since - seconds)

    @property
    def standby(self):
        return np.asarray(self.node._standby_position, dtype=float)

    # ── 1. 「该不该回位」的判据 ──────────────────────────────

    def test_faulted_robot_still_returns_by_default(self):
        """报故障不再是「不回位」的理由 (这正是本次改动要修的缺口)。"""
        self._start_task()
        self._feedback((0.30, 0.10, 0.12), err_status=2048)   # J4 的 motor_error_code
        self.assertTrue(self.node._should_return_to_standby())

    def test_stale_feedback_still_returns_by_default(self):
        self._start_task()
        self._break_feedback()
        self.assertTrue(self.node._should_return_to_standby())

    def test_opt_out_restores_old_behaviour(self):
        """置 false 时退回旧行为: 不可控就不回位。"""
        self._set(return_to_standby_on_error=False)
        self._start_task()
        self._break_feedback()
        self.assertFalse(self.node._should_return_to_standby())
        self._feedback((0.30, 0.10, 0.12))      # 可控时照常回位
        self.assertTrue(self.node._should_return_to_standby())

    def test_no_task_running_does_not_return(self):
        """IDLE 里没有任务在跑, 不该凭空多出一段运动。"""
        self._feedback((0.30, 0.10, 0.12))
        self.node._transition(self.node._STATE_IDLE)
        self.assertFalse(self.node._should_return_to_standby())

    # ── 2. 盲走回位 ────────────────────────────────────────

    def test_fault_triggers_blind_home_with_single_absolute_command(self):
        self._start_task()
        self._break_feedback()
        self.node._fail("测试: 机器人不可控")
        self.assertEqual(self.node._state, self.node._STATE_HOME)
        self.assertTrue(self.node._home_blind)
        self.assertEqual(len(self.node._waypoints), 1, "盲走不拆段, 只保留待机位本身")

        self.node._tick_motion()
        self.assertEqual(len(self.sent), 1, "盲走只发一次")
        np.testing.assert_allclose(self.sent[0], self.standby)
        self.assertTrue(self.log.has("盲走"))

    def test_blind_home_holds_until_dwell_elapses(self):
        self._set(blind_home_dwell=5.0)
        self._start_task()
        self._break_feedback()
        self.node._fail("测试: 机器人不可控")
        self.node._tick_motion()
        self._age("_blind_started", 1.0)
        self.node._tick_motion()
        self.assertEqual(self.node._state, self.node._STATE_HOME, "没等满就不该结束")
        self.assertEqual(len(self.sent), 1, "等待期间不能反复重发")

    def test_blind_home_ends_with_unconfirmed_warning_not_success(self):
        self._set(blind_home_dwell=1.0)
        self._start_task()
        self._break_feedback()
        self.node._fail("测试: 机器人不可控")
        self.node._tick_motion()
        self._age("_blind_started", 2.0)
        self.node._tick_motion()
        self.assertEqual(self.node._state, self.node._STATE_IDLE)
        self.assertTrue(self.log.has("无法确认是否到位"))
        self.assertFalse(
            self.log.has("已回到待机位"), "没确认就不能说已回到待机位"
        )

    def test_feedback_recovery_hands_back_to_normal_judgement(self):
        self._set(blind_home_dwell=5.0)
        self._start_task()
        self._break_feedback()
        self.node._fail("测试: 机器人不可控")
        self.node._tick_motion()
        self.assertTrue(self.node._home_blind)

        # bridge 回来了, 而且机械臂就停在待机位上
        self._feedback(self.standby)
        self.node._tick_motion()
        self.assertFalse(self.node._home_blind, "反馈恢复后应退回正常判据")
        self.assertEqual(self.node._state, self.node._STATE_HOME)

        # 正常判据接着走完: 容差守住 standby_settle_time 后放行
        self.node._tick_motion()                      # 起算 settle
        self._age("_tolerance_start", 1.0)
        self.node._tick_motion()
        self.assertEqual(self.node._state, self.node._STATE_IDLE)
        self.assertTrue(
            self.log.has("已回到待机位"), "反馈恢复后应走正常路径并报已回到待机位"
        )

    def test_midway_feedback_loss_switches_to_blind_judgement(self):
        """正常进 HOME, 走到一半 bridge 挂掉 -> 转盲走 (到位判定交给 blind_home_dwell)。

        2026-09-21 起正常回位**本来就是一条绝对指令** (不再拆段, 见
        _home_time_multiplier_for), 所以「转盲走」这条路上真正变的只有**判据**:
        从「容差 + settle」换成「等满 blind_home_dwell」。队列清成待机位这步退化成
        空操作, 但 _blind_home_fallback 会顺手把 _goal_sent 复位, 于是同一条绝对
        待机位**再发一遍** —— 目标一模一样, 只是让刚失去反馈的这段时间从这一刻重新
        计时。重发是幂等的, 这条用例把它钉下来 (免得以后有人看到两条一模一样的指令
        以为是 bug 去「修」)。
        """
        self._start_task(position=(0.55, 0.30, 0.05))
        self.node._fail("测试: 超时")           # 此时反馈正常 -> 走正常回位
        self.assertFalse(self.node._home_blind)
        self.assertEqual(len(self.node._waypoints), 1, "回位不再拆段: 一条绝对待机位")
        np.testing.assert_allclose(self.node._waypoints[0], self.standby)

        self.node._tick_motion()
        self.assertEqual(len(self.sent), 1)
        np.testing.assert_allclose(self.sent[0], self.standby)

        self._break_feedback()
        self.node._tick_motion()                # 同一 tick 内转盲走 + 重发绝对待机位
        self.assertTrue(self.node._home_blind)
        self.assertEqual(len(self.node._waypoints), 1)
        self.assertEqual(len(self.sent), 2)
        np.testing.assert_allclose(self.sent[-1], self.standby)
        self.assertTrue(self.log.has("转盲走"))

    def test_state_other_than_home_clears_blind_flag(self):
        """盲走标记只在 HOME 期间有效, 否则下一个任务会继承它。"""
        self._start_task()
        self._break_feedback()
        self.node._fail("测试: 机器人不可控")
        self.assertTrue(self.node._home_blind)
        self.node._transition(self.node._STATE_IDLE)
        self.assertFalse(self.node._home_blind)
        self.assertIsNone(self.node._blind_started)

    # ── 3. 一条绝对指令的时间预算 (2026-09-21) ──────────────

    # 离待机位约 0.845 m —— 远超 max_command_distance (0.50), 用来验豁免与倍数。
    FAR_START = (0.70, 0.50, 0.10)

    def _home_from(self, position, **params):
        """从 position 走正常 (非盲走) 回位, 返回待机位。"""
        if params:
            self._set(**params)
        self._start_task(position=position)
        self.node._fail("测试: 超时")
        self.assertEqual(self.node._state, self.node._STATE_HOME)
        self.assertFalse(self.node._home_blind, "反馈正常就该走正常回位")
        return self.standby

    def _leg_distance(self, position):
        return float(np.linalg.norm(np.asarray(position, dtype=float) - self.standby))

    def test_far_home_is_one_absolute_command(self):
        """超过 max_command_distance 的回位：一条指令、照发, 并打一行 WARN。"""
        self.assertGreater(
            self._leg_distance(self.FAR_START),
            float(self.node.get_parameter("max_command_distance").value),
            "测试前提: 起点必须超过单条指令上限",
        )
        self._home_from(self.FAR_START)
        self.assertEqual(len(self.node._waypoints), 1, "回位不再拆段")
        np.testing.assert_allclose(self.node._waypoints[0], self.standby)

        self.node._tick_motion()
        self.assertEqual(len(self.sent), 1, "超限也要发出去 —— 回位段豁免那个上限")
        np.testing.assert_allclose(self.sent[0], self.standby)
        self.assertEqual(
            self.node._state, self.node._STATE_HOME, "超限不该让回位判失败"
        )
        self.assertTrue(
            self.log.has("回位段豁免"), "豁免要有痕: 少了上限那层兜底, 日志得说清"
        )

    def test_home_timeout_budget_is_the_old_segment_count(self):
        """合成一条指令后, 超时预算按老拆段算式还回去 —— 否则远距离回位会被误判。"""
        base = float(self.node.get_parameter("motion_timeout").value)
        budget = float(self.node.get_parameter("max_command_distance").value)
        expected = int(np.ceil(self._leg_distance(self.FAR_START) / budget - 1e-9))
        self.assertGreater(expected, 1, "测试前提: 老算式应当算出不止一段")
        self._home_from(self.FAR_START)
        self.assertEqual(self.node._home_time_multiplier, expected)
        self.assertAlmostEqual(self.node._active_motion_timeout(), base * expected)

        self.node._tick_motion()                      # 下发
        self._age("_waypoint_enter_time", base + 1.0)  # 老的一个 motion_timeout
        self.node._tick_motion()
        self.assertEqual(
            self.node._state, self.node._STATE_HOME,
            "只过了老代码一段的时间, 远距离回位不该判超时",
        )
        self._age("_waypoint_enter_time", base * (expected - 1) + 1.0)
        self.node._tick_motion()                       # 合计超过 倍数 × motion_timeout
        self.assertEqual(
            self.node._state, self.node._STATE_IDLE,
            "超了还债后的总预算就该判失败 —— 倍数不是「永不超时」的借口",
        )
        self.assertTrue(self.log.has("运动超时"))

    def test_home_timeout_multiplier_is_clamped(self):
        """超时倍数额不放大到无界: max_command_distance 调小也不会变成等一晚上。"""
        base = float(self.node.get_parameter("motion_timeout").value)
        self._home_from(self.FAR_START, max_command_distance=0.05)
        self.assertEqual(self.node._home_time_multiplier, 4, "倍数截断在上限 4")
        self.assertAlmostEqual(self.node._active_motion_timeout(), base * 4.0)
        self.assertTrue(self.log.has("按上限 4 计"), "截断了要说一声")

    def test_home_multiplier_does_not_leak_out_of_home(self):
        """倍数与盲走标记一样只在 HOME 期间有效, 否则后续阶段超时会莫名其妙。"""
        self._home_from(self.FAR_START)
        self.assertGreater(self.node._home_time_multiplier, 1)
        self.node._transition(self.node._STATE_PRE_GRASP, [np.zeros(3)])
        self.assertEqual(self.node._home_time_multiplier, 1)
        self.assertAlmostEqual(
            self.node._active_motion_timeout(),
            float(self.node.get_parameter("motion_timeout").value),
        )

    def test_segmented_helpers_are_gone(self):
        """删干净: 回位/放置都不再插中间点, 那两个 builder 一个都不该留。"""
        for name in ("_build_home_waypoints", "_build_segmented_waypoints"):
            with self.subTest(helper=name):
                self.assertFalse(hasattr(self.node, name), f"{name} 应已删除")


class ConfigConsistencyTest(unittest.TestCase):
    """新增的两个键: 节点默认值必须与 config/bottle_grasp_executor.yaml 一致。

    不一致的话, `ros2 run` 直接起节点与 `ros2 launch` 起节点的行为会不同 ——
    而这两个键管的正是「出错时机械臂去哪」, 不能有两套答案。
    """

    NEW_KEYS = ("return_to_standby_on_error", "blind_home_dwell")

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
        for key in self.NEW_KEYS:
            with self.subTest(param=key):
                self.assertIn(key, self.yaml_params, f"{key} 没写进 yaml")
                default = self.node.get_parameter(key).value
                actual = self.yaml_params[key]
                self.assertIs(type(actual), type(default), f"{key} 类型不一致")
                self.assertEqual(actual, default, f"{key} 两边取值不一致")

    def test_yaml_enables_the_new_behaviour(self):
        self.assertTrue(self.yaml_params["return_to_standby_on_error"])
        self.assertGreater(self.yaml_params["blind_home_dwell"], 0.0)


if __name__ == "__main__":
    unittest.main()
