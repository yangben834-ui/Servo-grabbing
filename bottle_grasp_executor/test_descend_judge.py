#!/usr/bin/env python3
"""bottle_grasp_executor 下降 (DESCEND) 的回归测试。

2026-09-17 起 DESCEND 只有**一条指令**: 从悬停点直接下降到抓取点 (Z 下降
pre_grasp_offset_z - grasp_offset_z, 当前 9cm), 发出后等 descend_dwell 秒就进
GRASP_WAIT。**不判是否到达指定位置、不迭代末端修正、不分段** —— 落点准不准由
GRASP_WAIT 的人工确认兜底 (提示里会写「下降不判到位, 请确认夹爪确实到了抓取点」)。

这里断言的就是这套语义, 尤其是它**故意不做**的那几件事:
  - 机械臂纹丝不动 (被挡住 / IK 无解 / 重力下垂够不到) 也照走, 不再报失败;
  - 不再有「停稳即到位」「限速门」「末段迭代修正」这些分段下降的判据;
  - 全程只有一条 move_p 指令 —— 重发就是抖动。

不连机械臂、不连 CAN: 节点自己只收发 ROS 话题, 反馈直接写进内部缓存, 所以可以
整段驱动 _tick_descend, 观察「发了几条指令、什么时候进 GRASP_WAIT」。

跑法 (在包根目录):
  source /opt/ros/humble/setup.bash && source install/setup.bash
  /usr/bin/python3 -m unittest -v test_descend_judge
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

# 本次改动新增 / 改写的参数: 节点默认值必须与 config/*.yaml 一致, 否则
# ros2 run 直接起节点与 ros2 launch 起节点的行为会不一样 (见 test_new_params_match_yaml)。
NEW_DESCEND_KEYS = (
    "descend_dwell",
)

# 分段下降删掉后, 这些键不该再出现在 yaml 里 —— 留着会让人以为还能调回去。
RETIRED_DESCEND_KEYS = (
    "descend_single_step",
    "descend_single_dwell",
    "descend_step_z",
    "descend_step_tolerance",
    "descend_settle_time",
    "descend_settle_judge",
    "descend_stable_time",
    "descend_stable_epsilon",
    "descend_min_travel",
    "descend_min_command_interval",
    "descend_correction_final_only",
    "descend_max_correction",
)


def _arm_status_msg(motion_status=0):
    """够用的 H3001ArmStatus 替身 (_feedback_problem 与 _ik_rejection_problem 会看这些字段)。

    固件 IK 判决那 4 个字段留默认: ik_status_valid=0 在 _ik_rejection_problem 里
    **一律不判**, 所以这里不填就等于「本次没收到拒收判决」—— 下降这段「不看反馈判
    到位」因此不会被新通道搅动 (它有自己那套到达判据)。
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


class _ExecutorTestCase(unittest.TestCase):
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
        # 夹爪偏移只作用在「上报系 <-> 夹爪系」这一个边界上, 内部循环 (路点、到位判定、
        # 残差) 与它无关, 所以测试里置零 —— 参数默认值本来就是零 (yaml 才填 -0.13)。
        self.node._gripper_offset = np.zeros(3)
        # 记录真正下发的指令: 「抖动」= 同一条指令被反复重发, 靠这个计数断言。
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

    def _feedback(self, position, motion_status=0):
        """写一帧「刚收到」的 TCP 位姿 + 臂状态 (时间戳取当前 ROS 时钟)。"""
        now = self.node.get_clock().now().nanoseconds * 1e-9
        self.node._tcp_pose = (now, np.asarray(position, dtype=float))
        self.node._tcp_orientation = (0.0, 0.0, 0.0, 1.0)
        self.node._arm_status = (now, _arm_status_msg(motion_status))

    def _break_feedback(self):
        """反馈断掉 (话题不再更新): 时间戳往前挪过 input_timeout。"""
        timeout = float(self.node.get_parameter("input_timeout").value)
        stamp, pose = self.node._tcp_pose
        self.node._tcp_pose = (stamp - timeout - 1.0, pose)

    def _enter_descend(self, pre_z=0.38, final_z=0.30):
        """把节点摆到 DESCEND: 悬停点 pre_z -> 抓取点 final_z, 尚未下发。"""
        self.node._pre_grasp_target = np.array([0.20, 0.0, pre_z])
        self.node._final_grasp_target = np.array([0.20, 0.0, final_z])
        self.node._goal_orientation = (0.0, 0.0, 0.0, 1.0)
        self.node._transition(self.node._STATE_DESCEND)
        return self.node._final_grasp_target.copy()

    def _age(self, attribute, seconds):
        """模拟时间流逝: 不做真实 sleep, 把计时起点往前挪。

        被测的是判断逻辑本身, 不是 time.monotonic()。
        起点是 None 表示「当前没有在计时」, 此时无可提前。
        """
        since = getattr(self.node, attribute)
        if since is not None:
            setattr(self.node, attribute, since - seconds)

    def _age_descend(self, seconds=1.0):
        self._age("_descend_sent_time", seconds)

    def _age_stall(self, seconds=2.0):
        """把迭代末端修正的停稳计时往前挪 (用来证明下降根本不会走到那一步)。"""
        self._age("_settle_since", seconds)


class DescendSingleCommandTest(_ExecutorTestCase):
    """一条指令下降 9cm: 不判到位、不迭代修正、不分段。"""

    def setUp(self):
        super().setUp()
        self._set(dry_run=False)
        self.target = self._enter_descend()          # pre_z=0.38 -> final_z=0.30

    def test_one_command_for_the_whole_descent(self):
        self._feedback([0.20, 0.0, 0.38])
        self.node._tick_descend()
        self.assertEqual(len(self.sent), 1, "整段下降只发一条指令")
        np.testing.assert_allclose(self.sent[0], np.array([0.20, 0.0, 0.30]))
        # 进 DESCEND 时没有装载任何路点队列 —— 分段下降那套已经删掉了
        self.assertEqual(self.node._waypoints, [])

    def test_command_goes_to_the_grasp_point_not_a_step_short_of_it(self):
        """指令目标是抓取点本身, 不是「往下走一步」—— 距离由 Z 偏移之差决定。"""
        self._feedback([0.20, 0.0, 0.38])
        self.node._tick_descend()
        np.testing.assert_allclose(self.sent[0], self.target)
        self.assertAlmostEqual(float(self.sent[0][2] - 0.38), -0.08, places=9)

    def test_dwell_gates_the_advance(self):
        dwell = float(self.node.get_parameter("descend_dwell").value)
        self._feedback([0.20, 0.0, 0.38])
        self.node._tick_descend()                    # 下发
        self.node._tick_descend()                    # 同一时刻再 tick 一次
        self.assertEqual(self.node._state, self.node._STATE_DESCEND, "没等满 dwell 不该推进")
        self.assertEqual(len(self.sent), 1)
        self._age_descend(dwell + 0.01)
        self.node._tick_descend()
        self.assertEqual(self.node._state, self.node._STATE_GRASP_WAIT)

    def test_feedback_never_gates_and_never_corrects(self):
        """刻意不看反馈: 纹丝不动也照走, 且不产生修正量。

        「机械臂没动也放行」在分段下降里是要防的 (会把 IK 无解伪装成正常下降),
        现在是要的行为 —— 节点不再自称「已到位」, 改由 GRASP_WAIT 的人工确认兜底,
        所以这里断言的是「照走且不抖」。代价: 完全不动也不会报失败。
        """
        self._feedback([0.20, 0.0, 0.38])
        self.node._tick_descend()
        for _ in range(8):                           # 8s > dwell, 位置一直没变
            if self.node._state != self.node._STATE_DESCEND:
                break                                # 已经进 GRASP_WAIT, 停手
            self._feedback([0.20, 0.0, 0.38])
            self._age_descend(1.0)
            self._age_stall(2.0)                     # 迭代修正的停稳计时也往前挪
            self.node._tick_descend()
        self.assertEqual(self.node._state, self.node._STATE_GRASP_WAIT)
        self.assertEqual(len(self.sent), 1, "一条指令到底 —— 重发就是抖动")
        self.assertFalse(np.any(self.node._cmd_offset), "下降不做迭代修正")
        self.assertEqual(self.node._retry_count, 0)

    def test_sag_is_not_compensated(self):
        """重力下垂 (实测 3~55mm) 不再被迭代修掉 —— 这是单步的代价, 要写清楚。"""
        self._feedback([0.20, 0.0, 0.38])
        self.node._tick_descend()
        goal = self.node._goal_position.copy()
        self._feedback(goal - np.array([0.0, 0.0, 0.030]))   # 停在下垂 30mm 处
        self._age_descend(float(self.node.get_parameter("descend_dwell").value) + 0.01)
        self._age_stall(5.0)
        self.node._tick_descend()
        self.assertEqual(self.node._state, self.node._STATE_GRASP_WAIT)
        self.assertEqual(len(self.sent), 1)
        self.assertFalse(np.any(self.node._cmd_offset))

    def test_motion_timeout_does_not_apply(self):
        """下降不受 motion_timeout 管: 到位判定都删了, 超时判据也不该留下。"""
        self._feedback([0.20, 0.0, 0.38])
        self.node._tick_descend()
        self._age("_waypoint_enter_time", float(self.node.get_parameter("motion_timeout").value) + 5.0)
        self._age("_state_enter_time", float(self.node.get_parameter("motion_timeout").value) + 5.0)
        self.node._tick_descend()
        self.assertEqual(self.node._state, self.node._STATE_DESCEND, "不该因超时判失败")

    def test_tight_position_tolerance_does_not_matter(self):
        """position_tolerance 只管 PRE_GRASP / LIFT / HOME, 与下降无关。"""
        self._set(position_tolerance=1e-9, settle_time=0.0)
        self._feedback([0.20, 0.0, 0.38])
        self.node._tick_descend()
        self._age_descend(float(self.node.get_parameter("descend_dwell").value) + 0.01)
        self.node._tick_descend()
        self.assertEqual(self.node._state, self.node._STATE_GRASP_WAIT)
        self.assertEqual(len(self.sent), 1)

    def test_dry_run_sends_one_command_and_advances(self):
        self._set(dry_run=True)
        self.node._tick_descend()                    # 下发 (dry_run 无反馈也照发)
        self.assertEqual(len(self.sent), 1)
        self.assertEqual(self.node._state, self.node._STATE_DESCEND)
        self._age_descend(float(self.node.get_parameter("descend_dwell").value) + 0.01)
        self.node._tick_descend()
        self.assertEqual(self.node._state, self.node._STATE_GRASP_WAIT)

    def test_stale_feedback_fails_before_the_command(self):
        """唯一保留的前置检查: 反馈不可信时不下发, 直接判失败。"""
        self._feedback([0.20, 0.0, 0.38])
        self._break_feedback()
        self.node._tick_descend()
        self.assertEqual(len(self.sent), 0, "反馈超时不该下发")
        self.assertEqual(self.node._state, self.node._STATE_IDLE)

    def test_missing_target_fails(self):
        self._feedback([0.20, 0.0, 0.38])
        self.node._final_grasp_target = None
        self.node._tick_descend()
        self.assertEqual(len(self.sent), 0)
        self.assertEqual(self.node._state, self.node._STATE_IDLE)

    def test_distance_limit_still_applies(self):
        """落地前那条安全检查没有跟着分段一起删: 单段命令距离仍有上限。"""
        self._set(max_command_distance=0.01)         # 8cm 下降必然超
        self._feedback([0.20, 0.0, 0.38])
        self.node._tick_descend()
        self.assertEqual(len(self.sent), 0)
        self.assertEqual(self.node._state, self.node._STATE_IDLE)

    def test_segmented_machinery_is_gone(self):
        """删除要删干净: 分段下降的辅助函数一个都不该留 (留着就是死代码)。"""
        for name in (
            "_build_descend_waypoints",
            "_tick_descend_single_step",
            "_descend_plan_text",
            "_descend_command_gate_open",
            "_descend_step_travel",
            "_descend_step_stable",
            "_descend_settle_judge_applies",
            "_is_final_descend_waypoint",
            "_active_correction_cap",
        ):
            with self.subTest(helper=name):
                self.assertFalse(hasattr(self.node, name), f"{name} 应已删除")


class ConfigConsistencyTest(_ExecutorTestCase):
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
        for key in NEW_DESCEND_KEYS:
            with self.subTest(param=key):
                self.assertIn(key, self.yaml_params, f"{key} 没写进 yaml")
                default = self.node.get_parameter(key).value
                actual = self.yaml_params[key]
                self.assertIs(type(actual), type(default), f"{key} 类型不一致")
                self.assertEqual(actual, default, f"{key} 两边取值不一致")

    def test_segmented_descend_params_are_gone(self):
        """分段下降删干净了: yaml 里不留任何「还以为能调回去」的开关。"""
        for key in RETIRED_DESCEND_KEYS:
            with self.subTest(param=key):
                self.assertNotIn(key, self.yaml_params, f"{key} 应该已经删掉")

    def test_descend_distance_equals_the_hover_height(self):
        """下降距离 = pre_grasp_offset_z - grasp_offset_z **且** 落点就是感知抓取中心。

        这条钉住的是**关系**, 不是某个具体厘米数 —— 悬停多高就下降多少, 而落点始终是
        感知给的抓取中心 (grasp_offset_z 必须为 0, 否则就是在抓取点上再加一个偏置)。
        2026-09-21 之前这里写死 `assertAlmostEqual(distance, 0.09)`: 现场把悬停高度从
        9cm 调到 12cm 之后它就变成假红了 —— 9/12 都是标定值, 不是不变量。
        现场现值 = pre_grasp_offset_z (0.12) - 0 = 12cm; 上下界只是防手滑写错数量级。
        """
        distance = (
            self.yaml_params["pre_grasp_offset_z"] - self.yaml_params["grasp_offset_z"]
        )
        self.assertEqual(self.yaml_params["grasp_offset_z"], 0.0, "落点保持抓取中心")
        self.assertAlmostEqual(distance, self.yaml_params["pre_grasp_offset_z"], places=9)
        self.assertGreater(distance, 0.02, "下降行程不该小到看不见")
        self.assertLess(distance, 0.30, "下降行程不该大到要撞台面")

    def test_dwell_covers_the_blind_descent(self):
        """descend_dwell 是「给足走完的时间」, 不是到位判据 —— 必须包住整段下降。

        这条在**盲降**段是硬要求 (README_参数配置.md §7: `descend_dwell ≥ 抓取段下降的
        实际走完时间`): 下降全程不看反馈, 到没到全靠这个时间, 取小了会在机械臂还在
        动的时候就进 GRASP_WAIT 报「到了」。现场值 2.0s 对 12cm 的下降。
        这里用 settle_time 做数量级参照 (0.3s 的停稳时间是对「已经停下」的判定, 而
        12cm 的行走要长得多), 不钉具体秒数 —— 它是标定值, 会随速度/行程变。
        """
        dwell = self.yaml_params["descend_dwell"]
        self.assertGreater(
            dwell,
            self.yaml_params["settle_time"] * 5.0,
            "12cm 的走完时间远不止一个 settle_time; 取小了会在机械臂还在动时就放行",
        )


if __name__ == "__main__":
    unittest.main()
