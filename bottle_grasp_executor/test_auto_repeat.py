#!/usr/bin/env python3
"""bottle_grasp_executor 循环模式 (auto-repeat) 的回归测试。

2026-09-21 起 ~/execute_grasp 从「一次任务一次触发」改成**开关**: 调一次 = 启动任务
并开启循环, 再调一次 = 关闭循环。这个文件钉住四件最容易改坏的事:

  1. **开关语义**: 第二次调用只关循环, **不打断正在跑的那一次** —— 机械臂手里可能
     夹着瓶子、可能正在下降, 中途拧断有掉瓶/撞压风险; 空闲时再调一次又能重新启动。
  2. **回到待机位是「等」, 不是「做完」**: 跑完一轮、或跟丢目标回到待机位之后,
     循环在待机位一直等相机的新坐标 (机械臂不动、一条指令都不发、等多久都不收工),
     来了新目标才自动接下一轮。(2026-09-21 用户要求: 「回到工作位置后继续接受物体
     识别到的坐标, 然后抓取」。)
  3. **只认真新看到的候选**: 判据是**观察时刻** (_loop_guard_at) 而不是「有没有候选」
     —— 上一轮那个瓶子已经被放到相机视野之外, 它的坐标还留在 _latest_candidate 里
     的话就**对着空台面下降**了。回来之后收到的候选只有时间戳晚于守卫时刻的才算数,
     旧的顺手清掉。
  4. **只有真失败才收工**: 固件拒收 / 无反馈 / 夹爪失败 / 回位判不到位 → _fail 关循环;
     「视野里没有目标」(等待目标超时、TRACK_TARGET_LOST) 走 _fail(..., keep_loop=True),
     循环保持开启, 回待机位接着等。

不连机械臂、不连 CAN: 反馈直接写内部缓存, 候选直接喂 _select_and_store。

跑法 (在包根目录; HOME 要指到工作区, 否则 hand_eye_yaml_path 里那个 ~ 找不到标定):
  source /opt/ros/humble/setup.bash && source install/setup.bash
  HOME=/home/zdl/H3001_grasp /usr/bin/python3 -m unittest -v test_auto_repeat
"""

import os
import time
import unittest
from types import SimpleNamespace

import numpy as np
import rclpy
from std_srvs.srv import Trigger

from bottle_grasp_executor.bottle_grasp_executor_node import BottleGraspExecutor

PKG_ROOT = os.path.dirname(os.path.abspath(__file__))

# 测试里的几何 (与 test_track_wait.py 同一套): 抓取中心 0.30 m 高, 在**节点默认**
# 工作空间 (±0.60 / z ∈ [-0.10, 0.60]) 内 —— 用例跑的是节点默认值, 不是现场 yaml。
GRASP_CENTER = np.array([0.20, 0.0, 0.30])

# 放置位姿: 上报末端原点系 [mm, 度], 与 test_place.py 的取值同一量级 —— 只要加载期
# 那两条校验 (离基座原点 >5cm、换算前后都在 workspace_* 内) 过得去就行。位置和姿态的
# 具体数值与本文件无关 (这里不验放置, 只验「放置可用」这道硬闸)。
PLACE_REPORTED_MM_DEG = (400.0, -300.0, 200.0, 30.0, -20.0, 45.0)

# 30s 那条「等待目标超时」的判据是 _tick_wait_target 里的硬编码常量 (不是参数),
# 所以这里也只能写死 —— 抄的是节点里那个 30.0。
WAIT_TARGET_TIMEOUT = 30.0


def _arm_status_msg(motion_status=0):
    """够用的 H3001ArmStatus 替身 (_feedback_problem 与 _ik_rejection_problem 会看这些字段)。

    IK 那 4 个字段留默认 0: ik_status_valid=0 在 _ik_rejection_problem 里一律不判,
    等于「本次没收到拒收判决」—— 本文件不测 IK 拒收 (那是 test_place 的 IkRejectionTest)。
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

    本文件**多一层要小心的**: 循环模式自己就会发指令 (接下一轮), 用例数量又少, 所以
    这里不是「防一手」而是「唯一一道闸」。
    """

    def __init__(self):
        self.published = []

    def publish(self, message):
        self.published.append(message)


class _LoopTestCase(unittest.TestCase):
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
        # 单位手眼矩阵: 相机系 == base_link 系, 候选坐标可以直接写基座值。
        self.node._T_base_camera = np.eye(4)
        # 夹爪偏移只作用在「上报系 <-> 夹爪系」这个边界上, 与状态机逻辑无关,
        # 置零后断言里的坐标就是同一套数。
        self.node._gripper_offset = np.zeros(3)
        # dry_run 的代码默认是 True, 而循环/回位这些判据都在真机路径上。
        self._set(dry_run=False)
        # _tick 开头会按 candidates_source=file 去轮询 candidates_json_path, 而**本机
        # 真的存在那个文件** (demo 写出来的) —— 它会凭空塞一个候选进来, 把「循环不拿
        # 上一轮候选」这类用例直接毁掉。置一个非 None 的哨兵让 _tick 跳过轮询, 与节点在
        # candidates_source=topic 时的行为一致; 本文件的候选一律由 _candidate() 直接喂。
        self.node._candidates_sub = object()
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

    def _age(self, attribute, seconds):
        """模拟时间流逝: 不做真实 sleep, 把计时起点往前挪。"""
        since = getattr(self.node, attribute)
        if since is not None:
            setattr(self.node, attribute, since - seconds)

    def _candidate(self, position, track_id=1, quality=0.9, confidence=0.9):
        """喂一个候选 (走 _select_and_store, 与真实订阅路径同一入口)。"""
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

    def _enable_place(self):
        """打开放置并重跑一遍加载期校验 (参数是构造时读的, 测试里要手动重来)。

        循环模式的硬闸就是这一条 (place_enabled) —— 所以每个要武装循环的用例都得先
        过它。
        """
        self._set(
            place_enabled=True,
            place_pose_reported_mm_deg=[float(v) for v in PLACE_REPORTED_MM_DEG],
        )
        self.node._load_place_pose()
        self.assertTrue(self.node._place_ready, "测试前提: 放置位姿应当校验通过")

    def _enable_return(self):
        """让「任务结束回待机位」这条路径可用 (真机 yaml 里本来就是 true)。"""
        self._set(return_to_standby_on_finish=True)
        self.node._return_pending = True
        self.assertTrue(self.node._standby_ready, "测试前提: 待机位应当校验通过")

    def _armable(self):
        """把「能武装循环」的前提摆好: 放置可用 (硬闸) + 回位可用。"""
        self._enable_place()
        self._enable_return()

    def _execute(self):
        """调一次 ~/execute_grasp, 返回响应 (服务句柄由 rclpy 提供, 这里直接造)。"""
        return self.node._on_execute(Trigger.Request(), Trigger.Response())

    @property
    def standby(self):
        return np.asarray(self.node._standby_position, dtype=float)

    def _finish_task_successfully(self):
        """把节点摆到「这一次任务正常结束」的那条出口上 (_finish_task)。

        真实链路里这个出口是 LIFT (没配放置) 或 PLACE_WAIT (配了放置) 调用的; 这里把
        状态直接摆在 PRE_GRASP 是为了**不把本文件变成放置流程的测试** —— 被测的是
        出口本身 (_finish_task -> HOME -> IDLE) 与循环怎么接上它, 那一段与从哪个状态
        过来无关 (白名单里 PRE_GRASP 本来就在)。
        """
        position = np.asarray((0.30, 0.10, 0.12), dtype=float)
        self._feedback(position)
        self.node._return_pending = True
        self.node._transition(self.node._STATE_PRE_GRASP, [position])
        self.node._finish_task("测试: 抓取任务完成")
        self.assertEqual(self.node._state, self.node._STATE_HOME, "测试前提: 进回位了")

    def _drive_home_to_idle(self):
        """把 HOME 走完: 下发绝对待机位 -> 进容差 -> 停稳 -> 回 IDLE。

        时间靠回拨 _tolerance_start, 不真等 standby_settle_time。
        """
        self.assertFalse(
            self.node._home_blind, "测试前提: 这一趟是判到位的正常回位, 不是盲走"
        )
        self.node._tick_motion()                 # HOME 第一拍: 下发整条绝对待机位
        self.assertEqual(len(self.sent), 1, "测试前提: 回位指令已下发")
        self._feedback(self.standby)
        self.node._tick_motion()                 # 进容差, 开始计时
        self._age(
            "_tolerance_start",
            float(self.node.get_parameter("standby_settle_time").value) + 0.01,
        )
        self.node._tick_motion()                 # 停稳够了 -> 回待机位完成
        self.assertEqual(self.node._state, self.node._STATE_IDLE)

    def _complete_one_cycle(self):
        """跑完一整轮 (正常结束 + 回位到位), 回到 IDLE。"""
        self._finish_task_successfully()
        self._drive_home_to_idle()

    def _wait_tick(self):
        """在 IDLE 里 tick 一次: 循环进入「在待机位等目标」, 并记下守卫时刻。

        起手与等待的边界就在这里 —— 第一拍只记 _loop_guard_at (自己不许在这一拍起手,
        否则「回位到位」和「发下一条指令」就挤在同一拍上了)。
        """
        self.assertEqual(self.node._state, self.node._STATE_IDLE,
                         "测试前提: 应该已经回到待机位")
        self.node._tick()
        self.assertEqual(self.node._state, self.node._STATE_IDLE,
                         "没有新目标时不许起手")
        self.assertIsNotNone(self.node._loop_guard_at, "这一拍该进「等目标」了")

    def _new_target(self, position=None, track_id=2):
        """回来**之后**相机新看到的瓶子 (真实感知 10Hz 连续发布, 这里喂一帧就够)。"""
        self._candidate(
            GRASP_CENTER if position is None else position, track_id=track_id
        )


# ── 1. 开关语义 ──────────────────────────────────────────────


class LoopToggleTest(_LoopTestCase):
    """~/execute_grasp = 循环的开关。"""

    def test_first_call_starts_task_and_arms_loop(self):
        self._armable()
        response = self._execute()
        self.assertTrue(response.success, response.message)
        self.assertNotEqual(self.node._state, self.node._STATE_IDLE, "任务该启动了")
        self.assertTrue(self.node._loop_armed, "调一次就该开启循环")
        self.assertIn("循环模式已开启", response.message)

    def test_second_call_while_running_only_closes_the_loop(self):
        """正在跑任务时再调一次: 关循环, 但**不打断**机械臂。

        这是「关闭」的语义 (用户 2026-09-21 明确要求): 那一刻手里可能夹着瓶子、可能
        正在下降, 拧断有掉瓶/撞压风险 —— 关闭只在回到待机位准备下一次时生效。
        """
        self._armable()
        self._execute()
        state_before = self.node._state
        response = self._execute()
        self.assertTrue(response.success, "关闭是正常操作, 不是拒绝")
        self.assertFalse(self.node._loop_armed)
        self.assertEqual(self.node._state, state_before, "关闭循环不许打断正在跑的任务")
        self.assertIn("照常跑完", response.message)

    def test_second_call_while_idle_closes_the_loop(self):
        self._armable()
        self._execute()
        self.node._transition(self.node._STATE_IDLE)   # 手工摆到空闲
        response = self._execute()
        self.assertTrue(response.success)
        self.assertFalse(self.node._loop_armed)
        self.assertIn("当前空闲", response.message)

    def test_call_after_closing_starts_a_new_task_and_arms_again(self):
        """关掉之后再调一次 = 重新启动 + 重新武装 (不用重启节点)。"""
        self._armable()
        self._execute()
        self._execute()                               # 关
        self.node._transition(self.node._STATE_IDLE)
        response = self._execute()                    # 再开
        self.assertTrue(response.success, response.message)
        self.assertTrue(self.node._loop_armed)
        self.assertNotEqual(self.node._state, self.node._STATE_IDLE)

    def test_task_running_without_loop_is_still_rejected(self):
        """循环已经关掉、任务还在跑: 再调一次要按老规矩被拒 (不会插队)。"""
        self._armable()
        self._execute()
        self._execute()                               # 关循环, 任务继续跑
        response = self._execute()                    # 第三次: 想插一个新任务
        self.assertFalse(response.success, "任务进行中不该接受新任务")
        self.assertIn("拒绝新请求", response.message)


# ── 2. 循环接下一轮 ───────────────────────────────────────────


class LoopContinuesTest(_LoopTestCase):
    """跑完一轮 -> 回位 -> 在待机位等目标 -> 来了新目标才自动接下一轮。"""

    def test_next_cycle_starts_after_a_new_target_arrives(self):
        self._armable()
        self._execute()
        self._complete_one_cycle()
        self.assertTrue(self.node._loop_armed, "正常跑完不该关循环")
        self._wait_tick()                              # 先在待机位等着
        self._new_target()
        self.node._tick()                              # 来了新目标 -> 起手
        self.assertNotEqual(
            self.node._state, self.node._STATE_IDLE, "等到新目标后该自动开始下一轮"
        )
        self.assertTrue(self.node._loop_armed)

    def test_standby_wait_never_gives_up(self):
        """视野里一直没有目标: 一直等, 不自动收工, 机械臂不动。

        这是用户 2026-09-21 明确要的语义 (「回去之后继续接受相机识别到的坐标」): 下一个
        瓶子还没摆上来是正常工况, 不是故障。连 tick 200 拍 (10s, 真实时间) 都不许离开
        IDLE、不许发指令、不许关循环 —— 等目标这条路上**没有任何超时** (唯一那条 30s
        超时在 WAIT_TARGET 里, 只在「已经起手了却还是没目标」时才走)。
        """
        self._armable()
        self._execute()
        self._complete_one_cycle()
        self.sent.clear()
        self.node._tick()                              # 进「等目标」
        for _ in range(200):
            self.node._tick()
        self.assertEqual(
            self.node._state, self.node._STATE_IDLE, "空场景不许自动起手, 也不许收工"
        )
        self.assertTrue(self.node._loop_armed, "空场景不许自动关循环")
        self.assertEqual(self.sent, [], "等的时候一条指令都不许发")

    def test_home_arrival_tick_only_returns_to_idle(self):
        """回位到位那一拍只到 IDLE; 循环在**下一拍**才开始等目标。

        为什么在乎: HOME 的到位判定在 _tick_motion 里做, 而循环那一段在 _tick 的
        handler 派发之前 —— 顺序错了就会在同一拍抢着起步, 等于机械臂还没站稳就发下一条
        指令 (以前那版是「同拍就起手」, 现在是「同拍只到 IDLE」)。
        """
        self._armable()
        self._execute()
        self._finish_task_successfully()
        self.node._tick()                              # 下发绝对待机位
        self._feedback(self.standby)
        self.node._tick()                              # 进容差, 开始计时
        self._age(
            "_tolerance_start",
            float(self.node.get_parameter("standby_settle_time").value) + 0.01,
        )
        self.node._tick()                              # 停稳够了 -> 回位完成, 同拍只到 IDLE
        self.assertEqual(self.node._state, self.node._STATE_IDLE, "这一拍只该到 IDLE")
        self.assertIsNone(self.node._loop_guard_at, "还没进「等目标」")
        self.node._tick()                              # 下一拍: 进「等目标」
        self.assertEqual(self.node._state, self.node._STATE_IDLE)
        self.assertIsNotNone(self.node._loop_guard_at)

    def test_loop_start_rejected_disarms_instead_of_retrying(self):
        """循环起手时前提不成立 (放置坐标被改坏): 关循环、不启动、不原地重试。"""
        self._armable()
        self._execute()
        self._complete_one_cycle()
        self._wait_tick()
        self.node._place_ready = False                 # 假装加载期校验后又改坏了参数
        self._new_target()                             # 有目标 -> 走到 _begin_task 那一步
        self.node._tick()
        self.assertFalse(self.node._loop_armed)
        self.assertEqual(self.node._state, self.node._STATE_IDLE, "拒绝之后别乱动")


# ── 3. 失败就停 ─────────────────────────────────────────────


class LoopStopsOnFailureTest(_LoopTestCase):
    """真失败一次就停; 「视野里没有目标」不算真失败。"""

    def test_fail_closes_the_loop(self):
        self._armable()
        self._execute()
        self.assertTrue(self.node._loop_armed)
        self.node._fail("测试: 故意失败")
        self.assertFalse(self.node._loop_armed, "一次失败就该收工")

    def test_no_next_cycle_after_a_failed_one(self):
        self._armable()
        self._execute()
        # 先喂一帧反馈: 没有反馈时 _enter_home 走的是「盲走」那条 (按 dwell 收工, 见
        # LoopStopsOnFailureTest.test_abandoned_blind_home_closes_the_loop), 而这里要
        # 测的是「失败 -> 正常回位 -> IDLE -> 循环已经关了」这条普通链路。
        self._feedback((0.30, 0.10, 0.12))
        self.node._fail("测试: 故意失败")
        self._drive_home_to_idle()                     # 失败也会回待机位
        self._new_target()                             # 就算马上来了新目标也不许再起手
        for _ in range(5):
            self.node._tick()
        self.assertEqual(
            self.node._state, self.node._STATE_IDLE, "循环已经关了, 不该再自动启动"
        )

    # ── 「没有目标」不算失败 ──────────────────────────────────

    def test_no_target_for_30s_returns_home_but_keeps_the_loop(self):
        """起手后 30s 没有有效候选: 回待机位接着等, 循环**不关**。

        这是本次改动的核心 (用户 2026-09-21: 「回到工作位置后继续接受相机识别到的
        坐标」)。30s 那条判据本身保留 —— 它现在只负责「别在 WAIT_TARGET 里干等」,
        回待机位之后与它无关。
        """
        self._armable()
        self._execute()
        self.assertEqual(self.node._state, self.node._STATE_WAIT_TARGET)
        self.assertIsNone(self.node._latest_candidate, "测试前提: 起手时手里没有候选")

        self._feedback((0.30, 0.10, 0.12))             # 让回位走「判到位」那条
        self._age("_state_enter_time", WAIT_TARGET_TIMEOUT + 1.0)
        self.node._tick()
        self.assertTrue(self.node._loop_armed, "没等到目标不算真失败, 循环要保持开启")
        self.assertEqual(self.node._state, self.node._STATE_HOME, "回待机位接着等")

        # 回到待机位之后等到新目标 -> 照常抓 (端到端把用户那句话走一遍)
        self._drive_home_to_idle()
        self._wait_tick()
        self._new_target()
        self.node._tick()
        self.assertNotEqual(
            self.node._state, self.node._STATE_IDLE, "等到了就该接着抓"
        )

    def test_track_target_lost_keeps_the_loop(self):
        """悬停跟随时跟丢 (感知停了, 候选超过 max_result_age): 回待机位接着等。"""
        self._armable()
        self._candidate(GRASP_CENTER)
        self._execute()
        self._feedback((0.30, 0.10, 0.12))
        self.node._transition(self.node._STATE_TRACK_WAIT)   # 假装已经悬停跟随
        self.node._latest_candidate["received_at"] -= (
            float(self.node.get_parameter("max_result_age").value) + 1.0
        )

        self.node._follow_target(time.monotonic())
        self.assertTrue(self.node._loop_armed, "跟丢目标不算真失败, 循环要保持开启")
        self.assertEqual(self.node._state, self.node._STATE_HOME, "回待机位接着等")

    def test_track_target_never_seen_keeps_the_loop(self):
        """另一条跟丢: 悬停跟随期间一个候选都没收到过 (_target_age() is None)。"""
        self._armable()
        self._execute()
        self._feedback((0.30, 0.10, 0.12))
        self.node._transition(self.node._STATE_TRACK_WAIT)
        self.assertIsNone(self.node._latest_candidate, "测试前提: 从来没收过候选")

        self.node._follow_target(time.monotonic())
        self.assertTrue(self.node._loop_armed, "没收到过候选同样不算真失败")
        self.assertEqual(self.node._state, self.node._STATE_HOME)

    def test_abandoned_blind_home_closes_the_loop(self):
        """回位判不到位的异常出口也要关循环 (位置本来就不确定, 不能接着抓)。"""
        self._armable()
        self._execute()
        self.node._fail("测试: 机器人不可控")
        self.assertTrue(self.node._home_blind, "测试前提: 这一趟是盲走回位")
        self.node._tick_motion()                       # 盲走那一条指令也发出去
        self._age("_blind_started",
                  float(self.node.get_parameter("blind_home_dwell").value) + 0.01)
        self.node._tick_motion()
        self.assertEqual(self.node._state, self.node._STATE_IDLE)
        self.assertFalse(self.node._loop_armed)


# ── 4. 不许拿上一轮的候选 ────────────────────────────────────


class LoopCandidateFreshnessTest(_LoopTestCase):
    """回来之后只认**新观察到的**候选 (时间戳守卫) —— 旧候选一律丢掉, 否则会对着空台面下降。"""

    def test_previous_candidate_is_dropped_at_standby(self):
        self._armable()
        self._candidate(GRASP_CENTER)                  # 第 1 轮那个瓶子 (马上要被搬走)
        self._execute()
        self._complete_one_cycle()
        self.assertIsNotNone(
            self.node._latest_candidate, "测试前提: 上一轮那个候选还留在手里"
        )
        self.node._tick()                              # 进「等目标」: 旧候选不算数
        self.assertIsNone(self.node._latest_candidate, "旧候选必须丢掉")

    def test_cycle_two_never_grabs_the_old_target(self):
        """没有新目标时, 第 2 轮一步都不许动 —— 这是循环模式最危险的失效方式。

        旧坐标还在手里、稳定窗口一过就去抓空位 (机械臂对着空台面下降)。连 tick 20 次
        (1s 稳定窗口绰绰有余) 都不许离开 IDLE、不许锁、更不许发指令。
        """
        self._armable()
        self._candidate(GRASP_CENTER, track_id=1)
        self._execute()
        self._complete_one_cycle()
        self.sent.clear()

        for _ in range(20):
            self.node._tick()
        self.assertEqual(
            self.node._state, self.node._STATE_IDLE, "不许拿旧坐标去抓"
        )
        self.assertIsNone(self.node._locked_track_id, "连锁都不该锁")
        self.assertEqual(self.sent, [], "一条指令都不该发")

    def test_new_candidate_after_return_home_is_accepted(self):
        """守卫不是把循环卡死: 回来之后新收到的目标照常抓 (端到端)。

        这里刻意让**新目标**和上一轮那个在同一位置附近但 track_id 不同、坐标差 5cm,
        断言最终抓的是新坐标 —— 「回到工作位置后继续接受相机识别到的坐标」这句话的
        最小可验证形式。
        """
        self._armable()
        self._candidate(GRASP_CENTER, track_id=1)
        self._execute()
        self._complete_one_cycle()
        self._wait_tick()
        self.assertIsNone(self.node._latest_candidate, "旧候选已经丢掉了")

        self._new_target(GRASP_CENTER + np.array([0.0, 0.05, 0.0]), track_id=2)
        # _tick 每拍把当前候选记一个样本: 连 tick 够 min_stable_samples 帧、且都在
        # stable_duration 窗口内 (与真机 10Hz 连续发布同一形状)。
        for _ in range(10):
            self.node._tick()
        self.assertEqual(self.node._locked_track_id, 2, "新目标该被锁上")
        self.assertNotEqual(
            self.node._state, self.node._STATE_WAIT_TARGET, "稳定之后就该往下走"
        )
        np.testing.assert_allclose(
            self.node._grasp_center_base, GRASP_CENTER + np.array([0.0, 0.05, 0.0]),
            atol=1e-9,
            err_msg="抓的是新目标, 不是上一轮那个位置",
        )


# ── 5. 开启条件 ─────────────────────────────────────────────


class LoopArmingGatesTest(_LoopTestCase):
    """硬闸一条 (place_enabled), 其余只警告不拦。"""

    def test_place_disabled_runs_one_task_without_arming(self):
        """place_enabled=false: 任务照常跑, 但循环不开。

        理由写进 message 和日志: 关掉放置时每一轮只是把瓶子在原地放下再抓起来 ——
        同一个瓶子无限循环, 而循环模式成立的前提是「放下去的东西离开了相机视野」。
        """
        self._enable_return()
        response = self._execute()
        self.assertTrue(response.success, "单发抓取本身不该被拒")
        self.assertFalse(self.node._loop_armed, "关掉放置就不能循环")
        self.assertNotEqual(self.node._state, self.node._STATE_IDLE, "任务照常启动")
        self.assertIn("place_enabled=false", response.message)

    def test_place_enabled_but_invalid_rejects_the_task(self):
        """place_enabled=true 但坐标无效: 连任务都不启动 (原有行为一字不变)。"""
        self._set(place_enabled=True)
        self.assertFalse(self.node._place_ready, "测试前提: 从来没加载过放置位姿")
        response = self._execute()
        self.assertFalse(response.success)
        self.assertFalse(self.node._loop_armed)
        self.assertEqual(self.node._state, self.node._STATE_IDLE)

    def test_confirmations_required_warn_but_still_arm(self):
        """开确认只警告不拦: 循环开得起来, 只是每一轮都要人按 Enter。"""
        self._armable()
        self._set(grasp_confirm_required=True, place_confirm_required=True)
        response = self._execute()
        self.assertTrue(response.success, response.message)
        self.assertTrue(self.node._loop_armed, "确认点开着不该拦住循环的开启")

    def test_return_to_standby_disabled_warns_but_still_arms(self):
        """return_to_standby_on_finish=false: 循环不在待机位起步, 但仍然开。"""
        self._enable_place()
        self._set(return_to_standby_on_finish=False)
        response = self._execute()
        self.assertTrue(response.success, response.message)
        self.assertTrue(self.node._loop_armed)


if __name__ == "__main__":
    unittest.main()
