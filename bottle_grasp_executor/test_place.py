#!/usr/bin/env python3
"""bottle_grasp_executor 放置 (PLACE_MOVE / PLACE_WAIT) 的回归测试。

2026-09-17 起抓住物体后不直接回待机位, 而是先把物体放到 place_pose_reported_mm_deg
指定的**完整位姿** (位置 + 姿态):

  LIFT -> PLACE_MOVE    从抬升点**直线直达**放置位姿本身, 判到位;
       -> PLACE_WAIT    到达后停 place_descend_dwell 秒放下物体 (= 第三次确认),
                        然后回待机位。

这里断言的就是这套语义, 尤其是几个**故意如此**的地方:
  - **没有中间点** (2026-09-18 改): 原来是「先平移到放置点正上方 (x,y 用放置点、z 用
    抬升后的实际高度), 再垂直盲降」两段, 那个中间点用户从来没要求过, 真机上 IK 拒收的
    恰恰是它。现在整段只有一个目标, 就是参数那 6 个数换算出来的位姿;
  - **判到位**: 这一段是自由空间运动, 终点有明确答案, 所以走 _tick_motion 的到位判据
    (与 PRE_GRASP / LIFT / HOME 同一套, 带迭代末端修正)。旧的盲降那一段不判 —— 物体
    压在台面上以后机械臂就下不去了, 判它只会持续往下压; 现在没有那一段了;
  - 放置姿态**整个来自参数** (2026-09-18 之前是「进 PLACE_MOVE 时冻结当前 TCP 姿态」),
    全程用同一个常量 —— 既不再被抓取姿态绑死, 也不会把腕部下垂读成新姿态;
  - 参数是**上报末端原点系**的 6 个数 [X,Y,Z(mm), RX,RY,RZ(度)], 与
    standby_pose_reported_mm_deg 同一约定; 换算在加载期做一次 (下面 _reported_to_
    gripper 是测试里**另外写一份**的定义, 不复用节点的函数 —— 共用一份就等于没测);
  - **整段恒为一条指令** (2026-09-20 去掉拆段): 放置点可能离抓取点很远, 但不再按
    max_command_distance 插中间点 —— 路径交给固件规划, 调用点对 PLACE_MOVE 豁免那道
    上限 (上限本身没松, 见 test_the_limit_is_waived_for_configured_goals_only);
  - place_enabled=false 时整段跳过, 老流程 (LIFT -> HOME) 一字不变。

不连机械臂、不连 CAN: 反馈直接写进内部缓存, 所以可以整段驱动状态机, 观察
「发了几条指令、什么时候换状态」。计时全部用「把计时起点往前挪」代替真实 sleep。

跑法 (在包根目录):
  source /opt/ros/humble/setup.bash && source install/setup.bash
  /usr/bin/python3 -m unittest -v test_place
"""

import math
import os
import time
import unittest
from types import SimpleNamespace

import numpy as np
import rclpy
import yaml

from bottle_grasp_executor.bottle_grasp_executor_node import BottleGraspExecutor

PKG_ROOT = os.path.dirname(os.path.abspath(__file__))

# 本次改动新增的参数: 节点默认值必须与 config/*.yaml 一致, 否则 ros2 run 直接起节点
# 与 ros2 launch 起节点的行为会不一样 (见 test_new_params_match_yaml)。
#
# place_pose_reported_mm_deg **刻意不在**这里: 它是现场实测填的位姿, 与节点里的占位
# 默认值必然不同 —— 与 standby_pose_reported_mm_deg 同一约定 (那条也不在一致性名单里)。
# 取而代之的是下面 test_enabling_place_requires_a_real_pose 那条不变量。
#
# place_enabled **也从名单里移出去了** (2026-09-17): 它是**部署开关**, 代码默认给
# false (保守), yaml 里一旦按文档改成 true, 这条「两边取值一致」就变成假红 ——
# 而那不是 bug, 是它本来就该两边不同 (同 orientation_source / return_to_standby_on_finish)。
# 它现在由下面 test_enabling_place_requires_a_real_pose 那条不变量守着:
# 「开着就必须有个真坐标」。同样的道理, 夹爪侧的 gripper_enabled 也不进任何一致性名单。
# 2026-09-21 起改叫 NEW_PARAM_KEYS: 名单里不再只有放置那一个键 (固件 IK 判决的两个
# 键也进来了), 原来的名字会让人以为「不是放置的键就不该往这儿加」。
NEW_PARAM_KEYS = (
    "place_descend_dwell",
    # 固件 IK 判决 (2026-09-21)。这两个键**必须**两边一致: 它们是「要不要快速失败」
    # 的开关与宽限, 一边 true 一边 false 会让 ros2 run 与 ros2 launch 起出来的节点
    # 在真机上对同一次 IK 拒收给出不同处置 —— 一个当场停手, 一个等满 30s 超时。
    "ik_verdict_enabled",
    "ik_verdict_grace",
    # 下降 / 抬升的直线通道 (2026-09-24)。两边不一致的后果很具体: 现场按 yaml 只改了
    # move_p_topic (例如指到一条测试话题上), 而 move_l 仍留在代码默认的
    # /control/move_l —— 于是 PRE_GRASP 老老实实打到测试话题, 一进 DESCEND 就打到
    # **真桥**上, 机械臂当场动起来。这正是 _NullPublisher 那类事故的另一种来路。
    "move_l_topic",
    # 关节角通道 (2026-09-28)。同一个来路: 它是第三条运动出口, 两边不一致时
    # 「哪一段打到哪条话题」就跟着变 —— 而这条话题**绕过固件的 IK 判决**, 打错了
    # 是机械臂直接按关节角走。
    "move_j_topic",
    # 求逆解那一路的开关与阈值。这几个键决定了「哪一段走 J」和「解到什么程度才敢发」,
    # 两边不一致的后果比上面两条更重: 一个在求逆解、一个在走固件 IK, 日志会完全对不上。
    "ik_min_margin_deg",
    "ik_max_joint_jump_deg",
    # 抬升时顺带回正姿态 (2026-09-30)。两边不一致的后果: 一边在抬升里回正、一边不回,
    # 于是放置段一边是 ~54° 转腕、一边是 144° —— 而后者正是现场那次大摆 (IK 解不出、
    # 回落 move_p、err_status=512)。阈值还与 _enter_place 那条 >90° 的 warn 同源。
    "lift_align_threshold_deg",
    "lift_align_segments",
)

# 放置状态里失败也必须回位 (手里/台面上有东西时更不该把机械臂丢在原地)。
PLACE_STATES = ("_STATE_PLACE_MOVE", "_STATE_PLACE_WAIT")

# 测试用的放置位姿与抬升高度。
# 放置位姿按**参数约定**写 (上报末端原点系: X,Y,Z mm + RX,RY,RZ 度), 测试里再换算成
# 夹爪系 —— 与真实节点同一条路, 抄错单位会在下面的用例里立刻显形。
# 起点 LIFT_START 到放置点 0.36m 上下 < max_command_distance 的一半预算, 所以默认是
# **一段** (要测分段用 test_long_leg_is_split)。
PLACE_REPORTED_MM_DEG = (400.0, -300.0, 200.0, 30.0, -20.0, 45.0)
LIFT_Z = 0.45
LIFT_START = (0.20, 0.0, LIFT_Z)

# 测试用的夹爪偏移 (tool 系, m) —— 只有上面那组位姿的换算用到它。
TEST_OFFSET_TOOL = np.asarray([0.0, 0.0, 0.06])


def _rpy_to_quaternion(rx_deg, ry_deg, rz_deg):
    """ZYX 顺序 RPY(度) -> 四元数 (x, y, z, w)。

    测试里**自己写一份** (与节点的 _rpy_to_quaternion 同式但独立): 复用节点的实现
    就等于让「两边一起抄错」也通过。
    """
    cr, sr = math.cos(math.radians(rx_deg) * 0.5), math.sin(math.radians(rx_deg) * 0.5)
    cp, sp = math.cos(math.radians(ry_deg) * 0.5), math.sin(math.radians(ry_deg) * 0.5)
    cy, sy = math.cos(math.radians(rz_deg) * 0.5), math.sin(math.radians(rz_deg) * 0.5)
    return np.asarray([
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    ])


def _reported_to_gripper(reported_mm_deg, offset_tool=TEST_OFFSET_TOOL):
    """参数的 6 个数 -> 节点内部该得到的 (夹爪位置 m, 姿态四元数)。

       上报原点 = 前三个数 / 1000      (mm -> m)
       姿态     = ZYX(RX, RY, RZ)       (度 -> 四元数, 纯平移不改姿态, 两个系共用)
       夹爪     = 上报 + R @ offset     (offset 在 tool 系)
    """
    raw = np.asarray(reported_mm_deg, dtype=float)
    reported = raw[:3] * 0.001
    quaternion = _rpy_to_quaternion(*raw[3:])
    quaternion = quaternion / np.linalg.norm(quaternion)
    if quaternion[3] < 0.0:
        # 节点把四元数规范到 w >= 0 (q 与 -q 是同一个旋转)。这里跟上, 否则断言会
        # 因为「四个分量全反」而红 —— 那不是 bug。
        quaternion = -quaternion
    x, y, z, w = quaternion
    rotation = np.asarray([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])
    return reported + rotation @ offset_tool, quaternion


# 用例里的节点把夹爪偏移置零 (见 _ExecutorTestCase.setUp: 偏移只作用在「上报系 <->
# 夹爪系」这一个边界上, 与运动逻辑无关) —— 所以运动用例里 夹爪点 == 上报原点。
# 换算本身由 PlaceLoadTest.test_valid_pose_is_converted_at_load_time 单独测, 那里
# 会临时装上一个**非零**偏移。
PLACE_XYZ = np.asarray(PLACE_REPORTED_MM_DEG[:3], dtype=float) * 0.001
PLACE_QUATERNION = _reported_to_gripper(PLACE_REPORTED_MM_DEG)[1]


def _arm_status_msg(motion_status=0, ik_status=0, ik_status_valid=0,
                    ik_joint_limit_fault=0, ik_pose_pending=0):
    """够用的 H3001ArmStatus 替身 (_feedback_problem 与 _ik_rejection_problem 会看
    下面这几个字段)。

    ik_status 默认 **0** = 固件说收下了, 于是 _ik_rejection_problem 直接放行 ——
    默认值下所有老用例 (不动、超时、回位) 的语义一字不变。
    ik_status_valid / ik_joint_limit_fault / ik_pose_pending 默认 0 只是照真机填:
    真机上这三个字段**永远是 0** (SDK 漏拷, 见 test_rejection_fires_even_though_
    valid_is_zero), 判据里已经不允许再依赖它们。
    """
    return SimpleNamespace(
        err_status=0,
        ctrl_mode=1,
        motion_status=motion_status,
        joint_angle_limit=[0] * 6,
        communication_status_joint=[0] * 6,
        ik_status=ik_status,
        ik_status_valid=ik_status_valid,
        ik_joint_limit_fault=ik_joint_limit_fault,
        ik_pose_pending=ik_pose_pending,
    )


class _NullPublisher:
    """替掉真机的 move_p 发布器: 用例只做记录, **一条指令都不许发出去**。

    为什么非有它不可 (2026-09-18 16:49 真机事故): 这些用例是拿 dry_run=False 跑的
    (要验的正是真机那条运动逻辑), 而 `_send_move_p` 在 dry_run=False 分支里会**真的
    publish** 到 /control/move_p。当时 h3001_bridge 还开着, 于是跑一遍 test_place 就
    把机械臂开动了 —— 日志最后那几条 `[78.0, 0.0, 290.0, 2.4, 4.2, -180.0]` 就是用例
    里的回位指令, 用的是**代码默认待机位**; 它与 yaml 现场那组
    `[100.0, 0.0, 290.0, 84.4, 0.2, 90.0]` 差着一整个腕部朝向, 所以现场看到的是
    「什么都没干, 机械臂突然转过去」。

    只换发布器、不换 `_send_move_p`: 发送路径上别的副作用 (日志、_last_command_position)
    照旧要跑, 用例才验得到。
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
        # 残差) 与它无关, 所以测试里置零 —— 参数默认值本来就是零。
        self.node._gripper_offset = np.zeros(3)
        # 记录真正下发的 (位置, 姿态): 「抖动」= 同一条指令被反复重发, 靠计数断言;
        # 姿态用来验证「搬运姿态冻结」。
        self.sent = []
        original_send = self.node._send_move_p

        def record(position, orientation, **kwargs):
            self.sent.append(
                (np.asarray(position, dtype=float).copy(), tuple(orientation))
            )
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

    def _feedback(self, position, orientation=(0.0, 0.0, 0.0, 1.0), motion_status=0, **ik):
        """写一帧「刚收到」的 TCP 位姿 + 臂状态 (时间戳取当前 ROS 时钟)。

        **ik 透传给 _arm_status_msg: 用例要模拟「固件回了拒收判决」时用
        _feedback(..., ik_status=2, ik_status_valid=1)。
        """
        now = self.node.get_clock().now().nanoseconds * 1e-9
        self.node._tcp_pose = (now, np.asarray(position, dtype=float))
        self.node._tcp_orientation = tuple(orientation)
        self.node._arm_status = (now, _arm_status_msg(motion_status, **ik))

    def _break_feedback(self):
        """反馈断掉 (话题不再更新): 时间戳往前挪过 input_timeout。"""
        timeout = float(self.node.get_parameter("input_timeout").value)
        stamp, pose = self.node._tcp_pose
        self.node._tcp_pose = (stamp - timeout - 1.0, pose)

    def _age(self, attribute, seconds):
        """模拟时间流逝: 不做真实 sleep, 把计时起点往前挪。"""
        since = getattr(self.node, attribute)
        if since is not None:
            setattr(self.node, attribute, since - seconds)

    def _enable_place(self, pose=PLACE_REPORTED_MM_DEG, **params):
        """打开放置并重新跑一遍加载期校验 (参数是构造时读的, 测试里要手动重来)。

        pose 是参数原样的 6 个数 (上报末端原点系), 不是换算后的夹爪点。
        """
        values = {
            "place_enabled": True,
            "place_pose_reported_mm_deg": [float(v) for v in pose],
        }
        values.update(params)
        self._set(**values)
        self.node._load_place_pose()
        self.assertTrue(self.node._place_ready, "测试前提: 放置位姿应当校验通过")

    def _enable_return(self):
        """让「任务结束回待机位」这条路径可用 (真机 yaml 里本来就是 true)。"""
        self._set(return_to_standby_on_finish=True)
        self.node._return_pending = True
        self.assertTrue(self.node._standby_ready, "测试前提: 待机位应当校验通过")

    def _enter_lift_done(self, lift_position=LIFT_START):
        """把节点摆到 LIFT **刚到位**的那一刻 (指令已发, 反馈就是抬升目标)。"""
        target = np.asarray(lift_position, dtype=float)
        self.node._lift_target = target
        self.node._goal_orientation = (0.0, 0.0, 0.0, 1.0)
        self.node._transition(self.node._STATE_LIFT, [target])
        self._feedback(target)
        self.node._goal_sent = True
        self.node._goal_position = target.copy()
        self.node._tolerance_start = time.monotonic() - 10.0
        return target

    def _settle_motion(self, target):
        """把 _tick_motion 驱动到「当前路点到位」: 反馈停在 target, 再让停稳计时满足。"""
        self._feedback(target)
        self.node._tick_motion()                       # 进容差, 开始计时
        self._age("_tolerance_start",
                  float(self.node.get_parameter("settle_time").value) + 0.01)
        self.node._tick_motion()                       # 停稳够了 -> 推进/换状态

    def _enter_place_phase(self, lift_position=LIFT_START):
        """LIFT 到位 -> PLACE_MOVE 的第一条指令已下发, 返回 (第一条指令的目标, 放置点)。

        2026-09-18 起 PLACE_MOVE 是**一条直线直达放置位姿本身**, 没有「放置点正上方」
        那个中间点了: 所以第一条指令的目标就是分段队列的第一段, 距离在预算内时它**就是**
        放置点本身 (要测分段用 test_long_leg_is_split)。
        """
        self._enter_lift_done(lift_position)
        self.node._tick_motion()                       # LIFT 走完 -> _enter_place
        self.assertEqual(self.node._state, self.node._STATE_PLACE_MOVE)
        place = np.asarray(self.node._place_position, dtype=float)
        target = np.asarray(self.node._waypoints[0], dtype=float)
        self._feedback(lift_position)
        self.node._tick_motion()                       # 下发第一条
        return target, place

    def _reach_place(self, lift_position=LIFT_START):
        """从 LIFT 到位一路推到「抵达放置点、已进 PLACE_WAIT」, 返回放置点。

        逐段喂反馈把 PLACE_MOVE 的整条路点队列走完 (含最后一段), 再走完 PLACE_WAIT
        入口那一拍。用例要验「到达之后才放」时从这里起步。
        """
        self._enter_place_phase(lift_position)
        self.assertEqual(len(self.sent), 1, "测试前提: 第一条指令已下发")
        while self.node._state == self.node._STATE_PLACE_MOVE:
            target = np.asarray(self.node._waypoints[self.node._waypoint_index])
            self._settle_motion(target)
        self.assertEqual(self.node._state, self.node._STATE_PLACE_WAIT)
        self.sent.clear()
        return np.asarray(self.node._place_position, dtype=float)


class PlaceDisabledTest(_ExecutorTestCase):
    """place_enabled=false (本次交付的默认状态): 老流程一字不变。"""

    def test_default_is_disabled(self):
        self.assertFalse(self.node.get_parameter("place_enabled").value)
        self.assertFalse(self.node._place_ready)
        self.assertIsNone(self.node._place_position)
        self.assertIsNone(self.node._place_orientation)

    def test_lift_goes_straight_home(self):
        self._enable_return()
        self._enter_lift_done()
        self.node._tick_motion()
        self.assertEqual(self.node._state, self.node._STATE_HOME, "没配放置就该直接回待机位")
        self.assertEqual(len(self.sent), 0, "回位指令还没到下发的那一拍")

    def test_lift_without_return_goes_idle(self):
        self._enter_lift_done()
        self.node._tick_motion()
        self.assertEqual(self.node._state, self.node._STATE_IDLE)

    def test_lift_goes_home_in_one_command_to_the_standby_pose(self):
        """抬升到位后**一条**指令就把 standby_pose_reported_mm_deg 发下去。

        2026-09-18 现场要求: 夹住物体抬升完, 别再走放置那两段, 直接发待机位那 6 个数
        (夹爪保持闭合, 物体被带回去)。place_enabled=false 时 LIFT 落到
        _finish_task -> _enter_home, 这里钉住它下发的**就是参数本身**, 而不是什么
        中间点。

        为什么**恒为**「一条」: 2026-09-21 起回位不再拆段 (真机上山拆出来的中间点配上
        待机姿态被固件 IK 拒收, 见 _home_time_multiplier_for), 无论起点多远都只有
        [待机位] 这一个路点 —— 所以这里断言的不是「这段路够近」, 而是「队列长度恒为 1」。
        """
        self._set(dry_run=False)          # 回位起点取真实反馈, 与真机同一条路径
        self._enable_return()
        self._enter_lift_done()
        self.node._tick_motion()          # LIFT 到位 -> 进 HOME
        self.assertEqual(self.node._state, self.node._STATE_HOME)
        self.assertEqual(
            len(self.node._waypoints), 1,
            "回位恒为一条绝对待机位; 变成多条说明拆段又回来了, 见 _home_time_multiplier_for",
        )

        self.node._tick_motion()          # HOME 第一拍: 下发整条绝对待机位
        self.assertEqual(len(self.sent), 1, "抬升之后只该有这一条指令")
        position, orientation = self.sent[0]
        np.testing.assert_allclose(position, self.node._standby_position)
        np.testing.assert_allclose(orientation, self.node._standby_orientation)

        # 换回真偏移, 再换算一遍: 真正发到 /control/move_p 的**上报末端原点**必须就是
        # 参数里那三个数。上面那条断言比的是节点内部量 (夹爪点), 偏移被 setUp 置零后
        # 两者恰好相等 —— 也就是说光比它验不到「下发前换算」这一步。
        self.node._gripper_offset = np.asarray(
            self.node.get_parameter("gripper_offset_tool_xyz").value, dtype=float
        )
        reported_mm = self.node._gripper_to_reported(position, orientation) * 1000.0
        expected_mm = np.asarray(
            self.node.get_parameter("standby_pose_reported_mm_deg").value[:3],
            dtype=float,
        )
        np.testing.assert_allclose(reported_mm, expected_mm, atol=1e-6)


class PlaceLoadTest(_ExecutorTestCase):
    """加载期校验: 填错不 raise, 但会禁用放置 (随后 _on_execute 拒绝启动)。"""

    def _load(self, pose, enabled=True):
        self._set(
            place_enabled=enabled,
            place_pose_reported_mm_deg=[float(v) for v in pose],
        )
        self.node._load_place_pose()

    def test_placeholder_is_rejected(self):
        """占位值全零换算后离基座 0.06m (偏移), 在工作空间内 —— 只能靠「离基座原点
        太近」拦下, 拦的必须是**上报原点**而不是换算后的夹爪点。"""
        self._load((0.0, 0.0, 0.0, 0.0, 0.0, 0.0))
        self.assertFalse(self.node._place_ready)
        self.assertIsNone(self.node._place_position)
        self.assertIsNone(self.node._place_orientation)

    def test_wrong_length_is_rejected(self):
        """3 个数是**旧约定** (夹爪系/米), 从 2026-09-18 起必须写 6 个。"""
        self._load((0.40, -0.30, 0.20))
        self.assertFalse(self.node._place_ready)

    def test_millimetres_written_as_metres_is_rejected(self):
        """按米填 (0.4 而不是 400): 上报原点只有 0.0004m, 靠「离基座太近」拦下。"""
        self._load((0.4, -0.3, 0.2, 0.0, 0.0, 0.0))
        self.assertFalse(self.node._place_ready)

    def test_out_of_workspace_is_rejected(self):
        self._load((1500.0, 0.0, 200.0, 0.0, 0.0, 0.0))
        self.assertFalse(self.node._place_ready)

    def test_valid_pose_is_converted_at_load_time(self):
        """参数是上报末端原点系, 加载期换算成夹爪系 + 四元数 —— 对照测试里另写一份
        的定义 (_reported_to_gripper), 两边必须一致。

        这个用例**临时装上非零的夹爪偏移**: 偏移置零时换算退化成恒等变换, 那样测
        等于没测 (而真机上偏移是 60mm 量级, 单位/系抄错就出在这里)。
        """
        self.node._gripper_offset = TEST_OFFSET_TOOL.copy()
        self._load(PLACE_REPORTED_MM_DEG)
        self.assertTrue(self.node._place_ready)
        expected_position, expected_quaternion = _reported_to_gripper(
            PLACE_REPORTED_MM_DEG, TEST_OFFSET_TOOL
        )
        np.testing.assert_allclose(self.node._place_position, expected_position)
        quaternion = np.asarray(self.node._place_orientation, dtype=float)
        np.testing.assert_allclose(quaternion, expected_quaternion)
        self.assertGreaterEqual(quaternion[3], 0.0, "四元数要规范到 w >= 0")
        # 上报原点与夹爪点必须真的不同, 否则上面那两行断言是白测
        self.assertGreater(
            float(np.linalg.norm(expected_position - PLACE_XYZ)), 0.01
        )

    def test_orientation_is_the_parameters_not_the_grasp_one(self):
        """2026-09-18: 放置姿态不再沿用抓取姿态 —— 换一组 RPY 就必须换一组四元数。"""
        self._load(PLACE_REPORTED_MM_DEG)
        first = np.asarray(self.node._place_orientation, dtype=float)
        self._load((400.0, -300.0, 200.0, 0.0, 0.0, 90.0))
        second = np.asarray(self.node._place_orientation, dtype=float)
        self.assertFalse(np.allclose(first, second), "RZ 变了, 姿态就该跟着变")
        np.testing.assert_allclose(first, PLACE_QUATERNION)

    def test_reload_clears_a_previous_orientation(self):
        """先填好再改坏: _place_orientation 必须跟着 _place_ready 一起清掉。

        否则「_place_ready 为假」时它还留着上一组的姿态, 将来某条早退路径上的
        _descend_plan 会拿它当有效值用。
        """
        self._load(PLACE_REPORTED_MM_DEG)
        self.assertIsNotNone(self.node._place_orientation)
        self._load((0.0, 0.0, 0.0, 0.0, 0.0, 0.0))
        self.assertIsNone(self.node._place_orientation)

    def test_execute_is_refused_when_enabled_but_invalid(self):
        """开着放置却填错坐标: 现在拒绝, 而不是抓完才发现无处可放。"""
        self._load((0.0, 0.0, 0.0, 0.0, 0.0, 0.0))

        class _Response:
            success = None
            message = None

        response = self.node._on_execute(None, _Response())
        self.assertFalse(response.success)
        self.assertIn("place_pose_reported_mm_deg", response.message)
        self.assertEqual(self.node._state, self.node._STATE_IDLE)

    def test_execute_is_allowed_when_disabled(self):
        self._load((0.0, 0.0, 0.0, 0.0, 0.0, 0.0), enabled=False)

        class _Response:
            success = None
            message = None

        response = self.node._on_execute(None, _Response())
        self.assertTrue(response.success)


# 2026-09-30 现场那一组: 倒瓶 (top) 的抓取姿态、构型、夹爪点。
# 姿态取自桥日志里 PRE_GRASP 实际下发的那个四元数 (工具 Z 竖直朝下, rx≈179°),
# 构型取自同一轮 DESCEND 的 move_j (话题约定, 度), 夹爪点取自「已下降到抓取点」那行。
GRASP_TOP_ORIENTATION = (0.9997, 0.0059, 0.0206, 0.0067)
GRASP_TOP_JOINTS_DEG = (-6.8, -83.17, 47.05, -0.6, -51.45, 172.9)
GRASP_TOP_POINT = (0.2413, -0.0296, 0.0672)
# yaml 里那组现场实测的 side 抓取姿态 (2026-09-14 从实机读出)。
SIDE_ORIENTATION_XYZW = (0.031741, 0.768151, 0.002256, 0.639477)
SITE_PLACE_POSE = (160.0, -200.0, 300.0, -76.4, 86.2, -132.0)


class LiftAlignTest(_ExecutorTestCase):
    """抬升时顺带回正姿态 (2026-09-30): 把放置段那次大转腕挪到抬升里做。

    背景是现场那次失败 —— 抓完倒瓶 (top) 跳过抬升, 放置那一条指令要从离台面 6.7cm 的
    抓取点同时走 34.6cm + 转腕 144°: IK 解不出来 (残差 3.19mm/16.47°), 回落 move_p
    交给固件规划, 大摆到 `err_status=512`。修法是让抬升那一条指令的目标姿态改成 side
    抓取姿态, 于是到达放置段时手臂已经在 side 姿态上 (放置段只剩 ~54°)。

    离线 (同一套三道守门) 验过的那几个数就是下面这些断言。
    """

    def _site_setup(self, side_orientation=SIDE_ORIENTATION_XYZW):
        """把节点摆到现场那一轮的状态: top 抓完、手里夹着瓶子。"""
        self._set(
            dry_run=False,
            side_orientation_xyzw=list(side_orientation),
            lift_distance=0.10,
            place_enabled=True,
            place_pose_reported_mm_deg=list(SITE_PLACE_POSE),
        )
        self.node._load_place_pose()
        self.assertTrue(self.node._place_ready, "测试前提: 放置点要能通过校验")
        # 现场 gripper_offset_tool_xyz = [0,0,0.10] —— 回正的 FK 复核要跨「上报系 <->
        # 夹爪系」这个边界, 置零就把它测没了。
        self.node._gripper_offset = np.asarray([0.0, 0.0, 0.10])
        self.node._grasp_mode = "top"
        self.node._goal_orientation = GRASP_TOP_ORIENTATION
        self.node._final_grasp_target = np.asarray(GRASP_TOP_POINT)
        self._feedback(list(GRASP_TOP_POINT), GRASP_TOP_ORIENTATION)
        now = self.node.get_clock().now().nanoseconds * 1e-9
        self.node._joint_deg = (now, list(GRASP_TOP_JOINTS_DEG))

    def _assert_same_rotation(self, first, second, places=3):
        dot = abs(sum(float(a) * float(b) for a, b in zip(first, second)))
        self.assertAlmostEqual(dot, 1.0, places=places, msg="这不是同一个旋转")

    def test_predicate_is_silent_when_the_attitude_already_matches(self):
        """姿态本来就等于 side 姿态 -> 不回正 (`_lift_align_orientation` 是纯谓词)。"""
        self._site_setup()
        self._feedback(list(GRASP_TOP_POINT), SIDE_ORIENTATION_XYZW)
        self.assertIsNone(self.node._lift_align_orientation())

    def test_predicate_targets_the_side_attitude(self):
        """工具朝下 (现场实测) -> 回正目标就是 side 姿态参数本身。"""
        self._site_setup()
        target = self.node._lift_align_orientation()
        self.assertIsNotNone(target, "176° 的差必须触发回正")
        self._assert_same_rotation(target, SIDE_ORIENTATION_XYZW)

    def test_threshold_can_turn_the_align_off(self):
        """阈值放宽到 180° 就恒不回正 —— 现场要退回老行为时用这个旋钮。"""
        self._site_setup()
        self._set(lift_align_threshold_deg=180.0)
        self.assertIsNone(self.node._lift_align_orientation())

    def test_lift_align_builds_joint_sampled_waypoints(self):
        """抬升那一条指令: 拆成 N 段关节直线采样, 每段都过**完整**的三道守门。

        拆段不是为了绕开守门 (那条路会退回 move_p, 正是出事那条), 而是因为单轴差有
        150° (J6) —— 一条发出去会被「换构型大摆」拒掉。按关节直线拆开之后每段 ≈50°,
        三道守门照旧全开。断言的就是这个: **每段都能在完整守门下解出来**。
        """
        if not self.node._ik_ready:
            self.skipTest("本机没装上 pinner 依赖 / 找不到 vendored IK 三件套")
        self._site_setup()
        self.node._begin_lift()
        self.assertEqual(self.node._state, self.node._STATE_LIFT)
        segments = int(self.node.get_parameter("lift_align_segments").value)
        self.assertEqual(len(self.node._waypoints), segments)
        self.assertEqual(len(self.node._waypoint_orientations), segments)
        # 最后一段 = 抬升目标 + side 姿态 (FK 复核在内部已经把关, 这里是外部再确认)
        np.testing.assert_allclose(
            self.node._waypoints[-1], self.node._lift_target, atol=2e-3
        )
        self._assert_same_rotation(
            self.node._waypoint_orientations[-1], SIDE_ORIENTATION_XYZW
        )
        # 高度单调上升: 起步 6.7cm -> 9.5 -> 12.7 -> 16.7cm, 不许先下去再上来
        heights = [GRASP_TOP_POINT[2]] + [float(p[2]) for p in self.node._waypoints]
        self.assertEqual(heights, sorted(heights), f"抬升路径高度不单调: {heights}")
        # 每一段都在完整的三道守门 (收敛 / CAD 余量 / 单轴差) 下解出来
        seed = self.node._joint_seed_deg()
        for index, (position, orientation) in enumerate(
            zip(self.node._waypoints, self.node._waypoint_orientations), start=1
        ):
            joint, reason = self.node._solve_joint_target(
                position, orientation, seed_deg=seed
            )
            self.assertIsNotNone(joint, f"第 {index} 段过不了守门: {reason}")
            seed = np.asarray(self.node._last_solved_urdf_deg, dtype=float)
        # 回正之后那段放置: 几何上**解得出、余量也够**, 卡的只有守门 ③ —— 从回正后的
        # 构型去放置位姿, 差的是腕部 J4 63.2° / J6 55.3° (肩肘只有 29.2°), 比 60° 的
        # 门槛多 3°。2026-09-30 的决定是**这一段先不动**, 所以这条用例记录的是现状:
        # 一旦谁改了那个门槛或把放置段也拆段, 这里的数会变, 这条就会响。
        # (求解用 align=True 只为绕开那道门把几何量出来 —— 下发时它仍然走完整守门。)
        joint, reason = self.node._solve_joint_target(
            np.asarray([0.2052, -0.266, 0.3012]),
            self.node._place_orientation,
            seed_deg=seed,
            align=True,
        )
        self.assertIsNotNone(joint, f"回正之后放置段反而解不出来: {reason}")
        solved = np.asarray(self.node._last_solved_urdf_deg, dtype=float)
        delta = np.abs(solved - seed)
        self.assertLess(delta[:3].max(), 30.0, "肩肘那三个轴不该被回正放大")
        self.assertAlmostEqual(delta[3:].max(), 63.2, delta=1.0,
                               msg="腕部这一下就是放置段现在回落 move_p 的原因")
        margin = float(
            np.minimum(
                solved - np.degrees(self.node._ik_cad_lo),
                np.degrees(self.node._ik_cad_hi) - solved,
            ).min()
        )
        self.assertGreater(margin, 5.0, "几何没问题, 卡的是单轴门不是限位")
        # 从回正后的构型去放置, 解出来的是**够得到的那一版** (话题约定, 度) ——
        # 与现场从 side 构型出发成功的那一版 [-50.5 -63.36 38.02 11.35 26.69 -13.94]
        # 是同一个量级、同一类构型。
        np.testing.assert_allclose(
            joint, [-49.7, -58.5, 35.1, 13.9, 25.0, -16.4], atol=0.2
        )

    def test_align_falls_back_to_a_plain_lift_when_it_cannot_solve(self):
        """解不出来时回落「只抬升、不回正」, **不把整轮判失败** (任务还能跑完)。

        这里用节点里那组**代码默认**的 side_orientation_xyzw —— 它与现场那组不是同一个
        姿态, 在抬升后的位置上无解 (实测残差 3.06mm/96.02°), 正好当这个回落的输入。
        """
        # 先取「节点默认值」那一组 (还没被 _site_setup 覆盖成现场值)
        node_default_side = tuple(self.node.get_parameter("side_orientation_xyzw").value)
        self._site_setup(side_orientation=node_default_side)
        self.node._begin_lift()
        self.assertEqual(self.node._state, self.node._STATE_LIFT)
        self.assertEqual(len(self.node._waypoints), 1, "应回落到只有抬升那一条")
        np.testing.assert_allclose(
            self.node._waypoints[0], self.node._lift_target, atol=2e-3
        )
        self.assertIsNone(self.node._waypoint_orientations)


class PlaceMoveTest(_ExecutorTestCase):
    """PLACE_MOVE: 平移到放置点正上方, 判到位 (所以走 _tick_motion)。"""

    def setUp(self):
        super().setUp()
        self._set(dry_run=False)
        self._enable_place()

    def test_lift_advances_to_place_move(self):
        self._enter_lift_done()
        self.node._tick_motion()
        self.assertEqual(self.node._state, self.node._STATE_PLACE_MOVE)

    def test_goes_straight_to_the_place_pose(self):
        """**没有中间点**: 第一条指令发的就是放置位姿本身 (2026-09-18)。

        原来这里是「先平移到放置点正上方 (x,y 用放置点、z 用抬升后的实际高度)」,
        那个点用户从来没要求过, 真机上 IK 拒收的恰恰是它。这里直接钉住: 这一段路的
        终点 (也是唯一的路点) 就是参数那 6 个数换算出来的位姿。
        """
        target, place = self._enter_place_phase()
        self.assertEqual(len(self.sent), 1)
        np.testing.assert_allclose(self.sent[0][0], place)
        np.testing.assert_allclose(target, place)
        np.testing.assert_allclose(place, PLACE_XYZ, atol=1e-9)
        # 反过来说: 「放置点正上方」那个点不许再出现在指令里
        above = np.asarray((PLACE_XYZ[0], PLACE_XYZ[1], LIFT_START[2]))
        self.assertGreater(
            float(np.linalg.norm(place - above)), 1e-6,
            "测试前提: 本用例的放置位姿与「正上方」本来就不是同一点",
        )

    def test_arrival_advances_to_place_wait(self):
        """到位 -> 直接进 PLACE_WAIT 放下 (中间那段盲降已经删掉)。"""
        target, _ = self._enter_place_phase()
        self.assertEqual(len(self.sent), 1)
        np.testing.assert_allclose(self.sent[0][0], target)
        self._settle_motion(target)
        self.assertEqual(self.node._state, self.node._STATE_PLACE_WAIT)

    def test_short_leg_is_not_split(self):
        _, place = self._enter_place_phase()
        self.assertEqual(len(self.node._waypoints), 1)
        np.testing.assert_allclose(self.node._waypoints[0], place)

    def test_a_long_leg_is_still_one_command(self):
        """再远也是**一条** move_p —— 放置段不再按 max_command_distance 拆段。

        2026-09-20 改 (用户要求「运动规划全靠机械臂本身」): 那个上限防的是「感知给的
        坐标突然跳一大截」, 而放置点是 yaml 里的常量、不是感知, 距离远是正常的。
        这里用一个 0.85 m 的放置点 (远超 0.50 m 上限) 钉住: 路点队列必须仍然只有
        一条, 而且 _tick_motion 不许因为它超限而 _fail。

        工作空间按现场 yaml 的 ±0.90 放开来用 (节点默认只有 ±0.60, 是给部署前的
        保守值) —— 放置点本来就可以在工作空间的另一头。
        """
        self._set(workspace_x_max=0.90, workspace_y_min=-0.90)
        self._enable_place(pose=(800.0, -600.0, 200.0, 0.0, 0.0, 0.0))  # 约 0.85m
        _, place = self._enter_place_phase()
        self.assertEqual(len(self.node._waypoints), 1, "放置段不许再拆段")
        np.testing.assert_allclose(self.node._waypoints[0], place)
        distance = float(np.linalg.norm(place - np.asarray(LIFT_START, dtype=float)))
        self.assertGreater(
            distance, float(self.node.get_parameter("max_command_distance").value),
            "测试前提: 这一段的距离必须超过单条指令上限, 否则这条用例什么也没验到",
        )
        # 指令真的发出去了 —— 距离检查对放置段放行, 没有把任务判失败
        self.assertEqual(len(self.sent), 1)
        np.testing.assert_allclose(self.sent[0][0], place)
        self.assertEqual(
            self.node._state, self.node._STATE_PLACE_MOVE,
            "超限不该让放置段判失败 —— 那正是 _check_distance 豁免要做的事",
        )

    def test_the_limit_is_waived_for_configured_goals_only(self):
        """豁免是**调用点**按状态给的 (PLACE_MOVE / HOME), `_check_distance` 本身没松。

        2026-09-20 给放置段、2026-09-21 给回位: 两者的目标都是 yaml 里的常量、不是
        感知, 距离远是正常的。别处 (PRE_GRASP / LIFT / 跟随) 还要靠这道上限防
        「感知给的坐标突然跳一大截」(README §5.10), 所以放宽绝不能写进 _check_distance
        本身 —— 这条用例分两层钉住: 函式层照拦, 调用点按状态分派。

        PLACE_MOVE 那一半由 test_a_long_leg_is_still_one_command 覆盖 (那里真的走
        _tick_motion 发指令); 这里补的是「没被豁免的状态真的会因此判失败」。
        """
        far = np.asarray(LIFT_START, dtype=float) + np.asarray((1.0, 0.0, 0.0))
        self._feedback(LIFT_START)
        # 函式层: 默认仍然拦; 豁免是显式开关
        self.assertFalse(
            self.node._check_distance(far),
            "别处超限必须拦下; 若这条挂了说明豁免被写成了全局的",
        )
        self.assertTrue(
            self.node._check_distance(far, enforce_max=False),
            "豁免开关本身要能放行",
        )
        # 调用点: PRE_GRASP 不在豁免名单里, 超限就真的不下发、判失败
        self.node._goal_orientation = (0.0, 0.0, 0.0, 1.0)
        self.node._transition(self.node._STATE_PRE_GRASP, [far])
        self.node._tick_motion()
        self.assertEqual(len(self.sent), 0, "PRE_GRASP 超限不该下发")
        self.assertEqual(
            self.node._state, self.node._STATE_IDLE,
            "未豁免的状态超限就该判失败 (回位/放置是仅有的两个例外)",
        )

    def test_place_orientation_is_the_parameters_one(self):
        """搬运姿态来自参数 (加载期算好), 不是进 PLACE_MOVE 时才定的。

        2026-09-18 之前这里冻结的是**当时的 TCP 姿态**, 放置姿态因此被抓取姿态绑死。
        现在整段都用参数里的那一组, 与抬升时的实际姿态无关。
        """
        self._enter_lift_done()
        self._feedback(LIFT_START, orientation=(0.0, 0.0, 0.0, 1.0))
        self.node._tick_motion()
        np.testing.assert_allclose(
            np.asarray(self.node._place_orientation, dtype=float),
            np.asarray(PLACE_QUATERNION, dtype=float),
            err_msg="放置姿态必须是参数那组 RPY (加载期算好), 与抬升时的 TCP 姿态无关",
        )

    def test_a_tilted_wrist_does_not_change_the_place_orientation(self):
        """途中腕部姿态变了也不跟 —— 整段共用同一个常量姿态。"""
        self._enter_lift_done()
        self._feedback(LIFT_START, orientation=(0.0, 0.0, 0.0, 1.0))
        self.node._tick_motion()
        frozen = tuple(self.node._place_orientation)
        # 途中腕部变成别的姿态 (下垂 / 固件微调都行)
        tilted = (0.1, 0.2, 0.3, 0.927)
        self._feedback(LIFT_START, orientation=tilted)
        self.node._tick_motion()                       # 下发这一段
        self.assertEqual(len(self.sent), 1)
        np.testing.assert_allclose(np.asarray(self.sent[0][1]), np.asarray(frozen))
        # 到位判定那几拍里姿态又变了, 也不该改写下发的姿态
        self._feedback(np.asarray(self.node._waypoints[0]), orientation=tilted)
        self.node._tick_motion()
        self.assertEqual(len(self.sent), 1, "到位判定期间不该重发")


class PlaceMoveCorrectionTest(_ExecutorTestCase):
    """PLACE_MOVE 的迭代末端修正: 「停稳」的前提是「先动过」。

    2026-09-18 真机日志里的病: 这条指令刚发出去、机械臂还在原地待命的那一拍,
    停稳检测把「还没起步」读成了「停稳但够不到」, 于是按残差重发 —— 而站在起点上算出来
    的残差就是**整段位移**, 指令当场被加成两倍距离。那次是 24.4cm 的位移、累计修正算到
    244mm 被 max_correction_distance 拦下才没出事; 短于 8cm 的位移拦不住, 机械臂会冲过
    放置点。

    这里用**短位移** (3cm) 复现 —— 上限拦不住的那种。放置段现在判到位了 (不再是盲降),
    所以这套迭代修正**正是**它的一部分, 更要钉住。
    """

    # 放置点离抬升点 (0.20, 0, 0.45) 只有 **水平 3cm** (z 取同一个 450mm)。
    # 纯水平是刻意的: 迭代修正是**累加**的 (_cmd_offset += 残差), 而本类那条
    # 「重发之后还能继续迭代」的用例要连做 3 次, 每次残差必须**一样大且够小**,
    # 3*残差 才落在 max_correction_distance (8cm) 之内 —— 否则第 3 次就被上限判死,
    # _retry_count 冻在 1, 看起来像「重发之后不再迭代」而其实是测试自己的几何超限。
    # (改这个数之前先算一遍: 残差 = |放置点 - moved|, moved = LIFT_START + 1cm。)
    NEAR_POSE_MM_DEG = (230.0, 0.0, 450.0, 0.0, 0.0, 0.0)

    def setUp(self):
        super().setUp()
        self._set(dry_run=False)
        self._enable_place(pose=self.NEAR_POSE_MM_DEG)
        self.approach, self.place = self._enter_place_phase()
        self.assertEqual(len(self.sent), 1, "测试前提: 横移第一段已下发")

    def test_a_command_that_never_moved_is_not_read_as_stalled(self):
        """机械臂一步没动 -> 停稳计时再怎么往前挪也不许重发。"""
        limit = float(self.node.get_parameter("max_correction_distance").value)
        residual = float(np.linalg.norm(self.approach - np.asarray(LIFT_START)))
        self.assertLess(residual, limit, "测试前提: 残差没超上限, 上限挡不住这个 bug")
        for _ in range(20):
            self._feedback(LIFT_START)                  # 一动没动
            # 停稳计时往前挪满 —— 修之前正是这一下把「还没起步」喂成了「停稳」。
            self._age("_settle_since", 5.0)
            self.node._tick_motion()
        self.assertEqual(len(self.sent), 1, "没起步 != 停稳 —— 一条指令都不该重发")
        self.assertFalse(np.any(self.node._cmd_offset), "不该产生任何修正量")
        self.assertEqual(self.node._retry_count, 0)
        self.assertEqual(self.node._state, self.node._STATE_PLACE_MOVE)

    def test_it_still_corrects_once_the_arm_has_moved(self):
        """动过之后再停住够不到: 迭代修正照旧, 且修正量是**残差本身**、不是两倍位移。"""
        moved = np.asarray(LIFT_START) + np.asarray((0.010, 0.0, 0.0))   # 走了 1cm
        self._feedback(moved)
        self.node._tick_motion()                        # 看见它动过 -> 武装停稳检测
        self.assertTrue(self.node._stall_armed, "动过就该武装")
        self.assertEqual(len(self.sent), 1)

        settle = float(self.node.get_parameter("stall_detect_time").value)
        self._feedback(moved)
        self._age("_settle_since", settle + 0.01)
        self.node._tick_motion()

        self.assertEqual(len(self.sent), 2, "停稳且够不到 -> 修正一次")
        self.assertEqual(self.node._retry_count, 1)
        expected = np.asarray(self.approach, dtype=float) - moved
        np.testing.assert_allclose(self.node._cmd_offset, expected)
        np.testing.assert_allclose(self.sent[1][0], np.asarray(self.approach) + expected)
        # 「两倍位移」的 bug 会给出 2*(approach - LIFT_START), 与上面那条差一倍
        self.assertLess(
            float(np.linalg.norm(self.node._cmd_offset)),
            float(np.linalg.norm(
                np.asarray(self.approach) - np.asarray(LIFT_START)
            )),
            "修正量必须小于整段位移 —— 否则就是把还没走的那截也算进去了",
        )

    def test_arming_survives_a_correction_resend(self):
        """重发之后**不清**武装: 否则「重发后仍然没动」会被当成「还没起步」, 迭代卡死。

        节点的迭代上限 (max_correction_retries) 是兜底: 一直动不了就交给
        motion_timeout, 但不该在第一次重发后就再也不试。
        """
        moved = np.asarray(LIFT_START) + np.asarray((0.010, 0.0, 0.0))
        self._feedback(moved)
        self.node._tick_motion()
        settle = float(self.node.get_parameter("stall_detect_time").value)
        for _ in range(3):
            self._feedback(moved)                       # 重发之后也没再动
            self._age("_settle_since", settle + 0.01)
            self.node._tick_motion()
        self.assertGreaterEqual(self.node._retry_count, 2, "重发后应当还能继续迭代")
        self.assertTrue(self.node._stall_armed)


class PlaceArrivalTest(_ExecutorTestCase):
    """到达放置点之后: 判到位、不提前放、也不受「盲降不受超时管」那套影响。

    这一组替掉了原来的 PlaceDescendTest —— 那时 PLACE_DESCEND 是**盲降** (一条指令、
    不判到位、不受 motion_timeout 管), 现在整段只剩「直线直达 + 判到位」。
    """

    def setUp(self):
        super().setUp()
        self._set(dry_run=False)
        self._enable_return()      # 失败要能看出「回了待机位」, 与真机 yaml 一致
        self._enable_place()

    def test_arrival_is_judged_not_guessed(self):
        """这是自由空间运动, 终点有明确答案 —— 所以**必须**判到位。

        与抓取侧的 DESCEND 故意相反: 那一段压到台面/瓶身上以后机械臂就下不去了, 判到位
        只会为了追那个永远追不上的残差持续往下压。释放点悬空时没有这个问题。
        """
        self._enter_place_phase()
        far = np.asarray(LIFT_START) + np.asarray((0.05, 0.0, 0.0))
        self._feedback(far)                            # 差着 5cm
        self.node._tick_motion()
        self.assertEqual(self.node._state, self.node._STATE_PLACE_MOVE,
                         "没到就是没到 —— 不许像盲降那样按时间放行")

    def test_motion_timeout_applies(self):
        """判到位 => 一路推不动会被 motion_timeout 拦下并报失败 (盲降那时不会)。"""
        self._enter_place_phase()
        limit = float(self.node.get_parameter("motion_timeout").value) + 5.0
        self._age("_waypoint_enter_time", limit)
        self._age("_state_enter_time", limit)
        self._feedback(LIFT_START)                     # 一动没动
        self.node._tick_motion()
        self.assertEqual(self.node._state, self.node._STATE_HOME, "该失败并回位")

    def test_stale_feedback_fails_before_the_command(self):
        self._enter_lift_done()
        self._break_feedback()
        self.node._tick_motion()                       # LIFT 到位 -> _enter_place
        self.assertEqual(self.node._state, self.node._STATE_HOME, "该失败并回位")
        self.assertTrue(
            all(np.linalg.norm(np.asarray(p) - np.asarray(LIFT_START)) < 1e-9
                for p, _ in self.sent),
            "反馈不可信时不该发出任何放置指令",
        )


class _PlaceWaitSetup(_ExecutorTestCase):
    """PLACE_WAIT 的公共起点: 已经抵达放置点, 停在 PLACE_WAIT 等下一步。

    不写成 PlaceWaitTest 本身再让自动模式那个类继承 —— 那样父类的用例会在子类里
    再跑一遍, 而 "没人确认就该一直等" 在自动模式下必然失败。抽成只带 setUp 的基类,
    两边各自列自己的用例, 个数一一对得上。
    """

    def setUp(self):
        super().setUp()
        self._set(dry_run=False)
        self._enable_place()
        self.place = self._reach_place()


class PlaceWaitTest(_PlaceWaitSetup):
    """PLACE_WAIT: 第三次人工确认, 确认后回待机位。"""

    def test_waits_for_a_confirmation(self):
        self.node._tick_place_wait()
        self._feedback(self.place)
        self.node._tick_place_wait()
        self.assertEqual(self.node._state, self.node._STATE_PLACE_WAIT,
                         "没人确认就该一直等")
        self.assertEqual(len(self.sent), 0)

    def test_stale_enter_does_not_leak_in(self):
        """进 PLACE_WAIT 时旧标志必须清掉: 现在要按三次, 更可能被误按。"""
        self.node._transition(self.node._STATE_PLACE_MOVE, self.node._waypoints)
        self.node._enter_pressed = True
        self.node._confirm_received = True
        self.node._transition(self.node._STATE_PLACE_WAIT)
        self.assertFalse(self.node._enter_pressed)
        self.assertFalse(self.node._confirm_received)

    def test_enter_confirmation_finishes_the_task(self):
        self._feedback(self.place)
        self.node._enter_pressed = True
        self.node._tick_place_wait()
        self.assertEqual(self.node._state, self.node._STATE_IDLE)
        self.assertFalse(self.node._enter_pressed, "确认用过就要清掉")

    def test_confirm_topic_works_in_place_wait(self):
        self.node._on_grasp_confirm(SimpleNamespace(data=True))
        self.assertTrue(self.node._confirm_received)
        self._feedback(self.place)
        self.node._tick_place_wait()
        self.assertEqual(self.node._state, self.node._STATE_IDLE)

    def test_confirm_topic_is_ignored_in_other_states(self):
        self.node._transition(self.node._STATE_PLACE_MOVE, self.node._waypoints)
        self.node._on_grasp_confirm(SimpleNamespace(data=True))
        self.assertFalse(self.node._confirm_received, "别的状态不该被这条话题确认")

    def test_confirmation_returns_to_standby(self):
        self._enable_return()
        self.node._enter_pressed = True
        self.node._tick_place_wait()
        self.assertEqual(self.node._state, self.node._STATE_HOME)
        self.assertEqual(len(self.node._waypoints), 1, "回位恒为一条绝对待机位")
        np.testing.assert_allclose(self.node._waypoints[0], self.node._standby_position)

    def test_confirmation_without_standby_does_not_crash(self):
        """回位关掉 / 待机位不可用: 打一行日志停在原地, 不能拿 None 当待机位用。"""
        self.node._standby_ready = False
        self.node._enter_pressed = True
        self.node._tick_place_wait()
        self.assertEqual(self.node._state, self.node._STATE_IDLE)


class PlaceWaitAutoConfirmTest(_PlaceWaitSetup):
    """place_confirm_required=false (现场默认): 不等 Enter, 停满 dwell 就往下走。

    起点与 PlaceWaitTest 完全相同 (已经抵达放置点、停在 PLACE_WAIT), 差别只在开关,
    所以两边的用例一一对得上: 那边是「没人确认就一直等」, 这边是「照样走完」。
    本类里夹爪不由节点接管 (_gripper_ready=False), 覆盖的是 PLACE_WAIT -> _finish_task
    这条直线; 夹爪由节点接管时 PLACE_WAIT -> RELEASE 那条在 test_gripper.py 里。
    """

    def setUp(self):
        super().setUp()
        # 断言的是**代码默认值**必须是 true —— 现场是靠 yaml 关掉, 不是靠改代码;
        # 这条同时挡住「有人图省事把默认值直接改成 False」。
        self.assertTrue(
            self.node.get_parameter("place_confirm_required").value,
            "测试前提: 代码默认值必须是 true, 现场才由 yaml 关掉",
        )
        self.assertFalse(self.node._gripper_ready, "本类覆盖的是夹爪不由节点接管那条路")
        self._set(place_confirm_required=False)

    def _past_dwell(self):
        """把「进 PLACE_WAIT 的那一刻」往前挪满 place_descend_dwell。

        挪的是 _state_enter_time (进状态时打的抵达时间戳), **不是** _prompt_time:
        后者被 _transition 清零, 用它当起算点会让停留时间失效 —— 那正是本类
        test_dwell_gates_the_release 要挡住的回归。
        """
        self._age(
            "_state_enter_time",
            float(self.node.get_parameter("place_descend_dwell").value) + 0.01,
        )

    def test_dwell_gates_the_release(self):
        """到达 ≠ 立刻松手: 要先停满 place_descend_dwell, 别在还在晃的时候放。

        这个停留时间是从旧的「盲降 dwell」继承来的 (那时等的是「下降指令发出去之后
        机械臂走完」), 现在等的是「判到位之后机械臂真的停稳」。
        """
        self._feedback(self.place)
        self.node._tick_place_wait()
        self.assertEqual(self.node._state, self.node._STATE_PLACE_WAIT,
                         "刚进 PLACE_WAIT 就松手说明这个停留没生效")

    def test_finishes_without_a_confirmation(self):
        self._past_dwell()
        self._feedback(self.place)
        self.node._tick_place_wait()
        self.assertEqual(self.node._state, self.node._STATE_IDLE)

    def test_returns_to_standby_without_a_confirmation(self):
        self._enable_return()
        self._past_dwell()
        self.node._tick_place_wait()
        self.assertEqual(self.node._state, self.node._STATE_HOME)
        self.assertEqual(len(self.node._waypoints), 1, "回位恒为一条绝对待机位")
        np.testing.assert_allclose(self.node._waypoints[0], self.node._standby_position)

    def test_enter_is_not_required_but_still_harmless(self):
        """自动推进不依赖 Enter, 但此时误按一下也不该把状态机带歪。"""
        self._past_dwell()
        self._feedback(self.place)
        self.node._enter_pressed = True
        self.node._tick_place_wait()
        self.assertEqual(self.node._state, self.node._STATE_IDLE)
        self.assertFalse(self.node._enter_pressed, "确认用过就要清掉")


class PlaceSafetyTest(_ExecutorTestCase):
    """放置阶段的失败处置: 能回位就回位, 别把机械臂连同物体丢在原地。"""

    def setUp(self):
        super().setUp()
        self._set(dry_run=False)
        self._enable_return()
        self._enable_place()

    def test_all_place_states_return_to_standby(self):
        for name in PLACE_STATES:
            with self.subTest(state=name):
                self.node._state = getattr(self.node, name)
                self.node._return_pending = True
                self.assertTrue(
                    self.node._should_return_to_standby(),
                    f"{name} 期间失败也该回位",
                )

    def test_failure_during_place_move_returns_home(self):
        self._enter_lift_done()
        self.node._tick_motion()
        self.assertEqual(self.node._state, self.node._STATE_PLACE_MOVE)
        self._break_feedback()
        self.node._tick_motion()
        self.assertEqual(self.node._state, self.node._STATE_HOME)

    def test_place_point_out_of_workspace_fails_and_returns_home(self):
        """加载期校验过了, 但**运行期**工作空间被改窄到放不下这个点。

        workspace_* 是每次现读的参数 (`ros2 param set` 能边跑边改), 所以 _enter_place
        到点前再查一次不是多余的 —— 查的是夹爪点与上报原点两个 (两者差 |offset|)。
        """
        self._set(workspace_y_min=-0.10)               # 放置点 y = -0.30, 现在在界外
        self._enter_lift_done()
        self.node._tick_motion()
        self.assertEqual(len(self.sent), 0, "越界就不该发任何放置指令")
        self.assertEqual(self.node._state, self.node._STATE_HOME)


class IkRejectionTest(_ExecutorTestCase):
    """固件 IK 拒收 -> 当场判失败 (2026-09-21)。

    真机 10:17 那次放置失败: move_p 的 3 条 CAN 帧都发成功了, MCU 当场回了
    `IK 状态变化: status=2` 把目标**拒收**, 机械臂一步没走; 而执行节点只看得见
    「没动」, 于是等满 motion_timeout(30s) 再报一个 413mm 的偏差。现场从日志里
    分不出这是「被挡住推不动」还是「IK 无解」—— 两者要采取的动作完全不同。

    判据是**两条独立证据同时成立**: ik_status 非 0 **且** 机械臂从指令发出到现在
    一步没动。少任何一条都不判 —— 前者会拿「粘在心跳里的上一次判决」误伤新指令,
    后者会把「被挡住推不动」抢过来 (那是 stall / motion_timeout 的活)。

    真机 10:48 那次是本组用例的来由: 桥接日志写着 `status=2` 而执行节点照样等满了
    30s —— 因为当时判据还要求 ik_status_valid=1, 而那个字段被 SDK 漏拷, 恒为 0。

    判出拒收之后**先原样重发一次** (额度每条指令一次), 重发还被拒才判失败: 判决是逐次
    的, 同样的数常常第二次就被收下 (真机 10:16 被拒、10:17 一个数不改重发, 4.8s 到位)。
    """

    def setUp(self):
        super().setUp()
        # 与 PlaceSafetyTest 同一套前提: 真机那条运动逻辑 + 失败后能回位 + 有真放置点。
        self._set(dry_run=False)
        self._enable_return()
        self._enable_place()

    def _grace(self):
        return float(self.node.get_parameter("ik_verdict_grace").value)

    def _reject(self, position=LIFT_START, **ik):
        """喂一帧「手臂没动 + 固件回了拒收判决」的反馈 (真机 10:48 快照的样子)。

        position 默认就是 `_enter_place_phase` 发指令那一刻的位置, 所以这是「一步没动」。
        `ik_status_valid` 保持替身默认的 0 —— 真机上它**永远是 0** (SDK 漏拷字段),
        用例就得照着真机来, 不许再拿 valid=1 当前提。
        """
        fields = {"ik_status": 2}
        fields.update(ik)
        self._feedback(np.asarray(position, dtype=float), **fields)
        # 判决要过 ik_verdict_grace 才作数 (MCU 收齐 3 帧位姿再判决), 这里不做真
        # sleep, 按文件里既有的做法把计时起点往前挪。
        self._age("_ik_watch_since", self._grace() + 0.1)

    # ── 快速失败: 状态机 ──────────────────────────────────────

    def test_rejection_during_place_move_resends_then_fails_without_timeout(self):
        """放置途中被拒收 -> 先原样重发一次, 再被拒就回位, 全程不等 motion_timeout。

        断言里没有动过任何运动计时 (motion_timeout 默认几十秒) —— 两拍之后能走到 HOME,
        只可能来自 IK 判决那条通道。
        """
        self._enter_place_phase()
        self.assertEqual(self.node._state, self.node._STATE_PLACE_MOVE)
        self._reject()                                  # 手臂停在抬升点: 从没动过
        self.node._tick()
        self.assertEqual(
            self.node._state, self.node._STATE_PLACE_MOVE,
            "第一次被拒应当先原样重发一次, 不是当场判失败",
        )
        self.assertEqual(len(self.sent), 2, "重发了同一条指令")
        self._reject()                                  # 重发之后还是不动、还是拒收
        self.node._tick()
        self.assertEqual(
            self.node._state, self.node._STATE_HOME,
            "重发过还被拒就该判失败并回位, 而不是等满 motion_timeout 报一个大偏差",
        )

    def test_retry_resends_the_identical_command(self):
        """重发的是**同一条**指令 (同一个位置 + 同一个姿态), 不是另算一个目标。

        真机 10:17 那次就是这么救回来的: 待机位那条指令一个数都不改地重发, 4.8s 到位。
        位置比的是 _last_command_position (这条指令真正下发的那个点), 所以这里连姿态
        一起比 —— 姿态要是变了就成了另一条指令, 不再有「同样的数重发」那层证据。
        """
        self._enter_place_phase()
        first_position, first_orientation = self.sent[0]
        self._reject()
        self.node._tick()
        self.assertEqual(len(self.sent), 2)
        position, orientation = self.sent[1]
        np.testing.assert_allclose(position, first_position)
        np.testing.assert_allclose(orientation, first_orientation)
        self.assertFalse(self.node._ik_watch_done, "重发之后要重新立案, 否则第二次拒收没人判")
        self.assertTrue(self.node._ik_retry_used, "额度该用掉了")

    def test_message_says_a_retry_was_already_tried(self):
        """重发过还被拒的判词要写明「已经重发过一次」—— 现场才知道别再原地重试。

        这里直接调 _ik_rejection_problem / _retry_rejected_command 而不是 _tick: 谓词
        取走判词时就会**结案**, 直接调它之后再 _tick 已经没有案子可判了 (这正是
        test_verdict_closes_once_per_command 钉住的语义)。
        """
        self._enter_place_phase()
        self._reject()
        first = self.node._ik_rejection_problem()
        self.assertIsNotNone(first)
        self.assertNotIn("重发过一次", first, "第一次拒收还没重发过, 不该这么写")
        self.assertTrue(self.node._retry_rejected_command(), "第一次拒收应当重发")
        self._reject()
        second = self.node._ik_rejection_problem()
        self.assertIsNotNone(second)
        self.assertIn("重发过一次", second)

    def test_retry_quota_is_per_command(self):
        """额度按指令算: 下一条新指令重新给一次 (否则整趟任务只救得了一次)。"""
        self._enter_place_phase()
        self._reject()
        self.node._tick()
        self.assertTrue(self.node._ik_retry_used)
        self.node._send_move_p(np.asarray(LIFT_START, dtype=float) + 0.01,
                               (0.0, 0.0, 0.0, 1.0))
        self.assertFalse(self.node._ik_retry_used, "新指令要重新给一次额度")

    def test_no_retry_when_the_command_is_unknown(self):
        """不知道刚发的是什么时**不重发** —— 宁可判失败, 也不乱发一条来路不明的指令。"""
        self._enter_place_phase()
        self._reject()
        self.node._last_command_position = None
        self.assertFalse(self.node._retry_rejected_command())

    def test_idle_ignores_a_leftover_verdict(self):
        """IDLE 下没有任务在跑, 手里那条旧判决不该把节点推进 HOME。

        拒收的 ik_status 会一直停在 2 直到下一条指令被判决, 所以「不在任务里」这件事
        必须由状态来挡, 不能指望判决自己消失。
        """
        self.node._transition(self.node._STATE_IDLE)
        self.node._ik_watch_since = self.node.get_clock().now().nanoseconds * 1e-9
        self.node._ik_watch_done = False
        self._reject()
        self.node._tick()
        self.assertEqual(self.node._state, self.node._STATE_IDLE)

    # ── 判据: 三条缺一不可 ────────────────────────────────────

    def test_verdict_names_the_firmware_and_echoes_the_target(self):
        """描述串里要有「拒收」二字、固件的码和它的中文含义, 还要回显**上报系**那 6 个数。

        回显上报系那组 (而不是夹爪系) 是因为参数/yaml/GUI/固件说的都是这 6 个数 ——
        现场照着日志改配置, 抄的必须是同一组。
        中文含义来自固件码表: 只有数字的话, 现场还得回头翻文档才知道 2 是什么毛病。
        """
        self._enter_place_phase()
        self._reject()
        problem = self.node._ik_rejection_problem()
        self.assertIsNotNone(problem)
        self.assertIn("固件 IK 拒收", problem)
        self.assertIn("ik_status=2", problem)
        self.assertIn("超出工作空间", problem)
        self.assertIn("上报原点=", problem)

    def test_unknown_verdict_code_is_still_a_rejection(self):
        """码表里没有的码照样当拒收 —— 固件可能比这份表新。

        不许出现「码表查不到 -> 当成没问题 -> 等满 30s」这种降级: 拒收的判据是
        「非 0」, 码表只负责把原因译成人话。
        """
        self._enter_place_phase()
        self._reject(ik_status=99)
        problem = self.node._ik_rejection_problem()
        self.assertIsNotNone(problem)
        self.assertIn("ik_status=99", problem)
        # 认不出来的码只写数字, 不硬套一个中文名: 后面直接就是右括号。
        self.assertIn("ik_status=99)", problem)
        self.assertIn("上报原点=", problem)

    def test_no_verdict_before_the_grace_window(self):
        """宽限没到只是**继续等**, 不是「没拒收」—— 案件不能提前结掉。

        MCU 要收齐 send_target_pose 的 3 条帧 (帧间 20ms) 才重新判决, 判决再经 CAN
        回来; 等不够就会读到**上一条**指令的判决。所以这里既要返回 None, 又要保住
        _ik_watch_done=False: 提前结案 = 这条指令再也不判了。
        """
        self._enter_place_phase()
        self._feedback(np.asarray(LIFT_START, dtype=float), ik_status=2)
        self.assertIsNone(self.node._ik_rejection_problem())
        self.assertFalse(self.node._ik_watch_done, "宽限没到只是在等, 不该结案")

    def test_verdict_from_before_the_command_is_not_used(self):
        """上一条指令的拒收判决不能算到这一条头上: 快照必须是指令之后发布的。

        真机上这就是最常见的一帧 —— 状态话题一直在发, 拒收值又一直挂着, 只有**时间戳**
        能说明它是哪条指令的判决。
        """
        self._enter_place_phase()
        self._reject()
        stamp, status = self.node._arm_status
        self.node._arm_status = (stamp - 5.0, status)   # 挪到指令之前发布的那一帧
        self.assertIsNone(
            self.node._ik_rejection_problem(),
            "指令之前发布的判决不能用来判这条指令",
        )

    def test_rejection_fires_even_though_valid_is_zero(self):
        """**回归用例**: ik_status_valid=0 也必须能判出拒收 (真机上它永远是 0)。

        2026-09-21 10:48 真机: 桥接日志白纸黑字写着 `IK 状态变化: status=2`, 机械臂
        一步没动, 而执行节点老老实实等满了 motion_timeout 才报「运动超时 偏差 206mm」。
        原因就是当时的判据要求 `ik_status_valid == 1`, 而 SDK 的 get_state() 建
        ArmState 副本时把 valid 漏掉了 (sdk_linux.py:816) —— 那个条件永远不成立,
        整条快速失败通道是死代码。

        区分「固件说收下了」和「固件没上报过」不能靠 valid: 前者是 verdict==0,
        后者真机上表现为**一直**是 0, 而「有没有拒收」由 verdict 非 0 加「一步没动」
        共同决定。这个字段以后 SDK 修好了再说, 判据里不许依赖它。
        """
        self._enter_place_phase()
        self._reject(ik_status_valid=0)          # 真机的样子: 永远是 0
        problem = self.node._ik_rejection_problem()
        self.assertIsNotNone(
            problem, "valid=0 是真机的常态, 拿它当门槛 = 快速失败永远不会触发"
        )
        self.assertIn("ik_status=2", problem)

    # ── 判据 4: 「一步没动」这条独立证据 ──────────────────────

    def test_verdict_is_ignored_once_the_arm_has_moved(self):
        """机械臂动过 -> 固件是收下了的, 挂在心跳里的非 0 值属于更早的某条指令。

        这是「粘住不散」最要命的一步: 拒收之后 ik_status 会一直停在 2, 而 0x5A9
        是 ~50Hz 周期心跳, 时间戳永远是新的 —— 只看「状态快照比指令新」的话,
        下一条指令发出后一瞬间就会被上一次的判决误伤, 明明在动却判它拒收。
        """
        self._enter_place_phase()
        self._reject(position=np.asarray(LIFT_START, dtype=float) + 0.01)
        self.assertIsNone(
            self.node._ik_rejection_problem(),
            "动过就说明目标被收下了, 不许拿上一条指令的拒收值判这一条",
        )
        self.assertTrue(self.node._ik_watch_done, "有结论就该结案, 别每拍重算")

    def test_no_tcp_feedback_means_no_verdict(self):
        """取不到 TCP 反馈 -> 「动没动」这条证据不成立 -> 不判, 留给 timeout 兜底。

        宁可慢 30s 报超时, 也不许在证据不足时编一个「固件拒收」出来 —— 那个结论会
        把现场引向「去改位姿」, 而真正的问题可能是没反馈。
        """
        self._enter_place_phase()
        self._reject()
        self.node._tcp_pose = None
        self.assertIsNone(self.node._ik_rejection_problem())
        self.assertFalse(self.node._ik_watch_done, "没判就不算结案")

    def test_command_that_asks_for_no_movement_is_not_judged(self):
        """指令本来就只要求原地微调时, 「没动」说明不了任何问题 -> 不判。

        这种指令 (目标与当前位置之差不足 stall_detect_epsilon) 走的是「到位」那条路,
        它一动都不动是**正常**的; 拿它当拒收的证据, 现场就会看到一条微调指令莫名
        被判「固件 IK 拒收」。所以要求位移本身够大, 这条证据才作数。
        """
        self._enter_place_phase()
        watch = np.asarray(self.node._ik_watch_pose, dtype=float)
        # 判据读的是 _last_command_position (这条指令真正下发的点, 不会跨段过期)
        self.node._last_command_position = watch + 0.0002   # 远小于 epsilon(0.0005)
        self._reject()
        self.assertIsNone(self.node._ik_rejection_problem())
        self.assertFalse(self.node._ik_watch_done, "没判就不算结案")

    def test_accepted_verdict_is_not_a_problem(self):
        """固件说收下了 (status=0) 就不是问题 —— 这时一动不动才是「被挡住」，
        该由 stall/motion_timeout 那条路去管, 两条通道不许互相抢。"""
        self._enter_place_phase()
        self._reject(ik_status=0)
        self.assertIsNone(self.node._ik_rejection_problem())

    def test_switch_off_keeps_the_old_wait_for_timeout_behaviour(self):
        """ik_verdict_enabled=false -> 老行为一字不变 (等超时兜底)。"""
        self._set(ik_verdict_enabled=False)
        self._enter_place_phase()
        self._reject()
        self.assertIsNone(self.node._ik_rejection_problem())

    def test_verdict_closes_once_per_command(self):
        """一条指令只结一次案。

        拒收是**逐条指令**的判决, 而 ik_status 会一直停在 2 直到下一条被判决 ——
        不结案的话, 后续每条新指令发出后的一瞬间都会被这条旧判决误伤。结案本身也要
        留痕 (_ik_watch_done=True), 否则「判过了」和「还没判」在下一拍又分不出来。
        """
        self._enter_place_phase()
        self._reject()
        self.assertIsNotNone(self.node._ik_rejection_problem())
        self.assertTrue(self.node._ik_watch_done)
        self.assertIsNone(self.node._ik_rejection_problem(), "同一条指令不该重复判")

    def test_new_command_reopens_the_case(self):
        """新指令重新立案: 拒收是逐条判决的, 上一条的结论不能顺延。"""
        self._enter_place_phase()
        self.assertFalse(self.node._ik_watch_done, "指令刚下发就该处于待判")
        self._reject()
        self.node._ik_rejection_problem()
        self.assertTrue(self.node._ik_watch_done)
        self._set(dry_run=False)
        self.node._send_move_p(np.asarray(LIFT_START, dtype=float) + 0.01,
                               (0.0, 0.0, 0.0, 1.0))
        self.assertFalse(self.node._ik_watch_done, "新指令要重新立案")


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
        for key in NEW_PARAM_KEYS:
            with self.subTest(param=key):
                self.assertIn(key, self.yaml_params, f"{key} 没写进 yaml")
                default = self.node.get_parameter(key).value
                actual = self.yaml_params[key]
                self.assertIs(type(actual), type(default), f"{key} 类型不一致")
                self.assertEqual(actual, default, f"{key} 两边取值不一致")

    def test_pose_parameter_exists_in_yaml(self):
        """place_pose_reported_mm_deg 不进一致性名单 (现场实测值), 但必须真的写在 yaml 里,
        而且是**上报末端原点系**那 6 个数 (与 standby_pose_reported_mm_deg 同一约定)。"""
        self.assertIn("place_pose_reported_mm_deg", self.yaml_params)
        # 2026-09-18 起是 6 个数; 3 个数是旧约定 (夹爪系/米), 出现在 yaml 里就是没改干净
        self.assertNotIn("place_pose_base_m", self.yaml_params, "旧的 3 参数写法没删掉")
        values = self.yaml_params["place_pose_reported_mm_deg"]
        self.assertEqual(len(values), 6)
        for value in values:
            self.assertIsInstance(value, float, "6 个数都要写成浮点, 否则解析不出数组类型")

    def test_every_yaml_sequence_is_type_homogeneous(self):
        """手改位姿最容易踩的坑: 整数混进 double 序列 —— 节点**连起都起不来**。

        2026-09-18 17:20 现场: 把放置点改现场实测值时写成
        `place_pose_reported_mm_deg: [222, -290.0, 258.1, 84.4, 0.2, 30.0]`, 那个 222
        被 YAML 读成**整数**, 而 ROS 2 的参数文件解析器要求整个序列同类型, 于是

            ros2 launch ... -> rclpy.init()
            RCLError: Couldn't parse params file: Error: Sequence should be of same type.
                      Value type 'double' do not belong at line_num 92

        注意这是**解析期**的错, 不是「值填错了」——节点根本没构造出来, 加载期的那些
        校验 (离原点太近、数量不对、按米填) 一条都没机会跑, 所以它们拦不住这一类。
        下面这条统一钉住 yaml 里**每个**数组: 要么全是浮点, 要么全是整数, 不许混。
        """
        for key, value in self.yaml_params.items():
            if not isinstance(value, list) or not value:
                continue
            with self.subTest(param=key):
                kinds = {type(item) for item in value}
                self.assertEqual(
                    len(kinds), 1,
                    f"{key} 里混了类型 {sorted(kind.__name__ for kind in kinds)}"
                    " —— 整数混进浮点序列会让节点在解析参数阶段就起不来, 一律写成 x.0",
                )

    def test_yaml_pose_is_in_the_same_frame_as_standby(self):
        """两个位姿参数的**数量与量纲**必须一致 (都是 mm + 度), 否则就是又抄串了。

        待机位的 X 是 100mm 量级, 放置点的 X 也是同一量级 —— 一个按米填 (0.13) 会
        差两个数量级。这条只钉「同一个约定」, 不钉具体值。
        """
        standby = np.asarray(
            self.yaml_params["standby_pose_reported_mm_deg"], dtype=float
        )
        place = np.asarray(self.yaml_params["place_pose_reported_mm_deg"], dtype=float)
        self.assertEqual(len(standby), len(place), "两个位姿参数的个数必须一样 (6)")
        for value in place[:3]:
            self.assertGreater(
                abs(value), 1.0,
                "位置像是按**米**填的 (mm 参数应为几十~几百), 与待机位不是同一约定",
            )

    def test_enabling_place_requires_a_real_pose(self):
        """开着放置就必须有一个真位姿 —— 占位值会让 _on_execute 拒绝启动。

        这条不变量把「改了开关忘了填位姿」钉在 yaml 上: 两者必须一起改。
        """
        if not self.yaml_params["place_enabled"]:
            return
        reported = np.asarray(
            self.yaml_params["place_pose_reported_mm_deg"][:3], dtype=float
        ) * 0.001
        self.assertGreater(
            float(np.linalg.norm(reported)), 0.05,
            "place_enabled=true 但 place_pose_reported_mm_deg 还是占位值 —— "
            "任务会被拒绝启动",
        )

    def test_gripper_offset_default_matches_yaml(self):
        """夹爪偏移**以 yaml 为准** (2026-09-18 定): 代码默认值必须与它同值。

        它不跟别的键一样可以两边不同 —— 那不是部署开关, 也不是「保守值 vs 现场值」的
        选择, 而是一条**物理事实** (上报原点离夹爪多远, 实测出来的)。ros2 run 不带
        params-file 时少这 6cm, 真机上就是整体偏 6cm, 没有"保守一点"的中间态。
        yaml 那边也写了同一句 (config 里 gripper_offset_tool_xyz 那段)。
        """
        self.assertIn("gripper_offset_tool_xyz", self.yaml_params)
        self.assertEqual(
            self.node.get_parameter("gripper_offset_tool_xyz").value,
            self.yaml_params["gripper_offset_tool_xyz"],
            "代码默认值与 yaml 不一致 —— 夹爪偏移以 yaml 为准, 改一边就要改另一边",
        )

    def test_place_dwell_covers_the_place_leg(self):
        """place_descend_dwell 是「进 PLACE_WAIT 之后再停多久才松手」, 下限是**走完时间**。

        约束来自 README_参数配置.md §7: `place_descend_dwell ≥ 该段下降的实际走完时间`
        —— 短了会在机械臂还在走的时候张开。而「走完了没有」这件事**不归它管**:
        PLACE_MOVE 是判到位 (position_tolerance) + 停稳 (settle_time) 才进 PLACE_WAIT 的,
        所以进到这里机械臂已经停住了, 这个 dwell 纯属再给一点余量。

        2026-09-21 之前这里写的是 `> settle_time * 5.0` (=1.5s, 现场 0.5s → 假红)。
        那条 5× 经验是从**盲降**段借来的 —— 抓取下降只看时间不看反馈, 才必须自己
        把整段走完时间包进去 (见 descend_dwell)。放置这段前面已经有到位+停稳两道关,
        照搬 5× 是过度套用。0.5 是现场验证过的值, 保持它, 把判据换成本来的约束。
        """
        dwell = self.yaml_params["place_descend_dwell"]
        self.assertGreater(dwell, 0.0, "0 就是「判到位立刻松手」, 没有任何余量")
        self.assertGreaterEqual(
            dwell,
            self.yaml_params["settle_time"],
            "比一个 settle_time 还短, 说明这点余量小到没有意义 —— 现场值是 0.5s",
        )
        self.assertLess(dwell, 3.0, "大到这个量级先怀疑是拿错了参数 (盲降的 dwell)")


class MoveModeRoutingTest(_ExecutorTestCase):
    """哪一段走 MOVE_L (直线)、哪一段走 MOVE_P, 以及被拒之后按**原模式**重发。

    历史: 2026-09-24 DESCEND / LIFT 一起改走 /control/move_l (现场反馈 move_p 的下降
    路径不是直线); 2026-09-28 LIFT **退回 move_p** —— 抬升被固件的 L 规划器反复拒收
    (`ik_status=5 关节单步过大`), 详见节点里 _LINEAR_MOTION_STATES 上那段注释。
    这条契约只有**一个**开关: _LINEAR_MOTION_STATES。所以下面第一件事就是钉住
    「名单里恰好是这几段」, 而不是逐个调用点去查 —— 调用点都走 _send_move_p 的默认
    分派, 谁都不许写死模式。
    """

    # 每个状态该不该走直线。这张表**故意与代码分写一份** (同 place_pose 的做法):
    # 复用 _LINEAR_MOTION_STATES 就等于没测。
    EXPECTED_LINEAR = {
        "_STATE_DESCEND": True,
        "_STATE_LIFT": False,
        "_STATE_PRE_GRASP": False,
        "_STATE_HOME": False,
        "_STATE_PLACE_MOVE": False,
        "_STATE_TRACK_WAIT": False,
        "_STATE_WAIT_TARGET": False,
        "_STATE_IDLE": False,
    }

    POSE = np.array([0.35, 0.0, 0.30])
    ORIENTATION = (0.0, 1.0, 0.0, 0.0)

    def _send_in_state(self, state_name, **kwargs):
        """在指定状态下真发一条指令 (dry_run=False, 走真正的 publish 分支)。

        **显式关掉求逆解那条路** (2026-09-28): 这个类是「笛卡尔两条话题怎么分派」的
        契约, 加进 J 之后它必须先被排除掉。别指望「测试里没有关节反馈, IK 自然回落
        到旧路」那个巧合 —— 哪天有人给用例补上一帧反馈, 这里就会静默变成在测 J。
        J 那条路有自己的用例类 (IkMotionTest)。
        """
        self._set(dry_run=False, ik_motion_enabled=False)
        self.node._state = getattr(self.node, state_name)
        self.node._move_p_pub.published.clear()
        self.node._move_l_pub.published.clear()
        self.node._move_j_pub.published.clear()
        self.node._send_move_p(self.POSE, self.ORIENTATION, **kwargs)
        return (
            len(self.node._move_p_pub.published),
            len(self.node._move_l_pub.published),
            len(self.node._move_j_pub.published),
        )

    def test_the_linear_set_is_exactly_descend(self):
        # 名单里放的是状态**取值** ("DESCEND"), 上面那张表用的是属性名, 换算一下再比
        expected = tuple(
            getattr(self.node, name)
            for name, linear in self.EXPECTED_LINEAR.items()
            if linear
        )
        self.assertEqual(
            tuple(self.node._LINEAR_MOTION_STATES),
            expected,
            "线性段名单变了 —— 改它之前先确认现场确实要那一段走直线",
        )

    def test_every_state_routes_to_the_expected_topic(self):
        for name, expect_linear in self.EXPECTED_LINEAR.items():
            with self.subTest(state=name):
                move_p, move_l, move_j = self._send_in_state(name)
                if expect_linear:
                    self.assertEqual((move_p, move_l, move_j), (0, 1, 0), f"{name} 该走 move_l")
                else:
                    self.assertEqual((move_p, move_l, move_j), (1, 0, 0), f"{name} 该走 move_p")
                # 记在案的模式必须与真发出去的一致 —— 重发全靠这个字段
                self.assertEqual(self.node._last_command_linear, expect_linear)
                self.assertIsNone(
                    self.node._last_command_joint, "关掉求逆解时不该留下关节目标"
                )

    def test_retry_keeps_the_mode_of_the_rejected_command(self):
        """被拒的是 L 指令, 重发就必须还是 L。

        重发的时刻状态可能已经推进 (判决在 ik_verdict_grace 之后才结案), 所以这里特意
        把状态改成一个走 P 的, 再调重发 —— 查状态的写法会退化成 move_p, 那正是要防的。
        """
        self._send_in_state("_STATE_DESCEND")  # _last_command_linear = True
        self.node._sent_orientation = self.ORIENTATION
        self.node._ik_retry_used = False
        self.node._state = self.node._STATE_HOME  # 此刻状态是「走 P」的
        self.node._move_p_pub.published.clear()
        self.node._move_l_pub.published.clear()

        self.assertTrue(self.node._retry_rejected_command(), "该重发却没重发")
        self.assertEqual(
            (len(self.node._move_p_pub.published), len(self.node._move_l_pub.published)),
            (0, 1),
            "重发改了模式 —— 下降中途从直线折成点到点, 夹爪可能斜着扎到瓶子上",
        )

    def test_retry_keeps_p_when_the_rejected_command_was_p(self):
        """反向也要成立, 免得把上一条的 True 漏成默认值。"""
        self._send_in_state("_STATE_PRE_GRASP")  # _last_command_linear = False
        self.node._sent_orientation = self.ORIENTATION
        self.node._ik_retry_used = False
        self.node._state = self.node._STATE_DESCEND  # 此刻状态是「走 L」的
        self.node._move_p_pub.published.clear()
        self.node._move_l_pub.published.clear()

        self.assertTrue(self.node._retry_rejected_command(), "该重发却没重发")
        self.assertEqual(
            (len(self.node._move_p_pub.published), len(self.node._move_l_pub.published)),
            (1, 0),
            "P 指令被 P 重发才对; 跟着当前状态走会莫名变成直线",
        )

    def test_move_l_topic_has_its_own_publisher(self):
        """两个话题必须是两条独立发布器 —— 共用一个的话模式就只能靠消息内容区分了。"""
        self.assertIsNot(self.node._move_p_pub, self.node._move_l_pub)
        self.assertEqual(
            self.node.get_parameter("move_l_topic").value, "/control/move_l"
        )

    # ── 关节空间那条路 (2026-09-28) ──

    # 哪些状态**优先**走 move_j。与上面那张表一样, 故意与代码分写一份。
    EXPECTED_JOINT = {
        "_STATE_PRE_GRASP": True,
        "_STATE_TRACK_WAIT": True,
        "_STATE_DESCEND": True,
        "_STATE_LIFT": True,
        "_STATE_PLACE_MOVE": True,
        "_STATE_HOME": True,
        "_STATE_WAIT_TARGET": False,
        "_STATE_IDLE": False,
    }

    def test_the_joint_set_is_exactly_the_motion_states(self):
        """六个运动段一个不缺 —— 少一个就意味着那一段还在走固件的 IK。"""
        expected = tuple(
            getattr(self.node, name)
            for name, joint in self.EXPECTED_JOINT.items()
            if joint
        )
        self.assertEqual(
            tuple(self.node._IK_MOTION_STATES), expected,
            "走 move_j 的名单变了 —— 改它之前先确认现场确实要那一段绕开固件 IK",
        )

    def test_the_two_lists_mean_different_things(self):
        """两张表**不互斥**: 一张是「优先走 J」, 一张是「回退成笛卡尔时哪段走 L」。

        DESCEND 同时在两张表里是**对的** —— 优先走 J, 走不成时仍回退到直线。
        这个用例钉住这条语义, 免得将来有人把它当 bug「修」成互斥。
        """
        self.assertIn(self.node._STATE_DESCEND, self.node._IK_MOTION_STATES)
        self.assertIn(self.node._STATE_DESCEND, self.node._LINEAR_MOTION_STATES)
        self.assertEqual(
            self.node.get_parameter("move_j_topic").value, "/control/move_j"
        )
        self.assertIsNot(self.node._move_j_pub, self.node._move_p_pub)
        self.assertIsNot(self.node._move_j_pub, self.node._move_l_pub)


class IkMotionTest(_ExecutorTestCase):
    """上位机求逆解 -> /control/move_j 那条路 (2026-09-28)。

    单独一个类, 因为_MoveModeRoutingTest 是**关掉它**测笛卡尔分派的 (见那边的注释)。
    这里注入一帧关节反馈当种子, 用**真实的求逆解**跑 —— vendored 的 ik_h3001 +
    URDF + CAD CSV 都在包里, 所以这些用例是真在解, 不是桩。
    """

    # 待机位 (上报原点 mm/deg) 与它的 URDF 关节解 (实测):
    #   q_URDF = [1.14, 38.4, -12.31, 3.05, -31.72, -1.21], 最小余量 12.3°
    # 反馈 (话题约定) = sign × q_URDF。
    STAND_RPY = (15.4, 84.2, 15.0)
    STAND_XYZ = np.array([0.150, 0.0, 0.244])

    def _feed_joints(self, topic_deg):
        """注入一帧关节反馈 (话题约定, 度)。"""
        self.node._joint_deg = (
            self.node.get_clock().now().nanoseconds * 1e-9,
            np.asarray(topic_deg, dtype=float),
        )

    def _feed_stand_pose_joints(self):
        urdf = np.array([1.14, 38.4, -12.31, 3.05, -31.72, -1.21])
        sign = np.asarray(self.node.get_parameter("ik_joint_sign").value, dtype=float)
        self._feed_joints(sign * urdf)

    def _orientation(self):
        return self.node._rpy_to_quaternion(*np.radians(self.STAND_RPY))

    def _send(self, position=None, orientation=None):
        self.node._move_p_pub.published.clear()
        self.node._move_l_pub.published.clear()
        self.node._move_j_pub.published.clear()
        self.node._send_move_p(
            self.STAND_XYZ if position is None else position,
            self._orientation() if orientation is None else orientation,
        )

    def test_a_solvable_target_goes_out_as_move_j(self):
        """核心契约: 解得出 -> 发 /control/move_j, 而且是 JointState (不是 PoseStamped)。"""
        self._set(dry_run=False)
        self._feed_stand_pose_joints()
        self.node._state = self.node._STATE_PRE_GRASP
        self._send()
        self.assertEqual(len(self.node._move_j_pub.published), 1)
        self.assertEqual(len(self.node._move_p_pub.published), 0)
        self.assertEqual(len(self.node._move_l_pub.published), 0)
        message = self.node._move_j_pub.published[0]
        # 消息类型从 PoseStamped 变成 JointState —— 现有断言一条都抓不住,
        # 所以这里逐项核: 名字 (桥按名字查表, 缺一个就 raise)、弧度、有限性。
        self.assertEqual(list(message.name), [f"joint{i}" for i in range(1, 7)])
        self.assertEqual(len(message.position), 6)
        self.assertTrue(np.all(np.isfinite(message.position)))
        # 发出去的必须是**话题约定**的弧度: 与种子的差应该很小 (目标就在当前位姿上,
        # 所以解出来的构型与当前构型基本一致)。
        got_deg = np.degrees(np.asarray(message.position))
        self.assertLess(float(np.abs(got_deg - self.node._joint_deg[1]).max()), 5.0)
        # 记账: 这一条是关节指令
        self.assertIsNotNone(self.node._last_command_joint)
        np.testing.assert_allclose(
            np.asarray(self.node._last_command_joint), got_deg, atol=1e-6
        )

    def test_no_joint_feedback_falls_back_to_move_p(self):
        """没有种子 (没订阅到关节反馈) -> 回落笛卡尔那条路, 不是失败。"""
        self._set(dry_run=False)
        self.node._joint_deg = None
        self.node._state = self.node._STATE_PRE_GRASP
        self._send()
        self.assertEqual(len(self.node._move_j_pub.published), 0)
        self.assertEqual(len(self.node._move_p_pub.published), 1)
        self.assertIsNone(self.node._last_command_joint)

    def test_an_unreachable_target_falls_back_to_move_p(self):
        """够不到的位姿 -> 回落 (而不是把一条解不出来的关节指令发出去)。"""
        self._set(dry_run=False)
        self._feed_stand_pose_joints()
        self.node._state = self.node._STATE_PRE_GRASP
        self._send(position=np.array([0.150, 0.0, 2.500]))   # 2.5m 高, 远超工作空间
        self.assertEqual(len(self.node._move_j_pub.published), 0)
        self.assertEqual(len(self.node._move_p_pub.published), 1)

    def test_disabled_switch_keeps_the_old_route(self):
        self._set(dry_run=False, ik_motion_enabled=False)
        self._feed_stand_pose_joints()
        self.node._state = self.node._STATE_PRE_GRASP
        self._send()
        self.assertEqual(len(self.node._move_j_pub.published), 0)
        self.assertEqual(len(self.node._move_p_pub.published), 1)

    def test_blind_home_never_uses_move_j(self):
        """盲走回位必须走 move_p: 它的前提就是反馈已不可信, 而 IK 的种子正是反馈。

        (两者出自桥的同一次发布 —— 桥挂了两个一起没。拿不可信的东西当种子是最危险的。)
        """
        self._set(dry_run=False)
        self._feed_stand_pose_joints()
        self.node._state = self.node._STATE_HOME
        self.node._home_blind = True
        self._send()
        self.assertEqual(len(self.node._move_j_pub.published), 0)
        self.assertEqual(len(self.node._move_p_pub.published), 1)

    def test_descend_still_falls_back_to_move_l_not_move_p(self):
        """回退时用的是那一段**原本**的模式: DESCEND 回退走 L, 不是 P。"""
        self._set(dry_run=False)
        self.node._joint_deg = None            # 逼它回落
        self.node._state = self.node._STATE_DESCEND
        self._send()
        self.assertEqual(len(self.node._move_l_pub.published), 1)
        self.assertEqual(len(self.node._move_p_pub.published), 0)
        self.assertTrue(self.node._last_command_linear)

    def test_dry_run_sends_nothing(self):
        """dry_run 一条都不发 —— 但它**真的跑一遍 IK** (日志里能看到关节角)。"""
        self._set(dry_run=True)
        self._feed_stand_pose_joints()
        self.node._state = self.node._STATE_PRE_GRASP
        self._send()
        self.assertEqual(len(self.node._move_j_pub.published), 0)
        self.assertEqual(len(self.node._move_p_pub.published), 0)
        self.assertEqual(len(self.node._move_l_pub.published), 0)

    def test_a_retry_never_goes_out_as_move_j(self):
        """重发不走 J: 被拒之后要换的是下发方式, 原样重发同样的关节角没有意义。"""
        self._set(dry_run=False)
        self._feed_stand_pose_joints()
        self.node._state = self.node._STATE_PRE_GRASP
        self._send()
        self.assertEqual(len(self.node._move_j_pub.published), 1)
        self.node._move_j_pub.published.clear()
        self.node._move_p_pub.published.clear()
        self.node._send_move_p(
            self.STAND_XYZ, self._orientation(), is_retry=True, linear=False
        )
        self.assertEqual(len(self.node._move_j_pub.published), 0)
        self.assertEqual(len(self.node._move_p_pub.published), 1)

    def test_the_jump_gate_rejects_a_configuration_change(self):
        """换构型判据: 把「当前构型」喂成一个和目标解差很远的构型 -> 必须回落。

        这一条是三道守门里最救命的一道 —— 换个构型的解**收敛、误差 0、余量也够**,
        只有「与种子的单轴差」抓得到它。
        """
        self._set(dry_run=False)
        # 一个与待机解差 ~90° 的构型当种子 (J2 从 38.4° 挪到 -51.6° 的量级)
        self._feed_joints([1.14, 51.6, -12.31, 3.05, 31.72, -1.21])
        self.node._state = self.node._STATE_PRE_GRASP
        self._send()
        self.assertEqual(
            len(self.node._move_j_pub.published), 0,
            "离种子那么远的解不该被当成「同一条构型」发出去",
        )
        self.assertEqual(len(self.node._move_p_pub.published), 1)


if __name__ == "__main__":
    unittest.main()
