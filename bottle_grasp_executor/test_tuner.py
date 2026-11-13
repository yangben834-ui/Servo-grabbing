#!/usr/bin/env python3
"""h3001_tuner.py 的回归测试 —— **只测纯逻辑层**(换算/校验/逐行改写)。

为什么值得测
--------------
这个界面唯一的破坏性动作是"保存", 而它要保护的恰恰是那个 yaml 里最不能丢的东西:
580 行注释("每条参数为什么是这个值")。所以这里盯的不是界面好不好看, 是四件事:
  - **只换值**: 改完只有目标行变, 其它行逐字节不动、行数不变;
  - **幂等**: 什么都不改直接保存, 文件逐字节相同;
  - **坏值不落盘**: 校验不过就一个字节都不写, 写坏了能从备份回来;
  - **换算与节点同式**: RPY/四元数往返、位姿 -> 夹爪点, 拿现场那几组数钉住。

不构造 `Tk()`(无头环境没有 $DISPLAY 会 TclError), 不 import rclpy(界面本身也不依赖它)。
要跑界面本身请手工 `python3 h3001_tuner.py`。

跑法 (在包根目录):
  source /opt/ros/humble/setup.bash && source install/setup.bash
  /usr/bin/python3 -m unittest -v test_tuner
"""

import math
import os
import shutil
import sys
import tempfile
import unittest

import numpy as np
import yaml

PKG_ROOT = os.path.dirname(os.path.abspath(__file__))
WORKSPACE = os.path.dirname(PKG_ROOT)
sys.path.insert(0, WORKSPACE)                       # h3001_tuner.py 在工作区根, 不在包内

import h3001_tuner as tuner                          # noqa: E402

REAL_YAML = os.path.join(PKG_ROOT, "config", "bottle_grasp_executor.yaml")

# 现场实测值(2026-09-29 13:14 那个真机节点日志里印出来的), 拿它们钉住换算:
#   「待机位: 上报原点=[0.15 0. 0.244] m, 夹爪=[0.2495 -0.0008 0.2537] m, 偏移模长 100.0 mm」
# 夹爪偏移当时是 [0,0,0.08], 日志里那条是 80mm; 现在 yaml 是 [0,0,0.1] -> 100mm。
STANDBY_VEC6 = [150.0, 0.0, 244.0, 15.4, 84.2, 15.0]
STANDBY_GRIPPER = np.array([0.2495, -0.0008, 0.2537])
# 位置抄的现场 yaml (160, -200, 300), 姿态是现场那组 —— 下面只拿它当"合法的放置位姿"用。
PLACE_VEC6 = [160.0, -200.0, 300.0, -76.4, 86.2, -132.0]
SIDE_QUAT = [0.031741, 0.768151, 0.002256, 0.639477]
OFFSET_TOOL = [0.0, 0.0, 0.1]

# 原始几何 (2026-09-29 现场的 standby/place 两组姿态): 都贴着万向锁 —— 这正是
# "四元数才是权威输入、RPY 只读" 这条设计的依据。
SITE_POSES_RPY = [(15.4, 84.2, 15.0), (-76.4, 86.2, -132.0)]


def editable_defaults() -> dict:
    """给"改这些键"用的合法值(校验能过的那种)。"""
    return {
        "pre_grasp_offset_z": 0.12,
        "side_orientation_xyzw": [0.0, 0.7071068, 0.0, 0.7071068],
        "grasp_offset_z": -0.01,
    }


class LineEditTest(unittest.TestCase):
    """逐行改写: 只动值, 别的一个字节都不碰。"""

    @classmethod
    def setUpClass(cls):
        with open(REAL_YAML, "r", encoding="utf-8", newline="") as handle:
            cls.lines = handle.read().splitlines(keepends=True)

    def test_only_the_target_lines_change(self):
        """**核心用例**: 改三个键, 其它行逐字节相同、行数不变、其它键的值不变。"""
        before_params = yaml.safe_load("".join(self.lines))[tuner.ROOT_KEY][tuner.PARAMS_KEY]
        changes = editable_defaults()
        new_lines, diffs, missing = tuner.build_edit(self.lines, changes)

        self.assertEqual(missing, [], "这些键本来就该在 yaml 里")
        self.assertEqual(sorted(d.key for d in diffs), sorted(changes), "三个都该改")
        self.assertEqual(len(new_lines), len(self.lines), "只换值不该改变行数")

        touched = {diff.line_no - 1 for diff in diffs}
        for index, (old, new) in enumerate(zip(self.lines, new_lines)):
            if index in touched:
                continue
            self.assertEqual(old, new, f"第 {index + 1} 行不该变")

        after_params = yaml.safe_load("".join(new_lines))[tuner.ROOT_KEY][tuner.PARAMS_KEY]
        for key, value in before_params.items():
            if key in changes:
                continue
            self.assertEqual(after_params[key], value, f"{key} 不该变")
        for key, value in changes.items():
            self.assertEqual(after_params[key], value, f"{key} 没写对")
        # 注释与空行的行数不变(它们是被 protect 的那些行里的绝大多数)
        self.assertEqual(sum(1 for line in self.lines if line.lstrip().startswith("#")),
                         sum(1 for line in new_lines if line.lstrip().startswith("#")))

    def test_saving_unchanged_values_is_byte_identical(self):
        """幂等: 值没变就一行都不该动(连 `0.150` 归一成 `0.15` 这种也不许)。"""
        params = yaml.safe_load("".join(self.lines))[tuner.ROOT_KEY][tuner.PARAMS_KEY]
        values = {key: params[key] for key in tuner.PARAM_BY_KEY if key in params}
        new_lines, diffs, missing = tuner.build_edit(self.lines, values)
        self.assertEqual(diffs, [])
        self.assertEqual(missing, [])
        self.assertEqual("".join(new_lines), "".join(self.lines))

    def test_prefix_spacing_is_preserved(self):
        """`key:` 后面原来是几个空格就留几个 —— 现场那份里 top_orientation_xyzw 就是两个空格。"""
        line = "    top_orientation_xyzw:  [0.0, 0.0, 0.0, 1.0]\n"
        self.assertEqual(tuner.render_line(line, "[1.0, 0.0, 0.0, 0.0]"),
                         "    top_orientation_xyzw:  [1.0, 0.0, 0.0, 0.0]\n")

    def test_inline_comment_and_ending_are_preserved(self):
        """将来某行要是带了行内注释 / CRLF, 改值也不许把它们弄丢。"""
        line = "    dry_run: true  # 真机跑之前记得关\r\n"
        self.assertEqual(tuner.render_line(line, "false"),
                         "    dry_run: false  # 真机跑之前记得关\r\n")
        self.assertEqual(tuner.captured_value(line, "dry_run"), True)

    def test_missing_key_is_reported_not_invented(self):
        """yaml 里没有这个键: 报告出来, **不代写**(这个仓库的注释是手写的, 不许自动生成)。"""
        lines = [line for line in self.lines
                 if not line.startswith("    top_skip_track:")]
        self.assertEqual(len(lines), len(self.lines) - 1)
        new_lines, diffs, missing = tuner.build_edit(
            lines, {**editable_defaults(), "top_skip_track": False})
        self.assertEqual(missing, ["top_skip_track"])
        self.assertEqual(sorted(d.key for d in diffs), sorted(editable_defaults()),
                         "该写的照写, 缺的那个只是被跳过")
        self.assertEqual(len(new_lines), len(lines))
        self.assertNotIn("    top_skip_track:", "".join(new_lines))

    def test_duplicate_key_is_refused(self):
        """同一个键出现两次: 拒绝改(猜错了就是改错行)。"""
        lines = list(self.lines) + ["    dry_run: false\n"]
        with self.assertRaises(ValueError):
            tuner.build_edit(lines, {"dry_run": True})

    def test_similar_key_names_do_not_collide(self):
        """`track_enabled` 不许匹配到 `track_enabled_x` 那种同前缀的键。"""
        lines = ["bottle_grasp_executor:\n", "  ros__parameters:\n",
                 "    track_enabled_x: false\n", "    track_enabled: true\n"]
        block = tuner.find_block(lines)
        self.assertEqual(tuner.find_key_line(lines, block, "track_enabled"), 3)
        self.assertIsNone(tuner.find_key_line(lines, block, "track_enab"))

    def test_key_outside_the_block_is_ignored(self):
        """块外(别的节点段 / 注释里)的同名键不算数。"""
        lines = ["other_node:\n", "  ros__parameters:\n", "    dry_run: true\n",
                 "bottle_grasp_executor:\n", "  ros__parameters:\n",
                 "    # dry_run: true\n", "    dry_run: false\n"]
        block = tuner.find_block(lines)
        self.assertEqual(tuner.find_key_line(lines, block, "dry_run"), 6)


class FormatTest(unittest.TestCase):
    """写回文件用的文本格式 —— 这里每一条都对应一次"节点起不来"。"""

    def test_floats_never_use_exponent_form(self):
        """`5e-05` 那种指数形式必须写全: PyYAML 认, rcl 的解析器没有保证。"""
        for value in (0.0005, 5e-05, 0.00005, 1e-7, 300.0, 0.0, -0.01, 1234567.5):
            text = tuner.fmt_float(value)
            self.assertNotIn("e", text, f"{value} 写成了指数形式: {text}")
            self.assertIn(".", text, f"{value} 看起来不像浮点: {text}")
            self.assertEqual(float(text), value)

    def test_sequence_elements_stay_floats(self):
        """数组元素必须写成 `300.0` 不能是 `300`: 类型不一致会让**节点直接起不来**。"""
        param = tuner.PARAM_BY_KEY["standby_pose_reported_mm_deg"]
        text = tuner.format_value(param, [300, 0, 250, 90, 90, 180])
        self.assertEqual(text, "[300.0, 0.0, 250.0, 90.0, 90.0, 180.0]")
        parsed = yaml.safe_load(f"k: {text}")["k"]
        self.assertTrue(all(isinstance(item, float) for item in parsed))

    def test_bool_and_int_text(self):
        self.assertEqual(tuner.format_value(tuner.PARAM_BY_KEY["dry_run"], True), "true")
        self.assertEqual(tuner.format_value(tuner.PARAM_BY_KEY["dry_run"], False), "false")
        self.assertEqual(tuner.format_value(tuner.PARAM_BY_KEY["min_stable_samples"], 6), "6")
        self.assertEqual(tuner.format_value(tuner.PARAM_BY_KEY["orientation_source"], "fixed"),
                         "fixed")

    def test_display_matches_write_format(self):
        """输入框里显示的与写进文件的是同一套格式 —— 否则会出现「看着 0.5, 存进去 0.500…1」。"""
        self.assertEqual(tuner.format_display(0.15), "0.15")
        self.assertEqual(tuner.format_display(6), "6")
        self.assertEqual(tuner.format_display(True), "true")


class MathTest(unittest.TestCase):
    """换算与节点/桥同式。这几组数是现场真机上读出来的, 拿它们钉住。"""

    def test_rpy_quaternion_roundtrip_is_exact_for_site_poses(self):
        """现场两组姿态 RPY -> q -> RPY -> q: **四元数**必须逐位一致(旋转不变)。"""
        for rpy in SITE_POSES_RPY:
            first = tuner.rpy_to_quaternion(*rpy)
            back = tuner.quaternion_to_rpy_zyx(first)
            second = tuner.rpy_to_quaternion(*back)
            np.testing.assert_allclose(first, second, atol=1e-12,
                                       err_msg=f"{rpy} 往返之后旋转变了")

    def test_site_poses_sit_near_gimbal_lock(self):
        """**这条是"RPY 只读"那条设计的护栏**: 谁把姿态的 RPY 框改成双向绑定, 这里会红。"""
        for rpy in SITE_POSES_RPY:
            margin = 90.0 - abs(rpy[1])
            self.assertLess(margin, 15.0,
                            f"{rpy} 的万向锁余量 {margin}° —— 现场那几组就是贴着奇异的")

    def test_side_quaternion_decomposition(self):
        """现场 side 姿态: 反解出来是 166.4/79.2/164.0, 离万向锁 10.8°。

        也就是说 —— 这四个小数背后其实是**绕 Y 轴转约 100°**, 不是"三轴各转一点"。
        界面上把那行 RPY 标成只读, 依据就是这条。
        """
        rpy = tuner.quaternion_to_rpy_zyx(SIDE_QUAT)
        for got, want in zip(rpy, (166.4, 79.2, 164.0)):
            self.assertAlmostEqual(got, want, places=1)
        self.assertAlmostEqual(tuner.gimbal_lock_margin_deg(SIDE_QUAT), 10.8, places=1)

    def test_pose_preview_matches_the_live_machine_log(self):
        """位姿换算: 与 2026-09-29 13:14 真机日志里那行「夹爪目标」逐位相同。"""
        preview = tuner.pose_preview(STANDBY_VEC6, OFFSET_TOOL)
        np.testing.assert_allclose(preview["gripper"], STANDBY_GRIPPER, atol=5e-5)
        self.assertAlmostEqual(preview["offset_norm"], 0.1, places=9)

    def test_pose_report_line_keeps_the_node_format(self):
        """预览那行要能直接和节点启动日志对照, 所以格式必须一样。

        numpy 的 array2string 对每个分量按最长的那一个对齐(现场日志里
        `[0.15  0.    0.244]` 就是两个空格), 所以这里只钉结构不钉空格。
        """
        text = tuner.format_pose_report("待机位", STANDBY_VEC6, OFFSET_TOOL)
        self.assertIn("上报原点=[0.15", text)
        self.assertIn("0.244] m (与 GUI/参数一致才是对的)", text)
        self.assertIn("(模长 100.0 mm)", text)
        self.assertIn("姿态=[+0.01184482, +0.67167384, +0.00691461, +0.74072002]", text)

    def test_derived_travels(self):
        """两条下降行程, 以及 top <= 0 时沿用通用值这件事。"""
        travel = tuner.derived_travels({"pre_grasp_offset_z": 0.15, "grasp_offset_z": 0.0,
                                        "top_pre_grasp_offset_z": 0.01})
        self.assertAlmostEqual(travel["side_travel"], 0.15)
        self.assertAlmostEqual(travel["top_travel"], 0.01)
        self.assertFalse(travel["top_uses_general"])

        travel = tuner.derived_travels({"pre_grasp_offset_z": 0.15, "grasp_offset_z": -0.01,
                                        "top_pre_grasp_offset_z": -1.0})
        self.assertAlmostEqual(travel["side_travel"], 0.16)
        self.assertAlmostEqual(travel["top_travel"], 0.16, msg="top 没设时该沿用通用值")
        self.assertTrue(travel["top_uses_general"])


class CheckTest(unittest.TestCase):
    """校验。前四条跨字段不变量照搬 test_track_wait 的 ConfigConsistencyTest。"""

    def base(self) -> dict:
        """一份能过校验的底稿 (从真 yaml 抄, 免得与现场值漂移)。"""
        return {
            "pre_grasp_offset_z": 0.15, "top_pre_grasp_offset_z": 0.01,
            "grasp_offset_z": 0.0, "descend_dwell": 2.0,
            "track_min_command_interval": 1.0, "max_result_age": 4.0,
            "track_deadband": 0.010, "track_max_lag": 0.050,
            "track_settle_time": 0.30, "track_stable_epsilon": 0.0005,
            "auto_grasp_stable_time": 2.0, "auto_grasp_stable_epsilon": 0.010,
            "settle_time": 0.30, "stall_detect_time": 1.0, "ik_verdict_grace": 0.8,
            "gripper_hold_torque": 1.0, "gripper_hard_torque_limit": 2.0,
            "gripper_resistance_torque": 1.0, "gripper_timeout": 40.0,
            "gripper_script_timeout": 25.0, "place_enabled": True,
            "standby_pose_reported_mm_deg": STANDBY_VEC6,
            "place_pose_reported_mm_deg": PLACE_VEC6,
            "side_orientation_xyzw": SIDE_QUAT, "top_orientation_xyzw": SIDE_QUAT,
            "gripper_offset_tool_xyz": OFFSET_TOOL,
            "workspace_x_min": -1.9, "workspace_x_max": 1.9,
            "workspace_y_min": -1.9, "workspace_y_max": 1.9,
            "workspace_z_min": -1.1, "workspace_z_max": 1.9,
            "top_skip_track": True, "top_skip_lift": True,
        }

    def errors(self, values: dict) -> list:
        return [p for p in tuner.validate(values) if p.level == "error"]

    def test_clean_draft_has_no_errors(self):
        self.assertEqual(self.errors(self.base()), [])

    def test_staleness_must_exceed_the_cadence(self):
        values = self.base()
        values["max_result_age"] = 1.0                 # == 跟随周期
        self.assertTrue(any(p.key == "max_result_age" for p in self.errors(values)))

    def test_deadband_must_be_smaller_than_max_lag(self):
        values = self.base()
        values["track_deadband"] = 0.060               # > track_max_lag
        self.assertTrue(any(p.key == "track_deadband" for p in self.errors(values)))

    def test_auto_grasp_epsilon_window(self):
        values = self.base()
        values["auto_grasp_stable_epsilon"] = 0.002    # 比实测下限 5mm 还细 -> 一次都不触发
        self.assertTrue(any(p.key == "auto_grasp_stable_epsilon"
                            for p in self.errors(values)))
        values["auto_grasp_stable_epsilon"] = 0.020    # 比死区还粗
        self.assertTrue(any(p.key == "auto_grasp_stable_epsilon"
                            for p in self.errors(values)))

    def test_gripper_hold_must_stay_under_the_hard_limit(self):
        values = self.base()
        values["gripper_hold_torque"] = 2.5
        self.assertTrue(any(p.key == "gripper_hold_torque" for p in self.errors(values)))

    def test_gripper_timeout_must_exceed_the_script_timeout(self):
        values = self.base()
        values["gripper_timeout"] = 20.0
        self.assertTrue(any(p.key == "gripper_timeout" for p in self.errors(values)))

    def test_place_enabled_needs_a_real_pose(self):
        values = self.base()
        values["place_pose_reported_mm_deg"] = [0.0] * 6
        problems = self.errors(values)
        self.assertTrue(any("离基座原点" in p.text for p in problems))

    def test_near_origin_is_error_for_place_but_only_warn_for_standby(self):
        """**两处不对称是刻意的**(节点里就是这样): 待机位只 warn, 放置位直接拒绝。"""
        warn_only = self.base()
        warn_only["standby_pose_reported_mm_deg"] = [1.0, 0.0, 2.0, 0.0, 0.0, 0.0]
        problems = tuner.validate(warn_only)
        near = [p for p in problems if "离基座原点" in p.text and "待机" in p.text]
        self.assertTrue(near)
        self.assertEqual([p.level for p in near], ["warn"])
        self.assertFalse(any(p.level == "error" and "离基座原点" in p.text
                             and "待机" in p.text for p in problems))

        blocked = self.base()
        blocked["place_pose_reported_mm_deg"] = [1.0, 0.0, 2.0, 0.0, 0.0, 0.0]
        self.assertTrue(any(p.level == "error" and "离基座原点" in p.text
                            for p in tuner.validate(blocked)))

    def test_pose_outside_the_workspace_is_rejected(self):
        values = self.base()
        # y = 2500mm = 2.5m, 超出 workspace_y_max (现场 1.9)
        values["place_pose_reported_mm_deg"] = [160.0, 2500.0, 300.0, -76.4, 86.2, -132.0]
        self.assertTrue(any("WORKSPACE_REJECTED" in p.text for p in self.errors(values)))
        # 夹爪点与上报原点**都要**在界内: 一个在界内另一个仍可能越界
        values["place_pose_reported_mm_deg"] = [160.0, 1850.0, 300.0, -76.4, 86.2, -132.0]
        self.assertEqual(self.errors(values), [], "1.85m 还在界内(上限 1.9)")

    def test_meter_filled_pose_hits_the_near_origin_rule(self):
        """按米填 (0.19 而不是 190) —— 实机真出过这个错, 而且工作空间的 z 下限拦不住它。"""
        values = self.base()
        values["place_pose_reported_mm_deg"] = [0.19, -0.3, 0.25, 0.0, 0.0, 0.0]
        self.assertTrue(any("离基座原点" in p.text for p in self.errors(values)))

    def test_gimbal_lock_warning_does_not_block_saving(self):
        values = self.base()                            # 现场那两组姿态本来就贴万向锁
        problems = tuner.validate(values)
        self.assertTrue(any("万向锁" in p.text for p in problems))
        self.assertEqual(self.errors(values), [], "贴万向锁只该提示, 不该拦保存")

    def test_travel_speed_warning_fires_for_the_unverified_combo(self):
        """15cm / 2.0s = 7.5cm/s, 比唯一实测过的 4.5cm/s 快 —— 必须提示(这是对的)。"""
        problems = tuner.validate(self.base())
        self.assertTrue(any("cm/s" in p.text for p in problems))
        slower = self.base()
        slower["descend_dwell"] = 4.0                   # 3.75cm/s < 4.5
        self.assertFalse(any("cm/s" in p.text for p in tuner.validate(slower)))

    def test_bridge_invert_mismatch_is_flagged(self):
        """`ik_target_invert_rx_ry` 必须与桥的 `invert_rx_ry` 同值, 否则 IK 解的是另一个位姿。"""
        values = self.base()
        mismatch = tuner.validate(values, bridge_invert=False, node_invert=True)
        self.assertTrue(any(p.key == "ik_target_invert_rx_ry" for p in mismatch))

        agreed = tuner.validate(values, bridge_invert=False, node_invert=False)
        self.assertFalse(any(p.key == "ik_target_invert_rx_ry" for p in agreed))

        unknown = tuner.validate(values, bridge_invert=None, node_invert=None)
        self.assertFalse(any(p.key == "ik_target_invert_rx_ry" for p in unknown),
                         "读不到桥的开关时不该瞎报")

    def test_missing_key_is_reported_as_a_warning(self):
        problems = tuner.validate(self.base(), None, ["top_skip_track"])
        self.assertTrue(any(p.level == "warn" and p.key == "top_skip_track" for p in problems))


class WriteTest(unittest.TestCase):
    """落盘: 备份、原子替换、回读校验、指纹。全部在临时目录里跑, 绝不碰真文件。"""

    def setUp(self):
        self.dir = tempfile.mkdtemp(prefix="tuner-test-")
        self.path = os.path.join(self.dir, "params.yaml")
        shutil.copy2(REAL_YAML, self.path)
        with open(self.path, "r", encoding="utf-8", newline="") as handle:
            self.original = handle.read()

    def tearDown(self):
        shutil.rmtree(self.dir, ignore_errors=True)

    def test_write_keeps_everything_but_the_value(self):
        changes = {"pre_grasp_offset_z": 0.123}
        report = tuner.save_params(self.path, changes, tuner.snapshot(self.path))
        self.assertEqual(report.changed, ("pre_grasp_offset_z",))
        self.assertTrue(report.backup_path.exists(), "必须留备份(这个工作区不是 git 仓库)")
        self.assertEqual(report.backup_path.read_text(encoding="utf-8"), self.original)

        before = self.original.splitlines(keepends=True)
        with open(self.path, "r", encoding="utf-8", newline="") as handle:
            after = handle.read().splitlines(keepends=True)
        self.assertEqual(len(before), len(after), "只换值不该改变行数")

        # 改动行由 build_edit 给的行号决定 —— **不能**用「行里有 pre_grasp_offset_z」去找:
        # 那个键名在十几行注释里也出现过。
        _new, diffs, _missing = tuner.build_edit(before, changes)
        touched = {diff.line_no - 1 for diff in diffs}
        self.assertEqual(len(touched), 1)
        for index, (old, new) in enumerate(zip(before, after)):
            if index in touched:
                self.assertIn("0.123", new, "目标行没写上新值")
            else:
                self.assertEqual(old, new, f"第 {index + 1} 行不该变")
        # 文件模式也要留着(新建的临时文件默认 600, 原文件是 644/755)
        self.assertEqual(os.stat(self.path).st_mode & 0o777,
                         os.stat(REAL_YAML).st_mode & 0o777)

    def test_rollback_when_verification_fails(self):
        """回读校验不过 -> 原文件恢复, **备份还在**(它是唯一的证据, 不许被吃掉)。"""
        original_verify = tuner.verify_written
        tuner.verify_written = lambda *a, **k: ["故意让它失败"]
        try:
            with self.assertRaises(ValueError) as caught:
                tuner.save_params(self.path, {"pre_grasp_offset_z": 0.123},
                                  tuner.snapshot(self.path))
        finally:
            tuner.verify_written = original_verify
        self.assertIn("故意让它失败", str(caught.exception))
        with open(self.path, "r", encoding="utf-8", newline="") as handle:
            self.assertEqual(handle.read(), self.original)
        backups = [name for name in os.listdir(self.dir) if ".bak-" in name]
        self.assertEqual(len(backups), 1, "备份不该被回滚吃掉")

    def test_stale_fingerprint_refuses_to_write(self):
        """载入之后文件被别人改过: 拒绝写(否则按旧行号覆盖, 静默吞掉别人的改动)。"""
        before = tuner.snapshot(self.path)
        with open(self.path, "a", encoding="utf-8") as handle:
            handle.write("\n# 别人加的一行\n")
        with open(self.path, "r", encoding="utf-8") as handle:
            modified = handle.read()
        with self.assertRaises(ValueError) as caught:
            tuner.save_params(self.path, {"pre_grasp_offset_z": 0.123}, before)
        self.assertIn("被改动过", str(caught.exception))
        with open(self.path, "r", encoding="utf-8") as handle:
            self.assertEqual(handle.read(), modified, "拒绝写就该一个字节都不动")
        self.assertEqual([n for n in os.listdir(self.dir) if ".bak-" in n], [],
                         "拒绝写的时候连备份都不该产生")

    def test_no_change_writes_nothing(self):
        params = yaml.safe_load(self.original)[tuner.ROOT_KEY][tuner.PARAMS_KEY]
        values = {key: params[key] for key in tuner.PARAM_BY_KEY if key in params}
        report = tuner.save_params(self.path, values, tuner.snapshot(self.path))
        self.assertEqual(report.changed, ())
        self.assertIsNone(report.backup_path)
        with open(self.path, "r", encoding="utf-8", newline="") as handle:
            self.assertEqual(handle.read(), self.original)

    def test_backup_names_do_not_collide(self):
        """同一秒内保存两次不能把上一个备份盖掉。"""
        first = tuner.backup_path(tuner.Path(self.path), now=1700000000.0)
        first.write_text("x", encoding="utf-8")
        second = tuner.backup_path(tuner.Path(self.path), now=1700000000.0)
        self.assertNotEqual(first, second)
        self.assertFalse(second.exists())

    def test_verify_catches_a_changed_comment(self):
        """回读校验必须抓得住「注释被吃掉」—— 只比"值对不对"是拦不住的。"""
        lines = self.original.splitlines(keepends=True)
        broken = [line for line in lines if not line.lstrip().startswith("#")]
        broken_path = os.path.join(self.dir, "broken.yaml")
        with open(broken_path, "w", encoding="utf-8", newline="") as handle:
            handle.write("".join(broken))
        problems = tuner.verify_written(broken_path, {}, lines, [])
        self.assertTrue(any("行数变了" in text for text in problems))


class TableTest(unittest.TestCase):
    """参数表本身与现场的 yaml: 表里写的键必须在文件里都有。"""

    @classmethod
    def setUpClass(cls):
        with open(REAL_YAML, "r", encoding="utf-8") as handle:
            cls.params = yaml.safe_load(handle)[tuner.ROOT_KEY][tuner.PARAMS_KEY]

    def test_every_editable_key_exists_in_the_yaml(self):
        """表里的键 yaml 里都得有 —— 这条同时也是**巡检**: 节点加了新参数而表没跟上时,
        界面会在校验栏里报「yaml 里没有这个键」, 而这里会红。"""
        for param in tuner.PARAMS:
            self.assertIn(param.key, self.params, f"{param.key} 不在 yaml 里")

    def test_editable_values_parse_into_the_declared_kind(self):
        for param in tuner.PARAMS:
            value = self.params[param.key]
            if param.kind == "bool":
                self.assertIsInstance(value, bool, param.key)
            elif param.kind == "choice":
                self.assertIn(value, param.choices, param.key)
            elif param.kind in tuner.VEC_LEN:
                self.assertEqual(len(value), tuner.VEC_LEN[param.kind], param.key)
                self.assertTrue(all(isinstance(v, float) for v in value),
                                f"{param.key} 的数组元素必须是浮点(否则节点起不来)")
            elif param.kind == "int":
                self.assertIsInstance(value, int, param.key)
            else:
                self.assertIsInstance(value, float, param.key)

    def test_every_param_has_a_label_note_and_group(self):
        for param in tuner.PARAMS:
            self.assertTrue(param.label.strip(), param.key)
            self.assertTrue(param.note.strip(), f"{param.key} 缺一句后果说明")
            self.assertIn(param.group, [group for group, _title in tuner.GROUPS], param.key)

    def test_groups_cover_every_param(self):
        for group, _title in tuner.GROUPS:
            self.assertTrue([p for p in tuner.PARAMS if p.group == group],
                            f"{group} 这组是空的")

    def test_real_yaml_still_loads_and_the_node_could_read_it(self):
        """改不了真文件, 但至少确认它现在就是解析得开的(界面写坏的第一道防线在下游测试)。"""
        self.assertIn("place_pose_reported_mm_deg", self.params)
        self.assertTrue(tuner.PARAM_BY_KEY["standby_pose_reported_mm_deg"].reload_required)


if __name__ == "__main__":
    unittest.main()
