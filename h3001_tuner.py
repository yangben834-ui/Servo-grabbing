#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""H3001 抓取参数调参界面 —— 只改 config/bottle_grasp_executor.yaml 里那些常用的值。

用法
------
    # 1) 看一遍、改、保存 (常规用法。**不需要** source ROS)
    /usr/bin/python3 h3001_tuner.py

    # 2) 只读: 载入 + 校验 + 打印"将要写入的改动", 一个字节都不写盘 (先看数对不对)
    /usr/bin/python3 h3001_tuner.py --dry-run

    # 3) 指定别的 yaml (默认就是本工作区的现场配置)
    /usr/bin/python3 h3001_tuner.py --yaml /path/to/bottle_grasp_executor.yaml

从控制台起: `h3001_console.py` 按钮区的「调参」(起的是**子进程**, 见下面"为什么单独一个进程")。

它管什么
----------
把真机上**常动的那些参数**按六组摆出来(位姿 / 抓取姿态 / 高度与行程 / 时序与确认 / 开关 /
夹爪), 每组带实时换算预览与跨字段校验, 保存时**只改值**。界面**不做运行期下发**(不发
ros2 param set): 保存完自己去控制台点「起/停 执行器」重启。这是刻意的 —— 机械臂在跑的时候
点一下"保存"就不该让它动。

**注释比数值重要, 所以绝不重写文件**
--------------------------------------
这个 yaml 里每条参数都带着"为什么是这个值"的来龙去脉, 那是这个仓库最值钱的东西。
`yaml.safe_load` + `yaml.dump` 一趟回来注释、空行、顺序全没了 —— 所以这里**逐行替换**:
定位到 `key:` 那一行, 只换冒号后面的值那一段(连 `key:` 后面原来是几个空格都照原样留着),
行数不变、其它行逐字节不变。写完回读校验, 任一条不满足就从备份回滚。

配套的三条保护:
  - 只写**源码树里**那份 (`bottle_grasp_executor/config/...`)。`install/` 那份是指向它的
    符号链接链 —— 拿 install 路径去 `os.replace` 会把链接本身换成一个普通文件, 把链接链打断,
    而执行器读的还是老内容(现场表现: 改了没生效, 而且没人知道为什么)。
  - 载入时记 `(mtime, size, sha256)`, 保存前重算; 不一致就拒绝写(说明有人在别处改过),
    免得按旧行号覆盖掉别人的改动。
  - 每次都备份 `<yaml>.bak-<时间戳>`, 同目录, 不自动删 —— 这个工作区**不是 git 仓库**,
    这是唯一的安全网。

**为什么单独一个进程**(而不是控制台里开个子窗口)
--------------------------------------------------
控制台的 `RosBridge` 已经在那个进程里 `rclpy.init()` 并且 spin 着, 同进程再 init 一次会冲突;
而且控制台**必须** source 过 ROS 才能起, 那样这个界面就永远拿不到"没 source ROS 也能开"这条。
子进程还有个便宜: 它崩了不会把控制台带下去。
代价是摸不到控制台那个「起/停 执行器」按钮 —— 所以由**控制台**盯着这个文件的 mtime, 变了就
在日志区打一行提醒(见 h3001_console.py 的 _poll_yaml)。

唯一需要 ROS 的功能是「抄当前末端」(读 /feedback/tcp_pose 填位姿): 它懒 import rclpy,
没 source 过 ROS 时那个按钮点了会说明原因, 其余功能一律可用。

换算函数为什么另写一份
----------------------
与 `bottle_grasp_executor/scripts/check_place_pose.py` 里那份同式但**故意各写一份**
(那个文件的注释里写了理由): 拿一条独立路径去核节点算出来的数, 复用它的实现等于让两边
一起错也看不出来。这里同理 —— 而且那个模块顶部 `import rclpy`, 本界面不能依赖它。
参照实现: 节点 `_rpy_to_quaternion`(ZYX 前向)、`h3001_bridge.py` 的 `_pose_to_list`
(ZYX 反解; `invert_rx_ry` 现场 false)。
"""

from __future__ import annotations

import argparse
import hashlib
import math
import os
import re
import shutil
import sys
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Iterable, Sequence

import numpy as np
import yaml

import tkinter as tk
from tkinter import messagebox, ttk

# ── 路径 ────────────────────────────────────────────────────
# 工作区就是本文件所在目录 (与 h3001_console.py 同一个约定)。
WORKSPACE = Path(__file__).resolve().parent
SOURCE_YAML = (WORKSPACE / "bottle_grasp_executor" / "config"
               / "bottle_grasp_executor.yaml")
INSTALL_YAML = (WORKSPACE / "install" / "bottle_grasp_executor" / "share"
                / "bottle_grasp_executor" / "config" / "bottle_grasp_executor.yaml")
BRIDGE_YAML = WORKSPACE / "h3001_bridge" / "config" / "h3001_bridge.yaml"

ROOT_KEY = "bottle_grasp_executor"          # yaml 顶层键 = 节点名
PARAMS_KEY = "ros__parameters"              # 参数所在的第二层
TCP_TIMEOUT_S = 2.0                         # 读 /feedback/tcp_pose 的等待上限 (s)


# ── 参数表 ──────────────────────────────────────────────────

@dataclass(frozen=True)
class Param:
    """一个可编辑参数的全部元信息(界面上的一切都从这张表长出来)。

    kind 决定用哪种控件、怎么写回 yaml:
      float/int      -> 单个输入框
      bool           -> Checkbutton (`true` / `false`)
      vec3/vec6/quat -> N 个输入框 (写成 `[a, b, c]`)
      choice         -> 下拉
    reload_required 只用来**提示**(位姿那几项是加载期读一次, 改完必须重启执行器);
    用户选的是"只写 yaml", 所以它不参与保存逻辑。
    """

    key: str
    label: str
    group: str
    kind: str = "float"
    unit: str = ""
    lo: float | None = None
    hi: float | None = None
    choices: tuple[str, ...] = ()
    note: str = ""              # 一句后果, 显示在底栏 (点/悬停到控件上时)
    reload_required: bool = False


GROUPS: tuple[tuple[str, str], ...] = (
    ("pose", "① 位姿 (上报系 mm+度)"),
    ("orient", "② 抓取姿态"),
    ("height", "③ 高度与行程"),
    ("timing", "④ 时序与确认"),
    ("switch", "⑤ 开关"),
    ("gripper", "⑥ 夹爪"),
)


def _params() -> tuple[Param, ...]:
    """这张表就是界面的全部内容。note 抄自 yaml 里那段注释的**结论**, 别写成长文 ——
    底栏只放得下两三行。"""
    pose = [
        Param("standby_pose_reported_mm_deg", "起始/待机位姿", "pose", "vec6",
              reload_required=True,
              note="[X,Y,Z(mm), RX,RY,RZ(度)], **上报末端原点系** —— 就是 /feedback/tcp_pose "
                   "和厂商 GUI 上那 6 个数。回位段的目标。填错(比如按米填)会让回位功能被"
                   "禁用, 抓取不受影响。"),
        Param("place_pose_reported_mm_deg", "放置位姿", "pose", "vec6",
              reload_required=True,
              note="与待机位同一套约定 (mm + 度)。放置段**不沿用抓取姿态**, 全程用这里的 "
                   "RX/RY/RZ, 所以这组数要挑一个机械臂解得出来、够得到的朝向。"
                   "填错会让放置功能被禁用、任务被拒绝启动。"),
        Param("gripper_offset_tool_xyz", "夹爪偏移 (tool 系)", "pose", "vec3", unit="m",
              reload_required=True,
              note="tool 系下「上报原点 -> 夹爪」的平移, 模长就是上面预览里那个 mm。"
                   "这是**物理事实不是部署开关**(代码默认值与 yaml 必须同值): 改它等于把"
                   "**所有**目标整体平移这么多 —— 差一点也可能抓得起来, 所以"
                   "「抓起来了」不等于这个值是对的。有疑问重新标定。"),
    ]
    orient = [
        Param("orientation_source", "姿态来源", "orient", "choice",
              choices=("fixed", "current_tcp"),
              note="fixed = 用下面两个四元数; current_tcp = 保持触发瞬间的 TCP 姿态。"
                   "现场是 fixed(side 姿态已从实机读出, 结果可复现)。"),
        Param("side_orientation_xyzw", "side 抓取姿态 (四元数)", "orient", "quat",
              note="从**侧面**抓用的 TCP 姿态, 2026-09-14 由实机读出(机械臂静止, 407 帧零漂移)。"
                   "四元数是**唯一权威输入** —— 下面那行 RPY 是只读的: 这组姿态离万向锁很近, "
                   "反解出来的角度不是人输的那组。要按角度改, 用右边那个「从 RPY 填入」。"),
        Param("top_orientation_xyzw", "top 抓取姿态 (四元数)", "orient", "quat",
              note="倒瓶(躺瓶)从**正上方**抓用的姿态。**这是抄自松灵 piper 的值, 对 H3001 的"
                   "夹爪/法兰约定几乎肯定不对, 用 top 模式前必须按 H3001 重新标定。**"
                   "现场它与 side 同值, 属于占位。"),
    ]
    height = [
        Param("pre_grasp_offset_z", "side 悬停高度", "height", "float", unit="m",
              note="预抓取点(悬停点) = 抓取中心 + 这个高度; 它**同时就是 side 那一段的下降行程**"
                   "(行程 = 它 - grasp_offset_z)。改它等于同时改悬停高度与下降距离 —— "
                   "要跟着核 descend_dwell, 否则会在半空闭合夹爪。"),
        Param("top_pre_grasp_offset_z", "top 悬停高度", "height", "float", unit="m",
              note="<= 0 = 不单独设, 沿用上面那个。倒瓶那一模式是工具朝下抓, 末端原点在夹爪"
                   "**上方一整个工具偏移**, 所以悬停抬高 1cm 法兰就高 1cm: 2026-09-28 实测悬停 "
                   "0.20 时法兰报到 531.9mm, 固件判不可达, 于是压到 1cm。"
                   "**它同时就是 top 那一段的下降行程。**"),
        Param("grasp_offset_z", "抓取点偏移", "height", "float", unit="m",
              note="最终抓取点 = 感知给的抓取中心 + 这个值。想抓瓶子上部就给正值把目标点抬高; "
                   "想只改落点而不动悬停高度, 改这里(例如 -0.01)。"),
        Param("lift_distance", "抬升高度", "height", "float", unit="m",
              note="抓完之后往上抬多少。top + top_skip_lift 时这一段整个跳过(夹爪本来就在"
                   "物体正上方)。"),
        Param("descend_dwell", "下降 dwell", "height", "float", unit="s",
              note="向下发出**一条**下降指令后等多久才闭合夹爪。**这不是到位判据** —— "
                   "到点就走, 与机械臂实际走到哪无关。取值只要不短于那段行程走完的时间; "
                   "半空闭合夹爪(夹个空)就是这里给少了。"),
    ]
    timing = [
        Param("max_result_age", "候选过期阈值", "timing", "float", unit="s",
              note="候选比这还旧就当没有候选: WAIT_TARGET 判新鲜度, TRACK_WAIT 判感知是不是停了"
                   "(停了就判失败回待机位)。**必须大于跟随周期**, 否则还没读到下一次坐标就先判失败。"),
        Param("track_min_command_interval", "跟随周期", "timing", "float", unit="s",
              note="每 N 秒读一次最新坐标。这是**离散跟随**不是连续伺服: 每一跳都把悬停点更新到"
                   "目标现在在哪。调小跟得更勤, 代价是启停更频繁、更容易抖。"),
        Param("track_deadband", "跟随死区", "timing", "float", unit="m",
              note="目标相对**上一条已下发的悬停指令**移动超过它才重发。基准刻意不是机械臂实际"
                   "位置 —— 这台机末端有固定重力下垂, 用实际位置当基准会一直够不到、一直追。"),
        Param("track_settle_time", "跟随停稳时间", "timing", "float", unit="s",
              note="「上一条跟随指令走完了」的判据(位置变化 < track_stable_epsilon 连续保持这么久)。"
                   "周期(1~2s)远比任何一次几厘米横移长, 所以这道门正常每拍都放行。"),
        Param("track_stable_epsilon", "跟随停稳容差", "timing", "float", unit="m",
              note="判「机械臂停稳」的位置变化阈值。**别拿它当「物体没动」的判据** —— "
                   "静止物体的坐标峰峰值就有 3~4mm, 那个判据是 auto_grasp_stable_epsilon。"),
        Param("track_max_lag", "跟随落后上限", "timing", "float", unit="m",
              note="目标比机械臂跑得快时的例外通道: 落后超过它就不等停稳直接重发(跟得上比不抖动"
                   "更重要)。它量的是「机械臂距**上一条指令**还有多远」, 不是距新目标 —— "
                   "后者里含固定下垂, 会让这条通道常开、等于取消停稳门。"),
        Param("auto_grasp_stable_time", "自动下降稳定时间", "timing", "float", unit="s",
              note="机械臂到位后物体位置连续这么久没变就**自动下降**(无人值守的最后一次按 Enter "
                   "就省掉了)。0 = 关掉, 退回等人按 Enter / 发 grasp_confirm。"
                   "计时的起点是「机械臂到位」而不是「物体停下」。"),
        Param("auto_grasp_stable_epsilon", "自动下降容差", "timing", "float", unit="m",
              note="判「物体位置没变」的容差。**不能调小**: 实测静止瓶子的 position_base 峰峰值 "
                   "3.1~4.2mm, 滚动 3s 窗口在 eps=2mm 时通过率 0/73, 到 5mm 才 73/73。"
                   "也不该比跟随死区还粗(那等于承认「已经动到不想跟了还算没动」)。"),
        Param("stable_duration", "目标稳定窗口", "timing", "float", unit="s",
              note="WAIT_TARGET 判「目标稳了, 可以出发」的窗口长度(与最少样本数一起看)。"),
        Param("min_stable_samples", "稳定窗口最少样本", "timing", "int",
              note="窗口内至少要有这么多个样本才判稳定。"),
        Param("target_stability_tolerance", "目标稳定容差", "timing", "float", unit="m",
              note="稳定窗口内所有样本相对当前位置的最大偏差上限(超过就重新等)。"
                   "与自动下降容差同值同义, 一个是「进 PRE_GRASP 之前稳不稳」, "
                   "一个是「进 DESCEND 之前还在不在原地」。"),
        Param("position_tolerance", "到位容差", "timing", "float", unit="m",
              note="PRE_GRASP / LIFT / PLACE_MOVE / HOME 判「到位」的位置误差上限。"
                   "回位用下面那个更松的 standby_tolerance。"),
        Param("settle_time", "到位停稳时间", "timing", "float", unit="s",
              note="进了容差还要连续保持这么久才算到位(防「路过」被当成到达)。"),
        Param("standby_tolerance", "回位到位容差", "timing", "float", unit="m",
              note="比 position_tolerance 松: 回位只要求「离开工作区、停在待机位附近」, "
                   "不是抓取精度, 而重力下垂在不同姿态下实测 3~55mm。"
                   "卡在 5mm 会每次任务都白等一个运动超时再报失败。"),
        Param("place_descend_dwell", "放置 dwell", "timing", "float", unit="s",
              note="抵达放置点(**判到位**)之后再停多久才放下物体。名字是旧流程(盲降 dwell)留下的, "
                   "现在等的是「机械臂真的停稳」—— 别在还在晃的时候松手。"),
        Param("ik_verdict_grace", "IK 判决宽限", "timing", "float", unit="s",
              note="指令发出后等多久再读固件 IK 判决。**别低于 0.5**: MCU 要收齐 3 帧位姿"
                   "(帧间 20ms)才重新判决, 等不够会读到**上一条**指令的判决, 把好指令误判成拒收。"),
        Param("stall_detect_time", "停稳判定时间", "timing", "float", unit="s",
              note="迭代末端修正用的停稳判据(位置连续这么久变化不超过 stall_detect_epsilon)。"
                   "**必须明显长于 settle_time 与 track_settle_time**, 否则会在机械臂还在走时"
                   "误判停稳、把残差当成「够不到的偏差」反复修正。"),
        Param("motion_timeout", "运动超时", "timing", "float", unit="s",
              note="PRE_GRASP / LIFT / PLACE_MOVE / HOME 每个路点的超时预算。"
                   "**HOME 段真正用的不是这个数, 是它 xN**(N≤4, 由 max_command_distance 与"
                   "夹爪偏移算出来) —— 回位那段路比一个路点长得多。"),
    ]
    switch = [
        Param("dry_run", "模拟执行", "switch", "bool",
              note="只打印目标位姿、不发任何机械臂命令, 可在无机械臂时验证全流程。"
                   "**真机跑之前记得关。**"),
        Param("track_enabled", "悬停跟随", "switch", "bool",
              note="关掉 = 跳过整个 TRACK_WAIT: 不跟随、也不要那唯一一次确认, 悬停到位直接下降"
                   "(整条链变成零确认)。新环节有问题先关它验证其余部分, 不用回滚代码。"),
        Param("top_skip_track", "倒瓶不跟踪", "switch", "bool",
              note="只对 top(倒瓶)模式生效: 那一模式从正上方抓, 机械臂一到瓶子正上方相机就看不着"
                   "它, 跟随必然判跟丢回待机位 —— 开着它则悬停到位直接下降, 下降点用锁定目标那一刻"
                   "的坐标。side 不受影响。代价: top 这一轮落点无人确认。"),
        Param("top_skip_lift", "倒瓶不抬升", "switch", "bool",
              note="只对 top 模式生效: 抓完不抬升直接去放置(夹爪本来就停在物体正上方)。"
                   "**依赖放置段能走** —— 没配放置点时节点会打一行 warn 并仍然抬升。"),
        Param("place_enabled", "放置", "switch", "bool",
              note="抓完是否先放到放置位姿再回待机位。开着就必须有个有效的放置位姿, 否则 "
                   "execute_grasp 直接拒绝启动(不会抓完才发现无处可放)。"
                   "**它同时是循环模式唯一的硬闸** —— 关着循环开不起来。"),
        Param("grasp_confirm_required", "夹取要确认", "switch", "bool",
              note="true = GRASP_WAIT 停下等人按 Enter 才闭合夹爪(「手还伸在夹爪里也来得及退开」"
                   "只在 true 时成立)。false = dwell 一到就自动闭合 —— 而下降**不判是否到位**, "
                   "所以闭合时刻完全由 descend_dwell 决定。"),
        Param("place_confirm_required", "放置要确认", "switch", "bool",
              note="true = PLACE_WAIT 停下等人确认物体确实放下了。false = 停满 place_descend_dwell "
                   "就自动张开。放得太早/放歪先调大那个 dwell, 而不是改回 true。"),
        Param("return_to_standby_on_finish", "结束回待机位", "switch", "bool",
              note="任务结束(成功或失败)后回待机位再停。关掉则机械臂留在原地。"),
        Param("return_to_standby_on_error", "出错也回待机位", "switch", "bool",
              note="机械臂报故障/反馈断了时**也**回位(盲走)。**不建议关**: 2026-09-16 真机上正是"
                   "这条把机械臂丢在抓取点上方了 —— 回位是一条绝对指令, 不依赖反馈。"),
        Param("endpoint_correction_enabled", "末端迭代修正", "switch", "bool",
              note="补这台机的固定重力下垂(E2PROM 重力参数六轴全 0, 末端够不到目标, 偏差随姿态 "
                   "3~55mm)。关掉就每次固定差那么多。治本是找厂商要 J1~J6 的 m/Gx/Gy/Gz。"),
        Param("ik_motion_enabled", "上位机求逆解", "switch", "bool",
              note="运动指令**优先**走关节空间(节点自己求逆解 -> /control/move_j)。出问题的第一件"
                   "事就是把它关掉: 全部退回 move_p/move_l, 与 2026-09-28 之前逐字相同。"
                   "(加载期校验不过时节点会自己关。)"),
        Param("ik_verdict_enabled", "固件拒收快判", "switch", "bool",
              note="「命令发出去了、机械臂一动不动」时用固件回的 ik_status 当场判失败, "
                   "而不是等满 motion_timeout 报一个巨大的偏差。没什么理由关。"),
        Param("gripper_enabled", "节点接管夹爪", "switch", "bool",
              note="每轮先张开、GRASP_WAIT 的确认变成「现在夹」、PLACE_WAIT 变成「现在放」。"
                   "**只在加载期读一次, 改它要重启节点**。关掉则夹爪由人工在另一个终端操作。"),
    ]
    gripper = [
        Param("gripper_velocity", "开合速度", "gripper", "float", unit="rad/s",
              note="张开/闭合共用的速度。取自原先人工跑的那两条命令的 --velocity 2。"),
        Param("gripper_resistance_torque", "阻力阈值", "gripper", "float", unit="N·m",
              note="闭合中力矩超过「空载力矩 + 这个值」就停下, 认为夹到东西了。"
                   "**这是「闭合到哪里停」的判据, 不是成功判据** —— 够不到就一路合到底。"),
        Param("gripper_hard_torque_limit", "力矩硬上限", "gripper", "float", unit="N·m",
              note="任何时候力矩碰到它就立即停止并判失败(防夹坏物体/防堵转)。"
                   "**不要**为了「夹得更紧」调大 —— 抬起期间夹多紧由保持力矩决定。"
                   "必须大于保持力矩(节点加载期会拦)。"),
        Param("gripper_hold_torque", "保持力矩", "gripper", "float", unit="N·m",
              note="0 = 关(停下就失能, 搬运期间全靠机构自锁)。> 0 = 闭合后**常驻发帧**顶着这个力矩"
                   "直到放置(固件有看门狗, 不发帧就切电, 所以力只能由主机持续发帧维持)。"
                   "现场 1.0 折算约 55~100 N 指尖力 —— **台架上看到瓶身凹陷就往下调**。"),
        Param("gripper_hold_timeout", "保持最长时间", "gripper", "float", unit="s",
              note="保持的最后兜底, 只在没人叫停也没人管时才生效。到点脚本失能放手并打一行 "
                   "[warn]; 机械臂若还悬空, 物体会掉。给窄了会把「人在 PLACE_WAIT 慢慢确认」截断。"),
        Param("gripper_hold_kd", "保持阻尼", "gripper", "float", unit="N·m·s/rad",
              note="保持帧的 MIT 阻尼项, 范围 [0, 5](12bit 打包, 超了静默截断)。"
                   "位置刚度 kp 固定为 0 = 纯力矩控制。"),
        Param("gripper_script_timeout", "脚本超时", "gripper", "float", unit="s",
              note="传给脚本的 --timeout: 脚本内部一次闭合/张开的墙钟上限。"),
        Param("gripper_timeout", "节点侧超时", "gripper", "float", unit="s",
              note="从 spawn 到拿到结论最多等多久。**必须大于脚本超时** —— 反过来的话日志里只剩"
                   "一句「超时」, 看不到脚本 [abort] 的原因。"),
    ]
    return tuple(pose + orient + height + timing + switch + gripper)


PARAMS: tuple[Param, ...] = _params()
PARAM_BY_KEY: dict[str, Param] = {p.key: p for p in PARAMS}
VEC_LEN = {"vec3": 3, "vec6": 6, "quat": 4}
NUMBER_KINDS = ("float", "int")


# ── 校验结论 ────────────────────────────────────────────────

@dataclass(frozen=True)
class Problem:
    """一条校验结论。error 会**拒绝保存**, warn 只提示。"""

    level: str            # "error" | "warn"
    key: str              # 相关参数(可为 "" = 跨字段/文件级)
    text: str

    def render(self) -> str:
        return ("✗ " if self.level == "error" else "! ") + self.text


# ── 纯换算 (与节点/桥同式, 故意另写一份) ────────────────────

def rpy_to_quaternion(rx_deg: float, ry_deg: float, rz_deg: float) -> np.ndarray:
    """ZYX 顺序 RPY(度) -> (x, y, z, w)。

    与节点 `_rpy_to_quaternion` 同式。**前向转换在 |ry|=90° 不退化**, 所以位姿那 6 个数
    可以放心按角度编辑(反方向才会退化, 见 quaternion_to_rpy_zyx)。
    返回值的 w 规范到 >= 0 (q 与 -q 是同一个旋转, 但这样和文档里的数一致)。
    """
    cr, sr = math.cos(math.radians(rx_deg) * 0.5), math.sin(math.radians(rx_deg) * 0.5)
    cp, sp = math.cos(math.radians(ry_deg) * 0.5), math.sin(math.radians(ry_deg) * 0.5)
    cy, sy = math.cos(math.radians(rz_deg) * 0.5), math.sin(math.radians(rz_deg) * 0.5)
    quaternion = np.asarray(
        [
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * sp * sy,
        ],
        dtype=np.float64,
    )
    norm = float(np.linalg.norm(quaternion))
    if norm < 1e-12:
        raise ValueError("四元数模长为零 —— RX/RY/RZ 填错了?")
    quaternion = quaternion / norm
    return -quaternion if quaternion[3] < 0.0 else quaternion


def quaternion_to_matrix(quaternion: Sequence[float]) -> np.ndarray:
    """(x, y, z, w) 单位四元数 -> tool 到 base 的 3x3 旋转矩阵 (与节点同式)。"""
    x, y, z, w = quaternion
    return np.asarray(
        (
            (1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)),
            (2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)),
            (2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)),
        ),
        dtype=np.float64,
    )


def quaternion_to_rpy_zyx(quaternion: Sequence[float]) -> tuple[float, float, float]:
    """(x, y, z, w) -> ZYX 顺序 RPY(**度**)。与 `h3001_bridge.py` 的 `_pose_to_list` 同式。

    ⚠ 这是**反解**: |ry| -> 90° 时退化(万向锁) —— 那里 RX/RZ 不再可分, 而且解出来的角度
    与"人当初输入的那组"可能不是一组数(同一个旋转有多组 RPY)。所以界面上这个结果**只读**,
    要按角度改姿态得走 rpy_to_quaternion 的正向(那条路不退化)。
    """
    x, y, z, w = quaternion
    rx = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    ry = math.asin(max(-1.0, min(1.0, 2.0 * (w * y - z * x))))
    rz = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return math.degrees(rx), math.degrees(ry), math.degrees(rz)


def gimbal_lock_margin_deg(quaternion: Sequence[float]) -> float:
    """姿态离万向锁还有多少度 (90 - |ry|)。小于 ~15° 时反解出来的 RPY 就别当真了。"""
    return 90.0 - abs(quaternion_to_rpy_zyx(quaternion)[1])


def reported_to_gripper(reported_m, quaternion, offset_tool) -> np.ndarray:
    """上报末端原点 -> 夹爪点 (夹爪 = 上报 + R @ offset, 与节点 `_reported_to_gripper` 同式)。"""
    return (np.asarray(reported_m, dtype=np.float64)
            + quaternion_to_matrix(quaternion) @ np.asarray(offset_tool, dtype=np.float64))


def quaternion_angle_deg(first: Sequence[float], second: Sequence[float]) -> float:
    """两个姿态的夹角 (度), 取 [0, 180]。"""
    dot = abs(float(np.dot(np.asarray(first, dtype=np.float64),
                           np.asarray(second, dtype=np.float64))))
    return math.degrees(2.0 * math.acos(min(1.0, dot)))


def pose_preview(vec6: Sequence[float], offset_tool: Sequence[float]) -> dict:
    """位姿那 6 个数换算出来的一切(供界面逐位对照节点启动日志)。

    出错返回 {"error": 原因} —— 调用方要能区分"算不出来"和"算出来是这些数"。
    """
    try:
        reported = np.asarray(vec6[:3], dtype=np.float64) * 0.001
        quaternion = rpy_to_quaternion(*vec6[3:6])
    except (ValueError, TypeError, IndexError) as exc:
        return {"error": str(exc)}
    gripper = reported_to_gripper(reported, quaternion, offset_tool)
    return {
        "reported": reported,
        "gripper": gripper,
        "offset": gripper - reported,
        "offset_norm": float(np.linalg.norm(gripper - reported)),
        "quaternion": quaternion,
    }


def format_pose_report(label: str, vec6: Sequence[float], offset_tool) -> str:
    """复刻节点启动日志那一行的格式, 便于人拿着两边逐位对照。

    节点那行长这样(见 `_load_standby_pose`):
        待机位: 上报原点=[...] m (与 GUI/参数一致才是对的), 夹爪=[...] m,
        偏移=[...] m (模长 N mm), 姿态=[+0.01184482, ...]
    四元数手写 8 位小数: np.round 对小分量会用科学计数法, 而这行就是拿来对数的。
    """
    preview = pose_preview(vec6, offset_tool)
    if "error" in preview:
        return f"{label}: 算不出来 ({preview['error']})"
    quaternion_text = "[" + ", ".join(f"{v:+.8f}" for v in preview["quaternion"]) + "]"
    return (
        f"{label}: 上报原点={np.round(preview['reported'], 4)} m "
        f"(与 GUI/参数一致才是对的), 夹爪={np.round(preview['gripper'], 4)} m, "
        f"偏移={np.round(preview['offset'], 4)} m "
        f"(模长 {preview['offset_norm'] * 1000.0:.1f} mm), 姿态={quaternion_text}"
    )


def derived_travels(values: dict) -> dict:
    """两条下降行程 (m)。

    side 段 = pre_grasp_offset_z - grasp_offset_z;
    top  段 = 单独设了 top_pre_grasp_offset_z(>0)就用它, 否则沿用通用值。
    """
    grasp = float(values.get("grasp_offset_z", 0.0))
    general = float(values.get("pre_grasp_offset_z", 0.0))
    top_raw = float(values.get("top_pre_grasp_offset_z", -1.0))
    top = top_raw if top_raw > 0.0 else general
    return {
        "side_travel": general - grasp,
        "top_travel": top - grasp,
        "top_uses_general": top_raw <= 0.0,
        "general": general,
        "top_raw": top_raw,
    }


# ── 格式化: 写回 yaml 的文本 ────────────────────────────────

def fmt_float(value: float) -> str:
    """浮点 -> yaml 里的文本。**两条硬要求**:

    1. 必须看着就是浮点(`'.' in s`): `rcl_yaml_param_parser` 要求同一个序列里元素类型一致,
       把 `300.0` 写成 `300` 会让它推断成整数数组、与参数声明的小数数组冲突 ——
       **节点直接起不来**(不是静默出错)。test_place.py 里有一条测试扫全 yaml 的每个数组。
    2. **不能出现指数形式**(`5e-05`): PyYAML 认, 但那个解析器没有保证。0.0005 这种值必须写全。
    """
    number = float(value)
    if not math.isfinite(number):
        raise ValueError(f"不是有限数: {value!r}")
    text = repr(number)
    if "e" in text or "E" in text:
        text = f"{number:.12f}".rstrip("0")
        if text.endswith("."):
            text = text + "0" if number != 0.0 else "0.0"
    if "." not in text:
        text += ".0"
    return text


def format_value(param: Param, value: Any) -> str:
    """按参数类型写出行内文本(与 yaml 现场风格一致: `[0.0, 0.0, 0.1]`)。"""
    if param.kind == "bool":
        return "true" if value else "false"
    if param.kind == "int":
        return str(int(value))
    if param.kind == "choice":
        return str(value)
    if param.kind in VEC_LEN:
        return "[" + ", ".join(fmt_float(v) for v in value) + "]"
    return fmt_float(value)


def format_display(value: Any) -> str:
    """输入框里显示什么。与 format_value 同一套格式 —— 显示与写盘一致, 就不会出现
    「看着是 0.5、存进去是 0.5000000000001」这种事。"""
    if isinstance(value, bool):
        return "true" if value else "false"
    if isinstance(value, int):
        return str(value)
    if isinstance(value, float):
        return fmt_float(value)
    return str(value)


# ── 读 yaml ─────────────────────────────────────────────────

def as_path(path) -> Path:
    """接受 str / Path 两种写法(与 os.path 那一套的惯例一致), 统一成 Path。"""
    return path if isinstance(path, Path) else Path(path)


def read_lines(path) -> list[str]:
    """逐行读(**保留行尾**)。

    `newline=""` 是必须的: 默认的通用换行会把 CRLF 归一成 LF, 那样「不改任何值直接保存」
    就不再是逐字节相同了(幂等性是这条链上最便宜的一条自检)。
    """
    with open(as_path(path), "r", encoding="utf-8", newline="") as handle:
        return handle.read().splitlines(keepends=True)


def snapshot(path) -> dict:
    """文件指纹: 保存前用它确认「我读的那一份还是现在这一份」。"""
    path = as_path(path)
    data = path.read_bytes()
    stat = path.stat()
    return {
        "mtime_ns": stat.st_mtime_ns,
        "size": stat.st_size,
        "sha256": hashlib.sha256(data).hexdigest(),
    }


def load_params(path) -> dict:
    """读 yaml 里 `ros__parameters` 那一层(与 test_track_wait 的取法一致)。"""
    with open(as_path(path), "r", encoding="utf-8") as handle:
        document = yaml.safe_load(handle) or {}
    return ((document.get(ROOT_KEY) or {}).get(PARAMS_KEY)) or {}


def find_block(lines: Sequence[str]) -> tuple[int, int]:
    """定位**本节点**那个 `ros__parameters:` 块的行区间 [start, end)。

    先找顶层的 `bottle_grasp_executor:`(别的节点段/注释里也可能出现 `ros__parameters:`),
    再找它下面那个 —— 只在这个区间里找键。否则 `dry_run` 这种短名很容易在别处撞上。
    块尾 = 第一个缩进 <= 2 的非空非注释行(文件到结尾就是 len)。
    """
    root = None
    for index, line in enumerate(lines):
        if re.match(r"^" + re.escape(ROOT_KEY) + r"\s*:\s*$", line):
            root = index
            break
    start = None
    for index in range(root + 1 if root is not None else 0, len(lines)):
        if re.match(r"^\s*" + re.escape(PARAMS_KEY) + r"\s*:\s*$", lines[index]):
            start = index + 1
            break
    if start is None:
        raise ValueError(f"yaml 里找不到 {ROOT_KEY}.{PARAMS_KEY} —— 是不是改了文件结构?")
    end = len(lines)
    for index in range(start, len(lines)):
        line = lines[index]
        if not line.strip() or line.lstrip().startswith("#"):
            continue
        if len(line) - len(line.lstrip(" ")) <= 2:
            end = index
            break
    return start, end


def key_pattern(key: str) -> re.Pattern:
    """匹配一行参数定义。缩进**正好** 4 个空格(块内一级), 键名后面必须紧跟 `:`
    (所以 `track_enabled` 不会匹配到 `track_enabled_x`)。"""
    return re.compile(
        r"^(?P<indent> {4})(?P<key>" + re.escape(key) + r")(?P<sep>\s*:\s*)(?P<rest>[^\n]*)$"
    )


def find_key_line(lines: Sequence[str], block: Sequence[int], key: str) -> int | None:
    """键所在的行号。不存在返回 None;**出现不止一次直接报错**(不猜 —— 猜错了就是改错行)。"""
    start, end = block
    pattern = key_pattern(key)
    hits = [i for i in range(start, end) if pattern.match(lines[i])]
    if len(hits) > 1:
        raise ValueError(
            f"{key} 在 yaml 里出现了 {len(hits)} 次 (第 "
            + ", ".join(str(i + 1) for i in hits) + " 行) —— 拒绝改, 请先手工理清"
        )
    return hits[0] if hits else None


def split_line(line: str) -> tuple[str, str, str, str]:
    """把一行参数拆成 (前缀, 值, 值尾空格, 行内注释+行尾换行)。

    前缀 = `    key:` + 冒号后面原有的空格(**原样保留** —— 现场那份 yaml 里
    `top_orientation_xyzw:` 后面就是两个空格);
    值 = 一直到 `#` 之前(我们的值里不会出现 `#`, 所以按第一个 `#` 切就够);
    尾部 = 值后面的空格 + 行内注释 + 行尾(本文件现在没有行内注释, 保留这一栏以防将来有)。
    """
    ending = line[len(line.rstrip("\r\n")):]                # "\n" / "\r\n" / ""
    body_line = line[:len(line) - len(ending)] if ending else line
    match = re.match(r"^(?P<prefix>\s*\S+\s*:\s*)(?P<rest>.*)$", body_line)
    if match is None:
        raise ValueError(f"这一行看不懂: {line!r}")
    prefix, rest = match.group("prefix"), match.group("rest")
    hash_index = rest.find("#")
    if hash_index < 0:
        value_and_pad, trail = rest, ""
    else:
        value_and_pad, trail = rest[:hash_index], rest[hash_index:]
    value = value_and_pad.rstrip(" ")
    return prefix, value, value_and_pad[len(value):], trail + ending


def captured_value(line: str, key: str) -> Any:
    """把一行的值抠出来并**用 YAML 自己解析一遍**。

    这是防正则漂移的那道闸: 万一正则把值切错了(同尾键、将来某行带了 `#`), 这里解出来的
    数就与 load_params 读到的不一致 —— 调用方据此拒绝改这一行, 而不是把错的数写回去。
    """
    _prefix, value_text, _pad, _trail = split_line(line)
    try:
        return yaml.safe_load(f"{key}: {value_text}")[key]
    except Exception as exc:                                # noqa: BLE001
        raise ValueError(f"{key} 这一行的值解析不了 ({exc}): {value_text!r}") from exc


def render_line(line: str, value_text: str) -> str:
    """用新的值文本重建这一行(前缀、值尾空格、行内注释、行尾换行都原样保留)。"""
    prefix, _value, pad, trail = split_line(line)
    return f"{prefix}{value_text}{pad}{trail}"


# ── 写 yaml ─────────────────────────────────────────────────

@dataclass(frozen=True)
class Diff:
    """一行将要发生的改动。"""

    key: str
    line_no: int            # 1 起
    old_line: str
    new_line: str
    old_value: Any
    new_value: Any


@dataclass
class SaveReport:
    """保存结果(给界面写状态栏, 也给测试断言)。"""

    backup_path: Path | None
    changed: tuple[str, ...] = ()
    skipped_missing: tuple[str, ...] = ()


def build_edit(lines: Sequence[str], values: dict) -> tuple[list[str], list[Diff], list[str]]:
    """算出新的行内容。**纯函数, 一个字节都不落盘。**

    只重写**值真的变了**的那些行: 值没变就原样留着 —— 这样「不改任何值直接保存」必然是
    逐字节相同(幂等), 也顺手避免了把 `0.150` 归一成 `0.15` 这类与用户无关的改动。

    返回 (新行, 改动清单, yaml 里没有的键)。
    """
    block = find_block(lines)
    new_lines = list(lines)
    diffs: list[Diff] = []
    missing: list[str] = []

    for param in PARAMS:
        if param.key not in values:
            continue
        index = find_key_line(lines, block, param.key)
        if index is None:
            missing.append(param.key)
            continue
        current = captured_value(lines[index], param.key)
        wanted = values[param.key]
        if current == wanted:                     # 值没变: 一个字节都不动
            continue
        new_line = render_line(lines[index], format_value(param, wanted))
        new_lines[index] = new_line
        diffs.append(Diff(param.key, index + 1, lines[index].rstrip("\r\n"),
                          new_line.rstrip("\r\n"), current, wanted))
    return new_lines, diffs, missing


def verify_written(path, expected: dict, original_lines: Sequence[str],
                   changed_indexes: Iterable[int]) -> list[str]:
    """回读校验。返回问题清单(空 = 通过)。

    五条, 从强到弱: 能解析 / 改动键的值等于期望 / **行数相同** / **未改动的行逐字节相同** /
    未改动的键值全等。最后两条才是真正保护注释的那一对 —— 只比"值对不对"拦不住
    「注释被吃掉、顺序被重排、空行被删」。
    """
    problems: list[str] = []
    path = as_path(path)
    try:
        text = path.read_text(encoding="utf-8")
        document = yaml.safe_load(text)
    except Exception as exc:                                # noqa: BLE001
        return [f"写完之后 yaml 解析不了: {exc}"]
    params = ((document or {}).get(ROOT_KEY) or {}).get(PARAMS_KEY) or {}
    for key, wanted in expected.items():
        if key not in params:
            problems.append(f"{key} 写完之后不见了")
        elif params[key] != wanted:
            problems.append(f"{key} 写进去是 {params[key]!r}, 期望 {wanted!r}")
    new_lines = text.splitlines(keepends=True)
    if len(new_lines) != len(original_lines):
        problems.append(
            f"行数变了: {len(original_lines)} -> {len(new_lines)} (只该换值, 不该增删行)"
        )
    protected = set(changed_indexes)
    for index in range(min(len(new_lines), len(original_lines))):
        if index in protected:
            continue
        if new_lines[index] != original_lines[index]:
            problems.append(f"第 {index + 1} 行不该变, 却变了: {new_lines[index]!r}")
    return problems


def backup_path(path, now: float | None = None) -> Path:
    """备份路径。带**微秒**: 一秒内保存两次不能把上一个备份盖掉 —— 这个工作区不是
    git 仓库, `.bak` 是唯一的安全网。"""
    path = as_path(path)
    stamp = time.strftime("%Y%m%d-%H%M%S", time.localtime(now))
    micros = int((now if now is not None else time.time()) * 1_000_000) % 1_000_000
    candidate = path.with_name(f"{path.name}.bak-{stamp}-{micros:06d}")
    suffix = 1
    while candidate.exists():
        candidate = path.with_name(f"{path.name}.bak-{stamp}-{micros:06d}-{suffix}")
        suffix += 1
    return candidate


def save_params(path, values: dict, before: dict | None = None) -> SaveReport:
    """把 values 写回 yaml。**只写值, 注释/空行/顺序/行数一律不动。**

    `before` 是载入时记下的指纹(snapshot 的返回值): 传了就要求文件到现在还没被改动过 ——
    否则按旧行号覆盖会把别人(另一个窗口、手改、另一个会话)的改动静默吞掉。

    任一步失败都从备份恢复(用**复制**, 不用 os.replace —— 后者会把备份本身吃掉, 而它是
    唯一的证据)。抛 ValueError, 里面写清是哪一步。
    """
    path = as_path(path)
    if not path.exists():
        raise ValueError(f"文件不存在: {path}")
    if not os.access(path, os.W_OK):
        raise ValueError(f"文件不可写: {path}")
    if before is not None and snapshot(path) != before:
        raise ValueError(
            "参数文件在载入之后被改动过 (mtime/大小/内容 至少一项变了) —— 拒绝写入, "
            "免得覆盖别人的改动。请点「重新载入」看一眼再改。"
        )

    lines = read_lines(path)
    new_lines, diffs, missing = build_edit(lines, values)
    if not diffs:
        return SaveReport(None, (), tuple(missing))

    backup = backup_path(path)
    shutil.copy2(path, backup)                     # 先留退路, 再动原文件

    mode = path.stat().st_mode
    temp = path.with_name(path.name + ".tmp-tuner")
    try:
        with open(temp, "w", encoding="utf-8", newline="") as handle:
            handle.write("".join(new_lines))
        os.chmod(temp, mode & 0o7777)              # 新建的文件是 600, 原文件是 644/755
        os.replace(temp, path)                     # 原子替换: 写一半断电也不会留半个文件
    except Exception as exc:                       # noqa: BLE001
        if temp.exists():
            temp.unlink()
        shutil.copy2(backup, path)                 # 复制回去, 备份留着
        raise ValueError(f"写入失败, 已从备份恢复: {exc}") from exc

    expected = {diff.key: diff.new_value for diff in diffs}
    problems = verify_written(path, expected, lines, [d.line_no - 1 for d in diffs])
    if problems:
        shutil.copy2(backup, path)                 # 同上: 复制, 别把备份吃掉
        raise ValueError("写完回读校验不过, 已从备份恢复: " + "; ".join(problems)
                         + f"\n备份: {backup}")
    return SaveReport(backup, tuple(d.key for d in diffs), tuple(missing))


# ── 校验 ────────────────────────────────────────────────────

def workspace_limits(values: dict) -> dict:
    """工作空间边界(米)。与节点 `_workspace_problem` 同序: x, y, z。"""
    return {
        "x": (float(values.get("workspace_x_min", -1.9)),
              float(values.get("workspace_x_max", 1.9))),
        "y": (float(values.get("workspace_y_min", -1.9)),
              float(values.get("workspace_y_max", 1.9))),
        "z": (float(values.get("workspace_z_min", -1.1)),
              float(values.get("workspace_z_max", 1.9))),
    }


def workspace_problem(target, limits: dict, label: str) -> str | None:
    """纯谓词: 越界返回描述串, 在界内返回 None (与节点 `_workspace_problem` 同式)。"""
    for axis, (low, high) in limits.items():
        value = float(target["xyz".index(axis)])
        if not math.isfinite(value):
            return f"{label}坐标含非有限值"
        if not (low <= value <= high):
            return f"{label} {axis}={value:.3f} 超出 [{low}, {high}]"
    return None


def check_pose(key: str, vec6, values: dict, label: str,
               *, block_on_near_origin: bool) -> list[Problem]:
    """位姿的加载期校验(镜像节点 `_load_standby_pose` / `_load_place_pose`)。

    **两处不对称是刻意的**: 「上报原点离基座原点太近」在待机位那边节点只 warn
    (原话: 工作空间的 z 下限拦不住按米填的错), 在放置位那边直接 error 并禁用放置
    (默认占位值就是全零)。
    """
    problems: list[Problem] = []
    if vec6 is None or len(vec6) != 6:
        return [Problem("error", key, f"{label}: 需要 6 个数 [X,Y,Z(mm), RX,RY,RZ(度)]")]
    if not all(math.isfinite(float(v)) for v in vec6):
        return [Problem("error", key, f"{label}: 6 个数里有没有限值")]

    offset = values.get("gripper_offset_tool_xyz", [0.0, 0.0, 0.1])
    preview = pose_preview(vec6, offset)
    if "error" in preview:
        return [Problem("error", key, f"{label}: {preview['error']}")]
    reported = preview["reported"]
    distance = float(np.linalg.norm(reported))
    if distance < 0.05:
        problems.append(Problem(
            "error" if block_on_near_origin else "warn", key,
            f"{label}的上报原点离基座原点只有 {distance * 1000.0:.1f} mm "
            "—— 检查位置是不是误按米填写 (应为 mm)"))
    limits = workspace_limits(values)
    for target, tag in ((reported, f"{label}(上报原点)"), (preview["gripper"], f"{label}(夹爪)")):
        problem = workspace_problem(target, limits, tag)
        if problem is not None:
            problems.append(Problem("error", key, f"WORKSPACE_REJECTED {problem}"))

    margin = gimbal_lock_margin_deg(preview["quaternion"])
    if margin < 15.0:
        problems.append(Problem(
            "warn", key,
            f"{label}离万向锁只有 {margin:.1f}° —— 姿态本身没问题(四元数唯一), "
            "但上面那行显示出来的 RPY 不是唯一的一组数, 别照它去「修」"))
    return problems


def check_quaternion(key: str, quaternion, label: str) -> list[Problem]:
    """四元数: 模长与"是否已归一"(`_orientation_for_mode` 会自己归一, 所以只 warn)。"""
    if quaternion is None or len(quaternion) != 4:
        return [Problem("error", key, f"{label}: 需要 4 个数 (x, y, z, w)")]
    if not all(math.isfinite(float(v)) for v in quaternion):
        return [Problem("error", key, f"{label}: 4 个数里有没有限值")]
    norm = math.sqrt(sum(float(v) ** 2 for v in quaternion))
    if norm < 1e-12:
        return [Problem("error", key, f"{label}: 四元数模长为零")]
    if abs(norm - 1.0) > 0.01:
        return [Problem("warn", key, f"{label}: 模长 {norm:.4f} 不是 1 (节点会自己归一)")]
    return []


def check_cross_field(values: dict, bridge_invert: bool | None,
                      node_invert: bool | None) -> list[Problem]:
    """跨字段不变量。前四条照搬 test_track_wait.py 的 ConfigConsistencyTest ——
    它们都是真机上出过问题的, 不是纸面上的洁癖。"""
    problems: list[Problem] = []

    def number(key, default=0.0):
        try:
            return float(values.get(key, default))
        except (TypeError, ValueError):
            return float("nan")

    period = number("track_min_command_interval")
    max_age = number("max_result_age")
    deadband = number("track_deadband")
    max_lag = number("track_max_lag")
    epsilon = number("auto_grasp_stable_epsilon")
    hold = number("auto_grasp_stable_time")

    if not (max_age > period) or not (period > 0.0):
        problems.append(Problem(
            "error", "max_result_age",
            f"max_result_age ({max_age}) 必须大于跟随周期 ({period}): "
            "否则还没读到下一次坐标就先判跟丢"))
    if not (deadband < max_lag):
        problems.append(Problem(
            "error", "track_deadband",
            f"跟随死区 ({deadband}) 必须小于落后上限 ({max_lag}): "
            "否则例外通道恒开, 停稳门等于没有"))
    if not (0.005 <= epsilon <= deadband):
        problems.append(Problem(
            "error", "auto_grasp_stable_epsilon",
            f"自动下降容差 ({epsilon}) 必须落在 [0.005, 跟随死区 {deadband}]: "
            "低于 5mm 一次都不会触发(实测), 比死区还粗等于承认"
            "「已经动到不想跟了还算没动」"))
    for key in ("track_settle_time", "track_stable_epsilon"):
        if not (number(key) > 0.0):
            problems.append(Problem("error", key, f"{key} 必须为正"))
    if hold < 0.0:
        problems.append(Problem("error", "auto_grasp_stable_time", "不能为负 (0 = 关掉)"))
    elif 0.0 < hold < 2.0 * period:
        problems.append(Problem(
            "warn", "auto_grasp_stable_time",
            f"自动下降稳定时间 ({hold}s) 不到两个跟随周期 ({2.0 * period:.1f}s) —— "
            "物体坐标的正常抖动就可能让它永不触发"))

    stall = number("stall_detect_time")
    if not math.isnan(stall) and stall <= max(number("settle_time"),
                                              number("track_settle_time")):
        problems.append(Problem(
            "warn", "stall_detect_time",
            f"停稳判定时间 ({stall}s) 不比 settle_time / track_settle_time 长: "
            "机械臂还在走就会被判停稳, 迭代修正会把残差当成下垂反复推"))

    if not (number("ik_verdict_grace") >= 0.5):
        problems.append(Problem("warn", "ik_verdict_grace",
                                "低于 0.5s 会读到**上一条**指令的 IK 判决, 把好指令误判成拒收"))

    general = number("pre_grasp_offset_z")
    grasp = number("grasp_offset_z")
    if general < grasp:
        problems.append(Problem("warn", "pre_grasp_offset_z",
                                f"悬停高度 ({general}) 比落点 ({grasp}) 还低 —— "
                                "这一段就不是「下降」而是上升了"))

    # 行程 / dwell。唯一在真机上验证过的组合是 9cm / 2.0s = 4.5cm/s, 比它快只是
    # "没实测过", 不是错 —— yaml 自己也写着「这个组合没有实测过」。
    try:
        travels = derived_travels(values)
        dwell = number("descend_dwell")
    except (TypeError, ValueError):
        travels, dwell = None, 0.0
    if travels is not None and dwell > 0.0:
        fastest = max(travels["side_travel"], travels["top_travel"]) / dwell
        if fastest > 0.045:
            problems.append(Problem(
                "warn", "descend_dwell",
                f"下降最快 {fastest * 100.0:.1f} cm/s —— 比实测过的 (9cm/2.0s = 4.5cm/s) 快: "
                "这个组合没实测过, 半空闭合夹爪(夹个空)就是这么来的。盯一眼日志里那个"
                "「实际」栏, 或者把 dwell 调大"))

    if values.get("top_skip_track") and number("top_pre_grasp_offset_z") <= 0.0:
        problems.append(Problem(
            "warn", "top_skip_track",
            "top 不跟踪(悬停到位直接下降)却沿用通用的悬停高度 —— 从很高处正上方盲降, "
            "与 2026-09-28 把 top 压到 1cm 的理由冲突"))
    if values.get("top_skip_lift") and not values.get("place_enabled"):
        problems.append(Problem("warn", "top_skip_lift",
                                "没开放置却让 top 跳过抬升: 节点会打一行 warn 并**不跳**"))

    if values.get("place_enabled"):
        place = values.get("place_pose_reported_mm_deg")
        preview = pose_preview(place, [0.0, 0.0, 0.0]) if place else {"error": "没有值"}
        if "error" in preview or float(np.linalg.norm(preview["reported"])) < 0.05:
            problems.append(Problem(
                "error", "place_enabled",
                "开着放置却没有有效的放置位姿 —— execute_grasp 会直接拒绝启动"
                "(不会抓完才发现无处可放)"))

    hold_torque = number("gripper_hold_torque")
    hard = number("gripper_hard_torque_limit")
    if hold_torque < 0.0 or hold_torque >= hard:
        problems.append(Problem(
            "error", "gripper_hold_torque",
            f"保持力矩 ({hold_torque}) 必须 >= 0 且小于力矩硬上限 ({hard}) —— "
            "硬上限只写在闭合循环里, 保持比它还大就绕过了那道保护"))
    if number("gripper_timeout") <= number("gripper_script_timeout"):
        problems.append(Problem("error", "gripper_timeout",
                                "节点侧超时必须大于脚本超时, 否则日志里只剩一句「超时」, "
                                "看不到脚本 [abort] 的原因"))
    if 0.0 < hold_torque < number("gripper_resistance_torque"):
        problems.append(Problem("warn", "gripper_hold_torque",
                                "保持力矩低于闭合停下时的阻力阈值 —— 切过去的一瞬间夹持力"
                                "会先降一下(脚本会打一行 [warn])"))

    if bridge_invert is not None and node_invert is not None:
        if bool(node_invert) != bool(bridge_invert):
            problems.append(Problem(
                "warn", "ik_target_invert_rx_ry",
                f"ik_target_invert_rx_ry={node_invert} 与桥的 invert_rx_ry={bridge_invert} "
                "不一致 —— **必须同值**, 否则 IK 解的是另一个位姿"))
    return problems


def check_ranges(values: dict) -> list[Problem]:
    """表里声明的上下限(lo/hi)。"""
    problems: list[Problem] = []
    for param in PARAMS:
        value = values.get(param.key)
        if value is None or param.kind not in NUMBER_KINDS:
            continue
        try:
            number = float(value)
        except (TypeError, ValueError):
            problems.append(Problem("error", param.key, f"{param.label}: 不是数字"))
            continue
        if not math.isfinite(number):
            problems.append(Problem("error", param.key, f"{param.label}: 不是有限数"))
        elif param.lo is not None and number < param.lo:
            problems.append(Problem("error", param.key,
                                    f"{param.label} ({number}) 小于下限 {param.lo}"))
        elif param.hi is not None and number > param.hi:
            problems.append(Problem("error", param.key,
                                    f"{param.label} ({number}) 大于上限 {param.hi}"))
    return problems


def validate(values: dict, bridge_invert: bool | None = None,
             missing: Sequence[str] = (), node_invert: bool | None = None) -> list[Problem]:
    """全部校验, error 排在前面。

    `node_invert` 是 yaml 里 `ik_target_invert_rx_ry` 的值 —— 它不在可编辑表里(改它要
    与桥成对), 但它必须与桥一致, 所以由调用方从 yaml 里读出来传进来。
    """
    problems = list(check_ranges(values))
    problems += check_quaternion("side_orientation_xyzw",
                                 values.get("side_orientation_xyzw"), "side 抓取姿态")
    problems += check_quaternion("top_orientation_xyzw",
                                 values.get("top_orientation_xyzw"), "top 抓取姿态")
    problems += check_pose("standby_pose_reported_mm_deg",
                           values.get("standby_pose_reported_mm_deg"), values,
                           "待机位", block_on_near_origin=False)
    problems += check_pose("place_pose_reported_mm_deg",
                           values.get("place_pose_reported_mm_deg"), values,
                           "放置位", block_on_near_origin=bool(values.get("place_enabled")))
    problems += check_cross_field(values, bridge_invert, node_invert)
    for key in missing:
        param = PARAM_BY_KEY.get(key)
        problems.append(Problem("warn", key,
                                f"yaml 里没有 {(param.label if param else key)} 这个键 —— "
                                "本界面不代写新键, 请手工加一行(连注释一起)"))
    return sorted(problems, key=lambda item: 0 if item.level == "error" else 1)


# ── 读桥的 invert_rx_ry (只做一致性提醒) ────────────────────

def bridge_invert_rx_ry(path: Path | None = None) -> bool | None:
    """读 h3001_bridge.yaml 的 invert_rx_ry; 读不到返回 None(不提示)。"""
    try:
        with open(path or BRIDGE_YAML, "r", encoding="utf-8") as handle:
            document = yaml.safe_load(handle) or {}
    except Exception:                                       # noqa: BLE001
        return None
    params = ((document.get("h3001_bridge") or {}).get(PARAMS_KEY)) or {}
    value = params.get("invert_rx_ry")
    return None if value is None else bool(value)


# ── 「抄当前末端」(唯一需要 ROS 的功能, 懒加载) ─────────────

class TcpPoseReader(threading.Thread):
    """读一帧 /feedback/tcp_pose 就退出。**只写普通字段, 不碰任何 Tk 对象。**

    主线程在 `_tick` 里轮询 `done` —— 与 h3001_console.py 的 RosBridge + _poll_ros 同一模式
    (读代码的人不用学第二套)。

    ⚠ 订阅必须用 `qos_profile_sensor_data`(BEST_EFFORT): 桥发的是传感器 QoS, 拿默认的
    RELIABLE 订阅**一条都收不到, 而且只打一行 warn** —— check_place_pose.py 与 console 的
    注释里都写死了这条。
    """

    def __init__(self, topic: str, timeout_s: float = TCP_TIMEOUT_S):
        super().__init__(daemon=True)
        self.topic = topic
        self.timeout_s = timeout_s
        self.pose: tuple | None = None      # (位置 xyz(m), 四元数 xyzw)
        self.error: str = ""
        self.done = False

    def run(self) -> None:
        try:
            import rclpy
            from geometry_msgs.msg import PoseStamped
            from rclpy.node import Node
            from rclpy.qos import qos_profile_sensor_data
        except Exception as exc:                            # noqa: BLE001
            self.error = (f"没 source ROS ({exc}) —— 手抄厂商 GUI 上那 6 个数即可, 或者 "
                          "source /opt/ros/humble/setup.bash + install/setup.bash 之后重开本窗口")
            self.done = True
            return
        node = None
        try:
            rclpy.init()
            node = Node("h3001_tuner_probe")
            box: dict = {}

            def on_pose(message) -> None:
                box["pose"] = message

            node.create_subscription(PoseStamped, self.topic, on_pose,
                                     qos_profile_sensor_data)
            deadline = time.monotonic() + self.timeout_s
            while time.monotonic() < deadline and "pose" not in box:
                rclpy.spin_once(node, timeout_sec=0.05)
            if "pose" not in box:
                self.error = (f"{self.timeout_s:.0f}s 内没收到 {self.topic} —— "
                              "h3001_bridge 起了吗? 试 ros2 topic hz " + self.topic)
            else:
                message = box["pose"].pose
                self.pose = (
                    (float(message.position.x), float(message.position.y),
                     float(message.position.z)),
                    (float(message.orientation.x), float(message.orientation.y),
                     float(message.orientation.z), float(message.orientation.w)),
                )
        except Exception as exc:                            # noqa: BLE001
            self.error = f"读 {self.topic} 出错: {exc}"
        finally:
            try:
                if node is not None:
                    node.destroy_node()
                rclpy.shutdown()
            except Exception:                               # noqa: BLE001
                pass
            self.done = True


def vec6_from_tcp(pose: tuple, invert_rx_ry: bool | None) -> list[float]:
    """(位置 m, 四元数) -> 参数要的那 6 个数 [mm, mm, mm, deg, deg, deg]。

    `invert_rx_ry` 与桥的那个开关成对(取反是自逆的): 现场是 false, 所以按 ZYX 直接反解
    得到的就是固件/GUI 上那组数。
    """
    position, quaternion = pose
    rx, ry, rz = quaternion_to_rpy_zyx(quaternion)
    if invert_rx_ry:
        rx, ry = -rx, -ry
    return [position[0] * 1000.0, position[1] * 1000.0, position[2] * 1000.0, rx, ry, rz]


# ── 界面控件 ────────────────────────────────────────────────

class NumberEditor(ttk.Frame):
    """一行数字输入框(1 个或多个)。`get()` 返回 (值列表|None, 错误串)。"""

    def __init__(self, parent, count: int, width: int = 11, on_edit=None, **kwargs):
        super().__init__(parent, **kwargs)
        self.count = count
        self.vars = [tk.StringVar() for _ in range(count)]
        for index, variable in enumerate(self.vars):
            entry = ttk.Entry(self, textvariable=variable, width=width, justify="right")
            entry.grid(row=0, column=index, padx=(0, 4))
            entry.bind("<FocusIn>", lambda _event, e=entry: e.selection_range(0, "end"))
            if on_edit is not None:
                variable.trace_add("write", lambda *_a, cb=on_edit: cb())

    def get(self) -> tuple[list[float] | None, str]:
        values = []
        for variable in self.vars:
            text = variable.get().strip()
            if text == "":
                return None, "有空着的格子"
            try:
                number = float(text)
            except ValueError:
                return None, f"{text!r} 不是数字"
            if not math.isfinite(number):
                return None, f"{text!r} 不是有限数"
            values.append(number)
        return values, ""

    def set(self, values: Sequence[Any]) -> None:
        for variable, value in zip(self.vars, values):
            variable.set(format_display(value))

    def set_enabled(self, enabled: bool) -> None:
        for child in self.winfo_children():
            if isinstance(child, ttk.Entry):
                child.configure(state="normal" if enabled else "disabled")


class QuatEditor(NumberEditor):
    """四元数输入 + **只读**的 RPY 派生显示 + 「从 RPY 填入」。

    RPY **不做双向绑定** —— 那是个陷阱: 现场 side 姿态离万向锁只有 ~11°, 反解出来的角度
    不是人输的那组数, 而且附近导数极大(改 1° 姿态可能摆几十度)。四元数才是权威输入;
    要按角度改就走「从 RPY 填入」那条**单向**通道(走正向转换, 在 |ry|=90° 不退化)。
    """

    def __init__(self, parent, key: str, on_edit=None, **kwargs):
        super().__init__(parent, 4, on_edit=on_edit, **kwargs)
        self.key = key


# ── 主窗口 ──────────────────────────────────────────────────

class TunerApp(tk.Tk):
    """调参窗口。所有 Tk 调用都在主线程(ROS 那个线程只写字段)。"""

    TICK_MS = 250

    def __init__(self, yaml_path: Path, dry_run: bool = False):
        super().__init__()
        self.title(f"H3001 参数调谐 —— {yaml_path.name}")
        self.geometry("1080x740")
        self.minsize(920, 600)

        self.yaml_path = Path(os.path.realpath(yaml_path))    # 永远写解析后的真实文件
        self.dry_run = dry_run
        self.editors: dict[str, Any] = {}
        self.derived_labels: dict[str, ttk.Label] = {}
        self._canvases: dict[ttk.Frame, tk.Canvas] = {}       # 页签 inner -> 它的 canvas
        self.setting_up = True                                # 程序化回填时别触发 dirty
        self.dirty = False
        self.before: dict | None = None                       # 载入时的文件指纹
        self.original_lines: list[str] = []
        self.loaded: dict = {}
        self.missing_keys: list[str] = []
        self.problems: list[Problem] = []
        self.reader: TcpPoseReader | None = None
        self.reader_for = ""
        self.bridge_invert = bridge_invert_rx_ry()

        self._build()
        self.reload()
        self.protocol("WM_DELETE_WINDOW", self._on_close)
        self.after(self.TICK_MS, self._tick)

    # ── 搭界面 ──────────────────────────────────────────────

    def _build(self) -> None:
        pad = {"padx": 8, "pady": 4}

        info = ttk.Frame(self)
        info.pack(side="top", fill="x", **pad)
        self.path_label = ttk.Label(info, text="", anchor="w")
        self.path_label.pack(side="left")
        self.link_label = ttk.Label(info, text="", anchor="w")
        self.link_label.pack(side="left", padx=(12, 0))

        checks = ttk.LabelFrame(self, text="校验 (error 会拒绝保存)")
        checks.pack(side="top", fill="x", **pad)
        self.check_text = tk.Text(checks, height=4, wrap="word", relief="flat",
                                  highlightthickness=0, background="#f7f7f7")
        self.check_text.pack(fill="x", padx=6, pady=2)
        self.check_text.configure(state="disabled")

        # **底栏先 pack**: Tk 的 pack 顺序决定谁先占位置, 反了底栏会被页签挤掉。
        bottom = ttk.Frame(self)
        bottom.pack(side="bottom", fill="x", **pad)
        ttk.Button(bottom, text="保存到 yaml", command=self._on_save).pack(side="left")
        ttk.Button(bottom, text="查看将要写入的改动",
                   command=self._on_show_diff).pack(side="left", padx=6)
        ttk.Button(bottom, text="重新载入", command=self._on_reload).pack(side="left")
        self.status = tk.StringVar(value="")
        ttk.Label(bottom, textvariable=self.status, anchor="w").pack(
            side="left", padx=12, fill="x", expand=True)

        note_bar = ttk.Frame(self)
        note_bar.pack(side="bottom", fill="x", **pad)
        self.note = tk.StringVar(
            value="把鼠标点进/悬停到任意一个控件上, 这里会显示它改了会怎样。")
        ttk.Label(note_bar, textvariable=self.note, anchor="w", wraplength=1020,
                  justify="left", foreground="#333").pack(side="left", fill="x", expand=True)

        self.notebook = ttk.Notebook(self)
        self.notebook.pack(side="top", fill="both", expand=True, **pad)
        for group, title in GROUPS:
            tab = ttk.Frame(self.notebook)
            self.notebook.add(tab, text=title)
            inner = self._make_scrollable(tab)
            self._build_group(inner, group)

    def _make_scrollable(self, parent) -> ttk.Frame:
        """一个可滚动的容器, 返回里面那个 inner frame。

        两个 Linux 专属的坑(不写就是滚不动/横向看不全):
          - 滚轮事件是 `<Button-4>` / `<Button-5>`, 不是 `<MouseWheel>`;
          - 滚轮事件**不冒泡**, 必须给每个子控件也 bind —— 所以建完页签后再递归 bind 一遍
            (见 _bind_wheel)。
        """
        canvas = tk.Canvas(parent, highlightthickness=0)
        scroll = ttk.Scrollbar(parent, orient="vertical", command=canvas.yview)
        inner = ttk.Frame(canvas)
        window = canvas.create_window((0, 0), window=inner, anchor="nw")
        canvas.configure(yscrollcommand=scroll.set)
        canvas.pack(side="left", fill="both", expand=True)
        scroll.pack(side="right", fill="y")
        inner.bind("<Configure>",
                   lambda _event: canvas.configure(scrollregion=canvas.bbox("all")))
        canvas.bind("<Configure>",
                    lambda event: canvas.itemconfigure(window, width=event.width))
        self._canvases[inner] = canvas
        return inner

    def _bind_wheel(self, inner: ttk.Frame) -> None:
        """给页签里所有控件补上滚轮(建完页签后调用一次)。"""
        canvas = self._canvases.get(inner)
        if canvas is None:
            return

        def on_wheel(event) -> None:
            canvas.yview_scroll(-1 if getattr(event, "num", None) == 4 else 1, "units")

        def walk(node) -> None:
            node.bind("<Button-4>", on_wheel)
            node.bind("<Button-5>", on_wheel)
            for child in node.winfo_children():
                walk(child)

        walk(inner)

    def _build_group(self, inner: ttk.Frame, group: str) -> None:
        params = [p for p in PARAMS if p.group == group]
        if group == "pose":
            self._build_pose_group(inner, params)
        elif group == "orient":
            self._build_orient_group(inner, params)
        elif group == "height":
            self._build_height_group(inner, params)
        elif group == "switch":
            self._build_switch_group(inner, params)
        else:
            self._build_grid_group(inner, params)
        # 子控件都建完了再递归挂滚轮 (见 _make_scrollable 的第二个坑)
        inner.after(60, lambda: self._bind_wheel(inner))

    def _build_pose_group(self, inner: ttk.Frame, params: Sequence[Param]) -> None:
        for param in params:
            if param.kind != "vec6":
                continue
            box = ttk.LabelFrame(inner, text=f"{param.label}  ({param.key})")
            box.pack(fill="x", padx=6, pady=6)
            editor = NumberEditor(box, 6, on_edit=self._mark_dirty)
            editor.grid(row=0, column=0, sticky="w", padx=6, pady=4)
            self.editors[param.key] = editor
            self._attach_note(editor, param)
            ttk.Label(box, text="X,Y,Z (mm) + RX,RY,RZ (度)").grid(
                row=0, column=1, sticky="w", padx=4)
            ttk.Button(box, text="抄当前末端",
                       command=lambda k=param.key: self._on_copy_tcp(k)).grid(
                row=0, column=2, sticky="e", padx=6)
            preview = ttk.Label(box, text="", anchor="w", justify="left",
                                foreground="#06c", wraplength=1000)
            preview.grid(row=1, column=0, columnspan=3, sticky="w", padx=6, pady=(0, 4))
            self.derived_labels[param.key] = preview

        param = PARAM_BY_KEY["gripper_offset_tool_xyz"]
        box = ttk.LabelFrame(inner, text=f"{param.label}  ({param.key})")
        box.pack(fill="x", padx=6, pady=6)
        editor = NumberEditor(box, 3, width=9, on_edit=self._mark_dirty)
        editor.grid(row=0, column=0, sticky="w", padx=6, pady=4)
        ttk.Label(box, text="m (tool 系)").grid(row=0, column=1, sticky="w")
        self.editors[param.key] = editor
        self._attach_note(editor, param)
        self.invert_label = ttk.Label(box, text="", anchor="w", justify="left",
                                      wraplength=1000, foreground="#666")
        self.invert_label.grid(row=1, column=0, columnspan=2, sticky="w", padx=6, pady=(0, 4))

    def _build_orient_group(self, inner: ttk.Frame, params: Sequence[Param]) -> None:
        for param in params:
            box = ttk.LabelFrame(inner, text=f"{param.label}  ({param.key})")
            box.pack(fill="x", padx=6, pady=6)
            if param.kind == "choice":
                variable = tk.StringVar()
                combo = ttk.Combobox(box, textvariable=variable,
                                     values=list(param.choices), state="readonly",
                                     width=18)
                combo.grid(row=0, column=0, sticky="w", padx=6, pady=4)
                combo.bind("<<ComboboxSelected>>", lambda _e: self._mark_dirty())
                self.editors[param.key] = variable
                self._attach_note(combo, param)
                ttk.Label(box, text="fixed = 用下面的四元数; "
                                    "current_tcp = 保持触发瞬间的姿态").grid(
                    row=0, column=1, sticky="w")
                continue
            editor = QuatEditor(box, param.key, on_edit=self._mark_dirty)
            editor.grid(row=0, column=0, sticky="w", padx=6, pady=4)
            self.editors[param.key] = editor
            self._attach_note(editor, param)
            ttk.Label(box, text="x, y, z, w").grid(row=0, column=1, sticky="w")
            ttk.Button(box, text="从 RPY 填入",
                       command=lambda k=param.key: self._on_fill_from_rpy(k)).grid(
                row=0, column=2, sticky="e", padx=6)
            rpy = ttk.Label(box, text="", anchor="w", justify="left", foreground="#06c",
                            wraplength=1000)
            rpy.grid(row=1, column=0, columnspan=3, sticky="w", padx=6, pady=(0, 4))
            self.derived_labels[param.key] = rpy

    def _build_height_group(self, inner: ttk.Frame, params: Sequence[Param]) -> None:
        box = ttk.LabelFrame(inner, text="抓取高度与下降行程")
        box.pack(fill="x", padx=6, pady=6)
        columns = 3
        for index, param in enumerate(params):
            self._add_number_row(box, param, index // columns, index % columns)
        derived = ttk.Label(box, text="", anchor="w", justify="left", foreground="#06c",
                            wraplength=1000)
        derived.grid(row=(len(params) + columns - 1) // columns, column=0,
                     columnspan=columns, sticky="w", padx=6, pady=(8, 4))
        self.derived_labels["__travel__"] = derived

    def _build_grid_group(self, inner: ttk.Frame, params: Sequence[Param]) -> None:
        box = ttk.Frame(inner)
        box.pack(fill="both", expand=True, padx=6, pady=6)
        columns = 3
        for index, param in enumerate(params):
            self._add_number_row(box, param, index // columns, index % columns)

    def _add_number_row(self, parent, param: Param, row: int, column: int) -> None:
        cell = ttk.Frame(parent)
        cell.grid(row=row, column=column, sticky="w", padx=(4, 16), pady=3)
        label = ttk.Label(cell, text=param.label, width=22, anchor="e")
        label.grid(row=0, column=0, sticky="e")
        editor = NumberEditor(cell, 1, width=9, on_edit=self._mark_dirty)
        editor.grid(row=0, column=1, sticky="w", padx=(6, 4))
        self.editors[param.key] = editor
        ttk.Label(cell, text=param.unit, foreground="#666").grid(row=0, column=2, sticky="w")
        self._attach_note(editor, param)
        label.bind("<Enter>", lambda _e, p=param: self.note.set(self._note_for(p)))

    def _build_switch_group(self, inner: ttk.Frame, params: Sequence[Param]) -> None:
        box = ttk.Frame(inner)
        box.pack(fill="both", expand=True, padx=6, pady=6)
        for index, param in enumerate(params):
            variable = tk.BooleanVar()
            check = ttk.Checkbutton(box, text=param.label, variable=variable,
                                    command=self._mark_dirty)
            check.grid(row=index // 2, column=index % 2, sticky="w", padx=8, pady=6)
            self.editors[param.key] = variable
            check.bind("<Enter>", lambda _e, p=param: self.note.set(self._note_for(p)))
            check.bind("<FocusIn>", lambda _e, p=param: self.note.set(self._note_for(p)))

    def _attach_note(self, widget, param: Param) -> None:
        widget.bind("<FocusIn>", lambda _e, p=param: self.note.set(self._note_for(p)))
        widget.bind("<Enter>", lambda _e, p=param: self.note.set(self._note_for(p)))

    def _note_for(self, param: Param) -> str:
        tail = "  【改后要重启执行器才生效】" if param.reload_required else ""
        return f"{param.label} ({param.key}) —— {param.note}{tail}"

    # ── 载入 / 收集 / 校验 ──────────────────────────────────

    def reload(self) -> None:
        """从磁盘重载, 并记下指纹与逐行原文 —— 保存时要靠它们判断"文件有没有被别处改过"。"""
        self.setting_up = True
        try:
            self.loaded = load_params(self.yaml_path)
            self.original_lines = read_lines(self.yaml_path)
            self.before = snapshot(self.yaml_path)
            block = find_block(self.original_lines)
        except Exception as exc:                            # noqa: BLE001
            self.setting_up = False
            messagebox.showerror("载入失败", str(exc))
            return
        self.missing_keys = [p.key for p in PARAMS
                             if find_key_line(self.original_lines, block, p.key) is None]
        for param in PARAMS:
            widget = self.editors.get(param.key)
            if widget is None:
                continue
            missing = param.key in self.missing_keys
            if isinstance(widget, NumberEditor):
                widget.set_enabled(not missing)
            if missing:
                continue
            value = self.loaded.get(param.key)
            if param.kind == "bool":
                widget.set(bool(value))
            elif param.kind == "choice":
                widget.set(str(value))
            else:
                widget.set(value if isinstance(value, (list, tuple)) else [value])
        self.setting_up = False
        self.dirty = False
        self._refresh_path_labels()
        self._refresh_derived()
        self._refresh_checks()
        self.status.set(f"已载入 {time.strftime('%H:%M:%S')}"
                        + ("  (dry-run: 不会写盘)" if self.dry_run
                           else "  —— 改完记得重启执行器"))

    def collect(self) -> tuple[dict, list[Problem]]:
        """从控件收值。缺键的参数**不进 values** —— 保存时会被跳过并报告出来。"""
        values: dict = {}
        problems: list[Problem] = []
        for param in PARAMS:
            if param.key in self.missing_keys:
                continue
            widget = self.editors.get(param.key)
            if widget is None:
                continue
            if param.kind == "bool":
                values[param.key] = bool(widget.get())
            elif param.kind == "choice":
                values[param.key] = widget.get()
            else:
                numbers, error = widget.get()
                if error:
                    problems.append(Problem("error", param.key, f"{param.label}: {error}"))
                    continue
                if param.kind == "int":
                    values[param.key] = int(round(numbers[0]))
                elif param.kind in NUMBER_KINDS:
                    values[param.key] = numbers[0]
                else:
                    values[param.key] = numbers
        return values, problems

    def _all_values(self) -> dict:
        """给校验用的一份全量字典: 从控件收的值 + yaml 里那些**不可编辑**但校验要看的键。

        `ik_target_invert_rx_ry` 就属于后者(它必须与桥的那个开关成对, 不该在这个界面里
        随手改)。`build_edit` 只遍历 PARAMS, 所以多出来的键不会被写进文件。
        """
        values, _problems = self.collect()
        merged = {key: self.loaded[key] for key in self.loaded
                  if key not in values and key.startswith(("workspace_", "ik_", "stall_",
                                                          "settle", "max_command"))}
        merged.update(values)
        return merged

    def _refresh_path_labels(self) -> None:
        self.path_label.configure(text=f"正在编辑: {self.yaml_path}")
        if not INSTALL_YAML.exists():
            self.link_label.configure(
                text="(还没 colcon build, install 里没有这份 —— 执行器起来时读的就是上面这个文件)",
                foreground="#666")
        elif Path(os.path.realpath(INSTALL_YAML)) == self.yaml_path:
            self.link_label.configure(text="(install 那份是它的符号链接, 两处同步)",
                                      foreground="#666")
        else:
            self.link_label.configure(
                text=f"⚠ install 里是**另一个文件** ({os.path.realpath(INSTALL_YAML)}) —— "
                     "执行器读的是那一份, 这里改了它不会生效, 得先 colcon build",
                foreground="#b00")
        node_invert = self.loaded.get("ik_target_invert_rx_ry")
        if hasattr(self, "invert_label"):
            self.invert_label.configure(
                text=f"ik_target_invert_rx_ry = {node_invert} "
                     f"(**必须与桥的 invert_rx_ry = {self.bridge_invert} 一致**, "
                     "不一致时 IK 解的是另一个位姿 —— 这一项不在本界面里, 要改请手工改)")

    def _refresh_derived(self) -> None:
        """刷新只读的派生显示(位姿预览、等效 RPY、两条行程)。"""
        values, _problems = self.collect()
        offset = values.get("gripper_offset_tool_xyz", [0.0, 0.0, 0.1])
        for key, label in self.derived_labels.items():
            if key == "__travel__":
                continue
            if key in ("standby_pose_reported_mm_deg", "place_pose_reported_mm_deg"):
                vec = values.get(key)
                label.configure(text=format_pose_report(PARAM_BY_KEY[key].label, vec, offset)
                                if vec else "")
            elif key in ("side_orientation_xyzw", "top_orientation_xyzw"):
                quaternion = values.get(key)
                if not quaternion:
                    label.configure(text="")
                    continue
                margin = gimbal_lock_margin_deg(quaternion)
                rpy = quaternion_to_rpy_zyx(quaternion)
                label.configure(
                    text=f"等效 RPY(只读) = [{rpy[0]:.1f}, {rpy[1]:.1f}, {rpy[2]:.1f}] 度 ｜ "
                         f"万向锁余量 {margin:.1f}°"
                         + ("  ⚠ 太近了: 这组角度不是唯一的, 改姿态请改四元数或走「从 RPY 填入」"
                            if margin < 15.0 else ""))
        label = self.derived_labels.get("__travel__")
        if label is not None and all(
                key in values for key in ("pre_grasp_offset_z", "grasp_offset_z",
                                          "top_pre_grasp_offset_z", "descend_dwell")):
            travel = derived_travels(values)
            dwell = float(values.get("descend_dwell") or 0.0)
            speed = (travel["side_travel"] / dwell * 100.0) if dwell > 0 else float("nan")
            label.configure(
                text=f"side 下降行程 = {travel['side_travel'] * 100.0:.1f} cm ｜ "
                     f"top 下降行程 = {travel['top_travel'] * 100.0:.1f} cm"
                     + ("(沿用通用悬停高度)" if travel["top_uses_general"] else "")
                     + f" ｜ 按当前 descend_dwell={dwell:.2f}s 算, side 段速度 {speed:.1f} cm/s")

    def _refresh_checks(self) -> None:
        values, parse_problems = self.collect()
        full = self._all_values()
        problems = list(parse_problems) + validate(
            full, self.bridge_invert, self.missing_keys,
            self.loaded.get("ik_target_invert_rx_ry"))
        # 去重: 从控件收值那一步的报错会在 validate 里再来一遍
        seen = set()
        unique: list[Problem] = []
        for problem in problems:
            signature = (problem.level, problem.key, problem.text)
            if signature in seen:
                continue
            seen.add(signature)
            unique.append(problem)
        self.problems = unique
        lines = (["✓ 校验通过 (上下限、位姿换算与工作空间、跨字段不变量都过)"]
                 if not unique else [problem.render() for problem in unique])
        self.check_text.configure(state="normal")
        self.check_text.delete("1.0", "end")
        self.check_text.insert("1.0", "\n".join(lines))
        self.check_text.configure(state="disabled")
        errors = sum(1 for problem in unique if problem.level == "error")
        self.check_text.configure(background="#ffecec" if errors else "#f7f7f7")
        for index, (group, title) in enumerate(GROUPS):
            count = sum(1 for problem in unique if problem.level == "error"
                        and problem.key in PARAM_BY_KEY
                        and PARAM_BY_KEY[problem.key].group == group)
            self.notebook.tab(index, text=title + ("  ✗" if count else ""))

    def _mark_dirty(self) -> None:
        """控件被改动: 重算派生量与校验(不写盘、也不打扰正在跑的机械臂)。"""
        if self.setting_up:
            return
        self.dirty = True
        self._refresh_derived()
        self._refresh_checks()

    # ── 动作 ────────────────────────────────────────────────

    def _on_reload(self) -> None:
        if self.dirty and not messagebox.askyesno(
                "重新载入", "有改动还没保存, 重新载入会丢掉它们。继续?"):
            return
        self.reload()

    def _compute_diff(self) -> tuple[list[Diff], list[str]]:
        values, _problems = self.collect()
        _new_lines, diffs, missing = build_edit(self.original_lines, values)
        return diffs, missing

    def _on_show_diff(self) -> None:
        _values, parse_problems = self.collect()
        if parse_problems:
            messagebox.showerror("有填错的格子",
                                 "\n".join(p.render() for p in parse_problems))
            return
        try:
            diffs, missing = self._compute_diff()
        except Exception as exc:                            # noqa: BLE001
            messagebox.showerror("算不出改动", str(exc))
            return
        if not diffs:
            messagebox.showinfo("没有改动", "与磁盘上的文件一模一样, 保存也不会动它。")
            return
        body = "\n".join(
            f"{diff.key}  (第 {diff.line_no} 行)\n"
            f"    旧: {diff.old_line.strip()}\n"
            f"    新: {diff.new_line.strip()}"
            for diff in diffs)
        if missing:
            body += ("\n\n以下键 yaml 里没有, **不会**被写入(请手工加一行):\n  "
                     + "\n  ".join(missing))
        self._show_text("将要写入的改动", body)

    def _show_text(self, title: str, body: str) -> None:
        window = tk.Toplevel(self)
        window.title(title)
        window.geometry("780x480")
        text = tk.Text(window, wrap="none", background="#101418", foreground="#d8dee9")
        text.pack(fill="both", expand=True, padx=8, pady=8)
        text.insert("1.0", body)
        text.configure(state="disabled")
        ttk.Button(window, text="关闭", command=window.destroy).pack(pady=(0, 8))

    def _on_save(self) -> None:
        self._refresh_checks()
        errors = [p for p in self.problems if p.level == "error"]
        if errors:
            messagebox.showerror(
                "拒绝保存", "校验没过, 一个字节都没写:\n\n"
                + "\n".join(problem.render() for problem in errors))
            return
        try:
            diffs, missing = self._compute_diff()
        except Exception as exc:                            # noqa: BLE001
            messagebox.showerror("算不出改动", str(exc))
            return
        if not diffs:
            messagebox.showinfo("没有改动", "与磁盘上的文件一模一样, 没有写盘。")
            return
        preview = "\n".join(
            f"{diff.key}: {slice_value(diff.old_value)}  ->  {slice_value(diff.new_value)}"
            for diff in diffs)
        tail = ("\n\n⚠ 这里面有**加载期读一次**的参数, 必须重启执行器才生效。"
                if any(PARAM_BY_KEY[d.key].reload_required for d in diffs) else
                "\n\n(这些参数节点运行期是现读的, 但本界面不发 ros2 param set —— "
                "重启执行器最干净。)")
        if missing:
            tail += "\n\n跳过(文件里没有): " + ", ".join(missing)
        if self.dry_run:
            self._show_text("dry-run: 将要写入的改动 (不落盘)", preview + tail)
            return
        if not messagebox.askyesno("确认写入",
                                   f"将写入 {self.yaml_path}\n\n{preview}{tail}\n\n继续?"):
            return
        try:
            report = save_params(self.yaml_path, self.collect()[0], self.before)
        except Exception as exc:                            # noqa: BLE001
            messagebox.showerror("保存失败", str(exc))
            self.status.set("保存失败 —— 见弹窗; 文件已按备份恢复")
            return
        self.reload()
        self.status.set(
            f"已写入 {len(report.changed)} 处"
            + (f", 备份 {report.backup_path.name}" if report.backup_path else "")
            + ("; 有键被跳过(见校验栏)" if report.skipped_missing else "")
            + "  —— 重启执行器才生效")

    def _on_copy_tcp(self, key: str) -> None:
        """读一帧 /feedback/tcp_pose 填进这个位姿。"""
        if self.reader is not None and not self.reader.done:
            self.status.set("上一帧还没读完, 稍等 ...")
            return
        topic = str(self.loaded.get("tcp_pose_topic", "/feedback/tcp_pose"))
        self.reader_for = key
        self.reader = TcpPoseReader(topic, TCP_TIMEOUT_S)
        self.reader.start()
        self.status.set(f"正在读 {topic} (最多 {TCP_TIMEOUT_S:.0f}s) ...")

    def _on_fill_from_rpy(self, key: str) -> None:
        """单向: 输 RX/RY/RZ(度) -> 预览四元数 -> 确认后写进四元数框。

        走的是**正向**转换(RPY -> 四元数), 在 |ry|=90° 不退化 —— 正是它让"按角度调姿态"
        绕开了反解的万向锁。写进去之前先把与当前姿态的夹角显示出来, 免得改完才发现方向反了
        (四元数没有"看形状就知道对不对"这回事)。
        """
        editor = self.editors.get(key)
        if not isinstance(editor, QuatEditor):
            return
        current, error = editor.get()
        if error or current is None:
            messagebox.showerror("先修好当前值", error or "四元数读不出来")
            return

        window = tk.Toplevel(self)
        window.title(f"从 RPY 填入 —— {PARAM_BY_KEY[key].label}")
        window.transient(self)
        variables = [tk.StringVar(value=f"{value:.1f}")
                     for value in quaternion_to_rpy_zyx(current)]
        for index, axis in enumerate("XYZ"):
            ttk.Label(window, text=f"R{axis} (度):").grid(row=index, column=0, sticky="e",
                                                          padx=6, pady=4)
            ttk.Entry(window, textvariable=variables[index], width=12,
                      justify="right").grid(row=index, column=1, padx=6, pady=4)
        preview = ttk.Label(window, text="", foreground="#06c", wraplength=460,
                            justify="left")
        preview.grid(row=3, column=0, columnspan=3, sticky="w", padx=6)
        result: dict = {}

        def refresh(*_args) -> None:
            try:
                quaternion = rpy_to_quaternion(*[float(v.get()) for v in variables])
            except Exception as exc:                        # noqa: BLE001
                preview.configure(text=f"填不了: {exc}")
                result.clear()
                return
            angle = quaternion_angle_deg(quaternion, current)
            preview.configure(
                text="四元数 = [" + ", ".join(f"{v:.6f}" for v in quaternion) + "]\n"
                     f"与当前姿态夹角 {angle:.1f}°"
                     + ("   ⚠ 超过 90°, 确认方向没反?" if angle > 90.0 else ""))
            result["quaternion"] = [float(v) for v in quaternion]

        for variable in variables:
            variable.trace_add("write", refresh)
        refresh()

        def accept() -> None:
            if "quaternion" not in result:
                return
            editor.set(result["quaternion"])
            window.destroy()
            self._mark_dirty()

        buttons = ttk.Frame(window)
        buttons.grid(row=4, column=0, columnspan=3, pady=8)
        ttk.Button(buttons, text="填入", command=accept).pack(side="left", padx=6)
        ttk.Button(buttons, text="取消", command=window.destroy).pack(side="left")

    def _tick(self) -> None:
        """每 250ms 只干一件事: 看后台那个「抄当前末端」线程读回来没有。

        长任务(等 ROS 那一帧)都在主线程之外跑 —— 主线程一卡, 窗口就假死。
        派生量与校验**不在这里周期性重算**: 那会把校验框每 250ms 重刷一遍、抢走滚动位置,
        所以它们只在控件被改动时重算(_mark_dirty)。
        """
        reader = self.reader
        if reader is not None and reader.done:
            self.reader = None
            key = self.reader_for
            if reader.error:
                self.status.set(reader.error)
            elif reader.pose is not None:
                vec6 = vec6_from_tcp(reader.pose, self.bridge_invert)
                editor = self.editors.get(key)
                if isinstance(editor, NumberEditor):
                    editor.set(vec6)
                margin = 90.0 - abs(vec6[4])
                self.status.set(
                    "已抄下当前末端: " + ", ".join(f"{v:.1f}" for v in vec6)
                    + " (位置 mm + RPY 度; 与 GUI 上的数可能长得不一样, 但旋转是同一个)"
                    + (f"  ⚠ RY 离 90° 只剩 {margin:.1f}°, 这组角度不是唯一的" if margin < 15 else ""))
                self._mark_dirty()
        elif reader is not None and not reader.done:
            pass                                   # 还在读, 下一拍再看
        self.after(self.TICK_MS, self._tick)

    def _on_close(self) -> None:
        if self.dirty and not messagebox.askyesno("退出", "有改动还没保存, 确定退出?"):
            return
        self.destroy()


def slice_value(value: Any) -> str:
    """确认框里显示值: 数组按原样, 其余用显示格式。"""
    if isinstance(value, (list, tuple)):
        return "[" + ", ".join(format_display(v) for v in value) + "]"
    return format_display(value)


# ── 入口 ────────────────────────────────────────────────────

def resolve_yaml(override: str | None = None) -> Path:
    """要编辑的那个文件。**永远是源码树那份**(install 是它的符号链接, 拿 install 路径去
    替换会把链接链打断)。`--yaml` 给了就用它。"""
    if override:
        return Path(os.path.realpath(override))
    if SOURCE_YAML.exists():
        return SOURCE_YAML
    return Path(os.path.realpath(INSTALL_YAML))


def console_check(path: Path) -> int:
    """--dry-run 的正文: 载入 -> 校验 -> 打印将要写入的改动。一个字节都不写盘。

    退出码: 0 = 校验通过(有没有改动都算), 1 = 有 error, 2 = 环境不对(读不到文件)。
    """
    if not path.exists():
        print(f"[stop] 找不到 {path}")
        return 2
    loaded = load_params(path)
    lines = read_lines(path)
    block = find_block(lines)
    missing = [p.key for p in PARAMS if find_key_line(lines, block, p.key) is None]
    values = {key: loaded[key] for key in PARAM_BY_KEY if key in loaded}
    full = {key: loaded[key] for key in loaded}
    full.update(values)
    problems = validate(full, bridge_invert_rx_ry(), missing,
                        loaded.get("ik_target_invert_rx_ry"))
    print(f"参数文件: {path}")
    print(f"可编辑 {len(PARAMS)} 项; 其中 yaml 里没有的 {len(missing)} 项"
          + (f": {', '.join(missing)}" if missing else ""))
    for problem in problems:
        print("  " + problem.render())
    if not problems:
        print("  ✓ 校验通过")
    _new, diffs, _skipped = build_edit(lines, values)
    if diffs:
        print(f"与文件当前值不同的项 ({len(diffs)}):")
        for diff in diffs:
            print(f"  {diff.key}: {slice_value(diff.old_value)} -> "
                  f"{slice_value(diff.new_value)}")
    else:
        print("与文件当前值完全一致(保存也不会动它)")
    return 1 if any(p.level == "error" for p in problems) else 0


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description="H3001 抓取参数调谐界面 (只改值不动注释; 不 source ROS 也能开)")
    parser.add_argument("--yaml", default=None,
                        help="要编辑的 yaml (默认: 工作区里源码树那份)")
    parser.add_argument("--dry-run", action="store_true",
                        help="只载入 + 校验 + 打印将要写入的改动, 不写盘(也不开窗口)")
    args = parser.parse_args(argv)

    path = resolve_yaml(args.yaml)
    if args.dry_run:
        return console_check(path)
    app = TunerApp(path)
    app.mainloop()
    return 0


if __name__ == "__main__":
    sys.exit(main())
