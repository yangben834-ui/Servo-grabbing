#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""H3001 抓取控制台 —— 一个窗口把「四个终端」按顺序管起来。

用法
------
source /opt/ros/humble/setup.bash
source install/setup.bash
/usr/bin/python3 h3001_console.py

**必须 source 过 ROS**: 面板自己订阅 /feedback 话题、还要调 ~/execute_grasp 服务,
这些走的是 rclpy。不需要 conda —— 面板用系统 python3 (tkinter 与 rclpy 都在里面)。

它管什么
----------
把 README §3 那四个终端按**顺序**起起来, 并把每一条的「起没起来」显示出来:

    0. 先 `pkill` 掉厂商的 web server —— README §3.0: 它和桥抢同一条 can0,
       两边都收不全帧 (这正是「桥的解析线程静默死掉」那条已知来路)。
    1. 桥     ros2 launch h3001_bridge            → 等 /feedback/tcp_pose 有数据
    2. 感知   conda 的 python 跑 YOLO/位姿 demo    → 等 /tmp 的候选 JSON 变新
    3. 执行器 ros2 launch bottle_grasp_executor   → 等日志里出现「就绪」
    4. 触发   ~/execute_grasp 服务 (按钮) —— **这一步不自动做**, 由你点

另外有一个「调参」按钮: 打开 h3001_tuner.py (抓取参数的图形界面, 改的是
bottle_grasp_executor/config/bottle_grasp_executor.yaml)。它起的是**另一个进程** ——
本进程的 RosBridge 已经 rclpy.init() 了, 同进程再起一个 ROS 界面会冲突, 而且那个界面
刻意做成"没 source ROS 也能开"。**界面改完文件不会自动生效**: 参数分两种, 位姿/放置/
夹爪开关/话题是加载期读一次、必须重启执行器。本面板会盯着那个文件的 mtime, 变了就在
日志区打一行提醒 —— 但**绝不自动重启**(机械臂可能正夹着瓶子在跑)。

**它不替代急停按钮。** 面板上的「软停」只是不再下发新指令 + 停掉进程; 机械臂会把
**当前这一条指令走完**(指令在固件手里), 也不会失能电机。真要停下来按硬件急停。

为什么要用 start_new_session
-----------------------------
每个子进程都开**独立的进程组**, 停的时候 `killpg` 整组杀。`ros2 launch` 自己**不转发
信号**, 只杀包装进程的话节点本体还活着 (2026-09 踩过: 包装进程没了, 机械臂还在动)。
整组杀才能把 launch 底下的节点一起收掉。

日志
------
每一条都写到 logs/<名字>.log, 每次启动截断重写 —— 文件里只有本轮日志, 
窗口里显示的也是本轮。要看上一轮请趁它还在窗口里时截图/复制, 面板不留档。
"""

from __future__ import annotations

import json
import os
import re
import signal
import subprocess
import sys
import threading
import time
from pathlib import Path

import tkinter as tk
from tkinter import messagebox, ttk

# ── 路径与命令 ──────────────────────────────────────────────
# 工作区就是本文件所在目录。感知脚本在它下面, 依赖 (SDK / 手眼标定) 也在它下面 ——
# 所以所有子进程都把 cwd 设在工作区 (桥找 SDK 时也靠 cwd: 优先级是
# `H3001_SDK_PATH` > 工作区里最新的 Linux SDK > 历史路径)。
WORKSPACE = Path(__file__).resolve().parent
LOG_DIR = WORKSPACE / "log" / "console"     # colcon 用的是 log/build_*, 同级放 console 不打架
CANDIDATES_JSON = Path("/tmp/bottle_grasp_candidates.json")
# 候选 JSON 的「新鲜度」窗口(秒)。感知脚本每次写文件都带一个 stamp(unix 秒),
# 只有 stamp 在这么多秒以内的候选才算「当前这一轮」。老文件/上一轮残留不会
# 因为「candidates 字段还在」就被误判为就绪 —— 这正是起执行器前必须排除的。
CANDIDATE_FRESH_S = 3.0

ROS_SETUP = "/opt/ros/humble/setup.bash"
WS_SETUP = str(WORKSPACE / "install" / "setup.bash")
# 感知用 conda 环境里的解释器 —— 直接给绝对路径, **不 activate**:
# activate 会改 PYTHONPATH / LD_LIBRARY_PATH, 和面板自己的 ROS 环境互相污染。
CONDA_PY = "/home/zdl/miniconda3/envs/grasp/bin/python"
PERCEPTION_DIR = WORKSPACE / "wutishibiezitaijiance-master"
PERCEPTION_SCRIPT = "test_yolo_realsense_v3f.py"

# 厂商 web server 的进程特征 —— 启动前要清掉 (README §3.0)。
VENDOR_WEB_PATTERN = "h3001_web_server_linux.py"

EXECUTE_SERVICE = "/bottle_grasp_executor/execute_grasp"

# 调参界面与它改的那个文件。「调参」按钮起的是**另一个进程** (理由见 _open_tuner);
# 面板自己盯着这个文件的 mtime, 变了就提醒「要重启执行器才生效」—— 因为界面是独立进程,
# 它没法回调这个面板。
TUNER_SCRIPT = WORKSPACE / "h3001_tuner.py"
PARAMS_YAML = WORKSPACE / "bottle_grasp_executor" / "config" / "bottle_grasp_executor.yaml"


def _ros_shell(body: str, prelude: str = "") -> list[str]:
    """拼一条「先 source 再跑」的 bash -c 命令行。

    prelude 是**在 exec 之前**要执行的语句 (比如 export) —— 不能塞进 body:
    body 会被加上 `exec` 前缀 (为了 PID/进程组对得上), 而 `exec` 只接**一条命令**,
    写成 `exec export X=1 && ros2 launch ...` 会去找一个叫 export 的程序 ✗。
    """
    parts = [f"source {ROS_SETUP}", f"source {WS_SETUP}", f"cd {WORKSPACE}"]
    if prelude:
        parts.append(prelude)
    parts.append(f"exec {body}")
    return ["bash", "-c", " && ".join(parts)]


def _perception_shell(offset: str, cylinder_axis: bool) -> list[str]:
    """感知那条命令 —— **不需要 ROS** (它只往 /tmp 写 JSON, 不 import rclpy)。"""
    argv = [
        CONDA_PY, "-u", PERCEPTION_SCRIPT,
        "--camera", "orbbec",
        "--json-out", str(CANDIDATES_JSON),
        "--no-point-cloud",
        "--tracker", "ocsort",
        "--offset", *offset.split(),
    ]
    if cylinder_axis:
        argv.append("--cylinder-axis")
    quoted = " ".join(f"'{a}'" if " " in a else a for a in argv)
    return ["bash", "-c", f"cd {PERCEPTION_DIR} && exec {quoted}"]


# ── 受管的子进程 ────────────────────────────────────────────


class Proc:
    """一个被面板管着的子进程 (独立进程组 + 自己的日志文件)。"""

    def __init__(self, key: str, title: str, pattern: str):
        self.key = key
        self.title = title
        # pgrep 用的特征 —— 用来认出**不是本面板起的**同类进程。同一时刻只许有一份:
        # 桥和感知双开都会抢硬件 (can0 / USB 相机), 所以点启动前必须查。
        self.pattern = pattern
        self.log_path = LOG_DIR / f"{key}.log"
        self.popen: subprocess.Popen | None = None
        self.started_at = 0.0
        self._offset = 0          # 日志读到哪了
        self._log_handle = None

    # ── 起 ──
    def start(self, argv: list[str]) -> None:
        if self.alive():
            return
        LOG_DIR.mkdir(parents=True, exist_ok=True)
        # "wb": 每次启动截断重写。不留历史 —— 窗口里看到的永远是本轮日志。
        self._log_handle = open(self.log_path, "wb", buffering=0)
        self._log_handle.write(
            f"===== {time.strftime('%Y-%m-%d %H:%M:%S')} 启动: "
            f"{' '.join(argv)}\n".encode("utf-8")
        )
        self.popen = subprocess.Popen(
            argv,
            cwd=str(WORKSPACE),
            stdout=self._log_handle,
            stderr=subprocess.STDOUT,
            stdin=subprocess.DEVNULL,
            start_new_session=True,
        )
        self.started_at = time.monotonic()
        self._offset = 0          # 从文件头读, 连上面那行分隔也一起看到

    # ── 停 ──
    def stop(self, grace_s: float = 4.0) -> str:
        """整组杀。先 SIGINT 让它自己收尾, 超时就 SIGKILL。返回一句说明。"""
        if self.popen is None:
            return "本来就没在跑"
        if self.popen.poll() is not None:
            return "进程已经退了"
        try:
            pgid = os.getpgid(self.popen.pid)
        except OSError:
            return "拿不到进程组"
        try:
            os.killpg(pgid, signal.SIGINT)
        except OSError:
            pass
        deadline = time.monotonic() + grace_s
        while time.monotonic() < deadline:
            if self.popen.poll() is not None:
                break
            time.sleep(0.1)
        if self.popen.poll() is None:
            try:
                os.killpg(pgid, signal.SIGKILL)
            except OSError:
                pass
            self.popen.wait(timeout=3)
            return "SIGINT 没收住, 已 SIGKILL 整组"
        return "已退出"

    def alive(self) -> bool:
        return self.popen is not None and self.popen.poll() is None

    # ── 读日志 ──
    def new_lines(self) -> list[str]:
        try:
            # 文件被外部截短时, _offset 会越界 -> 从 0 重读, 否则永远读不到新行
            if self.log_path.stat().st_size < self._offset:
                self._offset = 0
            with open(self.log_path, "rb") as handle:
                handle.seek(self._offset)
                chunk = handle.read()
                self._offset = handle.tell()
        except OSError:
            return []
        if not chunk:
            return []
        return chunk.decode("utf-8", "replace").splitlines()


# ── ROS 侧 (另一个线程里跑) ─────────────────────────────────


class RosBridge(threading.Thread):
    """订阅反馈、调服务 —— 全在后台线程, 结果放共享字段给界面读。

    界面线程只读这几个标量, 不做任何 ROS 调用 (rclpy 的 spin 在**这个**线程里),
    所以窗口不会因为 ROS 卡住。
    """

    def __init__(self):
        super().__init__(daemon=True)
        self.ready = False
        self.error = ""
        self.tcp_hz = 0.0
        self.tcp_stamp = 0.0          # 最后一次收到反馈的时刻
        self.arm_status = None
        self._tcp_times: list[float] = []
        self._lock = threading.Lock()
        self._trigger = None
        self._trigger_result = ""
        self._trigger_pending = False

    def run(self):
        try:
            import rclpy
            from h3001_msgs.msg import H3001ArmStatus
            from geometry_msgs.msg import PoseStamped
            from rclpy.node import Node
            from rclpy.qos import qos_profile_sensor_data
            from std_srvs.srv import Trigger
        except Exception as exc:                      # noqa: BLE001
            self.error = f"import 失败 ({exc}) —— 先 source ROS 再跑本面板"
            return
        rclpy.init()
        node = Node("h3001_console")
        node.create_subscription(
            PoseStamped, "/feedback/tcp_pose", self._on_tcp, qos_profile_sensor_data
        )
        node.create_subscription(
            H3001ArmStatus, "/feedback/arm_status", self._on_status, qos_profile_sensor_data
        )
        self._trigger = node.create_client(Trigger, EXECUTE_SERVICE)
        self.ready = True
        try:
            rclpy.spin(node)
        finally:
            node.destroy_node()
            rclpy.shutdown()

    def _on_tcp(self, _msg):
        now = time.monotonic()
        with self._lock:
            self.tcp_stamp = now
            self._tcp_times.append(now)
            self._tcp_times = [t for t in self._tcp_times if now - t <= 1.0]
            self.tcp_hz = float(len(self._tcp_times))

    def _on_status(self, msg):
        self.arm_status = msg

    def call_trigger(self) -> None:
        """点「触发抓取」—— 异步发, 结果由 poll_trigger 取。"""
        if self._trigger is None:
            self._trigger_result = "服务客户端还没建好 (ROS 没起来?)"
            return
        with self._lock:
            if self._trigger_pending:
                return
            self._trigger_pending = True
        if not self._trigger.wait_for_service(timeout_sec=2.0):
            self._trigger_result = f"服务 {EXECUTE_SERVICE} 不在 —— 执行器起来了吗?"
            with self._lock:
                self._trigger_pending = False
            return
        from std_srvs.srv import Trigger

        request = Trigger.Request()
        future = self._trigger.call_async(request)
        future.add_done_callback(self._on_trigger_done)

    def _on_trigger_done(self, future):
        try:
            response = future.result()
            self._trigger_result = (
                f"{'成功' if response.success else '拒绝'}: {response.message}"
            )
        except Exception as exc:                      # noqa: BLE001
            self._trigger_result = f"调用异常: {exc}"
        with self._lock:
            self._trigger_pending = False

    def poll_trigger(self) -> tuple[str, bool]:
        with self._lock:
            return self._trigger_result, self._trigger_pending


# ── 界面 ────────────────────────────────────────────────────


class Console(tk.Tk):
    TICK_MS = 250

    def __init__(self):
        super().__init__()
        self.title("H3001 抓取控制台")
        self.geometry("980x640")
        # pattern 用 **节点本体的可执行文件路径**, 不用话题/包名 —— 两个理由:
        #   1) `pgrep -f` 匹配的是整条命令行。用「h3001_bridge」这种短词, 会把
        #      `ros2 launch h3001_bridge ...` 的**包装进程**也算进来; 而包装进程/node
        #      是两条命 —— 节点崩了包装还挂着时, 面板会以为「还在跑」✗;
        #   2) 短词还容易误伤: 任何命令行里提到这个词的进程都算 (实测被自己的
        #      bash 包装匹配过一次)。
        self.procs = {
            "bridge": Proc("bridge", "桥接",
                           "install/h3001_bridge/lib/h3001_bridge/h3001_bridge"),
            "perception": Proc("perception", "感知", "test_yolo_realsense_v3f.py"),
            "executor": Proc("executor", "执行器",
                             "install/bottle_grasp_executor/lib/bottle_grasp_executor/"
                             "bottle_grasp_executor"),
        }
        self._external: dict[str, bool] = {}
        self._external_at = 0.0
        self.state = {"bridge": "已停止", "perception": "已停止", "executor": "已停止"}
        self.banner = ""            # 面板级提示 (清 web server 之类的)
        self.dry_run = tk.BooleanVar(value=False)
        self.cylinder_axis = tk.BooleanVar(value=True)
        self.speed = tk.StringVar(value="10")
        self.offset = tk.StringVar(value="0.0 0.0 -0.02")
        self.trigger_msg = tk.StringVar(value="")
        self.ros = RosBridge()
        self.ros.start()
        self._exec_lines: list[str] = []
        self._pending: list[str] = []       # 「启动全部」还没起的下一批 (见 _poll_pending)
        # 参数文件的指纹: 面板打开那一刻先记下来, 之后变了才提醒 (见 _poll_params_file)
        self._params_stamp: tuple[int, int] | None = None
        # 面板打开这一刻之前写下的日志都不算「本轮」—— 把每个组件的读位置对到
        # 当前文件尾。之后:
        #   · 外部启动的执行器追加写日志 → 只读到新增部分
        #   · 面板自己 _start 启动 → "wb" 截断 + _offset=0, 从头读本轮
        for proc in self.procs.values():
            try:
                proc._offset = proc.log_path.stat().st_size
            except OSError:
                pass          # 文件不存在(首次运行): _offset 保持 0
        self._build()
        self.protocol("WM_DELETE_WINDOW", self._on_close)
        self.after(self.TICK_MS, self._tick)

    # ── 界面搭建 ──
    def _build(self):
        pad = {"padx": 8, "pady": 4}

        # 状态区
        status = ttk.LabelFrame(self, text="状态")
        status.pack(fill="x", **pad)
        self.labels = {}
        rows = [
            ("bridge", "桥接"),
            ("perception", "感知"),
            ("executor", "执行器"),
        ]
        for index, (key, title) in enumerate(rows):
            ttk.Label(status, text=title, width=8).grid(row=index, column=0, sticky="w")
            label = ttk.Label(status, text="——", width=90, anchor="w")
            label.grid(row=index, column=1, sticky="w")
            self.labels[key] = label
        self.ros_label = ttk.Label(status, text="——", width=90, anchor="w")
        ttk.Label(status, text="ROS", width=8).grid(row=3, column=0, sticky="w")
        self.ros_label.grid(row=3, column=1, sticky="w")
        self.banner_label = ttk.Label(status, text="", foreground="#b00")
        self.banner_label.grid(row=4, column=0, columnspan=2, sticky="w")

        # 按钮区
        buttons = ttk.Frame(self)
        buttons.pack(fill="x", **pad)
        ttk.Button(buttons, text="启动全部", command=self._start_all).pack(side="left")
        ttk.Button(buttons, text="全部停止", command=self._stop_all).pack(side="left", padx=6)
        ttk.Button(buttons, text="触发抓取", command=self._trigger).pack(side="left", padx=6)
        ttk.Button(buttons, text="软停执行器", command=self._soft_stop).pack(side="left", padx=6)
        ttk.Separator(buttons, orient="vertical").pack(side="left", fill="y", padx=8)
        ttk.Button(buttons, text="起/停 桥", command=lambda: self._toggle("bridge")).pack(side="left")
        ttk.Button(buttons, text="起/停 感知", command=lambda: self._toggle("perception")).pack(side="left", padx=6)
        ttk.Button(buttons, text="起/停 执行器", command=lambda: self._toggle("executor")).pack(side="left")
        ttk.Button(buttons, text="清厂商 web server", command=self._kill_vendor_web).pack(side="left", padx=6)
        ttk.Button(buttons, text="清外部进程", command=self._kill_external).pack(side="left")
        ttk.Button(buttons, text="调参", command=self._open_tuner).pack(side="left", padx=6)

        # 参数区
        params = ttk.LabelFrame(self, text="参数 (改动要重启对应的那一条才生效)")
        params.pack(fill="x", **pad)
        ttk.Checkbutton(params, text="模拟执行 (dry_run:=true)",
                        variable=self.dry_run).grid(row=0, column=0, sticky="w")
        ttk.Label(params, text="速度 speed_percent:").grid(row=0, column=1, sticky="e")
        ttk.Entry(params, textvariable=self.speed, width=6).grid(row=0, column=2, sticky="w")
        ttk.Checkbutton(params, text="圆柱轴拟合 (--cylinder-axis)",
                        variable=self.cylinder_axis).grid(row=1, column=0, sticky="w")
        ttk.Label(params, text="感知 offset (米, 基座系):").grid(row=1, column=1, sticky="e")
        ttk.Entry(params, textvariable=self.offset, width=18).grid(row=1, column=2, sticky="w")
        ttk.Label(params, textvariable=self.trigger_msg, foreground="#06c").grid(
            row=2, column=0, columnspan=3, sticky="w"
        )

        # 日志区
        logs = ttk.LabelFrame(self, text="日志 (执行器)")
        logs.pack(fill="both", expand=True, **pad)
        self.log_text = tk.Text(logs, height=18, wrap="none", background="#101418",
                                foreground="#d8dee9", insertbackground="#d8dee9")
        self.log_text.pack(side="left", fill="both", expand=True)
        scroll = ttk.Scrollbar(logs, command=self.log_text.yview)
        scroll.pack(side="right", fill="y")
        self.log_text.configure(yscrollcommand=scroll.set)

    # ── 动作 ──
    def _kill_vendor_web(self) -> bool:
        """README §3.0: 厂商 web server 和桥抢 can0, 真机前必须清掉。"""
        try:
            found = subprocess.run(
                ["pgrep", "-af", VENDOR_WEB_PATTERN],
                capture_output=True, text=True, timeout=3,
            ).stdout.strip()
        except Exception as exc:                      # noqa: BLE001
            self.banner = f"查厂商 web server 失败: {exc}"
            return True
        if not found:
            self.banner = ""
            return True
        killed = subprocess.run(["pkill", "-f", VENDOR_WEB_PATTERN], timeout=3)
        time.sleep(0.5)
        left = subprocess.run(
            ["pgrep", "-af", VENDOR_WEB_PATTERN], capture_output=True, text=True
        ).stdout.strip()
        if left:
            self.banner = f"厂商 web server 还有残留, 手动 pkill -f {VENDOR_WEB_PATTERN}"
            return False
        self.banner = ""
        self._append(f"[控制台] 已清掉厂商 web server (它和桥抢 can0, README §3.0)")
        return True

    @staticmethod
    def _alive_by_argv(pattern: str) -> bool:
        """有没有哪个进程的 **argv[0]/argv[1]** 命中这个 pattern。

        为什么不用 `pgrep -f`: 它匹配**整条命令行**, 于是「命令行里提到这个词」的进程
        全都算 —— 实测被自己的 shell 包装误判过 (包装的命令文本里含 pattern), 而那种
        假阳性会**挡住启动按钮** (面板以为组件已经在跑就不让再起)。只看前两个参数、
        并跳过 shell 本体, 这一类误伤就没有了。

        为什么要看 argv[1]: 感知那条是 `python test_yolo_xxx.py`, 脚本名在第二个参数上。
        """
        shells = {"bash", "sh", "dash", "zsh", "env"}
        me = os.getpid()
        for entry in Path("/proc").iterdir():
            if not entry.name.isdigit() or int(entry.name) == me:
                continue
            try:
                parts = (entry / "cmdline").read_bytes().split(b"\0")
            except OSError:
                continue
            if not parts or not parts[0]:
                continue
            argv0 = parts[0].decode("utf-8", "replace")
            argv1 = parts[1].decode("utf-8", "replace") if len(parts) > 1 else ""
            if os.path.basename(argv0) in shells:
                continue                      # shell 包装本身不算 (它只是"命令行里提到了")
            if pattern in argv0 or pattern in argv1:
                return True
        return False

    def _refresh_external(self) -> None:
        """认一下哪些组件正在被**面板之外**的进程占着 (每秒一次就够)。"""
        if time.monotonic() - self._external_at < 1.0:
            return
        self._external_at = time.monotonic()
        for key, proc in self.procs.items():
            if proc.alive():
                self._external[key] = False          # 本面板自己起的, 不算外部
            else:
                self._external[key] = self._alive_by_argv(proc.pattern)
    def _start(self, key: str):
        proc = self.procs[key]
        if proc.alive():
            return
        if self._external.get(key):
            # 2026-09-30 修: 这行原来被写成了 `messagebox.showwarning(...)` ——
            # Tk 回调里抛出去, 只会打到启动面板那个终端的 stderr 上, 界面上看到的就是
            # 「点了没反应」。这四处 (连同下面三处 argv) 是 09-29 15:30 那次改动留下的
            # 半截代码, 现场表现就是「打不开感知」而实际是感知早被外部进程占着。
            messagebox.showwarning(
                f"{proc.title} 已被外部进程占用",
                f"检测到**面板之外**起的 {proc.title} (特征: {proc.pattern})。\n\n"
                "同一时刻只许有一份: 桥和感知双开都会抢硬件 (can0 / USB 相机)。\n\n"
                "要交给面板管, 二选一:\n"
                "  1) 到起它的那个终端 Ctrl-C 停掉, 再点这里的启动;\n"
                "  2) 点「清外部进程」把不是本面板起的同类进程清掉, 再启动。\n\n"
                "只是想接着用那一份 (不交给面板), 就什么都不用做。",
            )
            return
        if key == "executor":
            # 文件已截断, 窗口里的也得清 —— 两边都不留上一轮
            self._exec_lines.clear()
            self.log_text.delete("1.0", "end")
        # 三条命令按「本文件头部的说明 + 两个 launch 文件声明的参数 + 界面上那几个框」
        # 还原 (2026-09-30): 原来这三处 argv 全被写成了裸 `...`, 一按按钮就 AttributeError
        # 死在 Tk 回调里 —— 三个「起/停」按钮其实全都起不来。
        if key == "bridge":
            # speed_percent 必须从界面给: 桥的 launch 参数字典**排在 yaml 之后**,
            # 不给就会用 launch 里的默认值盖掉 yaml (§3.1)。启动行会打 `速度=N%`。
            argv = _ros_shell(
                "ros2 launch h3001_bridge h3001_bridge.launch.py "
                f"speed_percent:={self.speed.get()}"
            )
        elif key == "perception":
            argv = _perception_shell(self.offset.get(), bool(self.cylinder_axis.get()))
        else:
            argv = _ros_shell(
                "ros2 launch bottle_grasp_executor bottle_grasp_executor.launch.py "
                f"dry_run:={str(bool(self.dry_run.get())).lower()}"
            )
        proc.start(argv)
        self._append(f"[控制台] 启动{proc.title}: {' '.join(argv)}")

    def _toggle(self, key: str):
        proc = self.procs[key]
        if proc.alive():
            self._append(f"[控制台] 停{proc.title}: {proc.stop()}")
        else:
            self._start(key)

    def _start_all(self):
        if not self._kill_vendor_web():
            return
        try:
            CANDIDATES_JSON.unlink(missing_ok=True)
        except OSError as exc:
            self._append(f"[控制台] 清旧候选 JSON 失败 ({exc}) —— 用时间戳校验兜底")
        self._start("bridge")
        self._pending = ["perception", "executor"]
        self._append("[控制台] 按「桥 → 感知 → 执行器」的顺序启动, 每一步等就绪")

    def _stop_all(self):
        self._pending = []
        for key in ("executor", "perception", "bridge"):   # 倒序停
            proc = self.procs[key]
            if proc.alive():
                self._append(f"[控制台] 停{proc.title}: {proc.stop()}")

    def _kill_external(self):
        """把不是本面板起的同类进程清掉 (接管现场时用)。"""
        self._refresh_external()
        targets = [k for k, v in self._external.items() if v]
        if not targets:
            messagebox.showinfo("清外部进程", "没有检测到外部启动的同类进程。")
            return
        names = "、".join(self.procs[k].title for k in targets)
        if not messagebox.askyesno(
            "清外部进程",
            f"会 SIGTERM 掉这些外部启动的进程: {names}\n\n"
            "注意: 执行器被停掉时机械臂会把**当前这一条**指令走完 (指令在固件手里)。\n"
            "继续吗?",
        ):
            return
        for key in targets:
            # 这里可以用 pkill -f: 要杀的就是「命令行里提到它」的整棵树 (launch 包装
            # 和它底下的节点都要走)。代价是可能误伤, 所以只在人点了按钮之后做。
            pattern = self.procs[key].pattern
            subprocess.run(["pkill", "-f", pattern], timeout=3)
            self._append(f"[控制台] 已 pkill 外部进程: {pattern}")
        self._external_at = 0.0
        self._refresh_external()

    def _soft_stop(self):
        """软停执行器: 不再下发新指令。**机械臂会把当前这一条走完** —— 指令在固件手里。"""
        if not messagebox.askyesno(
            "软停执行器",
            "停掉执行器之后:\n"
            "  · 不会再下发新的运动指令\n"
            "  · 机械臂会把**当前这一条**指令走完 (指令在固件手里)\n"
            "  · 电机不失能, 夹爪保持现状\n\n"
            "真要立刻停下来, 请按硬件急停。\n\n继续吗?",
        ):
            return
        proc = self.procs["executor"]
        if proc.alive():
            self._append(f"[控制台] 软停执行器: {proc.stop()}")

    def _trigger(self):
        self.ros.call_trigger()
        self._append("[控制台] 已请求 ~/execute_grasp")

    def _open_tuner(self):
        """打开调参界面 (h3001_tuner.py)。

        **另起一个进程**, 不做成这个窗口的子窗口 —— 两个理由:
          · 本进程的 RosBridge 已经 `rclpy.init()` 并且 spin 着, 同进程再 init 一次会冲突;
          · 调参界面刻意做成"没 source ROS 也能开"(只有「抄当前末端」需要 ROS), 而本面板
            必须 source 过 ROS 才能起 —— 挂在它下面等于把这个性质丢掉。
        `start_new_session` 是为了让它不跟着本终端的 Ctrl-C 一起死 (它是个独立工具窗口)。

        代价是它摸不到这里的「起/停 执行器」, 所以由面板盯着文件的 mtime 提醒重启
        (见 _poll_params_file)。
        """
        if not TUNER_SCRIPT.exists():
            messagebox.showerror("调参", f"找不到 {TUNER_SCRIPT}")
            return
        if self._tuner_alive():
            self._append("[控制台] 调参界面已经开着 —— 那是个独立窗口, 到任务栏/其他桌面找它")
            return
        # sys.executable: 本面板就是这么起来的 (README 里那条命令用的是系统 python3,
        # tkinter 与 rclpy 都在里面), 别写死 "python3" —— 那可能是 conda 那个。
        subprocess.Popen([sys.executable, str(TUNER_SCRIPT)], cwd=str(WORKSPACE),
                         start_new_session=True)
        self._append(f"[控制台] 已打开调参界面 ({TUNER_SCRIPT.name}): "
                     "改完在这里点「起/停 执行器」重启才生效")

    def _tuner_alive(self) -> bool:
        """调参界面是不是已经开着(不管是不是本面板起的)。"""
        try:
            return subprocess.run(["pgrep", "-f", TUNER_SCRIPT.name],
                                  capture_output=True, timeout=3).returncode == 0
        except Exception:                                   # noqa: BLE001
            return False

    def _on_close(self):
        if any(p.alive() for p in self.procs.values()):
            if not messagebox.askyesno(
                "退出", "还有进程在跑。退出会把它们都停掉 (整组杀), 继续吗?"
            ):
                return
        self._stop_all()
        self.destroy()

    # ── 每拍刷新 ──
    def _tick(self):
        try:
            self._refresh_external()
            self._poll_procs()
            self._poll_ros()
            self._poll_pending()
            self._poll_log()
            self._poll_params_file()
        finally:
            self.after(self.TICK_MS, self._tick)

    def _poll_params_file(self):
        """参数文件被改过就在日志里说一声。

        调参界面是**另一个进程**, 它改完文件没法回调这个面板; 而它改的那些键里有一半
        (两个位姿、手眼标定、夹爪开关)是**加载期读一次**的 —— 不重启执行器就等于没改。
        面板这里只提醒, **绝不自动重启**: 机械臂可能正夹着瓶子在跑, 那一下会要命。
        """
        try:
            stat = PARAMS_YAML.stat()
        except OSError:
            return
        stamp = (stat.st_mtime_ns, stat.st_size)
        if self._params_stamp is None:
            self._params_stamp = stamp          # 面板打开这一刻不算"改动"
            return
        if stamp == self._params_stamp:
            return
        self._params_stamp = stamp
        self._append(
            f"[控制台] {PARAMS_YAML.name} 已改动 —— **位姿 / 放置 / 夹爪开关 / 话题**这些是"
            "加载期读一次, 要重启执行器才生效 (点「起/停 执行器」); "
            "运行期现读的那些 (高度、dwell、跟随参数) 也要等下一次任务才用新值。"
        )

    def _poll_procs(self):
        for key, proc in self.procs.items():
            if proc.alive():
                mark = "● 运行中 (本面板)"
            elif self._external.get(key):
                mark = "◐ 外部启动 (不是本面板起的)"
            else:
                mark = "○ 已停止"
            if key == "bridge":
                detail = (
                    f"反馈 {self.ros.tcp_hz:.0f} Hz, 最后一次 "
                    f"{time.monotonic() - self.ros.tcp_stamp:.1f}s 前"
                    if self.ros.tcp_stamp else "还没收到 /feedback/tcp_pose"
                )
            elif key == "perception":
                detail = self._candidates_summary()
            else:
                detail = self._executor_summary()
            self.labels[key].configure(text=f"{mark}   {detail}")

    def _candidates_summary(self) -> str:
        try:
            payload = json.loads(CANDIDATES_JSON.read_text())
        except Exception:                             # noqa: BLE001
            return "还没有候选 JSON"
        age = time.time() - float(payload.get("stamp", 0.0))
        items = payload.get("candidates", [])
        extra = ""
        if items:
            # 圆柱轴拟合的半径在 axis_fit 里, **单位是米** (感知脚本自己乘 1000 打日志)。
            fit = items[0].get("axis_fit") or {}
            radius = fit.get("radius")
            if fit.get("ok") and isinstance(radius, (int, float)):
                extra = f", Φ{radius * 2000.0:.0f}mm"      # 半径 -> 直径 mm
        return f"候选 {len(items)} 个{extra}, {age:.1f}s 前更新"

    def _executor_summary(self) -> str:
        text = "".join(self._exec_lines)
        state = "?"
        match = re.findall(r"状态: (\S+) -> (\S+)", text)
        if match:
            state = match[-1][1]
        ik = "求逆解: ——"
        if "上位机求逆解: 开" in text:
            ik = "求逆解: 已启用"
            if "运行期自检不通过" in text:
                ik = "求逆解: **自检不通过, 已降级回 move_p**"
        elif "上位机求逆解: 关" in text:
            ik = "求逆解: 关"
        # 循环开过又关掉时两种字样都在, 所以取**最后**一次出现的那种
        loop = "循环: ——"
        opened = text.rfind("循环模式已开启")
        closed = max(text.rfind("循环模式已关闭"), text.rfind("循环模式已自动关闭"))
        if opened >= 0 or closed >= 0:
            loop = "循环: 开" if opened > closed else "循环: 关"
        return f"状态 {state}, {ik}, {loop}"

    def _poll_ros(self):
        if not self.ros.ready:
            self.ros_label.configure(text=self.ros.error or "正在等 ROS 起来...")
            return
        status = self.ros.arm_status
        mode = "——"
        if status is not None:
            mode = (f"控制模式 {status.ctrl_mode}, ik_status {status.ik_status}, "
                    f"故障 {status.err_status}")
        self.ros_label.configure(text=mode)
        message, _pending = self.ros.poll_trigger()
        if message:
            self.trigger_msg.set(message)

    def _poll_pending(self):
        """启动全部: 按就绪条件一步步往下推 (每一步都不阻塞界面)。"""
        if not getattr(self, "_pending", None):
            return
        key = self._pending[0]
        ready = False
        if key == "perception":
            ready = self.ros.tcp_stamp > 0          # 桥有反馈了才起感知
            if not ready and time.monotonic() - self.procs["bridge"].started_at > 15:
                self._append("[控制台] 等了 15s 没等到 /feedback/tcp_pose —— "
                             "桥起不来? 先看它的日志 (logs/console/bridge.log)")
                self._pending = []
                return
        elif key == "executor":
            ready = self.procs["perception"].alive() and self._candidates_ok()
        if ready:
            self._start(key)
            self._pending.pop(0)

    def _candidates_ok(self) -> bool:
        """候选 JSON 存在、有内容、且 **stamp 是新的** —— 三者都满足才算就绪。

        只看「有没有 candidates 字段」是危险的: 上一轮跑完留下的文件会让执行器
        在感知还没出结果时就被拉起。所以必须校验时间戳。
        """
        try:
            payload = json.loads(CANDIDATES_JSON.read_text())
        except Exception:                             # noqa: BLE001
            return False
        if not payload.get("candidates"):
            return False
        try:
            age = time.time() - float(payload.get("stamp", 0.0))
        except (TypeError, ValueError):
            return False
        # age < 0: 系统时钟被往回调过, 不可信, 当作不新鲜。
        return 0.0 <= age <= CANDIDATE_FRESH_S

    def _poll_log(self):
        """把执行器的日志增量贴到窗口里 (只贴执行器 —— 排障先看它)。"""
        proc = self.procs["executor"]
        for line in proc.new_lines():
            self._exec_lines.append(line)
            if len(self._exec_lines) > 400:
                self._exec_lines = self._exec_lines[-400:]
            self._append(line)

    def _append(self, line: str):
        self.log_text.insert("end", line + "\n")
        self.log_text.see("end")


def main() -> int:
    if not Path(ROS_SETUP).exists():
        print(f"找不到 {ROS_SETUP} —— 本面板需要 ROS Humble", file=sys.stderr)
        return 2
    Console().mainloop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
