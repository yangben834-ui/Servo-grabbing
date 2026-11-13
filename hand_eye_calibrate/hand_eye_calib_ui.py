#!/usr/bin/env python3
"""
手眼标定 UI 工具
- 默认相机：Orbbec (ROS2)
- 支持 RealSense 和 Orbbec (ROS2) 相机
- 支持新建数据文件夹
- 支持指定文件夹采集（继续 or 清空重采）
- 标定目录 / 验证目录 独立配置
"""

import os
import shlex
import shutil
import signal
import subprocess
import threading
import tkinter as tk
from tkinter import ttk, filedialog, messagebox, simpledialog, scrolledtext


class HandEyeCalibUI:
    def __init__(self, root):
        self.root = root
        self.root.title("手眼标定工具")
        self.root.geometry("1000x800")
        self.root.minsize(860, 680)

        default_work = os.path.expanduser("~/hand_eye_calibrate")
        if not os.path.isdir(default_work):
            default_work = os.getcwd()

        self.camera_process = None
        self.task_process = None

        self.work_dir_var      = tk.StringVar(value=default_work)
        self.data_dir_var      = tk.StringVar(value="./H3003")
        self.validate_dir_var  = tk.StringVar(value="")   # 留空则用 data_dir
        self.camera_var        = tk.StringVar(value="Orbbec (ROS2)")  # ← 默认 Orbbec
        self.mode_var          = tk.StringVar(value="eye_to_hand")
        self.status_var        = tk.StringVar(value="就绪")
        self.data_info_var     = tk.StringVar(value="")
        self.validate_info_var = tk.StringVar(value="")

        self._build_ui()
        self._on_camera_change()
        self._refresh_data_info()
        self._refresh_validate_info()

    # ==================================================================
    # UI 构建
    # ==================================================================
    def _build_ui(self):
        # ---------- 配置区 ----------
        cfg = ttk.LabelFrame(self.root, text="配置", padding=10)
        cfg.pack(fill="x", padx=10, pady=(10, 5))
        cfg.columnconfigure(1, weight=1)
        cfg.columnconfigure(3, weight=1)

        # 第 0 行：相机 / 模式
        ttk.Label(cfg, text="相机类型:").grid(row=0, column=0, sticky="w", padx=(0, 5), pady=4)
        self.camera_combo = ttk.Combobox(
            cfg, textvariable=self.camera_var,
            values=["Orbbec (ROS2)", "RealSense"],   # 顺序上把 Orbbec 放前
            state="readonly", width=18,
        )
        self.camera_combo.grid(row=0, column=1, sticky="w", padx=(0, 15), pady=4)
        self.camera_combo.bind("<<ComboboxSelected>>", self._on_camera_change)

        ttk.Label(cfg, text="标定模式:").grid(row=0, column=2, sticky="w", padx=(0, 5), pady=4)
        ttk.Combobox(
            cfg, textvariable=self.mode_var,
            values=["eye_to_hand", "eye_in_hand"],
            state="readonly", width=18,
        ).grid(row=0, column=3, sticky="w", padx=(0, 15), pady=4)

        # 第 1 行：工作目录
        ttk.Label(cfg, text="工作目录:").grid(row=1, column=0, sticky="w", padx=(0, 5), pady=4)
        ttk.Entry(cfg, textvariable=self.work_dir_var).grid(
            row=1, column=1, columnspan=2, sticky="we", padx=(0, 5), pady=4)
        ttk.Button(cfg, text="浏览...", command=self._browse_work_dir, width=8).grid(
            row=1, column=3, sticky="w", padx=(0, 15), pady=4)

        # 第 2 行：数据目录
        ttk.Label(cfg, text="数据目录:").grid(row=2, column=0, sticky="w", padx=(0, 5), pady=4)
        ttk.Entry(cfg, textvariable=self.data_dir_var).grid(
            row=2, column=1, sticky="we", padx=(0, 5), pady=4)
        self.data_dir_var.trace_add("write", lambda *_: self._refresh_data_info())

        data_btns = ttk.Frame(cfg)
        data_btns.grid(row=2, column=2, columnspan=2, sticky="w", padx=(0, 15), pady=4)
        ttk.Button(data_btns, text="浏览...", width=8,
                   command=self._browse_data_dir).pack(side="left", padx=(0, 4))
        ttk.Button(data_btns, text="新建文件夹...", width=14,
                   command=self.create_new_data_folder).pack(side="left", padx=(0, 4))
        ttk.Button(data_btns, text="打开", width=6,
                   command=self._open_data_dir).pack(side="left")

        # 第 3 行：数据目录状态
        self.data_info_label = ttk.Label(
            cfg, textvariable=self.data_info_var,
            foreground="#666666", font=("TkDefaultFont", 9))
        self.data_info_label.grid(row=3, column=1, columnspan=3, sticky="w",
                                  padx=(0, 5), pady=(0, 4))

        # 第 4 行：验证目录
        ttk.Label(cfg, text="验证目录:").grid(row=4, column=0, sticky="w", padx=(0, 5), pady=4)
        ttk.Entry(cfg, textvariable=self.validate_dir_var).grid(
            row=4, column=1, sticky="we", padx=(0, 5), pady=4)
        self.validate_dir_var.trace_add("write", lambda *_: self._refresh_validate_info())

        validate_btns = ttk.Frame(cfg)
        validate_btns.grid(row=4, column=2, columnspan=2, sticky="w", padx=(0, 15), pady=4)
        ttk.Button(validate_btns, text="浏览...", width=8,
                   command=self._browse_validate_dir).pack(side="left", padx=(0, 4))
        ttk.Button(validate_btns, text="用数据目录", width=12,
                   command=self._use_data_dir_as_validate).pack(side="left", padx=(0, 4))
        ttk.Button(validate_btns, text="打开", width=6,
                   command=self._open_validate_dir).pack(side="left")

        # 第 5 行：验证目录状态
        self.validate_info_label = ttk.Label(
            cfg, textvariable=self.validate_info_var,
            foreground="#666666", font=("TkDefaultFont", 9))
        self.validate_info_label.grid(row=5, column=1, columnspan=3, sticky="w",
                                      padx=(0, 5), pady=(0, 2))

        # ---------- 操作区 ----------
        ops = ttk.LabelFrame(self.root, text="操作步骤", padding=10)
        ops.pack(fill="x", padx=10, pady=5)
        for i in range(3):
            ops.columnconfigure(i, weight=1)

        self.btn_start_cam = ttk.Button(
            ops, text="① 启动相机节点 (Orbbec)", command=self.start_camera_node)
        self.btn_start_cam.grid(row=0, column=0, sticky="we", padx=4, pady=4)

        self.btn_stop_cam = ttk.Button(
            ops, text="停止相机节点", command=self.stop_camera_node, state="disabled")
        self.btn_stop_cam.grid(row=0, column=1, sticky="we", padx=4, pady=4)

        self.btn_collect = ttk.Button(
            ops, text="② 采集数据", command=self.collect_data)
        self.btn_collect.grid(row=1, column=0, sticky="we", padx=4, pady=4)

        self.btn_calib = ttk.Button(
            ops, text="③ 执行标定", command=self.run_calibration)
        self.btn_calib.grid(row=1, column=1, sticky="we", padx=4, pady=4)

        self.btn_validate = ttk.Button(
            ops, text="④ 验证标定", command=self.validate_calibration)
        self.btn_validate.grid(row=1, column=2, sticky="we", padx=4, pady=4)

        # ---------- 日志区 ----------
        log_frame = ttk.LabelFrame(self.root, text="日志输出", padding=5)
        log_frame.pack(fill="both", expand=True, padx=10, pady=(5, 0))

        self.log_text = scrolledtext.ScrolledText(
            log_frame, wrap="word", height=18,
            bg="#1e1e1e", fg="#d4d4d4", insertbackground="#ffffff",
            font=("Consolas", 10),
        )
        self.log_text.pack(fill="both", expand=True)

        log_bar = ttk.Frame(self.root)
        log_bar.pack(fill="x", padx=10, pady=(2, 0))
        ttk.Button(log_bar, text="清空日志", command=self._clear_log).pack(side="right")

        # ---------- 状态栏 ----------
        status_bar = ttk.Label(self.root, textvariable=self.status_var,
                               relief="sunken", anchor="w", padding=(8, 3))
        status_bar.pack(fill="x", side="bottom")

    # ==================================================================
    # 路径解析
    # ==================================================================
    def _resolve(self, p):
        p = (p or "").strip()
        if not p:
            return ""
        if not os.path.isabs(p):
            p = os.path.join(self.work_dir_var.get(), p)
        return os.path.normpath(p)

    def _resolve_data_dir(self):
        return self._resolve(self.data_dir_var.get())

    def _resolve_validate_dir(self):
        v = self.validate_dir_var.get().strip()
        if not v:
            return self._resolve_data_dir()
        return self._resolve(v)

    # ==================================================================
    # 状态提示
    # ==================================================================
    def _count_entries(self, path):
        if not os.path.isdir(path):
            return 0
        try:
            return len([f for f in os.listdir(path) if not f.startswith('.')])
        except Exception:
            return 0

    def _refresh_data_info(self):
        path = self._resolve_data_dir()
        if not path:
            self.data_info_var.set("")
            self.data_info_label.config(foreground="#666666")
            return
        if not os.path.exists(path):
            self.data_info_var.set("ℹ 目录不存在，采集时将自动创建")
            self.data_info_label.config(foreground="#888888")
            return
        if not os.path.isdir(path):
            self.data_info_var.set("⚠ 路径存在但不是目录")
            self.data_info_label.config(foreground="#cc0000")
            return
        n = self._count_entries(path)
        if n == 0:
            self.data_info_var.set(f"✔ 目录为空，可用于采集  ({path})")
            self.data_info_label.config(foreground="#009900")
        else:
            self.data_info_var.set(f"⚠ 目录已有 {n} 项，采集时会提示“继续 / 清空”  ({path})")
            self.data_info_label.config(foreground="#cc6600")

    def _refresh_validate_info(self):
        raw = self.validate_dir_var.get().strip()
        if not raw:
            path = self._resolve_data_dir()
            self.validate_info_var.set(f"ℹ 留空 → 使用数据目录: {path}")
            self.validate_info_label.config(foreground="#666666")
            return
        path = self._resolve_validate_dir()
        if not os.path.isdir(path):
            self.validate_info_var.set(f"⚠ 验证目录不存在: {path}")
            self.validate_info_label.config(foreground="#cc0000")
        else:
            n = self._count_entries(path)
            self.validate_info_var.set(f"✔ 验证目录存在，{n} 项  ({path})")
            self.validate_info_label.config(foreground="#009900")

    # ==================================================================
    # 浏览 / 打开
    # ==================================================================
    def _browse_work_dir(self):
        d = filedialog.askdirectory(initialdir=self.work_dir_var.get() or os.getcwd(),
                                    title="选择工作目录")
        if d:
            self.work_dir_var.set(d)

    def _browse_data_dir(self):
        cur = self._resolve_data_dir() or os.getcwd()
        d = filedialog.askdirectory(initialdir=cur, title="选择数据文件夹")
        if d:
            self.data_dir_var.set(d)
            self.log(f"[选择] 数据目录: {d}")
            self._refresh_data_info()
            self._refresh_validate_info()

    def _browse_validate_dir(self):
        cur = self._resolve_validate_dir() or os.getcwd()
        d = filedialog.askdirectory(initialdir=cur, title="选择验证数据文件夹")
        if d:
            self.validate_dir_var.set(d)
            self.log(f"[选择] 验证目录: {d}")
            self._refresh_validate_info()

    def _use_data_dir_as_validate(self):
        self.validate_dir_var.set(self.data_dir_var.get())
        self._refresh_validate_info()

    def _open_data_dir(self):
        d = self._resolve_data_dir()
        if os.path.isdir(d):
            subprocess.Popen(["xdg-open", d])
        else:
            messagebox.showwarning("提示", f"数据目录不存在: {d}")

    def _open_validate_dir(self):
        d = self._resolve_validate_dir()
        if os.path.isdir(d):
            subprocess.Popen(["xdg-open", d])
        else:
            messagebox.showwarning("提示", f"验证目录不存在: {d}")

    # ==================================================================
    # 新建数据文件夹
    # ==================================================================
    def create_new_data_folder(self):
        base = self.work_dir_var.get() or os.getcwd()
        name = simpledialog.askstring(
            "新建数据文件夹",
            "请输入文件夹名称（可输入相对工作目录的名称，也可输入绝对路径）：\n\n"
            f"工作目录：{base}",
            parent=self.root,
        )
        if name is None:
            return
        name = name.strip()
        if not name:
            return

        if os.path.isabs(name):
            target = os.path.normpath(name)
        else:
            target = os.path.normpath(os.path.join(base, name))

        if os.path.exists(target):
            if not os.path.isdir(target):
                messagebox.showerror("错误", f"路径已存在且不是目录：\n{target}")
                return
            if not messagebox.askyesno(
                    "文件夹已存在",
                    f"文件夹已存在：\n{target}\n\n是否直接使用该文件夹？"):
                return
        else:
            try:
                os.makedirs(target, exist_ok=True)
                self.log(f"[新建] 数据文件夹: {target}")
            except Exception as e:
                messagebox.showerror("错误", f"创建失败:\n{e}")
                return

        self.data_dir_var.set(target)
        self._refresh_data_info()
        self._refresh_validate_info()
        self.set_status(f"已选择数据文件夹: {target}")

    # ==================================================================
    # 数据目录准备（继续 / 清空）
    # ==================================================================
    def _prepare_data_dir(self, data_dir):
        if not os.path.isdir(data_dir):
            try:
                os.makedirs(data_dir, exist_ok=True)
                self.log(f"[新建] 数据目录: {data_dir}")
            except Exception as e:
                messagebox.showerror("错误", f"无法创建数据目录:\n{e}")
                return None
            return 'fresh'

        count = self._count_entries(data_dir)
        if count == 0:
            self.log(f"[信息] 数据目录为空: {data_dir}")
            return 'fresh'

        choice = self._ask_collect_mode(data_dir, count)
        if choice is None:
            self.log("[取消] 用户取消了采集")
            return None

        if choice == 'fresh':
            try:
                for f in os.listdir(data_dir):
                    if f.startswith('.'):
                        continue
                    p = os.path.join(data_dir, f)
                    if os.path.isdir(p) and not os.path.islink(p):
                        shutil.rmtree(p)
                    else:
                        os.remove(p)
                self.log(f"[清空] 已清空目录内容: {data_dir}")
            except Exception as e:
                messagebox.showerror("错误", f"清空目录失败:\n{e}")
                return None

        self._refresh_data_info()
        return choice

    def _ask_collect_mode(self, data_dir, count):
        dlg = tk.Toplevel(self.root)
        dlg.title("数据目录非空")
        dlg.transient(self.root)
        dlg.resizable(False, False)
        dlg.grab_set()

        result = {'value': None}
        content = ttk.Frame(dlg, padding=20)
        content.pack(fill="both", expand=True)

        ttk.Label(content, text="⚠  数据目录中已存在数据",
                  font=("TkDefaultFont", 11, "bold")).pack(anchor="w")
        ttk.Label(content, text=f"目录：{data_dir}").pack(anchor="w", pady=(10, 0))
        ttk.Label(content, text=f"已有条目：{count} 项").pack(anchor="w")
        ttk.Label(content, text="\n请选择操作方式：").pack(anchor="w", pady=(10, 0))

        btn_frame = ttk.Frame(content)
        btn_frame.pack(pady=(10, 0))

        def choose(v):
            result['value'] = v
            dlg.destroy()

        ttk.Button(btn_frame, text="继续采集", width=12,
                   command=lambda: choose('continue')).pack(side="left", padx=4)
        ttk.Button(btn_frame, text="清空后重采", width=12,
                   command=lambda: choose('fresh')).pack(side="left", padx=4)
        ttk.Button(btn_frame, text="取消", width=12,
                   command=lambda: choose(None)).pack(side="left", padx=4)

        dlg.update_idletasks()
        px = self.root.winfo_rootx() + (self.root.winfo_width() - dlg.winfo_width()) // 2
        py = self.root.winfo_rooty() + (self.root.winfo_height() - dlg.winfo_height()) // 2
        dlg.geometry(f"+{max(px, 0)}+{max(py, 0)}")
        dlg.protocol("WM_DELETE_WINDOW", lambda: choose(None))

        dlg.wait_window()
        return result['value']

    # ==================================================================
    # 通用
    # ==================================================================
    def _on_camera_change(self, event=None):
        if self.camera_var.get() == "RealSense":
            self.btn_start_cam.config(state="disabled")
            self.btn_stop_cam.config(state="disabled")
        else:
            self.btn_start_cam.config(
                state="normal" if self.camera_process is None else "disabled")
            self.btn_stop_cam.config(
                state="normal" if self.camera_process is not None else "disabled")

    def _clear_log(self):
        self.log_text.delete("1.0", "end")

    def log(self, msg):
        self.log_text.insert("end", str(msg) + "\n")
        self.log_text.see("end")

    def set_status(self, text):
        self.status_var.set(text)

    def _child_env(self):
        """子进程环境：强制 Python 不缓冲 stdout，日志才能实时出来。

        为什么必须加：子进程的 stdout 是**管道**（不是 tty），Python 此时默认
        走块缓冲——攒够 4~8KB 或进程退出时才真正写出来。采集脚本在采集过程中
        打的那些 `print()`（已采集第 N 组、[门控] …）就全卡在子进程的缓冲区里，
        日志区一直空白，直到脚本结束才一次性刷一大片。看起来就像"看不到日志"。
        PYTHONUNBUFFERED=1 让每次 print 立即写出。ros2 launch 之类的子进程
        也一并受益。
        """
        env = dict(os.environ)
        env["PYTHONUNBUFFERED"] = "1"
        return env

    def _run(self, cmd, cwd=None, tag=""):
        self.log(f"\n[执行] {cmd}\n")
        self.set_status(f"{tag} 执行中...")

        def worker():
            try:
                proc = subprocess.Popen(
                    cmd, shell=True, executable="/bin/bash",
                    stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                    text=True, bufsize=1, cwd=cwd,
                    preexec_fn=os.setsid,
                    env=self._child_env(),
                )
                self.task_process = proc
                for line in iter(proc.stdout.readline, ""):
                    if line:
                        self.root.after(0, self.log, line.rstrip())
                proc.wait()
                rc = proc.returncode
                self.root.after(0, self.log, f"[完成] {tag} 退出码: {rc}")
                self.root.after(0, self.set_status, f"{tag} 完成 (退出码 {rc})")
                if tag == "数据采集":
                    self.root.after(0, self._refresh_data_info)
            except Exception as e:
                self.root.after(0, self.log, f"[错误] {e}")
                self.root.after(0, self.set_status, f"错误: {e}")
            finally:
                self.task_process = None

        threading.Thread(target=worker, daemon=True).start()

    # ==================================================================
    # 步骤 1：启动 Orbbec 相机
    # ==================================================================
    def start_camera_node(self):
        if self.camera_process is not None:
            messagebox.showwarning("提示", "相机节点已在运行")
            return

        cmd = ("source /opt/ros/humble/setup.bash && "
               "source ~/orbbec_ws/install/setup.bash && "
               "ros2 launch orbbec_camera gemini_330_series.launch.py "
               "depth_registration:=true align_target_stream:=COLOR "
               "enable_point_cloud:=false")

        self.log("\n[执行] 启动 Orbbec 相机节点...\n")
        self.set_status("相机节点启动中...")

        def worker():
            try:
                proc = subprocess.Popen(
                    cmd, shell=True, executable="/bin/bash",
                    stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                    text=True, bufsize=1,
                    preexec_fn=os.setsid,
                    env=self._child_env(),
                )
                self.camera_process = proc
                for line in iter(proc.stdout.readline, ""):
                    if line:
                        self.root.after(0, self.log, line.rstrip())
                proc.wait()
                self.root.after(0, self.log, "[信息] 相机节点已退出")
            except Exception as e:
                self.root.after(0, self.log, f"[错误] 相机节点: {e}")
            finally:
                self.camera_process = None
                self.root.after(0, self._on_camera_change)
                self.root.after(0, self.set_status, "相机节点已停止")

        threading.Thread(target=worker, daemon=True).start()
        self.btn_start_cam.config(state="disabled")
        self.btn_stop_cam.config(state="normal")
        self.set_status("相机节点运行中...")

    def stop_camera_node(self):
        if self.camera_process is None:
            return
        try:
            os.killpg(os.getpgid(self.camera_process.pid), signal.SIGINT)
            self.log("[信息] 已发送 SIGINT 至相机节点")
        except Exception as e:
            self.log(f"[错误] 停止相机节点: {e}")

    # ==================================================================
    # 步骤 2：采集数据
    # ==================================================================
    def collect_data(self):
        work_dir = self.work_dir_var.get()
        if not os.path.isdir(work_dir):
            messagebox.showerror("错误", f"工作目录不存在: {work_dir}")
            return

        data_dir = self._resolve_data_dir()
        if not data_dir:
            messagebox.showerror("错误", "请先指定数据目录")
            return

        mode = self.mode_var.get()
        cam = self.camera_var.get()

        action = self._prepare_data_dir(data_dir)
        if action is None:
            return

        mode_txt = "继续采集" if action == 'continue' else "全新采集"
        self.log(f"[模式] {mode_txt}  →  {data_dir}")
        self.log("[提示] 采集的图像窗口会单独弹出。键盘事件在那个图像窗口上，"
                 "请先把焦点点过去，再按 s 存图、按 q 退出。")
        self.log("[提示] 采集过程中这里会实时打印：已采集第 N 组、位姿、以及每张的"
                 "[门控] 结论。")
        self.log("[提示] 自洽性门控：每采一张都会自查标定板是否还在法兰原位。"
                 "板动过或检不出棋盘时，会自动删掉这一张、**编号不前进**，"
                 "等你把板夹回去后重新按 s 采同一编号（不会整轮退出）。")
        self.log("[提示] 停稳检查：机械臂没停稳就不存图（会打印 [跳过] 并让你重按 s）。")

        if cam == "RealSense":
            cmd = (f"cd {shlex.quote(work_dir)} && "
                   f"python3 data_collect.py "
                   f"--mode {mode} --data-dir {shlex.quote(data_dir)}")
        else:
            cmd = (f"source /opt/ros/humble/setup.bash && "
                   f"cd {shlex.quote(work_dir)} && "
                   f"/usr/bin/python3 data_collect_orbbec_ros.py "
                   f"--mode {mode} --data-dir {shlex.quote(data_dir)}")

        self._run(cmd, tag="数据采集")

    # ==================================================================
    # 步骤 3：执行标定
    # ==================================================================
    def run_calibration(self):
        work_dir = self.work_dir_var.get()
        data_dir = self._resolve_data_dir()
        mode = self.mode_var.get()

        if not os.path.isdir(work_dir):
            messagebox.showerror("错误", f"工作目录不存在: {work_dir}")
            return
        if not os.path.isdir(data_dir):
            messagebox.showerror("错误", f"数据目录不存在: {data_dir}")
            return

        cmd = (f"conda deactivate 2>/dev/null; "
               f"source /opt/ros/humble/setup.bash && "
               f"cd {shlex.quote(work_dir)} && "
               f"/usr/bin/python3 hand_eye_calibrate.py "
               f"--mode {mode} --data-dir {shlex.quote(data_dir)}")

        self._run(cmd, tag="手眼标定")

    # ==================================================================
    # 步骤 4：验证标定（--calib-dir + --validate-dir）
    # ==================================================================
    def validate_calibration(self):
        work_dir     = self.work_dir_var.get()
        calib_dir    = self._resolve_data_dir()
        validate_dir = self._resolve_validate_dir()
        mode         = self.mode_var.get()

        if not os.path.isdir(work_dir):
            messagebox.showerror("错误", f"工作目录不存在: {work_dir}")
            return
        if not os.path.isdir(calib_dir):
            messagebox.showerror("错误", f"标定目录（calib-dir）不存在: {calib_dir}")
            return
        if not os.path.isdir(validate_dir):
            messagebox.showerror("错误", f"验证目录（validate-dir）不存在: {validate_dir}")
            return

        cmd = (f"cd {shlex.quote(work_dir)} && "
               f"python3 validate_calibration.py "
               f"--calib-dir {shlex.quote(calib_dir)} "
               f"--validate-dir {shlex.quote(validate_dir)} "
               f"--mode {mode}")

        self.log(f"[验证] calib-dir    = {calib_dir}")
        self.log(f"[验证] validate-dir = {validate_dir}")
        self._run(cmd, tag="标定验证")

    # ==================================================================
    # 关闭
    # ==================================================================
    def on_close(self):
        if self.camera_process is not None:
            try:
                os.killpg(os.getpgid(self.camera_process.pid), signal.SIGTERM)
            except Exception:
                pass
        if self.task_process is not None:
            try:
                os.killpg(os.getpgid(self.task_process.pid), signal.SIGTERM)
            except Exception:
                pass
        self.root.destroy()


def main():
    root = tk.Tk()
    app = HandEyeCalibUI(root)
    root.protocol("WM_DELETE_WINDOW", app.on_close)
    root.mainloop()


if __name__ == "__main__":
    main()