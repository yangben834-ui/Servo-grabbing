#!/usr/bin/env python3
"""Gloria-M 夹爪: close 闭合后**可以一直保持力矩**, open 是一次性动作。

两种模式:

    --mode open    开到张开端 → 失能 → 退出 (一次性)
    --mode close   低速闭合, 遇到阻力就停:
                   ├ --hold-torque 0 (默认): 停完 → 失能 → 退出 (与改动前逐字相同)
                   └ --hold-torque T > 0   : 停在阻力点上**不退出**, 切 MIT 持续保持
                     T N·m, 直到 stdin 收到一行 OPEN (节点在放置时发过来), 或者
                     --hold-timeout 到点 / stdin EOF / SIGTERM。

**为什么要常驻 (2026-09-28 加回)**
-----------------------------------
抬升和搬运全程都需要夹持力, 而这个力**只能由主机持续发帧维持**: SDK 唯一提到
看门狗的地方 (`client.py` 的 "disconnect() does not send a disable command ...
continue to hold its last commanded position until the firmware watchdog times out
and cuts power") 说明固件会切电, 而 MIT 停发帧会怎样 SDK 根本没写。所以「发一条
就退」做不到保持 —— 必须常驻发帧 (demo 03 每 2ms 一帧, `sdk_linux` 的 clamp 每
10ms 一帧, 这是唯一有代码依据的做法)。

改之前 (2026-09-18 ~ 09-28) 是「一条命令一个一次性进程, 跑完失能退出」, 夹持力
全靠机构自锁; SDK 写明失能即 limp, 自锁也从没实测过。

**`[holding]` 是协议, 不是日志**
----------------------------------
第一帧 `send_mit` 真的发出去**之后**才打 `[holding] ...`。节点靠这一行知道
「夹持力已经建立」并推进状态机 —— 打早了, 节点会在夹爪其实还没顶住的抬升途中
往前走。除这一行之外, 输出仍然只是给人看的排障信息。

**退出语义** (节点侧定的是「失败/退出立刻松开」)

    stdin 收到 OPEN   → 张开 → 失能 → 退 0   (唯一的「有意放手」路径)
    stdin EOF         → 失能 → 退 0          (父进程没了)
    SIGTERM / SIGINT  → 失能 → 退 130        (不 drive 张开: 快、不会失败)
    --hold-timeout 到 → 失能 → 退 0 + [warn]

后三条都是**失能即 limp**, 不是把手指主动打开 —— 机械臂若悬空, 物体会掉下来。
这是 2026-09-28 有意选的, 别当 bug 追 (要改的话是节点侧的策略, 不是这里的默认值)。

`--resistance-torque` 只决定「在哪停下」, 抬起期间夹得紧不紧由 `--hold-torque`
决定, 两者没有关系。保持力矩**低于**停下那一刻测到的扭矩时, 切过去的一瞬间夹持力
会先降一下, 脚本会打一行 `[warn]` 提醒。

闭合判据与 demos/04 完全一致
------------------------------
阻力 = 扭矩超过「空载中位数 + --resistance-torque」并持续 --torque-debounce,
或堵转 (走够 --minimum-travel 后速度掉到 --stall-velocity 以下并持续 --stall-time)。
原样抄 04 而不是「重写一个更好的」: 用户已经用 04 把 `--resistance-torque 0.3`
调到对本机可靠了, 换一套判据等于把这个标定作废。

**撞硬扭矩上限不再算错误**: 两指之间夹到硬东西时扭矩会一步冲到上限, 快到阻力
判据的消抖窗口根本来不及成立 (2026-09-18 真机日志: 0.43s 冲到 -2.160 N·m,
阈值 +0.321)。那时夹爪其实已经夹上了, 原来却按 [abort] 退 2, 节点据此判任务
失败。现在改成**按「夹住了」处理**: 停下、打一行 warn、照常失能退出 0。

输出约定
----------
脚本的 stdout/stderr 会**原样进节点日志** (节点按 `[` 开头的标记行提级),
格式是给人看的排障信息 —— **唯一的例外是 `[holding]` 那一行**, 它是给节点解析的
(见上)。保持期间那行低频 `hold ...` **故意不以 `[` 开头**: 节点会把它降到 DEBUG,
免得 100Hz 的循环把日志刷爆 (刷爆不只是难看, 管道写满会让保持循环卡住)。

退出码: 0 正常做完 (含「一路合到底没夹到东西」与「保持被正常松开」) / 2 出错 /
130 Ctrl+C 或 SIGTERM

单独跑 (台架实测、标定阻力阈值、保持实测都用这个, 不需要起 ROS):

    # 一次性 (与改动前相同)
    /usr/bin/python3 -u scripts/gripper_grip_hold.py --mode close \
        --sdk-src /home/qin/Gloria-M-SDK/Gloria-M-SDK/src \
        --port /dev/ttyACM0 --velocity 2 --resistance-torque 0.3 \
        --hard-torque-limit 2 --timeout 25

    # 闭合后保持 1.0 N·m, 20 秒后从管道收到 OPEN 就松开
    ( sleep 20; echo OPEN ) | /usr/bin/python3 -u scripts/gripper_grip_hold.py \
        --mode close --hold-torque 1.0 --hold-timeout 120 \
        --sdk-src /home/qin/Gloria-M-SDK/Gloria-M-SDK/src \
        --port /dev/ttyACM0 --velocity 2 --resistance-torque 1.0 \
        --hard-torque-limit 2 --timeout 25
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import os
import selectors
import signal
import statistics
import sys
import time


# ── SDK 引导 ────────────────────────────────────────────────
# gloria_m_sdk 没有 pip 安装过, 只是把仓库的 src/ 加进 sys.path (SDK 的 demo 也是这么做的)。
# 默认路径是这台机器上的实测位置; 换机器用 --sdk-src 或环境变量覆盖。
DEFAULT_SDK_SRC = os.environ.get(
    "GLORIA_M_SDK_SRC", "Gloria-M-SDK/Gloria-M-SDK/src"
)


def _bootstrap_sdk(sdk_src: str) -> None:
    if sdk_src and sdk_src not in sys.path:
        sys.path.insert(0, sdk_src)


# 标定值 (与 demos/04、demos/05 一致): 这两个数是现场标出来的, 不要凭感觉改。
#   0.217076 rad = 完全闭合端, 2.641912 rad = 安全张开端
OPEN_Q = 2.641912
CLOSE_Q = 0.217076


@dataclass
class ResistanceDetector:
    """与 demos/04 完全相同的阻力判据 (扭矩增量 + 堵转, 各自带消抖)。

    原样抄过来而不是「重写一个更好的」: 用户已经用 04 把
    `--resistance-torque 0.3` 调到对本机可靠了, 换一套判据等于把这个标定作废。
    """

    torque_threshold: float
    torque_debounce_s: float
    stall_velocity: float
    stall_debounce_s: float
    minimum_travel: float
    target_tolerance: float
    _torque_started_at: float | None = None
    _stall_started_at: float | None = None

    def update(
        self,
        *,
        now: float,
        start_position: float,
        position: float,
        velocity: float,
        torque: float,
        target: float,
    ) -> str | None:
        """Return ``torque`` or ``stall`` after a condition is debounced."""
        if abs(torque) >= self.torque_threshold:
            if self._torque_started_at is None:
                self._torque_started_at = now
            elif now - self._torque_started_at >= self.torque_debounce_s:
                return "torque"
        else:
            self._torque_started_at = None

        travelled = abs(position - start_position)
        target_error = abs(position - target)
        stalled = (
            travelled >= self.minimum_travel
            and target_error > self.target_tolerance
            and abs(velocity) <= self.stall_velocity
        )
        if stalled:
            if self._stall_started_at is None:
                self._stall_started_at = now
            elif now - self._stall_started_at >= self.stall_debounce_s:
                return "stall"
        else:
            self._stall_started_at = None

        return None


def _parse_int(value: str) -> int:
    return int(value, 0)


def _wait_for_fresh_feedback(g: "GloriaGripper", *, timeout_s: float) -> None:
    previous = float(g.state.updated_at)
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        g.motor.refresh()
        time.sleep(0.01)
        g.motor.poll()
        if g.state.updated_at > previous:
            return
    raise RuntimeError("no fresh motor feedback; check feedback ID and CAN wiring")


def _measure_idle_torque(
    g: "GloriaGripper", *, samples: int, interval_s: float, timeout_s: float
) -> float:
    """空载静态扭矩中位数。阻力阈值 = 它 + --resistance-torque。"""
    values: list[float] = []
    last_feedback = float(g.state.updated_at)
    deadline = time.monotonic() + timeout_s

    while len(values) < samples and time.monotonic() < deadline:
        g.motor.refresh()
        time.sleep(interval_s)
        g.motor.poll()
        if g.state.updated_at > last_feedback:
            last_feedback = float(g.state.updated_at)
            values.append(abs(float(g.state.torque)))

    if len(values) < samples:
        raise RuntimeError(
            f"only received {len(values)}/{samples} idle feedback samples"
        )
    return float(statistics.median(values))


def _read_watchdog_ms(g: "GloriaGripper") -> float | None:
    """读固件看门狗超时 (RID 9 `Variable.TIMEOUT`) —— 保持帧的周期必须远小于它。

    读不到返回 None: 这个值只用来**打日志、判断周期够不够**, 拿不到不该让保持
    功能整个失败 (老固件没这个寄存器也说不定)。
    """
    try:
        from gloria_m_sdk import Variable  # noqa: PLC0415 - 引导之后才 import

        value = g.params.read(int(Variable.TIMEOUT))
    except Exception:  # noqa: BLE001 - 读不到不算错
        return None
    return None if value is None else float(value)


# ── 保持 (2026-09-28) ───────────────────────────────────────
#
# 这一段的每一行都服务于同一件事: **不能让保持帧断掉**。帧一断, 固件看门狗切电,
# 夹持力没了, 瓶子在搬运途中掉 —— 那就是这个功能要防的事故。

# 节点靠这一行知道「夹持力已经建立」(见模块 docstring)。前缀单独提出来, 免得
# 打标记的那行和判据那行各写各的字符串。
HOLD_MARK_PREFIX = "[holding]"


def _safe_write(text: str) -> None:
    """往 stdout 写一行, **绝不因为写不出去而阻塞或抛异常**。

    保持期间 fd 1 被设成非阻塞。管道读端是节点 —— 节点死了或卡住时管道会满, 那时
    print() 会阻塞住整个保持循环, 帧就断了。宁可丢一行日志, 也不能停帧。
    """
    try:
        os.write(1, (text + "\n").encode("utf-8", "replace"))
    except (BlockingIOError, InterruptedError):
        pass  # 管道满了
    except OSError:
        pass  # 读端没了 (EPIPE 之类)


class OpenWatch:
    """非阻塞地盯着 stdin 上有没有一行 `OPEN`。`poll()` → 'OPEN' / 'EOF' / None。

    全程**按字节**来, 不碰 sys.stdin 的文本层 —— 父进程写的是 bytes, 混用 str 和
    bytes 比较正是当年那版踩过的坑。

    stdin 读不了 (不可 select / 已被关掉) 时退化成「永远收不到 OPEN」, 靠
    --hold-timeout 兜底: 那种情况下保持功能仍然可用, 只是没人能提前叫停它。
    """

    OPEN = "OPEN"
    EOF = "EOF"

    def __init__(self, fd: int = 0):
        self._fd = fd
        self._pending = b""
        self._selector = None
        try:
            # 不设非阻塞的话, select 说可读也可能在 read 上卡住 (读到半个行时)。
            os.set_blocking(fd, False)
            selector = selectors.DefaultSelector()
            selector.register(fd, selectors.EVENT_READ)
            self._selector = selector
        except (OSError, ValueError, KeyError) as exc:
            print(f"[warn] stdin 不可读 ({exc}); 保持阶段只能靠 --hold-timeout 退出",
                  file=sys.stderr)

    def poll(self) -> str | None:
        if self._selector is None:
            return None
        if not self._selector.select(timeout=0.0):
            return None
        try:
            chunk = os.read(self._fd, 4096)
        except (BlockingIOError, InterruptedError):
            return None
        except OSError:
            return self.EOF
        if not chunk:
            # EOF 必须单独回报: 之后 select **一直**说可读, 不处理就是 100Hz 空转。
            return self.EOF
        self._pending += chunk
        if b"\n" not in self._pending:
            return None
        raw, self._pending = self._pending.split(b"\n", 1)
        return raw.decode("utf-8", "replace").strip().upper()


def _hold(
    gripper: "GloriaGripper",
    *,
    args: argparse.Namespace,
    stop_q: float,
    stop_tau: float,
    watch: OpenWatch,
) -> str:
    """停在阻力点上持续保持 ``--hold-torque`` N·m, 直到被要求松开。

    返回松开的原因: ``'open'`` (收到 OPEN 行, main 去把它张开) / ``'eof'`` /
    ``'timeout'``。后两种 main 不张开, 直接失能退出 —— 见模块 docstring 那张表。

    **开头三步的顺序不能改**, 每一步都在补上一个「力会断掉」的窗口:
      1. 先用 PV 位置环顶在停下那个点上 —— 切模式要几十毫秒, 那几十毫秒里不能没力;
      2. `set_mode(MIT)` 之后**再 enable() 一次** (SDK 要求 set_mode 在 enable 之前;
         运行中切模式会不会清掉使能状态它没写, 补一次只要 0.1s);
      3. 第一帧 `send_mit` 真的发出去之后**才**打 `[holding]`。
    """
    from gloria_m_sdk import ControlMode  # noqa: PLC0415 - 引导之后才 import

    hold_torque = float(args.hold_torque)
    if abs(stop_tau) > hold_torque + 1e-9:
        print(f"[warn] 保持力矩 {hold_torque:.3f} N·m 低于停下时测到的 "
              f"{abs(stop_tau):.3f} N·m —— 切过去的一瞬间夹持力会先降一下",
              file=sys.stderr)

    # 1) 位置环顶住。SDK 原文: velocity=0 时 "the motor becomes a stiff position
    #    controller" —— 用它的刚性盖住下面那次模式切换。
    gripper.motion.send_pos_vel(position=float(stop_q), velocity=0.0, poll=True)
    # 2) 切 MIT, 并补一次使能。
    gripper.motor.set_mode(ControlMode.MIT)
    gripper.motor.enable()
    watchdog_ms = _read_watchdog_ms(gripper)

    period = 1.0 / max(1.0, float(args.hold_hz))
    tau_cmd = -hold_torque  # 负 = 闭合方向

    try:
        # 3) 第一帧, 然后才打标记 —— 标记就是「力已经加上」的承诺。
        gripper.motion.send_mit(
            kp=float(args.hold_kp), kd=float(args.hold_kd),
            q=float(stop_q), dq=0.0, tau=tau_cmd, poll=True,
        )
        # 等一帧**新**反馈再打标记: 刚发出去时读到的还是位置环那一下的力矩, 台架上就
        # 看不出「力到底加上没有」。等不到就不等 —— 标记的时效比那一个读数重要。
        settle_deadline = time.monotonic() + min(0.2, float(args.feedback_timeout))
        baseline_updated_at = float(gripper.state.updated_at)
        while time.monotonic() < settle_deadline:
            gripper.motor.poll()
            if float(gripper.state.updated_at) > baseline_updated_at:
                break
            time.sleep(0.002)
        os.set_blocking(1, False)
        _safe_write(
            f"{HOLD_MARK_PREFIX} tau_cmd={tau_cmd:+.3f} "
            f"kd={float(args.hold_kd):.2f} kp={float(args.hold_kp):.2f} "
            f"q={float(stop_q):+.3f} "
            f"tau_fb={float(gripper.state.torque):+.3f} "
            f"wdt_ms={watchdog_ms if watchdog_ms is not None else 'unknown'} "
            f"period_ms={period * 1000.0:.1f}"
        )

        started_at = time.monotonic()
        last_feedback_value = float(gripper.state.updated_at)
        last_feedback_at = started_at
        last_print_at = started_at
        next_frame = started_at
        frames = 0

        while True:
            command = watch.poll()
            if command == OpenWatch.OPEN:
                return "open"
            if command == OpenWatch.EOF:
                _safe_write("[warn] stdin 已关闭 (父进程没了) —— 失能松开")
                return "eof"

            gripper.motion.send_mit(
                kp=float(args.hold_kp), kd=float(args.hold_kd),
                q=float(stop_q), dq=0.0, tau=tau_cmd, poll=True,
            )
            frames += 1
            now = time.monotonic()
            state = gripper.state

            if state.updated_at > last_feedback_value:
                last_feedback_value = float(state.updated_at)
                last_feedback_at = now
            elif now - last_feedback_at > float(args.feedback_timeout):
                # 反馈断了说明链路有问题 —— 保持帧多半也到不了, 报出去 (退 2),
                # 别在这儿装作还在保持。
                raise RuntimeError(
                    f"保持期间反馈断了 {now - last_feedback_at:.2f}s"
                )

            if now - started_at >= float(args.hold_timeout):
                _safe_write(
                    f"[warn] 保持已到 --hold-timeout "
                    f"{float(args.hold_timeout):.0f}s 仍没人要求松开 —— "
                    f"失能放手 (节点可能已经不在了)"
                )
                return "timeout"

            if now - last_print_at >= 1.0 / max(0.1, float(args.hold_print_hz)):
                last_print_at = now
                # **故意不以 "[" 开头**: 节点会把这种行降到 DEBUG, 免得整个搬运
                # 期间刷屏 (刷屏不只是难看, 管道写满会让上面那行 send_mit 断掉)。
                _safe_write(
                    f"hold t={now - started_at:5.1f}s "
                    f"q={float(state.position):+.3f} "
                    f"dq={float(state.velocity):+.3f} "
                    f"tau={float(state.torque):+.3f} "
                    f"rate={frames / max(1e-6, now - started_at):.1f}Hz"
                )

            next_frame += period
            delay = next_frame - time.monotonic()
            if delay > 0:
                time.sleep(delay)
            else:
                next_frame = time.monotonic()  # 落后了就重置基准, 别累积
    finally:
        # 恢复阻塞: 后面 main 的收尾 (以及 finally 里那次 disable) 还要正常打日志。
        try:
            os.set_blocking(1, True)
        except OSError:
            pass


def _open_after_hold(gripper: "GloriaGripper", *, args: argparse.Namespace) -> None:
    """保持之后张开: **必须先切回 PV**, 现在还是 MIT。

    不切的话 `send_pos_vel` 里的 `_require_mode(POS_VEL)` 直接抛 GloriaModeError。
    切模式那一步失败时退到「用 MIT 正扭矩自己张开」—— 那时失能的代价是把物体留在
    夹爪里; 两条路都失败就抛出去, 由 main 报退出码 2 (节点据此判夹爪失败, 而不是
    误以为「已张开」然后报放置完成)。
    """
    from gloria_m_sdk import ControlMode  # noqa: PLC0415 - 引导之后才 import

    try:
        gripper.motor.set_mode(ControlMode.POS_VEL)
        gripper.motor.enable()
        _drive_to(gripper, target_q=float(args.open_q),
                  velocity=float(args.velocity), args=args, label="open")
        return
    except Exception as exc:  # noqa: BLE001 - 退到 MIT 那条路再决定要不要报错
        print(f"[warn] 切回 PV 张开失败 ({exc}); 改用 MIT 正扭矩张开",
              file=sys.stderr)

    deadline = time.monotonic() + float(args.timeout)
    while time.monotonic() < deadline:
        gripper.motion.send_mit(
            kp=0.0, kd=float(args.hold_kd), q=float(args.open_q), dq=0.0,
            tau=abs(float(args.open_tau)), poll=True,
        )
        position = float(gripper.state.position)
        if position >= float(args.open_q) - float(args.target_tolerance):
            print(f"[open] MIT 张开到位: q={position:+.3f}")
            return
        time.sleep(float(args.loop_sleep))
    raise RuntimeError("保持之后张开失败 (PV 与 MIT 两条路都不行)")


# ── 两个动作 ────────────────────────────────────────────────


def _drive_to(
    gripper: "GloriaGripper",
    *,
    target_q: float,
    velocity: float,
    args: argparse.Namespace,
    label: str,
) -> None:
    """把夹爪从当前位置开到 ``target_q``, 直到到位/堵转/超时。

    与 demos/05 的开循环一致 (目标 + 反馈新鲜度 + 硬扭矩 + 堵转 + 总超时)。
    **只用于 open**: 张开方向是空载的, 一路开到端点即可, 不需要阻力判据。
    """
    started_at = time.monotonic()
    last_feedback_value = float(gripper.state.updated_at)
    last_feedback_at = started_at
    last_print_at = 0.0
    stall_started_at: float | None = None

    while True:
        gripper.motion.send_pos_vel(
            position=float(target_q), velocity=float(velocity), poll=True
        )
        now = time.monotonic()
        state = gripper.state

        if state.updated_at > last_feedback_value:
            last_feedback_value = float(state.updated_at)
            last_feedback_at = now
        if now - last_feedback_at > float(args.feedback_timeout):
            raise RuntimeError(f"feedback stale for {now - last_feedback_at:.2f}s")
        if abs(float(state.torque)) >= float(args.hard_torque_limit):
            raise RuntimeError(
                f"hard torque limit triggered: {state.torque:+.3f} N·m"
            )

        target_error = abs(float(state.position) - float(target_q))
        if target_error <= float(args.target_tolerance):
            print(f"[{label}] reached: q={state.position:+.3f}, "
                  f"error={target_error:.3f} rad")
            return

        stalled = abs(float(state.velocity)) <= float(args.stall_velocity)
        if stalled:
            if stall_started_at is None:
                stall_started_at = now
            elif now - stall_started_at >= float(args.stall_time):
                raise RuntimeError(
                    f"{label} stalled at q={state.position:+.3f}, "
                    f"dq={state.velocity:+.3f}"
                )
        else:
            stall_started_at = None

        elapsed = now - started_at
        if elapsed >= float(args.timeout):
            raise RuntimeError(
                f"{label} timeout after {elapsed:.1f}s at q={state.position:+.3f}"
            )

        if now - last_print_at >= 1.0 / max(1.0, float(args.print_hz)):
            last_print_at = now
            print(f"t={elapsed:5.2f}s | target={target_q:+.3f} "
                  f"q={state.position:+.3f} dq={state.velocity:+.3f} "
                  f"tau={state.torque:+.3f}")
        time.sleep(float(args.loop_sleep))


def _close(gripper: "GloriaGripper", *, args: argparse.Namespace) -> tuple[float, float, str]:
    """低速闭合, 遇到阻力就停; 停在**遇到阻力的那个位置**, 不继续挤。

    与 demos/04 同一个循环。停下的三种情况都算「这个动作做完了」, 都返回给 main
    (由 main 决定是保持还是走老路失能退出):
      - 阻力成立     → 夹住东西了;
      - 撞硬扭矩上限 → **也按夹住了处理** (见模块注释最后一段), 只打一行 warn;
      - 一路合到目标 → 夹爪里是空的 (`no-object`), 照实打一行说明。
    真出错 (反馈断 / 超时) 仍然抛异常 → 退 2, 因为那时「闭合」根本没做完。

    返回 ``(停下时的 q, 停下时的扭矩, 原因)``: 保持要用前两个 (力矩前馈的平衡点,
    以及「切过去会不会先松一下」的判据), 原因用来区分「夹到东西」和「夹了个空」。
    """
    _wait_for_fresh_feedback(gripper, timeout_s=args.feedback_timeout)
    start_position = float(gripper.state.position)

    idle_torque = _measure_idle_torque(
        gripper,
        samples=args.idle_samples,
        interval_s=0.02,
        timeout_s=max(1.0, args.feedback_timeout * 4.0),
    )
    torque_threshold = idle_torque + float(args.resistance_torque)
    if torque_threshold >= float(args.hard_torque_limit):
        raise RuntimeError(
            f"idle torque {idle_torque:.3f} N·m leaves no margin below "
            f"hard limit {args.hard_torque_limit:.3f} N·m"
        )

    detector = ResistanceDetector(
        torque_threshold=torque_threshold,
        torque_debounce_s=float(args.torque_debounce),
        stall_velocity=float(args.stall_velocity),
        stall_debounce_s=float(args.stall_time),
        minimum_travel=float(args.minimum_travel),
        target_tolerance=float(args.target_tolerance),
    )
    print(f"[start] close q={start_position:+.3f} -> {args.close_q:+.3f} rad | "
          f"velocity={args.velocity:.3f} rad/s | idle_tau={idle_torque:.3f} | "
          f"resistance_threshold={torque_threshold:.3f} N·m")

    started_at = time.monotonic()
    last_feedback_value = float(gripper.state.updated_at)
    last_feedback_at = started_at
    last_print_at = 0.0

    while True:
        gripper.motion.send_pos_vel(
            position=float(args.close_q), velocity=float(args.velocity), poll=True
        )
        now = time.monotonic()
        state = gripper.state

        if state.updated_at > last_feedback_value:
            last_feedback_value = float(state.updated_at)
            last_feedback_at = now
        if now - last_feedback_at > float(args.feedback_timeout):
            raise RuntimeError(f"feedback stale for {now - last_feedback_at:.2f}s")

        if abs(float(state.torque)) >= float(args.hard_torque_limit):
            # 带上阈值和耗时: 撞上限时最要紧的两个数就是「阈值设在哪」和「冲上来
            # 多快」, 只给一个上限值, 排障时还得回头翻 [start] 那行才知道余量多薄。
            print(f"[warn] 扭矩冲到硬上限 {state.torque:+.3f} N·m "
                  f"(阻力阈值 {torque_threshold:+.3f}, 消抖 "
                  f"{float(args.torque_debounce):.2f}s, 用时 {now - started_at:.2f}s) "
                  f"—— 消抖窗口还没成立就冲到上限, 多半是两指之间夹到了硬东西 "
                  f"(瓶盖 / 台面) 或者夹的位置不对; 按「已夹住」停下, 不再继续挤",
                  file=sys.stderr)
            return (float(state.position), float(state.torque), "hard-limit")

        target_error = abs(float(state.position) - float(args.close_q))
        if target_error <= float(args.target_tolerance):
            print("[stop] 一路合到底都没遇到阻力, 夹爪里可能是空的")
            return (float(state.position), float(state.torque), "no-object")

        reason = detector.update(
            now=now,
            start_position=start_position,
            position=float(state.position),
            velocity=float(state.velocity),
            torque=float(state.torque),
            target=float(args.close_q),
        )
        if reason is not None:
            print(f"[stop] resistance detected by {reason}: "
                  f"q={state.position:+.3f}, dq={state.velocity:+.3f}, "
                  f"tau={state.torque:+.3f} N·m")
            return (float(state.position), float(state.torque), reason)

        elapsed = now - started_at
        if elapsed >= float(args.timeout):
            raise RuntimeError(
                f"close timeout after {elapsed:.1f}s at q={state.position:+.3f}"
            )

        if now - last_print_at >= 1.0 / max(1.0, float(args.print_hz)):
            last_print_at = now
            print(f"t={elapsed:5.2f}s | q={state.position:+.3f} "
                  f"dq={state.velocity:+.3f} tau={state.torque:+.3f} | "
                  f"threshold={torque_threshold:.3f}")
        time.sleep(float(args.loop_sleep))


# ── 主流程 ──────────────────────────────────────────────────


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Gloria-M gripper: 一条命令 = 一个动作 (close / open), 做完失能退出"
    )
    parser.add_argument("--mode", choices=("open", "close"), required=True,
                        help="close: 闭合到遇阻; open: 开到张开端。都做完即退出")
    parser.add_argument("--sdk-src", default=DEFAULT_SDK_SRC,
                        help="gloria_m_sdk 的 src 目录 (要被加进 sys.path)")
    parser.add_argument("--port", default="/dev/ttyACM0")
    parser.add_argument("--baud", type=int, default=921600)
    parser.add_argument("--id", type=_parse_int, default=0x01)
    parser.add_argument("--fb-id", type=_parse_int, default=0x101)
    parser.add_argument("--open-q", type=float, default=OPEN_Q)
    parser.add_argument("--close-q", type=float, default=CLOSE_Q)
    parser.add_argument("--velocity", type=float, default=2.0,
                        help="开/闭共用的速度 (rad/s); 现场用的是 2")
    parser.add_argument("--resistance-torque", type=float, default=0.3,
                        help="比空载静态扭矩高多少算遇到阻力 [N·m]")
    parser.add_argument("--torque-debounce", type=float, default=0.12)
    parser.add_argument("--hard-torque-limit", type=float, default=2.0,
                        help="闭合时撞上它就停下 (按已夹住处理), 不再继续挤")
    parser.add_argument("--stall-velocity", type=float, default=0.025)
    parser.add_argument("--stall-time", type=float, default=0.35)
    parser.add_argument("--minimum-travel", type=float, default=0.05)
    parser.add_argument("--target-tolerance", type=float, default=0.05)
    parser.add_argument("--feedback-timeout", type=float, default=0.5)
    parser.add_argument("--timeout", type=float, default=25.0)
    parser.add_argument("--loop-sleep", type=float, default=0.02)
    parser.add_argument("--print-hz", type=float, default=10.0)
    parser.add_argument("--idle-samples", type=int, default=10)
    # ── 保持 (2026-09-28, 只在 --mode close 下有意义) ──
    parser.add_argument("--hold-torque", type=float, default=0.0,
                        help="闭合停下后持续保持的力矩 [N·m]; 0 = 关闭保持, "
                             "停完就失能退出 (与改动前逐字相同)")
    parser.add_argument("--hold-timeout", type=float, default=300.0,
                        help="保持的最长时间 [s]; 到点失能松开 —— 最后兜底, "
                             "只在没人叫停也没人管的时候生效")
    parser.add_argument("--hold-hz", type=float, default=100.0,
                        help="保持帧的频率 [Hz]。必须远高于固件看门狗超时, "
                             "否则帧间隔里会被切电")
    parser.add_argument("--hold-kd", type=float, default=0.5,
                        help="MIT 阻尼项 [N·m·s/rad]; 范围 [0, 5]")
    parser.add_argument("--hold-kp", type=float, default=0.0,
                        help="MIT 位置刚度 [N·m/rad]; 0 = 纯力矩 (此时 q 无效), "
                             ">0 才有位置环 + 力矩前馈")
    parser.add_argument("--hold-print-hz", type=float, default=1.0,
                        help="保持期间那行状态日志的频率 [Hz]")
    parser.add_argument("--open-tau", type=float, default=0.5,
                        help="切不回 PV 时, MIT 兜底张开用的正扭矩 [N·m]")
    return parser


def main() -> int:
    args = _build_parser().parse_args()
    _bootstrap_sdk(args.sdk_src)

    try:
        # 延迟到引导之后再 import: sys.path 是刚改的。
        from gloria_m_sdk import (  # noqa: PLC0415
            ControlMode,
            GloriaGripper,
            GloriaSdkError,
            Limits,
            PositionRange,
        )
    except ModuleNotFoundError as exc:
        print(f"[abort] 找不到 gloria_m_sdk ({exc}); 用 --sdk-src 指定它的 src 目录",
              file=sys.stderr)
        return 2

    if args.close_q >= args.open_q:
        raise SystemExit("close-q must be smaller than open-q for this calibration")
    if args.velocity <= 0.0:
        raise SystemExit("velocity must be positive")
    if args.resistance_torque <= 0.0 or args.hard_torque_limit <= 0.0:
        raise SystemExit("torque thresholds must be positive")
    if args.idle_samples < 3:
        raise SystemExit("idle-samples must be at least 3")
    if args.hold_torque < 0.0:
        raise SystemExit("hold-torque must not be negative")
    if args.hold_torque >= args.hard_torque_limit:
        # 硬上限只写在闭合循环里; 保持比它还大就等于绕过了那道保护。
        raise SystemExit("hold-torque must stay below hard-torque-limit")
    if args.hold_torque > 0.0 and args.hold_timeout <= 0.0:
        raise SystemExit("hold-timeout must be positive when hold-torque is on")
    # MIT 打包是 12bit 定标 (kp 0~500 / kd 0~5): 超范围是**静默截断**不是报错,
    # 所以在这里显式拦一次, 免得现场调了个 60 却按 5 在跑。
    if not 0.0 <= args.hold_kp <= 500.0:
        raise SystemExit("hold-kp must be within [0, 500]")
    if not 0.0 <= args.hold_kd <= 5.0:
        raise SystemExit("hold-kd must be within [0, 5]")
    if args.hold_hz <= 0.0:
        raise SystemExit("hold-hz must be positive")

    safe_position = PositionRange(min=float(args.close_q), max=float(args.open_q))
    gripper = GloriaGripper(
        args.port,
        baudrate=args.baud,
        command_id=args.id,
        feedback_id=args.fb_id,
        limits=Limits(pmax=3.14, vmax=10.0, tmax=12.0),
        safe_position=safe_position,
    )

    enabled = False

    def _on_sigterm(_signum, _frame):
        # 节点 destroy_node 里的 terminate() 走这条路。抛 KeyboardInterrupt 复用
        # 下面那条「收拾干净再退出」的既有分支, 而不是让进程被直接砍掉。
        raise KeyboardInterrupt

    signal.signal(signal.SIGTERM, _on_sigterm)

    try:
        gripper.connect(apply_limits=False)
        gripper.motor.set_mode(ControlMode.POS_VEL)
        print("[mode] PV: ok")
        gripper.motor.enable()
        enabled = True
        print("[enable] ok")

        if args.mode == "open":
            _wait_for_fresh_feedback(gripper, timeout_s=args.feedback_timeout)
            start_position = float(gripper.state.position)
            if not safe_position.min - 0.20 <= start_position <= safe_position.max + 0.20:
                raise RuntimeError(
                    f"initial position {start_position:+.3f} rad is outside the "
                    "calibrated range"
                )
            _drive_to(gripper, target_q=float(args.open_q),
                      velocity=float(args.velocity), args=args, label="open")
            return 0

        stop_q, stop_tau, stop_reason = _close(gripper, args=args)

        if args.hold_torque <= 0.0 or stop_reason == "no-object":
            # 两种「不保持」: 保持开关关着, 或者两指之间根本没东西 (夹了个空) ——
            # 后者没什么可保持的, 照旧失能退出, 由节点照常抬升 (老行为一字不变)。
            print("[done] 闭合结束 (夹持力不再由本进程维持 —— 失能后靠机构自锁)")
            return 0

        print(f"[hold] 闭合停在 q={stop_q:+.3f} ({stop_reason}), 开始保持 "
              f"{float(args.hold_torque):.3f} N·m (上限 {float(args.hold_timeout):.0f}s)")
        why = _hold(gripper, args=args, stop_q=stop_q, stop_tau=stop_tau,
                    watch=OpenWatch())
        if why == "open":
            # 节点在放置时要我们张开 —— 张完才算这一趟做完, 张开失败要报错 (退 2),
            # 否则节点会以为「已张开」然后报放置完成。
            _open_after_hold(gripper, args=args)
            print("[done] 保持已松开 (夹爪已张开)")
        else:
            # 'eof' / 'timeout': 没人来叫停, 由下面的 finally 失能放手。
            print(f"[done] 保持结束 ({why}), 失能放手")
        return 0

    except KeyboardInterrupt:
        print("\n[ctrl+c] stop requested")
        return 130
    except (RuntimeError, GloriaSdkError, OSError) as exc:
        # GloriaSdkError 是 SDK 所有异常的基类 —— 串口打不开 (GloriaConnectionError)、
        # 模式不对、通信失败都走这里。OSError 单独列一个: 串口被拔掉时 pyserial 抛的
        # SerialException 不在 SDK 那棵树里。节点看到的是「退出码 2 + 一行 [abort]」,
        # 而不是一个 Python traceback。
        print(f"\n[abort] {exc}", file=sys.stderr)
        return 2
    finally:
        if enabled and gripper.is_connected:
            try:
                gripper.motor.disable()
                print("[disable] ok")
            except Exception as exc:  # noqa: BLE001
                print(f"[warn] disable 失败: {exc}", file=sys.stderr)
        gripper.disconnect()


if __name__ == "__main__":
    raise SystemExit(main())
