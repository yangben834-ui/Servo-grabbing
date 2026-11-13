# Copyright (c) 2026 Synria Robotics Co., Ltd.
# Author: Synria Robotics Team
# Website: https://synriarobotics.ai

"""
02 – PV mode gripper gentle close (PV Gentle Close)

Description:
    First opens the gripper to the open position, then slowly closes to the
    target position using PV (position + velocity) mode.  The low closing
    speed produces a smooth, gentle motion suitable for picking up delicate
    objects.

How it works:
    In PV mode, each control cycle sends a target position and a speed limit
    to the motor.  The motor firmware handles trajectory planning and speed
    limiting internally.  The opening phase uses a higher speed (open_vel);
    the closing phase uses a lower speed (close_vel) for gentle approach.
    After reaching the target the gripper holds for the configured hold time
    before exiting.

Usage:
    python 02_pv_control.py --port /dev/ttyACM0 --close-vel 0.1

Exit:
    Press Ctrl+C to exit; the motor will be disabled automatically.
"""

from __future__ import annotations

import argparse
import os
import sys
import time

_REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
_SRC_DIR = os.path.join(_REPO_ROOT, "src")
if _SRC_DIR not in sys.path:
    sys.path.insert(0, _SRC_DIR)

from gloria_m_sdk import (
    ControlMode,
    GloriaGripper,
    Limits,
    PositionRange,
)

# =============================================================================
# ★ 用户参数区 — 直接在此修改即可，无需命令行参数
#   (All parameters can also be overridden via command-line arguments)
# =============================================================================

# ── 连接 / Connection ─────────────────────────────────────────────────────────
PORT          = "/dev/ttyACM0"  # 当前标定使用的串口；其他机器可通过 --port 覆盖
BAUD          = 921600   # 串口波特率
MOTOR_ID      = 0x01     # 电机命令 CAN ID
MOTOR_FB_ID   = 0x101    # 电机反馈 CAN ID

# ── 夹爪位置 / Gripper positions ──────────────────────────────────────────────
# 2026-08-25 两次手动标定：机械端点约为 +0.1916 / -2.5959 rad。
# 默认值已分别向行程内部保留约 0.14 rad 安全余量。
OPEN_Q        = 0.05     # 安全开口目标位置 [rad]
CLOSE_Q       = -2.45    # 安全闭合目标位置 [rad]

# ── 速度 / Velocities ─────────────────────────────────────────────────────────
OPEN_VEL      = 0.2      # 开口速度 [rad/s]
CLOSE_VEL     = 0.1      # 闭合速度 [rad/s]；越小越柔和

# ── 时序 / Timing ─────────────────────────────────────────────────────────────
HOLD_TIME     = 2.0      # 到位后夹持保持时间 [s]
LOOP_SLEEP    = 0.01     # 控制循环间隔 [s]
PRINT_HZ      = 5.0      # 状态打印频率 [Hz]

# ── 到位判断 / Settle detection ───────────────────────────────────────────────
SETTLE_THRESHOLD = 0.05  # 位置误差小于此值视为到位 [rad]
SETTLE_TIME      = 0.5   # 到位后需持续稳定的时间 [s]
TIMEOUT          = 35.0  # 每段运动最大允许时间 [s]
FEEDBACK_TIMEOUT = 0.5   # 反馈超过该时间未更新则立即退出 [s]
MAX_ABS_TORQUE   = 3.0   # PV 测试反馈扭矩绝对值保护 [N·m]

# =============================================================================


def _parse_int(value: str) -> int:
    return int(value, 0)


def _resolve_port(port: str) -> str:
    """Return *port* unchanged, or auto-detect a single available COM port."""
    if port and port.lower() != "auto":
        return port
    from serial.tools import list_ports

    found = list(list_ports.comports())
    if not found:
        raise SystemExit("[port] No serial ports found. Plug in the serial-to-CAN adapter.")
    if len(found) > 1:
        names = ", ".join(p.device for p in found)
        raise SystemExit(f"[port] Multiple serial ports found ({names}); pass --port explicitly.")
    print(f"[port] auto-detected {found[0].device} ({found[0].description})")
    return found[0].device


def main() -> int:
    ap = argparse.ArgumentParser(description="PV mode gripper gentle close")
    ap.add_argument("--port", default=PORT, help="serial port; 'auto' picks the only available one")
    ap.add_argument("--baud", type=int, default=BAUD)
    ap.add_argument("--id", type=_parse_int, default=MOTOR_ID, help="motor command CAN ID")
    ap.add_argument("--fb-id", type=_parse_int, default=MOTOR_FB_ID, help="motor feedback CAN ID")
    ap.add_argument("--open-q", type=float, default=OPEN_Q, help="open position [rad]")
    ap.add_argument("--close-q", type=float, default=CLOSE_Q, help="close position [rad]")
    ap.add_argument("--open-vel", type=float, default=OPEN_VEL, help="opening velocity [rad/s]")
    ap.add_argument("--close-vel", type=float, default=CLOSE_VEL, help="closing velocity [rad/s]; lower = gentler")
    ap.add_argument("--loop-sleep", type=float, default=LOOP_SLEEP, help="control loop sleep time [s]")
    ap.add_argument("--print-hz", type=float, default=PRINT_HZ, help="print frequency [Hz]")
    ap.add_argument("--settle-threshold", type=float, default=SETTLE_THRESHOLD, help="settle threshold [rad]")
    ap.add_argument("--settle-time", type=float, default=SETTLE_TIME, help="dwell time after settling [s]")
    ap.add_argument("--timeout", type=float, default=TIMEOUT, help="max time per segment [s]")
    ap.add_argument("--hold-time", type=float, default=HOLD_TIME, help="grip hold duration after closing [s]")
    ap.add_argument("--feedback-timeout", type=float, default=FEEDBACK_TIMEOUT, help="abort if feedback stops updating [s]")
    ap.add_argument("--max-abs-torque", type=float, default=MAX_ABS_TORQUE, help="abort above this absolute feedback torque [N·m]")
    args = ap.parse_args()
    args.port = _resolve_port(args.port)

    safe_q = PositionRange(min=min(args.open_q, args.close_q), max=max(args.open_q, args.close_q))
    limits = Limits(pmax=3.14, vmax=10.0, tmax=12.0)

    g = GloriaGripper(
        args.port,
        baudrate=args.baud,
        command_id=args.id,
        feedback_id=args.fb_id,
        limits=limits,
        safe_position=safe_q,
    )
    enabled = False
    try:
        g.connect(apply_limits=False)
        g.motor.set_mode(ControlMode.POS_VEL)
        print("[mode] PV: ok")
        g.motor.enable()
        enabled = True
        print("[enable] ok")

        # Read the initial position before issuing any move commands.
        _wait_for_fresh_feedback(g, timeout_s=args.feedback_timeout)
        print(f"[init] position = {g.state.position:.3f} rad")

        # ---- Phase 1: open to open_q ----
        print(f"\n=== Phase 1: open to {args.open_q:.2f} rad ===")
        _move_to(g, target=args.open_q, velocity=args.open_vel,
                 loop_sleep=args.loop_sleep, print_hz=args.print_hz,
                 settle_threshold=args.settle_threshold, settle_time=args.settle_time,
                 timeout=args.timeout, feedback_timeout=args.feedback_timeout,
                 max_abs_torque=args.max_abs_torque)
        print(f"\n  Open complete, current position = {g.state.position:.3f} rad")
        time.sleep(0.5)

        # ---- Phase 2: gentle close to close_q ----
        print(f"\n=== Phase 2: gentle close to {args.close_q:.2f} rad ===")
        _move_to(g, target=args.close_q, velocity=args.close_vel,
                 loop_sleep=args.loop_sleep, print_hz=args.print_hz,
                 settle_threshold=args.settle_threshold, settle_time=args.settle_time,
                 timeout=args.timeout, feedback_timeout=args.feedback_timeout,
                 max_abs_torque=args.max_abs_torque)
        print(f"\n  Close complete, current position = {g.state.position:.3f} rad")

        # Hold position: keep sending the closed position with velocity=0
        # so the motor holds stiffly rather than going limp.
        print(f"  Holding grip for {args.hold_time:.1f} s ...")
        hold_end = time.perf_counter() + args.hold_time
        last_feedback_value = float(g.state.updated_at)
        last_feedback_at = time.perf_counter()
        while time.perf_counter() < hold_end:
            g.motion.send_pos_vel(position=args.close_q, velocity=0.0, poll=True)
            now = time.perf_counter()
            if g.state.updated_at > last_feedback_value:
                last_feedback_value = float(g.state.updated_at)
                last_feedback_at = now
            if now - last_feedback_at > args.feedback_timeout:
                raise RuntimeError("feedback became stale while holding")
            if abs(float(g.state.torque)) > args.max_abs_torque:
                raise RuntimeError(
                    f"torque protection triggered while holding: {g.state.torque:+.3f} N·m"
                )
            time.sleep(args.loop_sleep)

        print("  Done!")

    except KeyboardInterrupt:
        print("\n[ctrl+c] exiting")
    except RuntimeError as exc:
        print(f"\n[abort] {exc}", file=sys.stderr)
        return 2
    finally:
        if enabled and g.is_connected:
            g.motor.disable()
            print("[disable] ok")
        g.disconnect()

    return 0


def _move_to(
    g: GloriaGripper,
    *,
    target: float,
    velocity: float,
    loop_sleep: float,
    print_hz: float,
    settle_threshold: float,
    settle_time: float,
    timeout: float,
    feedback_timeout: float,
    max_abs_torque: float,
) -> None:
    """Block until the motor reaches *target* within *settle_threshold* [rad].

    Each iteration sends a PV command with the given *target* and *velocity*.
    The motor's firmware handles internal trajectory planning; this function
    only monitors the feedback position and decides when to stop.

    Settle logic
    ------------
    The motor is considered "settled" when the position error stays below
    *settle_threshold* for at least *settle_time* seconds in a row.  A single
    overshoot resets the settle timer.

    Timeout
    -------
    If the position does not settle within *timeout* seconds the function
    raises ``RuntimeError`` so the caller disables the motor and does not
    report an unsuccessful move as complete.

    Parameters
    ----------
    g:                Connected GloriaGripper facade.
    target:           Target position [rad].
    velocity:         Speed limit [rad/s]; lower = slower and gentler.
    loop_sleep:       Sleep between iterations [s] (e.g. 0.01 s ≈ 100 Hz).
    print_hz:         Console status print rate [Hz].
    settle_threshold: Position error below which the motor is "at target" [rad].
    settle_time:      How long the error must stay below threshold [s].
    timeout:          Maximum segment duration before early return [s].
    """
    seg_start = time.perf_counter()
    settled_at: float | None = None
    last_print_at = 0.0
    print_interval = 1.0 / max(1.0, print_hz)
    last_feedback_value = float(g.state.updated_at)
    last_feedback_at = time.perf_counter()

    while True:
        g.motion.send_pos_vel(position=target, velocity=velocity, poll=True)

        now = time.perf_counter()
        elapsed = now - seg_start
        if g.state.updated_at > last_feedback_value:
            last_feedback_value = float(g.state.updated_at)
            last_feedback_at = now
        if now - last_feedback_at > feedback_timeout:
            raise RuntimeError(
                f"feedback stale for {now - last_feedback_at:.2f}s while moving to {target:+.3f}"
            )
        if abs(float(g.state.torque)) > max_abs_torque:
            raise RuntimeError(
                f"torque protection triggered: {g.state.torque:+.3f} N·m"
            )
        pos_err = abs(g.state.position - target)

        if (now - last_print_at) >= print_interval:
            last_print_at = now
            print(
                f"  t={elapsed:5.2f}s | "
                f"target={target:+.3f} q_fb={g.state.position:+.3f} | "
                f"vel={g.state.velocity:+.3f} tau={g.state.torque:+.3f}"
            )

        if pos_err < settle_threshold:
            if settled_at is None:
                settled_at = now
            elif (now - settled_at) >= settle_time:
                print(f"  Settled! (error {pos_err:.3f} rad)")
                break
        else:
            settled_at = None

        if elapsed > timeout:
            raise RuntimeError(
                f"move timeout: q_fb={g.state.position:+.3f}, "
                f"target={target:+.3f}, error={pos_err:.3f} rad"
            )

        time.sleep(loop_sleep)


def _wait_for_fresh_feedback(g: GloriaGripper, *, timeout_s: float) -> None:
    previous = float(g.state.updated_at)
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        g.motor.refresh()
        time.sleep(0.01)
        g.motor.poll()
        if g.state.updated_at > previous:
            return
    raise RuntimeError("no fresh motor feedback; check feedback ID and CAN wiring")


if __name__ == "__main__":
    raise SystemExit(main())
