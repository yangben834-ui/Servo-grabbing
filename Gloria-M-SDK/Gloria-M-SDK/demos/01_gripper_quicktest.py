# Copyright (c) 2026 Synria Robotics Co., Ltd.
# Author: Synria Robotics Team
# Website: https://synriarobotics.ai

"""
01 – PV mode gripper reciprocating cycle quick test (Gripper Quick Test)

Description:
    The gripper repeatedly moves between open position (open_q) and close
    position (close_q) to quickly verify that the motor PV (position +
    velocity) control mode is working correctly.

How it works:
    In PV mode, each cycle sends a target position and movement velocity
    command to the motor.  The motor firmware handles trajectory planning
    internally.  Once the feedback position is within the settle threshold
    for the required dwell time, the direction switches automatically.

Usage:
    python 01_gripper_quicktest.py --port /dev/ttyACM0 --vel 0.1

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


# Calibrated twice on 2026-08-25.  Mechanical endpoints were approximately
# +0.1916 rad (open) and -2.5959 rad (closed); these defaults keep an inward
# safety margin at both ends.
OPEN_Q = 0.05
CLOSE_Q = -2.45
VELOCITY = 0.10
SEGMENT_TIMEOUT = 35.0
FEEDBACK_TIMEOUT = 0.50
MAX_ABS_TORQUE = 3.0


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
    ap = argparse.ArgumentParser(description="PV mode gripper reciprocating cycle")
    ap.add_argument("--port", default="/dev/ttyACM0", help="serial port")
    ap.add_argument("--baud", type=int, default=921600)
    ap.add_argument("--id", type=_parse_int, default="0x01", help="motor command CAN ID")
    ap.add_argument("--fb-id", type=_parse_int, default="0x101", help="motor feedback CAN ID")
    ap.add_argument("--open-q", type=float, default=OPEN_Q, help="calibrated safe open position [rad]")
    ap.add_argument("--close-q", type=float, default=CLOSE_Q, help="calibrated safe close position [rad]")
    ap.add_argument("--vel", type=float, default=VELOCITY, help="movement velocity [rad/s]")
    ap.add_argument("--loop-sleep", type=float, default=0.01, help="control loop sleep time [s]")
    ap.add_argument("--print-hz", type=float, default=5.0, help="print frequency [Hz]")
    ap.add_argument("--settle-threshold", type=float, default=0.05, help="settle threshold [rad]")
    ap.add_argument("--settle-time", type=float, default=0.3, help="dwell time after settling [s]")
    ap.add_argument("--timeout", type=float, default=SEGMENT_TIMEOUT, help="max time per segment [s]; timeout aborts the test")
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
        # Limits were already configured during commissioning.  Avoid a flash
        # write every time this repeatable motion demo starts.
        g.connect(apply_limits=False)
        g.motor.set_mode(ControlMode.POS_VEL)
        print("[mode] PV: ok")
        g.motor.enable()
        enabled = True
        print("[enable] ok")

        # Fetch the current position so the first segment starts from a
        # known state rather than from zero.
        _wait_for_fresh_feedback(g, timeout_s=args.feedback_timeout)
        print(f"[init] position = {g.state.position:.3f} rad")

        cycle = 0
        last_print_at = 0.0

        while True:
            # Alternate between close and open targets each cycle.
            if cycle % 2 == 0:
                target = float(args.close_q)
                direction = "closing"
            else:
                target = float(args.open_q)
                direction = "opening"

            print(f"\n--- Segment {cycle + 1}: {direction} → {target:.2f} rad ---")
            seg_start = time.perf_counter()
            settled_at = None
            last_feedback_value = float(g.state.updated_at)
            last_feedback_at = time.perf_counter()

            while True:
                # Send PV command every loop iteration.  The motor
                # firmware smooths the trajectory internally.
                g.motion.send_pos_vel(position=target, velocity=args.vel, poll=True)

                now = time.perf_counter()
                elapsed = now - seg_start
                if g.state.updated_at > last_feedback_value:
                    last_feedback_value = float(g.state.updated_at)
                    last_feedback_at = now
                if now - last_feedback_at > args.feedback_timeout:
                    raise RuntimeError(
                        f"feedback stale for {now - last_feedback_at:.2f}s; aborting"
                    )
                if abs(float(g.state.torque)) > args.max_abs_torque:
                    raise RuntimeError(
                        f"torque protection triggered: {g.state.torque:+.3f} N·m"
                    )
                pos_err = abs(g.state.position - target)

                # Rate-limited console output so the terminal doesn't flood.
                if (time.time() - last_print_at) >= 1.0 / max(1.0, float(args.print_hz)):
                    last_print_at = time.time()
                    print(
                        f"  t={elapsed:5.2f}s | "
                        f"target={target:+.3f} q_fb={g.state.position:+.3f} | "
                        f"vel={g.state.velocity:+.3f} tau={g.state.torque:+.3f}"
                    )

                # Settle detection: position must stay within the threshold
                # for the full settle_time before the segment is considered done.
                if pos_err < args.settle_threshold:
                    if settled_at is None:
                        settled_at = time.perf_counter()
                    elif (time.perf_counter() - settled_at) >= args.settle_time:
                        print(f"  Settled! (error {pos_err:.3f} rad)")
                        break
                else:
                    settled_at = None

                # A timeout is a failed move, not a completed segment.  Do
                # not reverse automatically after hitting a mechanical stop.
                if elapsed > args.timeout:
                    raise RuntimeError(
                        f"segment timeout: q_fb={g.state.position:+.3f}, "
                        f"target={target:+.3f}, error={pos_err:.3f} rad"
                    )

                time.sleep(float(args.loop_sleep))

            cycle += 1

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


def _wait_for_fresh_feedback(g: GloriaGripper, *, timeout_s: float) -> None:
    """Wait for a new state packet before any motion command is sent."""
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
