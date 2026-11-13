#!/usr/bin/env python3
"""Return the Gloria-M gripper to its calibrated safe fully-open position."""

from __future__ import annotations

import argparse
import os
import sys
import time


_REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
_SRC_DIR = os.path.join(_REPO_ROOT, "src")
if _SRC_DIR not in sys.path:
    sys.path.insert(0, _SRC_DIR)

from gloria_m_sdk import (  # noqa: E402
    ControlMode,
    GloriaGripper,
    Limits,
    PositionRange,
)


PORT = "/dev/ttyACM0"
SAFE_OPEN_Q = 2.641912
SAFE_CLOSE_Q = 0.217076
OPEN_VELOCITY = 0.15


def _parse_int(value: str) -> int:
    return int(value, 0)


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


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Return the gripper to its calibrated safe open position"
    )
    parser.add_argument("--port", default=PORT)
    parser.add_argument("--baud", type=int, default=921600)
    parser.add_argument("--id", type=_parse_int, default=0x01)
    parser.add_argument("--fb-id", type=_parse_int, default=0x101)
    parser.add_argument("--open-q", type=float, default=SAFE_OPEN_Q)
    parser.add_argument("--close-q", type=float, default=SAFE_CLOSE_Q)
    parser.add_argument("--velocity", type=float, default=OPEN_VELOCITY)
    parser.add_argument("--target-tolerance", type=float, default=0.03)
    parser.add_argument("--hard-torque-limit", type=float, default=2.0)
    parser.add_argument("--stall-velocity", type=float, default=0.02)
    parser.add_argument("--stall-time", type=float, default=0.40)
    parser.add_argument("--minimum-travel", type=float, default=0.05)
    parser.add_argument("--feedback-timeout", type=float, default=0.5)
    parser.add_argument("--timeout", type=float, default=25.0)
    parser.add_argument("--loop-sleep", type=float, default=0.01)
    parser.add_argument("--print-hz", type=float, default=10.0)
    return parser


def main() -> int:
    args = _build_parser().parse_args()
    if args.open_q <= args.close_q:
        raise SystemExit("open-q must be larger than close-q for this calibration")
    if args.velocity <= 0.0:
        raise SystemExit("velocity must be positive")
    if args.hard_torque_limit <= 0.0:
        raise SystemExit("hard-torque-limit must be positive")

    safe_position = PositionRange(
        min=float(args.close_q), max=float(args.open_q)
    )
    gripper = GloriaGripper(
        args.port,
        baudrate=args.baud,
        command_id=args.id,
        feedback_id=args.fb_id,
        limits=Limits(pmax=3.14, vmax=10.0, tmax=12.0),
        safe_position=safe_position,
    )

    enabled = False
    try:
        gripper.connect(apply_limits=False)
        gripper.motor.set_mode(ControlMode.POS_VEL)
        print("[mode] PV: ok")
        gripper.motor.enable()
        enabled = True
        print("[enable] ok")

        _wait_for_fresh_feedback(gripper, timeout_s=args.feedback_timeout)
        start_position = float(gripper.state.position)
        if not safe_position.min - 0.20 <= start_position <= safe_position.max + 0.20:
            raise RuntimeError(
                f"initial position {start_position:+.3f} rad is outside the "
                "calibrated range"
            )

        print(
            f"[start] q={start_position:+.3f} -> {args.open_q:+.3f} rad | "
            f"velocity={args.velocity:.3f} rad/s"
        )

        started_at = time.monotonic()
        last_feedback_value = float(gripper.state.updated_at)
        last_feedback_at = started_at
        last_print_at = 0.0
        stall_started_at: float | None = None

        while True:
            gripper.motion.send_pos_vel(
                position=float(args.open_q),
                velocity=float(args.velocity),
                poll=True,
            )
            now = time.monotonic()
            state = gripper.state

            if state.updated_at > last_feedback_value:
                last_feedback_value = float(state.updated_at)
                last_feedback_at = now
            if now - last_feedback_at > float(args.feedback_timeout):
                raise RuntimeError(
                    f"feedback stale for {now - last_feedback_at:.2f}s"
                )
            if abs(float(state.torque)) >= float(args.hard_torque_limit):
                raise RuntimeError(
                    f"hard torque limit triggered: {state.torque:+.3f} N·m"
                )

            target_error = abs(float(state.position) - float(args.open_q))
            if target_error <= float(args.target_tolerance):
                print(
                    f"[home] open position reached: q={state.position:+.3f}, "
                    f"error={target_error:.3f} rad"
                )
                return 0

            travelled = abs(float(state.position) - start_position)
            stalled = (
                travelled >= float(args.minimum_travel)
                and abs(float(state.velocity)) <= float(args.stall_velocity)
            )
            if stalled:
                if stall_started_at is None:
                    stall_started_at = now
                elif now - stall_started_at >= float(args.stall_time):
                    raise RuntimeError(
                        f"opening stalled at q={state.position:+.3f}, "
                        f"dq={state.velocity:+.3f}"
                    )
            else:
                stall_started_at = None

            elapsed = now - started_at
            if elapsed >= float(args.timeout):
                raise RuntimeError(
                    f"open timeout after {elapsed:.1f}s at q={state.position:+.3f}"
                )

            if now - last_print_at >= 1.0 / max(1.0, float(args.print_hz)):
                last_print_at = now
                print(
                    f"t={elapsed:5.2f}s | target={args.open_q:+.3f} "
                    f"q={state.position:+.3f} dq={state.velocity:+.3f} "
                    f"tau={state.torque:+.3f}"
                )
            time.sleep(float(args.loop_sleep))

    except KeyboardInterrupt:
        print("\n[ctrl+c] stop requested")
        return 130
    except RuntimeError as exc:
        print(f"\n[abort] {exc}", file=sys.stderr)
        return 2
    finally:
        if enabled and gripper.is_connected:
            gripper.motor.disable()
            print("[disable] ok")
        gripper.disconnect()


if __name__ == "__main__":
    raise SystemExit(main())
