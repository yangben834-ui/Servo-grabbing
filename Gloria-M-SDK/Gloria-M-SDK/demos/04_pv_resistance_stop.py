#!/usr/bin/env python3
"""Close the calibrated gripper and stop automatically on resistance.

The detector combines two independent signals:

1. feedback torque rises above the measured idle torque for a debounce period;
2. after meaningful travel, velocity remains near zero while the target is
   still far away (stall detection).

On resistance, target arrival, timeout, stale feedback, or Ctrl+C, the motor
is disabled before the serial port is closed.  No zero or flash parameter is
changed.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import os
import statistics
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
OPEN_Q = 2.641912
CLOSE_Q = 0.217076
CLOSE_VELOCITY = 0.08


@dataclass
class ResistanceDetector:
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


def _measure_idle_torque(
    g: GloriaGripper, *, samples: int, interval_s: float, timeout_s: float
) -> float:
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


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="PV close with automatic torque/stall resistance stop"
    )
    parser.add_argument("--port", default=PORT)
    parser.add_argument("--baud", type=int, default=921600)
    parser.add_argument("--id", type=_parse_int, default=0x01)
    parser.add_argument("--fb-id", type=_parse_int, default=0x101)
    parser.add_argument("--open-q", type=float, default=OPEN_Q)
    parser.add_argument("--close-q", type=float, default=CLOSE_Q)
    parser.add_argument("--velocity", type=float, default=CLOSE_VELOCITY)
    parser.add_argument(
        "--resistance-torque",
        type=float,
        default=1.0,
        help="required increase above idle absolute torque [N·m]",
    )
    parser.add_argument("--torque-debounce", type=float, default=0.12)
    parser.add_argument("--hard-torque-limit", type=float, default=3.0)
    parser.add_argument("--stall-velocity", type=float, default=0.025)
    parser.add_argument("--stall-time", type=float, default=0.35)
    parser.add_argument("--minimum-travel", type=float, default=0.05)
    parser.add_argument("--target-tolerance", type=float, default=0.05)
    parser.add_argument("--feedback-timeout", type=float, default=0.5)
    parser.add_argument("--timeout", type=float, default=40.0)
    parser.add_argument("--loop-sleep", type=float, default=0.02)
    parser.add_argument("--print-hz", type=float, default=10.0)
    parser.add_argument("--idle-samples", type=int, default=10)
    return parser


def main() -> int:
    args = _build_parser().parse_args()
    if args.close_q >= args.open_q:
        raise SystemExit("close-q must be smaller than open-q for this calibration")
    if args.velocity <= 0.0:
        raise SystemExit("velocity must be positive")
    if args.resistance_torque <= 0.0 or args.hard_torque_limit <= 0.0:
        raise SystemExit("torque thresholds must be positive")
    if args.idle_samples < 3:
        raise SystemExit("idle-samples must be at least 3")

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
        print(
            f"[start] q={start_position:+.3f} -> {args.close_q:+.3f} rad | "
            f"velocity={args.velocity:.3f} rad/s | idle_tau={idle_torque:.3f} | "
            f"resistance_threshold={torque_threshold:.3f} N·m"
        )

        started_at = time.monotonic()
        last_feedback_value = float(gripper.state.updated_at)
        last_feedback_at = started_at
        last_print_at = 0.0

        while True:
            gripper.motion.send_pos_vel(
                position=float(args.close_q),
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

            target_error = abs(float(state.position) - float(args.close_q))
            if target_error <= float(args.target_tolerance):
                print(
                    f"[stop] close target reached: q={state.position:+.3f}, "
                    f"error={target_error:.3f} rad"
                )
                break

            reason = detector.update(
                now=now,
                start_position=start_position,
                position=float(state.position),
                velocity=float(state.velocity),
                torque=float(state.torque),
                target=float(args.close_q),
            )
            if reason is not None:
                print(
                    f"[stop] resistance detected by {reason}: "
                    f"q={state.position:+.3f}, dq={state.velocity:+.3f}, "
                    f"tau={state.torque:+.3f} N·m"
                )
                break

            elapsed = now - started_at
            if elapsed >= float(args.timeout):
                raise RuntimeError(
                    f"motion timeout after {elapsed:.1f}s at q={state.position:+.3f}"
                )

            if now - last_print_at >= 1.0 / max(1.0, float(args.print_hz)):
                last_print_at = now
                print(
                    f"t={elapsed:5.2f}s | q={state.position:+.3f} "
                    f"dq={state.velocity:+.3f} tau={state.torque:+.3f} | "
                    f"threshold={torque_threshold:.3f}"
                )
            time.sleep(float(args.loop_sleep))

        return 0
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
