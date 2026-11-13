#!/usr/bin/env python3
"""Interactive, non-actuating angle calibration for a Gloria-M gripper.

The motor is kept disabled throughout the procedure.  The operator manually
moves the unloaded gripper to its open and closed endpoints; this script then
samples feedback, rejects outliers, checks repeatability, and calculates safe
software endpoints with an inward margin.

This script deliberately never calls ``set_zero()`` and never sends a motion
command.
"""

from __future__ import annotations

import argparse
from dataclasses import asdict, dataclass
from datetime import datetime
import json
import math
import os
from pathlib import Path
import statistics
import sys
import time
from typing import Iterable


_REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
_SRC_DIR = os.path.join(_REPO_ROOT, "src")
if _SRC_DIR not in sys.path:
    sys.path.insert(0, _SRC_DIR)

from gloria_m_sdk import GloriaGripper  # noqa: E402


@dataclass(frozen=True)
class EndpointStats:
    name: str
    position: float
    minimum: float
    maximum: float
    stddev: float
    samples: int


@dataclass(frozen=True)
class CalibrationResult:
    open_mechanical: float
    close_mechanical: float
    safe_open: float
    safe_close: float
    safe_min: float
    safe_max: float
    travel: float
    inward_margin: float
    opening_direction: int


def _robust_stats(name: str, values: Iterable[float]) -> EndpointStats:
    data = [float(value) for value in values if math.isfinite(float(value))]
    if len(data) < 3:
        raise ValueError(f"{name}: at least 3 valid samples are required")

    median = statistics.median(data)
    deviations = [abs(value - median) for value in data]
    mad = statistics.median(deviations)

    # A zero MAD is common for quantized encoder feedback.  In that case keep
    # every sample close to the median while still rejecting obvious glitches.
    tolerance = max(6.0 * mad, 0.01)
    filtered = [value for value in data if abs(value - median) <= tolerance]
    if len(filtered) < 3:
        raise ValueError(f"{name}: too many inconsistent feedback samples")

    return EndpointStats(
        name=name,
        position=statistics.median(filtered),
        minimum=min(filtered),
        maximum=max(filtered),
        stddev=statistics.pstdev(filtered),
        samples=len(filtered),
    )


def _derive_calibration(
    open_position: float,
    close_position: float,
    *,
    margin: float,
    minimum_travel: float,
) -> CalibrationResult:
    travel = abs(open_position - close_position)
    if travel < minimum_travel:
        raise ValueError(
            f"measured travel {travel:.4f} rad is below the required "
            f"{minimum_travel:.4f} rad; the gripper may not have been moved"
        )
    if margin <= 0.0:
        raise ValueError("margin must be positive")
    if margin * 2.0 >= travel:
        raise ValueError(
            f"margin {margin:.4f} rad is too large for travel {travel:.4f} rad"
        )

    opening_direction = 1 if open_position > close_position else -1
    safe_open = open_position - opening_direction * margin
    safe_close = close_position + opening_direction * margin
    return CalibrationResult(
        open_mechanical=open_position,
        close_mechanical=close_position,
        safe_open=safe_open,
        safe_close=safe_close,
        safe_min=min(safe_open, safe_close),
        safe_max=max(safe_open, safe_close),
        travel=travel,
        inward_margin=margin,
        opening_direction=opening_direction,
    )


def _collect_samples(
    gripper: GloriaGripper,
    *,
    count: int,
    interval_s: float,
    timeout_s: float,
) -> list[float]:
    values: list[float] = []
    last_update = float(gripper.state.updated_at)
    deadline = time.monotonic() + timeout_s

    while len(values) < count and time.monotonic() < deadline:
        gripper.motor.refresh()
        time.sleep(interval_s)
        gripper.motor.poll()
        state = gripper.state
        if state.updated_at > last_update:
            values.append(float(state.position))
            last_update = float(state.updated_at)
            print(
                f"\r  samples {len(values):>2}/{count}: "
                f"q={state.position:+.4f} rad  "
                f"dq={state.velocity:+.4f} rad/s  "
                f"tau={state.torque:+.4f} N·m",
                end="",
                flush=True,
            )

    print()
    if len(values) < count:
        raise RuntimeError(
            f"only received {len(values)}/{count} fresh feedback samples; "
            "check --fb-id, CAN wiring, and motor power"
        )
    return values


def _wait_for_operator(message: str) -> None:
    try:
        input(message)
    except EOFError as exc:
        raise RuntimeError("interactive terminal input is required") from exc


def _print_endpoint(stats: EndpointStats) -> None:
    print(
        f"  {stats.name}: q={stats.position:+.4f} rad, "
        f"range=[{stats.minimum:+.4f}, {stats.maximum:+.4f}], "
        f"std={stats.stddev:.5f}, n={stats.samples}"
    )


def _save_result(
    path: Path,
    *,
    port: str,
    command_id: int,
    feedback_id: int,
    open_stats: EndpointStats,
    close_stats: EndpointStats,
    result: CalibrationResult,
) -> None:
    payload = {
        "created_at": datetime.now().astimezone().isoformat(),
        "port": port,
        "command_id": command_id,
        "feedback_id": feedback_id,
        "open_samples": asdict(open_stats),
        "close_samples": asdict(close_stats),
        "calibration": asdict(result),
    }
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")


def _parse_int(value: str) -> int:
    return int(value, 0)


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Manually position the disabled gripper and record safe angle endpoints"
    )
    parser.add_argument("--port", default="/dev/ttyACM0", help="serial port")
    parser.add_argument("--baud", type=int, default=921600)
    parser.add_argument("--id", type=_parse_int, default=0x01, help="motor command CAN ID")
    parser.add_argument("--fb-id", type=_parse_int, default=0x101, help="motor feedback CAN ID")
    parser.add_argument("--samples", type=int, default=20, help="fresh samples per endpoint")
    parser.add_argument("--sample-interval", type=float, default=0.05, help="sample interval [s]")
    parser.add_argument("--sample-timeout", type=float, default=5.0, help="timeout per endpoint [s]")
    parser.add_argument(
        "--margin",
        type=float,
        default=None,
        help="inward safety margin [rad]; default is max(0.02, 5%% of measured travel)",
    )
    parser.add_argument("--min-travel", type=float, default=0.05, help="minimum valid travel [rad]")
    parser.add_argument(
        "--output",
        default=None,
        help="JSON output path; default: demos/calibration/angle_calibration_TIMESTAMP.json",
    )
    parser.add_argument("--no-save", action="store_true", help="print only; do not save JSON")
    return parser


def main() -> int:
    args = _build_parser().parse_args()
    if args.samples < 3:
        raise SystemExit("--samples must be at least 3")
    if args.sample_interval <= 0.0 or args.sample_timeout <= 0.0:
        raise SystemExit("sample interval and timeout must be positive")

    print("Gloria-M manual angle calibration")
    print("  - The motor will remain DISABLED.")
    print("  - No motion command and no set_zero command will be sent.")
    print("  - Remove all objects before calibration.")
    print("  - Do not force a non-backdrivable mechanism by hand.\n")

    gripper = GloriaGripper(
        args.port,
        baudrate=args.baud,
        command_id=args.id,
        feedback_id=args.fb_id,
    )

    connected = False
    try:
        # Avoid writing PMAX/VMAX/TMAX and motor flash during calibration.
        gripper.connect(apply_limits=False)
        connected = True
        gripper.motor.disable()
        print(f"[connected] {args.port}; motor disabled\n")

        _wait_for_operator(
            "Manually place the unloaded gripper at MAXIMUM OPEN. "
            "Do not force it; press Enter when stable... "
        )
        open_stats = _robust_stats(
            "OPEN",
            _collect_samples(
                gripper,
                count=args.samples,
                interval_s=args.sample_interval,
                timeout_s=args.sample_timeout,
            ),
        )
        _print_endpoint(open_stats)

        _wait_for_operator(
            "\nManually place the unloaded gripper at FULLY CLOSED. "
            "Do not squeeze the hard stop; press Enter when stable... "
        )
        close_stats = _robust_stats(
            "CLOSE",
            _collect_samples(
                gripper,
                count=args.samples,
                interval_s=args.sample_interval,
                timeout_s=args.sample_timeout,
            ),
        )
        _print_endpoint(close_stats)

        travel = abs(open_stats.position - close_stats.position)
        margin = args.margin if args.margin is not None else max(0.02, 0.05 * travel)
        result = _derive_calibration(
            open_stats.position,
            close_stats.position,
            margin=margin,
            minimum_travel=args.min_travel,
        )

        direction_text = "increasing angle" if result.opening_direction > 0 else "decreasing angle"
        print("\nCalibration result")
        print(f"  mechanical travel : {result.travel:.4f} rad")
        print(f"  opening direction : {direction_text}")
        print(f"  inward margin     : {result.inward_margin:.4f} rad")
        print(f"  safe OPEN_Q       : {result.safe_open:+.4f}")
        print(f"  safe CLOSE_Q      : {result.safe_close:+.4f}")
        print(f"  PositionRange     : min={result.safe_min:+.4f}, max={result.safe_max:+.4f}")
        print("\nPaste into the PV demos:")
        print(f"OPEN_Q  = {result.safe_open:.6f}")
        print(f"CLOSE_Q = {result.safe_close:.6f}")

        if not args.no_save:
            if args.output:
                output = Path(args.output).expanduser()
            else:
                stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                output = Path(_REPO_ROOT) / "demos" / "calibration" / f"angle_calibration_{stamp}.json"
            _save_result(
                output,
                port=args.port,
                command_id=args.id,
                feedback_id=args.fb_id,
                open_stats=open_stats,
                close_stats=close_stats,
                result=result,
            )
            print(f"\n[saved] {output}")

        print("\nNo motor zero or flash parameter was changed.")
        return 0
    except KeyboardInterrupt:
        print("\n[cancelled] calibration interrupted")
        return 130
    except (RuntimeError, ValueError) as exc:
        print(f"\n[error] {exc}", file=sys.stderr)
        return 2
    finally:
        if connected:
            try:
                gripper.motor.disable()
            finally:
                gripper.disconnect()


if __name__ == "__main__":
    raise SystemExit(main())
