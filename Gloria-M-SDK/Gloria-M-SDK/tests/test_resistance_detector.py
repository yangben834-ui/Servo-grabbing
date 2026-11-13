from __future__ import annotations

import importlib.util
from pathlib import Path
import sys


_SCRIPT = Path(__file__).parents[1] / "demos" / "04_pv_resistance_stop.py"
_SPEC = importlib.util.spec_from_file_location("gloria_resistance_stop_demo", _SCRIPT)
assert _SPEC is not None and _SPEC.loader is not None
_MODULE = importlib.util.module_from_spec(_SPEC)
sys.modules[_SPEC.name] = _MODULE
_SPEC.loader.exec_module(_MODULE)


def _detector():
    return _MODULE.ResistanceDetector(
        torque_threshold=1.0,
        torque_debounce_s=0.1,
        stall_velocity=0.02,
        stall_debounce_s=0.3,
        minimum_travel=0.05,
        target_tolerance=0.05,
    )


def test_torque_requires_debounce() -> None:
    detector = _detector()
    common = dict(
        start_position=0.0,
        position=-0.1,
        velocity=-0.1,
        torque=1.2,
        target=-2.0,
    )
    assert detector.update(now=1.0, **common) is None
    assert detector.update(now=1.05, **common) is None
    assert detector.update(now=1.11, **common) == "torque"


def test_torque_spike_resets_debounce() -> None:
    detector = _detector()
    common = dict(
        start_position=0.0,
        position=-0.1,
        velocity=-0.1,
        target=-2.0,
    )
    assert detector.update(now=1.0, torque=1.2, **common) is None
    assert detector.update(now=1.05, torque=0.2, **common) is None
    assert detector.update(now=1.11, torque=1.2, **common) is None


def test_stall_requires_travel_and_debounce() -> None:
    detector = _detector()
    common = dict(
        start_position=0.0,
        position=-0.2,
        velocity=0.0,
        torque=0.2,
        target=-2.0,
    )
    assert detector.update(now=1.0, **common) is None
    assert detector.update(now=1.2, **common) is None
    assert detector.update(now=1.31, **common) == "stall"


def test_stall_is_ignored_near_target() -> None:
    detector = _detector()
    common = dict(
        start_position=0.0,
        position=-1.98,
        velocity=0.0,
        torque=0.2,
        target=-2.0,
    )
    assert detector.update(now=1.0, **common) is None
    assert detector.update(now=1.5, **common) is None
