from __future__ import annotations

import importlib.util
from pathlib import Path
import sys

import pytest


_SCRIPT = Path(__file__).parents[1] / "demos" / "00_angle_calibration.py"
_SPEC = importlib.util.spec_from_file_location("gloria_angle_calibration_demo", _SCRIPT)
assert _SPEC is not None and _SPEC.loader is not None
_MODULE = importlib.util.module_from_spec(_SPEC)
sys.modules[_SPEC.name] = _MODULE
_SPEC.loader.exec_module(_MODULE)


def test_robust_stats_rejects_feedback_glitch() -> None:
    stats = _MODULE._robust_stats("OPEN", [0.18, 0.181, 0.179, 0.18, 2.5])
    assert stats.samples == 4
    assert stats.position == pytest.approx(0.18)
    assert stats.maximum < 0.2


def test_derive_calibration_when_open_angle_is_larger() -> None:
    result = _MODULE._derive_calibration(
        0.5, -0.5, margin=0.05, minimum_travel=0.1
    )
    assert result.opening_direction == 1
    assert result.safe_open == pytest.approx(0.45)
    assert result.safe_close == pytest.approx(-0.45)
    assert result.safe_min == pytest.approx(-0.45)
    assert result.safe_max == pytest.approx(0.45)


def test_derive_calibration_when_open_angle_is_smaller() -> None:
    result = _MODULE._derive_calibration(
        -0.5, 0.5, margin=0.05, minimum_travel=0.1
    )
    assert result.opening_direction == -1
    assert result.safe_open == pytest.approx(-0.45)
    assert result.safe_close == pytest.approx(0.45)


def test_derive_calibration_rejects_tiny_travel() -> None:
    with pytest.raises(ValueError, match="below the required"):
        _MODULE._derive_calibration(
            0.18, 0.17, margin=0.002, minimum_travel=0.05
        )
