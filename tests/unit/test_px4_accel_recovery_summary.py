"""Regression of recovery acceptance against the real archived bench evidence."""
import json
from pathlib import Path
import shutil
import sys

import pytest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "scripts"))
from summarize_px4_accel_recovery import summarize

EVIDENCE = ROOT / "docs/validation/px4_accel_cal_20261029"


def test_real_failed_calibration_and_recovered_baseline_are_distinct():
    result = summarize(EVIDENCE)
    assert result["capture_completed"]
    assert not result["new_calibration_quality_accepted"]
    assert result["restored_parameter_persistence_verified"]
    assert result["restored_accel_bias_gate_passed_in_observed_window"]
    assert result["restored_max_abs_accel_bias"] == pytest.approx(0.03572)
    assert not result["flight_ready"]


@pytest.mark.parametrize("field", ["parameters", "assessment"])
def test_changed_reboot_evidence_cannot_be_reported_as_restored(tmp_path, field):
    evidence = tmp_path / "evidence"
    shutil.copytree(EVIDENCE, evidence)
    path = evidence / "restored_reboot_report.json"
    report = json.loads(path.read_text(encoding="utf-8"))
    if field == "parameters":
        report["parameters"]["CAL_ACC0_XSCALE"] = 1.07
    else:
        report["samples"][0]["assessment"]["0"]["accel"]["bias"][0] = 0.4
    path.write_text(json.dumps(report), encoding="utf-8")
    with pytest.raises(ValueError):
        summarize(evidence)
