"""Fault-inject the real captured start/stop sequence; never access hardware."""
from copy import deepcopy
import json
from pathlib import Path
import sys
import pytest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "scripts"))
from px4_rc_powered_stop_summary import audit


def fixture():
    return json.loads((ROOT / "docs/validation/px4_powered_stop_20261030/sb_stop_report.json").read_text())


def test_actual_sequence_requires_operator_observation():
    r = fixture()
    assert not audit(r)["physical_sb_stop_verified"]
    a = audit(r, operator_all_started_and_stopped=True)
    assert a["physical_sb_stop_verified"]
    assert a["ordered_observation_indices"] == [79, 81, 84]
    assert not a["reaction_time_measured"] and not a["flight_ready"]


@pytest.mark.parametrize("fault", ["no_motion", "stale_esc", "wrong_mapping", "wrong_reason", "incomplete", "tool_arm"])
def test_missing_or_contradictory_evidence_fails(fault):
    r = deepcopy(fixture())
    if fault == "no_motion":
        r["rounds"] = r["rounds"][85:]
    elif fault == "stale_esc":
        r["rounds"][79]["raw"]["esc_status"] = "never published"
    elif fault == "wrong_mapping":
        r["rounds"][79]["raw"]["esc_status"] = r["rounds"][79]["raw"]["esc_status"].replace("actuator_function: 101", "actuator_function: 104")
    elif fault == "wrong_reason":
        r["final"]["raw"]["vehicle_status"] = r["final"]["raw"]["vehicle_status"].replace("latest_disarming_reason: 8", "latest_disarming_reason: 7")
    elif fault == "incomplete": r["complete"] = False
    elif fault == "tool_arm": r["arm_commands"] = 1
    assert not audit(r, operator_all_started_and_stopped=True)["physical_sb_stop_verified"]
