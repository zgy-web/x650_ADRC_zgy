"""Reject false positives caused by requests, old events or competing timers."""
from pathlib import Path
import hashlib
import json
import sys
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "scripts"))
import px4_rc_safety_summary as m


def row(*, armed=False, kill=False, lost=False, action=0, stamp=200, output=0):
    return {"vehicle_status": {"arming_state": 2 if armed else 1, "failsafe": lost},
            "actuator_armed": {"armed": armed, "kill": kill},
            "input_rc": {"rc_lost": lost, "rc_failsafe": lost},
            "manual_control_setpoint": {"valid": not lost},
            "failsafe_flags": {"manual_control_signal_lost": lost},
            "manual_control_switches": {"arm_switch": 3},
            "action_request": {"source": 1, "action": action, "timestamp": stamp},
            "dshot": {i: output if i < 4 else 0 for i in range(9)}}


def evaluate(rows, case, reason=2, refusal=False):
    return m.evaluate(rows, case, initial_timestamp=100, final_reason=reason, refusal_seen=refusal)[0]


def test_request_without_armed_state_does_not_prove_sf_operation():
    c = evaluate([row(action=1), row(action=0)], "sf_disarm")
    assert c["arm_request_observed"] and not c["armed_then_disarmed"]


def test_preflight_autodisarm_does_not_prove_sf_disarm():
    c = evaluate([row(armed=True, action=1, output=109), row(action=0)], "sf_disarm", reason=7)
    assert c["armed_then_disarmed"] and not c["disarming_reason_rc_switch"]


def test_preflight_timer_can_win_after_successful_kill_output_cut():
    c = evaluate([row(armed=True, action=1, output=109), row(armed=True, kill=True, action=4),
                  row(kill=True, action=4)], "kill_disarm", reason=7)
    assert c["kill_cutoff_observed"] and c["armed_killed_disarmed"]
    assert not c["disarming_reason_kill_switch"]


def test_only_disarmed_zero_output_does_not_prove_kill_cutoff():
    c = evaluate([row(), row(kill=True, action=4)], "kill_disarm", reason=8)
    assert not c["kill_cutoff_observed"]


def test_old_arm_event_cannot_prove_new_inhibition_test():
    c = evaluate([row(kill=True, action=1, stamp=50), row(kill=True)], "kill_inhibit", refusal=True)
    assert not c["arm_request_observed"]


def test_invalid_manual_flag_alone_is_not_rc_link_loss():
    x = row(); x["manual_control_setpoint"]["valid"] = False
    assert not m.rc(x, True)


def test_loss_restoration_must_follow_loss():
    c = evaluate([row(kill=True), row(kill=True), row(kill=True, lost=True)], "rc_loss_restore")
    assert not c["valid_lost_restored"]


def test_unknown_output_is_not_zero():
    x = row(); x["dshot"] = {"unknown": "truncated"}
    assert not m.zero(x)


def test_unarmed_loss_cannot_prove_armed_failsafe():
    c = evaluate([row(), row(lost=True), row(kill=True)], "armed_rc_loss")
    assert not c["armed_then_loss_failsafe_then_recovered"]


def test_successful_sf_chain_has_all_criteria():
    assert all(evaluate([row(), row(armed=True, action=1, output=109), row()], "sf_disarm").values())


def test_actual_kill_enum_is_not_message_schema_enum():
    rows = [row(armed=True, action=1, output=109), row(armed=True, kill=True, action=4), row(kill=True)]
    assert all(evaluate(rows, "kill_disarm", reason=8).values())
    assert not evaluate(rows, "kill_disarm", reason=9)["disarming_reason_kill_switch"]


def test_operator_kill_intervention_invalidates_isolated_rc_loss_run():
    rows = [row(armed=True, action=1, output=109), row(armed=True, lost=True), row(kill=True)]
    c = evaluate(rows, "armed_rc_loss", reason=7)
    assert c["armed_then_loss_failsafe_then_recovered"]
    assert not c["no_kill_intervention"]


def test_loss_only_after_disarming_does_not_prove_armed_failsafe():
    rows = [row(armed=True, action=1, output=109), row(lost=True), row()]
    assert not evaluate(rows, "armed_rc_loss")["armed_then_loss_failsafe_then_recovered"]


EVIDENCE = Path(__file__).resolve().parents[2] / "docs/validation/px4_rc_takeover_20261029"


def test_reason_table_matches_pinned_runtime_source():
    source = json.loads((EVIDENCE / "pinned_events_enums.json").read_bytes())
    entries = source["components"]["1"]["enums"]["arm_disarm_reason_t"]["entries"]
    assert m.REASONS == {int(k): v["name"] for k, v in entries.items()}


@pytest.mark.parametrize("name,case,expected", [
    ("kill_arm_inhibit_v1", "kill_inhibit", True),
    ("rc_loss_restore_v1", "rc_loss_restore", True),
    ("sf_arm_disarm_v1", "sf_disarm", True),
    ("sb_kill_disarm_v1", "kill_disarm", True),
    ("armed_rc_loss_v1", "armed_rc_loss", False),
    ("armed_rc_loss_v2", "armed_rc_loss", True),
])
def test_recorded_events_and_operator_contamination(name, case, expected):
    result = m.audit(json.loads((EVIDENCE / (name + ".json")).read_bytes()), case)
    assert result["observed_logic_pass"] is expected
    assert not result["flight_ready"]
    assert not result["physical_motor_stop_verified"]
    assert not result["nx_offboard_takeover_verified"]


def test_firmware_change_requires_new_reason_verification():
    r = json.loads((EVIDENCE / "sb_kill_disarm_v1.json").read_bytes())
    r["firmware"] = "unverified"
    result = m.audit(r, "kill_disarm")
    assert not result["observed_logic_pass"] and not result["checks"]["exact_firmware"]


def test_archived_evidence_bytes_match_original_manifest():
    manifest = json.loads((EVIDENCE / "evidence_manifest.json").read_bytes())
    for entry in manifest["files"]:
        data = (EVIDENCE / entry["file"]).read_bytes()
        assert hashlib.sha256(data).hexdigest() == entry["sha256"]
        assert len(data) == entry["bytes"]
