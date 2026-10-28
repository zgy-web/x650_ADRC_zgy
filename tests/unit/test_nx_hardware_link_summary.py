import copy
import importlib.util
import json
from pathlib import Path
import pytest

ROOT = Path(__file__).resolve().parents[2]
spec = importlib.util.spec_from_file_location("hw_link_summary", ROOT/"scripts/summarize_nx_hardware_link.py")
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


def fixture():
    rows = [dict(event="hardware_link_started", duration_s=75, settle_s=15, input_origin="physical_px4", source_sha256="a"*64)]
    rows += [dict(event="diagnostic_command_sent", command=401), dict(event="matched_diagnostic_ack", command=401, result=0)]
    for second in range(16, 76):
        rows.append(dict(schema="flyarm.hardware_link.v1", final=second==75, complete=second==75, bench_checks_pass=second==75,
          elapsed_s=second+.01, observed_state=True, ticks=3000, transport_fault=False, state_reset_seen=False,
          armed_seen=False, offboard_seen=False, hil_seen=False, diagnostic_commands_sent=1, ack_accepted=True, ack_result=0,
          flight_setpoints_sent=0, offboard_requests_sent=0, arm_requests_sent=0, mpc_executed=False, flight_ready=False))
    rows += [dict(event="hardware_link_finished", probe_exit=0, agent_exited_early=False),
             dict(event="hardware_link_cleanup", agent_stopped=True, probe_stopped=True)]
    return rows


def evaluate(tmp_path, rows):
    p = tmp_path/"synthetic.jsonl"
    p.write_text("\n".join(json.dumps(r) for r in rows), encoding="utf-8")
    return module.summarize(p)


def test_pass_never_implies_trajectory_or_flight(tmp_path):
    report = evaluate(tmp_path, fixture())
    assert report["diagnostic_link_pass"]
    assert report["trajectory_payload_received_by_px4"] == "NOT_TESTED"
    assert report["offboard_takeover"] == "NOT_TESTED"


@pytest.mark.parametrize("fault", ["missing_middle", "timeout", "stale", "reset", "armed", "no_ack", "denied", "duplicate_command", "flight_command", "simulation", "mpc_claim"])
def test_fault_cannot_pass(tmp_path, fault):
    rows = copy.deepcopy(fixture())
    if fault == "missing_middle": del rows[20:25]
    elif fault == "timeout": rows[-3]["complete"] = False
    elif fault == "stale": rows[20]["transport_fault"] = True
    elif fault == "reset": rows[20]["state_reset_seen"] = True
    elif fault == "armed": rows[20]["armed_seen"] = True
    elif fault == "no_ack": rows.pop(2)
    elif fault == "denied": rows[2]["result"] = 2
    elif fault == "duplicate_command": rows.insert(1, dict(event="diagnostic_command_sent", command=401))
    elif fault == "flight_command": rows[20]["flight_setpoints_sent"] = 1
    elif fault == "simulation": rows[0]["input_origin"] = "mock"
    elif fault == "mpc_claim": rows[20]["mpc_executed"] = True
    assert not evaluate(tmp_path, rows)["diagnostic_link_pass"]


def test_truncated_is_not_pass(tmp_path):
    with pytest.raises(ValueError, match="Incomplete"):
        evaluate(tmp_path, fixture()[:-1])
