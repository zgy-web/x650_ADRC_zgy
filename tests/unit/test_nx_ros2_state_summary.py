"""Incomplete telemetry must not be promoted to a successful hardware check."""
import copy
import importlib.util
import json
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[2]
spec = importlib.util.spec_from_file_location("nx_state_summary", ROOT/"scripts/summarize_nx_ros2_state_session.py")
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


def reports():
    # Real final measurements, wrapped in a synthetic report stream to exercise
    # incomplete/truncated/failed recording handling. This is not a new flight.
    summary = json.loads((ROOT/"docs/validation/nx_ros2_state_20261028/state_receive_summary.json").read_text())
    final = dict(schema="flyarm.ros2.hardware_state.v1", final=True, elapsed_s=75.003,
        settle_s=15, armed_seen=False, core_receipts_fresh=True,
        body=summary["final_body_flags"], estimator=summary["final_estimator"],
        status=summary["final_status"], timesync=dict(fresh=True, protocol=2, round_trip_time_us=600),
        channels={k:dict(settled=v) for k,v in summary["topics"].items()})
    rows=[]
    for second in range(16,76):
        row=copy.deepcopy(final); row.update(elapsed_s=second+.003,final=second==75); rows.append(row)
    rows.append(dict(event="session_finished",probe_exit=0,agent_exited_early=False))
    return rows


def evaluate(tmp_path, rows):
    path=tmp_path/"session.jsonl"
    path.write_text("rclcpp diagnostic retained\n"+"\n".join(json.dumps(x) for x in rows))
    return module.summarize(path)


def test_transport_pass_does_not_make_invalid_position_valid(tmp_path):
    result=evaluate(tmp_path,reports())
    assert result["state_receive_checks_pass"]
    assert not result["final_body_flags"]["position_velocity_usable"]


@pytest.mark.parametrize("fault",["missing_middle","stale","armed","regression","no_topic","agent_exited"])
def test_faults_cannot_pass(tmp_path,fault):
    rows=reports()
    if fault=="missing_middle": del rows[20:25]
    elif fault=="stale": rows[20]["core_receipts_fresh"]=False
    elif fault=="armed": rows[20]["armed_seen"]=True
    elif fault=="regression": rows[-2]["channels"]["/fmu/out/vehicle_attitude"]["settled"]["source_regressions"]=1
    elif fault=="no_topic": rows[-2]["channels"].pop("/fmu/out/vehicle_odometry")
    elif fault=="agent_exited": rows[-1]["agent_exited_early"]=True
    assert not evaluate(tmp_path,rows)["state_receive_checks_pass"]


def test_truncated_session_is_rejected(tmp_path):
    with pytest.raises(ValueError,match="Incomplete"):
        evaluate(tmp_path,reports()[:-1])
