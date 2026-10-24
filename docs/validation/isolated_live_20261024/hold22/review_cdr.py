"""Read-only CDR identity/content check for a completed hold22 run.

Requires the existing ROS 2 environment and flyarm message definitions. Prints
derived JSON without modifying source records or starting any ROS nodes.
"""
import base64
from collections import Counter
import hashlib
import json
from pathlib import Path
import sys

from rclpy.serialization import deserialize_message
from flyarm_control_interfaces import msg


def plain(value):
    if hasattr(value, "get_fields_and_field_types"):
        return {key: plain(getattr(value, key)) for key in value.get_fields_and_field_types()}
    if isinstance(value, dict):
        return {key: plain(item) for key, item in value.items()}
    if hasattr(value, "tolist"):
        return value.tolist()
    if hasattr(value, "item"):
        return value.item()
    if not isinstance(value, (str, int, float, bool, type(None))):
        return [plain(item) for item in value]
    return value


def main(run):
    root = Path(run) / "isolated_live"
    kinds = {"dispatch": msg.DispatchTick, "sends": msg.ActuatorSend,
             "receipts": msg.ExecutionReceipt, "events": msg.GateEvent}
    expected = {name: json.loads((root / (name + ".json")).read_text()) for name in kinds}
    counters = Counter()
    source = root / "source_recording/diagnostics.jsonl"
    for line in source.read_text().splitlines():
        record = json.loads(line)
        if record["kind"] != "ros":
            continue
        kind = record["topic"]
        decoded = deserialize_message(base64.b64decode(record["cdr"]["base64"]), kinds[kind])
        assert plain(decoded) == expected[kind][counters[kind]], (kind, counters[kind])
        counters[kind] += 1
    assert all(counters[name] == len(expected[name]) for name in kinds)
    candidates = []
    worker = root / "recording/diagnostics.jsonl"
    for line in worker.read_text().splitlines():
        record = json.loads(line)
        snapshot = deserialize_message(base64.b64decode(record["snapshot_cdr"]["base64"]), msg.ControlSnapshot)
        proposal = deserialize_message(base64.b64decode(record["proposal_cdr"]["base64"]), msg.ControlProposal)
        assert snapshot.sequence == proposal.sequence == record["sequence"]
        assert snapshot.session_id == proposal.session_id and snapshot.epoch == proposal.epoch
        assert not proposal.replay_only and not snapshot.replay_only and not record["error"]
        assert proposal.status == 0 and proposal.accepted_optimum and proposal.nonlinear_check_passed
        candidates.append({"proposal": plain(proposal), "snapshot": {
            "session_id": int(snapshot.session_id), "epoch": int(snapshot.epoch),
            "sequence": int(snapshot.sequence), "issued_monotonic_ns": int(snapshot.issued_monotonic_ns),
            "control_time_s": snapshot.control_time_s, "task_time_s": snapshot.task_time_s,
            "dt_s": snapshot.dt_s}})
    assert [x["proposal"]["sequence"] for x in candidates] == list(range(1, len(candidates) + 1))
    report = {"kind": "hold22_original_cdr_review_v1", "derived_analysis": True,
              "source_path": str(source), "source_sha256": hashlib.sha256(source.read_bytes()).hexdigest(),
              "worker_path": str(worker), "worker_sha256": hashlib.sha256(worker.read_bytes()).hexdigest(),
              "all_materialized_ros_fields_match_original_cdr": True, "counts": dict(counters),
              "all_worker_candidates_numerically_accepted": True, "candidates": candidates,
              "scope": "Byte-decoded message content and numerical status; not a full-hold pass or an execution receipt for every candidate."}
    print(json.dumps(report, indent=2, allow_nan=False))


if __name__ == "__main__":
    main(sys.argv[1])
