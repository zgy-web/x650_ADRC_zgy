"""Summarize PC-held hardware state JSONL; never label receipt as flight readiness."""
import argparse
import hashlib
import json
from pathlib import Path
import statistics


def summarize(path):
    data = path.read_bytes()
    rows = []
    non_json_lines = 0
    for line in data.decode("utf-8", errors="replace").splitlines():
        try:
            rows.append(json.loads(line))
        except ValueError:
            non_json_lines += 1  # rclcpp startup diagnostics are retained in input.
    samples = [x for x in rows if x.get("schema") == "flyarm.ros2.hardware_state.v1"]
    finals = [x for x in samples if x.get("final")]
    finishes = [x for x in rows if x.get("event") == "session_finished"]
    if len(finals) != 1 or len(finishes) != 1:
        raise ValueError("Incomplete/ambiguous session: need one final sample and one session_finished")
    final, finished = finals[0], finishes[0]
    settled = [x for x in samples if x["elapsed_s"] >= final["settle_s"]]
    topics = {k: dict(v["settled"]) for k, v in final["channels"].items()}
    for value in topics.values():
        value["first_to_last_receipt_span_s"] = ((value["count"]-1)/value["hz"] if value["hz"] else 0)
    expected = {"/fmu/out/"+name for name in ("vehicle_attitude", "vehicle_local_position_v1",
        "vehicle_odometry", "vehicle_status_v1", "sensor_combined", "estimator_status_flags", "timesync_status")}
    core = ["/fmu/out/"+name for name in ("vehicle_attitude", "vehicle_local_position_v1", "vehicle_odometry")]
    report_times = [x["elapsed_s"] for x in settled]
    complete_reports = (len(report_times) >= 60 and
        report_times[0] <= final["settle_s"]+1.5 and
        all(0 < b-a <= 1.5 for a,b in zip(report_times, report_times[1:])))
    rtts = [x["timesync"]["round_trip_time_us"] for x in settled
            if x["timesync"]["protocol"] == 2 and x["timesync"]["fresh"]]
    checks = {
        "probe_completed": finished["probe_exit"] == 0 and not finished["agent_exited_early"],
        "seven_topics_received": set(topics) == expected and all(x["count"] >= 2 for x in topics.values()),
        "source_stamps_monotone_nonzero": all(x["source_regressions"] == 0 and x["zero_source_timestamps"] == 0 for x in topics.values()),
        "no_armed_state_observed": not any(x["armed_seen"] for x in samples),
        "core_fresh_at_each_settled_report": bool(settled) and all(x["core_receipts_fresh"] for x in settled),
        "one_hz_report_window_complete": complete_reports,
        "at_least_60s_window": final["elapsed_s"] - final["settle_s"] >= 60,
        "core_receipts_cover_window": all(topics.get(k,{}).get("first_to_last_receipt_span_s",0) >= 59.5 for k in core),
    }
    return {
        "schema": "flyarm.ros2.hardware_state_summary.v1", "source_file": path.name,
        "source_sha256": hashlib.sha256(data).hexdigest(), "non_json_lines": non_json_lines,
        "checks": checks, "state_receive_checks_pass": all(checks.values()),
        "window_s": final["elapsed_s"]-final["settle_s"], "topics": topics,
        "timesync_rtt_1hz_snapshots_us": {
            "count": len(rtts), "min": min(rtts) if rtts else None,
            "median": statistics.median(rtts) if rtts else None, "max": max(rtts) if rtts else None},
        "final_body_flags": {k: final["body"][k] for k in ("attitude_usable","angular_velocity_usable",
            "position_velocity_usable","xy_valid","z_valid","v_xy_valid","v_z_valid","heading_good_for_control")},
        "final_estimator": final["estimator"], "final_status": final["status"],
        "limits": ["Receipt gaps use the NX steady clock; SSH transport is excluded.",
            "Receipt gaps and sampled Timesync RTT are not one-way USB latency or MPC compute latency.",
            "Status/estimator topics can be event-driven. Reported maxima are observations, not real-time bounds.",
            "This is an observation-only communication check, not position-control or flight approval."]}


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--input", type=Path, required=True)
    p.add_argument("--output", type=Path, required=True)
    args = p.parse_args()
    result = summarize(args.input)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("x", encoding="utf-8") as f:
        json.dump(result, f, ensure_ascii=False, indent=2, allow_nan=False)
    print(json.dumps({"output": str(args.output), "checks": result["checks"]}))
    return 0 if result["state_receive_checks_pass"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
