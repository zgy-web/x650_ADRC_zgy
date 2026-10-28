"""Separate diagnostic receipt evidence from untested flight/control claims."""
import argparse
import hashlib
import json
from pathlib import Path


def summarize(path):
    rows = []
    for line in path.read_text(encoding="utf-8", errors="replace").splitlines():
        try:
            value = json.loads(line)
            if isinstance(value, dict):
                rows.append(value)
        except ValueError:
            pass
    def events(name):
        return [r for r in rows if r.get("event") == name]
    reports = [r for r in rows if r.get("schema") == "flyarm.hardware_link.v1"]
    finals = [r for r in reports if r.get("final") is True]
    started, finished, cleanup = events("hardware_link_started"), events("hardware_link_finished"), events("hardware_link_cleanup")
    if len(finals) != 1 or len(started) != 1 or len(finished) != 1 or len(cleanup) != 1:
        raise ValueError("Incomplete or ambiguous hardware session")
    final, start, end, cleaned = finals[0], started[0], finished[0], cleanup[0]
    sent, ack = events("diagnostic_command_sent"), events("matched_diagnostic_ack")
    settled = [r for r in reports if r.get("elapsed_s", 0) >= start["settle_s"]]
    gaps = [b["elapsed_s"]-a["elapsed_s"] for a, b in zip(settled, settled[1:])]
    checks = dict(
        hardware_provenance=start.get("input_origin") == "physical_px4",
        completed=final.get("complete") is True and final.get("bench_checks_pass") is True and end.get("probe_exit") == 0,
        full_window=final.get("elapsed_s", 0) >= start["duration_s"] and len(settled) >= 59 and
                    bool(gaps) and all(0 < g < 2.1 for g in gaps) and settled[0]["elapsed_s"] <= start["settle_s"]+2,
        real_state_received=final.get("observed_state") is True and final.get("ticks", 0) >= 2500,
        uninterrupted=bool(settled) and all(r.get("transport_fault") is False and not r.get("state_reset_seen") for r in settled),
        disarmed_no_takeover=all(r.get("armed_seen") is False and r.get("offboard_seen") is False and r.get("hil_seen") is False for r in reports),
        only_one_diagnostic=len(sent) == 1 and sent[0].get("command") == 401 and final.get("diagnostic_commands_sent") == 1,
        accepted_response=len(ack) == 1 and ack[0].get("command") == 401 and ack[0].get("result") == 0 and
                          final.get("ack_accepted") is True and final.get("ack_result") == 0,
        no_flight_output=all(r.get("flight_setpoints_sent") == 0 and r.get("offboard_requests_sent") == 0 and r.get("arm_requests_sent") == 0 for r in reports),
        no_overclaim=all(r.get("mpc_executed") is False and r.get("flight_ready") is False for r in reports),
        agent_alive=end.get("agent_exited_early") is False,
        stopped=cleaned.get("agent_stopped") is True and cleaned.get("probe_stopped") is True,
    )
    return dict(schema="flyarm.hardware_link_summary.v1", source_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
                program_source_sha256=start["source_sha256"], checks=checks, diagnostic_link_pass=all(checks.values()),
                trajectory_payload_received_by_px4="NOT_TESTED", offboard_takeover="NOT_TESTED",
                full_mpc_with_real_state="BLOCKED_PENDING_VALID_LOCALIZATION_AND_JOINT_FEEDBACK",
                final=final, limits=["401 ACCEPTED means the diagnostic request was handled; it does not mean arming checks passed.",
                  "ACK elapsed time is measured on NX, excludes SSH and is not one-way latency.",
                  "A command ACK cannot certify a TrajectorySetpoint payload or actuator consumption."])


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--input", type=Path, required=True)
    p.add_argument("--output", type=Path, required=True)
    a = p.parse_args()
    result = summarize(a.input)
    a.output.parent.mkdir(parents=True, exist_ok=True)
    a.output.write_text(json.dumps(result, indent=2, allow_nan=False), encoding="utf-8")
    print(json.dumps(result, indent=2, allow_nan=False))
    raise SystemExit(0 if result["diagnostic_link_pass"] else 2)
