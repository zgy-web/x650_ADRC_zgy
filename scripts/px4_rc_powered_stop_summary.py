"""Reparse powered SB-stop evidence; distinguish physical observation from telemetry.

Sequential NSH queries are not a synchronized latency measurement. The operator
confirmation is required in addition to fresh, mapped four-ESC telemetry.
"""
import argparse
import hashlib
import json
from pathlib import Path

from px4_debug_console import FIRMWARE
from px4_rc_safety_observer import summarize
from px4_rc_safety_summary import ordered, armed, zero
from px4_rc_switch_setup import fresh, field
from px4_motor_telemetry_check import telemetry_snapshot, assess_samples


def audit(report, *, operator_all_started_and_stopped=False):
    observations = report.get("rounds", []) + [report["final"]]
    rows, telemetry, errors = [], [], []
    for i, observation in enumerate(observations):
        raw = observation["raw"]
        rows.append(summarize(raw))
        try:
            for name, age in (("vehicle_status", 1.5), ("actuator_armed", 1.5),
                              ("manual_control_switches", 1.5), ("input_rc", .25),
                              ("manual_control_setpoint", .3)):
                fresh(raw[name], age)
            telemetry.append(telemetry_snapshot(raw["dshot"], raw["esc_status"], require_stopped=False))
        except (ValueError, KeyError) as exc:
            errors.append(dict(index=i, error=str(exc)))
            telemetry.append(None)

    def spinning(i):
        t = telemetry[i]
        return bool(t and all(e["esc_rpm"] > 0 for e in t["samples"].values()))

    def stopped(i):
        t = telemetry[i]
        return bool(t and all(e["esc_rpm"] == 0 for e in t["samples"].values()))

    indices = list(range(len(rows)))
    sequence = ordered(indices, [
        lambda i: armed(rows[i], True) and rows[i]["actuator_armed"].get("kill") is False
                  and all(rows[i].get("dshot", {}).get(k, 0) > 0 for k in range(4)) and spinning(i),
        lambda i: armed(rows[i], True) and rows[i]["actuator_armed"].get("kill") is True
                  and rows[i]["manual_control_switches"].get("kill_switch") == 1 and zero(rows[i]) and stopped(i),
        lambda i: armed(rows[i], False) and rows[i]["actuator_armed"].get("kill") is True
                  and zero(rows[i]) and stopped(i),
    ])
    reason = int(field(observations[-1]["raw"]["vehicle_status"], "latest_disarming_reason"))
    diagnostic = None
    try:
        diagnostic = assess_samples([t for t in telemetry if t])
        diagnostic["scope"] = "This powered start/stop observation window only; voltage/current are not calibrated"
    except ValueError as exc:
        errors.append(dict(error="telemetry counters: " + str(exc)))
    unknown = any("unknown" in x for s in rows for x in s.values())
    final = rows[-1]
    checks = dict(
        complete_capture=report.get("complete") is True,
        expected_firmware=report.get("firmware") == FIRMWARE,
        closed_port=report.get("port_closed") is True,
        no_tool_commands=all(report.get(k) == 0 for k in
                            ("arm_commands", "motor_commands", "mode_commands", "parameter_writes")),
        explicit_powered_bench=all(report.get("attestations", {}).get(k) is True for k in
                                  ("props_removed", "frame_secured", "operator_ready", "esc_power_connected")),
        parse_and_freshness_pass=not errors and not unknown,
        spinning_killed_then_disarmed=sequence is not None,
        reason_is_kill_switch=reason == 8,  # pinned firmware events/enums.json, not VehicleStatus.msg
        final_sf_off=final.get("manual_control_switches", {}).get("arm_switch") == 3,
        final_kill_set=final.get("actuator_armed", {}).get("kill") is True,
        final_disarmed=armed(final, False), final_zero=zero(final) and stopped(len(rows)-1),
        operator_all_started_and_stopped=operator_all_started_and_stopped is True,
    )
    spinning_samples = [{"index": i, "rpm": {str(k): e["esc_rpm"] for k, e in t["samples"].items()}}
                        for i, t in enumerate(telemetry) if t and spinning(i)]
    return dict(physical_sb_stop_verified=all(checks.values()), checks=checks, errors=errors,
                ordered_observation_indices=sequence, spinning_samples=spinning_samples,
                final_disarming_reason=reason, final_disarming_reason_name="kill_switch" if reason == 8 else "other",
                telemetry_counters=diagnostic, final=final, observations=len(rows),
                reaction_time_measured=False, airborne_stop_verified=False,
                nx_offboard_takeover_verified=False, flight_ready=False)


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--report", type=Path, required=True)
    p.add_argument("--output", type=Path, required=True)
    p.add_argument("--operator-all-started-and-stopped", action="store_true")
    args = p.parse_args()
    data = args.report.read_bytes()
    result = audit(json.loads(data), operator_all_started_and_stopped=args.operator_all_started_and_stopped)
    result.update(report_sha256=hashlib.sha256(data).hexdigest(),
                  auditor_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest())
    with args.output.open("x", encoding="utf-8") as stream:
        json.dump(result, stream, indent=2, ensure_ascii=False)
    print(json.dumps(result, indent=2, ensure_ascii=False))
    raise SystemExit(0 if result["physical_sb_stop_verified"] else 1)
