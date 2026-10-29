"""Offline audit of power-disconnected RC observations, never a flight approval.

Serial topics are sequential snapshots, not an atomic sample. This auditor
checks observed event order and final state; it does not measure reaction time.
"""
import argparse
import hashlib
import json
from pathlib import Path

from px4_rc_safety_observer import summarize, TOPICS
from px4_debug_console import FIRMWARE
from px4_rc_switch_setup import field


CASES = ("kill_inhibit", "rc_loss_restore", "sf_disarm", "kill_disarm", "armed_rc_loss")
# This firmware publishes events::px4::enums::arm_disarm_reason_t directly.
# Its events/enums.json differs from VehicleStatus.msg after mission_start:
# runtime 7 = auto_disarm_preflight, 8 = kill_switch (not message 8 and 9).
# See the pinned source and observed console strings in the validation report.
REASONS = {0: "transition_to_standby", 1: "stick_gesture", 2: "rc_switch",
           3: "command_internal", 4: "command_external", 5: "mission_start",
           6: "auto_disarm_land", 7: "auto_disarm_preflight", 8: "kill_switch",
           9: "lockdown", 10: "failure_detector", 11: "shutdown",
           12: "unit_test", 13: "rc_button", 14: "failsafe"}


def zero(s):
    values = s.get("dshot", {})
    return set(values) == set(range(9)) and all(v == 0 for v in values.values())


def rc(s, lost):
    return (s.get("input_rc", {}).get("rc_lost") is lost
            and s.get("input_rc", {}).get("rc_failsafe") is lost
            and s.get("manual_control_setpoint", {}).get("valid") is not lost
            and s.get("failsafe_flags", {}).get("manual_control_signal_lost") is lost)


def armed(s, value):
    return (s.get("actuator_armed", {}).get("armed") is value
            and s.get("vehicle_status", {}).get("arming_state") == (2 if value else 1))


def ordered(rows, predicates):
    found, start = [], 0
    for predicate in predicates:
        match = next((i for i in range(start, len(rows)) if predicate(rows[i])), None)
        if match is None:
            return None
        found.append(match)
        start = match + 1
    return found


def evaluate(rows, case, *, initial_timestamp, final_reason, refusal_seen):
    """Per-scenario evidence; callers must additionally enforce capture quality."""
    if case not in CASES or not rows:
        raise ValueError("Known scenario and nonempty observations required")
    final = rows[-1]
    def action(s, value):
        a = s.get("action_request", {})
        return a.get("source") == 1 and a.get("action") == value and a.get("timestamp", 0) > initial_timestamp
    checks = {"ends_disarmed": armed(final, False), "ends_zero": zero(final),
              "ends_rc_valid": rc(final, False),
              "ends_sf_off": final.get("manual_control_switches", {}).get("arm_switch") == 3}
    sequence = None
    if case == "kill_inhibit":
        checks.update(arm_request_observed=any(action(s, 1) for s in rows),
                      explicit_refusal_observed=refusal_seen,
                      all_disarmed=all(armed(s, False) for s in rows),
                      all_kill=all(s.get("actuator_armed", {}).get("kill") is True for s in rows),
                      all_zero=all(zero(s) for s in rows))
    elif case == "rc_loss_restore":
        sequence = ordered(rows, [lambda s: rc(s, False), lambda s: rc(s, True), lambda s: rc(s, False)])
        checks.update(valid_lost_restored=sequence is not None,
                      all_disarmed=all(armed(s, False) for s in rows),
                      all_kill=all(s.get("actuator_armed", {}).get("kill") is True for s in rows),
                      all_zero=all(zero(s) for s in rows))
    elif case == "sf_disarm":
        sequence = ordered(rows, [lambda s: armed(s, True) and not zero(s),
                                  lambda s: armed(s, False) and zero(s)])
        checks.update(armed_then_disarmed=sequence is not None,
                      arm_request_observed=any(action(s, 1) for s in rows),
                      disarm_request_observed=any(action(s, 0) for s in rows),
                      disarming_reason_rc_switch=final_reason == 2,
                      all_kill_released=all(s.get("actuator_armed", {}).get("kill") is False for s in rows))
    elif case == "kill_disarm":
        cutoff = ordered(rows, [lambda s: armed(s, True) and s.get("actuator_armed", {}).get("kill") is False and not zero(s),
                                lambda s: armed(s, True) and s.get("actuator_armed", {}).get("kill") is True and zero(s)])
        sequence = ordered(rows, [lambda s: armed(s, True) and s.get("actuator_armed", {}).get("kill") is False and not zero(s),
                                  lambda s: armed(s, True) and s.get("actuator_armed", {}).get("kill") is True and zero(s),
                                  lambda s: armed(s, False) and s.get("actuator_armed", {}).get("kill") is True and zero(s)])
        checks.update(kill_cutoff_observed=cutoff is not None,
                      armed_killed_disarmed=sequence is not None,
                      kill_request_observed=any(action(s, 4) for s in rows),
                      disarming_reason_kill_switch=final_reason == 8)
    elif case == "armed_rc_loss":
        sequence = ordered(rows, [lambda s: armed(s, True) and rc(s, False),
                                  lambda s: armed(s, True) and rc(s, True) and s.get("vehicle_status", {}).get("failsafe") is True,
                                  lambda s: armed(s, False) and rc(s, False) and zero(s)])
        checks.update(armed_then_loss_failsafe_then_recovered=sequence is not None,
                      no_kill_intervention=all(s.get("actuator_armed", {}).get("kill") is False for s in rows))
    return checks, sequence


def audit(report, case):
    raw = [x["raw"] for x in report["rounds"]] + [report["final_raw"]]
    rows = [summarize(x) for x in raw]
    unknown = [(i, name) for i, s in enumerate(rows) for name, value in s.items() if "unknown" in value]
    text = "\n".join(value for s in raw for value in s.values())
    try:
        final_reason = int(field(raw[-1]["vehicle_status"], "latest_disarming_reason"))
    except ValueError:
        final_reason = None
    checks, sequence = evaluate(rows, case, initial_timestamp=report["before"]["state"]["timestamp"],
                                final_reason=final_reason, refusal_seen="Arming denied:" in text)
    checks.update(capture_complete=report.get("complete") is True,
                  exact_firmware=report.get("firmware") == FIRMWARE,
                  port_closed=report.get("port_closed") is True,
                  operator_power_off=report.get("operator_power_off") is True,
                  no_unknown_fields=not unknown and all(set(s) == set(TOPICS) | {"dshot"} for s in rows),
                  no_tool_control_commands=all(report.get(k) == 0 for k in ("arm_commands", "motor_commands", "mode_commands", "parameter_writes")))
    return dict(case=case, observed_logic_pass=all(checks.values()), checks=checks,
                ordered_observation_indices=sequence, observations=len(rows), unknown=unknown,
                final_disarming_reason=final_reason, final_disarming_reason_name=REASONS.get(final_reason, "unknown"), final=rows[-1],
                physical_motor_stop_verified=False, airborne_failsafe_verified=False,
                nx_offboard_takeover_verified=False, reaction_time_measured=False,
                flight_ready=False)


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--report", type=Path, required=True)
    p.add_argument("--case", choices=CASES, required=True)
    p.add_argument("--output", type=Path, required=True)
    a = p.parse_args()
    data = a.report.read_bytes()
    result = audit(json.loads(data), a.case)
    result["report_sha256"] = hashlib.sha256(data).hexdigest()
    result["auditor_sha256"] = hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
    with a.output.open("x", encoding="utf-8") as f:
        json.dump(result, f, indent=2, ensure_ascii=False, allow_nan=False)
    print(json.dumps(result, indent=2, ensure_ascii=False))
    raise SystemExit(0 if result["observed_logic_pass"] else 1)
