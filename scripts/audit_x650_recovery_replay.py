#!/usr/bin/env python3
"""Independently audit paired native recovery replay, without solver/plant I/O.

The reports must refer to the exact same raw fixture and expected library hashes.
An accepted report is checked against every original QP row, actual packet
history, native nonlinear envelope and the original stopping-reserve formula.
This is a fixed recorded-state test, not a new physical closed-loop experiment.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path


OUTPUT_DIMENSIONS = {"command": 5, "position_reference": 3,
    "velocity_feedforward": 3, "acceleration_feedforward": 3,
    "joint_velocity": 2, "joint_reference": 2,
    "joint_velocity_reference": 2, "joint_acceleration_reference": 2,
    "nominal_acceleration": 3, "tip_actual": 3}


def require(condition, message):
    if not condition:
        raise ValueError(message)


def number(value):
    require(not isinstance(value, bool) and isinstance(value, (int, float))
            and math.isfinite(value), "Expected a finite number")
    return float(value)


def vector(value, size=None):
    require(isinstance(value, list) and (size is None or len(value) == size),
            "Numeric vector dimension differs")
    return [number(v) for v in value]


def strict_json(path):
    def pairs(items):
        out = {}
        for k, v in items:
            require(k not in out, "Duplicate JSON key")
            out[k] = v
        return out
    def bad_constant(value):
        raise ValueError("Non-finite JSON constant: " + value)
    return json.loads(Path(path).read_text(encoding="utf-8"),
                      object_pairs_hook=pairs, parse_constant=bad_constant)


def identity(path):
    path = Path(path)
    raw = path.read_bytes()
    return {"path": str(path.resolve()), "bytes": len(raw),
            "sha256": hashlib.sha256(raw).hexdigest()}


def bool_field(value, name):
    require(isinstance(value.get(name), bool), "Missing boolean: " + name)
    return value[name]


def reference_now(fixture):
    """Integrate original sent command history, never reset reference to q."""
    ctx, t = fixture["input"]["context"], number(fixture["input"]["t"])
    at = number(ctx["reference_time"])
    result = vector(ctx["reference"], 2)
    history = ctx["commands"]
    require(isinstance(history, list) and history and at <= t + 1e-10,
            "Actual command/reference coverage differs")
    times = [number(c["time"]) for c in history]
    require(times[0] <= at + 1e-10 and all(a < b for a, b in zip(times, times[1:]))
            and all(x <= t + 1e-10 for x in times), "Actual command times differ")
    active = vector(history[0]["velocity"], 2)
    for row in history:
        when, command = number(row["time"]), vector(row["velocity"], 2)
        if when <= at + 1e-10:
            active = command
            continue
        if when > t:
            break
        result = [a + (when - at) * u for a, u in zip(result, active)]
        at, active = when, command
    return [a + (t - at) * u for a, u in zip(result, active)]


def node_times(fixture):
    c, inp = fixture["config"], fixture["input"]
    times, at = [], number(inp["t"])
    for i in range(c["nodes"]):
        at += number(inp["dt"] if i == 0 else c["node_dt"])
        times.append(at)
    return times


def node_references(fixture, plan):
    c, inp = fixture["config"], fixture["input"]
    commands = vector(plan, 5 * c["nodes"])
    out, r = [], reference_now(fixture)
    for i in range(c["nodes"]):
        step = number(inp["dt"] if i == 0 else c["node_dt"])
        r = [r[j] + step * commands[5 * i + 3 + j] for j in range(2)]
        out += r
    return out


def stopping_violation(fixture, states):
    """Original nonlinear q+T*dq node constraint, not linearized q/dq."""
    c, s = fixture["config"], fixture["input"]["state"]
    values = vector(states, 4 * c["nodes"])
    age = number(s["control_time"]) - number(s["joint_stamp"])
    require(0 <= age <= .025 + 1e-10, "Actual joint state age differs")
    reserve = [number(c["velocity_limit"][3+j]) / (2 * number(c["acceleration_limit"][3+j]))
               + number(c["max_dt"]) + .05 + age for j in range(2)]
    violation = 0.
    for i in range(c["nodes"]):
        for j in range(2):
            value = values[4*i+j] + reserve[j] * values[4*i+2+j]
            violation = max(violation, number(c["q_lower"][j]) - value,
                            value - number(c["q_upper"][j]))
    return violation


def check_plan(fixture, plan, references):
    c, inp = fixture["config"], fixture["input"]
    values = vector(plan, 5 * c["nodes"])
    previous = vector(fixture["memory"]["previous_command"], 5)
    tolerance, violation = number(c["feasibility_tolerance"]), 0.
    for i in range(c["nodes"]):
        h = number(inp["dt"] if i == 0 else c["node_dt"])
        for j in range(5):
            value = values[5*i+j]
            violation = max(violation, abs(value)-number(c["velocity_limit"][j]),
                            abs(value-previous[j])-number(c["acceleration_limit"][j])*h)
        previous = values[5*i:5*i+5]
    expected = node_references(fixture, values)
    actual = vector(references, 2 * c["nodes"])
    require(max(abs(a-b) for a, b in zip(expected, actual)) <= 1e-9,
            "Candidate reference was reset or actual history was changed")
    for i in range(c["nodes"]):
        for j in range(2):
            violation = max(violation, number(c["q_lower"][j])-actual[2*i+j],
                            actual[2*i+j]-number(c["q_upper"][j]))
    require(violation <= tolerance, "Candidate plan violates original speed/slew/reference constraints")
    return violation


def check_envelope(fixture, envelope):
    c, values = fixture["config"], vector(envelope, 8)
    violation = 0.
    for j in range(2):
        require(values[j] <= values[j+2] and values[j+4] <= values[j+6],
                "Nonlinear envelope extrema order differs")
        violation = max(violation, number(c["q_lower"][j]) - (values[j]-.0003),
                        values[j+2]+.0003-number(c["q_upper"][j]),
                        -number(c["velocity_limit"][j+3])-(values[j+4]-.03),
                        values[j+6]+.03-number(c["velocity_limit"][j+3]))
    require(violation <= number(c["feasibility_tolerance"]),
            "Actual nonlinear envelope violates original bounds/margins")
    return violation


def check_qp(d, fixture):
    n, m = d["qp_columns"], d["qp_rows"]
    require(n == 5 * fixture["config"]["nodes"] and isinstance(m, int) and m > 0,
            "Final QP dimensions differ")
    C, lo, hi, z = vector(d["C"], m*n), vector(d["lower"], m), vector(d["upper"], m), vector(d["solution"], n)
    violation = max(0., max(abs(x)-1 for x in z))
    for i in range(m):
        value = math.fsum(a*b for a, b in zip(C[i*n:(i+1)*n], z))
        require(lo[i] <= hi[i], "Original QP bound ordering differs")
        violation = max(violation, lo[i]-value, value-hi[i])
    require(violation <= number(fixture["config"]["feasibility_tolerance"]),
            "Final solution violates an original assembled QP row")
    plan = vector(d["plan"], n)
    scale = vector(fixture["config"]["velocity_limit"], 5)
    require(max(abs(plan[i]-z[i]*scale[i % 5]) for i in range(n)) <= 1e-10,
            "Final physical plan is not the original normalized QP solution")
    # Original assembly appends 2*N trust rows followed by five first-command
    # rows. Verify exact row coordinates and width; do not trust a radius flag.
    for i in range(fixture["config"]["nodes"]):
        for j in range(2):
            row = m - 5 - 2*fixture["config"]["nodes"] + 2*i+j
            require(row >= 0 and all(C[row*n+k] == (1. if k == 5*i+3+j else 0.) for k in range(n))
                    and abs(hi[row]-lo[row]-.04) <= 1e-12,
                    "Final QP changed original normalized 0.02 trust radius/rows")
    return violation


def check_packets(fixture, packets, plan):
    actual = fixture["input"]["context"]["packets"]
    require(isinstance(packets, list) and len(packets) >= len(actual), "Candidate packets are missing")
    require(packets[:len(actual)] == actual, "Candidate changed an actual/in-flight MIT packet")
    times = [number(p["send_time"]) for p in packets]
    require(all(a < b for a, b in zip(times, times[1:])), "Candidate packet times are unordered")
    n, at = fixture["config"]["nodes"], number(fixture["input"]["t"])
    starts, ends, elapsed = [], [], 0.
    for i in range(n):
        starts.append(elapsed)
        elapsed += number(fixture["input"]["dt"] if i == 0 else fixture["config"]["node_dt"])
        ends.append(elapsed)
    period = number(fixture["config"]["bus_period"])
    require(period > 0 and len(packets) > len(actual)
            and times[-1] > at + elapsed - period - 1e-9,
            "Candidate packets do not cover the original horizon")
    require(all(abs((t-times[0])/period-round((t-times[0])/period)) <= 1e-7 for t in times),
            "Candidate packet bus grid differs")
    base = reference_now(fixture)
    def quantize(value, step):
        return math.copysign(math.floor(abs(value/step)+.5), value)*step
    for p in packets[len(actual):]:
        require(number(p["send_time"]) > number(fixture["input"]["t"])-1e-10,
                "Candidate fabricated a historical MIT packet")
        require(p["kp"] == fixture["config"]["kp"] and p["kd"] == fixture["config"]["kd"],
                "Candidate changed actuator gains")
        relative = number(p["send_time"])-at
        active = max(0, min(n-1, sum(start <= relative+1e-10 for start in starts)-1))
        for j in range(2):
            integrated = math.fsum((ends[i]-starts[i])*plan[5*i+3+j] for i in range(active))
            q_ref = base[j]+integrated+(relative-starts[active])*plan[5*active+3+j]
            require(abs(number(p["q_ref"][j])-quantize(q_ref, 2*math.pi/(2**24))) <= 1e-12
                    and abs(number(p["dq_ref"][j])-quantize(plan[5*active+3+j], 1/512)) <= 1e-12,
                    "Future packet is not integrated from actual reference/history and final plan")
    return True


def nonlinear_nodes(fixture, diagnostic):
    times = vector(diagnostic["query_times"])
    values = vector(diagnostic["candidate_states"], 4*len(times))
    require(times and all(a < b for a, b in zip(times, times[1:])), "Actual nonlinear query ordering differs")
    result = []
    for wanted in node_times(fixture):
        matches = [i for i, t in enumerate(times) if abs(t-wanted) <= 1e-9]
        require(len(matches) == 1, "Nonlinear prediction does not cover an original horizon node")
        result += values[4*matches[0]:4*matches[0]+4]
    return result


def check_recovery(fixture, d):
    require(bool_field(d, "recovery_diagnostics_available"), "Recovery diagnostics unavailable")
    require(bool_field(d, "recovery_triggered"), "Expected recovery was not triggered")
    require(number(d["recovery_trust_radius"]) == .02, "Recovery enlarged the trust radius")
    attempts = d["recovery_attempts"]
    require(isinstance(attempts, list) and 1 <= len(attempts) <= 2, "Recovery centre budget differs")
    checked = []
    for a in attempts:
        trusted = bool_field(a, "trusted")
        require(number(a["elapsed_ms"]) <= 50., "Recovery centre exceeded original total deadline")
        if not trusted:
            continue
        require(bool_field(a, "nonlinear_passed") and bool_field(a, "original_constraints_passed"),
                "Centre trusted without original/nonlinear validation")
        expected = node_times(fixture)
        times = vector(a["node_times"], len(expected))
        require(max(abs(x-y) for x, y in zip(times, expected)) <= 1e-9, "Recovery centre node times differ")
        slew = check_plan(fixture, a["centre_plan"], a["node_references"])
        envelope = check_envelope(fixture, a["envelope"])
        reserve = stopping_violation(fixture, a["node_states"])
        require(reserve <= number(fixture["config"]["feasibility_tolerance"]),
                "Trusted centre violates original nonlinear stopping reserve")
        require(number(a["original_primal_violation"]) <= number(fixture["config"]["feasibility_tolerance"])
                and abs(number(a["stopping_violation"])-reserve) <= 1e-12,
                "Trusted centre original/stopping residual differs")
        checked.append({"centre_kind": a["centre_kind"], "plan_violation": slew,
                        "envelope_violation": envelope, "nonlinear_stopping_violation": reserve})
    require(checked, "No independently checked trusted centre")
    projections = sum(bool_field(a, "projection_attempted") for a in attempts)
    require(projections <= 1 and d["subproblems"]+projections <= 4,
            "Recovery exceeded total control/projection QP call budget")
    return checked


def check_report(fixture, result, input_sha, expected_libraries, *, require_nonlinear_stopping):
    require(result.get("kind") == "x650_cpp_recovery_raw_replay_v1" and result.get("schema_version") == 1
            and result.get("complete") is True, "Native replay schema/completeness differs")
    require(result.get("input_sha256") == input_sha, "Native replay is not bound to this raw fixture")
    for key in ("core", "optimization", "predictor"):
        require(result.get("loaded_code", {}).get(key, {}).get("sha256") == expected_libraries[key],
                "Loaded library differs: " + key)
    scope = result.get("scope", {})
    require(scope.get("offline_fixed_snapshot") is True and scope.get("actuator_io") is False
            and scope.get("closed_loop_or_physical_validation") is False
            and scope.get("injected_delay") is False and scope.get("original_core_deadline_ms") == 50.
            and scope.get("initial_trust_radius") == .02, "Replay scope/policy differs")
    require(result["memory_before"] == fixture["memory"], "Restored memory differs from original raw fixture")
    returned, error, unexpected = (bool_field(result, k) for k in ("returned", "core_error", "unexpected_exception"))
    require(sum((returned, error, unexpected)) == 1 and not unexpected, "Unexpected native return/error classification")
    before, after = result["memory_before"], result["memory_after"]
    same = before == after
    require(bool_field(result, "memory_unchanged") is same, "Reported memory rollback differs from complete values")
    if not returned:
        require(same and result.get("output") is None and result.get("memory_commit_valid") is False,
                "Rejected/timeout call committed memory or invented a command")
        return {"returned": False, "memory_rollback_verified": True, "error": result["error"]}
    require(result.get("error") is None, "Returned replay reports error")
    out = result["output"]
    for key, size in OUTPUT_DIMENSIONS.items():
        vector(out[key], size)
    require(after["step_count"] == before["step_count"]+1 and after["last_t"] == fixture["input"]["t"]
            and after["previous_command"] == out["command"] and result.get("memory_commit_valid") is True,
            "Accepted replay did not commit exactly the validated command")
    require(after["anchor"] == before["anchor"], "Accepted step changed original anchor")
    d = result["diagnostics"]
    require(after["plan"] == d["plan"] and after["previous_stamp"] == fixture["input"]["state"]["stamp"],
            "Committed plan or actual state clock differs from the accepted call")
    require(all(bool_field(d, k) for k in ("accepted", "nonlinear_passed", "actual_bounds"))
            and bool_field(d, "previous_plan_fallback") is False, "Accepted replay lacks original candidate checks")
    require(number(result["core_step_wall_ms"]) <= 50. and number(d["compute_ms"]) <= 50.,
            "Accepted full Core call exceeded original 50 ms")
    require(isinstance(d["subproblems"], int) and 1 <= d["subproblems"] <= 3,
            "Control QP attempt budget differs")
    qp = check_qp(d, fixture)
    check_packets(fixture, d["candidate_packets"], d["plan"])
    envelope = check_envelope(fixture, d["candidate_envelope"])
    slew = check_plan(fixture, d["plan"], node_references(fixture, d["plan"]))
    actual_nodes = nonlinear_nodes(fixture, d)
    # Historical predicted_states has q/dq replaced with native actual values
    # after acceptance. It cannot be used to independently recompute the prior
    # linearization discrepancy. Check the original recorded scalar gate, and
    # only claim recomputation if a separate pre-replacement capture exists.
    require(0 <= number(d["q_linearization_error"]) <= .006
            and 0 <= number(d["dq_linearization_error"]) <= .10,
            "Recorded nonlinear/linear prediction inconsistency")
    linearization_recomputed = "linearized_states" in d
    if linearization_recomputed:
        predicted = vector(d["linearized_states"], 12*fixture["config"]["nodes"])
        q_error = max(abs(actual_nodes[4*i+j]-predicted[12*i+6+j])
                      for i in range(fixture["config"]["nodes"]) for j in range(2))
        dq_error = max(abs(actual_nodes[4*i+2+j]-predicted[12*i+8+j])
                       for i in range(fixture["config"]["nodes"]) for j in range(2))
        require(q_error <= .006 and dq_error <= .10
                and abs(q_error-d["q_linearization_error"]) <= 1e-10
                and abs(dq_error-d["dq_linearization_error"]) <= 1e-10,
                "Independent nonlinear/linear discrepancy differs from native evidence")
    reserve = stopping_violation(fixture, actual_nodes)
    if require_nonlinear_stopping:
        require(reserve <= number(fixture["config"]["feasibility_tolerance"]),
                "Accepted nonlinear candidate violates original stopping reserve")
    return {"returned": True, "original_qp_violation": qp, "nonlinear_envelope_violation": envelope,
            "plan_slew_reference_violation": slew, "nonlinear_stopping_violation": reserve,
            "nonlinear_stopping_required": require_nonlinear_stopping,
            "linearization_discrepancy_independently_recomputed": linearization_recomputed,
            "core_step_wall_ms": result["core_step_wall_ms"], "memory_commit_verified": True}


def audit(fixture, old, new, input_sha, old_libraries, new_libraries, expectation):
    require(expectation in ("unchanged", "recovered", "rejected"), "Unknown comparison expectation")
    # Historical old Core did not test nonlinear stopping reserve. Preserve its
    # policy for unchanged calls and expose the recomputed value explicitly.
    old_check = check_report(fixture, old, input_sha, old_libraries, require_nonlinear_stopping=False)
    new_check = check_report(fixture, new, input_sha, new_libraries,
                            require_nonlinear_stopping=expectation == "recovered")
    result = {"kind": "x650_cpp_recovery_independent_replay_audit_v1", "complete": False,
              "expectation": expectation, "input_sha256": input_sha,
              "old": old_check, "new": new_check, "physical_validation_claimed": False,
              "new_closed_loop_trajectory_claimed": False}
    if expectation == "rejected":
        require(not new_check["returned"], "Expected rejection returned a candidate")
    elif expectation == "unchanged":
        require(old_check["returned"] and new_check["returned"], "Expected successful unchanged calls")
        require(new["diagnostics"].get("recovery_triggered") is False, "Normal call unexpectedly used recovery")
        require(new_check["linearization_discrepancy_independently_recomputed"],
                "New Full Core lacks original linearization capture")
        errors = {key: max(abs(a-b) for a, b in zip(vector(old["output"][key]), vector(new["output"][key])))
                  for key in OUTPUT_DIMENSIONS}
        require(all(value <= (1e-9 if key == "position_reference" else 1e-8)
                    for key, value in errors.items()), "Normal result changed outside original absolute tolerance")
        require(old["memory_after"] == new["memory_after"], "Normal next memory changed")
        result["maximum_absolute_errors"] = errors
    else:
        require(not old_check["returned"] and old.get("error") == "QP infeasible",
                "Original QP infeasibility did not reproduce")
        require(new_check["returned"], "New Core did not recover this raw failure")
        require(new_check["linearization_discrepancy_independently_recomputed"],
                "Recovered Full Core lacks original linearization capture")
        result["trusted_centres"] = check_recovery(fixture, new["diagnostics"])
    result.update(complete=True, passed=True)
    return result


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ("input", "old", "new", "output"):
        parser.add_argument("--"+name, type=Path, required=True)
    parser.add_argument("--expectation", choices=("unchanged", "recovered", "rejected"), required=True)
    for version in ("old", "new"):
        for library in ("core", "optimization", "predictor"):
            parser.add_argument(f"--{version}-{library}-sha256", required=True)
    args = parser.parse_args(argv)
    require(not args.output.exists(), "Audit evidence must not be overwritten")
    receipts = {name: identity(getattr(args, name)) for name in ("input", "old", "new")}
    expected = {version: {library: getattr(args, f"{version}_{library}_sha256")
                         for library in ("core", "optimization", "predictor")}
                for version in ("old", "new")}
    try:
        result = audit(strict_json(args.input), strict_json(args.old), strict_json(args.new),
                       receipts["input"]["sha256"], expected["old"], expected["new"], args.expectation)
    except (ValueError, KeyError, TypeError) as error:
        result = {"kind": "x650_cpp_recovery_independent_replay_audit_v1", "complete": True,
                  "passed": False, "error": str(error), "expectation": args.expectation}
    result["artifacts"] = receipts
    result["auditor"] = identity(__file__)
    args.output.write_text(json.dumps(result, indent=2, allow_nan=False)+"\n", encoding="utf-8")
    return 0 if result["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
