"""Bind existing 24-row multirate inputs and their original C++ sequential oracle.

Pure offline format adapter; source measurements, real packet context, and
saved oracle Memory are preserved. No simulation or algorithm is executed.
"""
from __future__ import annotations
import argparse
import hashlib
import json
import math
from pathlib import Path

OUTPUT_FIELDS = ("command", "position_reference", "velocity_feedforward",
                 "acceleration_feedforward", "joint_velocity", "joint_reference",
                 "joint_velocity_reference", "joint_acceleration_reference",
                 "nominal_acceleration", "tip_actual")

def require(condition, message):
    if not condition:
        raise ValueError(message)

def frozen_target(t, center, kind):
    """Arithmetic port of the existing TrajectoryPlan, for offline preview only.

    Kept here so the public package does not require Orin benchmarking scripts.
    The current recorded target is checked before this preview is accepted.
    """
    require(kind in ("fast", "hold"), "Unsupported recording trajectory")
    if kind == "hold":
        return {"position": list(center), "velocity": [0.]*3, "acceleration": [0.]*3}
    period, ax, az, start, ramp, cycles = 8., .25, .10, 2., 2., 2
    w, total, duration = 2*math.pi/period, 2*math.pi*cycles, cycles*period+ramp
    s = t-start
    phi = speed = acc = 0.
    if s >= duration:
        phi = total
    elif s > 0 and (s < ramp or s > duration-ramp):
        ending = s > duration-ramp
        u = (duration-s if ending else s)/ramp
        a = 2.5*math.pow(u, 4)-3*math.pow(u, 5)+math.pow(u, 6)
        b = 10*math.pow(u, 3)-15*math.pow(u, 4)+6*math.pow(u, 5)
        c = 30*u*u*(1-u)*(1-u)
        phi = total-w*ramp*a if ending else w*ramp*a
        speed, acc = w*b, (-1 if ending else 1)*w*c/ramp
    elif s > 0:
        phi, speed = w*(s-.5*ramp), w
    d = [ax*math.cos(phi), 0., 2*az*math.cos(2*phi)]
    dd = [-ax*math.sin(phi), 0., -4*az*math.sin(2*phi)]
    p = list(center)
    p[0] += ax*math.sin(phi)
    p[2] += az*math.sin(2*phi)
    return {"position": p, "velocity": [v*speed for v in d],
            "acceleration": [dd[j]*speed*speed+d[j]*acc for j in range(3)]}

def raw_input(s, config, center, kind):
    n, p = s["command_count"], s["packet_count"]
    require(1 <= n <= 64 and 1 <= p <= 64, "Original TS20 context count outside bound")
    for key, count, stride in [("command_time_s", n, 1), ("command_velocity", n, 2),
        ("packet_send_time_s", p, 1), *[("packet_"+k, p, 2) for k in ("q_ref", "dq_ref", "tau_ff", "kp", "kd")]]:
        require(not any(s[key][count*stride:]), "Nonzero unused original context: "+key)
    context = {"reference_time": s["reference_time_s"], "bus_origin": s["bus_origin_s"],
        "reference": s["reference_r"], "commands": [
            {"time": s["command_time_s"][i], "velocity": s["command_velocity"][2*i:2*i+2]}
            for i in range(n)], "packets": [
            {"send_time": s["packet_send_time_s"][i], **{
                k: s["packet_"+k][2*i:2*i+2] for k in ("q_ref", "dq_ref", "tau_ff", "kp", "kd")}}
            for i in range(p)]}
    state = {k: s[v] for k, v in {"position": "position_enu", "velocity": "velocity_enu",
        "omega_body": "omega_body_flu", "acceleration": "acceleration_enu", "rotation": "rotation_row_major",
        "q": "q", "dq": "dq", "stamp": "state_time_s", "control_time": "control_time_s",
        "joint_stamp": "joint_time_s"}.items()}
    target = {k: s["target_"+k+"_enu"] for k in ("position", "velocity", "acceleration")}
    expected = frozen_target(s["task_time_s"], center, kind)
    require(max(abs(a-b) for k in target for a,b in zip(target[k], expected[k])) <= 1e-13,
            "Recorded target differs from frozen C++ trajectory")
    preview, offset = [], 0.
    for i in range(config["nodes"]):
        offset += s["dt_s"] if i == 0 else config["node_dt"]
        preview.append(frozen_target(s["task_time_s"]+offset, center, kind))
    return {"t": s["task_time_s"], "dt": s["dt_s"], "state": state,
            "target": target, "context": context, "preview": preview}

def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()

def bound(row):
    p = Path(row["path"])
    if p.stat().st_size != row["bytes"] or digest(p) != row["sha256"]:
        raise ValueError(f"Original binding mismatch: {p}")
    return json.loads(p.read_text())

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--recording", type=Path, required=True)
    parser.add_argument("--oracle", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        raise FileExistsError(args.output)
    recording = json.loads(args.recording.read_text())
    oracle = json.loads(args.oracle.read_text())
    if (recording["kind"] != "x650_cpp_multirate_derived_recording_v1"
        or oracle["kind"] != "x650_cpp_multirate_sequence_oracle_v1"
        or not oracle["complete"] or not oracle["passed"]
        or oracle["recording"]["sha256"] != digest(args.recording)
        or len(recording["rows"]) != len(oracle["rows"])):
        raise ValueError("Recording and accepted original sequential oracle differ")
    trajectory = recording["trajectory"]
    require(trajectory["kind"] in ("fast_figure8", "hold"), "Unsupported trajectory")
    kind = "fast" if trajectory["kind"] == "fast_figure8" else "hold"
    if kind == "fast":
        expected = {"amplitude_x": .25, "amplitude_z": .1, "period_s": 8.,
                    "cycles": 2, "start_s": 2., "ramp_s": 2., "duration_s": 24.}
        require(all(trajectory.get(k) == v for k, v in expected.items()),
                "Recording trajectory differs from fixed preview parameters")
    cases = []
    for source, expected in zip(recording["rows"], oracle["rows"]):
        row, old = bound(source), bound(expected)
        if (source["sequence"] != expected["sequence"]
            or row["sequence"] != old["sequence"] or not old["returned"]):
            raise ValueError("Original row sequence/return mismatch")
        value = raw_input(row["snapshot"], recording["config"],
                          recording["live_start"]["target_position_enu"], kind)
        cases.append({"id": f"recorded_{row['sequence']:06d}",
                      "memory": old["memory_before"], "input": value,
                      "oracle": {key: old["output"][key] for key in OUTPUT_FIELDS}})
    dataset = {"schema_version": 1, "kind": "x650_mpc_memory_benchmark_input_v1",
               "config": recording["config"], "cases": cases,
               "run": {"warmup_rounds": 0, "measured_rounds": 1, "cpu": -1,
                       "oracle_absolute_tolerance": 1e-7},
               "provenance": {"recording_sha256": digest(args.recording),
                              "oracle_report_sha256": digest(args.oracle),
                              "recording": str(args.recording), "oracle": str(args.oracle),
                              "actual_measurements_and_packet_history_preserved": True,
                              "memory_source": "Original sequential C++ oracle memory_before",
                              "physical_validation": False}}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(dataset, indent=2, allow_nan=False) + "\n")
    print(json.dumps({"output": str(args.output), "sha256": digest(args.output),
                      "cases": len(cases)}))

if __name__ == "__main__":
    main()
