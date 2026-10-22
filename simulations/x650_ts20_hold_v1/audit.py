#!/usr/bin/env python3
"""Independent postflight certification for the NEW conditional TS20 plant.

Common native/ULog/parameter checks are reused as functions.  The resulting
certificate has a new kind and actuator identity; no old certificate is promoted.
"""
from __future__ import annotations
import argparse
import csv
import json
from pathlib import Path
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
for directory in (ROOT, ROOT / "scripts", ROOT / "simulations/x650_wholebody_mpc_v1",
                  ROOT / "control_packages/x650_wholebody_v1"):
    if str(directory) not in sys.path:
        sys.path.insert(0, str(directory))
from simulations.x650_ts20_hold_v1 import task
import audit_x650_precision_stage as fixed
import audit_x650_upper_stage as upper
import analyze_x650_hover_precision as precision
from research_condition import condition_for_task, frozen_research_pid, canonical_hash
from simulations.x650_wholebody_mpc_v1 import run_case as shared
from x650_wholebody.ts20 import FrozenTs20Model
from x650_wholebody.ts20_prediction import Ts20PredictionModel


def rows(path):
    with Path(path).open(newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def fields(values, names):
    result = np.array([[float(row[key]) for key in names] for row in values])
    fixed.require(np.isfinite(result).all(), "Nonfinite recorded TS20/native values")
    return result


def free_forecast_in_blocks(predictor, times, initial_q, initial_dq, packets):
    """Respect the predictor's two-second budget without any measured reset."""
    t = np.asarray(times, float)
    if t.ndim != 1 or len(t) < 2 or not np.isfinite(t).all() or np.any(np.diff(t) <= 0):
        raise ValueError("Finite increasing complete prediction times required")
    q, v = np.empty((len(t), 2)), np.empty((len(t), 2))
    q[0], v[0] = initial_q, initial_dq
    cursor = 0
    while cursor < len(t) - 1:
        end = min(len(t) - 1, int(np.searchsorted(t, t[cursor] + 1.9, side="right") - 1))
        if end <= cursor:
            raise ValueError("Unable to construct a bounded continuous free-prediction block")
        forecast = predictor.rollout(float(t[cursor]), q[cursor].copy(), v[cursor].copy(), packets, t[cursor:end + 1])
        fixed.require(np.array_equal(np.asarray(forecast["time_s"]), t[cursor:end + 1]), "Predictor changed original query times")
        q[cursor:end + 1], v[cursor:end + 1] = forecast["q"], forecast["dq"]
        cursor = end
    fixed.require(np.isfinite([q, v]).all(), "Nonfinite complete free prediction")
    return q, v


def packet_state_binding(states, packets, *, bus_period_s=.005):
    """Require full packet evidence for every state declaration, including tail."""
    mapping = {}
    for packet in packets:
        sequence = int(float(packet["sequence"]))
        fixed.require(sequence not in mapping, "Duplicate actual MIT packet sequence")
        mapping[sequence] = packet
    for state in states:
        at = float(state["sim_s"])
        for prefix in ("last_send", "active"):
            sequence = int(float(state[prefix + "_sequence"]))
            fixed.require(sequence in mapping, "TS20 state declares a packet absent from actual MIT log")
            packet = mapping[sequence]
            for skey, pkey in ((prefix + "_s" if prefix == "last_send" else "active_send_s", "send_s"),
                              (prefix + "_apply_s", "apply_s"),
                              *((prefix + "_qref" + str(j) if prefix != "last_send" else "last_qref" + str(j), "qref" + str(j)) for j in (1, 2)),
                              *((prefix + "_dqref" + str(j) if prefix != "last_send" else "last_dqref" + str(j), "dqref" + str(j)) for j in (1, 2))):
                fixed.require(abs(float(state[skey]) - float(packet[pkey])) <= 1e-9,
                              "TS20 state/packet stream field mismatch: " + skey)
            send, apply = float(packet["send_s"]), float(packet["apply_s"])
            if prefix == "last_send":
                fixed.require(-1e-9 <= at - send <= bus_period_s + 1e-9,
                              "TS20 last-send packet stream does not cover full state window")
            else:
                fixed.require(apply <= at + 1e-9 and at - apply <= bus_period_s + 1e-9,
                              "TS20 active delayed packet does not cover full state window")
    return {"verified": True, "state_rows": len(states), "packet_rows": len(packets),
            "last_and_active_sequence_fields_verified": True, "tail_packet_coverage_required": True}


def actuator_check(folder, cfg, window):
    capture = task.read(folder / "ts20_recording_manifest.json")
    fixed.require(capture.get("complete") is True, "Incomplete TS20 capture")
    for name in ("ts20_state.csv", "ts20_packets.csv"):
        fixed.require(capture["captures"][name]["sha256"] == task.sha(folder / name), "TS20 capture bytes changed")
    raw, packet_rows = rows(folder / "ts20_state.csv"), rows(folder / "ts20_packets.csv")
    states = [row for row in raw if window[0] - 1e-9 <= float(row["sim_s"]) <= window[1] + 1e-9]
    binding = packet_state_binding(states, packet_rows)
    t = fields(states, ["sim_s"]).ravel()
    fixed.require(len(t) >= 2 and np.all(np.diff(t) > 0) and np.diff(t).max() <= .008 + 1e-9
                  and t[0] <= window[0] + .008 and t[-1] >= window[1] - .008,
                  "TS20 plugin does not cover the whole measurement window")
    q, qi, vi = (fields(states, [f"{prefix}{j}{suffix}" for j in (1, 2)])
                 for prefix, suffix in (("q", "_physical"), ("q", "_internal"), ("dq", "_internal")))
    protocol = cfg["ts20_protocol"]
    disagreement = float(abs(q - qi).max())
    fixed.require(disagreement <= protocol["internal_physical_q_tolerance_rad"],
                  "Internal servo position does not follow physical joint position")
    native = [row for row in rows(folder / "native_trace.csv")
              if window[0] - 1e-9 <= float(row["sim_s"]) <= window[1] + 1e-9]
    nt = fields(native, ["sim_s"]).ravel()
    nq = fields(native, ["arm_joint_1_q", "arm_joint_2_q"])
    nv = fields(native, ["arm_joint_1_qd", "arm_joint_2_qd"])
    index = np.searchsorted(nt, t)
    index = np.minimum(index, len(nt) - 1)
    previous = np.maximum(index - 1, 0)
    index = np.where(abs(nt[previous] - t) < abs(nt[index] - t), previous, index)
    fixed.require(float(abs(nt[index] - t).max()) <= 1e-7
                  and float(abs(nq[index] - q).max()) <= protocol["internal_physical_q_tolerance_rad"],
                  "TS20 physical position is not same-time independent native q")
    lo, hi, vmax = (np.asarray(protocol[key]) for key in ("q_lower_rad", "q_upper_rad", "joint_velocity_max_rad_s"))
    fixed.require(np.all(q >= lo - 2e-7) and np.all(q <= hi + 2e-7)
                  and np.all(abs(vi) <= vmax + 2e-7) and np.all(abs(nv) <= vmax + 2e-7),
                  "Whole-window physical or endpoint joint limits exceeded")
    motion = FrozenTs20Model(cfg["ts20_inputs"]["source_dir"], task.read(cfg["ts20_inputs"]["source_manifest"]))
    predictor = task.predictor_from_bound_plant(cfg, motion.metadata()["motion_model"])
    packet_times = fields(packet_rows, ["send_s"]).ravel()
    apply_times = fields(packet_rows, ["apply_s"]).ravel()
    fixed.require(len(packet_times) >= 2 and np.all(np.diff(packet_times) > 0)
                  and float(abs(np.diff(packet_times) - .005).max()) <= 1e-9
                  and float(abs(apply_times - packet_times - predictor.delay_s).max()) <= 1e-9,
                  "Actual MIT bus grid or single combined delay differs from prediction")
    # CSV sim_s is the control snapshot before calculation. The actual publish
    # trace has wall-clock boundaries, not an independently sampled physics
    # clock during compute. Therefore report this end-to-end observed lag with
    # its honest anchor; do not label it a separately identified ROS delay.
    controls = rows(folder / "control_trace.csv")
    control_t = fields(controls, ["sim_s"]).ravel()
    command = fields(controls, ["qd1_cmd", "qd2_cmd"])
    lags, seen = [[], []], [set(), set()]
    for row in states:
        for j in range(2):
            sequence = int(float(row[f"upper_sequence{j+1}"]))
            if sequence in seen[j]:
                continue
            seen[j].add(sequence)
            receipt = float(row[f"receipt{j+1}_s"])
            candidates = np.flatnonzero((control_t <= receipt + 1e-9)
                & (control_t >= receipt - .25)
                & (abs(command[:, j] - float(row[f"u{j+1}"])) <= 1e-10))
            if len(candidates):
                lags[j].append(float(receipt - control_t[candidates[-1]]))
    lag_report = [{"joint": j+1, "matched_command_events": len(values),
                   "minimum_s": min(values) if values else None,
                   "median_s": float(np.median(values)) if values else None,
                   "p95_s": float(np.percentile(values, 95)) if values else None,
                   "maximum_s": max(values) if values else None} for j, values in enumerate(lags)]
    packets = []
    for row in packet_rows:
        packet = {"send_time_s": float(row["send_s"])}
        for name, short in (("q_ref", "qref"), ("dq_ref", "dqref"), ("tau_ff", "tau"), ("kp", "kp"), ("kd", "kd")):
            packet[name] = [float(row[f"{short}{j}"]) for j in (1, 2)]
        packets.append(packet)
    # One free prediction from the first measured physical q and simulated
    # endpoint-speed report. Subsequent recorded q/dq are evaluation only.
    forecast_q, forecast_v = free_forecast_in_blocks(predictor, t, q[0], vi[0], packets)
    qerr, verr = forecast_q - q, forecast_v - vi
    maxq, maxv = float(abs(qerr).max()), float(abs(verr).max())
    prediction_pass = (maxq <= protocol["new_joint_prediction_q_peak_max_rad"]
                       and maxv <= protocol["new_joint_prediction_dq_peak_max_rad_s"])
    horizons = {}
    for horizon in (.02, .1, .3, .6):
        ix = int(np.argmin(abs(t - t[0] - horizon)))
        fixed.require(abs(t[ix] - t[0] - horizon) <= .004 + 1e-9, "Missing first-state prediction horizon")
        horizons[str(round(horizon * 1000)) + "ms"] = {"actual_horizon_s": float(t[ix] - t[0]),
            "q_abs_error_rad": abs(qerr[ix]).tolist(), "dq_abs_error_rad_s": abs(verr[ix]).tolist()}
    return {"verified": True, "samples": len(t), "internal_physical_q_peak_rad": disagreement,
            "state_packet_binding": binding,
            "native_plugin_q_peak_rad": float(abs(nq[index] - q).max()),
            "bus_period_s": .005, "single_combined_delay_s": predictor.delay_s,
            "control_snapshot_to_plugin_consumption_lag": {"joints": lag_report,
                "matching": "latest preceding equal-valued actual command within 250ms; repeated identical commands can be ambiguous",
                "anchor": "control snapshot sim_s before calculation, not an independent wire publication timestamp",
                "upper_computation_and_ros_transport_delay_compensated": False},
            "new_joint_prediction_verified": bool(prediction_pass),
            "prediction": {"first_state_only": True, "later_state_resets": False,
                           "q_peak_abs_rad": maxq, "dq_peak_abs_rad_s": maxv,
                           "first_state_horizons": horizons, "complete_window_s": float(t[-1] - t[0])}}


def audit(folder, launch, output):
    folder, launch, output = Path(folder).resolve(), Path(launch).resolve(), Path(output).resolve()
    if output.exists():
        raise ValueError("Use a new audit directory")
    output.mkdir(parents=True)
    meta, stage, cfg = (task.read(folder / name) for name in ("controller_metadata.json", "stage_result.json", "controller_task.json"))
    cfg = task.validate_task_config(cfg)
    fixed.require(meta.get("task") == cfg and stage.get("kind") == "x650_ts20_conditional_run_result_v1",
                  "A new native TS20 run is required")
    fixed.require(all(meta.get(key) is True for key in ("complete", "landed_and_disarmed", "diagnostic_parameters_restored"))
                  and meta.get("diagnostic_restore_pending") is False
                  and stage.get("complete") is True and stage.get("passed") is True
                  and stage.get("eligible_as_stage_gate") is False, "Failed or un-restored run cannot be certified")
    identity = task.condition(cfg, model_sha256=meta["model_sha256"], source_urdf_sha256=meta["source_urdf_sha256"])
    fixed.require(meta.get("ts20_operating_condition") == identity == stage.get("ts20_operating_condition"),
                  "TS20 condition/source identity mismatch")
    artifacts = fixed.source_and_launch_evidence(launch, folder, meta, cfg)
    artifacts.update(fixed.verified_artifacts(folder, stage))
    condition = condition_for_task(cfg)
    _, pid, _ = frozen_research_pid(cfg)
    documents = {key: task.read(folder / name) for key, name in {
        "session": "diagnostic_parameter_session.json", "journal": "diagnostic_restore_journal_completed.json",
        "before": "diagnostic_parameters_before.json", "applied": "diagnostic_parameters_applied.json",
        "restored": "diagnostic_parameters_restored.json", "px4_before": "px4_parameters_before.json",
        "px4_after": "px4_parameters_after.json"}.items()}
    parameters = fixed.verify_parameter_session(documents, meta, condition, pid)
    reference = task.read(folder / "hover_reference.json")
    target = precision.verify_reference(reference, precision.DEFAULT_REFERENCE_SOURCE)
    window = [float(meta[key]) for key in ("task_start_sim_s", "task_end_sim_s")]
    fixed.require(abs(window[1] - window[0] - cfg["duration_s"]) <= .05,
                  "Complete declared measurement interval is required")
    manifest = task.read(folder / "native_trace_manifest.json")
    fixed.require(manifest["sha256"] == task.sha(folder / "native_trace.csv") and manifest["complete"] is True,
                  "Native trace completion/hash mismatch")
    precision.capture_window_covers(manifest, window)
    native = precision.native_states(folder / "native_trace.csv", window)
    from pyulog import ULog
    ulog = ULog(str(folder / "px4.ulg"))
    configuration = fixed.effective_configuration(ulog, native, reference, window, fixed.records(documents["applied"]))
    controls = rows(folder / "control_trace.csv")
    selected = [row for row in controls if row["phase"] == "measure" and window[0] - 1e-9 <= float(row["sim_s"]) <= window[1] + 1e-9]
    ct = fields(selected, ["sim_s"]).ravel()
    fixed.require(len(ct) >= 2 and ct[0] <= window[0] + .05 and ct[-1] >= window[1] - .05
                  and np.all(np.diff(ct) > 0) and np.diff(ct).max() <= .05 + 1e-9,
                  "Actual send intervals exceed frozen 50 ms or omit part of window")
    compute = fields(selected, ["compute_ms"]).ravel()
    fixed.require(float(compute.max()) <= 50., "Control computation exceeds frozen 50 ms")
    ground = fixed.native_ground_check(folder, task.read(folder / "native_ground.json"))
    actuator = actuator_check(folder, cfg, window)
    if cfg["stage"] == "tip_hold":
        commands = upper.upper_commands(ulog, native, reference, controls, window, pid, response=False)
        fixed.require(commands["zero_acceleration_feedforward"] is True, "TS20 hold uses zero acceleration FF")
        flight = upper.flight_states(folder / "flight_samples.csv", window)
        center = shared.tip_world(np.asarray(meta["reference_base_enu_m"]), np.asarray(meta["reference_rotation"]), np.zeros(2),
                                shared.ArmModel.from_report(cfg["preparation_report"]))
        endpoint = shared.native_tip_metrics(folder, shared.ConstantTarget(center).sample, window[0], (0., cfg["duration_s"]))
        p = cfg["ts20_protocol"]
        fixed.require(endpoint["position_rmse_m"] <= p["tip_rms_max_m"] and endpoint["position_max_m"] <= p["tip_peak_max_m"]
                      and float(native["tilt"].max()) <= p["tilt_max_deg"], "TS20 endpoint-hold physical envelope failed")
        diag = [json.loads(line) for line in (folder / "controller_diagnostics.jsonl").read_text().splitlines() if line.strip()]
        diag = [row["diagnostics"] for row in diag if row.get("phase") == "measure"]
        fixed.require(len(diag) == len(selected) and all(row.get("accepted_optimum") is True
                      and row.get("previous_plan_fallback", False) is False and row.get("ts20_nonlinear_check_passed") is True
                      and row.get("solve_budget_exceeded", False) is False for row in diag), "MPC solution/candidate acceptance missing")
        result = {"endpoint": endpoint, "solver_solves": len(diag)}
    else:
        commands = fixed.fixed_position_commands(ulog, native, reference, controls, window)
        flight = fixed.flight_sample_check(folder / "flight_samples.csv", window, target)
        if cfg["stage"] == "joint_probe":
            result = fixed.joint_stage_check(folder, meta, window, target, controls)
            fixed.require(result["passed"] and actuator["new_joint_prediction_verified"], "New TS20 joint probe/prediction failed")
        else:
            position = precision.metrics(native["t"], native["position"] - target, "m")
            fixed.require(position["rmse_3d"] <= .02 and position["peak_3d"] <= .05
                          and float(native["tilt"].max()) <= 15., "New TS20 hover precision envelope failed")
            result = {"body_position": position}
    for path in folder.iterdir():
        if path.is_file() and path.suffix in (".json", ".jsonl", ".csv", ".ulg"):
            artifacts[str(path)] = task.sha(path)
    artifacts[str(Path(__file__).resolve())] = task.sha(__file__)
    report = {"kind": "x650_ts20_conditional_stage_certificate_v1", "complete": True, "passed": True,
        "stage": cfg["stage"], "source_run_directory": str(folder), "window_sim_s": window,
        "ts20_operating_condition": identity, "full_window_ulog_verified": True,
        "parameter_restore_verified": True, "native_ground_verified": True, "actuator_log_verified": True,
        "new_joint_prediction_verified": actuator["new_joint_prediction_verified"],
        "actuator": actuator, "configuration": configuration, "parameters": parameters,
        "commands": commands, "flight": flight, "ground": ground, "result": result,
        "timing": {"max_actual_send_interval_s": float(np.diff(ct).max()), "max_compute_ms": float(compute.max())},
        "qualification": cfg["ts20_protocol"]["qualification"], "artifacts": artifacts}
    (output / "ts20_stage_certificate.json").write_text(json.dumps(report, indent=2) + "\n")
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for key in ("run-dir", "launch-dir", "output"):
        parser.add_argument("--" + key, type=Path, required=True)
    args = parser.parse_args()
    report = audit(args.run_dir, args.launch_dir, args.output)
    print(json.dumps({key: report[key] for key in ("kind", "stage", "passed", "timing", "qualification")}, indent=2))


if __name__ == "__main__":
    main()
