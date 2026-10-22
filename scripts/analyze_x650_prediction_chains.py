#!/usr/bin/env python3
"""Original-time development diagnostics of failed MPC joint/attitude chains.

No controller optimization, response-model fitting, time-shift fitting or stage
certificate. Files stay at the supplied run paths; outputs are compact summaries.
"""
from __future__ import annotations
import argparse
import csv
import json
from pathlib import Path
import sys
import xml.etree.ElementTree as ET
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/"scripts"))
import plot_x650_mpc_failure as failure
import analyze_x650_joint_probe as joints
import analyze_x650_response_chain as response_chain
from x650_wholebody.model import ArmModel, tip_body, rotation_matrix
from x650_wholebody.mpc import rotation_increment

require, read, sha = failure.require, failure.read, failure.sha
MAX_NATIVE_GAP = .008
COMMAND_MATCH_TOL = 1e-6
MAX_COMMAND_EFFECT_SEARCH = .12


def stats(values):
    values = np.asarray(values, float)
    require(values.size and np.isfinite(values).all(), "Nonfinite or empty diagnostic statistic")
    return {"count": len(values), "min": float(values.min()), "p25": float(np.percentile(values, 25)),
            "median": float(np.median(values)), "p75": float(np.percentile(values, 75)),
            "p95": float(np.percentile(values, 95)), "max": float(values.max())}


def residual_summary(values):
    values = np.asarray(values, float)
    require(values.ndim == 2 and len(values) and np.isfinite(values).all(), "Invalid residual")
    return {"sample_count": len(values), "axis_rms": np.sqrt(np.mean(values**2, axis=0)).tolist(),
            "axis_peak_abs": np.max(abs(values), axis=0).tolist(), "axis_mean": values.mean(axis=0).tolist()}


def interpolate(times, values, queries, *, max_gap=MAX_NATIVE_GAP):
    """Only known original timestamps, bounded bracketing, no extrapolation."""
    t, y, q = np.asarray(times, float), np.asarray(values, float), np.asarray(queries, float)
    require(t.ndim == q.ndim == 1 and len(t) >= 2 and y.shape[0] == len(t)
            and np.isfinite(t).all() and np.isfinite(q).all() and np.all(np.diff(t) > 0), "Invalid original time vectors")
    require(q.min() >= t[0]-1e-9 and q.max() <= t[-1]+1e-9, "Diagnostic query lacks original native coverage")
    hi = np.minimum(np.searchsorted(t, q, side="left"), len(t)-1)
    exact = abs(t[hi]-q) <= 1e-9
    lo = np.where(exact, hi, np.maximum(hi-1, 0))
    require(np.all(t[hi]-t[lo] <= max_gap+1e-9) and np.isfinite(y[lo]).all() and np.isfinite(y[hi]).all(),
            "Diagnostic interpolation crosses missing/nonfinite native samples")
    w = np.zeros(len(q))
    np.divide(q-t[lo], t[hi]-t[lo], out=w, where=~exact)
    return y[lo]+w.reshape((-1,)+(1,)*(y.ndim-1))*(y[hi]-y[lo])


def quaternion_at(times, values, queries):
    q = np.asarray(values, float).copy()
    # q and -q are the same orientation; remove representation flips only.
    for i in range(1, len(q)):
        if q[i]@q[i-1] < 0: q[i] *= -1
    result = interpolate(times, q, queries)
    result /= np.linalg.norm(result, axis=1)[:, None]
    return response_chain.rotation_matrices(result)


def integration_check(t, q, dq):
    dt = np.diff(t)[:, None]
    right = np.vstack((q[0], q[0]+np.cumsum(dt*dq[1:], axis=0)))
    trapezoid = np.vstack((q[0], q[0]+np.cumsum(dt*.5*(dq[:-1]+dq[1:]), axis=0)))
    return {"right_endpoint_dq_integral_minus_q_rad": residual_summary(right-q),
            "trapezoid_dq_integral_minus_q_rad": residual_summary(trapezoid-q),
            "initialization": "Once from first native q; no reset/mean subtraction",
            "interpretation": "Discretization diagnostic, not an identified actuator model"}


def command_effect_events(ct, command, nt, actual, tolerance=COMMAND_MATCH_TOL, maximum_search=MAX_COMMAND_EFFECT_SEARCH):
    """Unique causal value-change witnesses; never match an earlier physical event.

    Pre-solve consumption timestamps are only lower bounds on command send time.
    Repeated/near-equal commands and many-to-one matches remain explicitly unresolved.
    """
    ct, command, nt, actual = map(lambda x: np.asarray(x, float), (ct, command, nt, actual))
    require(all(x.ndim == 1 and len(x) >= 2 and np.isfinite(x).all() for x in (ct, command, nt, actual))
            and len(ct) == len(command) and len(nt) == len(actual)
            and np.all(np.diff(ct) > 0) and np.all(np.diff(nt) > 0), "Invalid command-effect streams")
    native_events = np.flatnonzero(abs(np.diff(actual)) > 2*tolerance)+1
    events, claimed, previous_event = [], set(), -1
    for i in range(1, len(ct)):
        item = {"command_index": i, "consumption_sim_s": float(ct[i]), "command_rad_s": float(command[i])}
        if abs(command[i]-command[i-1]) <= 2*tolerance:
            item["status"] = "held_or_near_equal_command_no_identifiable_new_event"
            events.append(item)
            continue
        candidates = native_events[(nt[native_events] >= ct[i]-1e-9)
            &(nt[native_events] <= ct[i]+maximum_search+1e-9)
            &(abs(actual[native_events]-command[i]) <= tolerance)
            &(abs(actual[native_events-1]-command[i]) > tolerance)]
        if not len(candidates):
            item["status"] = "no_exact_new_native_value_in_causal_search"
        elif len(candidates) != 1:
            item["status"] = "multiple_native_witnesses_ambiguous"
        else:
            j = int(candidates[0])
            # Same/near value from any other causally eligible command makes
            # identity ambiguous. A newer command after this effect is excluded.
            possible = np.flatnonzero((ct <= nt[j]+1e-9)&(ct >= nt[j]-maximum_search-1e-9)
                                      &(abs(command-command[i]) <= 2*tolerance))
            if len(possible) != 1 or int(possible[0]) != i:
                item["status"] = "repeated_command_value_ambiguous"
            elif j in claimed or j <= previous_event:
                item["status"] = "native_event_reuse_or_order_conflict"
            elif nt[j-1] < ct[i]-1e-9:
                item["status"] = "native_event_interval_straddles_consumption"
            else:
                item.update(status="unique_causal_value_change_witness", native_first_value_sim_s=float(nt[j]),
                    native_event_bracket_sim_s=[float(nt[j-1]), float(nt[j])],
                    consumption_to_observed_effect_interval_s=[float(nt[j-1]-ct[i]), float(nt[j]-ct[i])],
                    value_error_rad_s=float(actual[j]-command[i]))
                claimed.add(j)
                previous_event = j
        events.append(item)
    counts = {key: sum(x["status"] == key for x in events) for key in sorted({x["status"] for x in events})}
    accepted = [x for x in events if x["status"] == "unique_causal_value_change_witness"]
    return {"command_rows": len(ct), "initial_command_not_identifiable": 1, "status_counts": counts,
        "value_tolerance_rad_s": tolerance, "causal_search_s": maximum_search,
        "effect_first_sample_minus_consumption_s": stats([x["consumption_to_observed_effect_interval_s"][1] for x in accepted]) if accepted else None,
        "effect_preceding_sample_minus_consumption_s": stats([x["consumption_to_observed_effect_interval_s"][0] for x in accepted]) if accepted else None,
        "interpretation": "Combined pre-solve compute, send, bridge and physics effect interval; NOT pure actuator delay or causal identity proof",
        "unresolved_examples": [x for x in events if x not in accepted][:8]}, events


def read_native(folder, window):
    rows = failure.display.csv_rows(folder/"native_trace.csv")
    t = joints.array(rows, ["sim_s"]).ravel()
    require(np.all(np.diff(t) > 0), "Native time must increase")
    mask = (t >= window[0]-1e-9)&(t <= window[1]+1e-9)
    require(mask.sum() >= 2 and all(rows[i][key] == "1" for i in np.flatnonzero(mask)
            for key in ("valid_base", "valid_joint_state", "valid_accel", "valid_tip")), "Invalid original native measurement state")
    require(t[mask][0] <= window[0]+MAX_NATIVE_GAP and t[mask][-1] >= window[1]-MAX_NATIVE_GAP
            and np.diff(t[mask]).max() <= MAX_NATIVE_GAP+1e-9, "Native measurement coverage missing")
    # Context remains available for real feedback headers preceding the window.
    def values(names):
        return np.asarray([[float(row[name]) for name in names] for row in rows], float)
    return {"t": t, "mask": mask, "position": values(["base_p"+a for a in "xyz"]),
            "velocity": values(["base_v"+a for a in "xyz"]), "acceleration": values(["base_a"+a for a in "xyz"]),
            "quaternion": values(["base_q"+a for a in "wxyz"]), "tip": values(["tip_"+a for a in "xyz"]),
            "q": values(["arm_joint_1_q", "arm_joint_2_q"]), "dq": values(["arm_joint_1_qd", "arm_joint_2_qd"])}


def gravity_for(task, explicit, inputs):
    if explicit is not None:
        require(np.isfinite(explicit) and explicit > 0, "Positive explicit diagnostic gravity required")
        return float(explicit), "explicit --gravity-m-s2 diagnostic assumption"
    world = Path(task["research_profile"]["world_sdf"])
    require(world.is_file(), "Original expanded world required, or supply explicit --gravity-m-s2")
    values = ET.parse(world).getroot().findall(".//world/gravity")
    require(len(values) == 1, "One explicit world gravity required, or supply --gravity-m-s2")
    g = np.fromstring(values[0].text, sep=" ")
    require(g.shape == (3,) and np.isfinite(g).all() and max(abs(g[:2])) <= 1e-12 and g[2] < 0, "World gravity is not -ENU Z")
    inputs[str(world.resolve())] = sha(world)
    return float(-g[2]), "original expanded world gravity"


def geometry_chain(native, model, anchor, reference_rotation, g):
    mask = native["mask"]
    t, p, q, actual_tip, a = (native[key][mask] for key in ("t", "position", "q", "tip", "acceleration"))
    R = response_chain.rotation_matrices(native["quaternion"][mask])
    r = np.asarray([tip_body(value, model) for value in q])
    r0 = tip_body([0., 0.], model)
    R0 = rotation_matrix(reference_rotation)
    lever = np.einsum("nij,nj->ni", R, r)
    fk_error = actual_tip-(p+lever)
    base = p-np.asarray(anchor)
    orientation = np.einsum("nij,j->ni", R-R0, r0)
    arm = (r-r0)@R0.T
    interaction = np.einsum("nij,nj->ni", R-R0, r-r0)
    target = np.asarray(anchor)+R0@r0
    error = actual_tip-target
    closure = error-(base+orientation+arm+interaction+fk_error)
    up = R[:, :, 2]
    specific = a+np.array([0., 0., g])
    norm = np.linalg.norm(specific, axis=1)
    require(np.all(norm > 1e-6) and np.all(up[:, 2] > 0), "Upright nonzero specific acceleration required")
    direction = specific/norm[:, None]
    angle = np.degrees(np.arccos(np.clip(np.sum(up*direction, axis=1), -1, 1)))
    projected = np.degrees(np.arctan2(up[:, :2], up[:, 2, None]))
    accel_tilt = np.degrees(np.arctan2(specific[:, :2], specific[:, 2, None]))
    report = {"native_fk_residual_m": residual_summary(fk_error), "exact_tip_decomposition_closure_m": residual_summary(closure),
        "tip_decomposition_m": {key: residual_summary(value) for key, value in
            (("base", base), ("orientation_at_q0", orientation), ("arm_at_R0", arm), ("orientation_arm_interaction", interaction), ("total", error))},
        "body_up_minus_normalized_acceleration_plus_gravity": residual_summary(up-direction),
        "body_up_vs_specific_acceleration_angle_deg": stats(angle),
        "gravity_m_s2": g,
        "interpretation": "Same-step original native geometry; acceleration relation is a diagnostic, not exact thrust dynamics. Body origin need not be COM; arm forces, drag and rotational transport may contribute. No unique causal attribution."}
    return report, {"t": t, "error": error, "base": base, "orientation": orientation, "arm": arm,
                    "interaction": interaction, "fk_error": fk_error, "projected_tilt": projected, "accel_tilt": accel_tilt}


def prediction_errors(data, native, feedback, entries, model):
    buckets, excluded, maximum_rebuild = {}, 0, 0.
    protocol = data["task"]["mixed_mpc_protocol"]
    configuration = protocol["mpc_controller_config"]
    configured_chain = configuration.get("prediction_chain")
    chain_enabled = "prediction_chain" in configuration or protocol.get("name") == "mixed_response_prediction_chain_v2"
    require(not chain_enabled or isinstance(configured_chain, dict) and configured_chain,
            "Prediction-chain protocol requires its explicit original configuration")
    initial_errors, raw_initial_errors, propagation_changes = [], [], []
    for row, entry in zip(feedback, entries):
        d = entry["diagnostics"]
        require(all(key in d for key in ("predicted_states", "predicted_tip_positions", "node_intervals_s")), "Original horizon diagnostics missing")
        pred = np.asarray(d["predicted_states"], float)
        tip_pred = np.asarray(d["predicted_tip_positions"], float)
        h = np.cumsum(np.asarray(d["node_intervals_s"], float))
        require(pred.shape == (len(h), 12) and tip_pred.shape == (len(h), 3)
                and np.isfinite(pred).all() and np.isfinite(tip_pred).all() and np.all(np.diff(np.r_[0., h]) > 0), "Invalid original mixed forecast")
        state = row["feedback"]["control_state"]
        if chain_enabled:
            require(d.get("prediction_chain") == configured_chain, "Saved prediction chain differs from original task configuration")
            require("predicted_rotations_flu_to_enu" in d, "Prediction-chain forecast lacks saved rotations; legacy reconstruction is prohibited")
            Rp = np.asarray(d["predicted_rotations_flu_to_enu"], float)
            require(Rp.shape == (len(h), 3, 3) and np.isfinite(Rp).all()
                    and np.allclose(np.swapaxes(Rp, 1, 2)@Rp, np.eye(3), atol=1e-8, rtol=0)
                    and np.all(abs(np.linalg.det(Rp)-1.) <= 1e-8),
                    "Saved prediction rotations must be proper finite FLU-to-ENU matrices")
            initial_q = np.asarray(d.get("prediction_initial_joint_position"), float)
            measured_q = np.asarray(state["q_rad"], float)
            require(initial_q.shape == measured_q.shape == (2,) and np.isfinite(initial_q).all()
                    and np.isfinite(measured_q).all(), "Prediction chain lacks finite original initial joint position")
            actual_q = interpolate(native["t"], native["q"], [float(entry["sim_s"])])[0]
            initial_errors.append(initial_q-actual_q)
            raw_initial_errors.append(measured_q-actual_q)
            propagation_changes.append(initial_q-measured_q)
        else:
            require(d.get("prediction_chain") is None, "Legacy task cannot relabel a saved prediction-chain forecast")
            R0, omega = rotation_matrix(state["rotation_flu_to_enu"]), np.asarray(state["omega_flu_rad_s"], float)
            tau = configuration["attitude_rate_decay_s"]
            Rp = np.asarray([R0@rotation_increment(omega*tau*(1-np.exp(-step/tau))) for step in h])
        rp = np.asarray([tip_body(q, model) for q in pred[:, 6:8]])
        rebuilt = pred[:, :3]+np.einsum("nij,nj->ni", Rp, rp)
        maximum_rebuild = max(maximum_rebuild, float(np.max(abs(rebuilt-tip_pred))))
        require(maximum_rebuild <= 1e-8, "Stored prediction is not the declared original attitude/geometry model")
        query = float(entry["sim_s"])+h
        supported = query <= min(data["window"][1], native["t"][-1])+1e-9
        excluded += int((~supported).sum())
        if not supported.any(): continue
        qt = query[supported]
        pa, qa, ta = (interpolate(native["t"], native[key], qt) for key in ("position", "q", "tip"))
        Ra = quaternion_at(native["t"], native["quaternion"], qt)
        ra = np.asarray([tip_body(q, model) for q in qa])
        fka = ta-(pa+np.einsum("nij,nj->ni", Ra, ra))
        bp = pred[supported, :3]-pa
        jp = np.einsum("nij,nj->ni", Rp[supported], rp[supported]-ra)
        ap = np.einsum("nij,nj->ni", Rp[supported]-Ra, ra)
        ep = tip_pred[supported]-ta
        closure = ep-(bp+jp+ap-fka)
        for i, node in enumerate(np.flatnonzero(supported)):
            buckets.setdefault(int(node)+1, []).append({"horizon_s": h[node], "tip": ep[i], "base": bp[i],
                "joint_geometry": jp[i], "attitude_geometry": ap[i], "native_fk": fka[i], "closure": closure[i],
                "q_error": pred[node, 6:8]-qa[i]})
    summary = []
    for node, rows in sorted(buckets.items()):
        summary.append({"node": node, "horizon_s": stats([row["horizon_s"] for row in rows]),
                        **{key: residual_summary([row[key] for row in rows]) for key in
                           ("tip", "base", "joint_geometry", "attitude_geometry", "native_fk", "closure", "q_error")}})
    return {"available": True, "original_forecasts": len(entries), "unsupported_nodes_beyond_partial_end": excluded,
        "maximum_stored_tip_rebuild_error_m": maximum_rebuild, "by_node": summary,
        "attitude_source": "original saved predicted_rotations_flu_to_enu" if chain_enabled else "original legacy angular-rate-decay reconstruction",
        "prediction_initial_joint_position": {
            "available": True, "source": "original diagnostics.prediction_initial_joint_position",
            "comparison_time": "Original control consumption sim_s, not command dispatch or future prediction node",
            "propagated_minus_native_at_control_rad": residual_summary(initial_errors),
            "raw_feedback_minus_native_at_control_rad": residual_summary(raw_initial_errors),
            "propagated_minus_raw_feedback_rad": residual_summary(propagation_changes),
            "interpretation": "Diagnostic propagation error only; no parameter fit, threshold or certificate"
        } if chain_enabled else {"available": False, "reason": "Legacy forecast has no declared propagated initial joint position"},
        "interpolation": "Native values at original forecast timestamps; bounded <=8ms interpolation, normalized quaternion interpolation, no time shift",
        "interpretation": "Overlapping issued forecasts, not independent observations. Later-node errors include future replanning different from the original predicted command sequence; not solely dynamics-model error."}


def formal_context(path, role, gravity):
    report = read(path)
    require(report.get("kind") == "x650_precision_response_protocol_audit" and all(report.get(key) is True for key in
            ("complete", "passed", "research_acceptance_verified", "full_window_ulog_verified")), "Optional formal run needs its accepted audit")
    require(report["response_probe_protocol"]["role"] == role, "Formal response role mismatch")
    folder = Path(report["source_run_directory"]).resolve()
    inputs = {str(Path(path).resolve()): sha(path)}
    for name in ("controller_metadata.json", "native_trace.csv", "control_trace.csv"):
        source = folder/name
        require(report["inputs_sha256"].get(str(source)) == sha(source), "Optional original formal input hash mismatch: "+name)
        inputs[str(source)] = sha(source)
    meta = read(folder/"controller_metadata.json")
    window = report["window_sim_s"]
    native = read_native(folder, window)
    g, source = gravity_for(meta["task"], gravity, inputs)
    geometry, curves = geometry_chain(native, ArmModel(**meta["physical_profile"]), meta["reference_base_enu_m"], meta["reference_rotation"], g)
    control = [row for row in failure.display.csv_rows(folder/"control_trace.csv") if row["phase"] == "measure"
               and window[0]-1e-9 <= float(row["sim_s"]) <= window[1]+1e-9]
    command = joints.array(control, ["base_velocity_target_"+axis for axis in "xyz"])
    return {"role": role, "source_run": folder.name, "window_sim_s": window, "geometry": geometry,
            "gravity_source": source, "inputs_sha256": inputs, "existing_model_reestimated": False}, curves, joints.array(control, ["sim_s"]).ravel(), command


def load_diagnostic_run(folder):
    folder = Path(folder).resolve()
    meta = read(folder/"controller_metadata.json")
    if meta.get("complete") is False:
        return failure.load_failure(folder)
    require(meta.get("complete") is True and meta.get("phase") == "complete", "Run must explicitly complete or fail")
    task = read(folder/"controller_task.json")
    require(meta["task"] == task and task.get("controller") == "mpc", "Original MPC task required")
    window = [float(meta["task_start_sim_s"]), float(meta["task_end_sim_s"])]
    require(np.isfinite(window).all() and window[1] > window[0]
            and abs(window[1]-window[0]-task["duration_s"]) <= .05, "Completed run lacks its full declared window")
    control = [row for row in failure.display.csv_rows(folder/"control_trace.csv") if row.get("phase") == "measure"
               and window[0]-1e-9 <= float(row["sim_s"]) <= window[1]+1e-9]
    selected, ct = failure.audit.selected_rows(control, window)
    require(all(row.get("pva_sent") in ("True", "true", "1") for row in selected), "Unsent measurement command")
    names = ("controller_metadata.json", "controller_task.json", "native_trace.csv", "control_trace.csv", "controller_diagnostics.jsonl")
    inputs = {str(folder/name): sha(folder/name) for name in names}
    return dict(folder=folder, metadata=meta, task=task, window=window, inputs=inputs, control=selected, control_t=ct-window[0])


def analyze(folder, gravity=None):
    data = load_diagnostic_run(folder)
    require(data["task"]["stage"] == "tip_hold", "This diagnostic requires fixed-tip hold")
    folder, inputs = data["folder"], data["inputs"]
    feedback_path = folder/"feedback_trace.jsonl"
    inputs[str(feedback_path)] = sha(feedback_path)
    feedback = [json.loads(line) for line in feedback_path.read_text(encoding="utf-8").splitlines() if line.strip()]
    feedback = [row for row in feedback if row.get("phase") == "measure" and data["window"][0]-1e-9 <= row["sim_s"] <= data["window"][1]+1e-9]
    entries = [json.loads(line) for line in (folder/"controller_diagnostics.jsonl").read_text(encoding="utf-8").splitlines() if line.strip()]
    entries = [row for row in entries if row.get("phase") == "measure" and data["window"][0]-1e-9 <= row["sim_s"] <= data["window"][1]+1e-9]
    ct = data["control_t"]+data["window"][0]
    require(len(entries) == len(ct) and np.allclose([row["sim_s"] for row in entries], ct, atol=1e-9, rtol=0), "Original predictions do not pair with every control")
    require(len(feedback) == len(ct) and np.allclose([row["sim_s"] for row in feedback], ct, atol=1e-9, rtol=0), "Original feedback does not pair with every control")
    native = read_native(folder, data["window"])
    mask, nt = native["mask"], native["t"]
    header = joints.array(data["control"], ["joint_stamp_s"]).ravel()
    qfb, dqfb = (joints.array(data["control"], keys) for keys in (["q1", "q2"], ["qd1", "qd2"]))
    require(np.all(np.diff(header) >= -1e-9) and np.max(abs(header-ct)) <= .25+1e-9, "Joint header age violates original contract")
    for row, q, dq, stamp in zip(feedback, qfb, dqfb, header):
        d = row["feedback"]
        require(abs(d["joint_stamp_s"]-stamp) <= 1e-9 and np.max(abs(np.asarray(d["control_state"]["q_rad"])-q)) <= 1e-9
                and np.max(abs(np.asarray(d["control_state"]["dq_rad_s"])-dq)) <= 1e-9, "Control and original feedback joint payloads differ")
    sync = {"consumption_minus_joint_header_s": stats(ct-header), "repeated_joint_headers": int(np.sum(np.diff(header) == 0)),
            "q_feedback_minus_native_at_header_rad": residual_summary(qfb-interpolate(nt, native["q"], header)),
            "dq_feedback_minus_native_at_header_rad_s": residual_summary(dqfb-interpolate(nt, native["dq"], header)),
            "q_feedback_minus_native_at_consumption_rad": residual_summary(qfb-interpolate(nt, native["q"], ct)),
            "dq_feedback_minus_native_at_consumption_rad_s": residual_summary(dqfb-interpolate(nt, native["dq"], ct)),
            "interpolation_max_gap_s": MAX_NATIVE_GAP, "time_shift_s": 0.}
    command = joints.array(data["control"], ["qd1_cmd", "qd2_cmd"])
    joint_report, events = {}, []
    for j in range(2):
        summary, rows = command_effect_events(ct, command[:, j], nt[mask], native["dq"][mask, j])
        joint_report[f"joint_{j+1}"] = summary
        events.extend({"joint": j+1, **row} for row in rows)
    g, gravity_source = gravity_for(data["task"], gravity, inputs)
    model = ArmModel(**data["metadata"]["physical_profile"])
    geometry, curves = geometry_chain(native, model, data["metadata"]["reference_base_enu_m"], data["metadata"]["reference_rotation"], g)
    prediction = prediction_errors(data, native, feedback, entries, model)
    report = {"kind": "x650_prediction_chain_development_diagnostic", "schema_version": 1,
        "diagnostic_only": True, "eligible_as_stage_gate": False, "experiment_complete": data["metadata"]["complete"],
        "source_run": folder.name, "partial_window_sim_s": data["window"], "sent_commands": len(ct),
        "formal_response_model_modified_or_refitted": False, "fitted_time_or_phase_shift": False,
        "no_full_stage_or_pid_ranking_claim": True,
        "joint_integral": integration_check(nt[mask], native["q"][mask], native["dq"][mask]),
        "relative_feedback_synchronization": sync, "joint_command_effect": joint_report,
        "geometry_and_acceleration_chain": geometry, "gravity_source": gravity_source,
        "original_prediction_errors": prediction, "inputs_sha256": inputs}
    return report, data, native, curves, events


def generate(folder, output, *, gravity=None, formal_train=None, formal_holdout=None):
    report, data, native, curves, events = analyze(folder, gravity)
    output = Path(output).resolve()
    require(not output.exists(), "A fresh output directory is required")
    contexts = []
    for role, path in (("train", formal_train), ("holdout", formal_holdout)):
        if path is not None:
            contexts.append(formal_context(path, role, gravity))
    output.mkdir(parents=True)
    event_path = output/"joint_command_effect_events.json"
    event_path.write_text(json.dumps(events, indent=2, allow_nan=False)+"\n", encoding="utf-8")
    start = data["window"][0]
    mask = native["mask"]
    t, ct = native["t"][mask]-start, data["control_t"]
    fig, axes = plt.subplots(3, 2, figsize=(16, 12), constrained_layout=True)
    command = joints.array(data["control"], ["qd1_cmd", "qd2_cmd"])
    for j in range(2):
        axes[0, j].plot(t, native["dq"][mask, j], label="Native actual dq")
        axes[0, j].step(ct, command[:, j], where="post", label="Sent value at pre-solve consumption time", ls="--")
        axes[0, j].set_ylabel(f"Joint {j+1} velocity (rad/s)")
    for j, name in enumerate("XY"):
        axes[1, j].plot(t, curves["projected_tilt"][:, j], label="Native body-up projected tilt")
        axes[1, j].plot(t, curves["accel_tilt"][:, j], ls="--", label="Direction of native a + gravity")
        axes[1, j].set_ylabel(name+" tilt direction (deg; yaw-independent)")
    for key, label in (("error", "Total tip Y error"), ("base", "Base Y"), ("orientation", "Orientation at q0"),
                       ("arm", "Joint geometry at R0"), ("interaction", "Orientation/joint interaction")):
        axes[2, 0].plot(t, 1000*curves[key][:, 1], label=label)
    axes[2, 0].set_ylabel("Exact native tip-Y decomposition (mm)")
    by_node = report["original_prediction_errors"]["by_node"]
    for key, label in (("tip", "Total forecast"), ("base", "Base"), ("joint_geometry", "Joint geometry"), ("attitude_geometry", "Attitude geometry")):
        axes[2, 1].plot([row["horizon_s"]["median"] for row in by_node], [1000*row[key]["axis_rms"][1] for row in by_node], marker=".", label=label)
    axes[2, 1].set_ylabel("Forecast Y RMS by node (mm)")
    for ax in axes.ravel():
        ax.grid(alpha=.25); ax.legend(fontsize=8); ax.set_xlabel("Original simulation time since measurement start (s)")
    axes[2, 1].set_xlabel("Original node horizon, median (s); later replanning is not model-only error")
    label = "Completed MPC" if report["experiment_complete"] else "FAILED MPC partial window"
    fig.suptitle(label+" development diagnostic | no delay fit / no model refit / no acceptance", fontsize=13)
    outputs = [event_path]
    for suffix in (".png", ".pdf"):
        path = output/("prediction_chains"+suffix); fig.savefig(path, dpi=150, bbox_inches="tight"); outputs.append(path)
    plt.close(fig)
    report["formal_context"] = [item[0] for item in contexts]
    for summary, values, command_t, commands in contexts:
        fig, axes = plt.subplots(3, 2, figsize=(15, 9), constrained_layout=True)
        origin = summary["window_sim_s"][0]
        for j, name in enumerate("XY"):
            axes[0, j].step(command_t-origin, commands[:, j], where="post", label="Original logical velocity")
            axes[0, j].set_ylabel(name+" velocity command (m/s)")
            axes[1, j].plot(values["t"]-origin, values["projected_tilt"][:, j], label="Native body-up tilt")
            axes[1, j].plot(values["t"]-origin, values["accel_tilt"][:, j], label="a + gravity direction", ls="--")
            axes[1, j].set_ylabel(name+" projected tilt (deg)")
            axes[2, j].plot(values["t"]-origin, 1000*values["orientation"][:, j], label="Native orientation geometry")
            axes[2, j].set_ylabel(name+" tip orientation contribution (mm)")
        for ax in axes.ravel(): ax.grid(alpha=.25); ax.legend(fontsize=8); ax.set_xlabel("Original measurement time (s)")
        fig.suptitle(summary["role"]+": original accepted formal response, additional geometry diagnostic only")
        for suffix in (".png", ".pdf"):
            path = output/("formal_"+summary["role"]+"_geometry_chain"+suffix); fig.savefig(path, dpi=150, bbox_inches="tight"); outputs.append(path)
        plt.close(fig)
        report["inputs_sha256"].update(summary["inputs_sha256"])
    require(all(sha(path) == digest for path, digest in report["inputs_sha256"].items()), "Original evidence changed during diagnostic")
    report.update(render_complete=True, generator_sha256=sha(__file__),
        helper_sources_sha256={str(Path(module.__file__).resolve()): sha(module.__file__) for module in
            (failure, failure.display, failure.audit, joints, response_chain,
             sys.modules[tip_body.__module__], sys.modules[rotation_increment.__module__])},
        outputs_sha256={str(path): sha(path) for path in outputs})
    (output/"prediction_chain_diagnostic.json").write_text(json.dumps(report, indent=2, allow_nan=False)+"\n", encoding="utf-8")
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--gravity-m-s2", type=float)
    parser.add_argument("--formal-train-audit", type=Path)
    parser.add_argument("--formal-holdout-audit", type=Path)
    args = parser.parse_args()
    report = generate(args.run, args.output, gravity=args.gravity_m_s2,
                      formal_train=args.formal_train_audit, formal_holdout=args.formal_holdout_audit)
    print(json.dumps({"diagnostic_only": True, "source_run": report["source_run"], "partial_window_sim_s": report["partial_window_sim_s"],
                      "joint_command_effect": report["joint_command_effect"], "output": str(args.output.resolve())}))


if __name__ == "__main__": main()
