#!/usr/bin/env python3
"""Audit native X650 commanded-position step response; no dynamics are simulated.

Uses the fixed declared world anchor, source-bound six-step protocol, and actual
first-send simulation times. It never fits response endpoints or re-centers the
reference on a measured position. This is a reference-response test, not an
external-force rejection experiment.
"""
from __future__ import annotations

import argparse
import csv
import importlib.util
import json
from pathlib import Path

import numpy as np


ROOT = Path(__file__).resolve().parents[1]
PRECISION_SOURCE = Path(__file__).with_name("analyze_x650_hover_precision.py")
DEFAULT_PROBE_SOURCE = ROOT/"simulations/x650_wholebody_mpc_v1/position_step_probe.py"
DEFAULT_REFERENCE_SOURCE = ROOT/"simulations/x650_wholebody_mpc_v1/hover_reference.py"


def module(path, name):
    spec = importlib.util.spec_from_file_location(name, path)
    result = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(result)
    return result


physics = module(PRECISION_SOURCE, "x650_step_native_metrics")
sha = physics.sha


def vector(value, label):
    result = np.asarray(value, float)
    if result.shape != (3,) or not np.isfinite(result).all():
        raise ValueError(label+" must be a finite three-vector")
    return result


def same_vector(actual, expected, label, tolerance=1e-9):
    result = vector(actual, label)
    if not np.allclose(result, expected, rtol=0, atol=tolerance):
        raise ValueError(label+" differs from the declared protocol/reference")
    return result


def verified_segments(document, reference, probe_source, reference_source):
    """Compare the archived protocol to its exact source and fixed origin."""
    source = module(probe_source, "x650_step_declared_protocol")
    expected = source.probe_metadata()
    # Runtime adds one reference document per segment; all other protocol fields
    # must equal the source declaration, including source hash and thresholds.
    stripped = dict(document)
    stripped["segments"] = [{k: v for k, v in row.items() if k != "reference"}
                            for row in document.get("segments", [])]
    if stripped != expected:
        raise ValueError("Position-step protocol differs from its bound source")
    anchor = physics.verify_reference(reference, reference_source)
    fixed_origin = reference["px4_reference_at_bind"]
    origin_keys = ("ref_lat", "ref_lon", "ref_alt", "xy_global", "z_global",
                   "xy_reset_counter", "z_reset_counter", "heading_reset_counter")
    for segment in document["segments"]:
        bound = segment.get("reference", {})
        target = physics.verify_reference(bound, reference_source)
        same_vector(target, anchor+np.asarray(segment["target_offset_world_enu_m"]), "Step absolute target")
        if (bound["frame_mode"] != reference["frame_mode"]
                or bound.get("world") != reference.get("world")
                or bound.get("fixed_yaw_ned_rad") != reference.get("fixed_yaw_ned_rad")
                or any(bound["px4_reference_at_bind"][k] != fixed_origin[k] for k in origin_keys)):
            raise ValueError("Step reference changed the fixed origin, world, frame or yaw")
    return document["segments"], expected["analysis_contract"]


def actual_windows(metadata, segments, anchor):
    """Use logged first sends, while retaining nominal schedule and send delay."""
    begin, primary_end, end = (float(metadata[key]) for key in
                              ("task_start_sim_s", "task_end_sim_s", "experiment_end_sim_s"))
    if (not np.isfinite([begin, primary_end, end]).all()
            or abs(primary_end-begin-20.) > .05+1e-9 or abs(end-begin-56.) > .05+1e-9):
        raise ValueError("Expected unchanged 20 s hover followed by the declared 36 s probe")
    events = metadata.get("position_step_send_events", [])
    steps = [event for event in events if event.get("segment_index") is not None]
    if [event.get("segment_index") for event in steps] != list(range(6)):
        raise ValueError("Actual first-send records must contain all six steps once in order")
    times = [float(event["first_send_sim_s"]) for event in events]
    if not np.isfinite(times).all() or np.any(np.diff(times) <= 0):
        raise ValueError("Actual command first-send times must be finite and increasing")
    resets = [event for event in events if event.get("segment_index") is None]
    if len(resets) > 1 or (resets and (events[-1] is not resets[0]
                                     or float(resets[0]["first_send_sim_s"]) < begin+56.-1e-8)):
        raise ValueError("Only one final post-probe return-to-anchor event is allowed")
    if resets:
        same_vector(resets[0]["target_world_enu_m"], anchor, "Final return target")
    result = []
    for index, (segment, event) in enumerate(zip(segments, steps)):
        sent = float(event["first_send_sim_s"])
        elapsed = float(event["elapsed_s"])
        nominal_begin, nominal_end = begin+segment["start_s"], begin+segment["end_s"]
        if (abs(sent-begin-elapsed) > 1e-8 or sent < nominal_begin-1e-8 or sent >= nominal_end):
            raise ValueError("Actual command event is outside its declared segment")
        same_vector(event["target_world_enu_m"], segment["reference"]["target_world_enu_m"], "Actual sent world target")
        same_vector(event["position_target_local_ned_m"], segment["reference"]["target_local_ned_m"], "Actual sent local target")
        stop = float(steps[index+1]["first_send_sim_s"]) if index < 5 else end
        if index == 5 and resets:
            stop = min(stop, float(resets[0]["first_send_sim_s"]))
        if stop <= sent or stop > end+1e-8:
            raise ValueError("Missing or inconsistent post-step observation window")
        result.append(dict(segment_index=index, nominal_window_sim_s=[nominal_begin, nominal_end],
                           actual_window_sim_s=[sent, stop], command_send_delay_s=sent-nominal_begin,
                           observation_duration_s=stop-sent))
    return result


def first_crossing(t, response, level):
    """Linear crossing between native samples; no model, phase or endpoint fit."""
    if response[0] >= level:
        return {"status": "left_censored", "time_s": None}
    matches = np.flatnonzero(response >= level)
    if not len(matches):
        return {"status": "not_reached", "time_s": None}
    right = int(matches[0]); left = right-1
    fraction = (level-response[left])/(response[right]-response[left])
    return {"status": "observed", "time_s": float(t[left]+fraction*(t[right]-t[left]))}


def settling(t, position_error, velocity, *, position_band=.02, speed_band=.03, hold_s=2.):
    """Two-second consecutive native-sample qualification, with <=8ms gaps."""
    good = (np.linalg.norm(position_error, axis=1) <= position_band+1e-12) & (
            np.linalg.norm(velocity, axis=1) <= speed_band+1e-12)
    runs, begin = [], None
    for index, value in enumerate(np.r_[good, False]):
        if value and begin is None:
            begin = index
        elif not value and begin is not None:
            stop = index-1
            if t[stop]-t[begin] >= hold_s-1e-9:
                runs.append((begin, stop))
            begin = None
    final_run = next((pair for pair in runs if pair[1] == len(t)-1), None)
    first = runs[0] if runs else None
    return dict(position_band_3d_m=position_band, speed_band_3d_m_s=speed_band,
        minimum_consecutive_duration_s=hold_s,
        settled_2s_observed=bool(runs),
        first_qualified_entry_time_s=None if first is None else float(t[first[0]]),
        first_qualification_confirmed_time_s=None if first is None else float(t[first[0]]+hold_s),
        settled_through_window_end=final_run is not None,
        permanent_observed_entry_time_s=None if final_run is None else float(t[final_run[0]]),
        qualifying_run_count=len(runs),
        not_settled_reason=None if runs else "No complete 2 s interval satisfies both position and speed thresholds",
        semantics="Consecutive recorded physical samples with <=8ms spacing; not a continuous-time collision or stability proof")


def response_metrics(native, segment, window, anchor, contract):
    t = native["t"]-window[0]
    target = vector(segment["reference"]["target_world_enu_m"], "Step target")
    previous = anchor+np.asarray(segment["previous_target_offset_world_enu_m"], float)
    error = native["position"]-target
    axis, direction, amplitude = segment["axis_index"], segment["direction"], segment["step_amplitude_m"]
    progress = direction*(native["position"][:, axis]-previous[axis])/amplitude
    crossings = [first_crossing(t, progress, value) for value in contract["rise_fraction"]]
    rise = (crossings[1]["time_s"]-crossings[0]["time_s"]
            if all(item["status"] == "observed" for item in crossings) else None)
    overshoot = max(0., float(np.max(direction*error[:, axis])))
    cross_axes = [index for index in range(3) if index != axis]
    cross = error.copy(); cross[:, axis] = 0.
    late = t >= t[-1]-2.-1e-9
    report = dict(segment_index=segment["segment_index"], segment_id=segment["segment_id"],
        axis=segment["axis"], direction=direction, commanded_step_m=segment["signed_step_m"],
        target_world_enu_m=target.tolist(), previous_commanded_target_world_enu_m=previous.tolist(),
        position_error_world_enu=physics.metrics(native["t"], error, "m"),
        final_2s_position_error_world_enu=physics.metrics(native["t"][late], error[late], "m"),
        cross_axis_names=["xyz"[index] for index in cross_axes],
        cross_axis_position_error=physics.metrics(native["t"], cross, "m"),
        physical_velocity_world_enu=physics.metrics(native["t"], native["velocity"], "m/s"),
        tilt_peak_deg=float(native["tilt"].max()), native_sample_count=len(t),
        native_max_interval_s=float(np.diff(native["t"]).max()),
        rise_10pct=crossings[0], rise_90pct=crossings[1], rise_10_90_s=rise,
        rise_semantics="Signed physical position relative to adjacent declared targets; no fitted start/end or measured re-centering",
        overshoot_m=overshoot, overshoot_percent_of_commanded_step=100*overshoot/amplitude,
        settling=settling(t, error, native["velocity"],
            position_band=contract["settling_position_error_m"], speed_band=contract["settling_speed_m_s"],
            hold_s=contract["settling_continuous_duration_s"]),
        native_first_sample_delay_from_send_s=float(t[0]), native_last_sample_before_window_end_s=float(window[1]-native["t"][-1]))
    return report


def native_manipulator_validity(path, window):
    """Check the complete rig remains observable, without inferring FK truth."""
    with Path(path).open(newline="", encoding="utf-8") as stream:
        rows = [row for row in csv.DictReader(stream)
                if window[0]-1e-9 <= float(row["sim_s"]) <= window[1]+1e-9]
    flags = ("valid_base", "valid_joint_state", "valid_tip")
    names = ("arm_joint_1_q", "arm_joint_2_q", "arm_joint_1_qd", "arm_joint_2_qd", "tip_x", "tip_y", "tip_z")
    if not rows or any(row.get(key) != "1" for row in rows for key in flags):
        raise ValueError("Complete step experiment requires valid native base, joint and tip states")
    values = np.asarray([[float(row[key]) for key in names] for row in rows], float)
    if not np.isfinite(values).all():
        raise ValueError("Nonfinite native manipulator state inside the step experiment")
    return dict(validity_flags=list(flags), validated_full_window=True,
                physical_joint_peak_abs_rad=np.max(abs(values[:, :2]), axis=0).tolist(),
                physical_joint_speed_peak_abs_rad_s=np.max(abs(values[:, 2:4]), axis=0).tolist(),
                tip_semantics="Native tip validity/finite fields checked; this position-step audit does not claim a new FK or contact validation")


def analyze(run_dir, output, *, probe_source=DEFAULT_PROBE_SOURCE, reference_source=DEFAULT_REFERENCE_SOURCE):
    folder, output = Path(run_dir).resolve(strict=True), Path(output).resolve()
    paths = {name: folder/filename for name, filename in dict(metadata="controller_metadata.json",
        reference="hover_reference.json", probe="position_step_probe.json", native="native_trace.csv",
        native_manifest="native_trace_manifest.json").items()}
    documents = {name: json.loads(path.read_text(encoding="utf-8")) for name, path in paths.items() if name != "native"}
    metadata, reference, probe, manifest = (documents[key] for key in ("metadata", "reference", "probe", "native_manifest"))
    if metadata.get("complete") is not True or metadata.get("landed_and_disarmed") is not True:
        raise ValueError("Step audit requires a completed and recovered experiment")
    if metadata.get("position_step_probe") != probe:
        raise ValueError("Standalone step protocol differs from completed metadata")
    if (manifest.get("complete") is not True or manifest.get("sha256") != sha(paths["native"])
            or manifest.get("model_sha256") != metadata.get("model_sha256")
            or manifest.get("source_urdf_sha256") != metadata.get("source_urdf_sha256")):
        raise ValueError("Independent native capture completion/hash/model mismatch")
    segments, contract = verified_segments(probe, reference, probe_source, reference_source)
    anchor = vector(reference["target_world_enu_m"], "Fixed anchor")
    windows = actual_windows(metadata, segments, anchor)
    full_window = [float(metadata["task_start_sim_s"]), float(metadata["experiment_end_sim_s"])]
    physics.capture_window_covers(manifest, full_window)
    # This also checks the pre-probe interval, not only isolated six-second pieces.
    full_native = physics.native_states(paths["native"], full_window)
    manipulator_validity = native_manipulator_validity(paths["native"], full_window)
    reports = []
    for segment, window in zip(segments, windows):
        data = physics.native_states(paths["native"], window["actual_window_sim_s"])
        reports.append({**window, **response_metrics(data, segment, window["actual_window_sim_s"], anchor, contract)})
    report = dict(kind="x650_native_commanded_position_steps_audit", schema_version=1,
        source_run=folder.name, fixed_anchor_world_enu_m=anchor.tolist(),
        nominal_probe_window_sim_s=[metadata["task_start_sim_s"]+20., metadata["task_start_sim_s"]+56.],
        primary_hover_window_sim_s=[metadata["task_start_sim_s"], metadata["task_end_sim_s"]],
        experiment_window_sim_s=full_window, native_full_window_max_interval_s=float(np.diff(full_native["t"]).max()),
        native_manipulator_validity=manipulator_validity,
        model_sha256=metadata["model_sha256"], source_urdf_sha256=metadata["source_urdf_sha256"],
        inputs_sha256={**{name: sha(path) for name, path in paths.items()},
                       "probe_source": sha(probe_source), "reference_source": sha(reference_source)},
        generator_sha256=sha(__file__), shared_physics_analysis_sha256=sha(PRECISION_SOURCE),
        analysis_contract=contract, segments=reports,
        all_segments_observed_2s_settling=all(row["settling"]["settled_2s_observed"] for row in reports),
        all_segments_settled_through_end=all(row["settling"]["settled_through_window_end"] for row in reports),
        actual_first_send_clock_used=True, no_fitted_phase_delay_origin_or_response_endpoints=True,
        experiment_semantics="Single-axis position-reference changes under the fixed world anchor; no external force injected",
        source_parameter_semantics="This native-only response report does not independently certify PX4 received setpoints, fusion flags or PID gains; use the ULog precision audit and command evidence alongside it",
        no_dynamics_simulated=True, optimal_pid_claimed=False)
    if output.exists():
        raise ValueError("Audit output exists; preserve evidence and choose a new directory")
    output.mkdir(parents=True)
    (output/"position_steps_audit.json").write_text(json.dumps(report, indent=2, allow_nan=False)+"\n", encoding="utf-8")
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-dir", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--probe-source", type=Path, default=DEFAULT_PROBE_SOURCE)
    parser.add_argument("--reference-source", type=Path, default=DEFAULT_REFERENCE_SOURCE)
    args = parser.parse_args()
    report = analyze(args.run_dir, args.output, probe_source=args.probe_source, reference_source=args.reference_source)
    print(json.dumps({key: value for key, value in report.items() if key != "segments"}, indent=2))


if __name__ == "__main__":
    main()
