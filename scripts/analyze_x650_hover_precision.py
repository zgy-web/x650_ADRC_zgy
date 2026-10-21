#!/usr/bin/env python3
"""Independent fixed-world hover physics/velocity audit; no fitted origin or time."""
from __future__ import annotations

import argparse
import csv
import hashlib
import importlib.util
import json
from pathlib import Path

import numpy as np


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_REFERENCE_SOURCE = ROOT/"simulations/x650_wholebody_mpc_v1/hover_reference.py"
PINNED_PX4 = "d6f12ad1c4f70ad3230afd7d86e971421e02fef4"


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def capture_window_covers(manifest, measurement_window):
    """A frozen capture may include later probes; primary metrics stay fixed."""
    captured, measured = (np.asarray(value, float) for value in
                         (manifest.get("task_window_sim_s"), measurement_window))
    if (captured.shape != (2,) or measured.shape != (2,) or not np.isfinite(captured).all()
            or not np.isfinite(measured).all() or captured[1] <= captured[0] or measured[1] <= measured[0]
            or captured[0] > measured[0]+1e-8 or captured[1] < measured[1]-1e-8):
        raise ValueError("Completed native capture does not cover the primary measurement window")
    return captured.tolist()


def metrics(t, error, unit):
    t, error = np.asarray(t, float), np.asarray(error, float)
    if (len(t) < 2 or error.shape != (len(t), 3) or not np.isfinite(t).all()
            or not np.isfinite(error).all() or np.any(np.diff(t) <= 0)):
        raise ValueError("Metrics require finite complete monotone three-axis samples")
    integrate = getattr(np, "trapezoid", None) or np.trapz
    duration = t[-1]-t[0]
    mean = integrate(error, t, axis=0)/duration
    norm, horizontal = np.linalg.norm(error, axis=1), np.linalg.norm(error[:, :2], axis=1)
    return dict(unit=unit, sample_count=len(t), duration_s=float(duration),
        rmse_3d=float(np.sqrt(integrate(norm**2, t)/duration)),
        rmse_xy=float(np.sqrt(integrate(horizontal**2, t)/duration)),
        rmse_z=float(np.sqrt(integrate(error[:, 2]**2, t)/duration)),
        axis_rmse=np.sqrt(integrate(error**2, t, axis=0)/duration).tolist(),
        peak_3d=float(norm.max()), peak_xy=float(horizontal.max()), peak_abs_z=float(abs(error[:, 2]).max()),
        mean_vector=mean.tolist(), mean_vector_norm=float(np.linalg.norm(mean)),
        centered_rms_3d=float(np.sqrt(integrate(np.sum((error-mean)**2, axis=1), t)/duration)),
        sample_p95_3d=float(np.percentile(norm, 95)),
        coordinate_system="world ENU", no_offset_or_phase_fit=True)


def native_states(path, window):
    with Path(path).open(newline="", encoding="utf-8") as stream:
        rows = [r for r in csv.DictReader(stream) if window[0]-1e-9 <= float(r["sim_s"]) <= window[1]+1e-9]
    if len(rows) < 2 or any(r["valid_base"] != "1" for r in rows):
        raise ValueError("Independent native base state is missing or invalid")
    def array(names):
        a = np.asarray([[float(r[name]) for name in names] for r in rows])
        if not np.isfinite(a).all():
            raise ValueError("Nonfinite native state: "+str(names))
        return a
    t = array(["sim_s"]).ravel()
    if (t[0] > window[0]+.008+1e-9 or t[-1] < window[1]-.008-1e-9
            or np.any(np.diff(t) <= 0) or np.diff(t).max() > .008+1e-9):
        raise ValueError("Native window lacks complete <=8ms physical coverage")
    quaternion = array(["base_q"+axis for axis in "wxyz"])
    norms = np.linalg.norm(quaternion, axis=1)
    if np.any(abs(norms-1) > 1e-5):
        raise ValueError("Native body orientation is not unit length")
    quaternion /= norms[:, None]
    tilt = np.degrees(np.arccos(np.clip(1-2*(quaternion[:, 1]**2+quaternion[:, 2]**2), -1, 1)))
    return dict(t=t, position=array(["base_p"+axis for axis in "xyz"]),
                velocity=array(["base_v"+axis for axis in "xyz"]), tilt=tilt)


def verify_reference(document, source):
    if (document.get("kind") != "x650_fixed_world_hover_reference" or document.get("schema_version") != 1
            or document.get("profile") != "fixed_world_position_v1" or document.get("target_frozen") is not True
            or document.get("truth_feedback_in_setpoint") is not False
            or document.get("fit_estimator_translation") is not False
            or document.get("coordinate_frame") != 1 or document.get("type_mask") != 2552
            or document.get("px4_source_contract_commit") != PINNED_PX4
            or document.get("source_sha256") != sha(source)):
        raise ValueError("Fixed native-position reference/source contract failed")
    target, local = (np.asarray(document[key], float) for key in ("target_world_enu_m", "target_local_ned_m"))
    if target.shape != (3,) or local.shape != (3,) or not np.isfinite([target, local]).all():
        raise ValueError("Reference needs finite fixed world/local targets")
    if document.get("frame_mode") not in {"raw_ev_world", "global_wgs84"}:
        raise ValueError("Unknown reference frame mode")
    return target


def dataset(ulog, name, optional=False):
    found = [d.data for d in ulog.data_list if d.name == name and d.multi_id == 0]
    if optional and not found:
        return None
    if len(found) != 1:
        raise ValueError("Need exactly one ULog "+name+"[0]")
    return found[0]


def fields(data, names):
    if any(name not in data for name in names):
        raise ValueError("Required ULog fields missing: "+str(names))
    return np.column_stack([np.asarray(data[name], float) for name in names])


def stamp(data):
    t = np.asarray(data["timestamp_sample" if "timestamp_sample" in data else "timestamp"], float)*1e-6
    if len(t) < 2 or not np.isfinite(t).all() or np.any(np.diff(t) < 0) or t.max() > 1e7:
        raise ValueError("Need raw monotone ULog HRT microseconds, not DDS epoch timestamps")
    return t


def resample(t, values, query, max_gap, hold=False):
    t, values, query = np.asarray(t, float), np.asarray(values, float), np.asarray(query, float)
    if (len(t) < 2 or t.ndim != 1 or values.ndim != 2 or len(values) != len(t)
            or not np.isfinite(t).all() or np.any(np.diff(t) < 0) or query.ndim != 1
            or not np.isfinite(query).all() or len(query) < 2 or np.any(np.diff(query) <= 0)):
        raise ValueError("Invalid source/query shape or monotone clocks")
    keep = np.r_[np.diff(t) != 0, True]
    t, values = t[keep], values[keep]
    if len(t) < 2 or query[0] < t[0]-1e-9 or query[-1] > t[-1]+1e-9:
        raise ValueError("ULog source does not cover the complete native window")
    q = np.clip(query, t[0], t[-1])
    left = np.clip(np.searchsorted(t, q, side="right")-1, 0, len(t)-1)
    if hold:
        if (q-t[left]).max() > max_gap+1e-9:
            raise ValueError("Setpoint/state-flag hold spans a missing-data gap")
        result = values[left]
    else:
        right = np.minimum(left+1, len(t)-1)
        right[abs(q-t[left]) < 1e-9] = left[abs(q-t[left]) < 1e-9]
        gap = t[right]-t[left]
        if gap.max() > max_gap+1e-9:
            raise ValueError("ULog interpolation spans a missing-data gap")
        alpha = np.divide(q-t[left], gap, out=np.zeros_like(q), where=gap > 0)
        result = values[left]+(values[right]-values[left])*alpha[:, None]
    if not np.isfinite(result).all():
        raise ValueError("Nonfinite ULog values in native measurement window")
    return result


def effective_parameters(ulog, window):
    current = dict(ulog.initial_parameters)
    changes = sorted(getattr(ulog, "changed_parameters", []), key=lambda row: row[0])
    for time_us, name, value in changes:
        if time_us*1e-6 <= window[0]+1e-9:
            current[name] = value
    during = [(float(t)*1e-6, name, value) for t, name, value in changes
              if window[0]+1e-9 < t*1e-6 <= window[1]+1e-9 and name.startswith(("MC_", "MPC_", "EKF2_"))]
    if during:
        raise ValueError("Control/estimation parameters changed inside the measurement window: "+str(during))
    selected = {name: float(value) for name, value in current.items() if name.startswith(("MC_", "MPC_", "EKF2_"))}
    return dict(source="ULog initial parameters plus every parameter change through window start; no after-restore substitution",
                parameters_at_measurement_start=selected, parameter_changes_in_window=during)


def origin_and_fusion_audit(ulog, reference, t):
    lpos = dataset(ulog, "vehicle_local_position")
    lt, bound = stamp(lpos), reference["px4_reference_at_bind"]
    mask = (lt >= t[0]-1e-9)&(lt <= t[-1]+1e-9)
    if not mask.any():
        raise ValueError("Missing local-position origin records in window")
    for key, tolerance in (("ref_lat", 1e-11), ("ref_lon", 1e-11), ("ref_alt", 1e-6)):
        if key not in lpos or np.max(abs(np.asarray(lpos[key])[mask]-bound[key])) > tolerance:
            raise ValueError("Fixed-reference origin changed: "+key)
    for key in ("xy_valid", "z_valid", "xy_global", "z_global", "xy_reset_counter", "z_reset_counter", "heading_reset_counter"):
        desired = True if key in ("xy_valid", "z_valid") else bound[key]
        if key not in lpos or np.any(np.asarray(lpos[key])[mask] != desired):
            raise ValueError("Invalid local-position or reset/origin change: "+key)
    origin_names = ("ref_lat", "ref_lon", "ref_alt", "xy_valid", "z_valid", "xy_global", "z_global",
                    "xy_reset_counter", "z_reset_counter", "heading_reset_counter")
    held = resample(lt, fields(lpos, origin_names), t, max_gap=.024, hold=True)
    for index, key in enumerate(origin_names):
        desired = True if key in ("xy_valid", "z_valid") else bound[key]
        tolerance = {"ref_lat": 1e-11, "ref_lon": 1e-11, "ref_alt": 1e-6}.get(key, 0)
        if np.any(abs(held[:, index]-desired) > tolerance):
            raise ValueError("Invalid held local-position origin/reset state: "+key)
    output = dict(origin_and_reset_stable=True, origin_records_cover_entire_window_with_max_hold_s=.024,
                  ref_timestamp="not used as clock or equality gate; DDS may translate it")
    if reference["frame_mode"] == "raw_ev_world":
        status = dataset(ulog, "estimator_status_flags")
        names = ("cs_ev_pos", "cs_ev_vel", "cs_ev_hgt", "cs_gnss_pos", "cs_gps_hgt", "cs_baro_hgt")
        st, raw_values = stamp(status), fields(status, names)
        values = resample(st, raw_values, t, max_gap=1.5, hold=True)
        desired = np.array([1., 1., 1., 0., 0., 0.])
        in_window = (st >= t[0]-1e-9)&(st <= t[-1]+1e-9)
        if np.any(values != desired) or np.any(raw_values[in_window] != desired):
            raise ValueError("Raw-EV conversion is not justified by full-window EV/GNSS/baro fusion flags")
        output["fusion_flags_entire_window"] = dict(zip(names, desired.astype(int).tolist()))
    return output


def verify_raw_ev_parameters(parameters):
    expected = {"EKF2_EV_CTRL": 7, "EKF2_EV_NOISE_MD": 1, "EKF2_GPS_CTRL": 0,
                "EKF2_BARO_CTRL": 0, "EKF2_HGT_REF": 3, "EKF2_EV_DELAY": 0,
                **{"EKF2_"+sensor+"_POS_"+axis: 0 for sensor in ("EV", "IMU") for axis in "XYZ"}}
    effective = parameters["parameters_at_measurement_start"]
    if any(name not in effective or abs(effective[name]-value) > 1e-9 for name, value in expected.items()):
        raise ValueError("Effective ULog parameters do not support raw-EV body-origin conversion")
    return expected


def full_experiment_ulog_guard(ulog, native, reference, window):
    """Audit estimation and gains throughout hover plus optional step response.

    Position-setpoint constancy is deliberately absent: only the primary hover
    window uses a constant reference. Declared step targets are audited separately.
    """
    state_gate = origin_and_fusion_audit(ulog, reference, native["t"])
    parameters = effective_parameters(ulog, window)
    required = verify_raw_ev_parameters(parameters) if reference["frame_mode"] == "raw_ev_world" else {}
    return dict(verified=True, analysis_window_sim_s=list(window),
        scope="Entire declared experiment, including all appended position-reference steps",
        native_sample_window_sim_s=[float(native["t"][0]), float(native["t"][-1])],
        native_sample_count=len(native["t"]), native_max_interval_s=float(np.diff(native["t"]).max()),
        reference_state_gate=state_gate, effective_parameters=parameters,
        required_raw_ev_body_origin_parameters=required,
        control_estimation_parameters_unchanged_entire_window=True,
        position_setpoint_constancy_checked_here=False,
        constant_target_check_scope="Primary stationary-hover window only; appended steps are intentionally changing targets",
        step_received_target_audit="Separate from this guard; native response plus sent-target evidence alone does not certify PX4 received every step")


def ulog_chain(ulog, native, reference, reference_source):
    if reference["frame_mode"] != "raw_ev_world":
        raise ValueError("Precision ULog conversion currently requires explicit raw_ev_world; use --native-only for global_wgs84")
    spec = importlib.util.spec_from_file_location("x650_precision_reference", reference_source)
    converter = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(converter)
    to_position, to_velocity = converter.local_positions_to_world, converter.local_velocities_to_world
    t, target = native["t"], np.asarray(reference["target_world_enu_m"])
    state_gate = origin_and_fusion_audit(ulog, reference, t)
    parameters = effective_parameters(ulog, (t[0], t[-1]))
    verify_raw_ev_parameters(parameters)
    lpos, sp, truth = (dataset(ulog, name) for name in ("vehicle_local_position", "vehicle_local_position_setpoint", "vehicle_local_position_groundtruth"))
    estimate_p = np.asarray(to_position(resample(stamp(lpos), fields(lpos, ("x", "y", "z")), t, .024), reference))
    estimate_v = np.asarray(to_velocity(resample(stamp(lpos), fields(lpos, ("vx", "vy", "vz")), t, .024), reference))
    sp_p = np.asarray(to_position(resample(stamp(sp), fields(sp, ("x", "y", "z")), t, .3, hold=True), reference))
    sp_v = np.asarray(to_velocity(resample(stamp(sp), fields(sp, ("vx", "vy", "vz")), t, .3, hold=True), reference))
    # GZBridge local groundtruth is world NED, never the estimator's rebased local origin.
    bridge_v_ned = resample(stamp(truth), fields(truth, ("vx", "vy", "vz")), t, .06)
    bridge_v = bridge_v_ned[:, [1, 0, 2]]*np.array([1., 1., -1.])
    positions = {"native_minus_estimate": metrics(t, native["position"]-estimate_p, "m"),
                 "estimate_minus_actual_position_setpoint": metrics(t, estimate_p-sp_p, "m"),
                 "actual_position_setpoint_minus_declared_world_target": metrics(t, sp_p-target, "m")}
    velocities = {"native_minus_estimate": metrics(t, native["velocity"]-estimate_v, "m/s"),
                  "estimate_minus_internal_velocity_setpoint": metrics(t, estimate_v-sp_v, "m/s"),
                  "native_minus_internal_velocity_setpoint": metrics(t, native["velocity"]-sp_v, "m/s"),
                  "ulog_bridge_groundtruth_minus_native": metrics(t, bridge_v-native["velocity"], "m/s")}
    command_match = positions["actual_position_setpoint_minus_declared_world_target"]["peak_3d"] <= .002
    return dict(reference_state_gate=state_gate, effective_parameters=parameters,
                position_world_enu=positions, velocity_world_enu=velocities,
                fixed_world_target_verified_within_2mm=command_match,
                time_alignment="Raw HRT timestamp_sample for state, timestamp for setpoint; scale 1e-6, shift 0; no delay/phase fit",
                resampling="Linear bounded state interpolation; causal logged setpoint hold; no position differentiation used for velocity")


def analyze(run_dir, output, *, native_path=None, metadata_path=None, reference_path=None,
            reference_source=DEFAULT_REFERENCE_SOURCE, ulog_path=None, native_only=False):
    folder, output = Path(run_dir).resolve(strict=True), Path(output).resolve()
    paths = {"native": Path(native_path or folder/"native_trace.csv").resolve(strict=True),
             "metadata": Path(metadata_path or folder/"controller_metadata.json").resolve(strict=True),
             "reference": Path(reference_path or folder/"hover_reference.json").resolve(strict=True)}
    reference_source = Path(reference_source).resolve(strict=True)
    metadata, reference = (json.loads(paths[key].read_text(encoding="utf-8")) for key in ("metadata", "reference"))
    if metadata.get("complete") is not True or metadata.get("landed_and_disarmed") is not True:
        raise ValueError("Precision audit requires completed and recovered flight metadata")
    target = verify_reference(reference, reference_source)
    prior_target = metadata.get("hover_metrics", {}).get("fixed_target_enu_m")
    if prior_target is None or not np.allclose(target, prior_target, rtol=0, atol=1e-9):
        raise ValueError("Reference target differs from the recorded flight's fixed task target")
    manifest_path = paths["native"].with_name(paths["native"].stem+"_manifest.json")
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    if (manifest.get("complete") is not True or manifest.get("sha256") != sha(paths["native"])
            or manifest.get("model_sha256") != metadata.get("model_sha256")
            or manifest.get("source_urdf_sha256") != metadata.get("source_urdf_sha256")):
        raise ValueError("Independent native capture hash/completion/model identity mismatch")
    window = [float(metadata[key]) for key in ("task_start_sim_s", "task_end_sim_s")]
    captured_window = capture_window_covers(manifest, window)
    native = native_states(paths["native"], window)
    report = dict(kind="x650_fixed_world_hover_precision_audit", source_run=folder.name,
        analysis_window_sim_s=window, frozen_capture_window_sim_s=captured_window,
        target_world_enu_m=target.tolist(), reference=reference,
        model_sha256=metadata["model_sha256"], source_urdf_sha256=metadata["source_urdf_sha256"],
        inputs_sha256={**{key: sha(path) for key, path in paths.items()}, "native_manifest": sha(manifest_path),
                       "reference_source": sha(reference_source)}, generator_sha256=sha(__file__),
        native_sampling_max_interval_s=float(np.diff(native["t"]).max()),
        position_native_world_enu=metrics(native["t"], native["position"]-target, "m"),
        native_motion_velocity_world_enu=metrics(native["t"], native["velocity"], "m/s"),
        native_velocity_semantics="Physical world motion relative to zero hover velocity; not tracking error against PX4's internal velocity demand",
        tilt_peak_deg=float(native["tilt"].max()), no_fit_of_phase_delay_origin_or_mean=True,
        native_only=native_only, optimal_pid_claimed=False, new_dynamics_simulated=False)
    if not native_only:
        path = Path(ulog_path or folder/"px4.ulg").resolve(strict=True)
        from pyulog import ULog
        report["inputs_sha256"]["ulog"] = sha(path)
        ulog = ULog(str(path))
        report["ulog_control_chain"] = ulog_chain(ulog, native, reference, reference_source)
        full_window = [window[0], float(metadata.get("experiment_end_sim_s", window[1]))]
        if not np.isfinite(full_window).all() or full_window[1] < window[1]-1e-8:
            raise ValueError("Full experiment window cannot end before primary hover")
        capture_window_covers(manifest, full_window)
        full_native = native if full_window == window else native_states(paths["native"], full_window)
        report["full_experiment_ulog_guard"] = full_experiment_ulog_guard(ulog, full_native, reference, full_window)
        effective = report["ulog_control_chain"]["effective_parameters"]["parameters_at_measurement_start"]
        declared = metadata.get("actual_pid_parameters", {})
        if declared:
            missing = [name for name in declared if name not in effective]
            if missing:
                raise ValueError("Measurement-period ULog lacks declared MC/MPC parameter keys: "+str(missing))
            measured = {name: effective[name] for name in declared}
            mismatch = {name: {"metadata": declared[name], "ulog": measured[name]} for name in declared
                        if not np.isclose(float(declared[name]), measured[name], rtol=2e-6, atol=1e-7)}
            report["measurement_period_pid"] = dict(parameters=measured,
                count=len(measured), matches_controller_metadata=not mismatch, metadata_mismatches=mismatch,
                sha256=hashlib.sha256(json.dumps(measured, sort_keys=True, separators=(",", ":")).encode()).hexdigest(),
                source="Actual effective ULog values during measurement; metadata/initial/after-restore values are never silently substituted")
    if output.exists():
        raise ValueError("Audit output exists; preserve evidence and choose a new directory")
    output.mkdir(parents=True)
    (output/"precision_audit.json").write_text(json.dumps(report, indent=2, allow_nan=False)+"\n", encoding="utf-8")
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-dir", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--native", type=Path)
    parser.add_argument("--metadata", type=Path)
    parser.add_argument("--reference", type=Path)
    parser.add_argument("--reference-source", type=Path, default=DEFAULT_REFERENCE_SOURCE)
    parser.add_argument("--ulog", type=Path)
    parser.add_argument("--native-only", action="store_true")
    args = parser.parse_args()
    report = analyze(args.run_dir, args.output, native_path=args.native, metadata_path=args.metadata,
                     reference_path=args.reference, reference_source=args.reference_source,
                     ulog_path=args.ulog, native_only=args.native_only)
    print(json.dumps({k: v for k, v in report.items() if k not in ("reference", "ulog_control_chain")}, indent=2))


if __name__ == "__main__":
    main()
