#!/usr/bin/env python3
"""Read-only native/PX4 hover decomposition, without fitted time/origin offsets.

Requires existing numpy and pyulog. Does not connect to a simulator or modify
parameters. Missing source-backed world datum produces explicit partial output,
never a best-fit translation. Run with --help for the optional source evidence.
"""
from __future__ import annotations
import argparse
import csv
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import xml.etree.ElementTree as ET
import numpy as np

PINNED_PX4 = "d6f12ad1c4f70ad3230afd7d86e971421e02fef4"
BASE_URL = "https://raw.githubusercontent.com/PX4/PX4-Autopilot/"+PINNED_PX4+"/"
SOURCE_CONTRACT = {
    "clock_and_enu_to_ned": BASE_URL+"src/modules/simulation/gz_bridge/GZBridge.cpp",
    "hrt_clock": BASE_URL+"platforms/posix/src/px4/common/drv_hrt.cpp",
    "lpos_sample_time_body_origin": BASE_URL+"src/modules/ekf2/EKF2.cpp",
    "local_map_projection": BASE_URL+"src/lib/geo/geo.cpp",
    "output_imu_offset": BASE_URL+"src/modules/ekf2/EKF/output_predictor/output_predictor.h",
}


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def transform_ned_enu(x):
    return np.asarray(x, dtype=float)[..., [1, 0, 2]]*np.array([1., 1., -1.])


def metrics(t, error):
    t, e = np.asarray(t, float), np.asarray(error, float)
    if t.ndim != 1 or e.shape != (len(t), 3) or len(t) < 2 or np.any(np.diff(t) <= 0) or not np.isfinite(e).all():
        raise ValueError("Metrics require complete finite, increasing three-axis samples")
    integrate = getattr(np, "trapezoid", None) or np.trapz
    duration = float(t[-1]-t[0])
    mean = integrate(e, t, axis=0)/duration
    centered = e-mean
    norm = np.linalg.norm(e, axis=1)
    early, late = t <= t[0]+2., t >= t[-1]-2.
    # A slope is a descriptive drift metric, never a phase/origin correction.
    tc = t-.5*(t[0]+t[-1])
    trend = integrate(tc[:, None]*e, t, axis=0)/integrate(tc**2, t)
    return dict(sample_count=len(t), duration_s=duration, mean_vector_m=mean.tolist(),
        mean_bias_norm_m=float(np.linalg.norm(mean)),
        rmse_3d_m=float(np.sqrt(integrate(norm**2, t)/duration)),
        centered_rms_3d_m=float(np.sqrt(integrate(np.sum(centered**2, axis=1), t)/duration)),
        axis_rmse_m=np.sqrt(integrate(e**2, t, axis=0)/duration).tolist(),
        axis_centered_std_m=np.sqrt(integrate(centered**2, t, axis=0)/duration).tolist(),
        peak_3d_m=float(norm.max()), sample_p95_3d_m=float(np.percentile(norm, 95)),
        axis_peak_to_peak_m=np.ptp(e, axis=0).tolist(),
        last_minus_first_2s_mean_m=(e[late].mean(axis=0)-e[early].mean(axis=0)).tolist(),
        descriptive_linear_drift_m_s=trend.tolist())


def resample(t, values, query, *, max_gap, hold=False):
    """Bounded same-clock interpolation; setpoints use causal zero-order hold."""
    t, values, q = np.asarray(t, float), np.asarray(values, float), np.asarray(query, float)
    if len(t) < 2 or np.any(np.diff(t) < 0) or not np.isfinite(t).all():
        raise ValueError("Missing/nonmonotone source time series")
    # Multiple publications of one measurement: retain the last actual record.
    keep = np.r_[np.diff(t) != 0, True]
    t, values = t[keep], values[keep]
    if len(t) < 2 or q.min() < t[0]-1e-9 or q.max() > t[-1]+1e-9:
        raise ValueError("Analysis window extends outside source coverage; no extrapolation")
    q = np.clip(q, t[0], t[-1])
    left = np.clip(np.searchsorted(t, q, side="right")-1, 0, len(t)-1)
    if hold:
        if np.max(q-t[left]) > max_gap+1e-9:
            raise ValueError("Causal setpoint hold exceeds permitted publication gap")
        output = values[left]
    else:
        right = np.minimum(left+1, len(t)-1)
        exact = np.abs(q-t[left]) < 1e-9
        right[exact] = left[exact]
        gap = t[right]-t[left]
        if gap.max() > max_gap+1e-9:
            raise ValueError("Source interpolation bridges a missing-data gap")
        fraction = np.divide(q-t[left], gap, out=np.zeros_like(q), where=gap > 0)
        output = values[left]+(values[right]-values[left])*fraction.reshape((-1,)+(1,)*(values.ndim-1))
    if not np.isfinite(output).all():
        raise ValueError("Nonfinite source values inside analysis window")
    return output


def dataset(ulog, name, *, optional=False):
    found = [d for d in ulog.data_list if d.name == name and d.multi_id == 0]
    if len(found) != 1:
        if optional and not found:
            return None
        raise ValueError(f"Need exactly one {name}[0], found {len(found)}")
    return found[0].data


def fields(data, names):
    missing = [key for key in names if key not in data]
    if missing:
        raise ValueError("ULog fields missing: "+str(missing))
    return np.column_stack([np.asarray(data[key], float) for key in names])


def sample_time(data):
    key = "timestamp_sample" if "timestamp_sample" in data else "timestamp"
    values = np.asarray(data[key], float)*1e-6
    if not np.isfinite(values).all() or np.max(values) > 1e7 or np.any(np.diff(values) < 0):
        raise ValueError("Expected raw ULog HRT microseconds, not DDS/ROS epoch timestamps")
    return values


def vector_summary(t, values, unit):
    t, values = np.asarray(t, float), np.asarray(values, float)
    if len(t) < 2 or not np.isfinite(values).all():
        raise ValueError("Need finite complete samples for "+unit+" summary")
    integrate = getattr(np, "trapezoid", None) or np.trapz
    duration = t[-1]-t[0]
    mean = integrate(values, t, axis=0)/duration
    return {"unit": unit, "sample_count": len(t), "mean": np.atleast_1d(mean).tolist(),
        "rms": np.atleast_1d(np.sqrt(integrate(values**2, t, axis=0)/duration)).tolist(),
        "min": np.atleast_1d(values.min(axis=0)).tolist(), "max": np.atleast_1d(values.max(axis=0)).tolist(),
        "peak_to_peak": np.atleast_1d(np.ptp(values, axis=0)).tolist()}


def quaternion_samples(data, names, query, *, hold=False):
    q = fields(data, names)
    norm = np.linalg.norm(q, axis=1)
    if np.any(norm < 1e-9):
        raise ValueError("Invalid quaternion record")
    q = q/norm[:, None]
    for i in range(1, len(q)):
        if q[i-1]@q[i] < 0:
            q[i] *= -1
    output = resample(sample_time(data), q, query, max_gap=.3 if hold else .06, hold=hold)
    return output/np.linalg.norm(output, axis=1)[:, None]


def add_inner_loop_diagnostics(result, ulog, t):
    """Optional logged inner-loop summaries cannot suppress position results."""
    try:
        actual, gt, desired = (dataset(ulog, name, optional=True) for name in
            ("vehicle_attitude", "vehicle_attitude_groundtruth", "vehicle_attitude_setpoint"))
        qa = quaternion_samples(actual, [f"q[{i}]" for i in range(4)], t) if actual is not None else None
        qg = quaternion_samples(gt, [f"q[{i}]" for i in range(4)], t) if gt is not None else None
        qd = quaternion_samples(desired, [f"q_d[{i}]" for i in range(4)], t, hold=True) if desired is not None else None
        attitude = {}
        for name, a, b in (("estimate_minus_truth", qa, qg), ("estimate_minus_setpoint", qa, qd), ("truth_minus_setpoint", qg, qd)):
            if a is not None and b is not None:
                angle = 2*np.rad2deg(np.arccos(np.clip(abs(np.sum(a*b, axis=1)), 0., 1.)))
                attitude[name] = vector_summary(t, angle, "deg: unsigned rotation distance")
        result["attitude_tracking"] = attitude
    except (ValueError, KeyError) as exc:
        result["uncertainties"].append("Attitude diagnostic unavailable: "+str(exc))
    try:
        actual = dataset(ulog, "vehicle_angular_velocity", optional=True)
        desired = dataset(ulog, "vehicle_rates_setpoint", optional=True)
        if actual is not None and desired is not None:
            w = resample(sample_time(actual), fields(actual, [f"xyz[{i}]" for i in range(3)]), t, max_gap=.06)
            wd = resample(sample_time(desired), fields(desired, ("roll", "pitch", "yaw")), t, max_gap=.3, hold=True)
            result["body_rate_tracking_frd"] = {"actual": vector_summary(t, w, "rad/s"),
                "setpoint": vector_summary(t, wd, "rad/s"), "actual_minus_setpoint": vector_summary(t, w-wd, "rad/s")}
    except (ValueError, KeyError) as exc:
        result["uncertainties"].append("Body-rate diagnostic unavailable: "+str(exc))
    try:
        motors = dataset(ulog, "actuator_motors", optional=True)
        if motors is not None:
            mt = np.asarray(motors["timestamp"], float)*1e-6
            mask = (mt >= t[0])&(mt <= t[-1])
            values = fields(motors, [f"control[{i}]" for i in range(4)])[mask]
            result["motor_commands_0_to_3"] = vector_summary(mt[mask], values, "normalized actuator command; not measured rotor thrust")
    except (ValueError, KeyError) as exc:
        result["uncertainties"].append("Motor diagnostic unavailable: "+str(exc))


def load_native(path):
    with Path(path).open(newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    t = np.array([float(row["sim_s"]) for row in rows])
    p = np.array([[float(row["base_p"+a]) for a in "xyz"] for row in rows])
    valid = np.array([int(row["valid_base"]) == 1 for row in rows])
    if len(t) < 2 or np.any(np.diff(t) <= 0) or not np.isfinite(t).all():
        raise ValueError("Native observer must contain monotone same-physics-step time")
    return t, p, valid


def read_world(path):
    root = ET.parse(path).getroot()
    worlds = [root] if root.tag == "world" else root.findall("world")
    if len(worlds) != 1:
        raise ValueError("World source must explicitly contain exactly one world")
    sc = worlds[0].find("spherical_coordinates")
    if sc is None:
        raise ValueError("World has no explicit spherical_coordinates datum; defaults are not guessed")
    required = ("latitude_deg", "longitude_deg", "elevation", "heading_deg")
    if any(sc.findtext(key) is None for key in required):
        raise ValueError("World datum requires explicit lat/lon/elevation/heading")
    values = {key: float(sc.findtext(key)) for key in required}
    if sc.findtext("surface_model", "EARTH_WGS84") != "EARTH_WGS84" or sc.findtext("world_frame_orientation", "ENU") != "ENU":
        raise ValueError("Only explicit WGS84 / ENU world datum is supported")
    if abs(values["heading_deg"]) > 1e-10:
        raise ValueError("Nonzero world heading needs an independently checked transform; refusing to guess")
    if not np.isfinite(list(values.values())).all():
        raise ValueError("World datum is nonfinite")
    return {"path": str(Path(path).resolve()), "sha256": sha(path), **values}


def map_reproject(north, east, ref_lat_deg, ref_lon_deg):
    """Pinned PX4 MapProjection::reproject, radius 6371000 m."""
    x, y = np.asarray(north)/6371000., np.asarray(east)/6371000.
    lat0, lon0 = np.deg2rad(ref_lat_deg), np.deg2rad(ref_lon_deg)
    c = np.hypot(x, y)
    sinc = np.divide(np.sin(c), c, out=np.ones_like(c), where=c != 0)
    lat = np.arcsin(np.clip(np.cos(c)*np.sin(lat0)+x*sinc*np.cos(lat0), -1., 1.))
    lon = lon0+np.arctan2(y*sinc, np.cos(lat0)*np.cos(c)-x*np.sin(lat0)*sinc)
    return np.rad2deg(lat), np.rad2deg(lon)


def ecef(lat_deg, lon_deg, altitude):
    # gz-math7 WGS84 ellA/ellB, matching SphericalCoordinates implementation.
    a, b = 6378137., 6356752.314245
    lat, lon = np.deg2rad(lat_deg), np.deg2rad(lon_deg)
    e2 = 1-(b/a)**2
    radius = a/np.sqrt(1-e2*np.sin(lat)**2)
    return np.stack(((radius+altitude)*np.cos(lat)*np.cos(lon),
                     (radius+altitude)*np.cos(lat)*np.sin(lon),
                     (radius*(1-e2)+altitude)*np.sin(lat)), axis=-1)


def geodetic_to_world(lat, lon, alt, world):
    lat0, lon0 = np.deg2rad([world["latitude_deg"], world["longitude_deg"]])
    rotation = np.array([[-np.sin(lon0), np.cos(lon0), 0.],
        [-np.sin(lat0)*np.cos(lon0), -np.sin(lat0)*np.sin(lon0), np.cos(lat0)],
        [np.cos(lat0)*np.cos(lon0), np.cos(lat0)*np.sin(lon0), np.sin(lat0)]])
    origin = ecef(world["latitude_deg"], world["longitude_deg"], world["elevation"])
    return (ecef(lat, lon, alt)-origin)@rotation.T


def local_to_world(p_ned, reference, world):
    latitude, longitude = map_reproject(p_ned[:, 0], p_ned[:, 1], reference[:, 0], reference[:, 1])
    return geodetic_to_world(latitude, longitude, reference[:, 2]-p_ned[:, 2], world)


def source_evidence(path):
    if path is None:
        return {"runtime_commit_verified": False, "pinned_contract_commit": PINNED_PX4}
    path = Path(path).resolve()
    commit = subprocess.check_output(["git", "-C", str(path), "rev-parse", "HEAD"], text=True).strip()
    tracked = ("src/modules/simulation/gz_bridge/GZBridge.cpp", "platforms/posix/src/px4/common/drv_hrt.cpp", "src/modules/ekf2/EKF2.cpp")
    dirty = subprocess.check_output(["git", "-C", str(path), "status", "--porcelain", "--", *tracked], text=True).strip()
    return {"path": str(path), "commit": commit, "runtime_commit_verified": commit == PINNED_PX4 and not dirty,
            "relevant_source_dirty": bool(dirty), "source_sha256": {name: sha(path/name) for name in tracked}}


def sensor_evidence(path, parameters):
    result = {"model_sensor_point_verified": False,
              "imu_position_parameters_frd_m": [parameters.get("EKF2_IMU_POS_"+a) for a in "XYZ"],
              "gps_position_parameters_frd_m": [parameters.get("EKF2_GPS_POS_"+a) for a in "XYZ"]}
    if path is None:
        return result
    base = ET.parse(path).getroot().find("model/link[@name='base_link']")
    if base is None:
        raise ValueError("Actual model SDF lacks base_link")
    for name in ("imu_sensor", "navsat_sensor"):
        sensor = base.find("sensor[@name='"+name+"']")
        if sensor is None:
            raise ValueError("Actual model lacks base_link "+name)
        pose = sensor.find("pose")
        values = np.zeros(6) if pose is None else np.fromstring(pose.text or "", sep=" ")
        if values.shape != (6,) or not np.isfinite(values).all() or np.any(abs(values) > 1e-12) or (pose is not None and pose.get("relative_to")):
            raise ValueError("Nonzero/relative sensor pose requires an explicit body-point transform")
        result[name+"_pose_flu"] = values.tolist()
    result.update(path=str(Path(path).resolve()), sha256=sha(path), model_sensor_point_verified=True)
    return result


def analyze(ulog, native_path, metadata, *, window=None, world_path=None, model_path=None, px4_source=None):
    t_native, p_native, valid = load_native(native_path)
    bounds = tuple(window or (metadata["task_start_sim_s"], metadata["task_end_sim_s"]))
    if len(bounds) != 2 or not np.isfinite(bounds).all() or bounds[1] <= bounds[0]:
        raise ValueError("Need an explicit complete finite analysis window")
    inside = (t_native >= bounds[0]-1e-9)&(t_native <= bounds[1]+1e-9)
    t, truth = t_native[inside], p_native[inside]
    if len(t) < 2 or t[0] > bounds[0]+.008 or t[-1] < bounds[1]-.008 or np.max(np.diff(t)) > .008+1e-9 or not valid[inside].all() or not np.isfinite(truth).all():
        raise ValueError("Native measurement window lacks full valid <=8ms physical sampling")
    lpos, gt, sp = (dataset(ulog, name) for name in ("vehicle_local_position", "vehicle_local_position_groundtruth", "vehicle_local_position_setpoint"))
    lt, gtt, spt = sample_time(lpos), sample_time(gt), sample_time(sp)
    estimate_ned = resample(lt, fields(lpos, ("x", "y", "z")), t, max_gap=.024)
    setpoint_ned = resample(spt, fields(sp, ("x", "y", "z")), t, max_gap=.3, hold=True)
    bridge_enu = resample(gtt, transform_ned_enu(fields(gt, ("x", "y", "z"))), t, max_gap=.06)
    bridge_error = bridge_enu-truth
    bridge = metrics(t, bridge_error)
    bridge["direct_no_shift_consistent_within_5mm"] = bridge["peak_3d_m"] <= .005
    target = metadata.get("hover_metrics", {}).get("fixed_target_enu_m")
    if target is None:
        raise ValueError("Need hover_metrics.fixed_target_enu_m; post-takeoff reference_base_enu_m is not the task target")
    target = np.asarray(target, float)
    if target.shape != (3,) or not np.isfinite(target).all():
        raise ValueError("Invalid preflight fixed task target")
    selected = {name: value for name, value in ulog.initial_parameters.items()
                if name.startswith(("EKF2_", "SIM_GPS")) or name in ("MPC_XY_P", "MPC_Z_P", "MPC_THR_HOVER")}
    result = {"kind": "x650_native_hover_control_chain_diagnostic", "analysis_window_sim_s": list(bounds),
        "time_alignment": {"ulog_clock": "raw HRT timestamp_sample for estimates / timestamp for setpoints", "native_clock": "Gazebo PostUpdate sim_s",
            "scale": 1e-6, "shift_s": 0., "phase_fitted": False, "origin_fitted": False,
            "gps_delay_subtracted_from_output": False, "contract_sources": SOURCE_CONTRACT,
            "bridge_error_enu_m": bridge},
        "px4_source": source_evidence(px4_source), "selected_initial_parameters": selected,
        "sensor_points": sensor_evidence(model_path, selected),
        "metrics": {"truth_minus_task_target_enu": metrics(t, truth-target),
                    "estimate_minus_controller_setpoint_ned": metrics(t, estimate_ned-setpoint_ned),
                    "controller_setpoint_change_from_window_start_ned": metrics(t, setpoint_ned-setpoint_ned[0]),
                    "native_displacement_from_window_start_enu": metrics(t, truth-truth[0])},
        "world_chain_available": False, "uncertainties": [],
        "interpretation": "Descriptive vector decomposition, not causal attribution. Norm/RMSE terms do not add; mean bias differs from centered jitter and drift."}
    add_inner_loop_diagnostics(result, ulog, t)
    columns = {"sim_s": t}
    for prefix, values in (("truth_world_enu_", truth), ("ulog_groundtruth_world_enu_", bridge_enu),
                           ("estimate_local_ned_", estimate_ned), ("controller_setpoint_local_ned_", setpoint_ned)):
        columns.update({prefix+a: values[:, i] for i, a in enumerate("xyz")})
    if not bridge["direct_no_shift_consistent_within_5mm"]:
        result["uncertainties"].append("ULog groundtruth/native mismatch exceeds 5mm without fitting; clock phase/model-vs-link reference point needs investigation")
    reset_names = ("xy_reset_counter", "z_reset_counter", "heading_reset_counter", "vxy_reset_counter", "vz_reset_counter")
    reset_report = {}
    for name in reset_names:
        if name in lpos:
            values = np.asarray(lpos[name])[((lt >= bounds[0])&(lt <= bounds[1]))]
            reset_report[name] = {"first": int(values[0]), "last": int(values[-1]), "changed": bool(np.any(np.diff(values.astype(int)) != 0))} if len(values) else {"missing_window": True}
    result["estimator_resets"] = reset_report
    if "timestamp_sample" in lpos:
        latency = (np.asarray(lpos["timestamp"], float)-np.asarray(lpos["timestamp_sample"], float))*1e-6
        latency = latency[(lt >= bounds[0])&(lt <= bounds[1])]
        result["lpos_publish_minus_sample_s"] = {"median": float(np.median(latency)), "max": float(np.max(latency))}
    world = None
    if world_path is not None:
        try:
            world = read_world(world_path)
        except ValueError as exc:
            result["uncertainties"].append(str(exc))
    else:
        result["uncertainties"].append("No source-backed Gazebo spherical_coordinates datum supplied; no absolute estimate/truth translation is inferred from hover data")
    if world is not None:
        result["world_datum"] = world
        reference = resample(lt, fields(lpos, ("ref_lat", "ref_lon", "ref_alt")), t, max_gap=.024, hold=True)
        if any(reset.get("changed") for reset in reset_report.values()):
            result["uncertainties"].append("Estimator reset within measurement window: full absolute chain requires piecewise reset-aware resampling")
        else:
            estimate = local_to_world(estimate_ned, reference, world)
            executed_sp = local_to_world(setpoint_ned, reference, world)
            gps_gt = dataset(ulog, "vehicle_global_position_groundtruth", optional=True)
            if gps_gt is not None:
                # Logger may record global truth at 1 Hz while the sensor runs
                # faster. Check its native timestamps against the dense native
                # observer; never invent high-rate geographic measurements.
                geo_t = sample_time(gps_gt)
                geo_mask = (geo_t >= t[0]) & (geo_t <= t[-1])
                geo_t = geo_t[geo_mask]
                geo = fields(gps_gt, ("lat", "lon", "alt"))[geo_mask]
                truth_at_geo = resample(t, truth, geo_t, max_gap=.008)
                gps_world = geodetic_to_world(geo[:, 0], geo[:, 1], geo[:, 2], world)
                result["world_datum"]["unnoised_gps_truth_check_m"] = metrics(geo_t, gps_world-truth_at_geo)
                datum_ok = result["world_datum"]["unnoised_gps_truth_check_m"]["peak_3d_m"] <= .005
            else:
                datum_ok = False
                result["uncertainties"].append("Unnoised GPS groundtruth topic absent: datum reconstruction lacks independent sensor/world check")
            offsets = [selected.get("EKF2_IMU_POS_"+a) for a in "XYZ"]
            zero_imu = all(v is not None and abs(float(v)) < 1e-12 for v in offsets)
            point_ok = zero_imu and result["sensor_points"]["model_sensor_point_verified"]
            result["world_chain_available"] = bool(datum_ok and point_ok and bridge["direct_no_shift_consistent_within_5mm"])
            if not point_ok:
                result["uncertainties"].append("Body reference point not independently verified from actual model sensors and IMU offsets; no absolute chain attribution")
            if not datum_ok:
                result["uncertainties"].append("Known world datum did not reproduce unnoised GPS truth within 5mm; transform is not accepted for attribution")
            if result["world_chain_available"]:
                parts = {"truth_minus_estimate_world_enu": truth-estimate,
                    "estimate_minus_executed_setpoint_world_enu": estimate-executed_sp,
                    "executed_setpoint_minus_task_world_enu": executed_sp-target}
                total = truth-target
                result["metrics"].update({key: metrics(t, value) for key, value in parts.items()})
                result["vector_chain_closure_max_m"] = float(np.max(np.linalg.norm(sum(parts.values())-total, axis=1)))
                for prefix, values in (("estimate_world_enu_", estimate), ("executed_setpoint_world_enu_", executed_sp)):
                    columns.update({prefix+a: values[:, i] for i, a in enumerate("xyz")})
    result["caveats"] = ["vehicle_local_position is current-output-predictor body origin, not delayed EKF state or vehicle COM",
        "110ms GPS fusion delay must not be subtracted again from current lpos timestamp_sample",
        "GZBridge adds correlated GPS noise even when SDF navsat has no explicit noise block",
        "Setpoint telemetry is downsampled; causal hold preserves logged availability, not unseen intermediate references",
        "RMSE uses the fixed preflight target; no flight-window recentering, best-fit phase, best-fit origin or delay optimization"]
    return result, columns


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-dir", type=Path, required=True)
    parser.add_argument("--ulog", type=Path)
    parser.add_argument("--native", type=Path)
    parser.add_argument("--metadata", type=Path)
    parser.add_argument("--world-sdf", type=Path, help="Actual world SDF with explicit WGS84 ENU/zero-heading spherical_coordinates")
    parser.add_argument("--model-sdf", type=Path, help="Actual composed flight-model SDF; verifies zero IMU/navsat poses at base_link")
    parser.add_argument("--px4-source", type=Path, help="Read-only checkout for pinned commit and dirty-source verification")
    parser.add_argument("--window", type=float, nargs=2)
    parser.add_argument("--output", type=Path, required=True, help="New output directory; existing directories are never overwritten")
    args = parser.parse_args()
    if args.output.exists():
        parser.error("Output already exists; preserve previous diagnostic evidence")
    try:
        from pyulog import ULog
    except ImportError:
        parser.error("Use an existing interpreter with pyulog and numpy; this analyzer does not install dependencies")
    run = args.run_dir.resolve()
    paths = {"ulog": args.ulog or run/"px4.ulg", "native": args.native or run/"native_trace.csv",
             "metadata": args.metadata or run/"controller_metadata.json"}
    metadata = json.loads(paths["metadata"].read_text(encoding="utf-8"))
    report, columns = analyze(ULog(str(paths["ulog"])), paths["native"], metadata, window=args.window,
        world_path=args.world_sdf, model_path=args.model_sdf, px4_source=args.px4_source)
    report["source_files"] = {key: {"path": str(path.resolve()), "sha256": sha(path)} for key, path in paths.items()}
    report["analyzer_sha256"] = sha(__file__)
    args.output.mkdir(parents=True)
    with (args.output/"aligned_chain.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(columns)
        writer.writerows(zip(*columns.values()))
    report["aligned_chain_sha256"] = sha(args.output/"aligned_chain.csv")
    (args.output/"diagnostic.json").write_text(json.dumps(report, indent=2, allow_nan=False)+"\n", encoding="utf-8")
    print(json.dumps({"output": str(args.output.resolve()), "world_chain_available": report["world_chain_available"],
                      "metrics": report["metrics"], "uncertainties": report["uncertainties"]}, indent=2, allow_nan=False))


if __name__ == "__main__":
    main()
