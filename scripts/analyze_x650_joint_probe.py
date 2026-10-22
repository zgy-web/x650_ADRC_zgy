#!/usr/bin/env python3
"""Audit a completed native joint probe and plot raw states, without new dynamics."""
from __future__ import annotations

import argparse
import csv
import hashlib
import json
from pathlib import Path

import numpy as np


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def read_json(path):
    return json.loads(Path(path).read_text(encoding="utf-8"))


def reference(t, duration):
    """Independent vector evaluation of run_case.joint_probe_reference, in rad."""
    t = np.asarray(t, float)
    if not np.isfinite(t).all() or duration < 16.:
        raise ValueError("Invalid joint-probe reference time/duration")
    segment = (duration-4.)/3.
    q, dq = np.zeros((len(t), 2)), np.zeros((len(t), 2))
    for i, amplitude in enumerate(([.08, 0.], [0., .08], [.06, -.06])):
        mask = (t > 2.+i*segment) & (t < 2.+(i+1)*segment)
        angle = 2*np.pi*(t[mask]-2.-i*segment)/segment
        q[mask] = np.sin(angle)[:, None]**3*np.asarray(amplitude)
        dq[mask] = (3*np.sin(angle)**2*np.cos(angle)*2*np.pi/segment)[:, None]*np.asarray(amplitude)
    return q, dq


def time_rms(t, values):
    integrate = getattr(np, "trapezoid", np.trapz)
    return np.sqrt(integrate(np.asarray(values)**2, t, axis=0)/(t[-1]-t[0]))


def cosine(actual, desired):
    denom = float(np.linalg.norm(actual)*np.linalg.norm(desired))
    return None if denom < 1e-12 else float(np.dot(actual, desired)/denom)


def array(rows, names):
    values = np.asarray([[float(row[name]) for name in names] for row in rows], float)
    if not np.isfinite(values).all():
        raise ValueError("Nonfinite native/control state: "+str(names))
    return values


def native_arrays(rows, begin, duration, anchor):
    rows = [row for row in rows if begin-1e-9 <= float(row["sim_s"]) <= begin+duration+1e-9]
    if len(rows) < 2 or any(row[k] != "1" for row in rows for k in ("valid_base", "valid_joint_state", "valid_tip")):
        raise ValueError("Joint probe requires valid independent native physics samples")
    t = array(rows, ["sim_s"]).ravel()-begin
    if (t[0] > .008+1e-9 or t[-1] < duration-.008-1e-9 or np.any(np.diff(t) <= 0)
            or np.diff(t).max() > .008+1e-9):
        raise ValueError("Native joint-probe window is incomplete, sparse or nonmonotone")
    quat = array(rows, ["base_q"+a for a in "wxyz"])
    norms = np.linalg.norm(quat, axis=1)
    if np.any(abs(norms-1) > 1e-5):
        raise ValueError("Native base quaternion is not unit length")
    quat /= norms[:, None]
    tilt = np.degrees(np.arccos(np.clip(1-2*(quat[:, 1]**2+quat[:, 2]**2), -1, 1)))
    position = array(rows, ["base_p"+a for a in "xyz"])
    return dict(t=t, q=array(rows, ["arm_joint_1_q", "arm_joint_2_q"]),
                dq=array(rows, ["arm_joint_1_qd", "arm_joint_2_qd"]), tilt=tilt,
                position=position, quaternion=quat, tip=array(rows, ["tip_"+a for a in "xyz"]),
                body_error=position-np.asarray(anchor))


def native_tip_audit(data, profile, *, expected_profile_name="x650_2link_v2"):
    """Compare independently observed distal-link tip to the declared +Y FK."""
    if (not isinstance(expected_profile_name, str) or not expected_profile_name
            or profile.get("profile_name") != expected_profile_name
            or not np.allclose(profile["mount_body"], [0, 0, -.18], atol=1e-12, rtol=0)
            or not np.allclose(profile["elbow_offset_body_m"], [0, .04, 0], atol=1e-12, rtol=0)):
        raise ValueError("Native direction audit requires the declared v2 geometry")
    angle = np.asarray(profile["angle_offsets"])+np.column_stack((data["q"][:, 0], data["q"].sum(1)))
    local = np.tile(np.asarray(profile["mount_body"])+profile["elbow_offset_body_m"], (len(angle), 1))
    for j, length in enumerate(profile["lengths"]):
        local += length*np.column_stack((np.cos(angle[:, j]), np.zeros(len(angle)), -np.sin(angle[:, j])))
    quat = data["quaternion"]
    # Unit quaternion rotation, evaluated independently of the controller code.
    v, w = quat[:, 1:], quat[:, :1]
    expected = data["position"]+local+2*np.cross(v, np.cross(v, local)+w*local)
    error = np.linalg.norm(expected-data["tip"], axis=1)
    if error.max() > 1e-4:
        raise ValueError("Independent native tip disagrees with declared joint axes/zero offsets")
    return dict(samples=len(error), maximum_error_m=float(error.max()),
                time_weighted_rms_m=float(time_rms(data["t"], error)), tolerance_m=1e-4,
                passed=True, semantics="Independent native distal-link WorldPose tip versus +Y-axis FK of same-step base pose/q; no phase or offset fit")


def metrics(data, desired, rate, mask):
    t, actual, velocity = data["t"][mask], data["q"][mask], data["dq"][mask]
    qref, dqref = desired[mask], rate[mask]
    body, tilt = data["body_error"][mask], data["tilt"][mask]
    error, norm = actual-qref, np.linalg.norm(body, axis=1)
    result = dict(sample_count=len(t), actual_interval_task_s=[float(t[0]), float(t[-1])],
        q_tracking_rmse_rad=time_rms(t, error).tolist(), q_tracking_peak_rad=np.max(abs(error), axis=0).tolist(),
        q_tracking_rmse_deg=np.degrees(time_rms(t, error)).tolist(),
        q_tracking_peak_deg=np.degrees(np.max(abs(error), axis=0)).tolist(),
        dq_tracking_rmse_rad_s=time_rms(t, velocity-dqref).tolist(),
        dq_tracking_peak_rad_s=np.max(abs(velocity-dqref), axis=0).tolist(),
        q_reference_minmax_rad=np.stack((qref.min(0), qref.max(0)), axis=1).tolist(),
        q_actual_minmax_rad=np.stack((actual.min(0), actual.max(0)), axis=1).tolist(),
        sign_cosine=[cosine(actual[:, j], qref[:, j]) for j in range(2)],
        velocity_sign_cosine=[cosine(velocity[:, j], dqref[:, j]) for j in range(2)],
        body_anchor_axis_rmse_m=time_rms(t, body).tolist(),
        body_anchor_axis_peak_m=np.max(abs(body), axis=0).tolist(),
        body_anchor_rmse_m=float(time_rms(t, norm)), body_anchor_peak_m=float(norm.max()),
        tilt_rms_deg=float(time_rms(t, tilt)), tilt_peak_deg=float(tilt.max()),
        joint_speed_abs_max_rad_s=np.max(abs(velocity), axis=0).tolist())
    inactive, sign_agreement = {}, []
    for j in range(2):
        amplitude = float(np.max(abs(qref[:, j])))
        active = abs(qref[:, j]) > max(1e-9, .05*amplitude)
        sign_agreement.append(None if not active.any() else float(np.mean(actual[active, j]*qref[active, j] > 0)))
        if amplitude < 1e-10:
            inactive[f"q{j+1}"] = dict(position_rms_rad=float(time_rms(t, actual[:, j])),
                position_peak_rad=float(np.max(abs(actual[:, j]))),
                position_rms_deg=float(np.degrees(time_rms(t, actual[:, j]))),
                position_peak_deg=float(np.degrees(np.max(abs(actual[:, j])))),
                velocity_rms_rad_s=float(time_rms(t, velocity[:, j])),
                velocity_peak_rad_s=float(np.max(abs(velocity[:, j]))))
    result.update(sign_agreement_fraction_above_5pct_reference=sign_agreement,
                  inactive_joint_cross_talk=inactive)
    return result


def analyze_arrays(data, duration):
    desired, rate = reference(data["t"], duration)
    segment = (duration-4.)/3.
    intervals = {"whole": (0., duration), "J1_only": (2., 2.+segment),
                 "J2_only": (2.+segment, 2.+2*segment), "both": (2.+2*segment, duration-2.)}
    report = {}
    for name, (a, b) in intervals.items():
        mask = (data["t"] >= a-1e-9) & (data["t"] <= b+1e-9)
        if mask.sum() < 2:
            raise ValueError("Insufficient samples for joint segment "+name)
        report[name] = metrics(data, desired, rate, mask)
        report[name]["declared_interval_task_s"] = [a, b]
    return desired, rate, report


def control_audit(rows, begin, duration):
    rows = [r for r in rows if r["phase"] == "measure" and -1e-9 <= float(r["sim_s"])-begin <= duration+1e-9]
    if len(rows) < 2:
        raise ValueError("No complete control-trace measurement rows")
    sim = array(rows, ["sim_s"]).ravel()
    t = array(rows, ["t_s"]).ravel()
    if (np.any(np.diff(sim) <= 0) or t[0] > .05 or t[-1] < duration-.05
            or np.max(abs(sim-begin-t)) > 1e-8 or np.max(np.diff(sim)) > .05+1e-9):
        raise ValueError("Control trace has a different time origin or incomplete control coverage")
    qref, dqref = reference(t, duration)
    actual = array(rows, ["q1", "q2"])
    commanded = array(rows, ["qd1_cmd", "qd2_cmd"])
    reconstructed = np.clip(dqref+3*(qref-actual), -.15, .15)
    residual = float(np.max(abs(commanded-reconstructed)))
    if residual > 1e-10:
        raise ValueError("Actual joint velocity command differs from declared probe law")
    return dict(sample_count=len(rows), max_command_reconstruction_error_rad_s=residual,
                maximum_control_interval_s=float(np.diff(sim).max()),
                law="qd_cmd = clip(qd_reference + 3*(q_reference - ROS measured q), +/-0.15 rad/s)",
                semantics="Command-law audit only; main tracking metrics use independent native physics states")


def load_run(run_dir):
    folder = Path(run_dir).resolve(strict=True)
    stage = read_json(folder/"joint_probe.json")
    meta, manifest = read_json(folder/"controller_metadata.json"), read_json(folder/"native_trace_manifest.json")
    if (stage.get("stage") != "joint_probe" or stage.get("complete") is not True
            or meta.get("stage") != "joint_probe" or meta.get("complete") is not True
            or manifest.get("complete") is not True or meta.get("landed_and_disarmed") is not True):
        raise ValueError("A completed, recovered native joint-probe run is required")
    inputs = ("native_trace.csv", "native_trace_manifest.json", "control_trace.csv")
    for name in inputs:
        if stage["artifacts"].get(name) != sha(folder/name):
            raise ValueError("Stage artifact checksum mismatch: "+name)
    if manifest.get("file") != "native_trace.csv" or manifest.get("sha256") != sha(folder/"native_trace.csv"):
        raise ValueError("Independent native capture checksum mismatch")
    for key in ("model_sha256", "source_urdf_sha256"):
        if not meta.get(key) or stage.get(key) != meta[key] or manifest.get(key) != meta[key]:
            raise ValueError("Model identity differs across native/stage/controller evidence")
    begin, end, duration = float(meta["task_start_sim_s"]), float(meta["task_end_sim_s"]), float(meta["requested_duration_s"])
    if (not np.isfinite([begin, end, duration]).all() or abs(end-begin-duration) > .05
            or not np.allclose(manifest.get("task_window_sim_s"), [begin, end], rtol=0, atol=1e-8)):
        raise ValueError("Task window differs across completed evidence")
    anchor = np.asarray(meta["reference_base_enu_m"], float)
    if anchor.shape != (3,) or not np.isfinite(anchor).all():
        raise ValueError("Fixed native body anchor is invalid")
    with (folder/"native_trace.csv").open(newline="", encoding="utf-8") as stream:
        data = native_arrays(list(csv.DictReader(stream)), begin, duration, anchor)
    with (folder/"control_trace.csv").open(newline="", encoding="utf-8") as stream:
        command_check = control_audit(list(csv.DictReader(stream)), begin, duration)
    desired, rate, sections = analyze_arrays(data, duration)
    kinematic_check = native_tip_audit(data, meta["physical_profile"])
    report = dict(kind="x650_independent_native_joint_probe_audit", source_run=folder.name,
        simulation_backend="gazebo_px4", new_dynamics_simulated=False,
        task_window_sim_s=[begin, begin+duration], fixed_base_anchor_enu_m=anchor.tolist(),
        model_sha256=meta["model_sha256"], source_urdf_sha256=meta["source_urdf_sha256"],
        inputs_sha256={name: sha(folder/name) for name in (*inputs, "joint_probe.json", "controller_metadata.json")},
        generator_sha256=sha(__file__), sample_count=len(data["t"]),
        maximum_native_interval_s=float(np.diff(data["t"]).max()), segments=sections,
        controller_command_audit=command_check, independent_native_tip_geometry_audit=kinematic_check,
        upper_whole_body_allocator_active=meta.get("allocator_active"),
        reference="run_case joint_probe_reference: sin^3; amplitudes [0.08,0], [0,0.08], [0.06,-0.06] rad",
        angle_convention="Gazebo measured joint q, +Y axis; positive angle rotates local +X toward -Z; q0 retains original folded 150deg/-150deg offsets",
        semantics="Raw same-physics-step q/dq/base pose; fixed recorded body anchor; time-weighted RMS; no fitted delay, phase, amplitude, offset or smoothing",
        actuator_limitation="Gazebo joint velocity actuator, not a calibrated real servo model",
        inference_limit="Small individual/combined joint commissioning only; does not establish large-arm-motion stability, whole-body endpoint compensation, or fast figure-eight performance")
    return data, desired, rate, duration, report


def plot(run_dir, output):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    data, desired, _, duration, report = load_run(run_dir)
    output = Path(output).resolve()
    output.mkdir(parents=True, exist_ok=True)
    if (output/"joint_probe_audit.json").exists():
        raise ValueError("Joint audit output exists; use a new directory")
    t, colors = data["t"], ("#1676b7", "#d55e44")
    fig, axes = plt.subplots(5, 1, figsize=(13, 13), sharex=True, layout="constrained")
    for j in range(2):
        axes[j].plot(t, np.degrees(desired[:, j]), color="#222b36", ls="--", lw=1.5, label=f"q{j+1} position reference")
        axes[j].plot(t, np.degrees(data["q"][:, j]), color=colors[j], lw=.95, label=f"q{j+1} native actual")
        axes[j].set_ylabel(f"Joint {j+1} angle (deg)")
        axes[2].plot(t, np.degrees(data["q"][:, j]-desired[:, j]), color=colors[j], lw=.9, label=f"q{j+1} actual - reference")
    axes[2].set_ylabel("Joint error (deg)")
    for j, color in enumerate((colors[0], "#338862", colors[1])):
        axes[3].plot(t, data["body_error"][:, j]*100, color=color, lw=.9, label=f"{'XYZ'[j]} - fixed anchor")
    axes[3].set_ylabel("Body displacement (cm)")
    axes[4].plot(t, data["tilt"], color=colors[0], lw=.9, label="Native body tilt")
    axes[4].set_ylabel("Body tilt (deg)")
    axes[4].set_xlabel("Elapsed task time (s), no time or phase alignment")
    boundaries = [2., 2.+(duration-4.)/3., 2.+2*(duration-4.)/3., duration-2.]
    for axis in axes:
        axis.axvspan(boundaries[0], boundaries[1], color="#d9e9f4", alpha=.45)
        axis.axvspan(boundaries[1], boundaries[2], color="#f6e3d8", alpha=.45)
        axis.axvspan(boundaries[2], boundaries[3], color="#e2efdf", alpha=.45)
        for boundary in boundaries:
            axis.axvline(boundary, color="#77838f", lw=.6, alpha=.6)
        axis.grid(alpha=.18)
        axis.legend(loc="upper right", ncol=3, fontsize=8)
        axis.set_xlim(0, duration)
    for a, b, text in zip(boundaries, boundaries[1:], ("J1 ONLY", "J2 ONLY", "BOTH JOINTS")):
        axes[0].text((a+b)/2, 1.02, text, transform=axes[0].get_xaxis_transform(), ha="center", fontsize=10)
    fig.suptitle("X650 + two-link arm | NATIVE SMALL JOINT PROBE\nActual physics states vs declared reference; Gazebo velocity actuators; no fast-trajectory claim", fontsize=14)
    png, pdf = output/"joint_probe_tracking_and_body.png", output/"joint_probe_tracking_and_body.pdf"
    fig.savefig(png, dpi=160)
    fig.savefig(pdf)
    plt.close(fig)
    report["outputs_sha256"] = {png.name: sha(png), pdf.name: sha(pdf)}
    (output/"joint_probe_audit.json").write_text(json.dumps(report, indent=2, allow_nan=False)+"\n", encoding="utf-8")
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-dir", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    print(json.dumps(plot(args.run_dir, args.output), indent=2, allow_nan=False))


if __name__ == "__main__":
    main()
