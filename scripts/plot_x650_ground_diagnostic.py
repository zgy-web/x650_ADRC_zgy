#!/usr/bin/env python3
"""Plot the completed native ground-only window; never label it as a flight."""
from __future__ import annotations

import argparse
import csv
import hashlib
import json
from pathlib import Path

import numpy as np


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def load_ground(run_dir):
    run_dir = Path(run_dir).resolve(strict=True)
    path, meta_path = run_dir/"native_ground.csv", run_dir/"native_ground.json"
    meta = json.loads(meta_path.read_text(encoding="utf-8"))
    manifest_path = run_dir/"native_ground_manifest.json"
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    if (meta.get("stage") != "native_ground" or meta.get("complete") is not True or meta.get("passed") is not True
            or manifest.get("complete") is not True or manifest.get("file") != path.name
            or manifest.get("sha256") != sha(path)
            or meta["artifacts"].get(path.name) != sha(path)
            or meta["artifacts"].get(manifest_path.name) != sha(manifest_path)):
        raise ValueError("Native ground evidence identity, completion or checksum failed")
    if any(meta[key] != manifest[key] for key in ("model_sha256", "source_urdf_sha256")):
        raise ValueError("Native ground evidence model/source identities disagree")
    begin, end = meta["interval_sim_s"]
    with path.open(newline="", encoding="utf-8") as stream:
        rows = [r for r in csv.DictReader(stream) if begin-1e-9 <= float(r["sim_s"]) <= end+1e-9]
    if len(rows) < 2 or any(r[name] != "1" for r in rows for name in ("valid_base", "valid_joint_state", "valid_tip")):
        raise ValueError("Ground window lacks valid independent native samples")
    def array(names):
        value = np.asarray([[float(r[name]) for name in names] for r in rows])
        if not np.isfinite(value).all():
            raise ValueError("Nonfinite native ground measurements")
        return value
    stamps = array(["sim_s"]).ravel()
    if np.any(np.diff(stamps) <= 0):
        raise ValueError("Native physics timestamps are not increasing")
    quaternion = array(["base_q"+axis for axis in "wxyz"])
    norm = np.linalg.norm(quaternion, axis=1)
    if np.any(abs(norm-1) > 1e-6):
        raise ValueError("Native ground orientation is not unit length")
    quaternion /= norm[:, None]
    tilt = np.degrees(np.arccos(np.clip(1-2*(quaternion[:, 1]**2+quaternion[:, 2]**2), -1, 1)))
    q = array(["arm_joint_1_q", "arm_joint_2_q"])
    dq = array(["arm_joint_1_qd", "arm_joint_2_qd"])
    z, speed = array(["base_pz"]).ravel(), np.linalg.norm(array(["base_vx", "base_vy", "base_vz"]), axis=1)
    recomputed = {"duration_s": float(stamps[-1]-stamps[0]), "sample_count": len(rows),
                  "tilt_max_deg": float(tilt.max()), "speed_max_m_s": float(speed.max()),
                  "folded_q_abs_max_rad": np.max(abs(q), axis=0).tolist(),
                  "joint_speed_abs_max_rad_s": np.max(abs(dq), axis=0).tolist(),
                  "max_physics_interval_s": float(np.diff(stamps).max())}
    for key, value in recomputed.items():
        if not np.allclose(value, meta["metrics"][key], rtol=1e-9, atol=1e-10):
            raise ValueError("Ground metric recomputation disagrees: "+key)
    recomputed.update(base_height_min_m=float(z.min()), base_height_max_m=float(z.max()),
                      base_height_peak_to_peak_m=float(np.ptp(z)))
    audit = {"kind": "native_ground_only_diagnostic", "experiment_is_flight": False,
             "hover_validated": False, "native_contact_validated": False,
             "source_run": run_dir.name, "interval_sim_s": [begin, end],
             "csv_sha256": sha(path), "ground_evidence_sha256": sha(meta_path),
             "ground_manifest_sha256": sha(manifest_path), "model_sha256": meta["model_sha256"],
             "source_urdf_sha256": meta["source_urdf_sha256"], "metrics_recomputed": recomputed,
             "plot_semantics": "All original native samples in the complete pre-takeoff ground window; elapsed time subtracts declared window start; no smoothing or phase alignment; millidegree/nanoradian display units are explicit on axes",
             "height_semantics": "World height of base_link origin while landing gear is on the ground, not a flight altitude",
             "precision_semantics": "Tiny joint angles and tilt are numerical simulation observations, not claims of real hardware accuracy"}
    return stamps-begin, z, tilt, q, audit


def plot(run_dir, output):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.ticker import FormatStrFormatter
    t, z, tilt, q, audit = load_ground(run_dir)
    output = Path(output).resolve()
    output.mkdir(parents=True, exist_ok=True)
    fig, axes = plt.subplots(3, 1, figsize=(12, 9), sharex=True, layout="constrained")
    blue, red = "#1676b7", "#d55e44"
    axes[0].plot(t, z, color=blue, lw=1.)
    nominal_z = .3697059509277344
    axes[0].axhline(nominal_z, color="#768494", ls="--", lw=1., label="CAD landing-gear support height")
    axes[0].set_ylim(nominal_z-.001, nominal_z+.001)
    axes[0].yaxis.set_major_formatter(FormatStrFormatter("%.4f"))
    axes[0].set_ylabel("Base origin world Z (m)")
    height_pp_um = audit["metrics_recomputed"]["base_height_peak_to_peak_m"]*1e6
    axes[0].text(.98, .92, f"Height peak-to-peak: {height_pp_um:.3f} micrometres", ha="right", va="top", transform=axes[0].transAxes)
    axes[0].legend(loc="lower right", fontsize=9)
    axes[1].plot(t, tilt*1000, color=blue, lw=1.)
    axes[1].set_ylabel("Body tilt (millidegrees)")
    axes[1].set_ylim(bottom=0)
    axes[1].text(.98, .92, f"Peak tilt: {tilt.max():.7f} deg", ha="right", va="top", transform=axes[1].transAxes)
    axes[2].plot(t, q[:, 0]*1e9, color=blue, lw=1., label="q1")
    axes[2].plot(t, q[:, 1]*1e9, color=red, lw=1., label="q2")
    axes[2].set_ylabel("Measured joint angle (nanoradians)")
    axes[2].set_xlabel("Elapsed native ground-window time (s)")
    axes[2].legend(loc="upper right")
    axes[2].text(.98, .08, f"max |q| = {np.max(abs(q)):.3g} rad", ha="right", va="bottom", transform=axes[2].transAxes)
    for axis in axes:
        axis.grid(alpha=.22)
        axis.set_xlim(0, t[-1])
    count = audit["metrics_recomputed"]["sample_count"]
    duration = audit["metrics_recomputed"]["duration_s"]
    hz = 1/audit["metrics_recomputed"]["max_physics_interval_s"]
    fig.suptitle(f"X650 + two-link arm | GROUND STABILITY ONLY\nNative Gazebo states before takeoff; {duration:.3f} s, {count:,} samples, {hz:.0f} Hz; no flight result", fontsize=15)
    png, pdf = output/"native_ground_diagnostic.png", output/"native_ground_diagnostic.pdf"
    fig.savefig(png, dpi=160)
    fig.savefig(pdf)
    plt.close(fig)
    audit.update(plot_png_sha256=sha(png), plot_pdf_sha256=sha(pdf), generator_sha256=sha(__file__))
    (output/"native_ground_diagnostic_audit.json").write_text(json.dumps(audit, indent=2)+"\n", encoding="utf-8")
    return {"png": str(png), "pdf": str(pdf), **audit}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-dir", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    print(json.dumps(plot(args.run_dir, args.output), indent=2))


if __name__ == "__main__":
    main()
