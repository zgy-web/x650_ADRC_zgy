"""Render an audited native-backend hold without rerunning or fitting physics."""
from __future__ import annotations

import argparse
import csv
import hashlib
import json
from pathlib import Path
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT), str(ROOT / "control_packages/x650_wholebody_v1")]
from x650_wholebody.model import ArmModel, tip_world


def read(path):
    return json.loads(Path(path).read_text())


def identity(path):
    path = Path(path).resolve(strict=True)
    return {"path": str(path), "sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
            "bytes": path.stat().st_size}


def statistics(values):
    values = np.asarray(values, float)
    if not len(values) or not np.isfinite(values).all():
        raise ValueError("Finite nonempty timing evidence required")
    return {"count": len(values), "min": float(values.min()),
            "median": float(np.median(values)), "p95": float(np.percentile(values, 95)),
            "p99": float(np.percentile(values, 99)), "max": float(values.max())}


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--run-dir", type=Path, required=True)
    ap.add_argument("--certificate", type=Path, required=True)
    ap.add_argument("--output", type=Path, required=True)
    args = ap.parse_args()
    run = args.run_dir.resolve(strict=True)
    certificate = read(args.certificate)
    if (certificate.get("passed") is not True or certificate.get("complete") is not True
            or certificate.get("stage") != "tip_hold"
            or certificate.get("source_run_directory") != str(run)):
        raise ValueError("A complete same-run endpoint-hold certificate is required")
    names = ("controller_metadata.json", "controller_task.json", "native_trace.csv",
             "control_trace.csv", "controller_diagnostics.jsonl", "control_execution_trace.jsonl",
             "loop_timing_evidence.json")
    inputs = {name: identity(run / name) for name in names}
    for value in inputs.values():
        if certificate["artifacts"].get(value["path"]) != value["sha256"]:
            raise ValueError("Audited data bytes changed: " + value["path"])
    meta, cfg = read(run / names[0]), read(run / names[1])
    origin, duration = meta["task_start_sim_s"], cfg["duration_s"]
    model = ArmModel.from_report(cfg["preparation_report"])
    target = tip_world(np.asarray(meta["reference_base_enu_m"]),
                       np.asarray(meta["reference_rotation"]), np.zeros(2), model)
    with (run / "native_trace.csv").open() as stream:
        native = [row for row in csv.DictReader(stream)
                  if -1e-9 <= float(row["sim_s"]) - origin <= duration + 1e-9]
    if any(row["valid_tip"] != "1" for row in native):
        raise ValueError("Invalid native tip sample")
    stamps = np.array([float(row["sim_s"]) - origin for row in native])
    error = np.array([[float(row["tip_" + a]) for a in "xyz"] for row in native]) - target
    norm = np.linalg.norm(error, axis=1)
    rms = float(np.sqrt(np.trapz(norm**2, stamps) / (stamps[-1] - stamps[0])))
    endpoint = certificate["result"]["endpoint"]
    if (abs(rms - endpoint["position_rmse_m"]) > 1e-12
            or abs(norm.max() - endpoint["position_max_m"]) > 1e-12
            or len(stamps) != endpoint["sample_count"]):
        raise ValueError("Independent figure calculation differs from the audit")
    with (run / "control_trace.csv").open() as stream:
        controls = [row for row in csv.DictReader(stream) if row["phase"] == "measure"]
    ct = np.array([float(row["sim_s"]) - origin for row in controls])
    compute = np.array([float(row["compute_ms"]) for row in controls])
    solver = []
    for line in (run / "controller_diagnostics.jsonl").open():
        row = json.loads(line)
        if row.get("phase") == "measure":
            d = row["diagnostics"]
            if (not d.get("accepted_optimum") or not d.get("ts20_nonlinear_check_passed")
                    or d.get("previous_plan_fallback") or d.get("solve_budget_exceeded")):
                raise ValueError("Unaccepted candidate in the plotted window")
            solver.append(d["solve_ms"])
    publications = []
    for line in (run / "control_execution_trace.jsonl").open():
        row = json.loads(line)
        if (row.get("event") == "publication_returned" and row.get("channel") == "px4_pva"
                and row.get("context", {}).get("phase") == "measure"):
            publications.append(row["publish_after"]["wall_monotonic_s"])
    if len(solver) != len(controls):
        raise ValueError("Candidate/control record counts differ")
    intervals = np.diff(ct) * 1000
    if abs(intervals.max()/1000 - certificate["timing"]["max_actual_send_interval_s"]) > 1e-10:
        raise ValueError("Control interval calculation differs from the audit")
    report = {"kind": "audited_native_endpoint_hold_presentation_v1", "passed": True,
        "run_directory": str(run), "certificate": identity(args.certificate), "inputs": inputs,
        "duration_s": duration, "endpoint": endpoint, "control_updates": len(controls),
        "timing": {"control_interval_sim_ms": statistics(intervals),
                   "recorded_compute_wall_ms": statistics(compute),
                   "qp_solve_wall_ms": statistics(solver)},
        "numerical_backend": cfg["numerical_backend"],
        "local_send_semantics": "MAVLink send returned locally; not a downstream actuator acknowledgement",
        "same_time_no_phase_alignment": True, "physics_rerun": False,
        "hardware_qualified": False, "fast_figure_eight_run": False}
    if len(publications) >= 2:
        report["timing"]["pva_local_send_interval_wall_ms"] = statistics(np.diff(publications)*1000)
    gc = read(run / "loop_timing_evidence.json")
    report["natural_gc_observation"] = gc.get("natural_gc")
    args.output.mkdir(parents=True, exist_ok=False)
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    plt.rcParams.update({"font.size": 11, "axes.spines.top": False,
                         "axes.spines.right": False, "savefig.facecolor": "white"})
    fig, axes = plt.subplots(3, 1, figsize=(11, 8), sharex=True,
                             gridspec_kw={"height_ratios": [1.3, 1, 1]})
    fig.suptitle("X650 + two-link TS20 | C++ upper MPC | 30 s endpoint hold", fontsize=16)
    axes[0].plot(stamps, norm*1000, color="#1175ad", lw=1.5, label="Actual 3D tip error")
    axes[0].axhline(rms*1000, color="#de8b00", ls="--", label=f"RMS {rms*1000:.3f} mm")
    axes[0].set_ylabel("Tip error [mm]")
    axes[0].set_title(f"Independent Gazebo tip; same-time error; peak {norm.max()*1000:.3f} mm", fontsize=11)
    axes[0].legend(loc="upper right", ncol=2)
    axes[1].plot(ct, compute, color="#7053a0", lw=1.0, label="Recorded compute + dispatch")
    axes[1].plot(ct, solver, color="#218c60", lw=1.0, label="QP solve only")
    axes[1].axhline(50, color="#bb3535", ls="--", label="50 ms rejection threshold")
    axes[1].set_ylabel("Wall time [ms]"); axes[1].set_ylim(0, 55)
    axes[1].legend(loc="upper right", ncol=3, fontsize=9)
    axes[2].plot(ct[1:], intervals, color="#1175ad", lw=1.0, label="Control update interval")
    axes[2].axhline(20, color="#888888", ls=":", label="20 ms nominal period")
    axes[2].axhline(50, color="#bb3535", ls="--", label="50 ms rejection threshold")
    axes[2].set_ylabel("Simulation time [ms]"); axes[2].set_ylim(0, 55)
    axes[2].set_xlabel("Elapsed hold time [s]")
    axes[2].legend(loc="upper right", ncol=3, fontsize=9)
    for ax in axes:
        ax.set_xlim(0, duration); ax.grid(alpha=.22)
    fig.text(.01, .012, "Conditional TS20 model and precision EV positioning; frozen PX4 PID. No hardware qualification.", fontsize=9)
    fig.tight_layout(rect=(0, .035, 1, .96))
    fig.savefig(args.output / "endpoint_hold.png", dpi=160)
    fig.savefig(args.output / "endpoint_hold.pdf")
    plt.close(fig)
    report["figure_source"] = identity(__file__)
    (args.output / "hold_summary.json").write_text(json.dumps(report, indent=2, allow_nan=False))
    print(json.dumps({k: report[k] for k in ("passed", "duration_s", "endpoint", "timing")}, indent=2))


if __name__ == "__main__":
    main()
