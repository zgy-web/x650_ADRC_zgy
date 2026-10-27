#!/usr/bin/env python3
"""Summarize six fresh, independently accepted upper-controller experiments.

Run AFTER all physics and independent audits have finished. This program reads
original evidence, verifies bound bytes, and writes only a new output directory.
It never launches ROS/Gazebo, alters evidence, or fits a time/position alignment.

Example:
  python3 scripts/summarize_x650_upper_comparison.py --reports \
    /stage/nmpc_hold_audit/report.json /stage/mpc_hold_audit/report.json \
    /stage/nmpc_normal_audit/report.json /stage/mpc_normal_audit/report.json \
    /stage/nmpc_fast_audit/report.json /stage/mpc_fast_audit/report.json \
    --output /stage/paired_summary_v1
"""
from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
from pathlib import Path
import time

import numpy as np

CASES = ("hold", "normal", "fast")
BACKENDS = ("mpc_callable", "nmpc")
DURATIONS = {"hold": 30., "normal": 32., "fast": 24.}
TASK_STAGE_FIELDS = {"scenario", "upper_controller", "stage", "trajectory",
                     "duration_s", "trajectory_parameters", "figure8_protocol"}
LABELS = {"mpc_callable": "MPC", "nmpc": "Upper NMPC"}


def require(value, message):
    if not value:
        raise ValueError(message)


def canonical(value):
    return json.dumps(value, sort_keys=True, separators=(",", ":"), allow_nan=False).encode()


def digest_value(value):
    return hashlib.sha256(canonical(value)).hexdigest()


def exact_differences(expected, actual, path=""):
    """Expose changed fixed/estimated fields; never normalize them silently."""
    if isinstance(expected, dict) and isinstance(actual, dict):
        result = []
        for key in sorted(set(expected) | set(actual)):
            location = path+"."+key if path else key
            if key not in expected or key not in actual:
                result.append(dict(field=location, expected=expected.get(key), actual=actual.get(key),
                                   missing_from="expected" if key not in expected else "actual"))
            else:
                result.extend(exact_differences(expected[key], actual[key], location))
        return result
    if isinstance(expected, list) and isinstance(actual, list) and len(expected) == len(actual):
        return [item for i,(a,b) in enumerate(zip(expected,actual))
                for item in exact_differences(a,b,f"{path}[{i}]")]
    return [] if expected == actual else [dict(field=path, expected=expected, actual=actual)]


def read(path):
    return json.loads(Path(path).read_text(encoding="utf-8"))


class Evidence:
    """Hash each file once, then detect edits while generating derived outputs."""
    def __init__(self):
        self.cache = {}
        self.selected = {}

    def identity(self, path, *, selected=False):
        path = Path(path).resolve(strict=True)
        require(path.is_file(), "Evidence is not a file: " + str(path))
        stat = path.stat()
        stamp = (stat.st_size, stat.st_mtime_ns)
        key = str(path)
        if key not in self.cache:
            sha = hashlib.sha256()
            with path.open("rb") as handle:
                for chunk in iter(lambda: handle.read(1024*1024), b""):
                    sha.update(chunk)
            self.cache[key] = (stamp, dict(path=key, bytes=stat.st_size, sha256=sha.hexdigest()))
        require(self.cache[key][0] == stamp, "Evidence changed during summary: " + key)
        result = self.cache[key][1]
        if selected:
            self.selected[key] = result
        return result

    def bound(self, item, *, selected=False):
        require(isinstance(item, dict) and {"path", "sha256"} <= set(item), "Missing artifact binding")
        actual = self.identity(item["path"], selected=selected)
        require(actual["sha256"] == item["sha256"]
                and ("bytes" not in item or actual["bytes"] == item["bytes"]),
                "Artifact bytes differ: " + item["path"])
        return actual

    def document(self, item):
        actual = self.bound(item, selected=True)
        return read(actual["path"])

    def finish(self):
        for path in self.cache:
            self.identity(path)


def windows(case):
    end = DURATIONS[case]
    result = {"endpoint": (0., end)}
    if case != "hold":
        result.update(tracking_endpoint=(2., end-4.), tail_endpoint=(end-4., end))
    return result


def expected_trajectory(case):
    """Exact public task specs from the original normal/fast task modules."""
    require(case in CASES, "Unknown explicit trajectory case")
    if case == "hold":
        return {"kind": "hold"}
    return dict(kind=case+"_figure8", amplitude_x=.25, amplitude_z=.10,
        period_s=12. if case == "normal" else 8., cycles=2, start_s=2.,
        ramp_s=2., duration_s=DURATIONS[case])


def reference_position(stamps, center, trajectory):
    """Original C2 phase law; recomputed errors must reproduce audited metrics."""
    center = np.asarray(center, dtype=float)
    require(center.shape == (3,) and np.isfinite(center).all(), "Invalid recorded target centre")
    if trajectory == {"kind": "hold"}:
        return np.broadcast_to(center, (len(stamps), 3)).copy()
    kinds = {"normal_figure8": "normal", "fast_figure8": "fast"}
    require(trajectory.get("kind") in kinds, "Unknown explicit normal/fast reference kind")
    require(trajectory == expected_trajectory(kinds[trajectory["kind"]]),
            "Reference amplitude, period, cycles, ramp or declared duration changed")
    period, ramp = trajectory["period_s"], trajectory["ramp_s"]
    cycles, start = trajectory["cycles"], trajectory["start_s"]
    require(period > 0 and 0 < ramp <= cycles*period and cycles >= 1 and start >= 0,
            "Invalid figure-eight phase parameters")
    duration = cycles*period+ramp
    omega, total = 2*np.pi/period, 2*np.pi*cycles
    s = np.asarray(stamps)-start
    phi = np.zeros_like(s)
    after = s >= duration
    first = (s > 0) & (s < ramp)
    last = (s > duration-ramp) & (s < duration)
    middle = (s >= ramp) & (s <= duration-ramp)
    def integral(u):
        return 2.5*u**4-3*u**5+u**6
    phi[after] = total
    phi[first] = omega*ramp*integral(s[first]/ramp)
    phi[last] = total-omega*ramp*integral((duration-s[last])/ramp)
    phi[middle] = omega*(s[middle]-.5*ramp)
    offsets = np.column_stack((trajectory["amplitude_x"]*np.sin(phi),
        np.zeros(len(s)), trajectory["amplitude_z"]*np.sin(2*phi)))
    return center+offsets


def load_run(path, evidence):
    report_id = evidence.identity(path, selected=True)
    report = read(path)
    require(report.get("kind") == "x650_upper_closed_loop_audit_v1"
            and all(report.get(k) is True for k in
                    ("passed", "complete", "execution_passed", "performance_passed", "physical_acceptance"))
            and report.get("hardware_qualified") is False
            and report.get("execution_errors") == report.get("performance_errors") == [],
            "Only fully accepted fresh simulation audits can enter the six-run comparison")
    backend, case = report["backend"], report["case_id"]
    require(backend in BACKENDS and case in CASES, "Unknown backend/case")
    artifacts = report["artifact_identities"]
    require(artifacts, "Audit has no original evidence bindings")
    for item in artifacts:
        evidence.bound(item)
    launch = evidence.document(report["launch"])
    cfg = evidence.document(report["task_config"])
    startup = evidence.document(report["startup_manifest"])
    launch_dir = Path(report["launch_directory"]).resolve(strict=True)
    run_dir = Path(report["run_directory"]).resolve(strict=True)
    require(Path(report["launch"]["path"]).resolve() == launch_dir/"launch.json"
            and launch["run_directories"] == [str(run_dir)] and cfg == launch["task"]
            and launch["task_sha256"] == report["task_sha256"] == report["task_config"]["sha256"]
            and all(launch.get(k) is True for k in ("complete", "runtime_closed", "sources_unchanged",
                "runtime_binaries_unchanged", "launch_inputs_unchanged")), "Launch/task binding differs")
    bound_paths = {str(Path(item["path"]).resolve()): item for item in artifacts}
    def original(relative, directory=launch_dir):
        selected = directory/relative
        require(str(selected) in bound_paths, "Missing audit binding: " + str(selected))
        return evidence.document(bound_paths[str(selected)])
    protocol = original("protocol.json")
    options = original("options.json")
    start = original("isolated_live/live_start.json", run_dir)
    require(cfg["upper_controller"]["backend"] == backend == options["backend"] == protocol["backend"]
            and cfg["upper_controller"]["case_id"] == case == protocol["case_id"]
            and startup["task_config"]["sha256"] == options["task_config_sha256"] == report["task_sha256"]
            and startup["trajectory"] == protocol["trajectory"]
            and startup["trajectory"] == expected_trajectory(case)
            and cfg["duration_s"] == protocol["duration_s"] == DURATIONS[case]
            and cfg["disturbance_enabled"] is False, "Experiment identity/reference differs")
    require(digest_value(startup["config"]) == report["config_sha256"]
            and launch["provenance"]["model_sha256"] == report["model_sha256"]
            and report["runtime_binaries_sha256"] == launch["runtime_binaries_sha256"],
            "Physical model/controller/runtime binding differs")
    for filename, digest in report["runtime_binaries_sha256"].items():
        evidence.bound(dict(path=filename, sha256=digest), selected=True)
    evidence.bound(options["worker_executable"], selected=True)
    require(report["runtime_binaries_sha256"].get(options["worker_executable"]["path"])
            == options["worker_executable"]["sha256"], "Selected worker not in actual runtime")
    for key in ("core_library", "optimization_library", "predictor_library"):
        evidence.bound(startup[key], selected=True)
    require(report["timing_profile"] == cfg["timing_profile"] == "mpc25_dispatch50_v1"
            and report["effective_profile"] == protocol["effective_timing_profile"]
            and report["effective_profile"]["nominal_solve_period_ns"] == 40_000_000
            and report["effective_profile"]["output_period_ns"] == 20_000_000,
            "Paired timing changed")
    origin = float(start["origin_s"])
    require(report["window_sim_s"] == [origin, origin+DURATIONS[case]], "Original physical window differs")
    observed = {}
    for key, interval in windows(case).items():
        metric = report[key]
        require(metric["same_time"] is True and metric["no_phase_alignment"] is True
                and metric["source"] == "native_trace.csv independent physical tip"
                and list(metric["interval_task_s"]) == list(interval), "Endpoint metric scope differs")
        require(all(math.isfinite(metric[k]) and metric[k] >= 0 for k in ("position_rmse_m", "position_max_m")),
                "Invalid endpoint metric")
        observed[key] = metric
    trace = run_dir/"native_trace.csv"
    require(str(trace) in bound_paths, "Native trace is not bound to audit")
    trace_id = evidence.bound(bound_paths[str(trace)], selected=True)
    return dict(report=report, identity=report_id, cfg=cfg, startup=startup, launch=launch,
        protocol=protocol, options=options, start=start, trace=trace_id, windows=observed,
        evidence_manifest_sha256=digest_value(artifacts), evidence_count=len(artifacts))


def check_paired(runs):
    require(len(runs) == 6, "Exactly six explicit new accepted audit reports required")
    indexed = {(r["report"]["backend"], r["report"]["case_id"]): r for r in runs}
    require(len(indexed) == 6 and set(indexed) == {(b,c) for b in BACKENDS for c in CASES},
            "Duplicate or missing backend/case")
    first = runs[0]
    def common(r):
        cfg, report = r["cfg"], r["report"]
        return dict(model_sha256=report["model_sha256"], config=r["startup"]["config"],
            physical_task={k:v for k,v in cfg.items() if k not in TASK_STAGE_FIELDS},
            original_task=cfg["upper_controller"]["original_task"],
            pid_sha256=report["parameters"]["pid_parameters_sha256"],
            effective_px4_parameters=report["configuration"]["effective_parameters"],
            timing=report["effective_profile"], resources=r["launch"]["execution_resources"],
            runtime=report["runtime_binaries_sha256"],
            numerical_options=r["options"]["options"],
            native_libraries={k:r["startup"][k] for k in
                ("core_library", "optimization_library", "predictor_library")})
    baseline = common(first)
    for run in runs:
        differences = exact_differences(baseline, common(run))
        require(not differences, "Fixed/configuration fields differ for "
                +run["report"]["backend"]+"/"+run["report"]["case_id"]+": "
                +json.dumps(differences, allow_nan=False))
    for case in CASES:
        m, n = (indexed[(b,case)] for b in BACKENDS)
        require(m["startup"]["trajectory"] == n["startup"]["trajectory"]
                and {k:v for k,v in m["cfg"].items() if k not in {"scenario", "upper_controller"}}
                == {k:v for k,v in n["cfg"].items() if k not in {"scenario", "upper_controller"}},
                "Paired case changed more than backend/qualification identity")
        require(m["report"]["task_sha256"] != n["report"]["task_sha256"],
                "Different backend task hashes unexpectedly identical")
    return indexed, baseline


def native_series(run):
    data = []
    origin, duration = run["start"]["origin_s"], DURATIONS[run["report"]["case_id"]]
    with Path(run["trace"]["path"]).open(newline="", encoding="utf-8") as stream:
        for row in csv.DictReader(stream):
            t = float(row["sim_s"])-origin
            if -1e-9 <= t <= duration+1e-9:
                require(row["valid_tip"] == "1", "Invalid physical tip sample")
                data.append([t, *[float(row["tip_"+axis]) for axis in "xyz"]])
    a = np.asarray(data, dtype=float)
    require(a.ndim == 2 and a.shape[1] == 4 and len(a) >= 2 and np.isfinite(a).all()
            and np.all(np.diff(a[:,0]) > 0) and np.max(np.diff(a[:,0])) <= .008+1e-9,
            "Nonfinite, sparse or nonmonotone physical tip trace")
    t = a[:,0]
    target = reference_position(t, run["start"]["target_position_enu"], run["startup"]["trajectory"])
    error = a[:,1:]-target
    integrate = np.trapezoid if hasattr(np,"trapezoid") else np.trapz
    for key, metric in run["windows"].items():
        lo, hi = metric["interval_task_s"]
        mask = (lo-1e-9 <= t) & (t <= hi+1e-9)
        stamps, e = t[mask], error[mask]
        require(len(stamps) == metric["sample_count"] and stamps[-1]-stamps[0] >= hi-lo-.04,
                "Native trace does not reproduce original audit window")
        norm2 = np.sum(e*e, axis=1)
        recalculated = dict(position_rmse_m=float(np.sqrt(integrate(norm2,stamps)/(stamps[-1]-stamps[0]))),
            position_max_m=float(np.sqrt(norm2.max())),
            axis_rmse_m=np.sqrt(integrate(e*e,stamps,axis=0)/(stamps[-1]-stamps[0])).tolist())
        require(all(np.allclose(recalculated[k], metric[k], rtol=1e-10, atol=1e-12)
                    for k in recalculated), "Recomputed physical metric differs from accepted audit: " + key)
    return dict(task_time_s=t, norm_mm=np.linalg.norm(error,axis=1)*1000,
                physical_position=a[:,1:], reference=target)


def relative(mpc, nmpc):
    return dict(mpc=mpc, nmpc=nmpc, nmpc_minus_mpc=nmpc-mpc,
        nmpc_minus_mpc_percent=100*(nmpc-mpc)/mpc if mpc != 0 else None,
        percent_denominator="MPC; null if zero")


def build_summary(indexed, common):
    paired = []
    for case in CASES:
        m, n = (indexed[(b,case)] for b in BACKENDS)
        metrics = {}
        for window in windows(case):
            metrics[window] = {key:relative(m["windows"][window][key]*1000,
                n["windows"][window][key]*1000) for key in ("position_rmse_m", "position_max_m")}
            metrics[window] = {key[:-2]+"_mm":value for key,value in metrics[window].items()}
        timing = {}
        for key in ("core_ms", "callback_ms", "gate_roundtrip_ms"):
            timing[key] = {stat:relative(m["report"]["worker"]["durations"][key][stat],
                n["report"]["worker"]["durations"][key][stat]) for stat in ("median","p95","p99","maximum")}
        timing["actual_send_gap_ms"] = {stat:relative(m["report"]["timing"]["actual_send_gap_ms"][stat],
                n["report"]["timing"]["actual_send_gap_ms"][stat]) for stat in ("median","p95","p99","maximum")}
        centers = {b:indexed[(b,case)]["start"]["target_position_enu"] for b in BACKENDS}
        paired.append(dict(case_id=case, duration_s=DURATIONS[case],
            trajectory=m["startup"]["trajectory"], endpoint_windows=metrics,
            interval_task_s={k:list(v) for k,v in windows(case).items()}, timing=timing,
            tilt_peak_deg=relative(m["report"]["tilt_peak_deg"], n["report"]["tilt_peak_deg"]),
            recorded_target_centers_enu_m=centers,
            target_center_difference_m=float(np.linalg.norm(np.asarray(centers["nmpc"])-centers["mpc_callable"])),
            target_center_policy="Use each original recorded LiveStart target; no post-hoc fitted translation",
            reports={b:indexed[(b,case)]["identity"] for b in BACKENDS}))
    runs = []
    for case in CASES:
        for backend in BACKENDS:
            run = indexed[(backend,case)]; r = run["report"]
            runs.append(dict(backend=backend, case_id=case, report=run["identity"],
                task_sha256=r["task_sha256"], config_sha256=r["config_sha256"], model_sha256=r["model_sha256"],
                original_artifact_manifest_sha256=run["evidence_manifest_sha256"],
                original_artifact_count=run["evidence_count"], native_trace=run["trace"],
                recorded_initial_conditions={k:run["start"][k] for k in
                    ("origin_s","anchor_position_enu","anchor_rotation_row_major","anchor_q",
                     "target_position_enu","ref_alt","yaw_ned")},
                consumption={k:v for k,v in r["consumption"].items() if k in
                    ("send_count","observed_receipt_count","all_sends_applied","zero_consumed","joint_counts")},
                memory_commits={k:v for k,v in r["memory_commits"].items() if k in
                    ("actual_send_commits","discarded_unsent_sequence","pending_send_sequence")},
                recovery={k:v for k,v in r["recovery"].items() if k in
                    ("ulog_landed","ulog_arming_state","recovery_failsafe_samples")}))
    return dict(kind="x650_upper_paired_comparison_v1", complete=True,
        exact_declared_conditions_verified=True, all_six_simulation_audits_passed=True,
        common_conditions_sha256=digest_value(common), common_conditions=common,
        single_run_per_backend_per_case=True, replicates_per_cell=1,
        superiority_claimed=False, statistical_significance_claimed=False,
        hardware_qualified=False, full_body_torque_nmpc=False,
        interpretation="Observed one-run differences under the checked conditions; not statistical evidence of superiority",
        difference_convention="NMPC minus MPC; negative error/time differences mean lower in this observed pair",
        no_phase_alignment=True, no_fitted_translation=True, reference_formula_crosschecked_by_audit_metrics=True,
        identical_measured_initial_states_claimed=False,
        paired_cases=paired, runs=runs)


def table_text(summary):
    lines = ["# Upper-controller paired simulation observations", "",
        "One fresh run per backend and case (n=1). All six independent audits passed. "
        "These observations do not establish superiority or statistical significance.", "",
        "Errors use the original physical tip and same-time world-frame reference. "
        "Tracking includes the original smooth phase ramps; the final four seconds are the tail hold.", "",
        "| Case | Window [s] | MPC RMSE [mm] | NMPC RMSE [mm] | Difference [%] | MPC peak [mm] | NMPC peak [mm] |",
        "|---|---:|---:|---:|---:|---:|---:|"]
    def number(value):
        return "undefined (MPC = 0)" if value is None else f"{value:.3f}"
    for case in summary["paired_cases"]:
        for window, metric in case["endpoint_windows"].items():
            rmse, peak = metric["position_rmse_mm"], metric["position_max_mm"]
            interval = case["interval_task_s"][window]
            lines.append(f"| {case['case_id']} / {window} | {interval[0]:g}–{interval[1]:g} | "
                f"{rmse['mpc']:.3f} | {rmse['nmpc']:.3f} | {number(rmse['nmpc_minus_mpc_percent'])} | "
                f"{peak['mpc']:.3f} | {peak['nmpc']:.3f} |")
    lines += ["", "Difference = 100 × (NMPC − MPC) / MPC. Negative means lower in this single observed pair.", "",
        "| Case | MPC core p95 / max [ms] | NMPC core p95 / max [ms] | MPC gate max [ms] | NMPC gate max [ms] |",
        "|---|---:|---:|---:|---:|"]
    for case in summary["paired_cases"]:
        core, gate = case["timing"]["core_ms"], case["timing"]["gate_roundtrip_ms"]
        lines.append(f"| {case['case_id']} | {core['p95']['mpc']:.3f} / {core['maximum']['mpc']:.3f} | "
            f"{core['p95']['nmpc']:.3f} / {core['maximum']['nmpc']:.3f} | "
            f"{gate['maximum']['mpc']:.3f} | {gate['maximum']['nmpc']:.3f} |")
    lines += ["", "Task hashes differ because backend and stage qualification identities differ. "
        "The physical model, model parameters, PID, effective PX4 parameters, numerical libraries, "
        "worker, timing profile, and declared paired trajectories were checked separately.", "",
        "Recorded target centres are retained in comparison.json. Each run uses its original centre; "
        "measured traces are never translated or phase shifted to reduce the error.", ""]
    return "\n".join(lines)


def plot_errors(indexed, series, out):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(3, 1, figsize=(10, 8), layout="constrained")
    for ax, case in zip(axes, CASES):
        duration = DURATIONS[case]
        if case != "hold":
            ax.axvspan(0,2,color="0.9",label="Preparation")
            ax.axvspan(duration-4,duration,color="#ecf4e9",label="Tail hold")
        for backend, color in (("mpc_callable", "#2878b5"), ("nmpc", "#d17a00")):
            data = series[(backend,case)]
            ax.plot(data["task_time_s"],data["norm_mm"],label=LABELS[backend],color=color,linewidth=1.1)
        ax.set(title=f"{case.capitalize()} — original same-time 3D endpoint error",
               xlabel="Original task time [s]",ylabel="Error [mm]",xlim=(0,duration))
        ax.set_ylim(bottom=0.)
        ax.grid(alpha=.25)
        ax.legend(loc="upper right" if case == "hold" else "lower left",fontsize=8,ncol=2)
    fig.suptitle("Upper MPC / bounded upper NMPC | one run per condition; no phase or translation fit",fontsize=11)
    fig.savefig(out/"endpoint_error.png",dpi=180)
    fig.savefig(out/"endpoint_error.pdf")
    plt.close(fig)


def export_endpoint_series(indexed, series, out, evidence):
    """Lossless float64 CSV view of validated original-window observations."""
    headers = ("task_time_s", "physical_x_m", "physical_y_m", "physical_z_m",
               "reference_x_m", "reference_y_m", "reference_z_m", "error_norm_mm")
    artifacts = []
    for case in CASES:
        for backend in BACKENDS:
            data = series[(backend, case)]
            values = np.column_stack((data["task_time_s"], data["physical_position"],
                                      data["reference"], data["norm_mm"]))
            require(values.shape[1] == len(headers) and np.isfinite(values).all(),
                    "Invalid validated endpoint CSV data")
            path = out/f"{case}_{backend}_endpoint.csv"
            with path.open("x", newline="", encoding="utf-8") as stream:
                writer = csv.writer(stream)
                writer.writerow(headers)
                writer.writerows([[format(float(value), ".17g") for value in row] for row in values])
            artifacts.append(dict(case_id=case, backend=backend, artifact=evidence.identity(path),
                source_native_trace=indexed[(backend, case)]["trace"], samples=len(values),
                numeric_format="17 significant digits; round-trip float64",
                scope="Derived original-window view; t=original sim_s minus recorded LiveStart origin_s; "
                      "physical world tip and original same-time reference; no fitted shift or translation",
                full_raw_native_trace_preserved=True))
    return artifacts


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--reports", nargs=6, type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--no-plots", action="store_true", help="Still verify raw-trace metrics; omit PNG/PDF only")
    args = parser.parse_args(argv)
    require(args.output.is_absolute() and not args.output.exists(), "New absolute output directory required")
    if not args.no_plots:
        try:
            import matplotlib  # Fail before creating partial delivery files.
        except ImportError as error:
            raise RuntimeError("PNG/PDF rendering needs matplotlib; use the prepared analysis environment "
                               "or --no-plots for verified JSON/table only") from error
    evidence = Evidence()
    evidence.identity(Path(__file__),selected=True)
    runs = [load_run(path.resolve(strict=True),evidence) for path in args.reports]
    require(all(not args.output.resolve().is_relative_to(Path(run["report"][key]).resolve())
                for run in runs for key in ("launch_directory", "run_directory")),
            "Summary output must be outside original launch/run evidence")
    indexed, common = check_paired(runs)
    series = {key:native_series(run) for key,run in indexed.items()}
    report = build_summary(indexed,common)
    evidence.finish()
    report.update(selected_evidence_identities=list(evidence.selected.values()),
                  all_transitive_audit_artifacts_verified=len(evidence.cache), created_unix=time.time())
    args.output.mkdir(parents=True)
    report["derived_endpoint_csvs"] = export_endpoint_series(indexed,series,args.output,evidence)
    (args.output/"comparison.json").write_text(json.dumps(report,indent=2,allow_nan=False)+"\n",encoding="utf-8")
    (args.output/"comparison.md").write_text(table_text(report),encoding="utf-8")
    if not args.no_plots:
        plot_errors(indexed,series,args.output)
    evidence.finish()
    generated = [evidence.identity(path) for path in sorted(args.output.iterdir()) if path.is_file()]
    (args.output/"delivery_manifest.json").write_text(json.dumps(dict(kind="x650_upper_comparison_delivery_v1",
        complete=True, artifacts=generated, comparison=evidence.identity(args.output/"comparison.json")),
        indent=2,allow_nan=False)+"\n",encoding="utf-8")
    print(json.dumps(dict(output=str(args.output),paired_cases=3,accepted_runs=6,
        exact_declared_conditions_verified=True,statistical_significance_claimed=False),indent=2))


if __name__ == "__main__":
    main()
