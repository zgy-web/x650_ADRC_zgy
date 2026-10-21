#!/usr/bin/env python3
"""Replay an existing upper-response identification as six scientific panels.

No fit is run. Frozen training time constants are replayed at the original
native observation times with the identifier's exact command-event predictor.
Complete but rejected models remain plottable and explicitly labeled rejected.
An optional JSON input map relocates individual bound source files without
editing the identification, its audits, or their original absolute paths.
"""
from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path, PurePosixPath
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
for directory in (ROOT/"scripts", ROOT/"simulations/x650_wholebody_mpc_v1"):
    sys.path.insert(0, str(directory))
import identify_x650_upper_response as identifier
from diagnostic_parameters import load_parameter_evidence
from research_condition import canonical_hash, validate_operating_condition

sha, read_json = identifier.sha, identifier.json_read
CONTROL_NAMES = frozenset(load_parameter_evidence().CORE_CONTROL_PARAMETERS)
COLORS = {"command": "#536176", "native": "#166aab", "prediction": "#ce691f"}


def require(value, message):
    if not value:
        raise ValueError(message)


def name(path):
    return PurePosixPath(str(path).replace("\\", "/")).name


def parent(path):
    return str(PurePosixPath(str(path).replace("\\", "/")).parent)


class BoundInputs:
    """Read only selected original bytes certified by the identification.

    This verifies replay inputs and existing acceptance records; it does not
    repeat the full ULog audit. ULog and source-archive hashes remain bound by
    those records, so a compact local copy need not transfer the whole ULog.
    """
    def __init__(self, directory, report, mapping=None):
        self.directory = Path(directory).resolve()
        self.artifacts = report.get("artifacts")
        require(isinstance(self.artifacts, dict) and bool(self.artifacts), "Identification needs hashed input artifacts")
        for path, digest in self.artifacts.items():
            identifier.require_hash(digest, path)
        self.mapping = {} if mapping is None else dict(mapping)
        require(all(isinstance(k, str) and isinstance(v, str) for k, v in self.mapping.items()), "Input map must map original paths to local paths")
        self.used, self.originals = {}, {}

    def read_path(self, original):
        require(original in self.artifacts, "Input is not bound by the identification: "+original)
        candidate = Path(self.mapping.get(original, original))
        if original not in self.mapping and not candidate.is_absolute():
            candidate = self.directory/candidate
        candidate = candidate.resolve()
        expected = self.artifacts[original]
        require(candidate.is_file() and sha(candidate) == expected, "Bound replay input missing or changed: "+original)
        self.used[str(candidate)] = expected
        self.originals[original] = expected
        return candidate

    def document(self, original):
        return read_json(self.read_path(original))

    def run_file(self, folder, filename):
        candidates = [path for path in self.artifacts if parent(path) == str(folder).replace("\\", "/") and name(path) == filename]
        require(len(candidates) == 1, "Need one bound run artifact: "+filename)
        return candidates[0]


def load_run_replay(role, report, inputs, condition):
    summary = report[role]
    folder = summary["folder"]
    candidates = []
    for original in inputs.artifacts:
        if name(original) == "upper_stage_audit.json":
            document = inputs.document(original)
            if document.get("source_run_directory") == folder:
                candidates.append((original, document))
    require(len(candidates) == 1, "Need exactly one original upper audit for "+role)
    audit_path, audit = candidates[0]
    flags = ("complete", "passed", "research_acceptance_verified", "full_window_ulog_verified",
             "parameter_restore_verified", "upper_command_verified", "feedback_contract_verified")
    require(audit.get("kind") == "x650_precision_upper_stage_audit" and audit.get("stage") == "response_probe"
            and all(audit.get(flag) is True for flag in flags), "Replay requires an independently accepted physical response run")
    require(audit.get("operating_condition") == condition and audit.get("operating_condition_sha256") == canonical_hash(condition)
            and audit.get("pid_parameters_sha256") == condition["pid_parameters_sha256"], "Upper audit differs from identification condition")
    certificates = [path for path in inputs.artifacts if parent(path) == parent(audit_path) and name(path) == "research_stage.json"]
    require(len(certificates) == 1, "Accepted response audit needs its bound research certificate")
    certificate = inputs.document(certificates[0])
    require(certificate.get("kind") == "x650_native_stage_evidence" and certificate.get("stage") == "response_probe"
            and certificate.get("source_run_directory") == folder and certificate.get("source_run") == audit.get("source_run")
            and certificate.get("diagnostic_only") is False and certificate.get("ground_truth_aided") is True
            and certificate.get("eligible_as_stage_gate") is True and all(certificate.get(flag) is True for flag in flags)
            and certificate.get("operating_condition") == condition
            and certificate.get("operating_condition_sha256") == canonical_hash(condition), "Accepted response certificate differs from original audit")
    for document, key in ((certificate, "artifacts"), (audit, "inputs_sha256")):
        values = document.get(key)
        require(isinstance(values, dict) and bool(values)
                and all(inputs.artifacts.get(path) == digest for path, digest in values.items()),
                "Identification does not bind the complete accepted audit's original inputs")
    require(certificate["artifacts"].get(audit_path) == inputs.artifacts[audit_path], "Certificate does not bind this exact upper audit")
    require(any(name(path) == "px4.ulg" for path in audit["inputs_sha256"]), "Original full ULog audit is missing")
    paths = {filename: inputs.run_file(folder, filename) for filename in
             ("controller_metadata.json", "controller_task.json", "stage_result.json", "native_trace.csv",
              "native_trace_manifest.json", "control_trace.csv")}
    meta, task, stage, manifest = (inputs.document(paths[filename]) for filename in
        ("controller_metadata.json", "controller_task.json", "stage_result.json", "native_trace_manifest.json"))
    require(meta.get("task") == task and task.get("stage") == "response_probe" and task.get("controller") == "pid_ff"
            and meta.get("complete") is True and meta.get("landed_and_disarmed") is True
            and stage.get("complete") is True and stage.get("passed") is True
            and meta.get("eligible_as_stage_gate") is False and meta.get("acceptance_pending") == "full_ulog_audit"
            and meta.get("operating_condition") == condition, "Original run metadata differs from its accepted research scope")
    actual = meta.get("actual_pid_parameters")
    require(isinstance(actual, dict) and set(actual) == CONTROL_NAMES
            and all(not isinstance(value, bool) and isinstance(value, (int, float)) and np.isfinite(value) for value in actual.values())
            and canonical_hash({key: float(value) for key, value in actual.items()}) == condition["pid_parameters_sha256"],
            "Actual complete 42-value PID identity differs")
    window = np.asarray(audit.get("window_sim_s"), float)
    require(window.shape == (2,) and np.isfinite(window).all() and abs(window[1]-window[0]-30.) <= .05
            and np.allclose(window, [meta["task_start_sim_s"], meta["task_end_sim_s"]], atol=1e-8, rtol=0),
            "Plot window differs from complete accepted response window")
    native_path, command_path = inputs.read_path(paths["native_trace.csv"]), inputs.read_path(paths["control_trace.csv"])
    require(manifest.get("complete") is True and manifest.get("sha256") == sha(native_path)
            and sha(command_path) == summary["command_csv_sha256"], "Native/command trace differs from identification input")
    for key in ("model_sha256", "source_urdf_sha256"):
        require(all(document.get(key) == report[key] for document in (audit, certificate, meta, stage, manifest)), "Physical identity differs: "+key)
    def selected(path, *, phase=False):
        with path.open(newline="", encoding="utf-8") as stream:
            return [row for row in csv.DictReader(stream) if (not phase or row["phase"] == "measure")
                    and window[0]-1e-9 <= float(row["sim_s"]) <= window[1]+1e-9]
    native_rows, commands = selected(native_path), selected(command_path, phase=True)
    def array(records, fields):
        value = np.asarray([[float(row[field]) for field in fields] for row in records])
        require(len(records) >= 2 and np.isfinite(value).all(), "Replay needs finite original samples")
        return value
    nt = array(native_rows, ("sim_s",)).ravel()
    ct = array(commands, ("sim_s",)).ravel()
    require(np.all(np.diff(nt) > 0) and np.diff(nt).max() <= .008+1e-9
            and nt[0] <= window[0]+.008 and nt[-1] >= window[1]-.008
            and np.all(np.diff(ct) > 0) and np.diff(ct).max() <= .05+1e-9
            and ct[0] <= window[0]+.05 and ct[-1] >= window[1]-.05, "Original native/control coverage changed")
    require(all(row.get(flag) == "1" for row in native_rows for flag in ("valid_base", "valid_joint_state")), "Invalid physical observation flags")
    velocity = array(native_rows, ["base_v"+axis for axis in "xyz"])
    command = array(commands, ["base_velocity_target_"+axis for axis in "xyz"])
    require(np.max(abs(array(commands, ["command_acceleration_"+axis for axis in "xyz"]))) <= 1e-12,
            "Response replay requires original zero acceleration FF")
    nt, ct = nt-window[0], ct-window[0]
    require(np.max(abs(array(commands, ("t_s",)).ravel()-ct)) <= 1e-8, "Declared task-time origin changed")
    fingerprint = identifier.json_hash({"command_times": np.round(ct, 9).tolist(), "commands": np.round(command, 9).tolist(),
        "truth_times": np.round(nt, 9).tolist(), "velocities": np.round(velocity, 9).tolist()})
    require(fingerprint == summary["normalized_measurement_sha256"], "Original replay observations differ from fitted/held-out data")
    return {"role": role, "folder": folder, "window_sim_s": window.tolist(), "command_times": ct, "commands": command,
            "truth_times": nt, "velocities": velocity, "normalized_measurement_sha256": fingerprint}


def load_identification(directory, input_map=None):
    directory = Path(directory).resolve()
    report_path = directory/"response_identification.json"
    report = read_json(report_path)
    require(report.get("kind") == "x650_native_stage_evidence" and report.get("stage") == "response_identification"
            and report.get("schema_version") == 2 and report.get("complete") is True,
            "Need a completed v2 identification report; incomplete/unbound failures cannot support response curves")
    require(type(report.get("passed")) is bool and report.get("identified") is report["passed"], "Identification outcome is ambiguous")
    condition = validate_operating_condition(report.get("operating_condition"))
    require(condition.get("research_profile") == "precision_ev_upper_v2"
            and report.get("operating_condition_sha256") == canonical_hash(condition), "Identification condition differs")
    inputs = BoundInputs(directory, report, {} if input_map is None else read_json(input_map))
    model = inputs.document("response_model.json")
    require(model.get("kind") == "x650_upper_velocity_response_model" and model.get("schema_version") == 2
            and model.get("passed") is report["passed"] and model.get("identified") is report["passed"]
            and model.get("operating_condition") == condition
            and model.get("operating_condition_sha256") == canonical_hash(condition)
            and model.get("base_time_constant_s") == report.get("base_time_constant_s"), "Frozen response model differs from identification report")
    require(report.get("identifier_source_sha256") == model.get("identifier_source_sha256") == sha(identifier.__file__),
            "Replay predictor must match the identifier source that generated the report")
    for document in (report, model):
        require(document.get("gain") == [1., 1., 1.] and document.get("bias_m_s") == [0., 0., 0.]
                and document.get("command_delay_s") == 0., "Only original unit-gain zero-delay model replay is supported")
    require([item.get("axis") for item in report.get("axes", [])] == list("xyz"), "Identification needs its three declared axis reports")
    taus = model["base_time_constant_s"]
    require(isinstance(taus, list) and len(taus) == 3, "Frozen axis time constants are missing")
    for tau, axis in zip(taus, report["axes"]):
        require(tau == axis.get("base_time_constant_s") and (tau is None or (not isinstance(tau, bool) and np.isfinite(tau) and tau > 0)),
                "Frozen time constant differs from its training axis result")
    runs = [load_run_replay(role, report, inputs, condition) for role in ("training", "holdout")]
    require(runs[0]["folder"] != runs[1]["folder"]
            and runs[0]["normalized_measurement_sha256"] != runs[1]["normalized_measurement_sha256"], "A copied run cannot serve as holdout")
    panels = []
    for axis, (tau, axis_report) in enumerate(zip(taus, report["axes"])):
        start, end = 2.+4*axis, 6.+4*axis
        for run in runs:
            mask = (run["truth_times"] >= start-1e-9)&(run["truth_times"] <= end+1e-9)
            times, actual = run["truth_times"][mask], run["velocities"][mask, axis]
            require(len(times) >= 2, "Axis window has no usable original physical observations")
            prediction = None if tau is None else identifier.predict_velocity(tau, run["command_times"],
                run["commands"][:, axis], times, actual[0])
            statistics = axis_report.get(run["role"])
            if statistics is not None:
                require(prediction is not None and abs(float(np.sqrt(np.mean((prediction-actual)**2)))-statistics["rmse_m_s"]) <= 1e-10,
                        "Frozen prediction no longer reproduces the original report RMSE")
            panels.append(dict(axis=axis, role=run["role"], start_s=start, end_s=end, tau_s=tau,
                times=times, actual=actual, prediction=prediction, command_times=run["command_times"],
                commands=run["commands"][:, axis], statistics=statistics, axis_passed=axis_report.get("passed") is True))
    used = {**inputs.used, str(report_path): sha(report_path), str(Path(identifier.__file__).resolve()): sha(identifier.__file__)}
    if input_map is not None:
        used[str(Path(input_map).resolve())] = sha(input_map)
    return dict(report=report, model=model, runs=runs, panels=panels, inputs_sha256=used,
                original_bound_artifacts=inputs.originals, identification_directory=str(directory))


def render(bundle, output):
    output = Path(output).resolve()
    require(not output.exists(), "Use a new plot directory; existing results are never overwritten")
    output.mkdir(parents=True)
    accepted = bundle["report"]["passed"]
    status = "MODEL ACCEPTED" if accepted else "MODEL REJECTED"
    with plt.rc_context({"font.family": "DejaVu Sans", "font.size": 10., "axes.titlesize": 11.,
                         "axes.labelsize": 10., "pdf.fonttype": 42, "axes.spines.top": False, "axes.spines.right": False}):
        fig, axes = plt.subplots(3, 2, figsize=(14.8, 12.2), sharey="row")
        fig.subplots_adjust(left=.075, right=.98, bottom=.14, top=.80, hspace=.72, wspace=.17)
        fig.suptitle("X650 upper-control response identification", x=.075, y=.976, ha="left", fontsize=19, fontweight="bold")
        fig.text(.075, .94, status+"  |  Frozen training time constants; independent holdout",
                 color="#176546" if accepted else "#b2212e", fontsize=14, fontweight="bold")
        fig.text(.075, .909, "Precision EV / folded arm / zero acceleration feedforward / original native physics samples", color="#465366")
        fig.text(.075, .881, "Zero-delay, unit-gain model. No refit, time shift, recentering or curve downsampling. Not an MPC result.", fontsize=9, color="#465366")
        for panel in bundle["panels"]:
            axis, column = panel["axis"], 0 if panel["role"] == "training" else 1
            ax = axes[axis, column]
            for offset, alpha in ((0., .6), (2., .4)):
                ax.axvspan(panel["start_s"]+offset, panel["start_s"]+offset+1., color="#e6edf5", alpha=alpha, zorder=0)
            ax.step(panel["command_times"], panel["commands"], where="post", color=COLORS["command"], lw=1.35, alpha=.9)
            ax.plot(panel["times"], panel["actual"], color=COLORS["native"], lw=1.6)
            if panel["prediction"] is not None:
                ax.plot(panel["times"], panel["prediction"], color=COLORS["prediction"], lw=1.55, ls="--")
            ax.axhline(0., color="#a2aab4", lw=.6)
            ax.set_xlim(panel["start_s"], panel["end_s"])
            ax.set_xticks(np.arange(panel["start_s"], panel["end_s"]+.01, 1.))
            ax.grid(True, color="#e4e8ed", lw=.65)
            coordinate = ("X / east", "Y / north", "Z / up")[axis]
            ax.set_title(f"{coordinate}  -  {'Training' if column == 0 else 'Independent holdout'}", loc="left", fontweight="bold", pad=33)
            stats = panel["statistics"]
            annotation = (f"Report RMSE {stats['rmse_m_s']:.4f} m/s  |  nRMSE {stats['nrmse_command_half_amplitude']:.3f}"
                          if stats is not None else "No valid frozen prediction: excitation/identification rejected")
            tau_label = "unavailable" if panel["tau_s"] is None else f"{panel['tau_s']:.4f} s"
            ax.text(0., 1.035, annotation+f"\nTraining tau: {tau_label}  |  Axis {'PASS' if panel['axis_passed'] else 'REJECTED'}",
                    transform=ax.transAxes, va="bottom", fontsize=8.3, color="#4a5665")
            if axis == 2:
                ax.set_xlabel("Task time [s]", fontsize=9)
            if column == 0:
                ax.set_ylabel("World-ENU velocity [m/s]")
        handles = [Line2D([0], [0], color=COLORS[key], lw=1.8, ls="--" if key == "prediction" else "-", label=label)
                   for key, label in (("command", "Commanded logical velocity u (ZOH)"),
                                      ("native", "Independent native physical velocity"),
                                      ("prediction", "Frozen training-tau prediction"))]
        fig.legend(handles=handles, loc="lower center", bbox_to_anchor=(.53, .058), ncol=3, frameon=False, fontsize=9)
        fig.text(.075, .032, "Each prediction starts from the first native velocity in its original 4 s axis window; displayed RMSE is copied from the ID report.", fontsize=8, color="#566273")
        if not accepted:
            fig.text(.98, .014, "MODEL REJECTED - this figure does not authorize a subsequent control stage", ha="right", fontsize=9, color="#b2212e", fontweight="bold")
        png, pdf = output/"upper_response_identification.png", output/"upper_response_identification.pdf"
        fig.savefig(png, dpi=240, facecolor="white")
        fig.savefig(pdf, facecolor="white", metadata={"Title": "X650 upper response identification - "+status})
        plt.close(fig)
    manifest = dict(kind="x650_upper_response_plot", schema_version=1, complete=True,
        identification_passed=accepted, figure_status=status, base_time_constant_s=bundle["model"]["base_time_constant_s"],
        operating_condition=bundle["report"]["operating_condition"], operating_condition_sha256=bundle["report"]["operating_condition_sha256"],
        no_refit=True, no_time_shift=True, no_recenter=True, no_curve_downsampling=True,
        displayed_metrics_source="Copied original response_identification.json metrics; replay RMSE checked at original full-resolution samples",
        verification_scope="Hash-bound replay of two complete independently accepted native/ULog response audits; ULog is not re-audited by plotting",
        plot_windows_task_s=[[2., 6.], [6., 10.], [10., 14.]],
        input_paths_sha256=bundle["inputs_sha256"], original_bound_artifacts=bundle["original_bound_artifacts"],
        generator_sha256=sha(__file__), predictor_sha256=sha(identifier.__file__),
        outputs_sha256={png.name: sha(png), pdf.name: sha(pdf)},
        mpc_performance_claim=False, endpoint_compensation_claim=False, hardware_performance_claim=False)
    (output/"plot_manifest.json").write_text(json.dumps(manifest, indent=2, allow_nan=False)+"\n", encoding="utf-8")
    return manifest


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--identification-dir", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--input-map", type=Path, help="JSON object: original bound path -> verified local copy (optional)")
    args = parser.parse_args(argv)
    result = render(load_identification(args.identification_dir, args.input_map), args.output)
    print(json.dumps({"output": str(args.output.resolve()), "status": result["figure_status"],
                      "base_time_constant_s": result["base_time_constant_s"]}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
