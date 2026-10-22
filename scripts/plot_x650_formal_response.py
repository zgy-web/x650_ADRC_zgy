#!/usr/bin/env python3
"""Display frozen formal-response predictions without fitting or issuing evidence.

Selected original CSV/JSON bytes and their accepted audit chain are verified.
This is an evidence viewer, not another physical/ULog audit. Optional input maps
permit compact local copies without rewriting original reports or path strings.
"""
from __future__ import annotations

import argparse
from copy import deepcopy
import csv
from datetime import datetime
import json
from pathlib import Path
import sys
import zipfile

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Patch
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/"scripts"))
import identify_x650_response_protocol as identifier
from plot_x650_upper_response import BoundInputs, CONTROL_NAMES, name, parent, require
from research_condition import canonical_hash, validate_operating_condition

sha, read_json = identifier.sha, identifier.json_read
AXES = "xyz"
COLORS = {"command": "#58677b", "native": "#1268a3", "prediction": "#a451a0", "residual": "#a14524"}


def merge_artifacts(*collections):
    result = {}
    for collection in collections:
        require(isinstance(collection, dict), "Artifact hashes must form a mapping")
        for path, digest in collection.items():
            identifier.require_hash(digest, path)
            require(path not in result or result[path] == digest, "Conflicting bound artifact digest: "+path)
            result[path] = digest
    return result


def validate_parameters(parameters):
    require(isinstance(parameters, dict) and set(parameters) == set(AXES), "Three frozen axis parameter sets required")
    for axis, values in parameters.items():
        keys = {"tau_s"} if axis == "z" else {"wn_rad_s", "zeta"}
        require(isinstance(values, dict) and set(values) == keys, "Unexpected response model structure")
        require(all(not isinstance(value, bool) and isinstance(value, (int, float))
                    and np.isfinite(value) and value > 0 for value in values.values()),
                "Frozen response parameters must be finite and positive")


def report_axes(report, role):
    axes = report.get("axes", [])
    require([axis.get("axis") for axis in axes] == list(AXES), "Three ordered axis outcomes required")
    require(type(report.get("passed")) is bool and all(type(axis.get("passed")) is bool for axis in axes)
            and report["passed"] == all(axis["passed"] for axis in axes), "Overall/axis acceptance disagree")
    for axis in axes:
        require(axis.get("parameters") == report["parameters"][axis["axis"]], "Axis parameters differ from frozen report")
        require(isinstance(axis.get(role), dict) and "whole_axis_window" in axis[role], "Completed numerical axis results required")
        evaluation=axis[role]
        require(type(evaluation.get("passed")) is bool,"Nested evaluation acceptance must be explicit")
        expected=evaluation["passed"]
        if role=="training":
            fit=axis.get("fit",{})
            require(fit.get("fit_success") is True and type(fit.get("identifiable")) is bool,
                    "Completed numerical training fit decision required")
            expected=expected and fit["identifiable"]
        require(axis["passed"] is expected,"Nested evaluation/fit and axis acceptance disagree")
    return axes


def replay_statistics(data,axis,prediction,window):
    """Verify numerical reproduction only; do not execute acceptance criteria."""
    mask=identifier.interval_mask(data["times"],window)
    require(mask.sum()>=2,"Original scoring interval has insufficient samples")
    residual=prediction[mask]-data["velocities"][mask,axis]
    return {"rmse_m_s":float(np.sqrt(np.mean(residual**2))),
            "maximum_error_m_s":float(np.max(abs(residual))),"bias_m_s":float(np.mean(residual))}


def load_run(summary, role, inputs, identity, protocol):
    folder = summary["folder"]
    audits = []
    for original in inputs.artifacts:
        if name(original) == "upper_stage_audit.json":
            document = inputs.document(original)
            if document.get("source_run_directory") == folder:
                audits.append((original, document))
    require(len(audits) == 1, "Need exactly one bound original formal response audit: "+role)
    audit_path, audit = audits[0]
    flags = ("complete", "passed", "research_acceptance_verified", "full_window_ulog_verified",
             "parameter_restore_verified", "upper_command_verified", "feedback_contract_verified")
    condition = identity["operating_condition"]
    require(audit.get("kind") == "x650_precision_response_protocol_audit" and audit.get("stage") == "response_probe"
            and all(audit.get(flag) is True for flag in flags), "Plot requires a physically audited response run")
    certificates = [path for path in inputs.artifacts if parent(path) == parent(audit_path) and name(path) == "research_stage.json"]
    require(len(certificates) == 1, "Formal response audit needs its original certificate")
    certificate = inputs.document(certificates[0])
    require(certificate.get("kind") == "x650_native_stage_evidence" and certificate.get("stage") == "response_probe"
            and certificate.get("eligible_as_stage_gate") is True and certificate.get("diagnostic_only") is False
            and all(certificate.get(flag) is True for flag in flags), "Formal physical certificate is incomplete")
    for document in (audit, certificate):
        require(document.get("source_run_directory") == folder
                and document.get("source_run") == name(folder)
                and document.get("operating_condition") == condition
                and document.get("operating_condition_sha256") == canonical_hash(condition)
                and document.get("pid_parameters_sha256") == identity["pid_parameters_sha256"]
                and document.get("response_probe_protocol") == protocol
                and document.get("response_probe_protocol_sha256") == identifier.response_protocol_hash(protocol),
                "Physical acceptance differs from the frozen role or operating condition")
    for document, key in ((audit, "inputs_sha256"), (certificate, "artifacts")):
        hashes = document.get(key)
        require(isinstance(hashes, dict) and hashes and all(inputs.artifacts.get(path) == digest for path, digest in hashes.items()),
                "Report does not bind all original accepted audit inputs")
    require(certificate["artifacts"].get(audit_path) == inputs.artifacts[audit_path]
            and any(name(path) == "px4.ulg" for path in audit["inputs_sha256"]), "Exact audit/full ULog chain missing")
    files = {filename: inputs.run_file(folder, filename) for filename in
             ("controller_metadata.json", "controller_task.json", "stage_result.json", "native_trace_manifest.json",
              "native_trace.csv", "control_trace.csv")}
    meta, task, stage, manifest = (inputs.document(files[filename]) for filename in
        ("controller_metadata.json", "controller_task.json", "stage_result.json", "native_trace_manifest.json"))
    require(meta.get("task") == task and task.get("stage") == "response_probe" and task.get("controller") == "pid_ff"
            and meta.get("complete") is True and meta.get("landed_and_disarmed") is True
            and stage.get("complete") is True and stage.get("passed") is True
            and meta.get("eligible_as_stage_gate") is False and meta.get("acceptance_pending") == "full_ulog_audit",
            "Original completed physical run metadata is inconsistent")
    for document in (meta, stage):
        require(document.get("operating_condition") == condition
                and document.get("response_probe_protocol") == protocol
                and document.get("response_probe_protocol_sha256") == identifier.response_protocol_hash(protocol),
                "Original run role/condition differs")
    require(task.get("response_probe_protocol") == protocol and protocol["role"] == role,
            "Original task has another predeclared role")
    actual_pid = meta.get("actual_pid_parameters")
    require(isinstance(actual_pid, dict) and set(actual_pid) == CONTROL_NAMES
            and all(not isinstance(v, bool) and isinstance(v, (int, float)) and np.isfinite(v) for v in actual_pid.values())
            and canonical_hash({k: float(v) for k,v in actual_pid.items()}) == identity["pid_parameters_sha256"],
            "Original actual 42-parameter PID identity differs")
    for key in ("model_sha256", "source_urdf_sha256"):
        require(all(document.get(key) == identity[key] for document in (audit, certificate, meta, stage, manifest)),
                "Physical identity differs: "+key)
    window = np.asarray(audit.get("window_sim_s"), float)
    require(window.shape == (2,) and np.isfinite(window).all() and abs(window[1]-window[0]-60.) <= .05
            and np.allclose(window, [meta["task_start_sim_s"], meta["task_end_sim_s"]], atol=1e-8, rtol=0),
            "Original complete 60-second interval changed")
    native, control = inputs.read_path(files["native_trace.csv"]), inputs.read_path(files["control_trace.csv"])
    require(manifest.get("complete") is True and manifest.get("sha256") == sha(native)
            and sha(control) == summary["command_csv_sha256"], "Native/control source bytes differ from the report")
    def rows(path, measurement=False):
        with path.open(newline="", encoding="utf-8") as stream:
            return [row for row in csv.DictReader(stream) if (not measurement or row["phase"] == "measure")
                    and window[0]-1e-9 <= float(row["sim_s"]) <= window[1]+1e-9]
    nr, cr = rows(native), rows(control, True)
    require(all(row.get(flag) == "1" for row in nr for flag in ("valid_base", "valid_joint_state")),
            "Invalid independent physical observations")
    def columns(records, fields):
        result = np.asarray([[float(row[field]) for field in fields] for row in records])
        require(len(result) >= 2 and np.isfinite(result).all(), "Finite original sampled data required")
        return result
    nt, ct = columns(nr, ["sim_s"]).ravel(), columns(cr, ["sim_s"]).ravel()
    require(nt[0] <= window[0]+.008 and nt[-1] >= window[1]-.008 and np.all(np.diff(nt) > 0)
            and np.max(np.diff(nt)) <= .008+1e-9 and ct[0] <= window[0]+.05 and ct[-1] >= window[1]-.05
            and np.all(np.diff(ct) > 0) and np.max(np.diff(ct)) <= .05+1e-9, "Original complete sampling coverage changed")
    v = columns(nr, ["base_v"+axis for axis in AXES])
    u = columns(cr, ["base_velocity_target_"+axis for axis in AXES])
    joints = columns(nr, ["arm_joint_1_q", "arm_joint_2_q", "arm_joint_1_qd", "arm_joint_2_qd"])
    require(np.max(abs(columns(cr, ["command_acceleration_"+axis for axis in AXES]))) <= 1e-12,
            "Formal response replay requires zero acceleration feedforward")
    nt, ct = nt-window[0], ct-window[0]
    require(np.max(abs(columns(cr, ["t_s"]).ravel()-ct)) <= 1e-8, "Task-time origin changed")
    fingerprint = canonical_hash({"command_times": np.round(ct,9).tolist(), "commands": np.round(u,9).tolist(),
        "truth_times": np.round(nt,9).tolist(), "velocities": np.round(v,9).tolist()})
    require(fingerprint == summary["normalized_measurement_sha256"], "Original measurements differ from fitted/validated input")
    launches=[]
    for original in inputs.artifacts:
        if name(original)=="launch.json":
            document=inputs.document(original)
            if folder in document.get("run_directories",[]):
                launches.append(document)
    require(len(launches)==1 and launches[0].get("complete") is True and launches[0].get("runtime_closed") is True
            and launches[0].get("task")==task,"Original completed launch binding missing")
    return {"folder":folder, "protocol":protocol, "truth_times":nt, "velocities":v,"launch":launches[0],
            "command_times":ct, "commands":u, "joints":joints, "window_sim_s":window.tolist()}


def load_bundle(training_report, *, frozen_candidate=None, validation=None, input_map=None):
    training_path = Path(training_report).resolve(strict=True)
    training = read_json(training_path)
    require(training.get("kind") == "x650_response_training_report" and training.get("schema_version") == 1
            and training.get("complete") is True and training.get("identified") is False
            and training.get("eligible_as_stage_gate") is False
            and training.get("training_passed") is training.get("passed"),
            "Completed numerical TRAIN ONLY report required; incomplete failures cannot yield response curves")
    validate_parameters(training.get("parameters"))
    report_axes(training, "training")
    identity = training["identity"]
    condition = validate_operating_condition(identity.get("operating_condition"))
    require(identity.get("operating_condition_sha256") == canonical_hash(condition)
            and identity.get("pid_parameters_sha256") == condition["pid_parameters_sha256"], "Training condition identity differs")
    mapping = {} if input_map is None else read_json(input_map)
    validation_path = None if validation is None else Path(validation).resolve(strict=True)
    validated = None if validation_path is None else read_json(validation_path)
    extra = {training["preregistration_path"]:training["preregistration_sha256"]}
    artifacts = merge_artifacts(training["artifacts"], extra, {} if validated is None else validated.get("artifacts",{}))
    inputs = BoundInputs(training_path.parent, {"artifacts":artifacts}, mapping)
    plan = inputs.document(training["preregistration_path"])
    require(plan.get("kind") == "x650_response_preregistration"
            and plan.get("protocols") == {role:identifier.build_response_protocol(role) for role in ("train","holdout")}
            and plan.get("analysis_sources_sha256") == training.get("analysis_sources_sha256"),
            "Training report differs from its original preregistered plan")
    predictor_sources = [Path(identifier.__file__).resolve(), ROOT/"scripts/compare_x650_response_models.py",
                         ROOT/"scripts/identify_x650_upper_response.py", ROOT/"simulations/x650_wholebody_mpc_v1/response_protocol.py"]
    for source in predictor_sources:
        matches = [digest for original,digest in training["analysis_sources_sha256"].items() if name(original) == source.name]
        require(matches == [sha(source)], "Replay predictor source differs from the original report: "+source.name)
    archive = plan.get("source_archive",{})
    inputs.artifacts = merge_artifacts(inputs.artifacts,{archive["path"]:archive["sha256"]})
    archive_path = inputs.read_path(archive["path"])
    with zipfile.ZipFile(archive_path) as zipped:
        require(set(zipped.namelist()) == set(archive["members"]), "Frozen source archive member set differs")
        import hashlib
        for member, record in archive["members"].items():
            require(hashlib.sha256(zipped.read(member)).hexdigest() == record["sha256"]
                    and plan["source_sha256"].get(record["source_path"]) == record["sha256"], "Frozen source archive differs")
    candidate = None
    if validated is not None:
        require(validated.get("kind") == "x650_native_stage_evidence" and validated.get("schema_version") == 3
                and validated.get("stage") == "response_identification" and validated.get("complete") is True
                and validated.get("identified") is validated.get("passed")
                and validated.get("eligible_as_stage_gate") is validated.get("passed"), "Completed independent validation required")
        report_axes(validated,"holdout")
        require(validated.get("parameters") == training["parameters"] and validated.get("training") == training["training"]
                and validated.get("preregistration_path") == training["preregistration_path"]
                and validated.get("preregistration_sha256") == training["preregistration_sha256"]
                and validated.get("analysis_sources_sha256") == training["analysis_sources_sha256"]
                and all(validated.get(key) == value for key,value in identity.items()), "Validation differs from frozen training identity")
        original = validated["frozen_candidate_path"]
        require(inputs.artifacts.get(original) == validated["frozen_candidate_sha256"], "Validation candidate digest chain missing")
        candidate_path = inputs.read_path(original)
        if frozen_candidate is not None:
            require(sha(frozen_candidate) == sha(candidate_path), "Explicit candidate differs from validated candidate")
        candidate = read_json(candidate_path)
    elif frozen_candidate is not None:
        candidate_path = Path(frozen_candidate).resolve(strict=True)
        candidate = read_json(candidate_path)
        inputs.used[str(candidate_path)] = sha(candidate_path)
    if candidate is not None:
        require(training["passed"] is True and candidate.get("kind") == "x650_frozen_response_candidate"
                and candidate.get("schema_version") == 1
                and candidate.get("training_passed") is True and candidate.get("identified") is False
                and candidate.get("eligible_as_stage_gate") is False
                and candidate.get("training_report_sha256") == sha(training_path)
                and candidate.get("parameters") == training["parameters"] and candidate.get("identity") == identity
                and candidate.get("training") == training["training"] and candidate.get("protocols") == plan["protocols"]
                and candidate.get("preregistration_sha256") == training["preregistration_sha256"]
                and candidate.get("analysis_sources_sha256") == training["analysis_sources_sha256"]
                and identifier.valid_created_time(candidate.get("created_unix")),
                "Candidate is not the unchanged training-only freeze")
    runs = []
    for role, report, summary in [("train",training,training["training"])] + ([] if validated is None else [("holdout",validated,validated["holdout"])]):
        run = load_run(summary,role,inputs,identity,plan["protocols"][role])
        launch_time=datetime.fromisoformat(run["launch"]["started_utc"].replace("Z","+00:00")).timestamp()
        finish_time=datetime.fromisoformat(run["launch"]["finished_utc"].replace("Z","+00:00")).timestamp()
        require(plan["created_unix"] < launch_time <= finish_time,"Response flight chronology differs from its preregistration")
        if role=="train" and candidate is not None:
            require(finish_time < candidate["created_unix"],"Candidate was not frozen after completed training")
        if role=="holdout":
            require(candidate is not None and launch_time > candidate["created_unix"],"Holdout predates its frozen candidate")
            archived=run["launch"].get("frozen_response_candidate_archive",{})
            require(archived.get("sha256")==validated["frozen_candidate_sha256"]
                    and inputs.artifacts.get(archived.get("path"))==archived.get("sha256"),
                    "Holdout lacks its original hash-bound candidate archive")
            inputs.read_path(archived["path"])
        data = identifier.prepare_run(run)
        status = ("TRAINING ACCEPTED / TRAIN ONLY" if report["passed"] else "REJECTED / TRAIN ONLY") if role == "train" else ("HOLDOUT PASS" if report["passed"] else "HOLDOUT REJECTED")
        role_panels = []
        for index, axis in enumerate(AXES):
            prediction = identifier.predict(data,index,training["parameters"][axis])
            reported_axis=report["axes"][index]
            evaluation=reported_axis["training" if role == "train" else "holdout"]
            original = evaluation["whole_axis_window"]
            reproduced = replay_statistics(data,index,prediction,original["window_s"])
            for key in ("rmse_m_s","maximum_error_m_s","bias_m_s"):
                require(abs(reproduced[key]-original[key]) <= 1e-10, "Frozen prediction does not reproduce original "+role+" "+axis+" "+key)
            role_panels.append({"axis":index,"name":axis,"prediction":prediction,"statistics":original,
                                "axis_passed":reported_axis["passed"],
                                "reported_diagnostics":{"role":role,"axis":axis,"axis_passed":reported_axis["passed"],
                                    "failure_reasons":deepcopy(evaluation.get("reasons",[])),
                                    "fit":deepcopy(reported_axis.get("fit")),"evaluation":deepcopy(evaluation)}})
        runs.append({"role":role,"status":status,"passed":report["passed"],"run":run,"data":data,"panels":role_panels})
    if len(runs) == 2:
        require(training["training"]["folder"] != validated["holdout"]["folder"]
                and training["training"]["normalized_measurement_sha256"] != validated["holdout"]["normalized_measurement_sha256"],
                "A copied training record cannot be displayed as an independent holdout")
    used = {**inputs.used,str(training_path):sha(training_path),**{str(p):sha(p) for p in predictor_sources}}
    if validation_path is not None: used[str(validation_path)] = sha(validation_path)
    if input_map is not None: used[str(Path(input_map).resolve())] = sha(input_map)
    return {"training":training,"validation":validated,"runs":runs,"inputs_sha256":used,
            "original_bound_artifacts":inputs.originals,"parameters":training["parameters"]}


def render(bundle, output):
    output = Path(output).resolve()
    require(not output.exists(), "Use a new plot directory; previous output is never overwritten")
    output.mkdir(parents=True)
    files = []
    for item in bundle["runs"]:
        data, protocol = item["data"], item["data"]["protocol"]
        fig, axes = plt.subplots(3,2,figsize=(15,10))
        fig.subplots_adjust(left=.075,right=.985,bottom=.13,top=.87,hspace=.52,wspace=.24)
        for panel,row in zip(item["panels"],axes):
            axis = panel["axis"]
            start,end = protocol["axis_windows_s"][axis]
            native_mask = identifier.interval_mask(data["times"],(start,end))
            command_mask = identifier.interval_mask(data["command_times"],(start,end))
            times = data["times"][native_mask]
            actual = data["velocities"][native_mask,axis]
            prediction = panel["prediction"][native_mask]
            for ax in row:
                ax.set_facecolor("#f7f9fa")
                for pulse in protocol["local_pulses"]:
                    ax.axvspan(start+pulse["start_s"],start+pulse["end_s"],color="#e5b96c",alpha=.22,zorder=0)
                ax.axhline(0,color="#6d7783",lw=.7,alpha=.8)
                ax.grid(axis="y",alpha=.2)
                ax.set_xlim(start,end)
                ax.set_xlabel("Original task time (s)")
            row[0].step(data["command_times"][command_mask],data["commands"][command_mask,axis]*1000,
                        where="post",color=COLORS["command"],lw=1.2,label="Recorded logical velocity command")
            row[0].plot(times,actual*1000,color=COLORS["native"],lw=1.7,label="Independent native velocity")
            row[0].plot(times,prediction*1000,color=COLORS["prediction"],lw=1.4,ls="--",label="Frozen prediction (no refit)")
            row[1].plot(times,(prediction-actual)*1000,color=COLORS["residual"],lw=1.2)
            stats=panel["statistics"]
            decision="PASS" if panel["axis_passed"] else "REJECTED"
            model="second order" if axis<2 else "first order"
            row[0].set_title(f"{panel['name'].upper()} | {model} | {decision}",loc="left",weight="bold")
            row[0].set_ylabel("Velocity (mm/s)")
            row[1].set_ylabel("Prediction - native (mm/s)")
            row[1].set_title(f"Scored {stats['window_s'][0]:g}-{stats['window_s'][1]:g} s: "
                             f"RMSE {stats['rmse_m_s']*1000:.3f} mm/s; max {stats['maximum_error_m_s']*1000:.3f} mm/s",
                             loc="left",fontsize=10)
        handles,labels=axes[0,0].get_legend_handles_labels()
        handles += [Patch(facecolor="#e5b96c",alpha=.3),Patch(facecolor="#f7f9fa",edgecolor="#c4cbd0")]
        labels += ["Predeclared active pulse","Zero-velocity rest"]
        fig.legend(handles,labels,loc="lower center",bbox_to_anchor=(.53,.025),ncol=3,fontsize=9,frameon=False)
        fig.text(.5,.012,"Original fit, segment and delay decisions/reasons: render_manifest.json; no acceptance recalculated.",
                 ha="center",fontsize=8,color="#59616d")
        title_color="#9d3325" if not item["passed"] else "#25594a"
        fig.suptitle("X650 formal response | "+item["status"]+"\n"
            "Original timestamps; continuous prediction from first native v, a0=0; display only",
            fontsize=15,weight="bold",color=title_color)
        for suffix in ("png","pdf"):
            path=output/("formal_response_"+item["role"]+"."+suffix)
            fig.savefig(path,dpi=170,metadata={"Creator":"plot_x650_formal_response.py"} if suffix=="pdf" else None)
            files.append({"path":str(path),"bytes":path.stat().st_size,"sha256":sha(path)})
        plt.close(fig)
    renderer_sources={str(Path(__file__).resolve()):sha(__file__),
                      str(ROOT/"scripts/plot_x650_upper_response.py"):sha(ROOT/"scripts/plot_x650_upper_response.py")}
    manifest={"kind":"x650_formal_response_render_manifest","schema_version":1,"complete":True,
        "display_only":True,"eligible_as_stage_gate":False,"fitting_performed":False,
        "full_ulog_audit_repeated":False,"inputs_sha256":bundle["inputs_sha256"],
        "original_bound_artifacts":bundle["original_bound_artifacts"],"renderer_sources_sha256":renderer_sources,
        "parameters":bundle["parameters"],"outcomes":[{key:item[key] for key in ("role","status","passed")} for item in bundle["runs"]],
        "axis_reported_diagnostics":[panel["reported_diagnostics"] for item in bundle["runs"] for panel in item["panels"]],
        "acceptance_recomputed":False,"reported_diagnostics_copied_without_reinterpretation":True,
        "prediction_window_s":[0.,54.],"anchor_return_54_60_s_plotted":False,"files":files,
        "scope":"Original accepted physical/audit chain plus selected replay bytes verified; no new model acceptance is issued"}
    identifier.save_new(output/"render_manifest.json",manifest)
    return manifest


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--training-report",type=Path,required=True)
    parser.add_argument("--frozen-candidate",type=Path)
    parser.add_argument("--validation",type=Path)
    parser.add_argument("--input-map",type=Path)
    parser.add_argument("--output",type=Path,required=True)
    args=parser.parse_args(argv)
    bundle=load_bundle(args.training_report,frozen_candidate=args.frozen_candidate,
                       validation=args.validation,input_map=args.input_map)
    manifest=render(bundle,args.output)
    print(json.dumps({"output":str(args.output),"outcomes":manifest["outcomes"],"files":manifest["files"]},indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
