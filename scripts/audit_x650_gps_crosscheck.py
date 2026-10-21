#!/usr/bin/env python3
"""Read-only GPS candidate provenance, effective-PID and fusion audit.

Use alongside analyze_x650_hover_chain.py (WGS84 position decomposition),
analyze_x650_hover_precision.py --native-only, and native position-step analysis.
This script neither flies nor changes parameters, files in a run, or runtime.
"""
from __future__ import annotations
import argparse
import importlib.util
import json
import math
from pathlib import Path
import sys
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(Path(__file__).resolve().parent))
import analyze_x650_hover_precision as precision
import run_x650_precision_candidates as candidates
from diagnostic_parameters import validate_pid_candidate

GPS_FLAGS = {"cs_gnss_pos": 1, "cs_gnss_vel": 1, "cs_gps_hgt": 1, "cs_baro_hgt": 1,
             "cs_ev_pos": 0, "cs_ev_vel": 0, "cs_ev_hgt": 0}


def effective_configuration(ulog, t, reference, originals, override):
    """Use effective window values, never restored values or initial-only gains."""
    origin = precision.origin_and_fusion_audit(ulog, reference, t)
    parameters = precision.effective_parameters(ulog, (float(t[0]), float(t[-1])))
    actual = parameters["parameters_at_measurement_start"]
    expected = {name: override.get(name, record["value"]) for name, record in originals.items()}
    for name, value in expected.items():
        if name not in actual or not math.isclose(actual[name], value, rel_tol=2e-6, abs_tol=1e-7):
            raise ValueError("Effective measurement PID differs from selected GPS candidate: "+name)
    flags = precision.dataset(ulog, "estimator_status_flags")
    names = tuple(GPS_FLAGS)
    st, raw = precision.stamp(flags), precision.fields(flags, names)
    held = precision.resample(st, raw, t, max_gap=1.5, hold=True)
    desired = np.array(list(GPS_FLAGS.values()))
    inside = (st >= t[0]-1e-9)&(st <= t[-1]+1e-9)
    if np.any(held != desired) or np.any(raw[inside] != desired):
        raise ValueError("GPS/barometer/EV fusion differs from the original GPS A1 contract inside the experiment")
    for name, value in {"EKF2_EV_CTRL": 0, "EKF2_GPS_CTRL": 7, "EKF2_BARO_CTRL": 1, "EKF2_HGT_REF": 1}.items():
        if actual.get(name) != value:
            raise ValueError("Canonical GPS estimator parameter mismatch: "+name)
    return {"verified": True, "origin_and_reset": origin, "effective_parameters": parameters,
        "actual_control_42": {name: actual[name] for name in expected},
        "candidate_readback": {name: actual[name] for name in override},
        "fusion_flags_entire_experiment": GPS_FLAGS.copy(),
        "source": "ULog initial + changes before measurement; changes within the entire 56 s experiment rejected"}


def audit(run_dir, baseline_run, output, *, reference_source=precision.DEFAULT_REFERENCE_SOURCE, ulog_path=None):
    folder, baseline, output = Path(run_dir).resolve(strict=True), Path(baseline_run).resolve(strict=True), Path(output).resolve()
    if output.exists():
        raise ValueError("Choose a new audit output directory")
    metadata = candidates.read_json(folder/"controller_metadata.json")
    task = metadata["task"]
    if candidates.canonical_hash(task) != candidates.canonical_hash(candidates.read_json(folder/"controller_task.json")):
        raise ValueError("Task file differs from flight metadata")
    override = validate_pid_candidate(task.get("pid_tuning_candidate"))
    if (override is None or task.get("diagnostic_profile") is not None or task.get("precision_test_profile") != "hover20_steps36_v1"
            or task.get("stage") != "hover_folded" or task.get("controller") != "pid_ff"
            or metadata.get("ground_truth_aided") is not False or metadata.get("aiding_confirmed") is not False):
        raise ValueError("GPS crosscheck requires explicit candidate, ordinary estimator and 20+36 fixed-position protocol")
    metadata, before = candidates.require_completed_run(folder, task)
    baseline_metadata = candidates.read_json(baseline/"controller_metadata.json")
    baseline_before = candidates.control_records(baseline/"px4_parameters_before.json")
    baseline_after = candidates.control_records(baseline/"px4_parameters_after.json")
    if baseline_metadata.get("complete") is not True or baseline_metadata.get("landed_and_disarmed") is not True:
        raise ValueError("Original GPS reference experiment is not completed/recovered")
    if before != baseline_before or baseline_before != baseline_after:
        raise ValueError("All 42 boot/restored control parameters must match original GPS baseline")
    if candidates.physics_identity(metadata) != candidates.physics_identity(baseline_metadata):
        raise ValueError("GPS candidate physics differs from original GPS baseline")
    reference = candidates.read_json(folder/"hover_reference.json")
    precision.verify_reference(reference, reference_source)
    if reference.get("frame_mode") != "global_wgs84":
        raise ValueError("Ordinary GPS run requires global_wgs84 reference conversion")
    original_reference = candidates.read_json(baseline/"hover_reference.json")
    if reference["world"]["source_sha256"] != original_reference["world"]["source_sha256"]:
        raise ValueError("Actual world datum differs from original GPS baseline")
    window = [float(metadata[key]) for key in ("task_start_sim_s", "experiment_end_sim_s")]
    if abs(window[1]-window[0]-56.) > .05 or abs(float(metadata["task_end_sim_s"])-window[0]-20.) > .05:
        raise ValueError("Need complete 20 s hover and 36 s steps")
    manifest = candidates.read_json(folder/"native_trace_manifest.json")
    if (manifest.get("complete") is not True or manifest.get("sha256") != precision.sha(folder/"native_trace.csv")
            or manifest.get("model_sha256") != metadata["model_sha256"]
            or manifest.get("source_urdf_sha256") != metadata["source_urdf_sha256"]):
        raise ValueError("Native trace provenance/completion/hash mismatch")
    precision.capture_window_covers(manifest, window)
    native = precision.native_states(folder/"native_trace.csv", window)
    from pyulog import ULog
    ulog_path = Path(ulog_path or folder/"px4.ulg").resolve(strict=True)
    configuration = effective_configuration(ULog(str(ulog_path)), native["t"], reference, before, override)
    declared = metadata.get("actual_pid_parameters", {})
    for name, value in configuration["actual_control_42"].items():
        if name not in declared or not math.isclose(float(declared[name]), value, rel_tol=2e-6, abs_tol=1e-7):
            raise ValueError("Effective ULog PID differs from controller metadata: "+name)
    files = [folder/name for name in ("controller_task.json", "controller_metadata.json", "hover_reference.json",
             "px4_parameters_before.json", "px4_parameters_after.json", "native_trace.csv", "native_trace_manifest.json")]
    files.extend((ulog_path, Path(reference_source), baseline/"controller_metadata.json", baseline/"px4_parameters_before.json"))
    report = {"kind": "x650_original_gps_candidate_crosscheck_guard", "schema_version": 1, "verified": True,
        "source_run": folder.name, "baseline_run": str(baseline), "experiment_window_sim_s": window,
        "candidate": task["pid_tuning_candidate"], "model_sha256": metadata["model_sha256"],
        "source_urdf_sha256": metadata["source_urdf_sha256"], "ground_truth_aided": False,
        "all_42_boot_and_restored_parameters_match_original_gps": True, "configuration": configuration,
        "inputs_sha256": {str(path): precision.sha(path) for path in files}, "generator_sha256": precision.sha(__file__),
        "scope": "Full-window parameter/fusion/origin guard; use WGS84 hover-chain audit and native step audit for physical errors and response",
        "optimal_pid_claimed": False, "no_fitted_origin_or_phase": True, "no_flight_or_parameter_write": True}
    output.mkdir(parents=True)
    (output/"gps_crosscheck_guard.json").write_text(json.dumps(report, indent=2, allow_nan=False)+"\n", encoding="utf-8")
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-dir", type=Path, required=True)
    parser.add_argument("--baseline-run", type=Path, required=True, help="Completed original GPS A1 run with unchanged 42 control gains")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--reference-source", type=Path, default=precision.DEFAULT_REFERENCE_SOURCE)
    parser.add_argument("--ulog", type=Path)
    args = parser.parse_args()
    report = audit(args.run_dir, args.baseline_run, args.output, reference_source=args.reference_source, ulog_path=args.ulog)
    print(json.dumps({"verified": report["verified"], "source_run": report["source_run"], "output": str(args.output)}))


if __name__ == "__main__":
    main()
