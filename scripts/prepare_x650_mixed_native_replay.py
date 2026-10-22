#!/usr/bin/env python3
"""Derive replay input from an accepted mixed-stage native trace; never simulate."""
from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path

import numpy as np
import plot_x650_mixed_stage_comparison as accepted


MAPPING = {
    "sim_time_s": "sim_s",
    **{"world_"+a+"_m": "base_p"+a for a in "xyz"},
    **{"world_q"+a: "base_q"+a for a in "wxyz"},
    "q1_rad": "arm_joint_1_q", "q2_rad": "arm_joint_2_q",
    **{"tip_world_"+a+"_m": "tip_"+a for a in "xyz"},
    **{key: key for key in ("valid_base", "valid_joint_state", "valid_tip")},
}


def prepare(audit_path, run_dir, output):
    audit_path, folder, output = (Path(p).resolve() for p in (audit_path, run_dir, output))
    accepted.require(not output.exists(), "Use a new output directory; originals are never overwritten")
    header = accepted.read(audit_path)
    accepted.require(header.get("controller") in ("pid_ff", "mpc"), "Unknown mixed controller")
    # Reuses hash/identity/window checks and reproduces the accepted native
    # time-weighted errors; this loader does not draw, fit or optimize.
    run = accepted.load_run(audit_path, folder, header["controller"])
    report, metadata, data = run["report"], run["metadata"], run["physical"]
    inputs = dict(run["inputs"])
    manifest_path = folder/"native_trace_manifest.json"
    bound = {str(Path(p).resolve()): digest for p, digest in report["inputs_sha256"].items()}
    accepted.require(manifest_path.is_file() and bound.get(str(manifest_path)) == accepted.sha(manifest_path),
                     "Original accepted native manifest is missing or changed")
    native_manifest = accepted.read(manifest_path)
    accepted.require(native_manifest.get("complete") is True
                     and native_manifest.get("file") == "native_trace.csv"
                     and native_manifest.get("sha256") == inputs[str(folder/"native_trace.csv")]
                     and native_manifest.get("synthetic") is not True,
                     "Native manifest is incomplete or belongs to another trace")
    for key in ("model_sha256", "source_urdf_sha256"):
        accepted.require(native_manifest.get(key) == report[key], "Native manifest model identity mismatch")
    window = np.asarray(report["window_sim_s"], float)
    accepted.require(np.allclose(native_manifest.get("task_window_sim_s", []), window, atol=1e-9, rtol=0),
                     "Native manifest task window differs from accepted window")
    inputs[str(manifest_path)] = accepted.sha(manifest_path)
    rows = accepted.csv_rows(folder/"native_trace.csv")
    selected = [(index, row) for index, row in enumerate(rows)
                if window[0]-1e-9 <= float(row["sim_s"]) <= window[1]+1e-9]
    accepted.require(len(selected) == len(data["t"]), "Native replay would omit accepted samples")
    geometry = accepted.native_reader.native_tip_audit(data, metadata["physical_profile"])
    targets = run["target"]  # Declared task law at original native times, not a fitted/held command.
    anchor = accepted.audit.vector(metadata["reference_base_enu_m"], 3, "fixed base anchor")
    converted = []
    for (index, original), target in zip(selected, targets):
        row = {destination: original[source] for destination, source in MAPPING.items()}
        row.update(phase="measure", model_sha256=report["model_sha256"], native_row_index=index)
        row.update({"tip_target_world_"+a+"_m": float(target[j]) for j, a in enumerate("xyz")})
        row.update({"target_world_"+a+"_m": float(anchor[j]) for j, a in enumerate("xyz")})
        converted.append(row)
    accepted.require(all(accepted.sha(p) == digest for p, digest in inputs.items()),
                     "Original accepted input changed during conversion")
    output.mkdir(parents=True, exist_ok=False)
    destination = output/"native_replay.csv"
    with destination.open("x", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(converted[0]))
        writer.writeheader()
        writer.writerows(converted)
    accepted.require(all(accepted.sha(p) == digest for p, digest in inputs.items()),
                     "Original accepted input changed during export")
    result = {
        "kind": "x650_mixed_native_derived_replay_input", "schema_version": 1,
        "complete": True, "file": destination.name, "sha256": accepted.sha(destination),
        "model_sha256": report["model_sha256"], "source_urdf_sha256": report["source_urdf_sha256"],
        "controller": report["controller"], "stage": report["stage"],
        "original_flight_recorder_capture": False, "derived_from_native_physics": True,
        "native_gazebo_screen_capture": False, "new_dynamics_simulated": False,
        "eligible_as_stage_gate": False, "new_physical_audit_performed": False,
        "source_run_directory": str(folder), "source_audit": str(audit_path),
        "inputs_sha256": inputs, "sample_count": len(converted),
        "task_window_sim_s": window.tolist(), "time_first_s": float(selected[0][1]["sim_s"]),
        "time_last_s": float(selected[-1][1]["sim_s"]), "all_window_samples_preserved": True,
        "outside_window_rows": len(rows)-len(selected), "field_mapping": MAPPING,
        "original_state_field_text_preserved": True, "mean_subtraction": False,
        "fitted_time_or_phase_alignment": False, "resampling": False,
        "reference_source": "Accepted task law at original sim_s minus predeclared task_start_sim_s",
        "phase_source": "All and only samples inside the accepted measurement window",
        "base_target_semantics": "Frozen world anchor, not the dynamic allocated body setpoint",
        "native_tip_geometry_check": geometry,
        "primary_native_tip_error": report["stage_acceptance"]["native_tip_error"],
        "renderer_metric_policy": "Renderer FK/sample-average metrics are display-only; primary metrics remain the accepted native audit",
        "generator_sha256": accepted.sha(__file__),
        "dependency_sources_sha256": {str(Path(module.__file__).resolve()): accepted.sha(module.__file__)
                                      for module in (accepted, accepted.audit, accepted.native_reader, accepted.precision)},
    }
    (output/"native_replay_manifest.json").write_text(json.dumps(result, indent=2, allow_nan=False)+"\n", encoding="utf-8")
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ("audit", "run", "output"):
        parser.add_argument("--"+name, type=Path, required=True)
    args = parser.parse_args()
    result = prepare(args.audit, args.run, args.output)
    print(json.dumps({"complete": result["complete"], "samples": result["sample_count"],
                      "output": str(args.output.resolve())}))


if __name__ == "__main__":
    main()
