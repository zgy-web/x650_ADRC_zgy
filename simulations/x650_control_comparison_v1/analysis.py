#!/usr/bin/env python3
"""Truth-based metrics and an auditable report for the position-outer-loop trials.

Only this offline module reads Gazebo truth for scoring. No ROS imports or
controller code are needed. Run ``python analysis.py <batch directory>`` after
the batch; incomplete and failed runs remain in the report.
"""
from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
from pathlib import Path
from typing import Iterable

import numpy as np


DURATIONS = {"hover": 30., "step": 35., "arm": 30., "force": 30., "z_smoke": 20.}
ALIASES = {f"{case}{int(duration)}": case for case, duration in DURATIONS.items()}
AXES = "xyz"
LIMITS = {"position_rmse_m": .25, "position_max_m": .5, "tilt_max_deg": 15.}
RECOVERY_BAND_M = .05
RECOVERY_HOLD_S = 2.
MAX_RECOVERY_SAMPLE_GAP_S = .15
WINDOWS = {
    "force": {"pre": (5., 10.), "active": (10., 12.), "after": (12., 30.), "response": (10., 20.)},
    "arm": {"pre": (0., 5.), "active": (5., 13.), "after": (13., 30.)},
}
TRACE_TIMING_COLUMNS = (
    "dt", "position_receipt_age_s", "joint_receipt_age_s", "truth_receipt_age_s",
    "clock_receipt_age_s", "wall_send_interval_s", "clock_no_progress_max_wall_s", "host_loadavg_1m",
)


def _case_name(case: str) -> str:
    return ALIASES.get(case, case)


def _number(row: dict, key: str) -> float:
    try:
        value = float(row[key])
    except (KeyError, TypeError, ValueError) as exc:
        raise ValueError(f"Missing or invalid trace column {key}") from exc
    if not math.isfinite(value):
        raise ValueError(f"Nonfinite trace column {key}")
    return value


def _boolean(value) -> bool:
    if isinstance(value, str):
        if value.lower() in ("true", "1"):
            return True
        if value.lower() in ("false", "0"):
            return False
        raise ValueError(f"Invalid boolean {value!r}")
    if value not in (True, False, 0, 1):
        raise ValueError(f"Invalid boolean {value!r}")
    return bool(value)


def _time_average(values: np.ndarray, times: np.ndarray):
    """Trapezoidal time mean; no extrapolation beyond observed sample stamps."""
    if len(times) < 2 or times[-1] <= times[0]:
        raise ValueError("At least two distinct increasing sample times are required")
    shape = (-1,) + (1,) * (values.ndim - 1)
    weights = np.diff(times).reshape(shape)
    return np.sum(.5 * (values[:-1] + values[1:]) * weights, axis=0) / (times[-1] - times[0])


def _measurement_rows(rows: Iterable[dict]):
    control = [r for r in rows if _number(r, "t") >= 0.]
    if not control:
        raise ValueError("Empty measurement trace")
    control_times = np.array([_number(r, "t") for r in control])
    if np.any(np.diff(control_times) <= 0.):
        raise ValueError("Control timestamps must be strictly increasing")
    measured = []
    last_stamp = None
    origin = _number(control[0], "sim_time_s") - control_times[0]
    for row in control:
        stamp = _number(row, "truth_stamp_s")
        sim = _number(row, "sim_time_s")
        if stamp > sim + 1e-6:
            raise ValueError("Truth timestamp is newer than the control sample")
        if abs((sim - _number(row, "t")) - origin) > 1e-5:
            raise ValueError("Simulation and trial clocks have inconsistent origins")
        if last_stamp is not None:
            if stamp < last_stamp - 1e-9:
                raise ValueError("Truth timestamps moved backwards")
            if abs(stamp - last_stamp) <= 1e-9:
                continue
        last_stamp = stamp
        # Do not count the last pre-trial measurement as a trial measurement.
        if stamp - origin >= -1e-8:
            measured.append((row, max(0., stamp - origin)))
    if len(measured) < 2:
        raise ValueError("Fewer than two unique in-trial truth measurements")
    return control, control_times, [r for r, _ in measured], np.array([t for _, t in measured])


def _recovery(times, error_norm, start: float, end: float):
    """First observed entry into the 5 cm band sustained for 2 s, or null."""
    entered = previous = None
    for t, error in zip(times, error_norm):
        if t < start - 1e-8 or t >= end - 1e-8:
            continue
        if previous is not None and t - previous > MAX_RECOVERY_SAMPLE_GAP_S:
            entered = None
        previous = t
        if error < RECOVERY_BAND_M:
            if entered is None:
                entered = t
            if t - entered >= RECOVERY_HOLD_S - 1e-8:
                return float(max(0., entered - start))
        else:
            entered = None
    return None


def _window_metrics(times, error_norm, start, end):
    mask = (times >= start) & (times < end)
    stamps, errors = times[mask], error_norm[mask]
    duration = float(stamps[-1] - stamps[0]) if len(stamps) else 0.
    complete = bool(len(stamps) >= 2 and stamps[0] - start <= .15 and
                    end - stamps[-1] <= .15 and np.max(np.diff(stamps)) <= .15 + 1e-8)
    return {
        "start_s": start, "end_s": end, "samples": len(stamps), "coverage_complete": complete,
        "first_observed_s": float(stamps[0]) if len(stamps) else None,
        "last_observed_s": float(stamps[-1]) if len(stamps) else None,
        "observed_span_s": duration,
        "rmse_m": float(np.sqrt(_time_average(errors**2, stamps))) if duration else None,
        "peak_m": float(np.max(errors)) if len(errors) else None,
        "iae_m_s": float(_time_average(errors, stamps) * duration) if duration else None,
    }


def _distribution(values):
    if not len(values):
        return {"n": 0, "p50": None, "p95": None, "p99": None, "max": None}
    p50, p95, p99 = np.percentile(values, [50, 95, 99])
    return {"n": len(values), "p50": float(p50), "p95": float(p95), "p99": float(p99),
            "max": float(max(values))}


def _trace_timing(rows):
    result = {}
    for field in TRACE_TIMING_COLUMNS:
        values = [_number(row, field) for row in rows if row.get(field) not in (None, "")]
        if any(value < 0 for value in values):
            raise ValueError(f"Negative timing or load value in {field}")
        result[field] = _distribution(values)
    return result


def summarize_trace(rows: Iterable[dict], case: str) -> dict:
    """Summarize observed trace, explicitly separating coverage from accuracy.

    ``target_x/y/z`` must be the world target evaluated at ``truth_stamp_s``;
    current command references may be logged separately as ``command_target_*``.
    Truth is deduplicated by source timestamp. Control statistics use all control
    samples. Incomplete traces are measurable but cannot pass acceptance.
    """
    case = _case_name(case)
    control, ct, truth, tt = _measurement_rows(rows)
    xyz = np.array([[_number(r, "truth_" + a) for a in AXES] for r in truth])
    target = np.array([[_number(r, "target_" + a) for a in AXES] for r in truth])
    error = xyz - target
    error_norm = np.linalg.norm(error, axis=1)
    squared_axis = _time_average(error * error, tt)
    commands = np.array([[_number(r, "v" + a + "_cmd") for a in AXES] for r in control])
    compute = np.array([_number(r, "compute_ms") for r in control])
    if np.any(compute < 0.):
        raise ValueError("Negative controller compute time")
    limited = np.array([_boolean(r["limited"]) for r in control], dtype=float)
    tilt = np.array([_number(r, "tilt_deg") for r in truth])
    tail = tt >= max(tt[0], tt[-1] - 5.)
    tail_error = _time_average(error[tail], tt[tail])
    nominal_duration = DURATIONS.get(case)
    complete = bool(nominal_duration is not None and tt[0] <= .15 and tt[-1] >= nominal_duration - .15)
    deltas = np.diff(commands, axis=0)
    control_intervals = np.diff(ct)
    speed_boundary = np.any(np.abs(commands) >= .4 - 1e-5, axis=1).astype(float)
    slew_boundary = np.any(np.abs(deltas) / control_intervals[:, None] >= 1. - 1e-4, axis=1)
    variation = np.sum(np.abs(deltas), axis=0)
    arm_error = np.array([[_number(r, f"q{i}") - _number(r, f"q{i}_target") for i in range(2)] for r in control])
    summary = {
        "case": case,
        "available": True,
        "measurement_complete": complete,
        "nominal_duration_s": nominal_duration,
        "duration_s": float(tt[-1] - tt[0]),
        "first_truth_time_s": float(tt[0]),
        "last_truth_time_s": float(tt[-1]),
        "control_samples": len(control),
        "truth_samples": len(truth),
        "duplicate_or_pretrial_truth_samples": len(control) - len(truth),
        "truth_sample_gap_max_s": float(np.max(np.diff(tt))),
        "position_rmse_m": float(np.sqrt(np.sum(squared_axis))),
        "position_max_m": float(np.max(error_norm)),
        "axis_rmse_m": {a: float(np.sqrt(squared_axis[i])) for i, a in enumerate(AXES)},
        "last_5s_bias_m": {a: float(tail_error[i]) for i, a in enumerate(AXES)},
        "last_5s_mean_error_norm_m": float(_time_average(error_norm[tail], tt[tail])),
        "tilt_max_deg": float(np.max(tilt)),
        "command_total_variation_m_s": float(np.sum(variation)),
        "command_total_variation_axis_m_s": {a: float(variation[i]) for i, a in enumerate(AXES)},
        "command_slew_max_m_s2": float(np.max(np.abs(deltas) / np.diff(ct)[:, None])),
        "limited_fraction": float(_time_average(limited, ct)),
        "speed_boundary_fraction": float(_time_average(speed_boundary, ct)),
        "slew_boundary_fraction": float(np.sum(control_intervals * slew_boundary) / np.sum(control_intervals)),
        "compute_ms": _distribution(compute),
        "timing": _trace_timing(control),
        "deadline_20ms_exceeded_count": int(np.sum(compute > 20.)),
        "deadline_20ms_exceeded_fraction": float(np.mean(compute > 20.)),
        "max_control_dt_s": max(_number(r, "dt") for r in control),
        "arm_rmse_rad": [float(v) for v in np.sqrt(_time_average(arm_error * arm_error, ct))],
        "max_closure_m": max(_number(r, "closure_m") for r in control),
        "max_axis_error_rad": max(_number(r, "axis_rad") for r in control),
        "definitions": {
            "position_frame": "Gazebo world; reference evaluated at the truth measurement timestamp",
            "time_weighting": "Trapezoidal integral over unique source timestamps / observed duration; no extrapolation",
            "command_total_variation": "Sum of absolute changes of actually sent velocity commands; not energy",
            "limited_fraction": "Time-weighted fraction clipped by the common adapter; does not measure internal MPC constraint activity",
            "speed_boundary_fraction": "Time-weighted fraction of sent commands with any axis |u| >=0.4-1e-5 m/s",
            "slew_boundary_fraction": "Interval-duration-weighted fraction with any sent-command axis |delta_u|/delta_t >=1-1e-4 m/s^2",
            "compute_percentiles": "Unweighted empirical percentiles over controller calls; 20 ms is the nominal period",
            "bias": "Signed time-mean position error in the last 5 observed seconds; not necessarily a settled state",
            "recovery": "First observed 3D error <0.05 m continuously for >=2 s; sample gaps >0.15 s break continuity; null means not observed",
        },
    }
    criteria = {key: bool(summary[key] <= limit) for key, limit in LIMITS.items()}
    summary.update(acceptance_limits=LIMITS.copy(), criteria=criteria,
                   accuracy_within_limits_observed=all(criteria.values()),
                   accuracy_accepted=bool(complete and all(criteria.values())))

    if case == "step":
        transitions = [(5., 0, 1.), (10., 0, -1.), (15., 1, 1.),
                       (20., 1, -1.), (25., 2, 1.), (30., 2, -1.)]
        steps = []
        for index, (start, axis, direction) in enumerate(transitions):
            end = transitions[index + 1][0] if index + 1 < len(transitions) else 35.
            selected = (tt >= start - 1e-8) & (tt < end - 1e-8)
            overshoot = None if not np.any(selected) else max(0., float(np.max(direction * error[selected, axis])))
            steps.append({"time_s": start, "axis": AXES[axis], "direction": direction,
                          "step_m": .2 * direction, "overshoot_m": overshoot,
                          "overshoot_percent": None if overshoot is None else 100. * overshoot / .2,
                          "settling_time_s": _recovery(tt, error_norm, start, end)})
        summary["steps"] = steps
    elif case in WINDOWS:
        windows = {name: _window_metrics(tt, error_norm, start, end)
                   for name, (start, end) in WINDOWS[case].items()}
        summary["windows"] = windows
        start, end = WINDOWS[case]["after"]
        indexes = np.flatnonzero((tt >= start - 1e-8) & (tt < end - 1e-8))
        first = int(indexes[0]) if len(indexes) else None
        near_start = first is not None and tt[first] - start <= MAX_RECOVERY_SAMPLE_GAP_S
        recovery_time = _recovery(tt, error_norm, start, end)
        summary["recovery_time_s"] = recovery_time
        summary["recovery"] = {
            "window_start_s": start, "window_end_s": end,
            "first_observed_s": float(tt[first]) if first is not None else None,
            "observed_until_s": float(tt[indexes[-1]]) if len(indexes) else None,
            "initially_in_band": bool(error_norm[first] < RECOVERY_BAND_M) if near_start else None,
            "observed": recovery_time is not None,
        }
        if case == "force":
            summary["disturbance_peak_error_m"] = windows["active"]["peak_m"]
        else:
            summary["arm_motion_peak_error_m"] = windows["active"]["peak_m"]
            summary["arm_move_recovery_time_s"] = _recovery(tt, error_norm, 5., 13.)
    return summary


def _read_trace(path: Path) -> list[dict]:
    with path.open(newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def _sha(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _mean_std(values: list[float]) -> dict:
    return {"n": len(values), "mean": None if not values else float(np.mean(values)),
            "std": None if len(values) < 2 else float(np.std(values, ddof=1))}


def _nested(data: dict, key: str):
    for part in key.split("."):
        if not isinstance(data, dict) or part not in data:
            return None
        data = data[part]
    return data


AGGREGATE_METRICS = ("position_rmse_m", "position_max_m", "tilt_max_deg",
                     "last_5s_mean_error_norm_m", "command_total_variation_m_s", "limited_fraction",
                     "speed_boundary_fraction", "slew_boundary_fraction",
                     "compute_ms.p50", "compute_ms.p95", "compute_ms.p99", "compute_ms.max",
                     "deadline_20ms_exceeded_count", "recovery_time_s")
TIMING_METRICS = tuple(f"timing.{field}.{stat}" for field in TRACE_TIMING_COLUMNS
                       for stat in ("p50", "p95", "p99", "max"))


def _terminal_controller_events(result: dict) -> list[dict]:
    """Recover rejected controller calls which never reached the command trace."""
    events = []
    failure = result.get("controller_failure")
    if isinstance(failure, dict):
        diagnostics = failure.get("diagnostics", {})
        events.append({"kind": "controller_failure", "t": failure.get("t"),
                       "compute_ms": failure.get("compute_ms"), "command_sent": False,
                       "diagnostics": diagnostics,
                       "solver_status": diagnostics.get("solver_status") if isinstance(diagnostics, dict) else None})
    if result.get("deadline_failure_ms") is not None:
        elapsed = result["deadline_failure_ms"]
        # Future runners may attach both exception and deadline information to
        # the same fatal call. Count it once rather than inventing two calls.
        if events and events[-1].get("compute_ms") == elapsed:
            events[-1]["deadline_exceeded"] = True
        else:
            events.append({"kind": "deadline_failure", "t": result.get("deadline_failure_t"),
                           "compute_ms": elapsed, "command_sent": False,
                           "deadline_exceeded": True, "solver_status": None})
    for event in events:
        value = event.get("compute_ms")
        if value is not None:
            try:
                value = float(value)
            except (TypeError, ValueError):
                value = None
            if value is not None and (not math.isfinite(value) or value < 0.):
                value = None
            event["compute_ms"] = value
        event["deadline_exceeded"] = bool(event.get("deadline_exceeded") or
                                          (value is not None and value > 20.))
    return events


def _finite_json(value):
    if isinstance(value, dict):
        return {key: _finite_json(item) for key, item in value.items()}
    if isinstance(value, list):
        return [_finite_json(item) for item in value]
    return None if isinstance(value, float) and not math.isfinite(value) else value


def _controller_timing(rows: list[dict], terminal: list[dict]) -> dict:
    sent_times = [_number(row, "compute_ms") for row in rows if _number(row, "t") >= 0.]
    if any(t < 0. for t in sent_times):
        raise ValueError("Negative controller compute time")
    failure_times = [event["compute_ms"] for event in terminal if event.get("compute_ms") is not None]
    values = sent_times + failure_times
    deadlines = sum(t > 20. for t in sent_times) + sum(event["deadline_exceeded"] for event in terminal)
    return {"sent_command_calls": len(sent_times), "unsent_failure_calls": len(terminal),
            "calls_with_timing": len(values),
            "compute_ms": _distribution(values),
            "deadline_20ms_exceeded_count": int(deadlines),
            "deadline_20ms_exceeded_fraction": None if not (len(sent_times) + len(terminal)) else deadlines / (len(sent_times) + len(terminal)),
            "definition": "Trace measurement calls plus rejected terminal calls recorded in result.json, including prestream failures. A rejected call is not a sent command; missing timing remains unknown."}


def _input_evidence(folder: Path, result: dict, manifest: dict) -> dict:
    expected = {"controller_config.json": manifest.get("config_sha256"),
                "response_model.json": manifest.get("model_sha256")}
    expected.update(manifest.get("source_sha256", {}))
    expected = {name: digest for name, digest in expected.items() if digest is not None}
    actual, missing, mismatches = {}, [], []
    for name, digest in expected.items():
        path = folder / name
        if not path.is_file():
            missing.append(name)
            continue
        actual[name] = _sha(path)
        if actual[name] != digest:
            mismatches.append(name)
    for reported, name in (("config_sha256", "controller_config.json"),
                           ("response_model_sha256", "response_model.json")):
        if name not in expected:
            continue
        if result.get(reported) is None:
            missing.append("result." + reported)
        elif result[reported] != expected[name]:
            mismatches.append("result." + reported)
    return {"status": "not_requested" if not expected else "mismatch" if mismatches else "missing" if missing else "verified",
            "snapshot_sha256": actual, "missing": missing, "mismatches": mismatches,
            "note": "manifest.model_sha256 is the response_model.json file hash; result.model_sha256 identifies the physical plant"}


def _batch_frozen_inputs(batch: Path, manifest: dict) -> dict:
    checks = {}
    physical_model = None
    for name, key in (("controller_config.json", "config_sha256"),
                      ("response_model.json", "model_sha256")):
        path = batch / name
        checks[name] = bool(path.is_file() and manifest.get(key) and _sha(path) == manifest[key])
    try:
        physical_model = json.loads((batch / "controller_config.json").read_text()).get("model_sha256")
    except (OSError, ValueError, AttributeError):
        pass
    checks["source_hashes_recorded"] = bool(manifest.get("source_sha256"))
    checks["physical_model_recorded"] = bool(physical_model)
    return {"verified": all(checks.values()), "checks": checks,
            "expected_physical_model_sha256": physical_model}


def _parameter_evidence(folder: Path, result: dict) -> dict:
    parameters = result.get("parameters") or {}
    before, after = parameters.get("before") or {}, parameters.get("after") or {}
    files = {}
    for phase, evidence in (("before", before), ("after", after)):
        filename, expected = evidence.get("file"), evidence.get("sha256")
        path = folder / filename if isinstance(filename, str) else None
        actual = _sha(path) if path is not None and path.is_file() else None
        files[phase] = {"file": filename, "expected_sha256": expected, "actual_sha256": actual,
                        "hash_matches": bool(expected and actual == expected)}
    checks = {"before_contract_matches": before.get("contract_matches") is True,
              "after_parameters_unchanged": after.get("parameters_unchanged") is True,
              "before_snapshot_hash_matches": files["before"]["hash_matches"],
              "after_snapshot_hash_matches": files["after"]["hash_matches"]}
    return {"verified": all(checks.values()), "checks": checks, "snapshots": files}


def _process_audit(batch, manifest, item):
    filename = item.get("process_audit_file")
    report = {"verified": False, "file": filename, "checks": {}}
    try:
        path = (batch / filename).resolve()
        if path.parent != batch:
            raise ValueError("Process audit is outside the batch directory")
        actual = _sha(path)
        audit = json.loads(path.read_text())
        checks = {
            "hash_matches": actual == item.get("process_audit_sha256"),
            "schema_matches": audit.get("schema_version") == 1,
            "identity_matches": bool(audit.get("trial_id") and
                audit["trial_id"] == item.get("process_audit_trial_id") and
                all(audit.get(k) == item.get(k) for k in ("algorithm", "case")) and
                audit.get("partition") and audit["partition"] == manifest.get("partition")),
            "accepted": item.get("process_audit_accepted") is True and audit.get("accepted") is True,
            "launch_attempted": audit.get("launch_attempted") is True,
        }
        for phase in ("before", "after"):
            snap = audit.get(phase) or {}
            checks[phase + "_clean"] = (snap.get("clean") is True and snap.get("processes") == [] and
                                        snap.get("errors") == [])
        report.update(checks=checks, sha256=actual, verified=all(checks.values()))
    except (OSError, ValueError, TypeError, AttributeError) as exc:
        report["error"] = str(exc)
    return report


def build_report(batch_dir: Path | str, *, plots: bool = True) -> dict:
    batch = Path(batch_dir).resolve()
    manifest_path = batch / "manifest.json"
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    frozen_inputs = _batch_frozen_inputs(batch, manifest)
    audit_required = manifest.get("schema_version", 1) >= 2 or manifest.get("process_audit_required") is True
    records, traces = [], {}
    attempts = [("interrupted", item) for item in manifest.get("interrupted_attempts", [])]
    attempts += [("scheduled", item) for item in manifest.get("runs", [])]
    for index, (origin, item) in enumerate(attempts):
        record = {key: item.get(key) for key in ("algorithm", "case", "repetition", "result_dir", "returncode")}
        record.update(execution_success=False, accuracy_accepted=False, metrics=None,
                      evidence_accepted=False, evidence_checks={},
                      attempt_origin=origin, finished=item.get("finished"),
                      terminal_controller_events=[], controller_failure_count=0,
                      controller_timing=None, trace_status="missing", result_status="missing",
                      input_evidence={"status": "unavailable"})
        if item.get("error"):
            record["error"] = item["error"]
        if audit_required:
            record["process_audit"] = _process_audit(batch, manifest, item)
            record["evidence_checks"]["process_audit_verified"] = record["process_audit"]["verified"]
        rows = []
        try:
            if not item.get("result_dir"):
                reason = ("Interrupted attempt without result directory" if origin == "interrupted" else
                          "Pending or running attempt; result directory not recorded yet" if item.get("finished") is False and item.get("returncode") is None else
                          "Platform failure before result directory")
                raise ValueError(item.get("error") or item.get("platform_failure") or reason)
            folder = Path(item["result_dir"])
            if not folder.is_absolute():
                folder = batch / folder
            result_path = folder / "result.json"
            result = json.loads(result_path.read_text(encoding="utf-8"))
            record["result_status"] = "available"
            record["result_sha256"] = _sha(result_path)
            record["model_sha256"] = result.get("model_sha256")
            record["input_evidence"] = _input_evidence(folder, result, manifest)
            record["parameter_evidence"] = _parameter_evidence(folder, result)
            record["result_algorithm"] = result.get("algorithm")
            record["result_case"] = result.get("case")
            record["evidence_checks"].update({
                "batch_frozen_inputs_verified": frozen_inputs["verified"],
                "run_frozen_inputs_verified": record["input_evidence"]["status"] == "verified",
                "algorithm_matches_manifest": result.get("algorithm") is not None and result.get("algorithm") == item.get("algorithm"),
                "case_matches_manifest": result.get("case") is not None and result.get("case") == item.get("case"),
                "physical_model_matches_frozen_config": bool(frozen_inputs["expected_physical_model_sha256"] and
                    result.get("model_sha256") == frozen_inputs["expected_physical_model_sha256"]),
                "px4_parameter_evidence_verified": record["parameter_evidence"]["verified"],
            })
            record["evidence_accepted"] = all(record["evidence_checks"].values())
            record["execution_success"] = bool(result.get("execution_success", result.get("success", False)))
            record["error"] = result.get("error") or record.get("error")
            record["controller_failure"] = _finite_json(result.get("controller_failure"))
            record["deadline_failure_ms"] = _finite_json(result.get("deadline_failure_ms"))
            record["timing_fault"] = _finite_json(result.get("timing_fault"))
            record["terminal_controller_events"] = _finite_json(_terminal_controller_events(result))
            record["controller_failure_count"] = sum(event["kind"] == "controller_failure" for event in record["terminal_controller_events"])
            record["controller_timing"] = _controller_timing([], record["terminal_controller_events"])
            trace_path = folder / "control_trace.csv"
            rows = _read_trace(trace_path)
            record["trace_sha256"] = _sha(trace_path)
            record["trace_status"] = "invalid"
            record["controller_timing"] = _controller_timing(rows, record["terminal_controller_events"])
            record["metrics"] = summarize_trace(rows, str(item["case"]))
            if audit_required and record["execution_success"]:
                timing = record["metrics"]["timing"]
                n = record["metrics"]["control_samples"]
                record["evidence_checks"]["runtime_timing_recorded"] = all(
                    values["n"] >= max(1, n - int(key == "wall_send_interval_s"))
                    for key, values in timing.items())
                record["evidence_accepted"] = all(record["evidence_checks"].values())
            record["trace_status"] = "available"
            record["metrics"]["sent_command_compute_ms"] = record["metrics"]["compute_ms"].copy()
            for key in ("compute_ms", "deadline_20ms_exceeded_count", "deadline_20ms_exceeded_fraction"):
                record["metrics"][key] = record["controller_timing"][key]
            record["metrics"]["definitions"]["compute_percentiles"] = record["controller_timing"]["definition"]
            record["accuracy_accepted"] = bool(record["execution_success"] and record["metrics"]["accuracy_accepted"])
            traces[index] = rows
        except (OSError, ValueError, KeyError, TypeError) as exc:
            record["analysis_error"] = str(exc)
            if audit_required and record["execution_success"]:
                record["evidence_checks"]["runtime_timing_recorded"] = False
                record["evidence_accepted"] = False
        records.append(record)

    groups = []
    for algorithm, case in sorted({(str(r["algorithm"]), str(r["case"])) for r in records}):
        selected = [r for r in records if str(r["algorithm"]) == algorithm and str(r["case"]) == case]
        eligible = [r for r in selected if r["metrics"] is not None]
        group = {"algorithm": algorithm, "case": case, "attempted": len(selected),
                 "metrics_available": len(eligible),
                 "measurement_complete": sum(r["metrics"]["measurement_complete"] for r in eligible),
                 "execution_successes": sum(r["execution_success"] for r in selected),
                 "accuracy_accepted": sum(r["accuracy_accepted"] for r in selected),
                 "evidence_accepted": sum(r["evidence_accepted"] for r in selected),
                 "interrupted_attempts": sum(r["attempt_origin"] == "interrupted" for r in selected),
                 "missing_traces": sum(r["trace_status"] == "missing" for r in selected),
                 "invalid_traces": sum(r["trace_status"] == "invalid" for r in selected),
                 "controller_failures": sum(r["controller_failure_count"] for r in selected),
                 "unsent_failure_calls": sum(len(r["terminal_controller_events"]) for r in selected),
                 "deadline_exceeded_count_total": sum(r["controller_timing"]["deadline_20ms_exceeded_count"] for r in selected if r["controller_timing"] is not None),
                 "input_evidence_verified": sum(r["input_evidence"]["status"] == "verified" for r in selected),
                 "input_evidence_mismatch": sum(r["input_evidence"]["status"] == "mismatch" for r in selected),
                 "metrics_include_incomplete_runs": True}
        group["execution_success_rate"] = group["execution_successes"] / group["attempted"]
        recovery_trials = [r["metrics"]["recovery"] for r in eligible if "recovery" in r["metrics"]]
        group["recovery_observed"] = sum(info["observed"] for info in recovery_trials)
        group["recovery_with_window_data"] = sum(info["observed_until_s"] is not None for info in recovery_trials)
        group["recovery_initially_in_band"] = sum(info["initially_in_band"] is True for info in recovery_trials)
        group["metrics"] = {key: _mean_std([float(v) for r in selected
                                           if (v := _nested(r["controller_timing"] if key.startswith("compute_ms.") or key == "deadline_20ms_exceeded_count" else r["metrics"], key)) is not None])
                            for key in AGGREGATE_METRICS + TIMING_METRICS}
        for window in WINDOWS.get(case, {}):
            complete_windows = [r["metrics"]["windows"][window] for r in eligible
                                if r["metrics"]["windows"][window]["coverage_complete"]]
            for key in ("rmse_m", "peak_m", "iae_m_s"):
                group["metrics"][f"windows.{window}.{key}"] = _mean_std([w[key] for w in complete_windows])
        groups.append(group)
    model_ids = sorted({r["model_sha256"] for r in records if r.get("model_sha256")})
    schedule = manifest.get("schedule", [])
    scheduled = manifest.get("runs", [])
    schedule_complete = bool(schedule and len(schedule) == len(scheduled) and
        all(run.get("finished") is True and all(run.get(key) == planned.get(key)
            for key in ("algorithm", "case", "repetition")) for run, planned in zip(scheduled, schedule)))
    batch_evidence_checks = {
        "frozen_inputs_verified": frozen_inputs["verified"],
        "scheduled_attempts_finished": schedule_complete,
        "no_interrupted_attempts": not manifest.get("interrupted_attempts"),
        "every_attempt_evidence_accepted": bool(records) and all(r["evidence_accepted"] for r in records),
        "physical_model_identity_consistent": bool(model_ids) and len(model_ids) == 1,
    }
    if audit_required:
        ids = [item.get("process_audit_trial_id") for _, item in attempts]
        batch_evidence_checks["process_audits_distinct"] = bool(ids) and all(ids) and len(set(ids)) == len(ids)
        batch_evidence_checks["every_process_audit_verified"] = bool(records) and all(
            r["process_audit"]["verified"] for r in records)
    summary = {"schema_version": 1, "manifest_sha256": _sha(manifest_path),
               "evidence_accepted": all(batch_evidence_checks.values()),
               "evidence_checks": batch_evidence_checks,
               "batch_frozen_inputs": frozen_inputs,
               "attempted": len(records), "scheduled_attempts": len(manifest.get("runs", [])),
               "process_audit_required": audit_required,
               "interrupted_attempt_count": len(manifest.get("interrupted_attempts", [])),
               "model_sha256_values": model_ids,
               "model_identity_consistent": bool(model_ids) and len(model_ids) == 1,
               "frozen_input_evidence": {"verified_runs": sum(r["input_evidence"]["status"] == "verified" for r in records),
                                          "mismatched_runs": sum(r["input_evidence"]["status"] == "mismatch" for r in records),
                                          "unverified_runs": sum(r["input_evidence"]["status"] not in ("verified", "mismatch") for r in records)},
               "aggregation": "All traces with valid metrics, including incomplete and failed trials. Such partial metrics do not establish a performance ranking. Sample standard deviation uses ddof=1; n<2 has null std.",
               "runs": records, "interrupted_attempts": [r for r in records if r["attempt_origin"] == "interrupted"], "groups": groups}
    (batch / "summary.json").write_text(json.dumps(summary, ensure_ascii=False, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    flat = []
    for group in groups:
        row = {k: v for k, v in group.items() if k != "metrics"}
        for metric, stats in group["metrics"].items():
            row.update({f"{metric}_{k}": v for k, v in stats.items()})
        flat.append(row)
    with (batch / "summary.csv").open("w", newline="", encoding="utf-8") as stream:
        if flat:
            columns = dict.fromkeys(key for row in flat for key in row)
            writer = csv.DictWriter(stream, fieldnames=columns); writer.writeheader(); writer.writerows(flat)
    lines = ["# X650 位置外环初步对比报告", "",
             "**证据校验：通过。**" if summary["evidence_accepted"] else "**证据校验：未通过；此批次不能作为已通过冻结输入校验的正式对比。**", "",
             "证据校验独立于执行成功和精度达标：实验失败可以有可信的失败证据；成功飞行也不能替代代码、配置、物理模型及 PX4 参数核对。", "",
             "三种算法共用 PX4 速度、姿态及角速度内环，机械臂执行统一轨迹。位置指标只使用 Gazebo 真值。", "",
             f"共记录 {len(records)} 次尝试，其中包含 {summary['interrupted_attempt_count']} 次恢复运行前的历史中断。执行完成与精度达标分别计数；失败、无结果目录及不完整轨迹均保留。", "",
             "| 算法 | 场景 | 尝试 | 证据通过 | 执行完成 | 完整测量 | 精度达标 | 有效指标 | 位置 RMSE 均值±样本标准差 (m) |",
             "|---|---|---:|---:|---:|---:|---:|---:|---:|"]
    for group in groups:
        stats = group["metrics"]["position_rmse_m"]
        value = "—" if stats["mean"] is None else f'{stats["mean"]:.5f}'
        if stats["std"] is not None:
            value += f' ± {stats["std"]:.5f}'
        lines.append(f'| {group["algorithm"]} | {group["case"]} | {group["attempted"]} | {group["evidence_accepted"]} | {group["execution_successes"]} | {group["measurement_complete"]} | {group["accuracy_accepted"]} | {group["metrics_available"]} | {value} |')
    if audit_required:
        passed = sum(r["process_audit"]["verified"] for r in records)
        lines += ["", f"进程审计通过 {passed}/{len(records)} 次。每次核对启动前、退出后快照、试次归属与文件哈希。"]
    windows_groups = [group for group in groups if group["case"] in WINDOWS]
    if windows_groups:
        lines += ["", "## 分段位置误差与恢复", "",
                  "各窗口取左闭右开的唯一真值样本，按实际覆盖时间积分。仅将覆盖完整的窗口纳入下表均值。IAE 为三维误差范数的时间积分。", "",
                  "| 算法 | 工况 | 窗口（仿真秒） | 有效窗口数 | RMSE (m) | 峰值 (m) | IAE (m·s) |",
                  "|---|---|---|---:|---:|---:|---:|"]
        for group in windows_groups:
            for name, (start, end) in WINDOWS[group["case"]].items():
                stats = [group["metrics"][f"windows.{name}.{key}"] for key in ("rmse_m", "peak_m", "iae_m_s")]
                fields = ["—" if s["mean"] is None else f'{s["mean"]:.5f}' for s in stats]
                lines.append(f'| {group["algorithm"]} | {group["case"]} | {name} [{start:g},{end:g}) | '
                             f'{stats[0]["n"]} | ' + " | ".join(fields) + " |")
        lines += ["", "| 算法 | 工况 | 观察截止 (s) | 有恢复窗数据 / 尝试 | 起点已在带内 | 观察到恢复 / 尝试 | 已恢复均值 (s) |",
                  "|---|---|---:|---:|---:|---:|---:|"]
        for g in windows_groups:
            mean = g["metrics"]["recovery_time_s"]["mean"]
            value = "—" if mean is None else f"{mean:.3f}"
            lines.append(f'| {g["algorithm"]} | {g["case"]} | {WINDOWS[g["case"]]["after"][1]:g} | '
                         f'{g["recovery_with_window_data"]}/{g["attempted"]} | {g["recovery_initially_in_band"]} | '
                         f'{g["recovery_observed"]}/{g["attempted"]} | {value} |')
        lines += ["", "起点状态取事件后 0.15 s 内的首个真值样本。逐次实际观察截止时间和缺失记录见 summary.json 的 recovery。"]
    lines += ["", "## 时序与主机负载", "",
              "summary.json 的 timing 给出每次测量的样本数、P50、P95、P99 与最大值，旧轨迹缺少的字段留空。dt 采用仿真秒，其余间隔及样本接收年龄采用主机单调时钟秒。clock_no_progress_max_wall_s 是每个控制周期看到的最长时钟未前进墙钟间隔，包含主机回调调度影响。host_loadavg_1m 为主机一分钟系统负载。CSV 汇总各试次分位数的均值及样本标准差。", "",
              "| 算法 | 工况 | 控制 dt P99 (s) | 位置接收年龄 P99 (s) | 时钟未前进间隔 P99 (s) | 计算 P99 (ms) | 主机负载 P50 |",
              "|---|---|---:|---:|---:|---:|---:|"]
    for g in groups:
        fields = [g["metrics"][k]["mean"] for k in ("timing.dt.p99", "timing.position_receipt_age_s.p99",
                  "timing.clock_no_progress_max_wall_s.p99", "compute_ms.p99", "timing.host_loadavg_1m.p50")]
        cells = ["—" if value is None else f"{value:.5f}" for value in fields]
        lines.append(f'| {g["algorithm"]} | {g["case"]} | ' + " | ".join(cells) + " |")
    lines += ["", "统计包含有有效轨迹的失败或不完整实验，其测量时长可能不同，不能据此直接排名。单次详情、缺失原因和数据哈希见 `summary.json`；CSV 给出各指标的有效数量、均值和样本标准差。", "",
              "真值按原始时间戳去重，位置指标按实际时间间隔积分。参考位置与真值测量时刻对齐；起飞和降落不纳入测量段。末 5 秒偏差不自动代表已进入稳态。", "",
              "恢复时间为三维误差小于 5 cm 并连续保持 2 秒的首次进入时间；null 表示在观察窗口中未恢复。控制指令总变化量衡量输出平滑性，不是电机能耗。", "",
              "limited_fraction 只记录公共适配器裁剪，不能表示 MPC 的约束活跃程度。speed_boundary_fraction 和 slew_boundary_fraction 独立使用实际发送指令，分别统计触及 ±0.4 m/s 速度边界及 1 m/s² 变化率边界的时间占比；因此即使 MPC 自身满足约束、适配器没有裁剪，也能正确观察边界使用情况。", "",
              "只有完整测量且三维 RMSE ≤0.25 m、最大误差 ≤0.5 m、最大倾角 ≤15°，并正常完成执行的实验计入精度达标。计算耗时和超时次数包括 result.json 中未发送指令的终止调用；即使没有轨迹，求解失败及其耗时也保留。", "",
              "当前结果仅代表本仿真模型、公共 PX4 内环和本组测试；三次重复不构成统计显著性或实机性能结论。"]
    lines += ["", "## 运行与计算故障", "",
              "| 算法 | 场景 | 控制器异常 | 未发送的失败调用 | 超过 20 ms 的调用总数 | 缺失轨迹 | 无效轨迹 |",
              "|---|---|---:|---:|---:|---:|---:|"]
    for group in groups:
        lines.append(f'| {group["algorithm"]} | {group["case"]} | {group["controller_failures"]} | {group["unsent_failure_calls"]} | {group["deadline_exceeded_count_total"]} | {group["missing_traces"]} | {group["invalid_traces"]} |')
    failures = [r for r in records if not r["accuracy_accepted"] or r.get("analysis_error")]
    if failures:
        lines += ["", "## 未通过或缺失的实验", ""]
        for r in failures:
            reason = r.get("analysis_error") or r.get("error") or "执行未完成、测量不完整或精度超出门槛"
            lines.append(f'- {r["algorithm"]} / {r["case"]} / 第 {r["repetition"]} 次：{reason}')
            for event in r["terminal_controller_events"]:
                lines.append(f'  - {event["kind"]}：耗时 {event.get("compute_ms")} ms，求解器状态 {event.get("solver_status")}，指令未发送。')
            if r.get("timing_fault"):
                lines.append(f'  - 时序故障记录 `{json.dumps(r["timing_fault"], ensure_ascii=False)}`')
    evidence = summary["frozen_input_evidence"]
    lines += ["", f'冻结输入核对：{evidence["verified_runs"]} 次完全匹配，{evidence["mismatched_runs"]} 次存在差异，{evidence["unverified_runs"]} 次证据不足。核对项目包括控制参数、辨识模型和代码快照；详情见每次运行的 input_evidence。']
    if evidence["mismatched_runs"] or evidence["unverified_runs"]:
        lines += ["此批次尚不能完整证明所有尝试使用同一组冻结输入，不能据此作严格公平性结论。"]
    if not summary["evidence_accepted"]:
        lines += ["", "未通过的批次证据条件：" + "、".join(key for key, passed in batch_evidence_checks.items() if not passed) + "。"]
        for record in records:
            if not record["evidence_accepted"]:
                failed_checks = [key for key, passed in record["evidence_checks"].items() if not passed]
                lines.append(f'- {record["algorithm"]} / {record["case"]} / 第 {record["repetition"]} 次：' +
                             ("、".join(failed_checks) if failed_checks else "结果文件或证据不可用") + "。")
    if len(model_ids) > 1:
        lines += ["", "模型哈希不一致：此批次不能视为同模型公平对比。"]
    if plots and traces:
        _plots(batch, records, traces)
        lines += ["", "## 曲线", ""]
        lines += [f'- [{case}](plots/{case}.png)' for case in sorted({str(records[i]["case"]) for i in traces})]
    (batch / "README_CN.md").write_text("\n".join(lines) + "\n", encoding="utf-8")
    return summary


def _plots(batch: Path, records: list[dict], traces: dict[int, list[dict]]):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    folder = batch / "plots"; folder.mkdir(exist_ok=True)
    colors = {"pid": "tab:blue", "ladrc": "tab:orange", "mpc": "tab:green"}
    for case in sorted({str(records[i]["case"]) for i in traces}):
        fig, axes = plt.subplots(3, 1, figsize=(11, 9), sharex=True)
        for index, rows in traces.items():
            record = records[index]
            if str(record["case"]) != case:
                continue
            control, ct, truth, tt = _measurement_rows(rows)
            errors = np.array([[_number(r, "truth_" + a) - _number(r, "target_" + a) for a in AXES] for r in truth])
            commands = np.array([[_number(r, "v" + a + "_cmd") for a in AXES] for r in control])
            label = f'{record["algorithm"].upper()} r{record["repetition"]}'
            if not record["execution_success"]:
                label += " (failed)"
            kwargs = {"color": colors.get(record["algorithm"]), "alpha": .65, "linewidth": 1., "label": label}
            axes[0].plot(tt, np.linalg.norm(errors, axis=1), **kwargs)
            axes[1].plot(ct, np.linalg.norm(commands, axis=1), **kwargs)
            axes[2].plot(ct, [_number(r, "compute_ms") for r in control], **kwargs)
        axes[0].axhline(.05, color="gray", linestyle="--", linewidth=.8)
        axes[2].axhline(20., color="red", linestyle="--", linewidth=.8, label="20 ms period")
        axes[0].set_ylabel("3D position error (m)")
        axes[1].set_ylabel("Sent velocity norm (m/s)")
        axes[2].set_ylabel("Controller compute (ms)")
        axes[2].set_xlabel("Trial simulation time (s)")
        axes[0].set_title(f"X650 position outer loops: {case}")
        axes[0].legend(loc="upper right", fontsize=7, ncol=3)
        for ax in axes:
            ax.grid(alpha=.25)
        fig.tight_layout(); fig.savefig(folder / f"{case}.png", dpi=160); plt.close(fig)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("batch_dir", type=Path)
    parser.add_argument("--no-plots", action="store_true")
    args = parser.parse_args(argv)
    result = build_report(args.batch_dir, plots=not args.no_plots)
    print(json.dumps({"attempted": result["attempted"], "summary": str(args.batch_dir / "summary.json")}, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
