#!/usr/bin/env python3
"""Independent, frozen-input numerical acceptance against the TS20 event oracle.

This is a software-model cross-check, not validation of 24 V, a loaded arm, or
hardware.  The oracle and candidate each receive the first q/dq and the complete
wire input only.  No later oracle state is passed to the candidate.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
import sys
import time

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "control_packages/x650_wholebody_v1"))
from x650_wholebody.ts20 import FrozenTs20Model, build_mit_commands


def canonical(value):
    return json.dumps(value, sort_keys=True, separators=(",", ":"), allow_nan=False)


def digest(value):
    return hashlib.sha256(canonical(value).encode()).hexdigest()


def json_safe(value):
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, np.generic):
        return value.item()
    if isinstance(value, dict):
        return {key: json_safe(item) for key, item in value.items()}
    if isinstance(value, (tuple, list)):
        return [json_safe(item) for item in value]
    return value


def protocol():
    """All fixtures and thresholds are fixed before loading the candidate."""
    return {
        "schema": "ts20_prediction_oracle_acceptance_v1",
        "purpose": "numerical consistency with frozen 20 V small-disc interface model",
        "candidate_kind": "independent numerical implementation",
        "physical_parameter_tuning_allowed": False,
        "validation_trajectories_may_be_used_for_tuning": False,
        "first_state_only": True,
        "later_oracle_state_resets": False,
        "q0_rad": [-.25, .35],
        "bus_period_s": .005,
        "kp": 6., "kd": 3276 / 32768,
        "duration_s": 1.2,
        "observation_step_s": .001,
        "horizons_s": [.020, .100, .300, .600],
        "gates": {"q_peak_abs_rad": .0003, "dq_peak_abs_rad_s": .03,
                  "initial_state_abs_tolerance": 1e-12,
                  "bridge_abs_tolerance": 1e-14,
                  "first_release": "first sampled motion after the first strictly superthreshold delayed packet"},
        "fixtures": [
            {"name": "hold", "dq0": [0., 0.], "kind": "zoh",
             "times": [0., .007, 1.2], "velocity": [[0., 0.]] * 3},
            {"name": "small_positive_step", "dq0": [0., 0.], "kind": "zoh",
             "times": [0., .007, .407, 1.2],
             "velocity": [[0., 0.], [.04, .03], [0., 0.], [0., 0.]]},
            {"name": "small_negative_step", "dq0": [0., 0.], "kind": "zoh",
             "times": [0., .013, .413, 1.2],
             "velocity": [[0., 0.], [-.04, -.03], [0., 0.], [0., 0.]]},
            {"name": "opposed_step", "dq0": [0., 0.], "kind": "zoh",
             "times": [0., .007, .307, .607, 1.2],
             "velocity": [[0., 0.], [.2, -.15], [0., 0.], [0., 0.], [0., 0.]]},
            {"name": "reversal", "dq0": [0., 0.], "kind": "zoh",
             "times": [0., .007, .213, .417, .623, .829, 1.2],
             "velocity": [[0., 0.], [.2, -.15], [-.2, .15], [.2, -.15],
                          [-.2, .15], [0., 0.], [0., 0.]]},
            {"name": "low_amplitude_sine", "dq0": [0., 0.], "kind": "sine",
             "amplitude": [.04, .03], "frequency_hz": [1., 1.],
             "phase_rad": [0., 0.], "command_period_s": .02},
            {"name": "phase_shifted_sine", "dq0": [.03, -.02], "kind": "sine",
             "amplitude": [.18, .14], "frequency_hz": [1.25, .75],
             "phase_rad": [0., math.pi / 2], "command_period_s": .02},
            {"name": "nonbus_aligned_pulses", "dq0": [0., 0.], "kind": "zoh",
             "times": [0., .007, .013, .022, .157, .303, .509, .711, 1.2],
             "velocity": [[0., 0.], [.2, -.1], [-.1, .2], [.15, -.12],
                          [-.15, .12], [.07, -.04], [-.07, .04], [0., 0.], [0., 0.]]},
        ],
        "interpretation": {
            "supply_24v_qualified": False,
            "actual_aerial_arm_load_qualified": False,
            "shaft_angle_qualified": False,
            "physical_torque_scale_qualified": False,
            "hardware_accessed": False,
            "new_closed_loop_flight": False,
        },
    }


def fixture_inputs(spec, declaration):
    if spec["kind"] == "zoh":
        return np.array(spec["times"], float), np.array(spec["velocity"], float)
    t = np.arange(round(declaration["duration_s"] / spec["command_period_s"]) + 1) * spec["command_period_s"]
    v = np.array(spec["amplitude"])[None, :] * np.sin(
        2 * np.pi * t[:, None] * np.array(spec["frequency_hz"])[None, :]
        + np.array(spec["phase_rad"])[None, :])
    # Establish explicit rest-command prehistory; do not start with an unknown
    # nonzero command already active in the delay line.
    v[0] = 0.
    v[-1] = 0.
    return t, v


def independent_bridge(times, velocity, q0, declaration):
    """Hand-integrated ZOH fixture; does not call the production bridge."""
    times, velocity, q0 = np.asarray(times), np.asarray(velocity), np.asarray(q0)
    period = declaration["bus_period_s"]
    n = int(round((times[-1] - times[0]) / period))
    bus = times[0] + np.arange(n + 1) * period
    if bus[-1] > times[-1] + 1e-12:
        raise ValueError("Fixture must end on its fixed bus grid")
    positions, speeds = [], []
    for bt in bus:
        q = q0.copy()
        for k in range(len(times) - 1):
            q += max(0., min(float(bt), float(times[k + 1])) - float(times[k])) * velocity[k]
        k = max(0, min(len(times) - 1, int(np.searchsorted(times, bt, side="right") - 1)))
        positions.append(q)
        speeds.append(velocity[k])

    def encode(a, step):
        # Deliberately independent from quantize_half_away in ts20.py.
        return np.array([[math.copysign(math.floor(abs(float(x) / step) + .5), float(x)) * step
                          for x in row] for row in a])

    qwire = encode(positions, 2 * math.pi / 2**24)
    vwire = encode(speeds, 1 / 512)
    command = {"q_ref": qwire, "dq_ref": vwire, "tau_ff": np.zeros_like(qwire),
               "kp": np.full_like(qwire, declaration["kp"]),
               "kd": np.full_like(qwire, declaration["kd"])}
    # A duplicate first command before time zero explicitly initializes the
    # delay pipeline.  Every subsequent packet remains on the 5 ms bus grid.
    bus = np.r_[bus[0] - period, bus]
    command = {key: np.vstack((a[0], a)) for key, a in command.items()}
    return bus, command


def check_bridge(times, velocity, q0, bus, command, declaration):
    production = build_mit_commands(times, velocity, q0,
        bus_period_s=declaration["bus_period_s"], kp=declaration["kp"], kd=declaration["kd"],
        qlimits=([-.75, -.35], [.45, .85]), vlimits=[.45, .35])
    tol = declaration["gates"]["bridge_abs_tolerance"]
    if not np.allclose(production["input_time_s"], bus[1:], atol=tol, rtol=0):
        raise ValueError("Production bridge differs from independent bus clock")
    maxima = {}
    for key in command:
        error = float(np.max(abs(production["command"][key] - command[key][1:])))
        maxima[key] = error
        if error > tol:
            raise ValueError("Production bridge differs from independent wire field: " + key)
    return maxima


def compare_prediction(t, candidate_q, candidate_dq, oracle_q, oracle_dq, declaration):
    t = np.asarray(t, float)
    arrays = [np.asarray(a, float) for a in (candidate_q, candidate_dq, oracle_q, oracle_dq)]
    if (t.ndim != 1 or len(t) < 2 or np.any(np.diff(t) <= 0)
            or any(a.shape != (len(t), 2) or not np.isfinite(a).all() for a in arrays)):
        raise ValueError("Finite increasing query times and N by 2 states required")
    dq, dv = arrays[0] - arrays[2], arrays[1] - arrays[3]
    qpeak = float(np.max(abs(dq)))
    vpeak = float(np.max(abs(dv)))
    initial = float(max(np.max(abs(dq[0])), np.max(abs(dv[0]))))
    integrate = getattr(np, "trapezoid", np.trapz)
    horizon = {}
    for h in declaration["horizons_s"]:
        ix = np.flatnonzero(np.isclose(t - t[0], h, atol=1e-12, rtol=0))
        if len(ix) != 1:
            raise ValueError("Each frozen horizon must be an exact query point")
        horizon[f"{round(h * 1000)}ms"] = {
            "q_abs_error_rad": abs(dq[ix[0]]).tolist(),
            "dq_abs_error_rad_s": abs(dv[ix[0]]).tolist()}
    gates = declaration["gates"]
    return {
        "pass": bool(qpeak <= gates["q_peak_abs_rad"] and vpeak <= gates["dq_peak_abs_rad_s"]
                     and initial <= gates["initial_state_abs_tolerance"]),
        "q_peak_abs_rad": qpeak, "dq_peak_abs_rad_s": vpeak,
        "q_time_rms_rad": np.sqrt(integrate(dq**2, t, axis=0) / (t[-1] - t[0])).tolist(),
        "dq_time_rms_rad_s": np.sqrt(integrate(dv**2, t, axis=0) / (t[-1] - t[0])).tolist(),
        "initial_state_abs_error": initial, "first_state_free_prediction_horizons": horizon,
        "sample_count": len(t), "duration_s": float(t[-1] - t[0]),
    }


def first_release_check(t, q0, dq0, bus, command, motion_model, candidate_dq, oracle_dq):
    """Independent static inequality and actual-time delay check, no fitted v threshold.

    This only tests the first release from a truly zero initial velocity.  Later
    stops/reversals are compared by the complete free trajectory metrics.
    """
    p = motion_model["fixed_parameters"]
    threshold = motion_model["friction_parameters"]["static_threshold"]
    delay = p["combined_interface_delay_s"]
    output = []
    for j in range(2):
        if dq0[j] != 0:
            output.append({"joint": j + 1, "tested": False, "reason": "nonzero initial velocity"})
            continue
        net = (command["kp"][:, j] * (command["q_ref"][:, j] - q0[j])
               + command["kd"][:, j] * command["dq_ref"][:, j]
               + command["tau_ff"][:, j] - p["offset_over_ka"])
        superthreshold = np.flatnonzero(abs(net) > threshold)
        timestamp = None
        expected_index = None
        if len(superthreshold):
            timestamp = max(float(t[0]), float(bus[superthreshold[0]] + delay))
            post = np.flatnonzero(t > timestamp + 1e-12)
            if len(post):
                expected_index = int(post[0])
        observed = []
        for values in (np.asarray(candidate_dq), np.asarray(oracle_dq)):
            moving = np.flatnonzero(abs(values[:, j]) > 1e-12)
            observed.append(int(moving[0]) if len(moving) else None)
        output.append({"joint": j + 1, "tested": True,
                       "expected_delayed_release_s": timestamp,
                       "expected_first_moving_query_index": expected_index,
                       "candidate_first_moving_query_index": observed[0],
                       "oracle_first_moving_query_index": observed[1],
                       "pass": observed == [expected_index, expected_index]})
    return {"pass": all(row.get("pass", True) for row in output), "joints": output}


def run_case(spec, declaration, oracle, candidate, folder):
    source_t, velocity = fixture_inputs(spec, declaration)
    q0, dq0 = np.array(declaration["q0_rad"]), np.array(spec["dq0"])
    t = np.arange(round(declaration["duration_s"] / declaration["observation_step_s"]) + 1) * declaration["observation_step_s"]
    bus, command = independent_bridge(source_t, velocity, q0, declaration)
    bridge_errors = check_bridge(source_t, velocity, q0, bus, command, declaration)
    packets = [{"send_time_s": float(bt), **{key: a[k].tolist() for key, a in command.items()}}
               for k, bt in enumerate(bus)]
    oracle_states, oracle_diag = [], []
    for j in range(2):
        trial = {"input_time_s": bus, "observation_time_s": t, "initial_time_s": float(t[0]),
                 "initial_state": [float(q0[j]), float(dq0[j])],
                 "command": {key: a[:, j] for key, a in command.items()}}
        result = oracle.simulate_motion(trial, max_seconds=30., max_rhs=1000000)
        oracle_states.append(np.asarray(result["prediction"]))
        oracle_diag.append(result["diagnostics"])
    oq = np.column_stack([state[:, 0] for state in oracle_states])
    ov = np.column_stack([state[:, 1] for state in oracle_states])
    # Deliberately one free rollout: no oracle state after t=0 enters this call.
    start = time.perf_counter()
    prediction = candidate.rollout(float(t[0]), q0.copy(), dq0.copy(), packets, t.copy())
    seconds = time.perf_counter() - start
    result = compare_prediction(t, prediction["q"], prediction["dq"], oq, ov, declaration)
    release = first_release_check(t, q0, dq0, bus, command, oracle.metadata()["motion_model"],
                                  prediction["dq"], ov)
    result["first_release_check"] = release
    result["pass"] = result["pass"] and release["pass"]
    if not np.array_equal(np.asarray(prediction["time_s"]), t):
        raise ValueError("Candidate changed query times")
    if spec["name"] == "hold":
        still = max(float(np.max(abs(oq-q0))), float(np.max(abs(ov))))
        if still != 0:
            raise ValueError("Oracle's declared zero-state stiction fixture did not stay still")
    np.savez_compressed(folder / (spec["name"] + ".npz"), time_s=t, oracle_q=oq, oracle_dq=ov,
                        candidate_q=prediction["q"], candidate_dq=prediction["dq"],
                        source_times=source_t, source_velocity=velocity)
    inputs = {"source_times_s": source_t.tolist(), "source_velocity_rad_s": velocity.tolist(),
              "initial_q_rad": q0.tolist(), "initial_dq_rad_s": dq0.tolist(),
              "packets": packets, "query_times_s": t.tolist()}
    (folder / (spec["name"] + "_inputs.json")).write_text(json.dumps(inputs, indent=2) + "\n")
    return {"name": spec["name"], "fixture_sha256": digest(spec), "inputs_sha256": digest(inputs),
            **result, "candidate_rollout_s": seconds, "bridge_peak_abs_errors": bridge_errors,
            "oracle_diagnostics": oracle_diag, "candidate_events": json_safe(prediction.get("events", []))}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for key in ("source", "manifest", "output"):
        parser.add_argument("--" + key, type=Path, required=True)
    parser.add_argument("--native-library", type=Path)
    parser.add_argument("--native-sha256")
    args = parser.parse_args()
    if bool(args.native_library) != bool(args.native_sha256):
        parser.error("--native-library and --native-sha256 must be supplied together")
    if args.output.exists():
        raise ValueError("Use a new output directory; validation evidence is not overwritten")
    args.output.mkdir(parents=True)
    declaration = protocol()
    summary = {"complete": False, "pass": False, "protocol": declaration,
               "protocol_sha256": digest(declaration), "cases": [],
               "script_sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest()}

    def save():
        (args.output / "validation_summary.json").write_text(json.dumps(summary, indent=2, allow_nan=False) + "\n")

    save()
    try:
        # Import only after the validation specification has been committed to
        # disk; it cannot silently adjust thresholds based on candidate outputs.
        from x650_wholebody.ts20_prediction import Ts20PredictionModel
        manifest = json.loads(args.manifest.read_text())
        oracle = FrozenTs20Model(args.source, manifest)
        summary["model"] = oracle.metadata()
        summary["candidate_sha256"] = hashlib.sha256(
            (ROOT / "control_packages/x650_wholebody_v1/x650_wholebody/ts20_prediction.py").read_bytes()).hexdigest()
        candidate = Ts20PredictionModel(oracle.metadata()["motion_model"],
            kp=declaration["kp"], kd=declaration["kd"], bus_period_s=declaration["bus_period_s"])
        summary["native_backend"] = None
        if args.native_library:
            identity = candidate.enable_native(args.native_library, expected_sha256=args.native_sha256)
            cpp = ROOT / "control_packages/x650_wholebody_v1/x650_wholebody/ts20_prediction_native.cpp"
            source_bytes = cpp.read_bytes()
            source_hash = hashlib.sha256(source_bytes).hexdigest()
            (args.output / "candidate_ts20_prediction_native.cpp").write_bytes(source_bytes)
            (args.output / "candidate_ts20_prediction.py").write_bytes(
                (ROOT / "control_packages/x650_wholebody_v1/x650_wholebody/ts20_prediction.py").read_bytes())
            summary["native_source_sha256"] = source_hash
            summary["native_backend"] = {**identity, "cpp_sha256": source_hash,
                "binding": "explicit CLI native backend enabled before all frozen run_case calls",
                "source_archive": "candidate_ts20_prediction_native.cpp"}
        save()
        for spec in declaration["fixtures"]:
            summary["cases"].append(run_case(spec, declaration, oracle, candidate, args.output))
            save()
        summary.update(complete=True, **{"pass": all(c["pass"] for c in summary["cases"])})
        save()
        print(json.dumps({"complete": True, "pass": summary["pass"],
                          "protocol_sha256": summary["protocol_sha256"],
                          "case_metrics": [{key: case[key] for key in (
                              "name", "pass", "q_peak_abs_rad", "dq_peak_abs_rad_s", "candidate_rollout_s")}
                              for case in summary["cases"]]}, indent=2))
        if not summary["pass"]:
            raise SystemExit(2)
    except BaseException as exc:
        summary["error"] = repr(exc)
        save()
        raise


if __name__ == "__main__":
    main()
