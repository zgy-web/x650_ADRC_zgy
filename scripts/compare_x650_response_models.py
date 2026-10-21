#!/usr/bin/env python3
"""Offline development comparison of first/second-order X650 velocity models.

The old training run alone selects parameters. The already inspected old
holdout is reused for model STRUCTURE DEVELOPMENT, not as a fresh model test.
This script cannot issue a stage certificate or an accepted response model.
"""
from __future__ import annotations

import argparse
from dataclasses import asdict, dataclass
import json
import math
from pathlib import Path
import sys

import numpy as np
from scipy.optimize import least_squares, minimize_scalar

sys.path.insert(0, str(Path(__file__).resolve().parent))
import identify_x650_upper_response as identifier

AXES = "xyz"
FIT_WINDOW_S = (1., 14.)
FULL_WINDOW_S = (0., 14.)
FIRST_TAU_BOUNDS_S = (.01, 1.5)
SECOND_WN_BOUNDS_RAD_S = (.3, 30.)
SECOND_ZETA_BOUNDS = (.1, 5.)
QUIET_VELOCITY_LIMIT_M_S = .01
QUIET_SLOPE_LIMIT_M_S2 = .01


@dataclass(frozen=True)
class Parameters:
    first_tau_s: float
    second_wn_rad_s: float
    second_zeta: float


def transition(wn, zeta, dt):
    """Stable exact homogeneous exp(A dt), including critical damping.

    A = [[0, 1], [-wn**2, -2*zeta*wn]]. A constant input u has
    equilibrium [u, 0], hence x_next = [u,0] + exp(A dt)*(x-[u,0]).
    """
    if not all(math.isfinite(x) for x in (wn, zeta, dt)) or min(wn, zeta) <= 0 or dt < 0:
        raise ValueError("Positive finite stable parameters and nonnegative dt required")
    decay = zeta*wn
    discriminant = zeta*zeta-1.
    if abs(discriminant) < 1e-12:
        f0 = math.exp(-decay*dt)
        f1 = dt*f0
    elif discriminant < 0:
        frequency = wn*math.sqrt(-discriminant)
        envelope = math.exp(-decay*dt)
        f0 = envelope*math.cos(frequency*dt)
        f1 = envelope*math.sin(frequency*dt)/frequency
    else:
        rate = wn*math.sqrt(discriminant)
        # Both exponents are nonpositive. Avoid exp(-decay*t)*cosh(rate*t),
        # whose intermediate cosh could overflow for a long interval.
        upper, lower = math.exp((-decay+rate)*dt), math.exp((-decay-rate)*dt)
        f0, f1 = .5*(upper+lower), .5*(upper-lower)/rate
    return (f0+decay*f1, f1, -wn*wn*f1, f0-decay*f1)


def predict_second_order(wn, zeta, command_times, commands, observation_times,
                         initial_velocity, initial_acceleration=0.):
    """Exact ZOH states at unshifted irregular observations; no state resets."""
    events, values, observations = (np.asarray(x, float) for x in
                                  (command_times, commands, observation_times))
    if (events.ndim != 1 or observations.ndim != 1 or values.shape != events.shape
            or not len(events) or not len(observations)
            or not all(np.isfinite(x).all() for x in (events, values, observations))
            or np.any(np.diff(events) <= 0) or np.any(np.diff(observations) <= 0)
            or not np.isfinite([initial_velocity, initial_acceleration]).all()):
        raise ValueError("Finite ordered commands/observations and initial state required")
    transition(wn, zeta, 0.)
    # Repeated identical commands do not change the continuous ZOH input.
    # Their removal introduces neither a sample grid nor a timestamp shift.
    keep = np.r_[True, np.diff(values) != 0.]
    events, values = events[keep], values[keep]
    previous = float(observations[0])
    index = int(np.searchsorted(events, previous, side="right")-1)
    held = 0. if index < 0 else float(values[index])
    next_index = index+1
    velocity, acceleration = float(initial_velocity), float(initial_acceleration)
    result = np.empty((len(observations), 2), float)
    result[0] = (velocity, acceleration)
    cache = {}

    def advance(finish):
        nonlocal previous, velocity, acceleration
        dt = finish-previous
        if dt < 0:
            raise ValueError("Prediction time regressed")
        if dt:
            if dt not in cache:
                cache[dt] = transition(wn, zeta, dt)
            p00, p01, p10, p11 = cache[dt]
            deviation = velocity-held
            velocity, acceleration = (held+p00*deviation+p01*acceleration,
                                      p10*deviation+p11*acceleration)
        previous = finish

    for sample, finish in enumerate(observations[1:], start=1):
        while next_index < len(events) and events[next_index] <= finish:
            advance(float(events[next_index]))
            held = float(values[next_index])
            next_index += 1
        advance(float(finish))
        result[sample] = (velocity, acceleration)
    return result


def interval_mask(times, window):
    return (times >= window[0]-1e-9) & (times <= window[1]+1e-9)


def prepare_run(run):
    times = np.asarray(run["truth_times"], float)
    mask = interval_mask(times, FULL_WINDOW_S)
    times, velocities = times[mask], np.asarray(run["velocities"], float)[mask]
    commands, command_times = np.asarray(run["commands"], float), np.asarray(run["command_times"], float)
    if (len(times) < 500 or times[0] > .008 or times[-1] < 14.-.008
            or velocities.shape != (len(times), 3) or commands.shape != (len(command_times), 3)
            or not all(np.isfinite(x).all() for x in (times, velocities, command_times, commands))
            or np.any(np.diff(times) <= 0) or np.max(np.diff(times)) > .008+1e-9
            or np.any(np.diff(command_times) <= 0)):
        raise ValueError("Need complete original native 0-14 s observations and ordered commands")
    quiet = interval_mask(times, (0., 1.))
    quiet_commands = interval_mask(command_times, (0., 1.))
    if quiet.sum() < 100 or not quiet_commands.any():
        raise ValueError("Insufficient 0-1 s prerest samples")
    centered = times[quiet]-times[quiet].mean()
    slopes = centered @ velocities[quiet] / float(centered @ centered)
    peak = np.max(abs(velocities[quiet]), axis=0)
    command_peak = np.max(abs(commands[quiet_commands]), axis=0)
    quiet_ok = (np.max(command_peak) <= 1e-10 and np.max(peak) <= QUIET_VELOCITY_LIMIT_M_S
                and np.max(abs(slopes)) <= QUIET_SLOPE_LIMIT_M_S2)
    initialization = {"native_start_time_s": float(times[0]),
        "velocity_m_s": velocities[0].tolist(), "acceleration_m_s2": [0., 0., 0.],
        "acceleration_fitted": False, "quiet_window_s": [0., 1.], "quiet_passed": bool(quiet_ok),
        "quiet_velocity_peak_m_s": peak.tolist(), "quiet_velocity_slope_m_s2": slopes.tolist(),
        "quiet_command_peak_m_s": command_peak.tolist(),
        "quiet_velocity_limit_m_s": QUIET_VELOCITY_LIMIT_M_S,
        "quiet_slope_limit_m_s2": QUIET_SLOPE_LIMIT_M_S2,
        "rationale": "Fixed a0=0 after pre-probe rest; check does not estimate a0. First second excluded from fitting/evaluation; full interval also reported."}
    if not quiet_ok:
        raise ValueError("Fixed-zero initial acceleration assumption failed its declared prerest check: "+json.dumps(initialization))
    if (np.max(abs(run["joints"][:, :2])) > .03
            or np.max(abs(run["joints"][:, 2:])) > .05):
        raise ValueError("Model development requires the existing folded-arm limits")
    for axis in range(3):
        excitation = identifier.axis_data(run, axis, identifier.Criteria())
        if not excitation["passed"]:
            raise ValueError("Original excitation check failed: "+AXES[axis]+" "+str(excitation["reasons"]))
    return {"times": times, "velocities": velocities, "commands": commands,
            "command_times": command_times, "initialization": initialization}


def predictions(data, axis, parameters):
    initial = float(data["velocities"][0, axis])
    first = identifier.predict_velocity(parameters.first_tau_s, data["command_times"],
        data["commands"][:, axis], data["times"], initial)
    second = predict_second_order(parameters.second_wn_rad_s, parameters.second_zeta,
        data["command_times"], data["commands"][:, axis], data["times"], initial)[:, 0]
    return {"first_order": first, "second_order": second}


def fit_axis(training, axis):
    """Select every parameter from training only; accepts no validation run."""
    mask = interval_mask(training["times"], FIT_WINDOW_S)
    actual = training["velocities"][:, axis]
    times, command_times, commands = training["times"], training["command_times"], training["commands"][:, axis]
    initial = float(actual[0])

    def first_loss(logtau):
        residual = identifier.predict_velocity(math.exp(logtau), command_times, commands, times, initial)-actual
        return float(np.mean(residual[mask]**2))

    first = minimize_scalar(first_loss, bounds=tuple(math.log(x) for x in FIRST_TAU_BOUNDS_S),
                            method="bounded", options={"xatol": 1e-8})

    def second_residual(logparams):
        wn, zeta = np.exp(logparams)
        return (predict_second_order(wn, zeta, command_times, commands, times, initial)[:, 0]-actual)[mask]

    lower = np.log([SECOND_WN_BOUNDS_RAD_S[0], SECOND_ZETA_BOUNDS[0]])
    upper = np.log([SECOND_WN_BOUNDS_RAD_S[1], SECOND_ZETA_BOUNDS[1]])
    starts = ((2., .5), (5., 1.), (12., 2.))
    fits = [least_squares(second_residual, np.log(start), bounds=(lower, upper),
                         ftol=1e-10, xtol=1e-10, gtol=1e-10, max_nfev=150) for start in starts]
    converged = [fit for fit in fits if fit.success and np.isfinite(fit.fun).all()]
    if not first.success or not converged:
        raise ValueError("Training optimization did not converge for "+AXES[axis])
    second = min(converged, key=lambda fit: float(np.dot(fit.fun, fit.fun)))
    wn, zeta = np.exp(second.x)
    parameters = Parameters(float(math.exp(first.x)), float(wn), float(zeta))
    diagnostics = {"fit_data": "training only", "fit_window_s": list(FIT_WINDOW_S),
        "free_run_start": "First native sample at approximately 0 s; no pulse or axis-window resets",
        "first_tau_bounds_s": list(FIRST_TAU_BOUNDS_S), "second_wn_bounds_rad_s": list(SECOND_WN_BOUNDS_RAD_S),
        "second_zeta_bounds": list(SECOND_ZETA_BOUNDS), "second_multistarts": [list(x) for x in starts],
        "converged_second_starts": len(converged), "second_training_mean_squared_error": float(np.mean(second.fun**2)),
        "second_bound_proximity_log_fraction": np.minimum((second.x-lower)/(upper-lower), (upper-second.x)/(upper-lower)).tolist(),
        "bounds_are_optimization_constraints_not_acceptance_criteria": True}
    return parameters, diagnostics


def metrics(actual, predicted, mask):
    residual = predicted[mask]-actual[mask]
    return {"samples": int(mask.sum()), "rmse_m_s": float(np.sqrt(np.mean(residual**2))),
        "maximum_error_m_s": float(np.max(abs(residual))), "bias_m_s": float(np.mean(residual)),
        "residual_definition": "predicted minus native physical velocity"}


def compare_loaded_runs(training, old_holdout):
    if (training["identity"] != old_holdout["identity"]
            or training["folder"] == old_holdout["folder"]
            or training["command_csv_sha256"] == old_holdout["command_csv_sha256"]
            or training["normalized_measurement_sha256"] == old_holdout["normalized_measurement_sha256"]):
        raise ValueError("Need distinct independently recorded runs with identical operating identities")
    prepared = {"training": prepare_run(training), "old_holdout_development": prepare_run(old_holdout)}
    axis_results, all_predictions = [], {}
    for axis, name in enumerate(AXES):
        parameters, fit = fit_axis(prepared["training"], axis)
        result = {"axis": name, "parameters": asdict(parameters), "fit": fit}
        for role, data in prepared.items():
            predicted = predictions(data, axis, parameters)
            all_predictions[(role, name)] = predicted
            windows = {"full_0_14_s": FULL_WINDOW_S, "primary_1_14_s": FIT_WINDOW_S,
                       "axis_excitation": (2.+4*axis, 6.+4*axis)}
            summaries = {}
            for label, window in windows.items():
                mask = interval_mask(data["times"], window)
                summaries[label] = {"window_s": list(window), **{model: metrics(data["velocities"][:, axis], values, mask)
                    for model, values in predicted.items()}}
                baseline = summaries[label]["first_order"]["rmse_m_s"]
                candidate = summaries[label]["second_order"]["rmse_m_s"]
                summaries[label]["rmse_reduction_fraction"] = (baseline-candidate)/baseline if baseline > 0 else None
            result[role] = summaries
        axis_results.append(result)
    summaries = {role: {**{key: run[key] for key in ("folder", "command_csv_sha256", "normalized_measurement_sha256")},
                       "initialization": prepared[role]["initialization"]}
                 for role, run in (("training", training), ("old_holdout_development", old_holdout))}
    report = {"kind": "x650_response_model_development_comparison", "schema_version": 1,
        "complete": True, "development_only": True, "eligible_as_stage_gate": False,
        "new_model_independent_validation": False,
        "old_holdout_role": "Previously inspected holdout is now model-structure development data; not an untouched test for the second-order model",
        "parameter_selection": "All first/second-order parameters are selected using training only, then frozen unchanged for the old holdout",
        "fair_comparison": "Both models start at the first native measurement and fit the same 1-14 s training interval. These first-order constants are refitted; not the old 4 s per-axis fit.",
        "first_order_equation": "dv/dt=(u-v)/tau", "second_order_equations": ["dv/dt=a", "da/dt=wn^2*(u-v)-2*zeta*wn*a"],
        "gain": 1., "command_delay_s": 0., "velocity_bias_m_s": 0.,
        "time_alignment": "Original logical command and native physical sample times, exact event-driven zero-order hold, no fitted phase, delay or origin",
        "prediction_initialization": "v0 is the first native sample of each run, a0 is fixed zero; no later measured state corrections",
        "operating_identity": training["identity"], "runs": summaries, "axes": axis_results,
        "input_artifacts": {**training["artifacts"], **old_holdout["artifacts"]},
        "analysis_sources": {str(Path(__file__).resolve()): identifier.sha(__file__),
                             str(Path(identifier.__file__).resolve()): identifier.sha(identifier.__file__)},
        "limitations": ["Existing folded-arm, ideal external-vision, zero acceleration-feedforward operating condition only",
            "Only a diagonal closed-loop input/output model: fitted a is not a measured physical actuator state",
            "No claim that second order uniquely explains the dynamics or that transport latency is absent",
            "No angular, arm, coupled whole-body or hardware model validation",
            "Only one fitting run; no parameter confidence interval or repeatability claim",
            "Old holdout reuse informs structure choice: a fresh predeclared independent experiment is required before any new model acceptance",
            "Changing model order here does not change existing MPC code, certificates, thresholds or flight gates"]}
    return report, prepared, all_predictions


def render_plot(report, prepared, predictions_by_run, output):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    figure, panels = plt.subplots(3, 2, figsize=(15, 10.5), sharex=True)
    roles = ("training", "old_holdout_development")
    for axis, name in enumerate(AXES):
        for column, role in enumerate(roles):
            panel, data = panels[axis, column], prepared[role]
            command_mask = interval_mask(data["command_times"], FULL_WINDOW_S)
            panel.step(data["command_times"][command_mask], data["commands"][command_mask, axis],
                       where="post", color="#667085", lw=1.1, label="Command u")
            panel.plot(data["times"], data["velocities"][:, axis], color="#166aab", lw=1.4, label="Native velocity")
            predicted = predictions_by_run[(role, name)]
            panel.plot(data["times"], predicted["first_order"], "--", color="#d27d2d", lw=1.25, label="First order, train fit")
            panel.plot(data["times"], predicted["second_order"], "-.", color="#6b4c9a", lw=1.25, label="Second order, train fit")
            panel.axvspan(0., 1., color="#bbbbbb", alpha=.15)
            primary = report["axes"][axis][role]["primary_1_14_s"]
            panel.set_title(("Training fit" if column == 0 else "Old holdout: DEVELOPMENT reuse")+" | "+name.upper()+
                "\n1-14 s RMSE: first %.2f / second %.2f mm/s" %
                (1000*primary["first_order"]["rmse_m_s"], 1000*primary["second_order"]["rmse_m_s"]), fontsize=10)
            panel.set_ylabel(name.upper()+" velocity (m/s)")
            panel.grid(alpha=.22)
            panel.set_xlim(0., 14.)
    for panel in panels[-1]:
        panel.set_xlabel("Original task time (s); continuous free run")
    handles, labels = panels[0, 0].get_legend_handles_labels()
    figure.legend(handles, labels, loc="upper center", bbox_to_anchor=(.5, .935), ncol=4, frameon=False)
    figure.suptitle("X650 response model comparison — DEVELOPMENT ONLY", fontsize=16, fontweight="bold", color="#8b2c24", y=.985)
    figure.text(.5, .953, "Unit gain; zero fitted delay/bias; v0 measured once, a0=0; all parameters fitted on training only", ha="center", fontsize=10)
    figure.text(.5, .025, "Old holdout was inspected and now informs model structure. Fresh predeclared data are required for acceptance.\n"
                "Gray first second: prerest check, excluded from the common fit score. No state reset at any pulse.", ha="center", fontsize=10)
    figure.subplots_adjust(left=.075, right=.98, bottom=.10, top=.865, hspace=.36, wspace=.20)
    paths = [Path(output)/"response_model_development.png", Path(output)/"response_model_development.pdf"]
    for path in paths:
        figure.savefig(path, dpi=160)
    plt.close(figure)
    return paths


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    for option in ("train", "old-holdout", "train-audit", "old-holdout-audit", "output"):
        parser.add_argument("--"+option, required=True, type=Path)
    args = parser.parse_args(argv)
    if args.output.exists():
        raise ValueError("Use a new output directory; existing evidence is never overwritten")
    training = identifier.load_research_run(args.train, args.train_audit)
    old_holdout = identifier.load_research_run(args.old_holdout, args.old_holdout_audit)
    report, prepared, predicted = compare_loaded_runs(training, old_holdout)
    args.output.mkdir(parents=True, exist_ok=False)
    plots = render_plot(report, prepared, predicted, args.output)
    report["plots"] = {path.name: identifier.sha(path) for path in plots}
    (args.output/"response_model_development.json").write_text(json.dumps(report, indent=2, allow_nan=False)+"\n", encoding="utf-8")
    print(json.dumps({"complete": True, "development_only": True, "eligible_as_stage_gate": False,
                      "output": str(args.output.resolve())}))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
