#!/usr/bin/env python3
"""Inspect an audited velocity -> attitude -> motion chain at original clocks.

This is a development diagnostic, never an acceptance certificate or a model
fit. State timestamps are timestamp_sample, setpoints use timestamp, and native
physics uses sim_s. No phase shift, interpolation, differentiation or fitted
delay is used to draw the streams or detect their sampled threshold events.
"""
from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT / "scripts") not in sys.path:
    sys.path.insert(0, str(ROOT / "scripts"))
import identify_x650_upper_response as identifier

NED_TO_ENU = np.array([[0., 1., 0.], [1., 0., 0.], [0., 0., -1.]])
FRD_TO_FLU = np.diag([1., -1., -1.])
AXES = "xyz"


def require(condition, message):
    if not condition:
        raise ValueError(message)


def rotation_matrices(quaternions):
    """Body-to-world rotation for finite, nearly unit wxyz quaternions."""
    q = np.asarray(quaternions, float)
    require(q.ndim == 2 and q.shape[1] == 4 and np.isfinite(q).all(), "Invalid wxyz array")
    norm = np.linalg.norm(q, axis=1)
    require(np.all(abs(norm - 1.) <= .001), "Nonunit attitude quaternion")
    w, x, y, z = (q / norm[:, None]).T
    return np.stack((
        1.-2.*(y*y+z*z), 2.*(x*y-z*w), 2.*(x*z+y*w),
        2.*(x*y+z*w), 1.-2.*(x*x+z*z), 2.*(y*z-x*w),
        2.*(x*z-y*w), 2.*(y*z+x*w), 1.-2.*(x*x+y*y)), axis=1).reshape(-1, 3, 3)


def projected_tilt_deg(quaternions, *, px4_frd_ned):
    """World X/Y projection of the upward body axis, independent of Euler yaw.

    PX4 q maps FRD to NED. C R D maps FLU to ENU, so the upward
    body axis is -C R[:,2]. Native q already maps FLU to ENU.
    """
    rotations = rotation_matrices(quaternions)
    upward = (-rotations[:, :, 2] @ NED_TO_ENU.T if px4_frd_ned else rotations[:, :, 2])
    require(np.all(upward[:, 2] > 0.), "Projected tilt requires upright flight")
    return np.rad2deg(np.arctan2(upward[:, :2], upward[:, 2, None]))


def validated_stream(times, values, window, *, label, max_gap, clock):
    """Select original samples covering a window; never invent sample times."""
    t, y, window = np.asarray(times, float), np.asarray(values, float), np.asarray(window, float)
    require(t.ndim == 1 and y.ndim == 2 and len(t) == len(y) and len(t) >= 2,
            "Invalid stream shape: " + label)
    require(np.isfinite(t).all() and np.all(np.diff(t) >= 0.) and t.max() < 1e7,
            "Need finite monotone raw simulation/HRT times: " + label)
    require(window.shape == (2,) and np.isfinite(window).all() and window[1] > window[0], "Invalid window")
    # At a repeated publication stamp retain the last published value. For state
    # stamps a repeated sample is permitted only if the measured fields agree.
    if clock == "timestamp_sample":
        duplicate = np.flatnonzero(np.diff(t) == 0.)
        for index in duplicate:
            if window[0] <= t[index] <= window[1]:
                require(np.array_equal(y[index], y[index+1]), "Conflicting state at one sample time: " + label)
    keep = np.r_[np.diff(t) > 0., True]
    duplicate_count = int((~keep).sum())
    t, y = t[keep], y[keep]
    lower = max(0, int(np.searchsorted(t, window[0], side="right"))-1)
    upper = min(len(t), int(np.searchsorted(t, window[1], side="left"))+1)
    t, y = t[lower:upper], y[lower:upper]
    require(len(t) >= 2 and t[0] <= window[0]+1e-9 and t[-1] >= window[1]-1e-9,
            "Stream does not bracket the complete measurement window: " + label)
    require(np.isfinite(y).all() and np.all(np.diff(t) > 0.) and np.diff(t).max() <= max_gap+1e-9,
            "Nonfinite or sparse measurement stream: " + label)
    gaps = np.diff(t)
    return {"times": t-window[0], "values": y, "timing": {
        "clock_field": clock, "scale_to_seconds": 1. if clock == "sim_s" else 1e-6,
        "time_shift_s": 0., "sample_count": len(t), "duplicate_timestamps_in_full_stream": duplicate_count,
        "first_sim_s": float(t[0]), "last_sim_s": float(t[-1]),
        "median_interval_s": float(np.median(gaps)), "maximum_interval_s": float(gaps.max()),
        "maximum_allowed_interval_s": max_gap}}


def ulog_stream(ulog, name, fields, window, *, state, max_gap):
    matches = [item.data for item in ulog.data_list if item.name == name and item.multi_id == 0]
    require(len(matches) == 1, "Need exactly one ULog instance 0: " + name)
    data = matches[0]
    clock = "timestamp_sample" if state else "timestamp"
    require(all(field in data for field in [clock, *fields]), "Missing ULog fields: " + name)
    values = np.column_stack([np.asarray(data[field], float) for field in fields])
    result = validated_stream(np.asarray(data[clock], float)*1e-6, values, window,
                              label=name, max_gap=max_gap, clock=clock)
    result["timing"].update(topic=name, fields=fields)
    return result


def prior_value(stream, at, column):
    index = int(np.searchsorted(stream["times"], at, side="left"))-1
    require(index >= 0, "No earlier sample to establish the event baseline")
    return float(stream["values"][index, column])


def first_sampled_crossing(stream, *, column, start, end, threshold, direction):
    """First reached sample after start; report its interval, never interpolated.

    Preserve the preceding sample time even if it precedes the command. Such an
    interval cannot establish whether the crossing followed the command. A value
    already across at the preceding sample has no newly observed crossing.
    """
    require(direction in (-1, 1) and end > start and np.isfinite(threshold), "Invalid threshold event")
    t, y = stream["times"], stream["values"][:, column]
    candidates = np.flatnonzero((t >= start-1e-9) & (t < end-1e-9))
    hits = candidates[direction*(y[candidates]-threshold) >= 0.]
    if not len(hits):
        return {"observed": False, "threshold": threshold, "search_start_t_s": start, "search_end_t_s": end}
    index = int(hits[0])
    previous = index-1
    was_already_reached = previous >= 0 and direction*(y[previous]-threshold) >= 0.
    interval = [float(t[previous]), float(t[index])] if previous >= 0 and not was_already_reached else None
    straddles = interval is not None and interval[0] < start <= interval[1]
    return {"observed": True, "threshold": threshold, "first_reached_t_s": float(t[index]),
            "elapsed_from_command_s": float(t[index]-start),
            "crossing_interval_t_s": interval,
            "interval_straddles_command": bool(straddles),
            "causal_onset_unresolved": bool(straddles or was_already_reached or previous < 0),
            "already_reached_at_previous_sample": bool(was_already_reached),
            "sample_value": float(y[index])}


def event_diagnostics(streams, axis, nominal_onset):
    command = streams["logical_velocity"]
    old = prior_value(command, nominal_onset, axis)
    candidates = np.flatnonzero((command["times"] >= nominal_onset-1e-9)
                               & (command["times"] <= nominal_onset+.05+1e-9))
    changed = candidates[abs(command["values"][candidates, axis]-old) > .02]
    require(len(changed) > 0, "Scheduled pulse lacks an observed command change")
    index = int(changed[0])
    start, requested = float(command["times"][index]), float(command["values"][index, axis])
    direction = 1 if requested > old else -1
    departure = np.flatnonzero((command["times"] > start)
                               & (abs(command["values"][:, axis]-requested) > 1e-9))
    require(len(departure) > 0, "Pulse has no recorded command departure")
    end = float(command["times"][int(departure[0])])
    require(end >= start+.3, "Pulse is shorter than the fixed early attitude demand window")
    result = {"axis": AXES[axis], "nominal_onset_t_s": nominal_onset, "command_change_t_s": start,
              "command_previous_m_s": old, "command_requested_m_s": requested,
              "pulse_end_t_s": end, "actual_pulse_hold_s": end-start,
              "event_search_interval": "From observed command onset (inclusive) to first recorded departure from its requested value (exclusive)",
              "internal_velocity_50_percent": first_sampled_crossing(streams["internal_velocity"],
                  column=axis, start=start, end=end, threshold=old+.5*(requested-old), direction=direction)}
    baseline = prior_value(streams["native_velocity"], start, axis)
    for fraction in (.1, .5):
        result[f"native_velocity_{round(100*fraction)}_percent"] = first_sampled_crossing(
            streams["native_velocity"], column=axis, start=start, end=end,
            threshold=baseline+fraction*(requested-baseline), direction=1 if requested > baseline else -1)
    result["native_velocity_baseline_m_s"] = baseline
    target = streams["attitude_target"]
    target_baseline = prior_value(target, start, axis)
    early = np.flatnonzero((target["times"] >= start-1e-9) & (target["times"] <= start+.3+1e-9))
    require(len(early) >= 2, "Insufficient attitude target samples in fixed 0.3-second onset window")
    peak = int(early[np.argmax(abs(target["values"][early, axis]-target_baseline))])
    excursion = float(target["values"][peak, axis]-target_baseline)
    require(abs(excursion) > 1e-5, "No measurable early attitude demand excursion")
    result["early_attitude_demand"] = {"fixed_window_s": .3, "baseline_deg": target_baseline,
        "sampled_peak_t_s": float(target["times"][peak]), "peak_value_deg": float(target["values"][peak, axis]),
        "signed_excursion_deg": excursion,
        "meaning": "Largest absolute projected-target excursion in the fixed first 0.3 s; descriptive, not a pure delay"}
    for name in ("attitude_target", "attitude_estimated", "attitude_native"):
        baseline = prior_value(streams[name], start, axis)
        result[name+"_50_percent_early_demand"] = first_sampled_crossing(
            streams[name], column=axis, start=start, end=end,
            threshold=baseline+.5*excursion, direction=1 if excursion > 0. else -1)
        result[name+"_50_percent_early_demand"]["own_precommand_baseline_deg"] = baseline
    return result


def native_integration_check(streams):
    """Kinematic consistency of native acceleration and velocity, without fitting."""
    acceleration, velocity = streams["native_acceleration"], streams["native_velocity"]
    require(np.array_equal(acceleration["times"], velocity["times"]),
            "Native acceleration/velocity do not have identical original sample times")
    selected = (velocity["times"] >= 0.) & (velocity["times"] <= 14.)
    times, measured, a = velocity["times"][selected], velocity["values"][selected], acceleration["values"][selected]
    require(len(times) >= 2, "No native acceleration integration window")
    increments = .5*(a[1:]+a[:-1])*np.diff(times)[:, None]
    integrated = measured[0]+np.vstack((np.zeros(3), np.cumsum(increments, axis=0)))
    residual = integrated-measured
    return {"method": "Trapezoidal integration at unchanged native timestamps, initialized once with measured initial velocity; no bias removal or fitted gain",
            "window_t_s": [float(times[0]), float(times[-1])], "sample_count": len(times),
            "axis_rmse_m_s": np.sqrt(np.mean(residual**2, axis=0)).tolist(),
            "axis_peak_absolute_error_m_s": np.max(abs(residual), axis=0).tolist(),
            "final_residual_m_s": residual[-1].tolist(), "development_only": True}


def load_chain(run, ulog_loader):
    window = np.asarray([run["metadata"]["task_start_sim_s"], run["metadata"]["task_end_sim_s"]])
    folder = Path(run["folder"])
    # The identifier has already checked native, control and ULog raw hashes
    # against the independent acceptance records; these original bytes are read.
    with (folder / "native_trace.csv").open(newline="", encoding="utf-8") as file:
        rows = list(csv.DictReader(file))
    t = np.asarray([float(row["sim_s"]) for row in rows])
    within = (t >= window[0]-.008) & (t <= window[1]+.008)
    require(all(rows[i].get("valid_accel") == "1" for i in np.flatnonzero(within)),
            "Native WorldLinearAcceleration is not valid throughout the diagnostic window")
    def native(names, label):
        values = np.asarray([[float(row[name]) for name in names] for row in rows])
        return validated_stream(t, values, window, label=label, max_gap=.008, clock="sim_s")
    streams = {"native_velocity": native(["base_v"+axis for axis in AXES], "native velocity"),
               "native_acceleration": native(["base_a"+axis for axis in AXES], "native WorldLinearAcceleration"),
               "attitude_native": native(["base_q"+axis for axis in "wxyz"], "native attitude")}
    streams["attitude_native"]["values"] = projected_tilt_deg(streams["attitude_native"]["values"], px4_frd_ned=False)
    ulog = ulog_loader(str(folder / "px4.ulg"))
    velocity_acceleration = ulog_stream(ulog, "vehicle_local_position_setpoint",
        ["vx", "vy", "vz", "acceleration[0]", "acceleration[1]", "acceleration[2]"], window, state=False, max_gap=.025)
    for label, columns in (("internal_velocity", slice(0, 3)), ("acceleration_demand", slice(3, 6))):
        streams[label] = {"times": velocity_acceleration["times"],
            "values": velocity_acceleration["values"][:, columns] @ NED_TO_ENU.T,
            "timing": dict(velocity_acceleration["timing"])}
    for label, topic, prefix, state, gap in (
            ("attitude_target", "vehicle_attitude_setpoint", "q_d", False, .08),
            ("attitude_estimated", "vehicle_attitude", "q", True, .02)):
        streams[label] = ulog_stream(ulog, topic, [f"{prefix}[{i}]" for i in range(4)], window, state=state, max_gap=gap)
        streams[label]["values"] = projected_tilt_deg(streams[label]["values"], px4_frd_ned=True)
    # Already independently audited complete command samples start at the task
    # boundary and may finish just before it. Keep their exact existing times.
    streams["logical_velocity"] = {"times": run["command_times"], "values": run["commands"], "timing": {
        "clock_field": "control_trace.sim_s", "scale_to_seconds": 1., "time_shift_s": 0.,
        "sample_count": len(run["command_times"]),
        "median_interval_s": float(np.median(np.diff(run["command_times"]))),
        "maximum_interval_s": float(np.max(np.diff(run["command_times"])))}}
    return streams


def plot_chain(streams, role, destination, source_run):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    colors = {"command": "#596777", "internal": "#7b55a3", "native": "#1268a5", "target": "#ce681c", "estimated": "#228c66"}
    fig, axes = plt.subplots(3, 2, figsize=(16, 10.5), sharex="col", constrained_layout=True)
    rows = (
        (("logical_velocity", "Logical velocity command", "command", "steps-post"),
         ("internal_velocity", "PX4 internal velocity target", "internal", "steps-post"),
         ("native_velocity", "Native physical velocity", "native", "default")),
        (("acceleration_demand", "PX4 acceleration demand", "target", "steps-post"),
         ("native_acceleration", "Native world linear acceleration", "native", "default")),
        (("attitude_target", "Projected attitude target", "target", "steps-post"),
         ("attitude_estimated", "PX4 estimated attitude", "estimated", "default"),
         ("attitude_native", "Native physical attitude", "native", "default")))
    for axis in range(2):
        display_window = (1.8+4*axis, 6.2+4*axis)
        for row, fields in enumerate(rows):
            ax = axes[row, axis]
            for name, label, color, drawstyle in fields:
                stream = streams[name]
                selected = (stream["times"] >= 0.) & (stream["times"] <= 14.)
                ax.plot(stream["times"][selected], stream["values"][selected, axis],
                        color=colors[color], linewidth=1.2, alpha=.9, label=label, drawstyle=drawstyle)
            for time in (2.+4*axis, 4.+4*axis):
                ax.axvline(time, color="#b5b8bf", linestyle=":", linewidth=.8)
            ax.axhline(0., color="#d4d7dd", linewidth=.7)
            ax.set_xlim(*display_window)
            ax.grid(alpha=.17)
            ax.legend(loc="best", fontsize=8.5)
            ax.set_ylabel(("Velocity [m/s]", "Acceleration [m/s²]", "Projected upward-axis tilt [deg]")[row])
        axes[0, axis].set_title(f"World ENU {AXES[axis].upper()} direction | display {display_window[0]:.1f}–{display_window[1]:.1f} s", fontsize=13)
        axes[2, axis].set_xlabel("Original simulation time since measurement start [s]")
    fig.suptitle(f"{role.capitalize()}: velocity command → attitude → physical motion\n"
                 f"{source_run}  |  DEVELOPMENT DIAGNOSTIC — no model fit or acceptance", fontsize=14)
    fig.supxlabel("State: raw timestamp_sample; setpoint: raw timestamp; physics: sim_s. "
                  "Zero time shift; no resampling. Analysis uses 0–14 s; panels zoom the original axis pulses.\n"
                  "Attitude is atan2(b_up,axis, b_up,z), not Euler roll/pitch.", fontsize=9)
    paths = {}
    for suffix in ("png", "pdf"):
        path = destination / f"{role}_response_chain.{suffix}"
        fig.savefig(path, dpi=180)
        paths[suffix] = {"path": str(path), "sha256": identifier.sha(path)}
    plt.close(fig)
    return paths


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ("train", "holdout", "train-audit", "holdout-audit", "output"):
        parser.add_argument("--"+name, type=Path, required=True)
    args = parser.parse_args(argv)
    require(not args.output.exists(), "Output must be a new directory; original reports are never overwritten")
    runs = {"training": identifier.load_research_run(args.train, args.train_audit),
            "holdout": identifier.load_research_run(args.holdout, args.holdout_audit)}
    require(runs["training"]["identity"] == runs["holdout"]["identity"], "Runs have different physical/control conditions")
    require(runs["training"]["command_csv_sha256"] != runs["holdout"]["command_csv_sha256"]
            and runs["training"]["normalized_measurement_sha256"] != runs["holdout"]["normalized_measurement_sha256"],
            "Need two independent raw runs")
    from pyulog import ULog
    chains = {role: load_chain(run, ULog) for role, run in runs.items()}
    events = {role: [event_diagnostics(streams, axis, onset) for axis, onset in ((0, 2.), (0, 4.), (1, 6.), (1, 8.))]
              for role, streams in chains.items()}
    args.output.mkdir(parents=True)
    report = {"schema_version": 2, "kind": "x650_response_chain_development_diagnostics",
        "complete": True, "development_only": True, "eligible_as_stage_gate": False, "controller_model_artifact": False,
        "model_fit_performed": False, "time_shift_s": 0., "runs": {},
        "semantics": {
            "native_acceleration": "Recorder WorldLinearAcceleration, world ENU; no numerical velocity differentiation",
            "px4_acceleration": "vehicle_local_position_setpoint.acceleration transformed NED to ENU; acceleration demand, not physical force",
            "attitude": "q wxyz body-to-world. PX4: b_up_ENU = -C_NED_ENU R(q)[:,2]; native: R(q)[:,2]. Tilt_i=atan2(b_up_i,b_up_z).",
            "threshold_events": "First sampled threshold hit during the actual recorded command hold, ending before its first departure. The preceding sample is never clipped to command onset: an interval crossing that onset is explicitly causally unresolved. No interpolation or time fit; closed-loop threshold timing is not pure delay.",
            "early_attitude_peak": "Largest absolute target excursion from its precommand baseline within the fixed first 0.3 seconds; actual tilt thresholds use each stream's own precommand baseline.",
            "model_selection": "Development evidence only. Reusing existing holdout for structure selection makes it development data for the next model; new independent validation is needed."}}
    for role, run in runs.items():
        streams = chains[role]
        report["runs"][role] = {"source_run": run["folder"],
            "operating_condition_sha256": run["identity"]["operating_condition_sha256"],
            "window_sim_s": [run["metadata"]["task_start_sim_s"], run["metadata"]["task_end_sim_s"]],
            "analysis_window_t_s": [0., 14.], "display_window_t_s": {"x": [1.8, 6.2], "y": [5.8, 10.2]},
            "inputs_sha256": run["artifacts"], "streams": {name: value["timing"] for name, value in streams.items()},
            "native_acceleration_integration": native_integration_check(streams),
            "events": events[role], "figures": plot_chain(streams, role, args.output, Path(run["folder"]).name)}
    source_paths = {Path(__file__).resolve(), Path(identifier.__file__).resolve()}
    for module in list(sys.modules.values()):
        path = getattr(module, "__file__", None)
        if path and Path(path).suffix == ".py" and Path(path).resolve().is_relative_to(ROOT):
            source_paths.add(Path(path).resolve())
    report["analysis_sources_sha256"] = {str(path): identifier.sha(path) for path in sorted(source_paths)}
    report["analysis_environment"] = {"python": sys.version, "numpy": np.__version__}
    (args.output / "response_chain_diagnostics.json").write_text(json.dumps(report, indent=2, allow_nan=False)+"\n", encoding="utf-8")
    print(json.dumps({"complete": True, "development_only": True, "output": str(args.output),
                      "figures": {role: record["figures"] for role, record in report["runs"].items()}}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
