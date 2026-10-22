"""Independent replay of the frozen command/endpoint prediction chain.

This module never imports a controller, its prediction helpers, or an optimizer.
Local publication records are not Gazebo receipt/application acknowledgements.
It is an additional commissioning check, not an actuator-identification gate.
"""
from __future__ import annotations

import hashlib
import json
from pathlib import Path
import numpy as np
from scipy.linalg import expm
from scipy.spatial.transform import Rotation


def require(condition, message):
    if not condition:
        raise ValueError(message)


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def read(path):
    def invalid(value):
        raise ValueError("Nonfinite JSON constant: "+value)
    return json.loads(Path(path).read_text(encoding="utf-8"), parse_constant=invalid)


def lines(path):
    return [json.loads(line, parse_constant=lambda v: (_ for _ in ()).throw(ValueError(v)))
            for line in Path(path).read_text(encoding="utf-8").splitlines() if line.strip()]


def array(value, shape, label):
    result = np.asarray(value, dtype=float)
    require(result.shape == shape and np.isfinite(result).all(), "Invalid "+label)
    return result


def match(value, expected, label, tolerance=1e-9):
    expected = np.asarray(expected, dtype=float)
    result = array(value, expected.shape, label)
    error = float(np.max(abs(result-expected)))
    require(error <= tolerance, "Prediction-chain mismatch: "+label)
    return error


def command(row):
    return array([*[float(row["base_velocity_target_"+a]) for a in "xyz"],
                  float(row["qd1_cmd"]), float(row["qd2_cmd"])], (5,), "sent command")


def key(value):
    return round(float(value), 9)


def indexed(entries, time_field, label):
    result = {}
    for entry in entries:
        t = key(entry[time_field])
        require(np.isfinite(t) and t not in result, "Duplicate/invalid "+label+" time")
        result[t] = entry
    return result


def replay_joints(now, stamp, q, dq, history, intervals, plan, delay):
    """Integrate measured q/dq through actual held history and future ZOH commands."""
    require(-1e-9 <= now-stamp <= .25+1e-9, "Joint source time outside forward replay domain")
    q, dq = np.array(q, float), np.array(dq, float)
    events = [(when+delay, np.asarray(u)[3:]) for when, u in history if when+delay > stamp+1e-10]
    offsets = np.r_[0., np.cumsum(intervals)[:-1]]
    events.extend((now+offset+delay, u[3:]) for offset, u in zip(offsets, plan))
    events.sort(key=lambda item: item[0])
    clock, index, result = stamp, 0, []
    for end in np.r_[now, now+np.cumsum(intervals)]:
        while index < len(events) and events[index][0] <= end+1e-10:
            when, value = events[index]
            require(when >= clock-1e-9, "Regressing replay command history")
            q = q+max(0., when-clock)*dq
            dq, clock = value.copy(), when
            index += 1
        q = q+(end-clock)*dq
        clock = end
        result.append((q.copy(), dq.copy()))
    return np.asarray(result)


def replay_base(state, intervals, plan, response):
    """Independent augmented matrix exponentials for each declared base axis."""
    p, v, a = (np.array(state[name], float) for name in ("position", "velocity", "acceleration_enu"))
    result, cache = [], {}
    for dt, u in zip(intervals, plan):
        for i, axis in enumerate(("x", "y")):
            wn, zeta = response[axis]["wn_rad_s"], response[axis]["zeta"]
            matrix = np.array([[0., 1., 0., 0.], [0., 0., 1., 0.],
                [0., -wn*wn, -2*zeta*wn, wn*wn], [0., 0., 0., 0.]])
            if (axis, dt) not in cache:
                cache[axis, dt] = expm(matrix*dt)
            p[i], v[i], a[i], _ = cache[axis, dt]@np.r_[p[i], v[i], a[i], u[i]]
        tau = response["z"]["tau_s"]
        decay = np.exp(-dt/tau)
        p[2] += tau*(1-decay)*v[2]+(dt-tau*(1-decay))*u[2]
        v[2] = decay*v[2]+(1-decay)*u[2]
        result.append(np.r_[p, v, a[:2]])
    return np.asarray(result)


def thrust_frame(acceleration, yaw, gravity):
    z = np.asarray(acceleration, float)+[0., 0., gravity]
    require(np.linalg.norm(z) > gravity*.25 and z[2] > gravity*.25, "Invalid replay thrust direction")
    z /= np.linalg.norm(z)
    y = np.cross(z, [np.cos(yaw), np.sin(yaw), 0.])
    require(np.linalg.norm(y) > .25, "Singular replay heading")
    y /= np.linalg.norm(y)
    return np.column_stack((np.cross(y, z), y, z))


def replay_geometry(state, predicted, plan, times, profile, settings, response):
    rotation = array(state["rotation"], (3, 3), "measured attitude")
    yaw = np.arctan2(rotation[1, 0], rotation[0, 0])
    roll = np.arctan2(rotation[2, 1], rotation[2, 2])
    pitch = np.arcsin(np.clip(-rotation[2, 0], -1., 1.))
    omega = array(state["omega_body"], (3,), "measured body rate")
    yaw_rate = (np.sin(roll)*omega[1]+np.cos(roll)*omega[2])/np.cos(pitch)
    gravity, tau = settings["gravity_m_s2"], settings["attitude_residual_decay_s"]
    nominal0 = thrust_frame(state["acceleration_enu"], yaw, gravity)
    residual = Rotation.from_matrix(nominal0.T@rotation).as_rotvec()
    rotations, tips = [], []
    for elapsed, x, u in zip(times, predicted, plan):
        decay = np.exp(-elapsed/tau)
        acceleration = [x[10], x[11], (u[2]-x[5])/response["z"]["tau_s"]]
        frame = thrust_frame(acceleration, yaw+tau*(1-decay)*yaw_rate, gravity)
        frame = frame@Rotation.from_rotvec(decay*residual).as_matrix()
        a1, a2 = np.asarray(profile["angle_offsets"])+[x[6], x[6]+x[7]]
        arm = (np.asarray(profile["mount_body"])+profile["lengths"][0]*np.array([np.cos(a1), 0., -np.sin(a1)])
            +np.asarray(profile["elbow_offset_body_m"])+profile["lengths"][1]*np.array([np.cos(a2), 0., -np.sin(a2)]))
        rotations.append(frame)
        tips.append(x[:3]+frame@arm)
    return np.asarray(rotations), np.asarray(tips), residual


def check_trace(folder, metadata):
    path = folder/"control_execution_trace.jsonl"
    manifest = read(folder/"control_execution_trace_manifest.json")
    events = lines(path)
    require(manifest.get("kind") == "x650_control_execution_trace" and manifest.get("schema_version") == 1
        and manifest.get("file") == path.name and manifest.get("sha256") == sha(path)
        and manifest.get("recording_closed") is True and manifest.get("experiment_complete") is True
        and manifest.get("recording_error") is None and manifest.get("failure_count") == 0
        and manifest.get("event_count") == manifest.get("event_sequence_attempts") == len(events)
        and "first_failure_snapshot" not in manifest and not (folder/"control_failure_snapshot.json").exists(),
        "Execution trace is incomplete, failed, or not hash-bound")
    require(metadata.get("control_execution_trace") == manifest, "Execution trace manifest differs from runtime metadata")
    previous_wall, last = -np.inf, {}
    for i, event in enumerate(events, 1):
        require(event.get("sequence") == i and event.get("schema_version") == 1
                and event.get("event") in ("controller_step_returned", "publication_returned")
                and "exception" not in event, "Unexpected/missing execution trace event")
        prefix = "compute" if event["event"] == "controller_step_returned" else "publish"
        before, after = (event[prefix+"_"+side] for side in ("before", "after"))
        walls = array([before["wall_monotonic_s"], after["wall_monotonic_s"]], (2,), "local trace wall times")
        require(previous_wall <= walls[0] <= walls[1], "Execution trace local times regress")
        previous_wall = walls[1]
        if event["event"] == "publication_returned":
            require(event.get("local_call_returned") is True and event.get("downstream_application_confirmed") is False,
                    "Local publication is missing or falsely claims application acknowledgement")
            last[event["channel"]] = {k: v for k, v in event.items() if k not in ("schema_version", "sequence", "event")}
    require(last == manifest.get("last_publications"), "Execution trace final publication state differs")
    return events


def audit_prediction_chain_execution(folder, control, metadata, protocol, window):
    """Verify actual publications/history and replay recorded forecasts; no solve."""
    folder = Path(folder).resolve()
    settings = protocol["mpc_controller_config"].get("prediction_chain")
    require(isinstance(settings, dict), "Prediction-chain preset required")
    selected = [r for r in control if r["phase"] == "measure" and window[0]-1e-9 <= float(r["sim_s"]) <= window[1]+1e-9]
    times = array([float(r["sim_s"]) for r in selected], (len(selected),), "measurement times")
    require(len(times) >= 2 and np.all(np.diff(times) > 0), "Missing/duplicate measurement rows")
    events = check_trace(folder, metadata)
    feedback = indexed([x for x in lines(folder/"feedback_trace.jsonl") if x.get("phase") == "measure"
        and window[0]-1e-9 <= x["sim_s"] <= window[1]+1e-9], "sim_s", "feedback")
    diagnostics = indexed([x for x in lines(folder/"controller_diagnostics.jsonl") if x.get("phase") == "measure"
        and window[0]-1e-9 <= x["sim_s"] <= window[1]+1e-9], "sim_s", "diagnostics")
    require(set(feedback) == set(diagnostics) == set(map(key, times)), "Trace/feedback/diagnostic coverage differs")
    steps = [e for e in events if e["event"] == "controller_step_returned" and e["phase"] == "measure"
        and -1e-9 <= float(e["t_s"]) <= window[1]-window[0]+1e-9]
    steps = indexed(steps, "t_s", "controller attempt")
    require(set(steps) == {key(float(r["t_s"])) for r in selected}, "Controller attempt coverage differs")
    publications = {}
    for e in events:
        if e["event"] != "publication_returned":
            continue
        k = (key(e["context"]["sim_s"]), e["channel"])
        require(k not in publications, "Duplicate actual publication")
        publications[k] = e
    # Exact retained prestream history, including its final real sent input.
    before = [r for r in control if r["phase"] != "measure" and float(r["sim_s"]) <= window[0]+1e-9
              and r.get("pva_sent") in (True, "True", "true", "1")]
    require(before, "Missing prestream command history")
    cutoff = float(before[-1]["sim_s"])-.75
    before = [r for r in before if float(r["sim_s"]) >= cutoff]
    saved = metadata.get("activation_command_history")
    require(isinstance(saved, list) and len(saved) == len(before), "Activation history lacks exact retained prestream rows")
    history = []
    for item, row in zip(saved, before):
        now = float(row["sim_s"])
        match([item["control_time_s"], item["task_time_s"]], [now, now-window[0]], "activation time")
        match(item["command"], command(row), "activation command")
        history.append((now, command(row)))
    all_before_first = [r for r in control if float(r["sim_s"]) < times[0]-1e-9]
    require(all_before_first and key(all_before_first[-1]["sim_s"]) == key(history[-1][0]),
            "Activation history omits the actual final preceding command")
    errors = dict(initial_q=0., initial_dq=0., node_q=0., node_dq=0., base_state=0., rotation=0., tip=0.)
    for row in [*before, *selected]:
        now = float(row["sim_s"])
        names = ("joint:arm_joint_1", "joint:arm_joint_2", "px4_pva")
        batch = [publications.get((key(now), name)) for name in names]
        require(all(batch), "Missing actual joint/PVA publication")
        require([e["sequence"] for e in batch] == sorted(e["sequence"] for e in batch), "Publication order changed")
        for e in batch:
            context = e["context"]
            match([context["sim_s"], context["t_s"], context["control_dt_s"], context["measured_joint_stamp_s"]],
                  [now, float(row["t_s"]), float(row["control_dt_s"]), float(row["joint_stamp_s"])], "publication context")
            require(context["phase"] == row["phase"], "Publication phase differs")
            for side in ("before", "after"):
                observed = e["publish_"+side]
                match([observed["processed_estimator_clock_s"], observed["joint_header_stamp_s"]],
                      [now, float(row["joint_stamp_s"])], "publication held callback times")
        for i, e in enumerate(batch[:2], 1):
            require(e["payload"].get("joint") == "arm_joint_"+str(i), "Wrong joint publication")
            match(e["payload"]["velocity_rad_s"], float(row[f"qd{i}_cmd"]), "joint publication")
        payload = batch[2]["payload"]
        require(payload["coordinate_frame"] == int(float(row["sent_coordinate_frame"]))
                and payload["type_mask"] == int(float(row["sent_type_mask"])), "PVA interface changed")
        for name in ("position", "velocity", "acceleration"):
            match(payload[name], [float(row["sent_"+name+"_target_"+a]) for a in "ned"], "PVA "+name)
        match([payload["yaw"], payload["yaw_rate"]], [float(row["sent_yaw_ned"]), 0.], "PVA yaw")
        if row["phase"] != "measure":
            continue
        step, fb, d = steps[key(row["t_s"])], feedback[key(now)]["feedback"], diagnostics[key(now)]["diagnostics"]
        require(step["sequence"] < batch[0]["sequence"] and step["feedback_snapshot"] == fb,
                "Controller attempt did not consume the recorded raw feedback before publishing")
        match([step["t_s"], step["dt_s"]], [float(row["t_s"]), float(row["control_dt_s"])], "controller time")
        for side in ("before", "after"):
            observed = step["compute_"+side]
            match([observed["processed_estimator_clock_s"], observed["joint_header_stamp_s"]],
                  [now, float(row["joint_stamp_s"])], "controller held callback times")
        state = step["state"]
        mapping = {"position": "position_enu_m", "rotation": "rotation_flu_to_enu", "velocity": "velocity_enu_m_s",
            "omega_body": "omega_flu_rad_s", "q": "q_rad", "dq": "dq_rad_s", "acceleration_enu": "acceleration_enu_m_s2"}
        for field, source in mapping.items():
            match(state[field], fb["control_state"][source], "consumed "+field)
        match([state["control_time_s"], state["joint_stamp_s"], state["stamp_s"]],
              [now, float(row["joint_stamp_s"]), fb["state_stamp_s"]], "consumed state timestamps")
        match(step["target"]["position"], d["allocation"]["tip_ref"], "consumed endpoint target")
        if metadata["task"]["controller"] == "mpc":
            count = protocol["mpc_controller_config"]["nodes"]
            intervals = array(d["node_intervals_s"], (count,), "prediction intervals")
            match(intervals, [float(row["control_dt_s"]), *([protocol["mpc_controller_config"]["node_dt_s"]]*(count-1))], "node intervals")
            plan = array(d["predicted_commands"], (count, 5), "predicted commands")
            match(plan[0], command(row), "executed first prediction command")
            prediction = array(d["predicted_states"], (count, 12), "predicted states")
            joints = replay_joints(now, float(row["joint_stamp_s"]), state["q"], state["dq"],
                                  history, intervals, plan, settings["joint_delay_s"])
            for label, actual, expected in (("initial_q", d["prediction_initial_joint_position"], joints[0, 0]),
                ("initial_dq", d["prediction_initial_joint_velocity"], joints[0, 1]),
                ("node_q", prediction[:, 6:8], joints[1:, 0]), ("node_dq", prediction[:, 8:10], joints[1:, 1])):
                errors[label] = max(errors[label], match(actual, expected, label, 2e-8))
            response = metadata["controller_details"]["allocator"]["response_model"]["parameters"]
            base = replay_base(state, intervals, plan, response)
            errors["base_state"] = max(errors["base_state"], match(prediction[:, [0, 1, 2, 3, 4, 5, 10, 11]], base, "base states", 2e-8))
            rotations, tips, residual = replay_geometry(state, prediction, plan, np.cumsum(intervals),
                metadata["physical_profile"], settings, response)
            errors["rotation"] = max(errors["rotation"], match(d["predicted_rotations_flu_to_enu"], rotations, "attitude output", 2e-8))
            errors["tip"] = max(errors["tip"], match(d["predicted_tip_positions"], tips, "endpoint output", 2e-8))
            match(d["prediction_attitude_initial_residual_rotvec"], residual, "initial attitude residual", 2e-8)
        history.append((now, command(row)))
        history = [(when, value) for when, value in history if when >= now-.75]
    filenames = ("control_execution_trace.jsonl", "control_execution_trace_manifest.json",
                 "feedback_trace.jsonl", "controller_diagnostics.jsonl", "control_trace.csv", "controller_metadata.json")
    artifacts = {str(folder/name): sha(folder/name) for name in filenames}
    artifacts[str(Path(__file__).resolve())] = sha(__file__)
    return {"verified": True, "samples": len(selected), "activation_history_samples": len(saved),
        "prediction_replayed": metadata["task"]["controller"] == "mpc", "maximum_replay_errors": errors,
        "artifacts": artifacts, "optimizer_executed": False, "parameters_fitted": False,
        "checks": {"closed_hash_bound_trace": True, "actual_publications_match": True,
                   "actual_activation_history_match": True, "consumed_raw_feedback_match": True},
        "scope": "Frozen controller replay only; local publication return is not a Gazebo application acknowledgement or independent actuator identification"}
