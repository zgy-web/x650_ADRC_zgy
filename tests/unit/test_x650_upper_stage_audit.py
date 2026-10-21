"""Synthetic full-window protocol checks; these fixtures are not flight evidence."""
from copy import deepcopy
import csv
import importlib.util
import json
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace as NS
import unittest
from unittest.mock import patch
import zipfile

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(Path(__file__).parent))
import test_x650_precision_stage_audit as original_audit_tests
spec = importlib.util.spec_from_file_location("upper_stage_audit_test", ROOT/"scripts/audit_x650_upper_stage.py")
audit = importlib.util.module_from_spec(spec)
spec.loader.exec_module(audit)
from simulations.x650_wholebody_mpc_v1.run_case import response_probe_velocity


def save(path, document):
    Path(path).write_text(json.dumps(document, indent=2)+"\n", encoding="utf-8")


def write_csv(path, records):
    with Path(path).open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(records[0]))
        writer.writeheader()
        writer.writerows(records)


def response(t, command_t, command, tau):
    """Independent analytic step superposition, not identifier prediction code."""
    answer = np.zeros(len(t))
    changes = np.r_[command[0], np.diff(command)]
    for index in np.flatnonzero(changes):
        elapsed = t-command_t[index]
        positive = elapsed >= 0.
        answer[positive] += changes[index]*(1-np.exp(-elapsed[positive]/tau))
    return answer


def stream_fixture(seed=0):
    old = original_audit_tests.PrecisionResearchAuditTests()
    ulog, native, reference, window, applied, _ = old.stream_fixture(30.)
    reference["target_world_enu_m"] = [1., 2., 3.]
    reference["fixed_yaw_ned_rad"] = 0.
    t = np.round(np.arange(0., 30.0001, .02), 9)
    times = 10.+t
    u = np.asarray([response_probe_velocity(float(time), 30.) for time in t])
    lt = audit.precision.stamp(audit.precision.dataset(ulog, "vehicle_local_position"))
    v = np.column_stack([response(lt-10., t, u[:, i], tau) for i, tau in enumerate((.16, .2, .25))])
    # Independent runs have distinct tiny observation noise, not renamed copies.
    v += .00001*np.sin(lt[:, None]*[11., 13., 17.]+seed)
    local = audit.precision.dataset(ulog, "vehicle_local_position")
    for i, name in enumerate(("vx", "vy", "vz")):
        local[name] = (v@audit.NED_TO_ENU.T)[:, i]
    local.update(vxy_reset_counter=np.zeros(len(lt)), vz_reset_counter=np.zeros(len(lt)))
    gains = np.array([applied["MPC_XY_P"]["value"], applied["MPC_XY_P"]["value"], applied["MPC_Z_P"]["value"]])
    records, diagnostics = [], []
    for index, (now, task_t, command) in enumerate(zip(times, t, u)):
        pn, vn = np.array([2., 1., 7.]), np.interp(now, lt, v[:, 0])
        # Full velocity is sampled at callback time; no shifted source epoch.
        vw = np.array([np.interp(now, lt, v[:, axis]) for axis in range(3)])
        vn = audit.NED_TO_ENU@vw
        p = pn+(audit.NED_TO_ENU@command)/gains
        row = dict(sim_s=float(now), t_s=float(task_t), phase="measure", pva_sent="True",
            sent_coordinate_frame=1, sent_type_mask=2048, sent_yaw_ned=0.,
            feedback_snapshot_sequence=index+1, q1=0., q2=0., qd1=0., qd2=0., qd1_cmd=0., qd2_cmd=0.)
        for prefix, values, axes in (("sent_position_target_", p, "ned"), ("sent_velocity_target_", [0., 0., 0.], "ned"),
            ("sent_acceleration_target_", [0., 0., 0.], "ned"), ("feedback_position_", pn, "ned"),
            ("feedback_velocity_", vn, "ned"), ("feedback_p", [1., 2., 3.], "xyz"), ("feedback_v", vw, "xyz"),
            ("base_velocity_target_", command, "xyz"), ("command_acceleration_", [0., 0., 0.], "xyz")):
            row.update({prefix+axis: float(value) for axis, value in zip(axes, values)})
        callbacks = {name: dict(sequence=index+1, received_sim_s=float(now), received_wall_s=1000.+float(now),
                    sim_age_s=0., wall_age_s=0., new_callback=True) for name in
                    ("position", "attitude", "angular_velocity", "estimator_flags", "joints", "clock")}
        detail = dict(feedback_profile="px4_estimated_raw_ev_world_v1", snapshot_protocol="latest_asynchronous_receipt_age_guarded",
            truth_feedback=False, truth_clock=False, sim_time_s=float(now), state_stamp_s=float(now),
            joint_stamp_s=float(now), joint_source_age_s=0., callbacks=callbacks,
            px4_raw_timestamps={name: dict(timestamp=int(now*1e6), timestamp_sample=int(now*1e6)) for name in
                               ("position", "attitude", "angular_velocity", "estimator_flags")},
            px4_snapshot=dict(position_ned_m=pn.tolist(), velocity_ned_m_s=vn.tolist(), attitude_wxyz=[1., 0., 0., 0.],
                angular_velocity_frd_rad_s=[0., 0., 0.], origin=reference["px4_reference_at_bind"].copy(),
                extra_reset_counters=dict(vxy_reset_counter=0, vz_reset_counter=0, quat_reset_counter=0),
                valid={key: True for key in ("xy_valid", "z_valid", "v_xy_valid", "v_z_valid", "heading_good_for_control")},
                dead_reckoning=False, estimator_flags={**{key: bool(value) for key, value in audit.fixed_audit.EV_FLAGS.items()},
                    "cs_inertial_dead_reckoning": False, "cs_wind_dead_reckoning": False}),
            control_state=dict(position_enu_m=[1., 2., 3.], velocity_enu_m_s=vw.tolist(),
                rotation_flu_to_enu=(audit.NED_TO_ENU@audit.FRD_TO_FLU).tolist(), omega_flu_rad_s=[0., 0., 0.],
                q_rad=[0., 0.], dq_rad_s=[0., 0.]))
        records.append(row)
        diagnostics.append(dict(sim_s=float(now), t_s=float(task_t), phase="measure", feedback=detail))
    trajectory = {"timestamp": times*1e6, "yaw": np.zeros(len(times))}
    for part in ("position", "velocity", "acceleration"):
        for i, axis in enumerate("ned"):
            trajectory[f"{part}[{i}]"] = np.array([row[f"sent_{part}_target_{axis}"] for row in records])
    internal = {"timestamp": times*1e6, **{axis: (u@audit.NED_TO_ENU.T)[:, i] for i, axis in enumerate(("vx", "vy", "vz"))}}
    ulog.data_list = [dataset for dataset in ulog.data_list if dataset.name != "vehicle_local_position_setpoint"]
    ulog.data_list.extend(NS(name=name, multi_id=0, data=data) for name, data in
        (("trajectory_setpoint", trajectory), ("vehicle_local_position_setpoint", internal),
         ("vehicle_attitude", {"timestamp_sample": lt*1e6, "quat_reset_counter": np.zeros(len(lt))})))
    native["velocity"] = np.column_stack([np.interp(native["t"], lt, v[:, i]) for i in range(3)])
    pid = {key: float(value["value"]) for key, value in applied.items() if key in audit.fixed_audit.CONTROL_NAMES}
    return ulog, native, reference, window, pid, records, diagnostics


def complete_fixture(root, seed=0):
    """Full synthetic persisted provenance plus fake ULog loader (never real evidence)."""
    root.mkdir(parents=True, exist_ok=True)
    run, launch, _ = original_audit_tests.PrecisionResearchAuditTests().complete_fixture(root)
    ulog, native, reference, window, pid, controls, feedback = stream_fixture(seed)
    task = audit.read_json(run/"controller_task.json")
    task.update(stage="response_probe", duration_s=30.)
    task["research_profile"]["name"] = "precision_ev_upper_v2"
    condition = audit.condition_for_task(task)
    bound_reference = audit.read_json(run/"hover_reference.json")
    bound_reference["fixed_yaw_ned_rad"] = 0.
    save(run/"hover_reference.json", bound_reference)
    operating = dict(operating_condition=condition, operating_condition_sha256=audit.canonical_hash(condition),
                     pid_parameters_sha256=condition["pid_parameters_sha256"])
    meta = audit.read_json(run/"controller_metadata.json")
    meta.update(operating, task=task, stage="response_probe", controller="pid_ff", feedback_contract=condition["feedback_contract"],
                requested_duration_s=30., task_end_sim_s=40.)
    feedback_document = dict(kind="x650_px4_estimated_feedback", schema_version=1,
        profile="px4_estimated_raw_ev_world_v1", frame_mode="raw_ev_world", truth_feedback=False, truth_clock=False,
        reference=bound_reference, max_age_s={"state": .10, "flags": 1.25, "joint": .25, "wall": 4., "flags_wall": 10.},
        extra_reset_counters_at_bind={"vxy_reset_counter": 0, "vz_reset_counter": 0, "quat_reset_counter": 0})
    meta["estimated_feedback"] = feedback_document
    save(run/"estimated_feedback.json", feedback_document)
    provenance = meta["provenance"]
    provenance.update(airframe_sha256="c"*64, flight_profile_sha256="d"*64, preparation_report_sha256="e"*64)
    for filename in ("estimated_feedback.py", "run_case.py"):
        path = ROOT/"simulations/x650_wholebody_mpc_v1"/filename
        provenance["source_files_sha256"][path.relative_to(ROOT).as_posix()] = audit.precision.sha(path)
    for name in ("diagnostic_parameter_session.json", "diagnostic_restore_journal_completed.json", "diagnostic_odometry.json", "native_ground.json"):
        document = audit.read_json(run/name)
        document.update(operating)
        save(run/name, document)
    native_rows = audit.rows(run/"native_trace.csv")
    template = native_rows[0]
    expanded = []
    for index, now in enumerate(native["t"]):
        row = {**template, "sim_s": float(now)}
        row.update({"base_v"+axis: float(native["velocity"][index, i]) for i, axis in enumerate("xyz")})
        expanded.append(row)
    write_csv(run/"native_trace.csv", expanded)
    manifest = audit.read_json(run/"native_trace_manifest.json")
    manifest.update(sha256=audit.precision.sha(run/"native_trace.csv"), task_window_sim_s=window)
    save(run/"native_trace_manifest.json", manifest)
    write_csv(run/"control_trace.csv", controls)
    (run/"feedback_trace.jsonl").write_text("\n".join(json.dumps(item) for item in feedback)+"\n", encoding="utf-8")
    flight = [{"phase": "measure", "sim_time_s": float(now), "failsafe": 0, "armed": 1, "landed": 0, "nav_state": 14}
              for now in np.arange(10., 40.0001, .02)]
    write_csv(run/"flight_samples.csv", flight)
    save(run/"controller_metadata.json", meta)
    save(run/"controller_task.json", task)
    stage = audit.read_json(run/"stage_result.json")
    stage.update(operating, stage="response_probe", artifacts={name: audit.precision.sha(run/name) for name in
                 ("native_trace.csv", "control_trace.csv", "feedback_trace.jsonl")})
    save(run/"stage_result.json", stage)
    save(launch/"task.json", task)
    save(launch/"launch.json", dict(complete=True, runtime_closed=True, exit_code=0, run_directories=[str(run)], task=task, provenance=provenance))
    with zipfile.ZipFile(launch/"controller_sources.zip", "w") as archive:
        for path in provenance["source_files_sha256"]:
            archive.write(ROOT/path, path)
    return run, launch, ulog


class UpperStageAuditTests(unittest.TestCase):
    def test_feedback_snapshot_accepts_real_receipt_clock_not_raw_dds_epoch(self):
        _, _, reference, window, _, control, feedback = stream_fixture()
        feedback[1]["feedback"]["px4_raw_timestamps"]["position"]["timestamp"] = 0
        report, _ = audit.feedback_check(control, feedback, reference, window)
        self.assertTrue(report["verified"])
        self.assertFalse(report["truth_clock"])
        self.assertEqual(report["sample_count"], 1501)

    def test_truth_clock_stale_callback_reset_wrong_state_and_joint_time_rejected(self):
        for fault in ("truth", "stale", "reset", "state", "joint"):
            with self.subTest(fault=fault):
                _, _, reference, window, _, control, feedback = stream_fixture()
                detail = feedback[80]["feedback"]
                if fault == "truth": detail["truth_clock"] = True
                elif fault == "stale": detail["callbacks"]["position"]["sim_age_s"] = .2
                elif fault == "reset": detail["px4_snapshot"]["extra_reset_counters"]["vz_reset_counter"] = 1
                elif fault == "state": control[80]["feedback_px"] += .001
                else: detail["joint_stamp_s"] -= 1.
                with self.assertRaises(ValueError):
                    audit.feedback_check(control, feedback, reference, window)

    def test_one_second_flag_publication_has_its_own_bounded_sim_and_wall_ages(self):
        _, _, reference, window, _, control, feedback = stream_fixture()
        for entry in feedback:
            callback = entry["feedback"]["callbacks"]["estimator_flags"]
            callback.update(received_sim_s=entry["sim_s"]-1.008, sim_age_s=1.008,
                            received_wall_s=1000.+entry["sim_s"]-4.638, wall_age_s=4.638)
        report, _ = audit.feedback_check(control, feedback, reference, window)
        self.assertAlmostEqual(report["maximum_callback_receipt_age_s"]["estimator_flags"], 1.008)
        for fault in ("flags_sim", "flags_wall", "ordinary_wall"):
            changed = deepcopy(feedback)
            name = "position" if fault == "ordinary_wall" else "estimator_flags"
            callback = changed[0]["feedback"]["callbacks"][name]
            if fault == "flags_sim":
                callback.update(received_sim_s=changed[0]["sim_s"]-1.251, sim_age_s=1.251)
            else:
                callback["wall_age_s"] = 4.001 if fault == "ordinary_wall" else 10.001
            with self.subTest(fault=fault), self.assertRaisesRegex(ValueError, "receipt age"):
                audit.feedback_check(control, changed, reference, window)

    def test_original_time_sent_received_and_internal_semantics_pass(self):
        ulog, native, reference, window, pid, control, _ = stream_fixture()
        report = audit.upper_commands(ulog, native, reference, control, window, pid, response=True)
        self.assertTrue(report["all_measurement_sends_observed"])
        self.assertLess(max(report["internal_minus_received_pva_reconstruction"]["axis_rmse"]), 1e-12)
        self.assertTrue(audit.velocity_proxy(ulog, native, reference)["passed"])

    def test_modified_send_receive_internal_or_ff_cannot_be_audited_away(self):
        for fault in ("send", "receive", "internal", "ff", "delay"):
            with self.subTest(fault=fault):
                ulog, native, reference, window, pid, control, _ = stream_fixture()
                if fault == "send": control[100]["sent_position_target_n"] += .01
                elif fault == "receive": audit.precision.dataset(ulog, "trajectory_setpoint")["velocity[0]"][100] = .5
                elif fault == "internal": audit.precision.dataset(ulog, "vehicle_local_position_setpoint")["vx"][:] += .03
                elif fault == "ff":
                    control[100]["sent_acceleration_target_n"] = .1
                    control[100]["command_acceleration_y"] = .1
                else: audit.precision.dataset(ulog, "trajectory_setpoint")["timestamp"] += .2e6
                with self.assertRaises(ValueError):
                    audit.upper_commands(ulog, native, reference, control, window, pid, response=True)

    def test_nav_ignored_nan_before_after_measurement_does_not_pollute_complete_pva(self):
        ulog, native, reference, window, pid, control, _ = stream_fixture()
        trajectory = audit.precision.dataset(ulog, "trajectory_setpoint")
        for key, value in trajectory.items():
            trajectory[key] = np.r_[9.99e6 if key == "timestamp" else np.nan, value,
                                     40.01e6 if key == "timestamp" else np.nan]
        report = audit.upper_commands(ulog, native, reference, control, window, pid, response=True)
        self.assertTrue(report["all_measurement_sends_observed"])
        self.assertEqual(report["received_rows"], 1501)
        trajectory["velocity[0]"][101] = np.nan
        with self.assertRaisesRegex(ValueError, "Measurement received PVA"):
            audit.upper_commands(ulog, native, reference, control, window, pid, response=True)

    def test_nan_held_at_first_measurement_is_not_filled_from_future_pva(self):
        ulog, native, reference, window, pid, control, _ = stream_fixture()
        trajectory = audit.precision.dataset(ulog, "trajectory_setpoint")
        # First complete target arrives 4 ms after the first internal query.
        # Send/receive matching alone can pass, but the real held target is NAV.
        trajectory["timestamp"][0] += .004e6
        for key, value in trajectory.items():
            trajectory[key] = np.r_[9.99e6 if key == "timestamp" else np.nan, value]
        with self.assertRaisesRegex(ValueError, "Nonfinite ULog values"):
            audit.upper_commands(ulog, native, reference, control, window, pid, response=True)

    def test_final_internal_query_holds_last_valid_pva_before_later_nav(self):
        ulog, native, reference, window, pid, control, _ = stream_fixture()
        trajectory = audit.precision.dataset(ulog, "trajectory_setpoint")
        for key, value in trajectory.items():
            trajectory[key] = np.r_[value[:-1], 40.5e6 if key == "timestamp" else np.nan]
        # Control/send ends at 39.98, internal query ends at 40.00. Its held
        # value is still the last complete PVA, never the later NAV record.
        later_send = {**control[-1], "sim_s": 40.02, "t_s": 30.02, "phase": "recovery"}
        report = audit.upper_commands(ulog, native, reference, [*control[:-1], later_send], window, pid, response=True)
        self.assertTrue(report["verified"])

    def test_proxy_rms_limit_is_not_recentered_or_delay_fitted(self):
        ulog, native, reference, _, _, _, _ = stream_fixture()
        audit.precision.dataset(ulog, "vehicle_local_position")["vz"] += .0101
        with self.assertRaisesRegex(ValueError, "proxy exceeds"):
            audit.velocity_proxy(ulog, native, reference)

    def test_complete_original_provisional_run_issues_separate_acceptance(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            run, launch, ulog = complete_fixture(root)
            original = (run/"stage_result.json").read_bytes()
            report = audit.audit(run, launch, root/"accepted", ulog_loader=lambda _: ulog)
            self.assertTrue(report["passed"])
            accepted = audit.read_json(root/"accepted/research_stage.json")
            self.assertTrue(accepted["upper_command_verified"])
            self.assertTrue(accepted["feedback_contract_verified"])
            self.assertEqual((run/"stage_result.json").read_bytes(), original)
            for path, digest in accepted["artifacts"].items():
                self.assertEqual(audit.precision.sha(path), digest)

    def test_failure_has_no_certificate_and_cannot_overwrite(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            with patch.object(audit, "analyze", side_effect=ValueError("original run failed")):
                with self.assertRaisesRegex(ValueError, "original run failed"):
                    audit.audit(root/"run", root/"launch", root/"failed")
            self.assertFalse(audit.read_json(root/"failed/upper_stage_audit.json")["passed"])
            self.assertFalse((root/"failed/research_stage.json").exists())
            with self.assertRaisesRegex(ValueError, "new upper audit directory"):
                audit.audit(root/"run", root/"launch", root/"failed")

    def test_native_physical_failure_retains_failure_report_without_certificate(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            run, launch, ulog = complete_fixture(root)
            physical = audit.rows(run/"native_trace.csv")
            physical[1000]["arm_joint_1_q"] = .04
            write_csv(run/"native_trace.csv", physical)
            manifest = audit.read_json(run/"native_trace_manifest.json")
            manifest["sha256"] = audit.precision.sha(run/"native_trace.csv")
            save(run/"native_trace_manifest.json", manifest)
            stage = audit.read_json(run/"stage_result.json")
            stage["artifacts"]["native_trace.csv"] = manifest["sha256"]
            save(run/"stage_result.json", stage)
            with self.assertRaisesRegex(ValueError, "physical stage failed"):
                audit.audit(run, launch, root/"rejected", ulog_loader=lambda _: ulog)
            report = audit.read_json(root/"rejected/upper_stage_audit.json")
            self.assertTrue(report["complete"])
            self.assertFalse(report["passed"])
            self.assertFalse(report["research_acceptance_verified"])
            self.assertFalse((root/"rejected/research_stage.json").exists())


if __name__ == "__main__":
    unittest.main()
