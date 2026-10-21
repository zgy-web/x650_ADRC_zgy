"""Synthetic postflight evidence tests; no simulated/hardware performance claim."""
from copy import deepcopy
import csv
import importlib.util
import json
from pathlib import Path
import tempfile
from types import SimpleNamespace as NS
import unittest
from unittest.mock import patch
import zipfile

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
spec = importlib.util.spec_from_file_location("precision_stage_audit_test", ROOT/"scripts/audit_x650_precision_stage.py")
audit = importlib.util.module_from_spec(spec)
spec.loader.exec_module(audit)


class PrecisionResearchAuditTests(unittest.TestCase):
    def parameter_fixture(self):
        before = {name: audit.expected_record(name, 1.) for name in audit.CONTROL_NAMES}
        before.update({name: audit.expected_record(name, 0 if audit.expected_record(name, value)["param_type"] == 6 else .02)
                       for name, value in audit.PRECISION_PROFILE.items()})
        before.update({name: audit.expected_record(name, 0.) for name in audit.PRECISION_REQUIRED})
        applied = deepcopy(before)
        applied.update({name: audit.expected_record(name, value) for name, value in audit.EXPECTED_WRITES.items()})
        pid = {name: float(applied[name]["value"]) for name in audit.CONTROL_NAMES}
        condition = {"pid_parameters_sha256": audit.canonical_hash(pid)}
        save = {"returncode": 0, "synchronous_default_save": True}
        session = {"purpose": "research", "diagnostic_only": False,
                   "operating_condition": condition, "operating_condition_sha256": audit.canonical_hash(condition),
                   "mutable_parameters": audit.EXPECTED_WRITES.copy(),
                   "before": before, "applied": applied, "restored": before,
                   "restore_post_save_readback": before, "restore_storage_save": save,
                   **{k: True for k in ("applied_verified", "restored_verified", "runtime_restored_verified", "restore_persistent_storage_verified")},
                   "write_attempts": [{"phase": phase, "name": name, "desired": source[name], "readback": source[name], "verified": True}
                       for phase, source in (("apply", applied), ("restore", before)) for name in audit.EXPECTED_WRITES]}
        journal = {"allowlist": audit.EXPECTED_WRITES.copy(), "original": before, "restored": before,
                   "restored_verified": True, "status": "restored_readback_and_synchronous_storage_save_verified", "storage_save": save,
                   "purpose": "research", "diagnostic_only": False, "operating_condition": condition,
                   "operating_condition_sha256": audit.canonical_hash(condition)}
        snapshot = {"contract_matches": True, "parameters": before}
        documents = dict(session=session, journal=journal, before=before, applied=applied, restored=before,
                         px4_before=snapshot, px4_after=deepcopy(snapshot))
        return documents, {"actual_pid_parameters": pid}, condition, pid

    def test_applied_42_are_distinct_from_restored_and_bytes_checked(self):
        documents, meta, condition, pid = self.parameter_fixture()
        report = audit.verify_parameter_session(documents, meta, condition, pid)
        self.assertEqual(len(report["applied_control_42"]), 42)
        self.assertAlmostEqual(report["applied_control_42"]["MPC_XY_VEL_P_ACC"], 2.2, places=6)
        self.assertNotEqual(documents["before"]["MPC_XY_VEL_P_ACC"], documents["applied"]["MPC_XY_VEL_P_ACC"])
        documents["applied"]["MPC_XY_VEL_P_ACC"]["value"] = 1.8
        with self.assertRaisesRegex(ValueError, "bytes/value"):
            audit.verify_parameter_session(documents, meta, condition, pid)

    def test_restore_mismatch_or_extra_write_or_old_diagnostic_cannot_pass(self):
        for fault in ("restore", "extra", "legacy"):
            with self.subTest(fault=fault):
                documents, meta, condition, pid = self.parameter_fixture()
                if fault == "restore":
                    documents["restored"] = deepcopy(documents["restored"])
                    documents["restored"]["MC_ROLL_P"] = audit.expected_record("MC_ROLL_P", 9.)
                elif fault == "extra":
                    documents["session"]["write_attempts"].append({"phase": "apply", "name": "MC_ROLL_P"})
                else:
                    documents["session"]["purpose"] = "diagnostic"
                with self.assertRaises(ValueError):
                    audit.verify_parameter_session(documents, meta, condition, pid)

    def test_frozen_42_or_unverified_storage_or_missing_apply_rejected(self):
        for fault in ("pid", "storage", "missing_apply"):
            with self.subTest(fault=fault):
                documents, meta, condition, pid = self.parameter_fixture()
                if fault == "pid":
                    pid = {**pid, "MC_PITCH_P": 9.}
                elif fault == "storage":
                    documents["session"]["restore_storage_save"] = {"returncode": 1, "synchronous_default_save": True}
                else:
                    documents["session"]["write_attempts"] = documents["session"]["write_attempts"][1:]
                with self.assertRaises(ValueError):
                    audit.verify_parameter_session(documents, meta, condition, pid)

    def stream_fixture(self, duration=30.):
        t = np.arange(9.98, 10.+duration+.0201, .004)
        sample = np.arange(10., 10.+duration+.0001, .004)
        bound = dict(ref_lat=47., ref_lon=8., ref_alt=10., xy_valid=True, z_valid=True,
                     xy_global=False, z_global=True, xy_reset_counter=0, z_reset_counter=0, heading_reset_counter=0)
        reference = {"frame_mode": "raw_ev_world", "px4_reference_at_bind": bound, "target_local_ned_m": [2., 1., 7.]}
        local = {"timestamp_sample": t*1e6, **{name: np.full(len(t), val) for name, val in bound.items()}}
        local.update({name: np.full(len(t), val) for name, val in zip(("x", "y", "z", "vx", "vy", "vz"), (2., 1., 7., 0., 0., 0.))})
        flags = {"timestamp": t*1e6, **{name: np.full(len(t), value) for name, value in audit.EV_FLAGS.items()}}
        sp = {"timestamp": t*1e6, **{name: np.full(len(t), val) for name, val in zip(("x", "y", "z"), (2., 1., 7.))}}
        docs, _, _, _ = self.parameter_fixture()
        params = {name: record["value"] for name, record in docs["applied"].items()}
        ulog = NS(initial_parameters=params, changed_parameters=[], data_list=[
            NS(name=name, multi_id=0, data=data) for name, data in
            (("vehicle_local_position", local), ("vehicle_local_position_setpoint", sp), ("estimator_status_flags", flags))])
        rows = []
        for time in np.arange(10., 10.+duration+.0001, .02):
            row = {"sim_s": str(time), "phase": "measure", "fixed_world_position_sent": "True",
                   **{"fixed_position_target_"+key: str(value) for key, value in zip("ned", (2., 1., 7.))},
                   **{key+axis: "0" for key in ("base_velocity_target_", "command_acceleration_") for axis in "xyz"}}
            rows.append(row)
        return ulog, {"t": sample}, reference, [10., 10.+duration], docs["applied"], rows

    def test_joint_full_30_seconds_checked_not_only_first_20(self):
        ulog, native, reference, window, applied, _ = self.stream_fixture()
        result = audit.effective_configuration(ulog, native, reference, window, applied)
        self.assertEqual(result["analysis_window_sim_s"], [10., 40.])
        flags = audit.precision.dataset(ulog, "estimator_status_flags")
        flags["cs_gnss_vel"][np.searchsorted(flags["timestamp"], 35e6)] = 1
        with self.assertRaisesRegex(ValueError, "fusion differs"):
            audit.effective_configuration(ulog, native, reference, window, applied)

    def test_joint_tail_reset_gain_change_and_missing_state_samples_rejected(self):
        for fault in ("reset", "gain", "gap"):
            with self.subTest(fault=fault):
                ulog, native, reference, window, applied, _ = self.stream_fixture()
                local = audit.precision.dataset(ulog, "vehicle_local_position")
                if fault == "reset":
                    local["xy_reset_counter"][np.searchsorted(local["timestamp_sample"], 35e6)] = 1
                elif fault == "gain":
                    ulog.changed_parameters.append((35e6, "MPC_XY_VEL_P_ACC", 1.8))
                else:
                    keep = ~((local["timestamp_sample"] > 35e6)&(local["timestamp_sample"] < 35.04e6))
                    for name in local:
                        local[name] = local[name][keep]
                with self.assertRaises(ValueError):
                    audit.effective_configuration(ulog, native, reference, window, applied)

    def test_constant_received_target_and_pure_fixed_commands_required(self):
        ulog, native, reference, window, _, rows = self.stream_fixture()
        self.assertTrue(audit.fixed_position_commands(ulog, native, reference, rows, window)["verified"])
        sp = audit.precision.dataset(ulog, "vehicle_local_position_setpoint")
        sp["x"][np.searchsorted(sp["timestamp"], 35e6)] += .001
        with self.assertRaisesRegex(ValueError, "not constant"):
            audit.fixed_position_commands(ulog, native, reference, rows, window)
        sp["x"][:] = 2.
        rows[-2]["command_acceleration_x"] = ".01"
        with self.assertRaisesRegex(ValueError, "Upper velocity/acceleration"):
            audit.fixed_position_commands(ulog, native, reference, rows, window)

    def test_source_zip_mismatch_and_nonowned_launch_fail(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp); run = root/"run"; run.mkdir(); launch = root/"launch"; launch.mkdir()
            task = {"stage": "hover_folded"}
            source = b"unaltered source\n"
            hashes = {"safe/file.py": audit.hashlib.sha256(source).hexdigest()}
            meta = {"provenance": {"source_files_sha256": hashes}}
            values = {"complete": True, "runtime_closed": True, "exit_code": 0,
                      "run_directories": [str(run)], "task": task, "provenance": meta["provenance"]}
            (launch/"launch.json").write_text(json.dumps(values)); (launch/"task.json").write_text(json.dumps(task))
            with zipfile.ZipFile(launch/"controller_sources.zip", "w") as z:
                z.writestr("safe/file.py", b"changed")
            with self.assertRaisesRegex(ValueError, "Archived source checksum"):
                audit.source_and_launch_evidence(launch, run, meta, task)
            values["run_directories"] = [str(root/"other")]
            (launch/"launch.json").write_text(json.dumps(values))
            with self.assertRaisesRegex(ValueError, "exactly this"):
                audit.source_and_launch_evidence(launch, run, meta, task)

    def test_failed_or_rejected_run_never_emits_acceptance_and_never_overwrites(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp); run = root/"run"; run.mkdir(); original = run/"stage_result.json"
            original.write_text('{"passed": false}')
            output = root/"audit"
            with patch.object(audit, "_analyze", side_effect=ValueError("Incomplete/failed native flight")):
                with self.assertRaisesRegex(ValueError, "Incomplete/failed"):
                    audit.audit(run, root/"launch", output)
            self.assertEqual(original.read_text(), '{"passed": false}')
            report = json.loads((output/"precision_stage_audit.json").read_text())
            self.assertFalse(report["passed"])
            self.assertFalse((output/"research_stage.json").exists())
            self.assertFalse((output/"research_ground.json").exists())
            with self.assertRaisesRegex(ValueError, "new audit directory"):
                audit.audit(run, root/"launch", output)

    def complete_fixture(self, root):
        """One entirely synthetic 20 s valid run, including original hashes."""
        run, launch = root/"run", root/"launch"
        run.mkdir(); launch.mkdir()
        def save(path, obj):
            path.write_text(json.dumps(obj, indent=2), encoding="utf-8")
        world = root/"world.sdf"
        world.write_text('<sdf version="1.9"><world name="default"><spherical_coordinates><surface_model>EARTH_WGS84</surface_model><latitude_deg>47</latitude_deg><longitude_deg>8</longitude_deg><elevation>0</elevation><heading_deg>0</heading_deg></spherical_coordinates></world></sdf>')
        documents, meta, _, pid = self.parameter_fixture()
        model, urdf = "a"*64, "b"*64
        profile = root/"pid.json"
        save(profile, {"model_sha256": model, "parameters": pid, "pid_parameters_sha256": audit.canonical_hash(pid)})
        task = {"stage": "hover_folded", "controller": "pid_ff", "duration_s": 20.,
                "research_profile": {"name": "precision_ev_selected_pid_v1", "world_sdf": str(world)},
                "hover_reference_profile": {"name": "fixed_world_position_v1", "world_sdf": str(world)},
                "pid_profile": str(profile), "diagnostic_profile": None, "pid_tuning_candidate": None, "precision_test_profile": None}
        condition = audit.condition_for_task(task)
        operating = {"operating_condition": condition, "operating_condition_sha256": audit.canonical_hash(condition)}
        provisional = {**operating, "complete": True, "passed": True, "success": True,
                       "landed_and_disarmed": True, "diagnostic_only": False, "ground_truth_aided": True,
                       "eligible_as_stage_gate": False, "acceptance_pending": "full_ulog_audit",
                       "model_sha256": model, "source_urdf_sha256": urdf}
        sources = {"simulations/x650_wholebody_mpc_v1/"+name: audit.precision.sha(ROOT/"simulations/x650_wholebody_mpc_v1"/name)
                   for name in ("hover_reference.py", "diagnostic_parameters.py", "diagnostic_odometry.py", "research_condition.py")}
        provenance = {"model_sha256": model, "source_urdf_sha256": urdf, "source_files_sha256": sources}
        meta.update(provisional, task=task, stage="hover_folded", provenance=provenance,
                    allocator_active=False, external_force_disturbance=False, diagnostic_parameters_restored=True,
                    diagnostic_restore_pending=False, reference_base_enu_m=[1., 2., 3.],
                    requested_duration_s=20., task_start_sim_s=10., task_end_sim_s=30.)
        for key in ("px4_before", "px4_after"):
            documents[key]["model_sha256"] = model
        documents["session"].update(operating, source_sha256=sources["simulations/x650_wholebody_mpc_v1/diagnostic_parameters.py"])
        documents["journal"].update(operating)
        for key, name in {"session": "diagnostic_parameter_session.json", "journal": "diagnostic_restore_journal_completed.json",
                          "before": "diagnostic_parameters_before.json", "applied": "diagnostic_parameters_applied.json",
                          "restored": "diagnostic_parameters_restored.json", "px4_before": "px4_parameters_before.json",
                          "px4_after": "px4_parameters_after.json"}.items():
            save(run/name, documents[key])
        (run/"diagnostic_parameter_storage_save.log").write_text("Synthetic success")
        (run/"observer.log").write_text("Synthetic observer")
        observer = {**operating, "purpose": "research", "ideal_data": True, "diagnostic_only": False,
                    "requested_rate_hz": 50., "gaussian_noise": 0., "model_snapshot_unmodified": True,
                    "physical_model_changes": [], "aiding_confirmed": True, "input_stream_confirmed": True,
                    "observer_loaded": True, "source_sha256": sources["simulations/x650_wholebody_mpc_v1/diagnostic_odometry.py"],
                    "artifacts": {"observer.log": audit.precision.sha(run/"observer.log")}}
        save(run/"diagnostic_odometry.json", observer)
        ulog, native, reference, window, _, command_rows = self.stream_fixture(20.)
        reference.update(kind="x650_fixed_world_hover_reference", schema_version=1, profile="fixed_world_position_v1",
                         target_frozen=True, truth_feedback_in_setpoint=False, fit_estimator_translation=False,
                         coordinate_frame=1, type_mask=2552, px4_source_contract_commit=audit.precision.PINNED_PX4,
                         source_sha256=audit.precision.sha(audit.precision.DEFAULT_REFERENCE_SOURCE),
                         target_world_enu_m=[1., 2., 3.], world={"source_sha256": audit.precision.sha(world)})
        save(run/"hover_reference.json", reference)
        sp = audit.precision.dataset(ulog, "vehicle_local_position_setpoint")
        for key in ("vx", "vy", "vz"):
            sp[key] = np.zeros(len(sp["timestamp"]))
        gt = {"timestamp_sample": sp["timestamp"], **{key: np.zeros(len(sp["timestamp"])) for key in ("vx", "vy", "vz")}}
        ulog.data_list.append(NS(name="vehicle_local_position_groundtruth", multi_id=0, data=gt))
        fields = ["sim_s", "valid_base", "valid_joint_state", "valid_tip", *["base_p"+a for a in "xyz"],
                  *["base_q"+a for a in "wxyz"], *["base_v"+a for a in "xyz"], "arm_joint_1_q", "arm_joint_2_q",
                  "arm_joint_1_qd", "arm_joint_2_qd", *["tip_"+a for a in "xyz"]]
        for name, times in (("native_trace.csv", native["t"]), ("native_ground.csv", np.arange(1., 6.0001, .004))):
            with (run/name).open("w", newline="", encoding="utf-8") as stream:
                writer = csv.DictWriter(stream, fieldnames=fields); writer.writeheader()
                for time in times:
                    row = dict.fromkeys(fields, 0)
                    row.update(sim_s=float(time), valid_base=1, valid_joint_state=1, valid_tip=1,
                               base_px=1., base_py=2., base_pz=3., base_qw=1., tip_x=1., tip_y=2., tip_z=2.8)
                    writer.writerow(row)
            save(run/(Path(name).stem+"_manifest.json"), {"complete": True, "file": name,
                 "sha256": audit.precision.sha(run/name), "model_sha256": model, "source_urdf_sha256": urdf,
                 "task_window_sim_s": [10., 30.] if name == "native_trace.csv" else [1., 6.]})
        with (run/"control_trace.csv").open("w", newline="", encoding="utf-8") as stream:
            writer = csv.DictWriter(stream, fieldnames=list(command_rows[0])); writer.writeheader(); writer.writerows(command_rows)
        flight_rows = [{"phase": "measure", "sim_time_s": float(time), "failsafe": 0, "armed": 1, "landed": 0, "nav_state": 14,
                        "target_world_x_m": 1., "target_world_y_m": 2., "target_world_z_m": 3.}
                       for time in np.arange(10., 30.0001, .05)]
        with (run/"flight_samples.csv").open("w", newline="", encoding="utf-8") as stream:
            writer = csv.DictWriter(stream, fieldnames=list(flight_rows[0])); writer.writeheader(); writer.writerows(flight_rows)
        save(run/"native_ground.json", {**provisional, "stage": "native_ground", "interval_sim_s": [1., 6.],
             "artifacts": {name: audit.precision.sha(run/name) for name in ("native_ground.csv", "native_ground_manifest.json")}})
        save(run/"controller_metadata.json", meta); save(run/"controller_task.json", task)
        (run/"px4.ulg").write_bytes(b"SYNTHETIC ULOG LOADER FIXTURE")
        save(run/"stage_result.json", {**provisional, "stage": "hover_folded",
             "artifacts": {name: audit.precision.sha(run/name) for name in ("native_trace.csv", "control_trace.csv")}})
        save(launch/"task.json", task)
        save(launch/"launch.json", {"complete": True, "runtime_closed": True, "exit_code": 0,
             "run_directories": [str(run)], "task": task, "provenance": provenance})
        with zipfile.ZipFile(launch/"controller_sources.zip", "w") as z:
            for name in sources:
                z.write(ROOT/name, name)
        return run, launch, ulog

    def test_complete_synthetic_research_run_certifies_and_failed_native_cannot(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            run, launch, ulog = self.complete_fixture(root)
            report = audit.audit(run, launch, root/"valid_audit", ulog_loader=lambda _: ulog)
            self.assertTrue(report["passed"])
            accepted = json.loads((root/"valid_audit"/"research_stage.json").read_text())
            self.assertTrue(accepted["eligible_as_stage_gate"])
            self.assertTrue(accepted["ground_truth_aided"])
            self.assertFalse(json.loads((run/"stage_result.json").read_text())["eligible_as_stage_gate"])
            for name in ("stage_result.json", "controller_metadata.json", "px4.ulg", "precision_stage_audit.json"):
                self.assertTrue(any(Path(path).name == name for path in accepted["artifacts"]))
            meta = json.loads((run/"controller_metadata.json").read_text()); meta["complete"] = False
            (run/"controller_metadata.json").write_text(json.dumps(meta))
            with self.assertRaisesRegex(ValueError, "Incomplete/failed"):
                audit.audit(run, launch, root/"failed_audit", ulog_loader=lambda _: ulog)
            self.assertFalse((root/"failed_audit"/"research_stage.json").exists())

    def test_joint_sign_tracking_gate_uses_original_limits_and_reports_precision_separately(self):
        with tempfile.TemporaryDirectory() as temp:
            folder = Path(temp)
            times = np.arange(0., 30.0001, .004)
            desired, rates = audit.joint_audit.reference(times, 30.)
            profile = {"profile_name": "x650_2link_v2", "mount_body": [0., 0., -.18],
                       "elbow_offset_body_m": [0., .04, 0.], "angle_offsets": [2.6179938779914944, 0.], "lengths": [.25, .25]}
            control_t = np.arange(0., 30.0001, .02)
            qref, dqref = audit.joint_audit.reference(control_t, 30.)
            controls = [{"phase": "measure", "sim_s": float(t+10.), "t_s": float(t), "q1": float(q[0]), "q2": float(q[1]),
                         "qd1_cmd": float(dq[0]), "qd2_cmd": float(dq[1])} for t, q, dq in zip(control_t, qref, dqref)]
            def write_motion(sign):
                q, dq = desired*sign, rates*sign
                angles = np.column_stack((q[:, 0], q.sum(axis=1)))+profile["angle_offsets"]
                tip = np.tile([0., .04, 2.-.18], (len(times), 1))
                for index in (0, 1):
                    tip += .25*np.column_stack((np.cos(angles[:, index]), np.zeros(len(times)), -np.sin(angles[:, index])))
                rows = [{"sim_s": float(t+10.), "valid_base": 1, "valid_joint_state": 1, "valid_tip": 1,
                         "base_px": 0., "base_py": 0., "base_pz": 2., "base_qw": 1., "base_qx": 0., "base_qy": 0., "base_qz": 0.,
                         "arm_joint_1_q": float(q[i, 0]), "arm_joint_2_q": float(q[i, 1]),
                         "arm_joint_1_qd": float(dq[i, 0]), "arm_joint_2_qd": float(dq[i, 1]),
                         "tip_x": float(tip[i, 0]), "tip_y": float(tip[i, 1]), "tip_z": float(tip[i, 2])} for i, t in enumerate(times)]
                with (folder/"native_trace.csv").open("w", newline="", encoding="utf-8") as stream:
                    writer = csv.DictWriter(stream, fieldnames=list(rows[0])); writer.writeheader(); writer.writerows(rows)
            meta = {"requested_duration_s": 30., "physical_profile": profile}
            write_motion(1)
            report = audit.joint_stage_check(folder, meta, [10., 40.004], [0., 0., 2.], controls)
            self.assertTrue(report["passed"])
            self.assertFalse(report["precision_hover_threshold_applied_to_joint_probe"])
            write_motion(-1)
            report = audit.joint_stage_check(folder, meta, [10., 40.004], [0., 0., 2.], controls)
            self.assertFalse(report["passed"])
            self.assertFalse(report["criteria"]["command_sign_is_correct"])


if __name__ == "__main__":
    unittest.main()
