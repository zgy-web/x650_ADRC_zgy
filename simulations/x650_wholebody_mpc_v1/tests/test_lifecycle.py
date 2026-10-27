"""Mock lifecycle only: no ROS, PX4, Gazebo or physical result is represented."""
from __future__ import annotations
from contextlib import ExitStack
import csv
import hashlib
import json
from pathlib import Path
import sys
import tempfile
import time
from types import SimpleNamespace as NS
import unittest
from unittest.mock import patch
import numpy as np

HERE = Path(__file__).resolve().parents[1]
ROOT = HERE.parents[1]
for p in (HERE, ROOT/"control_packages/x650_wholebody_v1", ROOT/"control_packages/x650_wholebody_v1/tests"):
    sys.path.insert(0, str(p))
import run_case as runner
from recording import FlightRecorder, NativeRecorder
from task import build_task_config, required_gates
from test_model import example_model
from test_recording import FIELDS, ros_message
from diagnostic_parameters import (PROFILE as DIAGNOSTIC_PROFILE, DiagnosticParameterSession, expected_record,
                                  PID_CANDIDATES, load_parameter_evidence as parameter_evidence_module)


class MockWorld:
    def __init__(self, model, fail_takeoff=False):
        self.model, self.fail_takeoff = model, fail_takeoff
        self.sim, self.z, self.q, self.dq = 1., .37, np.zeros(2), np.zeros(2)
        self.events, self.takeoff_time, self.native_stream, self.native_writer = [], None, None, None
        self.sent_references = []
        self.capture = None
        self.node = NS(data={}, received={}, spin_once=lambda **kw: None, destroy_subscription=lambda sub: None,
                       joint_values=lambda: {j: NS(position_rad=self.q[i], velocity_rad_s=self.dq[i]) for i, j in enumerate(runner.JOINTS)},
                       sim_time_s=lambda: self.sim)
        self.last_land_publish_sim, self.last_land_value = -np.inf, None
        self.node.data.update(status=NS(arming_state=1, failsafe=False, nav_state=5, timestamp=1000000),
            land=NS(landed=True, timestamp=1000000),
            position=NS(x=0., y=0., z=-self.z, vx=0., vy=0., vz=0., heading=0., timestamp=1000000))
        self.update()
        self.vehicle = NS(_pump=self.pump, wait=self.wait, wait_ready=lambda: None, takeoff=self.takeoff,
                          land=self.land, target=(1, 1), _command=self.mode_command,
                          _send_command=lambda command, params: self.events.append("nav:"+str(command)),
                          link=NS(mav=NS(set_position_target_local_ned_send=lambda *args: self.sent_references.append(args))))
        self.arm = NS(set_velocity=self.set_velocity, stop=lambda: self.dq.fill(0.))
        self.ctx = NS(node=self.node, vehicle=self.vehicle, arm=self.arm,
            model={"model_sha256": "a"*64, "model": "amsim_urdf_x650_2link_v2_flight", "joint_names": list(runner.JOINTS)},
            observe=lambda **kw: NS(vehicle=NS(armed=self.node.data["status"].arming_state == 2,
                                               landed=self.node.data["land"].landed)))

    def update(self):
        truth = ros_message(self.sim, yaw=0.)
        truth.pose.pose.position = NS(x=0., y=0., z=self.z)
        truth.twist.twist.linear = NS(x=0., y=0., z=0.)
        truth.twist.twist.angular = NS(x=0., y=0., z=0.)
        self.node.data.update(truth=truth, joints=NS(header=truth.header))
        self.node.data["position"].timestamp = int(round(self.sim*1e6))
        self.node.data["status"].timestamp = int(round(self.sim*1e6))
        self.node.received.update({k: time.monotonic() for k in ("truth", "joints", "position", "status")})
        if hasattr(self, "vehicle"):
            self.vehicle.global_position = NS(alt=self.z)
            self.vehicle.global_time = time.monotonic()
        # Match PX4 LandDetector: report changes or one-second PX4-clock tick,
        # rather than silently treating land as a 250 Hz stream in this mock.
        if self.sim-self.last_land_publish_sim >= 1. or self.node.data["land"].landed != self.last_land_value:
            self.node.data["land"].timestamp = int(round(self.sim*1e6))
            self.node.received["land"] = time.monotonic()
            self.last_land_publish_sim, self.last_land_value = self.sim, self.node.data["land"].landed

    def set_velocity(self, joint, velocity):
        self.dq[runner.JOINTS.index(joint)] = velocity

    def pump(self):
        self.sim += .004
        self.q += .004*self.dq
        self.update()
        if self.native_writer is not None:
            row = {k: 0. for k in FIELDS}
            row.update(sim_s=self.sim, step_dt_s=.004, valid_base=1, valid_joint_state=1, valid_tip=1,
                       base_qw=1., base_pz=self.z, arm_joint_1_q=self.q[0], arm_joint_2_q=self.q[1],
                       arm_joint_1_qd=self.dq[0], arm_joint_2_qd=self.dq[1])
            self.native_writer.writerow(row)
            self.native_stream.flush()
        if self.capture is not None:
            self.capture.receive(self.node.data["truth"])

    def wait(self, predicate, timeout_s, description):
        for _ in range(100000):
            self.vehicle._pump()
            if predicate():
                return
        raise RuntimeError("Unit fixture did not converge: "+description)

    def takeoff(self, height, stable_s):
        self.events.append("takeoff")
        self.takeoff_time = self.sim
        self.node.data["status"].arming_state = 2
        self.vehicle._send_command(22, [])
        if self.fail_takeoff:
            raise RuntimeError("unit takeoff failure")
        self.node.data["land"].landed = False
        self.z += height
        begin = self.sim
        self.wait(lambda: self.sim-begin >= stable_s, 10., "mock takeoff")

    def land(self):
        self.events.append("land")
        self.vehicle._send_command(21, [])
        self.z = .37
        self.node.data["land"].landed = True
        self.node.data["status"].arming_state = 1
        self.node.data["status"].nav_state = 18
        for _ in range(20):
            self.vehicle._pump()

    def mode_command(self, command, parameters):
        if command == 511:
            return
        if len(self.sent_references) < 10:
            raise RuntimeError("Offboard requested without actual reference prestream")
        self.node.data["status"].nav_state = 14

    def context(self):
        world = self
        class Context:
            def __enter__(self):
                return world.ctx
            def __exit__(self, *args):
                if world.native_stream is not None:
                    world.native_stream.close()
        return Context()


class MockParameters:
    def __init__(self, folder, *args):
        self.folder, self.snapshots, self.reports = Path(folder), {}, {}
    def capture(self, vehicle, label, *, enforce_contract):
        records = getattr(vehicle, "fixture_parameter_records", None)
        self.snapshots[label] = {"parameters": ({name: value.copy() for name, value in records.items()} if records is not None else
            {"MPC_XY_P": expected_record(1.5, 9), "MPC_Z_P": expected_record(1., 9)})}
        path = self.folder/("px4_parameters_"+label+".json")
        path.write_text(json.dumps(self.snapshots[label]))
        self.reports[label] = {"contract_matches": True, "parameters_unchanged": True}


class LifecycleTests(unittest.TestCase):
    def run_mock(self, folder, *, stage="hover_folded", bad_ground=False, fail_takeoff=False, fail_landing_marker=False,
                 diagnostic_profile=False, fail_diagnostic_restore=False, fail_ev_fusion=False,
                 fixed_reference=False, pid_candidate=None, position_steps=False, research=False):
        if research:
            fixed_reference, pid_candidate = True, PID_CANDIDATES["velocity_high"]
        model = example_model(elbow_offset_body_m=(0., .04, 0.))
        world = MockWorld(model, fail_takeoff)
        cfg = build_task_config(stage)
        if diagnostic_profile:
            cfg["diagnostic_profile"] = dict(DIAGNOSTIC_PROFILE)
        cfg["pid_tuning_candidate"] = pid_candidate
        if research:
            cfg["diagnostic_profile"], cfg["pid_tuning_candidate"] = None, None
        if position_steps:
            cfg["precision_test_profile"] = "hover20_steps36_v1"
        provenance = {"unit_fixture": True}
        if fixed_reference or pid_candidate is not None or position_steps:
            world_path = folder/"unit_world.sdf"
            world_path.write_text('<sdf version="1.9"><world name="unitworld"><spherical_coordinates>'
                '<latitude_deg>0</latitude_deg><longitude_deg>0</longitude_deg><elevation>0</elevation>'
                '<heading_deg>0</heading_deg></spherical_coordinates></world></sdf>', encoding="utf-8")
            cfg["hover_reference_profile"] = {"name": "fixed_world_position_v1", "world_sdf": str(world_path)}
            provenance["hover_world_sdf_sha256"] = hashlib.sha256(world_path.read_bytes()).hexdigest()
            for name, value in dict(ref_lat=0., ref_lon=0., ref_alt=0., ref_timestamp=1000000,
                    xy_valid=True, z_valid=True, xy_global=True, z_global=True,
                    xy_reset_counter=0, z_reset_counter=0, heading_reset_counter=0).items():
                setattr(world.node.data["position"], name, value)
        if pid_candidate is not None:
            world.vehicle.fixture_parameter_records = {
                name: expected_record(1 if name in ("MC_AIRMODE", "MPC_USE_HTE") else 1.,
                                      6 if name in ("MC_AIRMODE", "MPC_USE_HTE") else 9)
                for name in parameter_evidence_module().CORE_CONTROL_PARAMETERS}
            world.vehicle.fixture_parameter_records.update({name: expected_record(value, 9)
                for name, value in PID_CANDIDATES["baseline"].items()})
            world.initial_parameter_records = {name: record.copy() for name, record in world.vehicle.fixture_parameter_records.items()}
            world.parameter_writes = []
        if research:
            from research_condition import condition_for_task, canonical_hash
            selected = {name: record["value"] for name, record in world.initial_parameter_records.items()}
            selected.update({name: expected_record(value, 9)["value"] for name, value in pid_candidate.items()})
            profile = folder/"selected_pid.json"
            profile.write_text(json.dumps({"model_sha256": "a"*64, "parameters": selected}))
            cfg["pid_profile"] = str(profile)
            cfg["research_profile"] = {"name": "precision_ev_selected_pid_v1", "world_sdf": str(world_path)}
            condition = condition_for_task(cfg)
            provenance.update(operating_condition=condition, operating_condition_sha256=canonical_hash(condition))
        cfg["preparation_report"] = str(folder/"preparation.json")
        Path(cfg["preparation_report"]).write_text(json.dumps({"derived_urdf_sha256": "b"*64}))
        cfg["native_recorder_library"] = str(folder/"observer.so")
        Path(cfg["native_recorder_library"]).write_bytes(b"mock library, never executed")
        artifact = folder/"unit_gate_artifact.csv"
        artifact.write_text("synthetic unit-only pre-existing gate fixture")
        for gate in required_gates(cfg):
            path = folder/("unit_prior_"+gate+".json")
            path.write_text(json.dumps(dict(kind="x650_native_stage_evidence", stage=gate,
                complete=True, passed=True, simulation_backend="gazebo_px4", model_sha256="a"*64,
                source_urdf_sha256="b"*64, artifacts={artifact.name: hashlib.sha256(artifact.read_bytes()).hexdigest()})))
            cfg["stage_evidence"][gate] = str(path)
        spec = NS(sha256="a"*64, model_name=cfg["model"], px4_sys_autostart=4998)
        def attach(capture, ctx):
            capture.context, world.capture = ctx, capture
            capture.receive(ctx.node.data["truth"])
        def load(native, pump):
            world.native_stream = native.live.open("w", newline="", encoding="utf-8")
            world.native_writer = csv.DictWriter(world.native_stream, fieldnames=FIELDS)
            world.native_writer.writeheader()
            for _ in range(3):
                pump()
        stubs = {"amsim": NS(ScenarioContext=world.context), "amsim.modeling": NS(),
                 "amsim.modeling.manager": NS(validate_px4_airframe=lambda m: folder/"airframe"),
                 "pymavlink": NS(mavutil=NS(mavlink=NS(MAV_FRAME_LOCAL_NED=1, MAV_CMD_DO_SET_MODE=176,
                                                      MAV_CMD_NAV_TAKEOFF=22, MAV_CMD_NAV_LAND=21,
                                                      MAV_CMD_SET_MESSAGE_INTERVAL=511, MAVLINK_MSG_ID_GLOBAL_POSITION_INT=33)))}
        with ExitStack() as stack:
            stack.enter_context(patch.dict(sys.modules, stubs))
            stack.enter_context(patch.object(runner.ArmModel, "from_report", return_value=model))
            stack.enter_context(patch.object(runner, "load_parameter_evidence", return_value=MockParameters))
            stack.enter_context(patch.object(runner, "active_runtime_instance", return_value="unit_x650_0"))
            stack.enter_context(patch.object(FlightRecorder, "attach", attach))
            stack.enter_context(patch.object(NativeRecorder, "load", load))
            if cfg["hover_reference_profile"] is not None:
                stack.enter_context(patch("diagnostic_odometry.active_world", return_value=("unitworld", {})))
            if diagnostic_profile or pid_candidate is not None:
                values = {name: expected_record(value, kind) for name, value, kind in (
                    ("EKF2_EV_CTRL", 0, 6), ("EKF2_EV_NOISE_MD", 0, 6), ("EKF2_EVP_NOISE", .1, 9),
                    ("EKF2_EVV_NOISE", .1, 9), ("EKF2_BARO_CTRL", 1, 6),
                    ("EKF2_GPS_CTRL", 7, 6), ("EKF2_HGT_REF", 1, 6), ("MPC_XY_P", 1.5, 9), ("MPC_Z_P", 1., 9))}
                if pid_candidate is not None:
                    values.update(world.vehicle.fixture_parameter_records)
                if research:
                    from diagnostic_parameters import PRECISION_REQUIRED
                    values.update({name: expected_record(value, 9) for name, value in PRECISION_REQUIRED.items()})
                def parameter_send(sysid, compid, name, wire, kind):
                    import struct
                    name = name.decode()
                    if fail_diagnostic_restore and name == "EKF2_EV_CTRL" and wire == 0.:
                        return
                    raw = struct.pack("<f", wire)
                    values[name] = dict(value=struct.unpack("<i" if kind == 6 else "<f", raw)[0], param_type=kind, raw_value_hex=raw.hex())
                    if pid_candidate is not None:
                        world.parameter_writes.append({"name": name, "value": values[name]["value"], "sim_s": world.sim})
                        if name in world.vehicle.fixture_parameter_records:
                            world.vehicle.fixture_parameter_records[name] = values[name].copy()
                world.vehicle.link.mav.param_set_send = parameter_send
                world.vehicle.ack = {}
                original_send = world.vehicle._send_command
                def command_send(command, params):
                    if command == 245:
                        raise AssertionError("Early storage ACK cannot certify saved parameters")
                    else:
                        original_send(command, params)
                world.vehicle._send_command = command_send
                def make_session(ctx, profile, out, *, protected_pid_records, pid_candidate=None, **kwargs):
                    return DiagnosticParameterSession(ctx, profile, out, protected_pid_records=protected_pid_records,
                        pid_candidate=pid_candidate, **kwargs,
                        reader=lambda vehicle, names: ({name: values[name].copy() for name in names}, {}, {}),
                        journal_path=folder/"runtime/restore_pending.json",
                        storage_saver=lambda session: {"returncode": 0, "synchronous_default_save": True, "test_fixture": True})
                class Observer:
                    def __init__(self, *args, **kwargs):
                        self.document = {"aiding_confirmed": False, "diagnostic_only": True, "runtime_scoped_observer": True}
                    def load(self, pump):
                        self.document["input_stream_confirmed"] = True
                    def confirm_horizontal_fusion(self, pump, *, required_flags=None):
                        if fail_ev_fusion:
                            raise RuntimeError("unit EV fusion unconfirmed")
                        self.document["aiding_confirmed"] = True
                        return self.document
                stack.enter_context(patch.object(runner, "DiagnosticParameterSession", make_session))
                if diagnostic_profile or research:
                    stack.enter_context(patch.object(runner, "load_diagnostic_odometry", return_value=Observer))
            if fail_landing_marker:
                original_marker, calls = runner.transition_marker, [0]
                def marker(node):
                    calls[0] += 1
                    if calls[0] == 2:
                        raise RuntimeError("unit landing marker unavailable")
                    return original_marker(node)
                stack.enter_context(patch.object(runner, "transition_marker", marker))
            if bad_ground:
                stack.enter_context(patch.object(runner, "native_ground_metrics", return_value={"passed": False, "metrics": {}, "criteria": {"unit_ground_failure": False}}))
            try:
                result = runner.execute(cfg, spec, provenance, folder)
            except BaseException as error:
                return world, error
        return world, result

    def test_hover_takes_off_only_after_ground_gate_and_lands_cleanly(self):
        with tempfile.TemporaryDirectory() as tmp:
            folder = Path(tmp)
            world, result = self.run_mock(folder)
            if isinstance(result, BaseException):
                raise result
            self.assertTrue(result["complete"])
            self.assertEqual(world.events, ["takeoff", "nav:22", "land", "nav:21"])
            ground = json.loads((folder/"native_ground.json").read_text())
            self.assertTrue(ground["passed"])
            self.assertGreater(world.takeoff_time, ground["interval_sim_s"][1])
            stage = json.loads((folder/"hover_folded.json").read_text())
            self.assertTrue(stage["passed"])
            self.assertTrue(stage["landed_and_disarmed"])
            self.assertEqual(stage["kind"], "x650_native_stage_evidence")
            self.assertIn("native_trace.csv", stage["artifacts"])
            self.assertGreater(result["hover_metrics"]["duration_s"], 19.9)

    def test_research_applies_frozen_pid_before_verification_but_needs_postflight_audit(self):
        with tempfile.TemporaryDirectory() as tmp:
            folder = Path(tmp)
            world, result = self.run_mock(folder, research=True)
            if isinstance(result, BaseException):
                raise result
            self.assertTrue(result["complete"])
            self.assertAlmostEqual(result["actual_pid_parameters"]["MPC_XY_VEL_P_ACC"], 2.2, places=6)
            self.assertTrue(result["diagnostic_parameters_restored"])
            self.assertEqual(world.vehicle.fixture_parameter_records, world.initial_parameter_records)
            self.assertEqual(result["pid_profile"]["matched_parameter_count"], 42)
            for name in ("stage_result.json", "native_ground.json", "controller_metadata.json"):
                item = json.loads((folder/name).read_text())
                self.assertFalse(item["eligible_as_stage_gate"])
                self.assertFalse(item["diagnostic_only"])
                self.assertTrue(item["ground_truth_aided"])
                self.assertEqual(item["acceptance_pending"], "full_ulog_audit")
                self.assertEqual(item["operating_condition_sha256"], result["operating_condition_sha256"])

    def test_research_takeoff_failure_preserves_failure_and_restores_parameters(self):
        with tempfile.TemporaryDirectory() as tmp:
            folder = Path(tmp)
            world, error = self.run_mock(folder, research=True, fail_takeoff=True)
            self.assertIsInstance(error, RuntimeError)
            stage = json.loads((folder/"stage_result.json").read_text())
            self.assertFalse(stage["passed"])
            self.assertFalse(stage["eligible_as_stage_gate"])
            self.assertTrue(stage["diagnostic_parameters_restored"])
            self.assertEqual(world.vehicle.fixture_parameter_records, world.initial_parameter_records)

    def test_failed_ground_gate_never_arms_or_takes_off(self):
        with tempfile.TemporaryDirectory() as tmp:
            folder = Path(tmp)
            world, error = self.run_mock(folder, bad_ground=True)
            self.assertIsInstance(error, RuntimeError)
            self.assertIn("takeoff refused", str(error))
            self.assertEqual(world.events, [])
            self.assertFalse(json.loads((folder/"hover_folded.json").read_text())["passed"])

    def test_takeoff_failure_preserves_error_and_still_attempts_landing(self):
        with tempfile.TemporaryDirectory() as tmp:
            folder = Path(tmp)
            world, error = self.run_mock(folder, fail_takeoff=True)
            self.assertIsInstance(error, RuntimeError)
            self.assertIn("unit takeoff failure", str(error))
            self.assertEqual(world.events, ["takeoff", "nav:22", "land", "nav:21"])
            meta = json.loads((folder/"controller_metadata.json").read_text())
            self.assertFalse(meta["complete"])
            self.assertTrue(meta["landed_and_disarmed"])
            self.assertTrue(meta["folded_before_landing"])

    def test_joint_probe_requires_native_tracking_and_returns_folded(self):
        with tempfile.TemporaryDirectory() as tmp:
            folder = Path(tmp)
            world, result = self.run_mock(folder, stage="joint_probe")
            if isinstance(result, BaseException):
                raise result
            self.assertTrue(result["joint_probe_metrics"]["passed"])
            self.assertTrue(result["folded_before_landing"])
            self.assertLess(np.max(abs(world.q)), .006)
            self.assertEqual(world.events, ["takeoff", "nav:22", "land", "nav:21"])
            self.assertTrue(json.loads((folder/"joint_probe.json").read_text())["passed"])

    def test_bad_landing_marker_does_not_suppress_actual_land_command(self):
        with tempfile.TemporaryDirectory() as tmp:
            folder = Path(tmp)
            world, error = self.run_mock(folder, fail_landing_marker=True)
            self.assertIsInstance(error, RuntimeError)
            self.assertIn("unit landing marker unavailable", str(error))
            self.assertEqual(world.events, ["takeoff", "nav:22", "land", "nav:21"])
            meta = json.loads((folder/"controller_metadata.json").read_text())
            self.assertFalse(meta["complete"])
            self.assertTrue(meta["landing_command_attempted"])
            self.assertTrue(meta["landing_api_returned"])
            self.assertIn("landing_marker_error", meta)

    def test_response_probe_prestreams_before_offboard_and_does_not_claim_identification(self):
        with tempfile.TemporaryDirectory() as tmp:
            folder = Path(tmp)
            world, result = self.run_mock(folder, stage="response_probe")
            if isinstance(result, BaseException):
                raise result
            self.assertFalse(result["response_identification_claimed"])
            self.assertFalse(result["allocator_active"])
            self.assertGreater(len(world.sent_references), 100)
            self.assertTrue(json.loads((folder/"response_probe.json").read_text())["passed"])

    def test_diagnostic_hover_restores_parameters_and_cannot_publish_stage_gates(self):
        with tempfile.TemporaryDirectory() as tmp:
            folder = Path(tmp)
            world, result = self.run_mock(folder, diagnostic_profile=True)
            if isinstance(result, BaseException):
                raise result
            self.assertTrue(result["complete"])
            self.assertTrue(result["diagnostic_only"])
            self.assertTrue(result["aiding_confirmed"])
            self.assertTrue(result["diagnostic_parameters_restored"])
            self.assertFalse((folder/"hover_folded.json").exists())
            self.assertFalse((folder/"runtime/restore_pending.json").exists())
            self.assertEqual(json.loads((folder/"native_ground.json").read_text())["kind"], "x650_diagnostic_ground_observation")
            output = json.loads((folder/"stage_result.json").read_text())
            self.assertEqual(output["kind"], "x650_diagnostic_run_result")
            self.assertFalse(output["eligible_as_stage_gate"])

    def test_diagnostic_restore_failure_is_incomplete_and_keeps_interlock(self):
        with tempfile.TemporaryDirectory() as tmp:
            folder = Path(tmp)
            world, error = self.run_mock(folder, diagnostic_profile=True, fail_diagnostic_restore=True)
            self.assertIsInstance(error, RuntimeError)
            meta = json.loads((folder/"controller_metadata.json").read_text())
            self.assertFalse(meta["complete"])
            self.assertTrue(meta["landed_and_disarmed"])
            self.assertTrue(meta["diagnostic_restore_pending"])
            self.assertTrue((folder/"runtime/restore_pending.json").exists())

    def test_diagnostic_fusion_failure_never_takes_off_and_restores_on_ground(self):
        with tempfile.TemporaryDirectory() as tmp:
            folder = Path(tmp)
            world, error = self.run_mock(folder, diagnostic_profile=True, fail_ev_fusion=True)
            self.assertIsInstance(error, RuntimeError)
            self.assertEqual(world.events, [])
            meta = json.loads((folder/"controller_metadata.json").read_text())
            self.assertFalse(meta["complete"])
            self.assertFalse(meta["aiding_confirmed"])
            self.assertTrue(meta["diagnostic_parameters_restored"])
            self.assertFalse((folder/"runtime/restore_pending.json").exists())

    def test_ground_control_initialization_failure_restores_via_original_transport(self):
        with tempfile.TemporaryDirectory() as tmp:
            folder = Path(tmp)
            with patch.object(runner.ControlLoop, "pump", side_effect=RuntimeError("unit state provider not installed")):
                world, error = self.run_mock(folder, diagnostic_profile=True)
            self.assertIsInstance(error, RuntimeError)
            self.assertIn("state provider not installed", str(error))
            self.assertEqual(world.events, [])
            meta = json.loads((folder/"controller_metadata.json").read_text())
            self.assertFalse(meta["complete"])
            self.assertTrue(meta["diagnostic_parameters_restored"])
            self.assertFalse(meta["diagnostic_restore_pending"])
            self.assertFalse((folder/"runtime/restore_pending.json").exists())

    def test_pid_only_fixed_reference_uses_native_position_loop_and_restores_gains(self):
        with tempfile.TemporaryDirectory() as tmp:
            folder = Path(tmp)
            with patch.object(runner, "load_diagnostic_odometry", side_effect=AssertionError("PID-only must not create EV aiding")):
                world, result = self.run_mock(folder, pid_candidate="velocity_high")
            if isinstance(result, BaseException):
                raise result
            self.assertTrue(result["complete"])
            self.assertTrue(result["diagnostic_only"])
            self.assertFalse(result["ground_truth_aided"])
            self.assertFalse(result["aiding_requested"])
            self.assertFalse(result["aiding_confirmed"])
            self.assertTrue(result["diagnostic_parameters_restored"])
            self.assertFalse((folder/"hover_folded.json").exists())
            self.assertFalse((folder/"runtime/restore_pending.json").exists())
            self.assertEqual(world.vehicle.fixture_parameter_records, world.initial_parameter_records)
            self.assertEqual(set(write["name"] for write in world.parameter_writes), set(PID_CANDIDATES["velocity_high"]))
            for name, expected in PID_CANDIDATES["velocity_high"].items():
                self.assertAlmostEqual(result["actual_pid_parameters"][name], expected, delta=1e-6)
            self.assertGreater(len(world.sent_references), 100)
            positions = []
            for packet in world.sent_references:
                self.assertEqual(packet[3:5], (1, 2552))
                positions.append(packet[5:8])
            self.assertTrue(all(np.allclose(p, positions[0], atol=1e-9) for p in positions))
            self.assertTrue(np.allclose(positions[0], [0., 0., -2.37], atol=1e-7))
            stage = json.loads((folder/"stage_result.json").read_text())
            self.assertFalse(stage["eligible_as_stage_gate"])
            self.assertFalse(stage["ground_truth_aided"])

    def test_pid_only_steps_send_six_frozen_targets_after_static_hover(self):
        with tempfile.TemporaryDirectory() as tmp:
            folder = Path(tmp)
            with patch.object(runner, "load_diagnostic_odometry", side_effect=AssertionError("PID-only must not create EV aiding")):
                world, result = self.run_mock(folder, pid_candidate="velocity_high", position_steps=True)
            if isinstance(result, BaseException):
                raise result
            self.assertTrue(result["complete"])
            self.assertFalse(result["ground_truth_aided"])
            self.assertEqual(world.vehicle.fixture_parameter_records, world.initial_parameter_records)
            protocol = json.loads((folder/"position_step_probe.json").read_text())
            anchor = np.asarray(result["fixed_hover_reference"]["target_world_enu_m"])
            self.assertEqual(len(protocol["segments"]), 6)
            for segment in protocol["segments"]:
                self.assertTrue(np.allclose(segment["reference"]["target_world_enu_m"],
                    anchor+segment["target_offset_world_enu_m"], atol=1e-9))
                self.assertEqual(segment["reference"]["px4_reference_at_bind"],
                                 result["fixed_hover_reference"]["px4_reference_at_bind"])
            start = result["task_start_sim_s"]
            events = [event for event in result["position_step_send_events"] if event["segment_index"] is not None]
            self.assertEqual([event["segment_index"] for event in events], list(range(6)))
            for event, segment in zip(events, protocol["segments"]):
                elapsed = event["first_send_sim_s"]-start
                self.assertGreaterEqual(elapsed+1e-8, segment["start_s"])
                self.assertLess(elapsed-segment["start_s"], .05)
                self.assertTrue(np.allclose(event["position_target_local_ned_m"],
                    segment["reference"]["target_local_ned_m"], atol=1e-9))
                self.assertTrue(any(np.allclose(packet[5:8], event["position_target_local_ned_m"], atol=1e-9)
                    for packet in world.sent_references))
            self.assertGreaterEqual(result["experiment_end_sim_s"]-start, 56.)
            self.assertLess(result["task_end_sim_s"]-start, 20.05)
            self.assertGreater(result["hover_metrics"]["duration_s"], 19.9)
            self.assertLess(result["hover_metrics"]["duration_s"], 20.05)
            self.assertTrue(all(packet[3:5] == (1, 2552) for packet in world.sent_references))
            final_reference = result["fixed_hover_reference"]["target_local_ned_m"]
            self.assertTrue(np.allclose(world.sent_references[-1][5:8], final_reference, atol=1e-9))


if __name__ == "__main__":
    unittest.main()
