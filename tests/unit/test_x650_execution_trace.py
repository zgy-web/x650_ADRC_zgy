"""Failure snapshots and local publication boundaries, without ROS/Gazebo."""
import hashlib
import json
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace as NS
import unittest
from unittest.mock import patch

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
for directory in (ROOT/"simulations/x650_wholebody_mpc_v1", ROOT/"simulations/x650_wholebody_mpc_v1/tests",
                  ROOT/"control_packages/x650_wholebody_v1", ROOT/"scripts"):
    sys.path.insert(0, str(directory))
from execution_trace import ExecutionTrace, callback_clock_snapshot, evidence_value
from x650_wholebody.api import ControlState, EndpointTarget
from x650_wholebody.mpc import MpcSolveError
import test_upper_loop as loop_fixture
from test_x650_estimated_feedback import advance
import run_case


def strict_read(path):
    def reject(value):
        raise AssertionError("Nonstandard JSON constant: "+value)
    return json.loads(Path(path).read_text(), parse_constant=reject)


class ExecutionTraceTests(unittest.TestCase):
    def setUp(self):
        temp = tempfile.TemporaryDirectory()
        self.addCleanup(temp.cleanup)
        self.folder = Path(temp.name)
        self.trace = ExecutionTrace(self.folder)
        self.addCleanup(self.trace.close)
        self.node = NS(estimator_clock_s=2., data={"joints": NS(header=NS(stamp=NS(sec=1, nanosec=980000000)))},
                       received={"joints": 30.}, received_sim={"joints": 2.}, callback_sequences={"joints": 4})
        self.clock = lambda: callback_clock_snapshot(self.node)

    def test_publication_retains_held_clock_header_and_never_claims_execution_ack(self):
        actual = []
        with patch("execution_trace.time.monotonic", side_effect=[30.1, 30.2]):
            self.trace.publish("joint:a", {"velocity_rad_s": .1}, lambda: actual.append(.1),
                               clock_snapshot=self.clock, context={"t_s": .2})
        saved = self.trace.last_sent["joint:a"]
        self.assertEqual(actual, [.1])
        self.assertEqual(saved["publish_before"]["wall_monotonic_s"], 30.1)
        self.assertEqual(saved["publish_after"]["wall_monotonic_s"], 30.2)
        self.assertEqual(saved["publish_before"]["processed_estimator_clock_s"], 2.)
        self.assertEqual(saved["publish_after"]["processed_estimator_clock_s"], 2.)
        self.assertAlmostEqual(saved["publish_after"]["joint_header_stamp_s"], 1.98)
        self.assertEqual(saved["publish_after"]["joint_callback_sequence"], 4)
        self.assertTrue(saved["local_call_returned"])
        self.assertFalse(saved["downstream_application_confirmed"])

    def test_failed_second_joint_does_not_rewrite_last_successfully_sent_pair(self):
        self.trace.publish("joint:a", {"velocity_rad_s": .1}, lambda: None, clock_snapshot=self.clock, context={})
        error = RuntimeError("second publisher failed")
        def fail():
            raise error
        with self.assertRaises(RuntimeError) as caught:
            self.trace.publish("joint:b", {"velocity_rad_s": .2}, fail, clock_snapshot=self.clock, context={})
        self.assertIs(caught.exception, error)
        self.assertEqual(set(self.trace.last_sent), {"joint:a"})
        saved = strict_read(self.folder/"control_failure_snapshot.json")
        self.assertEqual(saved["event"], "publication_exception")
        self.assertEqual(set(saved["previous_publications"]), {"joint:a"})
        self.assertFalse(saved["local_call_returned"])

    def test_failure_keeps_pre_call_inputs_and_nonfinite_original_diagnostics(self):
        state = NS(position=np.array([1., 2., 3.]), rotation=np.eye(3), velocity=np.zeros(3),
                   omega_body=np.zeros(3), q=np.zeros(2), dq=np.zeros(2), stamp_s=2., acceleration_enu=np.ones(3))
        target = EndpointTarget([2., 3., 4.])
        feedback = {"raw": {"timestamp": 1234}, "valid": True}
        error = MpcSolveError("infeasible", {"kkt": float("inf"), "array": np.array([np.nan, -np.inf])})
        def fail(*args):
            state.position[0] = 99.
            feedback["raw"]["timestamp"] = 9999
            raise error
        with self.assertRaises(MpcSolveError) as caught:
            self.trace.controller_step(NS(step=fail), .1, .02, state, target, feedback=feedback,
                                       phase="measure", clock_snapshot=self.clock)
        self.assertIs(caught.exception, error)
        saved = strict_read(self.folder/"control_failure_snapshot.json")
        self.assertEqual(saved["state"]["position"], [1., 2., 3.])
        self.assertEqual(saved["feedback_snapshot"]["raw"]["timestamp"], 1234)
        self.assertEqual(saved["target"]["position"], [2., 3., 4.])
        self.assertEqual(saved["exception"]["diagnostics"]["kkt"], {"nonfinite": "+inf"})
        self.assertEqual(saved["exception"]["diagnostics"]["array"], [{"nonfinite": "nan"}, {"nonfinite": "-inf"}])
        self.assertFalse(saved["command_published"])

    def test_snapshot_io_failure_cannot_hide_original_control_exception(self):
        error = RuntimeError("original controller exception")
        def fail(*args):
            raise error
        with patch.object(self.trace, "_append", side_effect=OSError("disk unavailable")), self.assertRaises(RuntimeError) as caught:
            self.trace.controller_step(NS(step=fail), 1., .02, NS(), NS(), feedback={}, phase="measure", clock_snapshot=self.clock)
        self.assertIs(caught.exception, error)
        self.assertEqual(self.trace.first_failure["exception"]["message"], str(error))
        self.assertIn("disk unavailable", self.trace.logging_error["repr"])

    def test_closed_manifest_is_hash_bound_and_not_a_stage_certificate(self):
        self.trace.publish("joint:a", {"velocity_rad_s": 0.}, lambda: None, clock_snapshot=self.clock, context={})
        manifest = self.trace.close(experiment_complete=False)
        self.assertEqual(manifest["sha256"], hashlib.sha256(self.trace.path.read_bytes()).hexdigest())
        self.assertTrue(manifest["recording_closed"])
        self.assertFalse(manifest["experiment_complete"])
        self.assertFalse(manifest["eligible_as_stage_gate"])
        self.assertEqual(self.trace.close(), manifest)
        from audit_x650_precision_stage import verified_artifacts
        names = ("control_execution_trace.jsonl", "control_execution_trace_manifest.json")
        files = {name: hashlib.sha256((self.folder/name).read_bytes()).hexdigest() for name in names}
        checked = verified_artifacts(self.folder, {"artifacts": files})
        self.assertEqual(len(checked), 2)
        with (self.folder/names[0]).open("a") as stream:
            stream.write("changed\n")
        with self.assertRaises(ValueError):
            verified_artifacts(self.folder, {"artifacts": files})


class LoopFailureSnapshotTests(unittest.TestCase):
    setUp = loop_fixture.UpperLoopTests.setUp

    def test_controller_exception_has_failed_iteration_and_previous_actual_publications(self):
        joint_calls = []
        self.ctx.arm.set_velocity = lambda *args: joint_calls.append(args)
        module = NS(mavutil=NS(mavlink=NS(MAV_FRAME_LOCAL_NED=1)))
        with patch.dict(sys.modules, {"pymavlink": module}):
            self.loop.pump()
            advance(self.node, 10.02)
            self.node.received = {key: 100. for key in self.node.received}
            self.capture.now = lambda: 10.02
            self.loop.controller = NS(step=lambda *args: (_ for _ in ()).throw(error))
            self.loop.trajectory = run_case.ConstantTarget([2., 3., 1.])
            error = MpcSolveError("quality failure", {"primal_violation": np.inf})
            with self.assertRaises(MpcSolveError) as caught:
                self.loop.pump()
        self.assertIs(caught.exception, error)
        self.assertEqual(len(joint_calls), 2)
        self.assertEqual(len(self.sent), 1)
        saved = strict_read(self.path/"control_failure_snapshot.json")
        self.assertAlmostEqual(saved["dt_s"], .02)
        self.assertEqual(saved["state"]["position"], [2., 3., 1.5])
        self.assertEqual(saved["feedback_snapshot"]["px4_snapshot"]["position_ned_m"], [3., 2., 486.5])
        self.assertEqual(set(saved["previous_publications"]), {"joint:arm_joint_1", "joint:arm_joint_2", "px4_pva"})
        self.assertEqual(saved["exception"]["diagnostics"]["primal_violation"], {"nonfinite": "+inf"})
        self.assertFalse(saved["command_published"])
        manifest = self.loop.close(experiment_complete=False)
        self.assertEqual(manifest["failure_count"], 1)
        self.assertEqual(manifest["first_failure_snapshot"]["sha256"],
                         hashlib.sha256((self.path/"control_failure_snapshot.json").read_bytes()).hexdigest())


if __name__ == "__main__":
    unittest.main()
