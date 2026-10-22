"""Actual ControlLoop publication boundaries with synthetic ROS callbacks."""
import csv
from pathlib import Path
import sys
from types import SimpleNamespace as NS
import unittest
from unittest.mock import Mock, patch

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
for path in (ROOT/"simulations/x650_wholebody_mpc_v1/tests", ROOT/"simulations/x650_wholebody_mpc_v1",
             ROOT/"tests/unit", ROOT/"control_packages/x650_wholebody_v1"):
    sys.path.insert(0, str(path))
import test_upper_loop as fixtures
import run_case
from test_x650_estimated_feedback import advance
from x650_wholebody.api import ControlCommand


class ActivationHandoffTests(unittest.TestCase):
    setUp = fixtures.UpperLoopTests.setUp

    def prepare(self, controller):
        clock = patch.object(run_case.time, "monotonic", return_value=100.1)
        clock.start()
        self.addCleanup(clock.stop)
        self.node.received.update(land=100.05, status=100.05)
        self.cfg.update(stage="tip_hold", controller=controller, duration_s=2.)
        self.loop.task_start = None
        self.loop.strict = True
        self.loop.model = NS(joint_lower=[-2., -2.], joint_upper=[2., 2.])
        self.capture.phase = "settle"
        self.loop.trajectory = run_case.ConstantTarget([2., 3., 1.5])
        self.allocator = NS(previous_command=np.zeros(5))
        def prime(history, start):
            self.assertIsNone(self.loop.task_start)
            self.assertEqual(self.capture.phase, "settle")
            self.allocator.previous_command = history[-1][1].copy()
        self.allocator.prime_command_history = Mock(side_effect=prime)
        def step(t, dt, measured, target):
            self.first_step = (t, dt)
            u = self.allocator.previous_command
            return ControlCommand.from_executed_velocity(measured, u[:3], u[3:], nominal_position=self.loop.anchor,
                acceleration_feedforward=np.zeros(3), position_gains=[.95, .95, 1.])
        self.loop.controller = NS(allocator=self.allocator, step=step)
        self.module = NS(mavutil=NS(mavlink=NS(MAV_FRAME_LOCAL_NED=1)))
        self.module_patch = patch.dict(sys.modules, {"pymavlink": self.module})
        self.module_patch.start()
        self.addCleanup(self.module_patch.stop)
        self.loop.pump()
        self.assertEqual(len(self.sent), 1)
        self.events = []
        self.meta = {"marker": "large preparation metadata"}

    def pump_at(self, now):
        advance(self.node, now)
        self.node.received = {key: 100.1 for key in self.node.received}
        self.capture.now = lambda: now
        self.loop.pump()

    def wait_schedule(self, times):
        def wait(predicate, timeout, description):
            self.events.append("wait")
            for now in times:
                self.pump_at(now)
                self.events.append(("pumped", now, len(self.sent)))
                if predicate():
                    return
            raise RuntimeError("Timeout: "+description)
        self.ctx.vehicle.wait = wait

    def activate(self):
        with patch.object(run_case, "atomic_json", side_effect=AssertionError("No synchronous handoff metadata write")), \
             patch("builtins.print", side_effect=AssertionError("No synchronous handoff console write")):
            return run_case.activate_prediction_chain(self.ctx, self.loop, self.cfg, self.meta,
                                                     np.eye(3))

    def check_successful_handoff(self, controller):
        self.prepare(controller)
        self.wait_schedule([10.016, 10.024])
        start = self.activate()
        # A /clock advance before next_control was insufficient: activation
        # waited for the following pump that actually published a new batch.
        self.assertEqual(self.events, ["wait",
            ("pumped", 10.016, 1), ("pumped", 10.024, 2)])
        self.assertEqual(start, 10.024)
        self.assertEqual(self.loop.last, 10.024)
        self.assertAlmostEqual(self.loop.next_control, 10.044)
        self.assertEqual(self.capture.phase, "measure")
        self.assertEqual(self.meta["task_start_sim_s"], 10.024)
        self.assertEqual(self.meta["activation_command_history"][-1]["control_time_s"], 10.024)
        self.assertEqual(self.meta["activation_command_history"][-1]["task_time_s"], 0.)
        np.testing.assert_array_equal(self.allocator.previous_command, self.loop.sent_command_history[-1][1])
        self.assertEqual(self.allocator.prime_command_history.call_count, int(controller == "mpc"))
        self.pump_at(10.048)
        self.assertAlmostEqual(self.first_step[0], .024)
        self.assertAlmostEqual(self.first_step[1], .024)
        self.loop.control.flush()
        with (self.path/"control_trace.csv").open() as stream:
            rows = list(csv.DictReader(stream))
        self.assertEqual([r["phase"] for r in rows], ["settle", "settle", "measure"])
        self.assertAlmostEqual(float(rows[-1]["control_dt_s"]), float(rows[-1]["sim_s"])-float(rows[-2]["sim_s"]))
        self.assertAlmostEqual(float(rows[-1]["t_s"]), .024)

    def test_mpc_refreshes_actual_send_then_preserves_true_first_dt_without_io(self):
        self.check_successful_handoff("mpc")

    def test_pid_uses_the_identical_handoff_and_actual_previous_input(self):
        self.check_successful_handoff("pid_ff")

    def test_actual_long_gap_is_not_hidden_or_relaxed_during_refresh(self):
        self.prepare("mpc")
        self.wait_schedule([10.064])
        with self.assertRaisesRegex(RuntimeError, "0.0640s exceeds 0.05s"):
            self.activate()
        self.assertEqual(len(self.sent), 1)
        self.assertIsNone(self.loop.task_start)
        self.assertEqual(self.capture.phase, "settle")
        self.allocator.prime_command_history.assert_not_called()

    def test_callback_only_refresh_cannot_activate_or_invent_a_send(self):
        self.prepare("mpc")
        self.wait_schedule([10.004, 10.008, 10.016])
        with self.assertRaisesRegex(RuntimeError, "Timeout"):
            self.activate()
        self.assertEqual(len(self.sent), 1)
        self.assertIsNone(self.loop.task_start)
        self.allocator.prime_command_history.assert_not_called()


if __name__ == "__main__":
    unittest.main()
