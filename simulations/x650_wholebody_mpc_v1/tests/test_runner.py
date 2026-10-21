"""No-flight checks of references, real artifact gates and parameter policy."""
from __future__ import annotations
import hashlib
import json
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace as NS
import unittest
from unittest.mock import patch
import numpy as np

HERE = Path(__file__).resolve().parents[1]
ROOT = HERE.parents[1]
for p in (HERE, ROOT/"control_packages/x650_wholebody_v1"):
    sys.path.insert(0, str(p))
import run_case as runner
from run_case import (FigureEight, joint_probe_reference, response_probe_velocity,
    formal_gate_evidence, actual_pid_parameters, verify_pid_profile,
    LandReceiptWatchdog, transition_marker, confirm_land_state_after_command, ControlLoop)
from task import build_task_config, required_gates


class RunnerTests(unittest.TestCase):
    def clock_node(self):
        from test_recording import ros_message
        truth = ros_message(1., yaw=0.)
        return NS(data={"truth": truth, "joints": NS(header=truth.header),
                        "position": NS(timestamp=1000000), "land": NS(timestamp=500000, landed=True),
                        "status": NS(timestamp=1000000, arming_state=1, failsafe=False, nav_state=5)},
                  received={"truth": 100., "joints": 100., "position": 100., "status": 100., "land": 90.},
                  joint_values=lambda: {j: NS(position_rad=0., velocity_rad_s=0.) for j in runner.JOINTS})

    def test_land_receipts_ignore_epoch_jump_and_regression_but_fast_topics_stay_fresh(self):
        with tempfile.TemporaryDirectory() as tmp, patch.object(runner.time, "monotonic", return_value=100.):
            node = self.clock_node()
            ctx = NS(node=node, vehicle=NS(_pump=lambda: None))
            capture = NS(now=lambda: 1., check=lambda: None)
            model = NS(joint_lower=[-1., -.5], joint_upper=[1., 1.])
            loop = ControlLoop(ctx, {"control_period_s": .02}, model, capture, {}, Path(tmp))
            try:
                loop.read()  # cached initial receipt cannot establish liveness
                node.received["land"] = 99.
                loop.read()  # observe an actual new callback during non-strict ground
                loop.strict = True
                loop.read()
                self.assertEqual(loop.land_watchdog.sequence, 1)
                node.data["position"].timestamp = 1792554749151522+20536300
                node.data["land"].timestamp = 1792554749151522
                loop.read()  # source HRT + independently changing XRCE offsets
                self.assertAlmostEqual(loop.land_watchdog.diagnostics(node, 1.)["land_epoch_delta_diagnostic_s"], 20.5363)
                node.data["land"].timestamp = 2
                loop.read()  # a DDS epoch regression is diagnostic, not loss of callbacks
                node.received["position"] = 90.
                with self.assertRaisesRegex(RuntimeError, "Stale flight telemetry: position"):
                    loop.read()
            finally:
                loop.close()

    def test_land_watchdog_requires_new_receipt_and_rejects_stall_or_regression(self):
        node = self.clock_node()
        watcher = LandReceiptWatchdog(node)
        watcher.observe(node, 1.)
        with self.assertRaisesRegex(RuntimeError, "No new land callback"):
            watcher.check(1.)
        node.received["land"] = 100.
        watcher.observe(node, 2.)
        self.assertEqual(watcher.check(5.), 3.)
        with self.assertRaisesRegex(RuntimeError, "simulation seconds without a new callback"):
            watcher.check(5.001)
        watcher.observe(node, 2.)  # paused simulation and duplicate receipt stay stable
        self.assertEqual(watcher.sequence, 1)
        node.received["land"] = 99.
        with self.assertRaisesRegex(RuntimeError, "receipt monotonic time regressed"):
            watcher.observe(node, 2.)
        node.received["land"] = 101.
        with self.assertRaisesRegex(RuntimeError, "simulation clock regressed"):
            watcher.observe(node, 1.99)

    def test_watchdog_observes_each_pump_even_before_control_deadline(self):
        with tempfile.TemporaryDirectory() as tmp, patch.object(runner.time, "monotonic", return_value=100.):
            node = self.clock_node()
            ctx = NS(node=node, vehicle=NS(_pump=lambda: None))
            node.spin_once = lambda **kwargs: None
            capture = NS(now=lambda: 1., check=lambda: None)
            loop = ControlLoop(ctx, {}, NS(joint_lower=[-1., -.5], joint_upper=[1., 1.]), capture, {}, Path(tmp))
            try:
                loop.next_control = 2.
                node.received["land"] = 99.
                with patch.dict(sys.modules, {"pymavlink": NS(mavutil=NS())}):
                    loop.pump()
                self.assertEqual(loop.land_watchdog.sequence, 1)
                self.assertEqual(loop.land_watchdog.observed_sim_s, 1.)
                self.assertEqual(loop.rows, 0)
            finally:
                loop.close()

    def test_landing_confirmation_rejects_cached_landed_until_new_post_command_report(self):
        with patch.object(runner.time, "monotonic", return_value=100.):
            node = self.clock_node()
            marker = transition_marker(node)
            watcher = LandReceiptWatchdog(node)
            node.received["land"] = 99.
            watcher.observe(node, 1.)
            calls = []
            def wait(predicate, timeout, description):
                calls.append(predicate())
                node.data["position"].timestamp = 2000000
                node.received["status"] = 100.1
                node.data["status"].timestamp = 1
                calls.append(predicate())  # new disarmed status alone is insufficient
                node.received["land"] = 100.2
                node.data["land"].timestamp = 1  # epochs can regress even on new callbacks
                watcher.observe(node, 1.)
                calls.append(predicate())
            result = confirm_land_state_after_command(NS(node=node, vehicle=NS(wait=wait)), marker, landed=True,
                                                      watchdog=watcher, sim_clock=lambda: 1.)
            self.assertEqual(calls, [False, False, True])
            self.assertTrue(result["after"]["requires_new_report_after_command"])
            self.assertFalse(result["after"]["armed"])

    def test_new_callback_with_wrong_landed_or_arm_state_cannot_certify_takeoff(self):
        with patch.object(runner.time, "monotonic", return_value=100.):
            node = self.clock_node()
            marker, watcher = transition_marker(node), LandReceiptWatchdog(node)
            node.received.update(land=100.1, status=100.1)
            watcher.observe(node, 1.)
            calls = []
            def wait(predicate, timeout, description):
                calls.append(predicate())  # both new, still landed/disarmed
                node.data["land"].landed = False
                calls.append(predicate())  # airborne alone, still disarmed
                node.data["status"].arming_state = 2
                calls.append(predicate())
            confirm_land_state_after_command(NS(node=node, vehicle=NS(wait=wait)), marker, landed=False,
                                             watchdog=watcher, sim_clock=lambda: 1.)
            self.assertEqual(calls, [False, False, True])

    def test_figure_eight_matches_declared_period_amplitude_and_c2_endpoints(self):
        for stage, period in (("small", 16.), ("normal", 12.), ("fast", 8.)):
            cfg = build_task_config(stage)
            center = np.array([.1, .2, 2.])
            trajectory = FigureEight(center, **cfg["trajectory_parameters"])
            self.assertEqual(trajectory.period_s, period)
            for t in (0., trajectory.start_s, trajectory.end_s, trajectory.end_s+1.):
                p, v, a = trajectory.sample(t)
                np.testing.assert_allclose(p, center, atol=1e-14)
                np.testing.assert_allclose(v, 0., atol=1e-14)
                np.testing.assert_allclose(a, 0., atol=1e-14)
            samples = np.array([trajectory.sample(t)[0]-center for t in np.linspace(trajectory.start_s, trajectory.end_s, 10001)])
            self.assertAlmostEqual(samples[:, 0].max(), cfg["trajectory_parameters"]["amplitude_x"], places=6)
            self.assertAlmostEqual(samples[:, 2].max(), cfg["trajectory_parameters"]["amplitude_z"], places=6)

    def test_joint_probe_is_small_individual_then_combined_and_returns_home(self):
        duration = 30.
        for t in (0., 2., 28., 30.):
            q, v = joint_probe_reference(t, duration)
            np.testing.assert_allclose(q, 0., atol=1e-14)
            np.testing.assert_allclose(v, 0., atol=1e-14)
        segment = 26./3.
        q, _ = joint_probe_reference(2.+segment/4, duration)
        self.assertGreater(q[0], .07)
        self.assertEqual(q[1], 0.)
        q, _ = joint_probe_reference(2.+segment+segment/4, duration)
        self.assertGreater(q[1], .07)
        self.assertEqual(q[0], 0.)
        samples = [joint_probe_reference(t, duration) for t in np.linspace(0., duration, 1001)]
        self.assertLessEqual(np.max(abs(np.array([s[0] for s in samples]))), .0800001)
        self.assertLess(np.max(abs(np.array([s[1] for s in samples]))), .1)

    def test_response_pulses_have_both_signs_each_axis_and_no_external_force(self):
        for axis in range(3):
            positive = response_probe_velocity(2.5+4*axis, 30.)
            negative = response_probe_velocity(4.5+4*axis, 30.)
            self.assertGreater(positive[axis], 0.)
            np.testing.assert_array_equal(positive, -negative)
            self.assertEqual(np.count_nonzero(positive), 1)
        np.testing.assert_array_equal(response_probe_velocity(15., 30.), np.zeros(3))

    def test_gates_need_matching_model_and_unchanged_raw_files(self):
        with tempfile.TemporaryDirectory() as tmp:
            folder = Path(tmp)
            cfg = build_task_config("joint_probe")
            cfg["stage_evidence"] = {k: True for k in required_gates(cfg)}
            with self.assertRaises((ValueError, TypeError)):
                formal_gate_evidence(cfg, "a"*64, "b"*64)
            raw = folder/"native.csv"
            raw.write_text("synthetic unit artifact")
            for gate in required_gates(cfg):
                path = folder/(gate+".json")
                path.write_text(json.dumps(dict(kind="x650_native_stage_evidence", stage=gate,
                    complete=True, passed=True, simulation_backend="gazebo_px4", model_sha256="a"*64,
                    source_urdf_sha256="b"*64, artifacts={"native.csv": hashlib.sha256(raw.read_bytes()).hexdigest()})))
                cfg["stage_evidence"][gate] = str(path)
            self.assertEqual(set(formal_gate_evidence(cfg, "a"*64, "b"*64)), set(required_gates(cfg)))
            with self.assertRaises(ValueError):
                formal_gate_evidence(cfg, "c"*64, "b"*64)
            raw.write_text("tampered unit artifact")
            with self.assertRaises(ValueError):
                formal_gate_evidence(cfg, "a"*64, "b"*64)

    def test_actual_pid_readback_is_required_and_profile_does_not_retune(self):
        evidence = NS(snapshots={"before": {"parameters": {"MPC_XY_P": {"value": 1.8}, "MPC_Z_P": {"value": .9}, "SYS_AUTOSTART": {"value": 4998}}}})
        values = actual_pid_parameters(evidence)
        self.assertEqual(values, {"MPC_XY_P": 1.8, "MPC_Z_P": .9})
        self.assertFalse(verify_pid_profile(None, values)["tuning_claim"])
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp)/"pid.json"
            path.write_text(json.dumps({"parameters": values}))
            self.assertFalse(verify_pid_profile(path, values)["parameter_writes"])
            path.write_text(json.dumps({"parameters": {"MPC_XY_P": 2.}}))
            with self.assertRaises(RuntimeError):
                verify_pid_profile(path, values)

    def test_identified_response_with_wrong_pid_hash_is_rejected_before_mpc_construction(self):
        from x650_wholebody import mpc
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp)/"response_model.json"
            path.write_text(json.dumps({"identified": True, "passed": True,
                "base_time_constant_s": [.2, .2, .25], "pid_parameters_sha256": "0"*64}))
            cfg = {"controller": "mpc", "response_model": str(path)}
            actual = {"MPC_XY_P": 1.8, "MPC_Z_P": .9, "MC_ROLL_P": 6.5}
            with patch.object(mpc, "MpcController") as constructor:
                with self.assertRaisesRegex(RuntimeError, "current PX4 PID readback"):
                    runner.build_allocator(cfg, object(), actual, None)
                constructor.assert_not_called()


if __name__ == "__main__":
    unittest.main()
