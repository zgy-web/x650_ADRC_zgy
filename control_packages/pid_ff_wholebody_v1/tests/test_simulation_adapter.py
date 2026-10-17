"""Mock only: exercise the AMSIM adapter without ROS, MAVLink or physics."""
from __future__ import annotations

import importlib.util
import json
import math
import os
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace as NS
import unittest
from unittest.mock import patch

import numpy as np

ROOT = Path(__file__).resolve().parents[3]
SCENE_PATH = ROOT / "simulations/pid_ff_wholebody_v1/run_case.py"
spec = importlib.util.spec_from_file_location("flyarm_pid_ff_scene_adapter", SCENE_PATH)
scene = importlib.util.module_from_spec(spec)
spec.loader.exec_module(scene)


class FakeNode:
    def __init__(self):
        self.t = 10.
        self.data = {
            "status": NS(failsafe=False, nav_state=4),
            "position": NS(x=0., y=0., z=-2.5, vx=0., vy=0., vz=0., heading=0., timestamp=10000000),
        }
        self.update()

    def update(self):
        stamp = NS(sec=int(self.t), nanosec=round((self.t - int(self.t)) * 1e9))
        self.data["truth"] = NS(
            header=NS(stamp=stamp),
            pose=NS(pose=NS(position=NS(x=0., y=0., z=2.5), orientation=NS(w=1., x=0., y=0., z=0.))),
            twist=NS(twist=NS(linear=NS(x=0., y=0., z=0.), angular=NS(x=0., y=0., z=0.))),
        )
        self.data["joints"] = NS(header=NS(stamp=stamp))
        self.data["position"].timestamp = round(self.t * 1e6)

    def spin_once(self, timeout_sec=0.):
        pass

    def joint_values(self):
        return {name: NS(position_rad=0., velocity_rad_s=0.) for name in scene.JOINTS}

    def sim_time_s(self):
        return self.t

    def create_subscription(self, *args, **kwargs):
        return None


class FakeVehicle:
    def __init__(self, node, fail):
        self.node, self.fail = node, fail
        self.landed, self.target, self.sent, self.pending = False, (1, 1), 0, None
        self.takeoff_called = False
        self.link = NS(mav=NS(param_set_send=self.param, set_position_target_local_ned_send=self.send),
                       recv_match=self.recv)

    def param(self, *args):
        self.pending = NS(param_id=args[2], param_value=args[3])

    def recv(self, **kwargs):
        value, self.pending = self.pending, None
        return value

    def send(self, *args):
        self.sent += 1
        if self.fail and self.sent == 30:
            raise RuntimeError("injected output failure")

    def _pump(self):
        self.node.t += .02
        self.node.update()

    def wait_ready(self):
        pass

    def takeoff(self, *args, **kwargs):
        self.takeoff_called = True

    def land(self):
        self.landed = True

    def _command(self, *args):
        self.node.data["status"].nav_state = 14

    def wait(self, predicate, *args):
        if not predicate():
            raise AssertionError("Mock condition was not met")


class FakeContext:
    fail = False
    last = None

    def __init__(self):
        self.node = FakeNode()
        self.vehicle = FakeVehicle(self.node, self.fail)
        self.stops = 0
        self.arm = NS(stop=self.stop, set_velocity=lambda *args: None)
        type(self).last = self

    def stop(self):
        self.stops += 1

    def __enter__(self):
        return self

    def __exit__(self, *args):
        pass

    def wait(self, duration):
        self.node.t += duration
        self.node.update()


class SimulationAdapterTests(unittest.TestCase):
    def run_mock(self, *, fail=False, model_mismatch=False):
        FakeContext.fail = fail
        imports = {
            "amsim": NS(ScenarioContext=FakeContext),
            "pymavlink": NS(mavutil=NS(mavlink=NS(MAV_PARAM_TYPE_INT32=6, MAV_PARAM_TYPE_REAL32=9,
                                                   MAV_FRAME_LOCAL_NED=1, MAV_CMD_DO_SET_MODE=176))),
            "rosgraph_msgs": NS(), "rosgraph_msgs.msg": NS(Clock=object),
            "rclpy": NS(), "rclpy.qos": NS(QoSProfile=lambda **kwargs: kwargs,
                ReliabilityPolicy=NS(BEST_EFFORT=1), HistoryPolicy=NS(KEEP_LAST=1)),
        }
        wall = [0.]

        def monotonic():
            wall[0] += .01
            return wall[0]

        with tempfile.TemporaryDirectory(prefix="pid_ff_adapter_mock_") as temporary:
            output = Path(temporary)
            # Parsing and compatibility are covered by numerical/model tests.
            # This file exists solely to exercise adapter provenance hashing.
            (output / "model_snapshot.sdf").write_text("mock fixture: no physical simulation\n")
            environment = os.environ.copy()
            environment.pop("PID_FF_TASK_CONFIG", None)
            environment["AMSIM_RUN_DIR"] = str(output)
            with (patch.dict(sys.modules, imports), patch.dict(os.environ, environment, clear=True),
                  patch.object(scene, "time", NS(monotonic=monotonic, perf_counter=lambda: wall[0])),
                  patch.object(scene.ArmParameters, "from_sdf", return_value=(
                      scene.ArmParameters(total_mass=3.) if model_mismatch else scene.ArmParameters()))):
                if model_mismatch:
                    with self.assertRaisesRegex(ValueError, "Model differs"):
                        scene.scenario()
                elif fail:
                    with self.assertRaisesRegex(RuntimeError, "injected output failure"):
                        scene.scenario()
                else:
                    scene.scenario()
            metadata = json.loads((output / "controller_metadata.json").read_text())
            trace = (output / "control_trace.csv").read_text()
        ctx = FakeContext.last
        if model_mismatch:
            self.assertFalse(metadata["complete"])
            self.assertFalse(ctx.vehicle.takeoff_called)
            self.assertFalse(ctx.vehicle.landed)
            self.assertEqual(ctx.vehicle.sent, 0)
            self.assertIs(ctx.vehicle._pump.__func__, FakeVehicle._pump)
            return metadata
        self.assertEqual(metadata["complete"], not fail)
        self.assertTrue(ctx.vehicle.landed)
        self.assertGreaterEqual(ctx.stops, 2)
        self.assertIs(ctx.vehicle._pump.__func__, FakeVehicle._pump)
        self.assertEqual(len(metadata["verified_parameters"]), 26)
        self.assertTrue(metadata["model_compatibility_verified_before_takeoff"])
        self.assertTrue(metadata["landed_and_disarmed"])
        self.assertIn("arm_ff_a_x", trace.splitlines()[0].split(","))
        return metadata

    def test_hover_finishes_and_restores_vehicle_pump(self):
        result = self.run_mock()
        self.assertTrue(result["trajectory_finished"])
        self.assertGreater(result["control_rows"], 400)

    def test_failure_still_stops_arm_and_lands(self):
        result = self.run_mock(fail=True)
        self.assertIn("injected output failure", result["error"])

    def test_model_mismatch_is_rejected_before_takeoff(self):
        self.run_mock(model_mismatch=True)

    def test_child_frame_linear_velocity_becomes_world_velocity(self):
        ctx = FakeContext()
        ctx.node.data["truth"].pose.pose.orientation = NS(w=math.sqrt(.5), x=0., y=0., z=math.sqrt(.5))
        ctx.node.data["truth"].twist.twist.linear = NS(x=1., y=0., z=0.)
        state, joint_stamp = scene.read_state(ctx)
        np.testing.assert_allclose(state.velocity, [0., 1., 0.], atol=1e-14)
        self.assertEqual(state.stamp_s, joint_stamp)

    def test_flight_monitor_rejects_failsafe(self):
        ctx = FakeContext()
        ctx.node.data["status"].nav_state = 14
        ctx.node.data["status"].failsafe = True
        state, _ = scene.read_state(ctx)
        with self.assertRaisesRegex(RuntimeError, "failsafe"):
            scene.check_flight_state(ctx, state, state.position)

    def test_presets_keep_frozen_limits_and_speed(self):
        from launch import build_task_config, validate_task_config
        for speed, duration in (("hover", 8.), ("normal", 32.), ("fast", 24.)):
            cfg = build_task_config(speed, "ON")
            self.assertEqual(cfg["duration_s"], duration)
            self.assertEqual(cfg["mode"], "compensated")
            self.assertEqual(cfg, validate_task_config(cfg))
            cfg["control_period_s"] = .1
            with self.assertRaisesRegex(ValueError, "Unsupported"):
                validate_task_config(cfg)


if __name__ == "__main__":
    unittest.main()
