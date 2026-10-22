"""Offline reference contracts; synthetic fixtures are not motor evidence."""
from pathlib import Path
import hashlib
import json
import shutil
import sys
import unittest
import uuid

import numpy as np

PACKAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE))
from x650_wholebody.ts20 import (FrozenTs20Model, MANIFEST_SCHEMA,
    POSITION_STEP_RAD, VELOCITY_STEP_RAD_S, TORQUE_STEP_NM,
    build_mit_commands, quantize_half_away)


def fixture(root):
    """Own minimal local test package, never copies upstream implementation."""
    directory = root / "models" / "ts20_50"
    directory.mkdir(parents=True)
    contents = {
        "__init__.py": "from .runtime import simulate_motion\n",
        "runtime.py": ("from . import mechanics\n"
                       "def simulate_motion(trial, *, max_seconds, max_rhs):\n"
                       "    return dict(value=mechanics.VALUE, trial=trial, seconds=max_seconds, rhs=max_rhs)\n"),
        "mechanics.py": "VALUE = 123\n", "observers.py": "# own test fixture\n",
        "example.py": "raise RuntimeError('example must never execute')\n",
        "motion.json": json.dumps({"schema": "ts20_public_motion_v1", "motor": "TS20-50",
                                    "status": "frozen_conditional_interface_model"}),
        "reported_outputs.json": "{}", "thermal.json": "{}",
        "conditional_candidates.json": "{}", "provenance.json": "{}",
        "README.md": "Own synthetic package; not motor validation.",
        "requirements.txt": "# test fixture\n"}
    for name, content in contents.items():
        (directory / name).write_text(content, encoding="utf-8")
    files = {p.relative_to(root).as_posix(): hashlib.sha256(p.read_bytes()).hexdigest()
             for p in directory.iterdir()}
    return directory, {"schema": MANIFEST_SCHEMA, "motor": "TS20-50",
                       "upstream_commit": "a" * 40, "files": files}


class CommandBridgeTests(unittest.TestCase):
    def test_nonuniform_source_exact_integral_on_fixed_bus_grid(self):
        t = np.array([0., .007, .013, .022])
        velocity = np.array([[.2, -.1], [-.1, .3], [.4, -.2], [0., 0.]])
        result = build_mit_commands(t, velocity, [.1, -.2])
        np.testing.assert_allclose(result["input_time_s"], [0., .005, .01, .015, .02], atol=1e-16)
        # Independent hand integral crosses both non-grid velocity switches.
        q = np.array([[.1, -.2], [.101, -.2005], [.1011, -.1998],
                      [.1016, -.1993], [.1036, -.2003]])
        expected = quantize_half_away(q, POSITION_STEP_RAD)
        np.testing.assert_array_equal(result["command"]["q_ref"], expected)
        np.testing.assert_array_equal(result["command"]["dq_ref"],
            quantize_half_away(velocity[[0, 0, 1, 2, 2]], VELOCITY_STEP_RAD_S))
        np.testing.assert_allclose(result["metadata"]["integrated_reference_end"], [.1044, -.2007])
        self.assertEqual(result["metadata"]["reference_end_time_s"], .022)
        self.assertEqual(result["metadata"]["last_send_time_s"], .02)
        self.assertFalse(result["metadata"]["gain_encoding_applied"])
        self.assertFalse(result["metadata"]["supply_24v_qualified"])
        np.testing.assert_array_equal(result["command"]["kp"], 6.)
        np.testing.assert_array_equal(result["command"]["kd"], 3276 / 32768)
        np.testing.assert_array_equal(result["command"]["tau_ff"], 0.)

    def test_half_away_rounding_for_all_protocol_steps(self):
        for step in (POSITION_STEP_RAD, VELOCITY_STEP_RAD_S, TORQUE_STEP_NM):
            values = np.array([-.5, .5, -1.5, 1.5, -2.5, 2.5, 0.]) * step
            np.testing.assert_array_equal(quantize_half_away(values, step),
                np.array([-1., 1., -2., 2., -3., 3., 0.]) * step)

    def test_nonzero_origin_and_fixed_bus_no_short_tail(self):
        result = build_mit_commands([2., 2.006, 2.012], [[1., -1.]] * 3, [.3, .4])
        np.testing.assert_allclose(result["input_time_s"], [2., 2.005, 2.01])
        expected = np.array([[.3, .4], [.305, .395], [.31, .39]])
        np.testing.assert_allclose(result["command"]["q_ref"], expected, atol=POSITION_STEP_RAD / 2)
        for origin, end, expected_count in ((1., 1.015, 4), (5., 5.02, 5)):
            fixed = build_mit_commands([origin, end], [[0., 0.]] * 2, [0., 0.])
            self.assertEqual(len(fixed["input_time_s"]), expected_count)
            self.assertEqual(fixed["input_time_s"][-1], end)

    def test_limit_violations_reject_without_clipping(self):
        # The original trajectory exceeds a limit and returns between bus sends.
        with self.assertRaisesRegex(ValueError, "qlimits"):
            build_mit_commands([0., .002, .004, .01],
                [[1., 0.], [-1., 0.], [0., 0.], [0., 0.]], [0., 0.],
                qlimits=([-.001, -.1], [.001, .1]))
        with self.assertRaisesRegex(ValueError, "vlimits"):
            build_mit_commands([0., .01], [[.46, 0.], [0., 0.]], [0., 0.], vlimits=[.45, .35])
        # Quantization can cross a bound even when the source value is inside.
        with self.assertRaisesRegex(ValueError, "vlimits"):
            build_mit_commands([0., .01], [[.6 * VELOCITY_STEP_RAD_S, 0.]] * 2,
                               [0., 0.], vlimits=[.8 * VELOCITY_STEP_RAD_S, 1.])
        with self.assertRaisesRegex(ValueError, "qlimits"):
            build_mit_commands([0., .01], [[0., 0.]] * 2,
                [.6 * POSITION_STEP_RAD, 0.],
                qlimits=([-1., -1.], [.8 * POSITION_STEP_RAD, 1.]))

    def test_protocol_range_is_distinct_from_working_limits(self):
        with self.assertRaisesRegex(ValueError, "protocol encoding"):
            build_mit_commands([0., .01], [[64., 0.]] * 2, [0., 0.])
        with self.assertRaisesRegex(ValueError, "protocol encoding"):
            build_mit_commands([0., .01], [[0., 0.]] * 2, [1000., 0.])

    def test_malformed_inputs_reject(self):
        for t, v, q in (([0., 0.], [[0., 0.]] * 2, [0., 0.]),
                        ([0., .01], [0., 0.], [0., 0.]),
                        ([0., .01], [[0., 0.]] * 2, [False, False]),
                        ([0., .01], [[0., np.nan]] * 2, [0., 0.]),
                        ([0., .0005], [[0., 0.]] * 2, [0., 0.])):
            with self.assertRaises(ValueError):
                build_mit_commands(t, v, q)
        for kwargs in ({"bus_period_s": .02}, {"kp": -1}, {"kd": True},
                       {"qlimits": ([0., 0.], [0., 1.])}, {"vlimits": [0., 1.]}):
            with self.assertRaises(ValueError):
                build_mit_commands([0., .01], [[0., 0.]] * 2, [0., 0.], **kwargs)


class SourceIdentityTests(unittest.TestCase):
    def setUp(self):
        # Normal inherited permissions work in the managed Windows sandbox;
        # Python's mode-0700 tempfile directory does not allow its test children.
        self.root = PACKAGE / "tmp" / ("ts20_fixture_" + uuid.uuid4().hex)
        self.root.mkdir(parents=True)
        self.directory, self.manifest = fixture(self.root)

    def tearDown(self):
        if not self.root.resolve().is_relative_to((PACKAGE / "tmp").resolve()):
            raise RuntimeError("test cleanup target escaped package temporary directory")
        shutil.rmtree(self.root)

    def test_unique_alias_relative_import_and_budget_forwarding(self):
        first = FrozenTs20Model(self.root, self.manifest)
        second = FrozenTs20Model(self.root, self.manifest)
        self.assertNotEqual(first._alias, second._alias)
        self.assertNotIn("models.ts20_50", sys.modules)
        trial = {"explicit": "synthetic initial condition"}
        self.assertEqual(first.simulate_motion(trial, max_seconds=120, max_rhs=5_000_000),
            {"value": 123, "trial": trial, "seconds": 120, "rhs": 5_000_000})
        metadata = first.metadata()
        self.assertFalse(metadata["hardware_accessed"])
        self.assertFalse(metadata["aerial_arm_load_qualified"])
        metadata["source_manifest"]["motor"] = "changed"
        self.assertEqual(first.metadata()["source_manifest"]["motor"], "TS20-50")

    def test_corrupted_source_rejected_before_import(self):
        (self.directory / "mechanics.py").write_text("raise RuntimeError('must not execute')\n")
        with self.assertRaisesRegex(ValueError, "identity mismatch"):
            FrozenTs20Model(self.root, self.manifest)

    def test_mutation_after_loading_rejected_before_prediction(self):
        model = FrozenTs20Model(self.root, self.manifest)
        (self.directory / "motion.json").write_text("{}")
        with self.assertRaisesRegex(ValueError, "identity mismatch"):
            model.simulate_motion({})

    def test_unpinned_python_file_and_missing_file_reject(self):
        (self.directory / "extra.py").write_text("raise RuntimeError('must not execute')")
        with self.assertRaisesRegex(ValueError, "complete numerical source tree"):
            FrozenTs20Model(self.root, self.manifest)
        (self.directory / "extra.py").unlink()
        self.manifest["files"].pop("models/ts20_50/runtime.py")
        with self.assertRaisesRegex(ValueError, "omits required"):
            FrozenTs20Model(self.root, self.manifest)

    def test_wrong_motor_and_wrong_model_identity_reject(self):
        self.manifest["motor"] = "TS20-100"
        with self.assertRaisesRegex(ValueError, "TS20-50 manifest"):
            FrozenTs20Model(self.root, self.manifest)
        self.manifest["motor"] = "TS20-50"
        path = self.directory / "motion.json"
        path.write_text(json.dumps({"motor": "TS20-100"}), encoding="utf-8")
        self.manifest["files"]["models/ts20_50/motion.json"] = hashlib.sha256(path.read_bytes()).hexdigest()
        with self.assertRaisesRegex(ValueError, "not frozen TS20-50"):
            FrozenTs20Model(self.root, self.manifest)


if __name__ == "__main__":
    unittest.main()
