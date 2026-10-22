"""Synthetic adapter fixtures only; no flight or experiment result is produced."""
import json
import math
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT/"scripts"))
import prepare_x650_mixed_native_replay as adapter
import render_x650_arm_replay as renderer
from test_x650_mixed_stage_comparison_plot import fixture, write_csv, write_json


class NativeReplayTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.root = Path(self.temp.name)
        center = [.3*math.cos(math.radians(150))+.25, .04, 1-.18-.15]
        self.audit, self.run = fixture(self.root, "mpc", 25., center)
        metadata = adapter.accepted.read(self.run/"controller_metadata.json")
        metadata["physical_profile"] = dict(profile_name="x650_2link_v2", mount_body=[0, 0, -.18],
            elbow_offset_body_m=[0, .04, 0], angle_offsets=[math.radians(150), 0], lengths=[.3, .25])
        write_json(self.run/"controller_metadata.json", metadata)
        self.original = adapter.accepted.csv_rows(self.run/"native_trace.csv")
        for row in self.original:
            row["base_px"] = ".010000000000"
        write_csv(self.run/"native_trace.csv", self.original)
        write_json(self.run/"native_trace_manifest.json", dict(complete=True, file="native_trace.csv",
            sha256=adapter.accepted.sha(self.run/"native_trace.csv"), model_sha256="a"*64,
            source_urdf_sha256="b"*64, task_window_sim_s=[25., 25.2]))
        self.refresh()

    def refresh(self):
        report = adapter.accepted.read(self.audit)
        report["inputs_sha256"] = {str(p.resolve()): adapter.accepted.sha(p)
                                   for p in self.run.iterdir() if p != self.audit}
        write_json(self.audit, report)

    def tearDown(self):
        self.temp.cleanup()

    def test_preserves_every_native_state_and_bias_with_renderer_compatible_input(self):
        with patch("scipy.optimize.minimize", side_effect=AssertionError("no solve")), \
             patch("scipy.optimize.least_squares", side_effect=AssertionError("no fit")):
            manifest = adapter.prepare(self.audit, self.run, self.root/"derived")
        rows = adapter.accepted.csv_rows(self.root/"derived/native_replay.csv")
        self.assertEqual(len(rows), len(self.original))
        for original, derived in zip(self.original, rows):
            for destination, source in adapter.MAPPING.items():
                self.assertEqual(derived[destination], str(original[source]))
        self.assertFalse(manifest["original_flight_recorder_capture"])
        self.assertFalse(manifest["resampling"])
        self.assertFalse(manifest["eligible_as_stage_gate"])
        self.assertAlmostEqual(manifest["primary_native_tip_error"]["mean_vector"][0], .01)
        states = renderer.load_samples(self.root/"derived/native_replay.csv")
        np.testing.assert_allclose(states["provided_tip"]-states["target"], [[.01, 0, 0]]*len(rows), atol=1e-12)
        self.assertEqual(states["t"][0], 25.)
        self.assertEqual(manifest["sha256"], adapter.accepted.sha(self.root/"derived/native_replay.csv"))

    def test_unpassed_or_changed_original_is_rejected(self):
        report = adapter.accepted.read(self.audit)
        report["passed"] = False
        write_json(self.audit, report)
        with self.assertRaisesRegex(ValueError, "accepted"):
            adapter.prepare(self.audit, self.run, self.root/"bad")
        report["passed"] = True
        write_json(self.audit, report)
        with (self.run/"native_trace.csv").open("a") as stream:
            stream.write("\n")
        with self.assertRaisesRegex(ValueError, "missing or changed"):
            adapter.prepare(self.audit, self.run, self.root/"bad")
        self.assertFalse((self.root/"bad").exists())

    def test_invalid_native_flag_is_not_dropped(self):
        self.original[10]["valid_joint_state"] = "0"
        write_csv(self.run/"native_trace.csv", self.original)
        self.refresh()
        with self.assertRaisesRegex(ValueError, "valid independent native"):
            adapter.prepare(self.audit, self.run, self.root/"bad")

    def test_geometry_is_checked(self):
        metadata = adapter.accepted.read(self.run/"controller_metadata.json")
        metadata["physical_profile"]["lengths"][0] = .31
        write_json(self.run/"controller_metadata.json", metadata)
        self.refresh()
        with self.assertRaisesRegex(ValueError, "native tip disagrees"):
            adapter.prepare(self.audit, self.run, self.root/"bad")

    def test_native_manifest_identity_and_complete_window_are_checked(self):
        path = self.run/"native_trace_manifest.json"
        original = adapter.accepted.read(path)
        for change in ({"source_urdf_sha256": "c"*64}, {"task_window_sim_s": [25.02, 25.2]},
                       {"complete": False}):
            write_json(path, {**original, **change})
            self.refresh()
            with self.subTest(change=change), self.assertRaises(ValueError):
                adapter.prepare(self.audit, self.run, self.root/"bad")
        self.assertFalse((self.root/"bad").exists())

    def test_existing_output_is_preserved(self):
        output = self.root/"keep"
        output.mkdir()
        (output/"note").write_text("keep")
        with self.assertRaisesRegex(ValueError, "new output directory"):
            adapter.prepare(self.audit, self.run, output)
        self.assertEqual((output/"note").read_text(), "keep")


if __name__ == "__main__":
    unittest.main()
