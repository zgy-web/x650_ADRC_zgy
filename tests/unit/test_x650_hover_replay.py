"""Replay must preserve physical timestamps and quaternion conventions."""
import csv
import importlib.util
import json
from pathlib import Path
import tempfile
import unittest

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
spec = importlib.util.spec_from_file_location("x650_replay", ROOT / "scripts/render_x650_hover.py")
replay = importlib.util.module_from_spec(spec)
spec.loader.exec_module(replay)


class ReplayTests(unittest.TestCase):
    def data(self):
        return {"elapsed": np.array([0., .1]), "p": np.array([[0., 0., 0.], [2., 4., 6.]]),
                "q": np.array([[1., 0., 0., 0.], [0., 0., 0., 1.]])}

    def test_translation_and_wxyz_slerp(self):
        p, q, idx, gap = replay.sample_pose(self.data(), .05)
        np.testing.assert_allclose(p, [1, 2, 3])
        np.testing.assert_allclose(q, [2**-.5, 0, 0, 2**-.5], atol=1e-14)
        self.assertEqual(idx, 0); self.assertFalse(gap)

    def test_quaternion_double_cover_does_not_add_rotation(self):
        data = self.data(); data["q"][1] = [-1, 0, 0, 0]
        _, q, _, _ = replay.sample_pose(data, .05)
        np.testing.assert_allclose(q, [1, 0, 0, 0])

    def test_telemetry_gap_not_fabricated(self):
        data = self.data(); data["elapsed"][1] = 2
        p, q, _, gap = replay.sample_pose(data, 1)
        np.testing.assert_array_equal(p, data["p"][0])
        np.testing.assert_array_equal(q, data["q"][0])
        self.assertTrue(gap)

    def make_csv(self, path, times=(100., 100.05), q=(0, 0, 0, 1)):
        with path.open("w", newline="", encoding="utf-8") as stream:
            writer = csv.DictWriter(stream, fieldnames=replay.REQUIRED)
            writer.writeheader()
            for t in times:
                row = {field: 0 for field in replay.REQUIRED}
                row.update(sim_time_s=t, phase="measure", world_z_m=2, target_world_z_m=2,
                           world_qx=q[0], world_qy=q[1], world_qz=q[2], world_qw=q[3])
                writer.writerow(row)

    def test_raw_timestamps_are_preserved(self):
        with tempfile.TemporaryDirectory() as d:
            path = Path(d)/"flight.csv"; self.make_csv(path)
            data = replay.load_samples(path)
            np.testing.assert_allclose(data["t"], [100, 100.05])
            np.testing.assert_allclose(data["elapsed"], [0, .05])
            np.testing.assert_array_equal(data["error"], np.zeros((2, 3)))
            self.assertTrue(data["measure"].all())

    def test_repeated_clock_is_rejected(self):
        with tempfile.TemporaryDirectory() as d:
            path = Path(d)/"flight.csv"; self.make_csv(path, times=(100, 100))
            with self.assertRaisesRegex(ValueError, "strictly increasing"):
                replay.load_samples(path)

    def test_bad_quaternion_is_rejected(self):
        with tempfile.TemporaryDirectory() as d:
            path = Path(d)/"flight.csv"; self.make_csv(path, q=(0, 0, 0, 2))
            with self.assertRaisesRegex(ValueError, "unit quaternion"):
                replay.load_samples(path)

    def test_live_or_modified_recording_is_rejected(self):
        with tempfile.TemporaryDirectory() as d:
            path = Path(d)/"flight_samples.csv"; self.make_csv(path)
            data = replay.load_samples(path)
            with self.assertRaisesRegex(ValueError, "Completed"):
                replay.verify_recording(path, data)
            path.with_name("flight_samples_manifest.json").write_text(json.dumps({
                "file": path.name, "sha256": "not the csv hash", "model_sha256": "abc"}))
            with self.assertRaisesRegex(ValueError, "checksum"):
                replay.verify_recording(path, data)

    def test_model_identity_matches_finalized_capture(self):
        with tempfile.TemporaryDirectory() as d:
            path = Path(d)/"flight_samples.csv"; self.make_csv(path)
            data = replay.load_samples(path)
            for row in data["rows"]: row["model_sha256"] = "abc"
            manifest = {"file": path.name, "sha256": replay.sha(path), "model_sha256": "abc"}
            path.with_name("flight_samples_manifest.json").write_text(json.dumps(manifest))
            self.assertEqual(replay.verify_recording(path, data)["metadata"]["model_sha256"], "abc")
            data["rows"][1]["model_sha256"] = "different"
            with self.assertRaisesRegex(ValueError, "model hash"):
                replay.verify_recording(path, data)


if __name__ == "__main__":
    unittest.main()
