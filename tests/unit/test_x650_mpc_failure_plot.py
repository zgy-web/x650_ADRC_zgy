"""Synthetic partial-failure display tests; not real flight evidence."""
from dataclasses import asdict
from pathlib import Path
import sys
import tempfile
import unittest
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
for path in (ROOT/"scripts", ROOT/"control_packages/x650_wholebody_v1/tests", Path(__file__).parent):
    sys.path.insert(0, str(path))
from test_x650_mixed_stage_comparison_plot import fixture, write_json, write_csv
from test_model import example_model
import plot_x650_mpc_failure as plot


class FailurePlotTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.root = Path(self.temp.name)
        _, self.folder = fixture(self.root, "mpc", 10., [0, 0, .6])
        meta = plot.read(self.folder/"controller_metadata.json")
        meta.update(complete=False, phase="failed", reference_rotation=np.eye(3).tolist(),
                    physical_profile=asdict(example_model()), error="synthetic bounds failure")
        # Failed execution never acquired an official task_end.
        del meta["task_end_sim_s"]
        write_json(self.folder/"controller_metadata.json", meta)
        control = plot.display.csv_rows(self.folder/"control_trace.csv")
        for row in control:
            row.update(t_s=float(row["sim_s"])-10., pva_sent="True", q2=0.)
        write_csv(self.folder/"control_trace.csv", control)

    def tearDown(self):
        self.temp.cleanup()

    def test_partial_failed_run_renders_without_task_end_or_certificate(self):
        result = plot.generate(self.folder, self.root/"output")
        self.assertTrue(result["diagnostic_only"])
        self.assertFalse(result["experiment_complete"])
        self.assertFalse(result["eligible_as_stage_gate"])
        self.assertFalse(result["full_window_pid_comparison_permitted"])
        self.assertAlmostEqual(result["partial_duration_s"], .2)
        self.assertEqual(result["successful_sent_commands"], 11)
        self.assertEqual(len(result["outputs_sha256"]), 2)
        for path, digest in result["outputs_sha256"].items():
            self.assertEqual(plot.sha(path), digest)

    def test_refuses_complete_run_and_unsent_measurement(self):
        path = self.folder/"controller_metadata.json"
        meta = plot.read(path)
        meta["complete"] = True
        write_json(path, meta)
        with self.assertRaisesRegex(ValueError, "incomplete failed"):
            plot.load_failure(self.folder)
        meta["complete"] = False
        write_json(path, meta)
        path = self.folder/"control_trace.csv"
        rows = plot.display.csv_rows(path)
        rows[-1]["pva_sent"] = "False"
        write_csv(path, rows)
        with self.assertRaisesRegex(ValueError, "actually been sent"):
            plot.load_failure(self.folder)


if __name__ == "__main__":
    unittest.main()
