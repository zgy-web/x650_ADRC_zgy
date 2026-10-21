"""Frozen-model figure replay against synthetic audited source files."""
from copy import deepcopy
import importlib.util
import json
from pathlib import Path
import shutil
import sys
import tempfile
import unittest
from unittest.mock import patch

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(Path(__file__).parent))
import test_x650_upper_stage_audit as fixtures
spec = importlib.util.spec_from_file_location("upper_response_plot_test", ROOT/"scripts/plot_x650_upper_response.py")
plot = importlib.util.module_from_spec(spec)
spec.loader.exec_module(plot)


def identification_fixture(root):
    pairs = []
    for name, seed in (("training", 1), ("holdout", 3)):
        folder = root/name
        run, launch, ulog = fixtures.complete_fixture(folder, seed=seed)
        accepted = folder/"accepted"
        fixtures.audit.audit(run, launch, accepted, ulog_loader=lambda _: ulog)
        pairs.append((run, accepted))
    output = root/"identification"
    report = plot.identifier.write_identification(pairs[0][0], pairs[1][0], output,
        train_audit=pairs[0][1], holdout_audit=pairs[1][1])
    assert report["passed"], report.get("reasons")
    return output


class UpperResponsePlotTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.temporary = tempfile.TemporaryDirectory()
        cls.root = Path(cls.temporary.name)
        cls.identification = identification_fixture(cls.root)

    @classmethod
    def tearDownClass(cls):
        cls.temporary.cleanup()

    def test_replay_has_original_axis_times_and_never_calls_optimizer(self):
        with patch.object(plot.identifier, "minimize_scalar", side_effect=AssertionError("No refit allowed")):
            bundle = plot.load_identification(self.identification)
        self.assertEqual(len(bundle["panels"]), 6)
        np.testing.assert_allclose(bundle["model"]["base_time_constant_s"], [.16, .2, .25], atol=.0005, rtol=0.)
        for panel in bundle["panels"]:
            self.assertGreaterEqual(len(panel["times"]), 1000)
            self.assertAlmostEqual(panel["times"][0], panel["start_s"])
            self.assertAlmostEqual(panel["times"][-1], panel["end_s"])
            self.assertEqual(panel["prediction"][0], panel["actual"][0])
            self.assertLess(panel["statistics"]["rmse_m_s"], .0001)

    def test_accepted_render_is_hash_bound_and_will_not_overwrite(self):
        output = self.root/"accepted_plot"
        manifest = plot.render(plot.load_identification(self.identification), output)
        self.assertEqual(manifest["figure_status"], "MODEL ACCEPTED")
        self.assertTrue(manifest["no_refit"])
        self.assertFalse(manifest["mpc_performance_claim"])
        for filename, digest in manifest["outputs_sha256"].items():
            self.assertEqual(plot.sha(output/filename), digest)
        with self.assertRaisesRegex(ValueError, "new plot directory"):
            plot.render(plot.load_identification(self.identification), output)

    def test_rejected_completed_model_is_explicitly_labeled_not_promoted(self):
        target = self.root/"rejected_identification"
        shutil.copytree(self.identification, target)
        model = plot.read_json(target/"response_model.json")
        report = plot.read_json(target/"response_identification.json")
        for document in (model, report):
            document.update(passed=False, identified=False, eligible_as_stage_gate=False, research_acceptance_verified=False)
        report.update(success=False, reasons=["Synthetic rejection fixture; not a physical performance claim"])
        fixtures.save(target/"response_model.json", model)
        report["artifacts"]["response_model.json"] = plot.sha(target/"response_model.json")
        fixtures.save(target/"response_identification.json", report)
        bundle = plot.load_identification(target)
        manifest = plot.render(bundle, self.root/"rejected_plot")
        self.assertEqual(manifest["figure_status"], "MODEL REJECTED")
        self.assertFalse(manifest["identification_passed"])
        self.assertEqual(bundle["model"]["base_time_constant_s"], model["base_time_constant_s"])

    def test_compact_input_map_preserves_exact_original_hashes(self):
        original = plot.load_identification(self.identification)
        directory = self.root/"compact"
        directory.mkdir()
        report = plot.read_json(self.identification/"response_identification.json")
        mapping = {}
        for index, source in enumerate(original["original_bound_artifacts"]):
            old = self.identification/source if source == "response_model.json" else Path(source)
            destination = directory/f"{index:02d}_{plot.name(source)}"
            shutil.copyfile(old, destination)
            mapping[source] = str(destination)
        fixtures.save(directory/"map.json", mapping)
        mapped = plot.load_identification(self.identification, directory/"map.json")
        np.testing.assert_array_equal(mapped["panels"][0]["actual"], original["panels"][0]["actual"])
        native = next(path for path in mapping if plot.name(path) == "native_trace.csv")
        Path(mapping[native]).write_bytes(Path(mapping[native]).read_bytes()+b"\n")
        with self.assertRaisesRegex(ValueError, "missing or changed"):
            plot.load_identification(self.identification, directory/"map.json")

    def test_changed_tau_or_unbound_incomplete_report_cannot_fabricate_plot(self):
        target = self.root/"changed_model"
        shutil.copytree(self.identification, target)
        model = plot.read_json(target/"response_model.json")
        model["base_time_constant_s"][0] += .1
        fixtures.save(target/"response_model.json", model)
        with self.assertRaisesRegex(ValueError, "missing or changed"):
            plot.load_identification(target)
        report = plot.read_json(target/"response_identification.json")
        report["complete"] = False
        fixtures.save(target/"response_identification.json", report)
        with self.assertRaisesRegex(ValueError, "incomplete/unbound"):
            plot.load_identification(target)


if __name__ == "__main__":
    unittest.main()
