"""A changed platform must not acquire the new run's provenance acceptance."""
from pathlib import Path
from tempfile import TemporaryDirectory
import json
import sys
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from simulations.x650_ts20_hold_v1 import launch_quiet_v3 as launch


class PlatformProvenanceTests(unittest.TestCase):
    def exercise(self, change):
        # The directory is created below this task's workspace, not the user's
        # Python installation or any existing repository checkout.
        parent = ROOT.parent / "tmp"
        parent.mkdir(exist_ok=True)
        with TemporaryDirectory(prefix="platform_provenance_", dir=parent) as temporary:
            root = Path(temporary)
            cfg = {"scenario": "fixture"}
            files = ["scripts/amsim", "scenarios/fixture.yaml", "src/amsim/a.py",
                     "simulations/x650_ts20_hold_v1/launch_quiet_v3.py", launch.SOFTWARE]
            for name in files:
                path = root / name
                path.parent.mkdir(parents=True, exist_ok=True)
                path.write_bytes(b"fixture\n")
            output = root / "launch"
            output.mkdir()
            run = root / "actual_run"
            expected = {"src/amsim/a.py": launch.task.sha(root / "src/amsim/a.py")}
            with patch.object(launch, "SOFTWARE_SHA256", launch.task.sha(root / launch.SOFTWARE)), \
                    patch.object(launch, "git_snapshot", return_value={"head_value": "fixture"}), \
                    patch.object(launch.task, "source_files_for_task", return_value=[root / "src/amsim/a.py"]):
                before = launch.capture_before(root, output, cfg, {"source_files_sha256": expected})
                if change == "edit":
                    (root / "src/amsim/a.py").write_bytes(b"changed implementation\n")
                elif change == "new_module":
                    (root / "src/amsim/b.py").write_bytes(b"new import candidate\n")
                after = launch.capture_after(root, output, cfg, before, [str(run)])
                (output / "launch.json").write_text(json.dumps({"complete": True, "runtime_closed": True,
                    "run_directories": [str(run)], "provenance": {"source_files_sha256": expected},
                    "platform_provenance": {"file": "platform_provenance.json",
                                            "sha256": launch.task.sha(output / "platform_provenance.json")}}))
                if change is None:
                    self.assertTrue(after["source_unchanged"])
                    self.assertTrue(launch.verify_platform(output, run)["verified"])
                    (output / "platform_sources.zip").write_bytes(b"replacement archive")
                    with self.assertRaisesRegex(RuntimeError, "artifact mismatch"):
                        launch.verify_platform(output, run)
                else:
                    self.assertFalse(after["source_unchanged"])
                    with self.assertRaisesRegex(RuntimeError, "not complete"):
                        launch.verify_platform(output, run)

    def test_archive_and_unchanged_sources_verify_and_tampering_does_not(self):
        self.exercise(None)

    def test_edited_source_cannot_pass(self):
        self.exercise("edit")

    def test_new_platform_module_cannot_pass(self):
        self.exercise("new_module")


if __name__ == "__main__":
    unittest.main()
