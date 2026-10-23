"""Exclusive startup provenance, with an explicitly fake native handle."""
import hashlib
import json
from pathlib import Path
import sys
from types import SimpleNamespace as NS
import unittest
from unittest.mock import patch
import uuid
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from flyarm_mpc_reference.startup import write_startup_metadata


class StartupTests(unittest.TestCase):
    def test_installed_sources_and_loaded_identity_are_recorded_without_overwrite(self):
        directory = Path(__file__).resolve().parent / ("_startup_fixture_"+uuid.uuid4().hex)
        directory.mkdir()
        def cleanup():
            for item in directory.iterdir():
                item.unlink()
            directory.rmdir()
        self.addCleanup(cleanup)
        native = directory/"unit_fake_native.so"
        native.write_bytes(b"unit placeholder: never loaded or executed")
        sha = hashlib.sha256(native.read_bytes()).hexdigest()
        predictor = NS(_native=NS(_name=str(native)),
                       metadata=lambda: {"native_accelerator": {"path": str(native), "sha256": sha}})
        fake = NS(adapter=NS(allocator=NS(actuator_predictor=predictor)),
                  metadata=lambda: {"repo_root": str(Path(__file__).resolve().parents[4]),
                                    "synthetic_software_fixture_only": True})
        output = directory/"startup.json"
        # No SciPy is needed for this unit fake; the real factory imports it.
        with patch.dict(sys.modules, {"scipy": NS(__file__=__file__, __version__="unit_fake")}):
            result = write_startup_metadata(output, fake)
            raw = output.read_bytes()
            self.assertEqual(result["actually_loaded_native_library"]["sha256"], sha)
            self.assertIn("flyarm_mpc_reference.startup", result["loaded_module_sources"])
            self.assertFalse(json.loads(raw)["native_mpc"])
            with self.assertRaises(FileExistsError):
                write_startup_metadata(output, fake)
            self.assertEqual(output.read_bytes(), raw)
            fake.adapter.allocator.native_optimization = NS(lib=NS(_name=str(native)),
                receipt={"path":str(native),"sha256":sha})
            selected = write_startup_metadata(directory/"native_startup.json", fake)
            self.assertEqual(selected["actually_loaded_native_optimization"]["sha256"],sha)
            self.assertTrue(selected["native_numerical_kernels"])
            self.assertFalse(selected["native_mpc"])
            fake.adapter.allocator.native_optimization.receipt["sha256"] = "0"*64
            with self.assertRaisesRegex(ValueError,"optimization differs"):
                write_startup_metadata(directory/"invalid_startup.json",fake)
            self.assertFalse((directory/"invalid_startup.json").exists())


if __name__ == "__main__":
    unittest.main()
