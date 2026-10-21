"""Imported model resolution, deployment ownership and execution boundaries."""
from __future__ import annotations

import copy
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import types
import unittest
from unittest.mock import patch
import xml.etree.ElementTree as ET

import yaml

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "src"))

from amsim.modeling import manager
from amsim.modeling.urdf_flight import bind_flight_profile


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


class ImportedManagerTests(unittest.TestCase):
    def setUp(self):
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        self.root = Path(temporary.name)
        self.workspace = self.root / "project"
        self.source = self.workspace / "generated/imported_models/amsim_urdf_test"
        self.source.mkdir(parents=True)
        self.px4 = self.root / "PX4-Autopilot"
        self.airframes = self.px4 / "build/px4_sitl_default/etc/init.d-posix/airframes"
        self.airframes.mkdir(parents=True)
        (self.airframes / "4999_test").write_text("# unit fixture\n", encoding="utf-8")
        self.addCleanup(patch.stopall)
        patch.object(manager, "PROJECT_ROOT", self.workspace).start()
        patch.object(manager, "px4_dir", return_value=self.px4).start()
        self._write_import()

    def _write_import(self):
        self.profile = {
            "schema_version": 1, "type": "px4_multicopter", "body_frame": "FLU",
            "base_link": "base_link", "px4_sys_autostart": 4999,
            "controlled_joints": [], "rotors": [], "notes": "Synthetic integration fixture; not calibrated.",
        }
        root = ET.Element("sdf", version="1.9")
        model = ET.SubElement(root, "model", name=self.source.name)
        base = ET.SubElement(model, "link", name="base_link")
        ET.SubElement(ET.SubElement(base, "inertial"), "mass").text = "2.0"
        joints = []
        for i in range(4):
            name, link_name = f"spin_{i}", f"prop_{i}"
            link = ET.SubElement(model, "link", name=link_name)
            ET.SubElement(ET.SubElement(link, "inertial"), "mass").text = "0.1"
            joint = ET.SubElement(model, "joint", name=name, type="revolute")
            ET.SubElement(joint, "parent").text = "base_link"
            ET.SubElement(joint, "child").text = link_name
            axis = ET.SubElement(joint, "axis")
            ET.SubElement(axis, "xyz").text = "0 0 1"
            limits = ET.SubElement(axis, "limit")
            ET.SubElement(limits, "lower").text = "-1e16"
            ET.SubElement(limits, "upper").text = "1e16"
            joints.append({"name": name, "type": "continuous", "parent": "base_link", "child": link_name, "axis_in_root_at_zero": [0, 0, 1]})
            self.profile["rotors"].append({
                "motor_number": i, "joint": name, "link": link_name,
                "turning_direction": "cw" if i % 2 else "ccw",
                "motor_constant": 1.2e-5, "moment_constant": .027,
                "max_rot_velocity": 833., "time_constant_up": .013,
                "time_constant_down": .031, "rotor_drag_coefficient": 9.2e-5,
                "rolling_moment_coefficient": 1.2e-6, "rotor_velocity_slowdown_sim": 7.,
            })
        ET.ElementTree(root).write(self.source / "model.sdf", encoding="utf-8")
        (self.source / "model.config").write_text(f"<model><name>{self.source.name}</name><sdf version='1.9'>model.sdf</sdf></model>", encoding="utf-8")
        (self.source / "model.urdf").write_text("<robot name='test'/>", encoding="utf-8")
        (self.source / "source").mkdir()
        (self.source / "source/original.urdf").write_text("<robot name='original'/>", encoding="utf-8")
        (self.source / "meshes").mkdir()
        (self.source / "meshes/fixture.stl").write_text("synthetic resource", encoding="utf-8")
        self.manifest = {
            "schema_version": 1, "kind": "urdf_import", "model_name": self.source.name,
            "flight_ready": False, "root_link": "base_link", "total_mass_kg": 2.4,
            "links": [{"name": "base_link"}], "joints": joints,
            "artifacts": {p.relative_to(self.source).as_posix(): sha(p) for p in self.source.rglob("*") if p.is_file()},
        }
        self._save_manifest()

    def _save_manifest(self):
        (self.source / "import_manifest.json").write_text(json.dumps(self.manifest), encoding="utf-8")

    def flight(self):
        profile = self.root / "profile.yaml"
        profile.write_text(yaml.safe_dump(self.profile), encoding="utf-8")
        with patch("amsim.modeling.urdf_flight.subprocess.run", return_value=subprocess.CompletedProcess([], 0, "Valid.\n", "")):
            bound = bind_flight_profile(self.source, profile)
        return manager.resolve(str(bound.output_dir))

    def test_import_resolves_by_name_directory_and_manifest_without_rotor_commands(self):
        by_name = manager.resolve(self.source.name)
        by_directory = manager.resolve(str(self.source))
        by_manifest = manager.resolve(str(self.source / "import_manifest.json"))
        for spec in (by_name, by_directory, by_manifest):
            self.assertEqual(spec.source, "urdf_import")
            self.assertFalse(spec.flight_ready)
            self.assertEqual(spec.joints, [])
            self.assertAlmostEqual(spec.total_mass_kg, 2.4)
        self.assertTrue(any(row["source"] == "urdf_import" for row in manager.inventory()))

    def test_all_hashes_checked_including_mesh_and_untracked_files(self):
        (self.source / "meshes/fixture.stl").write_text("changed", encoding="utf-8")
        with self.assertRaisesRegex(ValueError, "hash mismatch"):
            manager.resolve(self.source.name)
        (self.source / "meshes/fixture.stl").write_text("synthetic resource", encoding="utf-8")
        (self.source / "unknown.txt").write_text("not tracked", encoding="utf-8")
        with self.assertRaisesRegex(ValueError, "untracked"):
            manager.resolve(self.source.name)

    def test_path_traversal_and_manifest_runtime_mismatch_rejected(self):
        original = copy.deepcopy(self.manifest)
        self.manifest["artifacts"]["../outside"] = "a" * 64
        self._save_manifest()
        with self.assertRaises(ValueError):
            manager.resolve(self.source.name)
        self.manifest = original
        self.manifest["flight_ready"] = True
        self._save_manifest()
        with self.assertRaisesRegex(ValueError, "flight_ready"):
            manager.resolve(self.source.name)

    def test_preview_rejected_by_environment_install_run_and_hil(self):
        from amsim import run_manager
        from amsim.hil import process, supervisor
        spec = manager.resolve(self.source.name)
        for action in (manager.env_for, manager.install):
            with self.assertRaisesRegex(RuntimeError, "no flight profile"):
                action(spec)
        scenario = types.SimpleNamespace(config_path=self.root / "unused.yaml", config={"model": self.source.name})
        with patch.object(run_manager, "get_scenario", return_value=scenario), patch.object(run_manager, "resolve_model", return_value=spec), patch.object(run_manager, "install_model") as install, patch.object(run_manager, "runtime_start") as start:
            with self.assertRaisesRegex(RuntimeError, "no flight profile"):
                run_manager.run("test")
            install.assert_not_called()
            start.assert_not_called()
        with patch.object(manager, "resolve", return_value=spec), patch.object(process.subprocess, "Popen") as start:
            with self.assertRaisesRegex(RuntimeError, "HIL"):
                process.start("unused")
            start.assert_not_called()
        with patch.object(supervisor, "resolve", return_value=spec), patch.object(sys, "argv", ["hil", "--mavlink", "unused"]), patch.object(supervisor, "install") as install:
            with self.assertRaisesRegex(RuntimeError, "HIL"):
                supervisor.main()
            install.assert_not_called()

    def test_bound_model_environment_and_airframe_are_explicit(self):
        spec = self.flight()
        env = manager.env_for(spec)
        self.assertEqual(spec.source, "urdf_flight")
        self.assertTrue(spec.flight_ready)
        self.assertEqual(env["AMSIM_PX4_SYS_AUTOSTART"], "4999")
        self.assertEqual(env["AMSIM_MODEL_JOINTS"], "[]")
        self.assertEqual(env["AMSIM_MODEL_FLIGHT_READY"], "1")
        (self.airframes / "4999_test").unlink()
        with self.assertRaisesRegex(RuntimeError, "0 built runtime scripts"):
            manager.env_for(spec)
        # A source-only ROMFS file is not sufficient for the built SITL runtime.
        source_airframes = self.px4 / "ROMFS/px4fmu_common/init.d-posix/airframes"
        source_airframes.mkdir(parents=True)
        (source_airframes / "4999_test").write_text("source only", encoding="utf-8")
        with self.assertRaises(RuntimeError):
            manager.install(spec)

    def test_legacy_generated_model_retains_airframe_layout_compatibility(self):
        source = self.root / "x500_amsim_legacy"
        source.mkdir()
        sdf = source / "model.sdf"
        sdf.write_text("<sdf version='1.9'><model name='x500_amsim_legacy'/></sdf>", encoding="utf-8")
        manifest = source / "model_manifest.json"
        manifest.write_text(json.dumps({"model_name": source.name, "model_sha256": sha(sdf)}), encoding="utf-8")
        spec = manager.ModelSpec(ref="legacy", model_name=source.name, sdf_path=sdf, sha256=sha(sdf), joints=["elbow"], source="generated", manifest_path=manifest)
        self.assertIsNone(manager.validate_px4_airframe(spec))
        self.assertEqual(manager.env_for(spec)["AMSIM_PX4_SYS_AUTOSTART"], "4001")
        with patch.object(manager, "validate_sdf", return_value=(True, "Valid.")):
            target = manager.install(spec)
        self.assertTrue((target / "model.sdf").is_file())

    def test_direct_supervisor_checks_installed_manifest_before_children(self):
        from amsim.runtime import sitl_supervisor
        spec = self.flight()
        env = manager.env_for(spec)
        with patch.object(manager, "validate_sdf", return_value=(True, "Valid.")):
            target = manager.install(spec)
        binary = self.px4 / "build/px4_sitl_default/bin/px4"
        binary.parent.mkdir(parents=True)
        binary.write_text("unit fixture", encoding="utf-8")
        with patch.dict("os.environ", env, clear=True), patch.object(sitl_supervisor, "px4_dir", return_value=self.px4), patch.object(sitl_supervisor.subprocess, "Popen") as start:
            (target / "meshes/fixture.stl").write_text("tampered", encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "hash mismatch"):
                sitl_supervisor.main()
            start.assert_not_called()
            (target / "meshes/fixture.stl").write_text("synthetic resource", encoding="utf-8")
            (self.airframes / "4999_test").unlink()
            with self.assertRaisesRegex(RuntimeError, "0 built runtime scripts"):
                sitl_supervisor.main()
            start.assert_not_called()
            (self.airframes / "4999_test").write_text("# restored fixture", encoding="utf-8")
            (target / "import_manifest.json").unlink()
            with self.assertRaisesRegex(FileNotFoundError, "Expected imported model manifest"):
                sitl_supervisor.main()
            start.assert_not_called()

    def test_install_copies_resources_and_tracks_ownership(self):
        spec = self.flight()
        with patch.object(manager, "validate_sdf", return_value=(True, "Valid.")):
            target = manager.install(spec)
            self.assertEqual((target / "meshes/fixture.stl").read_text(), "synthetic resource")
            self.assertEqual(json.loads((target / manager.INSTALL_MARKER).read_text())["owner"], "amsim.modeling.manager")
            self.assertEqual(manager.install(spec), target)
            (target / "meshes/fixture.stl").write_text("local edit", encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "hash mismatch"):
                manager.install(spec)
            self.assertEqual((target / "meshes/fixture.stl").read_text(), "local edit")

    def test_unowned_install_target_is_preserved(self):
        spec = self.flight()
        target = self.px4 / "Tools/simulation/gz/models" / spec.model_name
        target.mkdir(parents=True)
        (target / "keep.txt").write_text("user content", encoding="utf-8")
        with patch.object(manager, "validate_sdf", return_value=(True, "Valid.")):
            with self.assertRaisesRegex(RuntimeError, "unowned"):
                manager.install(spec)
        self.assertEqual((target / "keep.txt").read_text(), "user content")

    def test_run_snapshot_keeps_provenance_without_mesh_duplication(self):
        from amsim.run_manager import _snapshot_inputs
        spec = self.flight()
        config = self.root / "scenario.yaml"
        config.write_text("name: unit\n", encoding="utf-8")
        scenario = types.SimpleNamespace(config_path=config, module="amsim.scenarios.test")
        folder = self.root / "run"
        folder.mkdir()
        _snapshot_inputs(folder, scenario, spec)
        self.assertTrue((folder / "model_manifest.json").is_file())
        self.assertTrue((folder / "flight_profile.yaml").is_file())
        self.assertTrue((folder / "source_import_manifest.json").is_file())
        self.assertTrue((folder / "model_snapshot.sdf").is_file())
        self.assertFalse((folder / "meshes").exists())


if __name__ == "__main__":
    unittest.main()
