"""Installer isolation and refusal checks; never starts PX4 or writes a real checkout."""
from __future__ import annotations

import hashlib
import importlib.util
from pathlib import Path
import subprocess
import tempfile
import unittest
from unittest.mock import patch


ROOT = Path(__file__).resolve().parents[2]
SPEC = importlib.util.spec_from_file_location(
    "x650_airframe_installer_under_test", ROOT / "scripts/install_x650_airframe.py")
installer = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(installer)


class X650AirframeInstallerTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory(prefix="x650_installer_")
        self.addCleanup(self.temporary.cleanup)
        self.folder = Path(self.temporary.name)
        self.px4 = self.folder / "PX4-Autopilot"
        self.directories = [
            self.px4 / "ROMFS/px4fmu_common/init.d-posix/airframes",
            self.px4 / "build/px4_sitl_default/etc/init.d-posix/airframes",
        ]
        for directory in self.directories:
            directory.mkdir(parents=True)
            (directory / "4001_gz_x500").write_bytes(b"unrelated airframe\n")
        self.sources = self.folder / "reviewed_sources"
        self.sources.mkdir()
        self.source = self.make_source("4998_gz_x650_arm_v1")
        self.git = patch.object(installer.subprocess, "run", return_value=(
            subprocess.CompletedProcess(["git"], 0, installer.PINNED_PX4 + "\n", "")))
        self.git_mock = self.git.start()
        self.addCleanup(self.git.stop)

    def make_source(self, name, content=b"#!/bin/sh\nparam set CA_ROTOR_COUNT 4\n"):
        source = self.sources / name
        source.write_bytes(content)
        return source

    def snapshot(self):
        return {str(path.relative_to(self.px4)): path.read_bytes()
                for path in self.px4.rglob("*") if path.is_file()}

    def assert_refused_without_writes(self, source=None, message=None):
        before = self.snapshot()
        with self.assertRaisesRegex(ValueError, message or ".*"):
            installer.install(self.px4, self.source if source is None else source)
        self.assertEqual(self.snapshot(), before)

    def test_different_ids_coexist_in_source_and_built_romfs(self):
        old_source = self.make_source("4999_gz_x650_no_arm_v1", b"old reviewed airframe\n")
        old_report = installer.install(self.px4, old_source)
        old_snapshot = self.snapshot()
        new_report = installer.install(self.px4, self.source)
        self.assertEqual(old_report["airframe_id"], 4999)
        self.assertEqual(new_report["airframe_id"], 4998)
        self.assertFalse(new_report["flight_started"])
        self.assertEqual(new_report["px4_commit"], installer.PINNED_PX4)
        self.assertEqual(len(new_report["installed_paths"]), 2)
        for directory in self.directories:
            self.assertEqual((directory / self.source.name).read_bytes(), self.source.read_bytes())
        after = self.snapshot()
        for name, content in old_snapshot.items():
            self.assertEqual(after[name], content)

    def test_same_id_different_name_is_rejected_before_any_write(self):
        # A conflict in the second destination must also prevent the first write.
        for conflict_index in (0, 1):
            with self.subTest(conflict_index=conflict_index):
                conflict = self.directories[conflict_index] / "4998_unrelated_name"
                conflict.write_bytes(b"preserve existing owner\n")
                try:
                    self.assert_refused_without_writes(message="Airframe ID 4998 already used")
                    for directory in self.directories:
                        self.assertFalse((directory / self.source.name).exists())
                finally:
                    conflict.unlink()

    def test_same_name_different_content_is_rejected_before_any_write(self):
        for conflict_index in (0, 1):
            with self.subTest(conflict_index=conflict_index):
                conflict = self.directories[conflict_index] / self.source.name
                conflict.write_bytes(b"previous different configuration\n")
                try:
                    self.assert_refused_without_writes(message="Refusing to overwrite different existing airframe")
                finally:
                    conflict.unlink()

    def test_missing_build_directory_does_not_write_romfs(self):
        build_directory = self.directories[1]
        (build_directory / "4001_gz_x500").unlink()
        build_directory.rmdir()
        self.assert_refused_without_writes(message="Missing or external airframe directory")
        self.assertFalse((self.directories[0] / self.source.name).exists())

    def test_external_symlink_airframe_directory_is_rejected(self):
        external = self.folder / "outside_px4"
        external.mkdir()
        sentinel = external / "external_sentinel"
        sentinel.write_bytes(b"never modify an external directory\n")
        build_directory = self.directories[1]
        (build_directory / "4001_gz_x500").unlink()
        build_directory.rmdir()
        try:
            build_directory.symlink_to(external, target_is_directory=True)
        except OSError as exc:
            self.skipTest(f"This host cannot create directory symlinks: {exc}")
        self.assert_refused_without_writes(message="Missing or external airframe directory")
        self.assertEqual(list(external.iterdir()), [sentinel])
        self.assertEqual(sentinel.read_bytes(), b"never modify an external directory\n")

    def test_external_directory_resolution_is_rejected_on_hosts_without_symlink_rights(self):
        # Exercise the confinement check on every platform. The native link
        # tests above/below additionally cover actual filesystem symlinks when
        # the host allows them. This models the resolved path of a link, without
        # bypassing the production is_dir()/is_relative_to() checks.
        external = self.folder / "outside_resolved_directory"
        external.mkdir()
        directory_path = self.directories[1]
        original_resolve = Path.resolve

        def resolved_path(path, strict=False):
            return external if path == directory_path else original_resolve(path, strict=strict)

        with patch.object(Path, "resolve", resolved_path):
            self.assert_refused_without_writes(message="Missing or external airframe directory")

    def test_existing_target_symlink_is_rejected_even_for_identical_bytes(self):
        external_file = self.folder / "outside_target"
        external_file.write_bytes(self.source.read_bytes())
        target = self.directories[1] / self.source.name
        try:
            target.symlink_to(external_file)
        except OSError as exc:
            self.skipTest(f"This host cannot create file symlinks: {exc}")
        self.assert_refused_without_writes(message="Refusing to overwrite different existing airframe")
        self.assertEqual(external_file.read_bytes(), self.source.read_bytes())

    def test_changed_pinned_commit_is_rejected_before_any_write(self):
        self.git_mock.return_value = subprocess.CompletedProcess(["git"], 0, "0" * 40 + "\n", "")
        self.assert_refused_without_writes(message="PX4 commit changed")

    def test_default_behavior_installs_reviewed_4999_and_is_idempotent(self):
        content = installer.SOURCE.read_text(encoding="utf-8").replace("\r\n", "\n").encode()
        result = installer.install(self.px4)
        first = self.snapshot()
        again = installer.install(self.px4)
        self.assertEqual(self.snapshot(), first)
        self.assertEqual(again, result)
        self.assertEqual(result["airframe_id"], 4999)
        self.assertEqual(result["airframe_sha256"], hashlib.sha256(content).hexdigest())
        for directory in self.directories:
            self.assertEqual((directory / installer.NAME).read_bytes(), content)
        args, kwargs = self.git_mock.call_args
        self.assertEqual(args[0], ["git", "rev-parse", "HEAD"])
        self.assertEqual(kwargs["cwd"], self.px4.resolve())
        self.assertTrue(kwargs["check"])

    def test_explicit_source_normalizes_crlf_before_hash_and_write(self):
        source = self.make_source("4998_gz_x650_arm_crlf", b"#!/bin/sh\r\nparam set TEST 1\r\n")
        expected = b"#!/bin/sh\nparam set TEST 1\n"
        result = installer.install(self.px4, source)
        self.assertEqual(result["airframe_sha256"], hashlib.sha256(expected).hexdigest())
        for directory in self.directories:
            self.assertEqual((directory / source.name).read_bytes(), expected)

    def test_invalid_id_filename_is_rejected_without_writes(self):
        for name in ("0_gz_invalid", "not_a_numeric_id", "-4998_gz_invalid", "4998_bad-name"):
            with self.subTest(name=name):
                source = self.make_source(name)
                self.assert_refused_without_writes(source, "positive numeric ID")


if __name__ == "__main__":
    unittest.main()
