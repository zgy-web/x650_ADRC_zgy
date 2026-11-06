"""仅测本地配置生成与拒绝逻辑，不打开设备、不部署、不重启。"""
import importlib.util
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch


ROOT = Path(__file__).resolve().parents[2] / 'hardware/nx/orinnx_20261106'
FIXTURES = Path(__file__).resolve().parents[1] / 'fixtures/nx_uart_new_board'
SPEC = importlib.util.spec_from_file_location('new_nx_uart_fix', ROOT / 'apply_uart_fix.py')
fix = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(fix)
ORIGINAL = (FIXTURES / 'extlinux.fixture.conf').read_bytes()


class CandidateTests(unittest.TestCase):
    def setUp(self):
        self.config_patch = patch.object(fix, 'CONFIG_SHA', fix.sha(ORIGINAL))
        self.config_patch.start()
        self.addCleanup(self.config_patch.stop)

    def temporary_state(self, directory):
        config = Path(directory) / 'extlinux.conf'
        backup = Path(directory) / 'before.conf'
        config.write_bytes(ORIGINAL)
        return config, backup

    def test_reviewed_fixture_and_source_hashes(self):
        self.assertEqual(fix.sha(ORIGINAL), fix.CONFIG_SHA)
        self.assertEqual(fix.sha((ROOT / fix.SOURCE_NAME).read_bytes()), fix.SOURCE_SHA)
        self.assertEqual(fix.make_config(ORIGINAL), (FIXTURES / 'planned.fixture.conf').read_bytes())

    def test_original_primary_and_append_are_byte_preserved(self):
        planned = fix.make_config(ORIGINAL)
        expected_prefix = ORIGINAL.replace(b'DEFAULT primary\n',
                                           b'DEFAULT ' + fix.LABEL.encode() + b'\n', 1)
        self.assertTrue(planned.startswith(expected_prefix))
        original_primary = ORIGINAL[ORIGINAL.index(b'LABEL primary\n'):]
        self.assertIn(original_primary, planned)
        for line in ORIGINAL.splitlines(keepends=True):
            if line.startswith((b'      LINUX ', b'      INITRD ', b'      APPEND ')):
                self.assertEqual(planned.count(line), 2)
        append = next(line for line in ORIGINAL.splitlines(keepends=True)
                      if line.startswith(b'      APPEND '))
        self.assertTrue(append.endswith(b'nv-auto-config \n'))
        self.assertEqual(planned.splitlines(keepends=True).count(append), 2)

    def test_explicit_linux_fdt_and_overlay_only_in_new_entry(self):
        planned = fix.make_config(ORIGINAL)
        prefix, entry = planned.split(b'\nLABEL ' + fix.LABEL.encode() + b'\n')
        self.assertNotIn(b'      FDT ', prefix)
        self.assertNotIn(b'      OVERLAYS ', prefix)
        self.assertEqual(entry.count(b'      FDT '), 1)
        self.assertEqual(entry.count(b'      OVERLAYS '), 1)
        self.assertIn(b'      FDT /boot/dtb/kernel_tegra234-p3768-0000+p3767-0000-nv.dtb\n', entry)
        self.assertIn(b'      OVERLAYS /boot/dtb/flyarm-uart1-iommu-r365-20261106.dtbo\n', entry)
        self.assertNotIn(b'\\', entry)

    def test_hash_change_refused(self):
        with self.assertRaisesRegex(RuntimeError, 'differs from reviewed'):
            fix.make_config(ORIGINAL.replace(b'nv-auto-config', b'changed-config'))

    def test_extra_active_entry_refused_even_when_hash_reaudited(self):
        changed = ORIGINAL + b'\nLABEL unexpected\n      LINUX /boot/Other\n'
        with patch.object(fix, 'CONFIG_SHA', fix.sha(changed)):
            with self.assertRaisesRegex(RuntimeError, 'exactly one'):
                fix.make_config(changed)

    def test_original_and_installed_states_recognized(self):
        with tempfile.TemporaryDirectory(dir=ROOT) as directory:
            config, backup = self.temporary_state(directory)
            with patch.object(fix, 'CONFIG', config), patch.object(fix, 'BACKUP', backup):
                self.assertEqual(fix.originals_and_state(), (ORIGINAL, ORIGINAL, 'original'))
                backup.write_bytes(ORIGINAL)
                installed = fix.make_config(ORIGINAL)
                config.write_bytes(installed)
                self.assertEqual(fix.originals_and_state(), (ORIGINAL, installed, 'installed'))

    def test_bad_backup_refused_in_both_states(self):
        with tempfile.TemporaryDirectory(dir=ROOT) as directory:
            config, backup = self.temporary_state(directory)
            with patch.object(fix, 'CONFIG', config), patch.object(fix, 'BACKUP', backup):
                backup.write_bytes(b'wrong backup')
                with self.assertRaisesRegex(RuntimeError, 'backup differs'):
                    fix.originals_and_state()
                config.write_bytes(fix.make_config(ORIGINAL))
                with self.assertRaisesRegex(RuntimeError, 'Backup hash differs'):
                    fix.originals_and_state()

    def test_user_config_edits_and_overlay_only_legacy_refused(self):
        with tempfile.TemporaryDirectory(dir=ROOT) as directory:
            config, backup = self.temporary_state(directory)
            with patch.object(fix, 'CONFIG', config), patch.object(fix, 'BACKUP', backup):
                backup.write_bytes(ORIGINAL)
                installed = fix.make_config(ORIGINAL)
                for changed in (installed + b'# user edit\n',
                                installed.replace(b'      FDT ' + fix.BASE_DTB_TEXT.encode() + b'\n', b'')):
                    config.write_bytes(changed)
                    with self.assertRaisesRegex(RuntimeError, 'unrelated changes'):
                        fix.originals_and_state()

    def test_rollback_selection_is_idempotent_and_does_not_write(self):
        with tempfile.TemporaryDirectory(dir=ROOT) as directory:
            config, backup = self.temporary_state(directory)
            with patch.object(fix, 'CONFIG', config), patch.object(fix, 'BACKUP', backup):
                self.assertEqual(fix.rollback_config(), (ORIGINAL, ORIGINAL, 'original'))
                backup.write_bytes(ORIGINAL)
                installed = fix.make_config(ORIGINAL)
                config.write_bytes(installed)
                self.assertEqual(fix.rollback_config(), (ORIGINAL, installed, 'installed'))
                self.assertEqual(config.read_bytes(), installed)
                self.assertEqual(backup.read_bytes(), ORIGINAL)
                config.write_bytes(ORIGINAL)
                self.assertEqual(fix.rollback_config(), (ORIGINAL, ORIGINAL, 'original'))

    def test_rollback_refuses_unrelated_changes(self):
        with tempfile.TemporaryDirectory(dir=ROOT) as directory:
            config, backup = self.temporary_state(directory)
            with patch.object(fix, 'CONFIG', config), patch.object(fix, 'BACKUP', backup):
                backup.write_bytes(ORIGINAL)
                config.write_bytes(fix.make_config(ORIGINAL) + b'# new settings\n')
                with self.assertRaisesRegex(RuntimeError, 'unrelated changes'):
                    fix.rollback_config()


if __name__ == '__main__':
    unittest.main(verbosity=2)
