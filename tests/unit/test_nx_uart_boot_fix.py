"""Boot-config preservation/refusal tests; no hardware or sudo operations."""
import importlib.util
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch


ROOT = Path(__file__).resolve().parents[2]
SPEC = importlib.util.spec_from_file_location('uart_fix', ROOT / 'hardware/nx/apply_uart_fix.py')
fix = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(fix)
ORIGINAL = (b'TIMEOUT 30\nDEFAULT primary\n\nMENU TITLE L4T boot options\n\n'
            b'LABEL primary\n      MENU LABEL primary kernel\n'
            b'      LINUX /boot/Image\n      INITRD /boot/initrd\n'
            b'      APPEND ${cbootargs} root=PARTUUID=fixture rw quiet \n'
            b'\n# LABEL backup\n#    APPEND ${cbootargs}\n')


class BootFixTest(unittest.TestCase):
    def setUp(self):
        self.hash_patch = patch.object(fix, 'CONFIG_SHA', fix.sha(ORIGINAL))
        self.hash_patch.start()
        self.addCleanup(self.hash_patch.stop)

    def test_original_entry_and_boot_arguments_preserved(self):
        planned = fix.make_config(ORIGINAL)
        preserved = ORIGINAL.replace(b'DEFAULT primary\n', b'DEFAULT flyarm_uart1_iommu\n', 1)
        self.assertTrue(planned.startswith(preserved))
        self.assertEqual(planned.count(b'\nLABEL primary\n'), 1)
        self.assertEqual(planned.count(b'\nLABEL flyarm_uart1_iommu\n'), 1)
        for line in ORIGINAL.splitlines():
            if line.lstrip().startswith((b'LINUX ', b'INITRD ', b'APPEND ')):
                self.assertEqual(planned.splitlines().count(line), 2)
        self.assertEqual(planned.count(b'\n      FDT '), 1)
        self.assertIn(b'      FDT ' + str(fix.BASE_DTB).encode(), planned)
        self.assertNotIn(b'\n      FDT ', preserved)
        self.assertEqual(planned.count(b'OVERLAYS '), 1)
        self.assertNotIn(b'OVERLAYS ', preserved)

    def test_unreviewed_config_refused(self):
        with self.assertRaisesRegex(RuntimeError, 'differs'):
            fix.make_config(ORIGINAL.replace(b'quiet', b'debug'))

    def test_ambiguous_entry_refused_even_if_hash_reaudited(self):
        changed = ORIGINAL + b'\nLABEL another\n      LINUX /boot/Other\n'
        with patch.object(fix, 'CONFIG_SHA', fix.sha(changed)):
            with self.assertRaisesRegex(RuntimeError, 'exactly one'):
                fix.make_config(changed)

    def test_install_recognition_and_manual_edits_refused(self):
        with tempfile.TemporaryDirectory() as directory:
            config = Path(directory) / 'extlinux.conf'
            backup = Path(directory) / 'before.conf'
            config.write_bytes(ORIGINAL)
            with patch.object(fix, 'CONFIG', config), patch.object(fix, 'BACKUP', backup):
                self.assertEqual(fix.originals_and_state()[2], 'original')
                backup.write_bytes(ORIGINAL)
                config.write_bytes(fix.make_legacy_config(ORIGINAL))
                self.assertEqual(fix.originals_and_state()[2], 'legacy_overlay_only')
                config.write_bytes(fix.make_config(ORIGINAL))
                self.assertEqual(fix.originals_and_state()[2], 'installed')
                config.write_bytes(config.read_bytes() + b'# user edit\n')
                with self.assertRaisesRegex(RuntimeError, 'unrelated changes'):
                    fix.originals_and_state()
                config.write_bytes(ORIGINAL)
                backup.write_bytes(b'wrong backup')
                with self.assertRaisesRegex(RuntimeError, 'backup differs'):
                    fix.originals_and_state()

    def test_overlay_source_matches_audited_hash(self):
        self.assertEqual(fix.sha((ROOT / 'hardware/nx' / fix.SOURCE_NAME).read_bytes()), fix.SOURCE_SHA)


if __name__ == '__main__':
    unittest.main()
