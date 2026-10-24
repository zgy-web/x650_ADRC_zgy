import importlib.util
import json
from pathlib import Path
import tempfile
import unittest
import zipfile

SCRIPT=Path(__file__).resolve().parents[2]/'scripts/archive_x650_robustness_raw.py'
SPEC=importlib.util.spec_from_file_location('robust_archive',SCRIPT)
a=importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(a)


class RobustArchive(unittest.TestCase):
    def test_roundtrip_original_members_and_embedded_manifest(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp);out=root/'out';out.mkdir();p=root/'proof.json';p.write_bytes(b'{"pass": true}\n')
            receipt=a.build_archives([p],root=root,output_directory=out,prefix='raw',context={'test':True})
            self.assertTrue(receipt['complete']);self.assertEqual(receipt['verified_member_count'],1)
            manifest=(out/'raw_manifest.json').read_bytes()
            with zipfile.ZipFile(out/'raw.zip') as z:
                self.assertEqual(z.read('proof.json'),p.read_bytes())
                self.assertEqual(z.read('_archive_manifest.json'),manifest)
            with self.assertRaises(FileExistsError):
                a.build_archives([p],root=root,output_directory=out,prefix='raw',context={})

    def test_independently_readable_parts_under_cap(self):
        import random
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp);out=root/'out';out.mkdir();paths=[];rng=random.Random(20261024)
            for i in range(3):
                p=root/(str(i)+'.bin');p.write_bytes(rng.randbytes(24000));paths.append(p)
            receipt=a.build_archives(paths,root=root,output_directory=out,prefix='raw',context={},max_bytes=50000)
            self.assertGreater(len(receipt['archives']),1)
            for archive in receipt['archives']:
                self.assertLessEqual(archive['bytes'],50000)
                with zipfile.ZipFile(archive['path']) as z:self.assertIsNone(z.testzip())
            self.assertTrue(all(p.exists() for p in paths))

    def test_corrupt_member_rejected(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp);p=root/'data';p.write_bytes(b'original');item=a.snapshot_file(p,root)
            z=root/'bad.zip'
            with zipfile.ZipFile(z,'w') as archive:
                archive.writestr('_archive_manifest.json',b'{}');archive.writestr('data',b'changed')
            with self.assertRaises(RuntimeError):a.verify_archive(z,[item],b'{}')


if __name__=='__main__':unittest.main()
