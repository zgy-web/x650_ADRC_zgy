import importlib.util
import json
from pathlib import Path
import tempfile
import unittest

SCRIPT=Path(__file__).resolve().parents[2]/'scripts/archive_x650_robustness_delta.py'
SPEC=importlib.util.spec_from_file_location('robust_delta_archive',SCRIPT)
a=importlib.util.module_from_spec(SPEC);SPEC.loader.exec_module(a)


class DeltaArchive(unittest.TestCase):
    def test_marker_requires_explicit_closed_and_contained_paths(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp);(root/'result.json').write_text('{}');marker=root/'ready.json'
            for document in ({'complete':False,'paths':['result.json']},
                             {'complete':True,'paths':['../result.json']},
                             {'complete':True,'paths':['result.json','result.json']},
                             {'complete':True,'paths':[]}):
                marker.write_text(json.dumps(document))
                with self.assertRaises(ValueError):a.marker_paths(root,marker)
            marker.write_text(json.dumps({'complete':True,'paths':['result.json']}))
            _,paths=a.marker_paths(root,marker)
            self.assertEqual(paths,[root/'result.json'])

    def test_delta_excludes_old_archives_and_generated_mesh_cache(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp);folder=root/'plots_v1';folder.mkdir()
            (folder/'curve.png').write_bytes(b'figure')
            (folder/'raw_v1.zip').write_bytes(b'old raw')
            cache=folder/'geometry_cache';cache.mkdir();(cache/'mesh.bin').write_bytes(b'cache')
            files,excluded=a.selected_files(root,[folder])
            self.assertEqual(files,[folder/'curve.png']);self.assertEqual(len(excluded),2)


if __name__=='__main__':unittest.main()
