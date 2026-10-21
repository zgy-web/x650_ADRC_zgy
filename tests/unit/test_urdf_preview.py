from __future__ import annotations

import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest import mock
import xml.etree.ElementTree as ET

sys.path.insert(0,str(Path(__file__).resolve().parents[2]/"src"))
from amsim.modeling import urdf_preview as preview


SDF = '''<sdf version="1.9"><model name="amsim_urdf_test">
<link name="base_link"><inertial><pose>0.01 0.02 -0.08 0 0 0</pose><mass>3.02</mass>
<inertia><ixx>0.1</ixx><iyy>0.1</iyy><izz>0.12</izz></inertia></inertial>
<visual name="body"><geometry><mesh><uri>model://amsim_urdf_test/meshes/part.stl</uri><scale>0.001 0.001 0.001</scale></mesh></geometry></visual>
<collision name="body"><geometry><mesh><uri>model://amsim_urdf_test/meshes/part.stl</uri><scale>0.001 0.001 0.001</scale></mesh></geometry></collision>
</link></model></sdf>'''


class PreviewTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory(prefix="amsim-test-preview-")
        self.root = Path(self.tmp.name)
        self.model = self.root/"models"/"amsim_urdf_test"
        (self.model/"meshes").mkdir(parents=True)
        (self.model/"meshes"/"part.stl").write_text("solid fixture\nendsolid fixture\n")
        (self.model/"model.sdf").write_text(SDF)
        self.out = self.root/"preview"

    def tearDown(self):
        self.tmp.cleanup()

    def _fake_server(self, log=b"Gazebo Server started and completed its iterations.\n", exit_code=0):
        def start(*args,**kwargs):
            kwargs["stdout"].write(log)
            process=mock.Mock()
            process.pid=17291
            process.wait.return_value=exit_code
            process.poll.return_value=exit_code
            process.returncode=exit_code
            return process
        return start

    def test_dynamic_world_preserves_model_and_makes_meshes_self_resolving(self):
        original=(self.model/"model.sdf").read_bytes()
        world=preview.build_preview(self.model,self.out,height=.5)
        self.assertEqual((self.model/"model.sdf").read_bytes(),original)
        scene=ET.parse(world).getroot().find("world")
        self.assertEqual(scene.find("include/static").text,"false")
        self.assertEqual(scene.find("include/pose").text,"0 0 0.5 0 0 0")
        self.assertEqual({p.get("name") for p in scene.findall("plugin")},{"gz::sim::systems::Physics","gz::sim::systems::UserCommands","gz::sim::systems::SceneBroadcaster"})
        self.assertIsNotNone(scene.find("model/link/collision/geometry/plane"))
        self.assertEqual(scene.find("include/uri").text,(self.out/"preview_model.sdf").as_uri())
        model=ET.parse(self.out/"preview_model.sdf").getroot()
        for uri in model.findall(".//mesh/uri"):
            self.assertEqual(uri.text,(self.model/"meshes"/"part.stl").as_uri())
        # URI normalization must not alter mass, inertia, scale, collision or pose.
        for node in model.findall(".//mesh/uri"):
            node.text="model://amsim_urdf_test/meshes/part.stl"
        def canonical(node):
            return (node.tag,node.attrib,(node.text or "").strip(),tuple(canonical(c) for c in node))
        self.assertEqual(canonical(model),canonical(ET.fromstring(SDF)))
        report=json.loads((self.out/"preview_manifest.json").read_text())
        self.assertTrue(report["dynamic"])
        self.assertFalse(report["px4_started"])
        self.assertFalse(report["flight_validated"])
        self.assertEqual(report["gui_command"],["env",f"GZ_PARTITION={report['partition']}","gz","sim",str(world)])

    def test_interactive_previews_each_have_isolated_partitions(self):
        first=preview.build_preview(self.model,self.out)
        second=preview.build_preview(self.model,self.root/"second-preview")
        first_metadata=json.loads((first.parent/"preview_manifest.json").read_text())
        second_metadata=json.loads((second.parent/"preview_manifest.json").read_text())
        self.assertNotEqual(first_metadata["partition"],second_metadata["partition"])
        for metadata in (first_metadata,second_metadata):
            self.assertRegex(metadata["partition"],r"^amsim-urdf-preview-[0-9a-f]{32}$")
            self.assertEqual(metadata["gui_command"][1],"GZ_PARTITION="+metadata["partition"])

    def test_static_preview_is_explicit_and_output_is_not_overwritten(self):
        world=preview.build_preview(self.model,self.out,static=True)
        self.assertEqual(ET.parse(world).find("world/include/static").text,"true")
        with self.assertRaisesRegex(preview.UrdfPreviewError,"overwrite"):
            preview.build_preview(self.model,self.out)
        with self.assertRaisesRegex(preview.UrdfPreviewError,"separate"):
            preview.build_preview(self.model,self.model)

    def test_resource_path_escape_missing_resource_and_bad_height_rejected(self):
        for height in (-1,float("nan"),float("inf")):
            with self.subTest(height=height),self.assertRaises(preview.UrdfPreviewError):
                preview.build_preview(self.model,self.out,height=height)
        (self.model/"model.sdf").write_text(SDF.replace("meshes/part.stl","../outside.stl"))
        with self.assertRaisesRegex(preview.UrdfPreviewError,"escapes"):
            preview.build_preview(self.model,self.out)
        (self.model/"model.sdf").write_text(SDF.replace("meshes/part.stl","meshes/missing.stl"))
        with self.assertRaises(FileNotFoundError):
            preview.build_preview(self.model,self.out)
        self.assertFalse(self.out.exists())

    def test_bounded_check_is_partitioned_and_records_success_without_flight_claim(self):
        world=preview.build_preview(self.model,self.out)
        environment={"GZ_PARTITION":"existing-user-simulation","GZ_SIM_RESOURCE_PATH":"existing-assets"}
        with mock.patch.dict(preview.os.environ,environment),\
             mock.patch.object(preview.subprocess,"run",return_value=subprocess.CompletedProcess([],0,"Valid.\n","")) as validate,\
             mock.patch.object(preview.subprocess,"Popen",side_effect=self._fake_server()) as start:
            result=preview.check_preview(world,iterations=100,timeout_s=30)
            self.assertEqual(preview.os.environ["GZ_PARTITION"],"existing-user-simulation")
        self.assertTrue(result["ok"])
        self.assertFalse(result["flight_validated"])
        cmd=start.call_args.args[0]
        self.assertEqual(cmd[:3],["gz","sim","-s"])
        self.assertEqual(cmd[cmd.index("--iterations")+1],"100")
        env=start.call_args.kwargs["env"]
        self.assertTrue(env["GZ_PARTITION"].startswith("amsim-urdf-preview-"))
        self.assertNotEqual(env["GZ_PARTITION"],environment["GZ_PARTITION"])
        metadata=json.loads((self.out/"preview_manifest.json").read_text())
        self.assertEqual(env["GZ_PARTITION"],metadata["partition"])
        self.assertEqual(result["partition"],metadata["partition"])
        self.assertEqual(validate.call_args.args[0][:3],["gz","sdf","-k"])
        self.assertTrue((self.out/"gazebo_check.log").is_file())
        self.assertTrue((self.out/"preview_check.json").is_file())

    def test_loader_error_even_with_zero_exit_is_failure(self):
        world=preview.build_preview(self.model,self.out)
        with mock.patch.object(preview.subprocess,"run",return_value=subprocess.CompletedProcess([],0,"Valid.\n","")),\
             mock.patch.object(preview.subprocess,"Popen",side_effect=self._fake_server(b"[Err] Unable to load mesh.\n")):
            result=preview.check_preview(world)
        self.assertFalse(result["ok"])
        self.assertEqual(len(result["loader_errors"]),1)

    def test_timeout_terminates_only_launched_process(self):
        world=preview.build_preview(self.model,self.out)
        process=mock.Mock(pid=17291,returncode=-15)
        process.wait.side_effect=[subprocess.TimeoutExpired("gz",1),-15]
        process.poll.side_effect=[None,-15]
        with mock.patch.object(preview.subprocess,"run",return_value=subprocess.CompletedProcess([],0,"Valid.\n","")),\
             mock.patch.object(preview.subprocess,"Popen",return_value=process),\
             mock.patch.object(preview.os,"killpg",create=True) as killpg:
            result=preview.check_preview(world,timeout_s=1)
        self.assertFalse(result["ok"])
        self.assertTrue(result["timed_out"])
        if preview.os.name=="posix":
            killpg.assert_called_once_with(process.pid,preview.signal.SIGTERM)
        else:
            process.terminate.assert_called_once()
            killpg.assert_not_called()

    def test_missing_gazebo_records_clear_failure_and_does_not_start_server(self):
        world=preview.build_preview(self.model,self.out)
        with mock.patch.object(preview.subprocess,"run",side_effect=FileNotFoundError()),\
             mock.patch.object(preview.subprocess,"Popen") as start:
            result=preview.check_preview(world)
        start.assert_not_called()
        self.assertFalse(result["ok"])
        self.assertIn("Gazebo server",result["error"])

    def test_changed_asset_is_rejected_before_native_process(self):
        world=preview.build_preview(self.model,self.out)
        (self.model/"meshes"/"part.stl").write_text("changed")
        with mock.patch.object(preview.subprocess,"run") as run:
            with self.assertRaisesRegex(preview.UrdfPreviewError,"changed"):
                preview.check_preview(world)
        run.assert_not_called()

    def test_nonisolated_partition_is_rejected_before_native_process(self):
        world=preview.build_preview(self.model,self.out)
        manifest=self.out/"preview_manifest.json"
        metadata=json.loads(manifest.read_text())
        metadata["partition"]="main-platform"
        manifest.write_text(json.dumps(metadata))
        with mock.patch.object(preview.subprocess,"run") as run:
            with self.assertRaisesRegex(preview.UrdfPreviewError,"isolated partition"):
                preview.check_preview(world)
        run.assert_not_called()


if __name__=="__main__":
    unittest.main()
