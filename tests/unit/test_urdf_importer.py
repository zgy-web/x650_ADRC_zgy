"""Portable importer tests. Gazebo itself is exercised by server integration tests."""
from __future__ import annotations

import json
import math
from pathlib import Path
import stat
import subprocess
import sys
import tempfile
import unittest
from unittest import mock
import xml.etree.ElementTree as ET
import zipfile

sys.path.insert(0,str(Path(__file__).resolve().parents[2]/"src"))
from amsim.modeling import urdf_importer as importer


URDF = '''<robot name="example">
  <link name="base_link">
    <inertial><origin xyz="0 0.08 0.01" rpy="0 0 0"/><mass value="2.3"/>
    <inertia ixx="0.057" iyy="0.042" izz="0.054" ixy="0.0002" ixz="0" iyz="0"/></inertial>
    <visual><origin xyz="0 0.1 0"/><geometry><mesh filename="meshes/part.stl" scale="0.001 0.001 0.001"/></geometry></visual>
    <collision><origin xyz="0 0.1 0"/><geometry><mesh filename="meshes/part.stl" scale="0.001 0.001 0.001"/></geometry></collision>
  </link>
  <link name="rotor">
    <inertial><mass value="0.18"/><inertia ixx="0.001" iyy="0.001" izz="0.001" ixy="0" ixz="0" iyz="0"/></inertial>
    <visual><origin xyz="0 0 0.01"/><geometry><sphere radius="0.01"/></geometry></visual>
  </link>
  <joint name="rotor_joint" type="revolute"><parent link="base_link"/><child link="rotor"/>
    <origin xyz="0.23 0.17 -0.23" rpy="-1.5707963267948966 0 0"/><axis xyz="0 0 1"/>
    <limit lower="-1.57" upper="1.57" effort="10" velocity="5"/>
  </joint>
</robot>'''


def fake_gz(args, **kwargs):
    if args[2] == "-k":
        return subprocess.CompletedProcess(args,0,"Valid.\n","")
    source = ET.parse(args[3]).getroot()
    sdf = ET.Element("sdf",version="1.9")
    model = ET.SubElement(sdf,"model",name=source.get("name"))
    for node in source.findall("link"):
        link = ET.SubElement(model,"link",name=node.get("name"))
        inertial = ET.SubElement(link,"inertial")
        ET.SubElement(inertial,"mass").text=node.find("inertial/mass").get("value")
        for kind in ("visual","collision"):
            for n,geometry in enumerate(node.findall(kind)):
                mesh = geometry.find("geometry/mesh")
                if mesh is not None:
                    out = ET.SubElement(ET.SubElement(link,kind,name=f"{kind}_{n}"),"geometry")
                    ET.SubElement(ET.SubElement(out,"mesh"),"uri").text=mesh.get("filename")
    for joint in source.findall("joint"):
        ET.SubElement(model,"joint",name=joint.get("name"),type=joint.get("type"))
    return subprocess.CompletedProcess(args,0,ET.tostring(sdf,encoding="unicode"),"")


class ImportTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory(prefix="amsim-test-urdf-")
        self.root = Path(self.tmp.name)
        self.source = self.root/"input"/"robot.urdf"
        self.source.parent.mkdir()
        (self.source.parent/"meshes").mkdir()
        # A tiny self-contained ASCII STL; native parsing is tested on the server.
        (self.source.parent/"meshes"/"part.stl").write_text("solid fixture\nendsolid fixture\n")
        self.source.write_text(URDF)
        self.output = self.root/"generated"

    def tearDown(self):
        self.tmp.cleanup()

    def test_inspection_is_read_only_and_records_units_and_limits(self):
        before = self.source.read_bytes()
        with mock.patch.object(importer.subprocess,"run") as run:
            report = importer.inspect_urdf(self.source)
        run.assert_not_called()
        self.assertFalse(report["converted"])
        self.assertFalse(report["flight_ready"])
        self.assertAlmostEqual(report["total_mass_kg"],2.48)
        self.assertEqual(report["joints"][0]["type"],"revolute")
        self.assertEqual(report["joints"][0]["limits"]["velocity"],5)
        self.assertEqual(report["meshes"][0]["scale"],[0.001]*3)
        self.assertEqual(self.source.read_bytes(),before)
        self.assertFalse(self.output.exists())

    def test_root_reframe_changes_body_and_first_joint_not_child_geometry(self):
        before = self.source.read_bytes()
        with mock.patch.object(importer.subprocess,"run",side_effect=fake_gz):
            result = importer.import_urdf(self.source,"Example",output_root=self.output,
                                          root_rpy=(math.pi/2,0,0),root_xyz=(0,0,-.17))
        robot = ET.parse(result.urdf_path).getroot()
        base = robot.find("link")
        p = [float(x) for x in base.find("inertial/origin").get("xyz").split()]
        for actual,expected in zip(p,[0,-.01,-.09]):
            self.assertAlmostEqual(actual,expected)
        self.assertEqual(base.find("inertial/inertia").attrib,ET.fromstring(URDF).find("link/inertial/inertia").attrib)
        self.assertEqual(robot.findall("link")[1].find("visual/origin").get("xyz"),"0 0 0.01")
        joint = robot.find("joint")
        self.assertEqual(joint.get("type"),"revolute")
        self.assertEqual(joint.find("limit").attrib,ET.fromstring(URDF).find("joint/limit").attrib)
        report = json.loads(result.manifest_path.read_text())
        for actual,expected in zip(report["joints"][0]["origin"]["xyz"],[.23,.23,0]):
            self.assertAlmostEqual(actual,expected)
        for actual,expected in zip(report["joints"][0]["axis_in_root_at_zero"],[0,0,1]):
            self.assertAlmostEqual(actual,expected)
        self.assertEqual(self.source.read_bytes(),before)
        self.assertEqual((result.directory/"source"/"original.urdf").read_bytes(),before)
        self.assertEqual(result.root_link,"base_link")

    def test_portable_bundle_idempotence_and_refuse_modified_artifact(self):
        with mock.patch.object(importer.subprocess,"run",side_effect=fake_gz) as run:
            result = importer.import_urdf(self.source,"My Example",output_root=self.output)
            repeated = importer.import_urdf(self.source,"My Example",output_root=self.output)
        self.assertEqual(run.call_count,2)
        self.assertEqual(result,repeated)
        self.assertEqual(result.model_name,"amsim_urdf_my_example")
        self.assertEqual(len(list((result.directory/"meshes").iterdir())),1)
        for uri in ET.parse(result.sdf_path).findall(".//mesh/uri"):
            self.assertTrue(uri.text.startswith("model://amsim_urdf_my_example/meshes/"))
            self.assertTrue((result.directory/uri.text.split("/",3)[3]).is_file())
        result.urdf_path.write_text("changed")
        with self.assertRaisesRegex(importer.UrdfImportError,"modified"):
            importer.import_urdf(self.source,"My Example",output_root=self.output)

    def test_refuse_existing_output_and_different_source(self):
        target = self.output/"amsim_urdf_existing"
        target.mkdir(parents=True)
        sentinel = target/"keep.txt"
        sentinel.write_text("user data")
        with self.assertRaisesRegex(importer.UrdfImportError,"overwrite"):
            importer.import_urdf(self.source,"existing",output_root=self.output)
        self.assertEqual(sentinel.read_text(),"user data")
        with mock.patch.object(importer.subprocess,"run",side_effect=fake_gz):
            importer.import_urdf(self.source,"new",output_root=self.output)
        self.source.write_text(URDF.replace('mass value="2.3"','mass value="2.4"'))
        with self.assertRaisesRegex(importer.UrdfImportError,"different source"):
            importer.import_urdf(self.source,"new",output_root=self.output)

    def test_zip_unicode_nested_source_and_explicit_entry(self):
        archive = self.root/"机架.zip"
        with zipfile.ZipFile(archive,"w") as z:
            z.writestr("机架/robot.urdf",URDF)
            z.writestr("机架/second.urdf",URDF)
            z.writestr("机架/meshes/part.stl","solid test\nendsolid test")
        with self.assertRaisesRegex(importer.UrdfImportError,"select urdf_entry"):
            importer.inspect_urdf(archive)
        report = importer.inspect_urdf(archive,urdf_entry="机架/robot.urdf")
        self.assertEqual(report["source"]["urdf_entry"],"机架/robot.urdf")
        self.assertEqual(len(report["meshes"]),2)

    def test_zip_rejects_traversal_symlink_duplicates_and_size(self):
        archive = self.root/"bad.zip"
        for member in ("../outside.urdf","/absolute.urdf","C:/absolute.urdf",r"..\outside.urdf"):
            with self.subTest(member=member):
                with zipfile.ZipFile(archive,"w") as z:
                    z.writestr(member,URDF)
                with self.assertRaisesRegex(importer.UrdfImportError,"Unsafe archive"):
                    importer.inspect_urdf(archive)
        info = zipfile.ZipInfo("robot.urdf")
        info.create_system=3
        info.external_attr=(stat.S_IFLNK|0o777)<<16
        with zipfile.ZipFile(archive,"w") as z:
            z.writestr(info,"/etc/passwd")
        with self.assertRaisesRegex(importer.UrdfImportError,"special files"):
            importer.inspect_urdf(archive)
        with zipfile.ZipFile(archive,"w") as z:
            z.writestr("robot.urdf",URDF)
            z.writestr("ROBOT.urdf",URDF)
        with self.assertRaisesRegex(importer.UrdfImportError,"Duplicate archive"):
            importer.inspect_urdf(archive)
        with zipfile.ZipFile(archive,"w") as z:
            z.writestr("robot.urdf",URDF)
        with mock.patch.object(importer,"MAX_FILE_BYTES",8):
            with self.assertRaisesRegex(importer.UrdfImportError,"size limit"):
                importer.inspect_urdf(archive)
        self.assertFalse((self.root.parent/"outside.urdf").exists())

    def test_package_uri_requires_mapping_and_copies_dependency(self):
        self.source.write_text(URDF.replace("meshes/part.stl","package://airframe/meshes/part.stl"))
        with self.assertRaisesRegex(importer.UrdfImportError,"package_paths"):
            importer.inspect_urdf(self.source)
        with mock.patch.object(importer.subprocess,"run",side_effect=fake_gz):
            result = importer.import_urdf(self.source,"package",output_root=self.output,
                                          package_paths={"airframe":self.source.parent})
        self.assertEqual(len(list((result.directory/"meshes").iterdir())),1)

    def test_invalid_graph_and_inertia_rejected_before_conversion(self):
        cases = {
            "duplicate link":URDF.replace('name="rotor"','name="base_link"'),
            "invalid parent":URDF.replace('parent link="base_link"','parent link="missing"'),
            "nonpositive inertia":URDF.replace('ixx="0.057"','ixx="-0.057"'),
            "triangle inertia":URDF.replace('ixx="0.057"','ixx="5.7"'),
            "nonfinite mass":URDF.replace('mass value="2.3"','mass value="nan"'),
            "bad axis":URDF.replace('axis xyz="0 0 1"','axis xyz="0 0 0"'),
            "bad scale":URDF.replace('scale="0.001 0.001 0.001"','scale="0 1 1"'),
        }
        with mock.patch.object(importer.subprocess,"run") as run:
            for key,source in cases.items():
                with self.subTest(key=key):
                    self.source.write_text(source)
                    with self.assertRaises(importer.UrdfImportError):
                        importer.import_urdf(self.source,key,output_root=self.output)
        run.assert_not_called()

    def test_zero_outside_limits_reported_without_silent_change(self):
        self.source.write_text(URDF.replace('lower="-1.57"','lower="0.2"'))
        report = importer.inspect_urdf(self.source)
        self.assertFalse(report["joints"][0]["initial_position_within_limits"])
        self.assertEqual(report["joints"][0]["limits"]["lower"],.2)
        self.assertTrue(any("outside joint limits" in w for w in report["warnings"]))

    def test_complex_texture_dependencies_are_explicitly_rejected(self):
        obj = self.source.parent/"meshes"/"part.obj"
        obj.write_text("mtllib material.mtl\nv 0 0 0\n")
        self.source.write_text(URDF.replace("part.stl","part.obj"))
        with self.assertRaisesRegex(importer.UrdfImportError,"materials"):
            importer.inspect_urdf(self.source)
        dae = self.source.parent/"meshes"/"part.dae"
        dae.write_text('<COLLADA><library_images><image><init_from>texture.png</init_from></image></library_images></COLLADA>')
        self.source.write_text(URDF.replace("part.stl","part.dae"))
        with self.assertRaisesRegex(importer.UrdfImportError,"images/references"):
            importer.inspect_urdf(self.source)
        self.source.write_text(URDF.replace('<visual>','<visual><material><texture filename="texture.png"/></material>',1))
        with self.assertRaisesRegex(importer.UrdfImportError,"textures"):
            importer.inspect_urdf(self.source)

    def test_missing_native_converter_and_timeout_leave_no_published_model(self):
        for side_effect,pattern in ((FileNotFoundError(),"Native 'gz sdf'"),
                                    (subprocess.TimeoutExpired("gz",120),"timed out")):
            with self.subTest(pattern=pattern),mock.patch.object(importer.subprocess,"run",side_effect=side_effect):
                with self.assertRaisesRegex(importer.UrdfImportError,pattern):
                    importer.import_urdf(self.source,"failure",output_root=self.output)
            self.assertFalse((self.output/"amsim_urdf_failure").exists())
            self.assertEqual(list(self.output.glob(".urdf-import-*")),[])

    def test_conversion_failure_and_geometry_loss_do_not_publish(self):
        def lost_geometry(args,**kwargs):
            result = fake_gz(args,**kwargs)
            if args[2] == "-p":
                root = ET.fromstring(result.stdout)
                link = root.find("model/link")
                link.remove(link.find("visual"))
                result.stdout=ET.tostring(root,encoding="unicode")
            return result
        with mock.patch.object(importer.subprocess,"run",return_value=subprocess.CompletedProcess([],1,"","bad URDF")):
            with self.assertRaisesRegex(importer.UrdfImportError,"conversion failed"):
                importer.import_urdf(self.source,"failed",output_root=self.output)
        with mock.patch.object(importer.subprocess,"run",side_effect=lost_geometry):
            with self.assertRaisesRegex(importer.UrdfImportError,"dropped mesh"):
                importer.import_urdf(self.source,"lost",output_root=self.output)
        self.assertFalse((self.output/"amsim_urdf_lost").exists())


if __name__ == "__main__":
    unittest.main()
