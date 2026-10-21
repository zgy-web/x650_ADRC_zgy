"""Physical invariants of the X650 commissioning asset preparation."""
import importlib.util
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch
import xml.etree.ElementTree as ET

import numpy as np

SPEC = importlib.util.spec_from_file_location("x650_prepare", Path(__file__).resolve().parents[2]/"scripts/prepare_x650_flight.py")
x650 = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(x650)


class PreparationTests(unittest.TestCase):
    def source(self, directory):
        robot = ET.Element("robot", name="fixture")
        base = ET.SubElement(robot, "link", name="base_link")
        x650.set_properties(base, 2.3, np.array([.001,.09,-.002]), np.diag([.057,.042,.054]))
        for index, (name, _, _) in enumerate(x650.MAPPING):
            link = ET.SubElement(robot, "link", name=name)
            x650.set_properties(link, .18, np.array([.0001,-.0002,-.005]), np.diag([.0006,.0004,.0007]))
            for part in (x650.PROP_PARTS[name], "motor_"+name):
                filename = part+".stl"
                (directory/filename).write_bytes(b"fixture mesh")
                for tag in ("visual", "collision"):
                    node = ET.SubElement(link, tag)
                    x650.set_origin(node, np.array([.01,.02,.003]))
                    ET.SubElement(ET.SubElement(node,"geometry"), "mesh", filename=filename)
            joint = ET.SubElement(robot, "joint", name=name+"_joint", type="revolute")
            ET.SubElement(joint, "parent", link="base_link")
            ET.SubElement(joint, "child", link=name)
            x = .23 if index in (0,2) else -.23
            z = .23 if index in (0,3) else -.23
            x650.set_origin(joint, np.array([x,.17095298,z]), x650.rotation([-np.pi/2,0,0]))
            ET.SubElement(joint, "axis", xyz="0 0 1")
            ET.SubElement(joint, "limit", lower="-1.57", upper="1.57", effort="10", velocity="5")
        file = directory/"robot.urdf"
        ET.ElementTree(robot).write(file)
        return file

    def test_fitted_selected_kv330_curve_and_extrapolation(self):
        result = x650.fit_propulsion()
        self.assertAlmostEqual(result["fit"]["motor_constant"],4.828453605732144e-5, places=15)
        self.assertEqual(result["limits"]["maximum_rpm"], 6000)
        self.assertLess(result["hover_prediction"]["rpm"],3900)
        self.assertTrue(result["hover_prediction"]["below_lowest_measured_rpm"])

    def test_mass_com_inertia_and_rotor_balance(self):
        with tempfile.TemporaryDirectory() as temporary:
            parent=Path(temporary); source_dir=parent/"source"; source_dir.mkdir()
            source=self.source(source_dir)
            with patch.object(x650,"SOURCE_SHA",x650.sha(source)):
                report=x650.prepare(source,parent/"derived",4999)
            self.assertLess(abs(report["conservation"]["mass_error_kg"]),1e-12)
            self.assertLess(report["conservation"]["max_com_error_m"],1e-12)
            self.assertLess(report["conservation"]["max_inertia_error_kg_m2"],1e-12)
            robot=ET.parse(parent/"derived/robot.urdf").getroot()
            for name,_,_ in x650.MAPPING:
                link=robot.find(f"link[@name='{name}']")
                mass,com,inertia=x650.properties(link)
                self.assertAlmostEqual(mass,.021)
                np.testing.assert_allclose(com,0,atol=1e-15)
                for angle in (-np.pi/2,.7,np.pi):
                    r=x650.rotation([0,0,angle])
                    np.testing.assert_allclose(r@inertia@r.T,inertia,atol=1e-15)
                self.assertEqual(len(link.findall("visual")),1)
                joint=robot.find(f"joint[@name='{name}_joint']")
                self.assertEqual(joint.get("type"),"continuous")
                self.assertNotIn("lower",joint.find("limit").attrib)
                self.assertAlmostEqual(float(joint.find("limit").get("velocity")),200*np.pi)

    def test_unknown_source_is_rejected_without_output(self):
        with tempfile.TemporaryDirectory() as temporary:
            parent=Path(temporary); source_dir=parent/"source"; source_dir.mkdir()
            source=self.source(source_dir)
            with self.assertRaisesRegex(ValueError,"SHA256"):
                x650.prepare(source,parent/"derived",4999)
            self.assertFalse((parent/"derived").exists())

    def test_numbering_and_yaw_sign_match_explicit_simulation_convention(self):
        profile=x650.profile(4999,x650.fit_propulsion())
        self.assertEqual(profile["px4_sys_autostart"],4999)
        self.assertEqual([r["link"] for r in profile["rotors"]],["link_004","link_002","link_003","link_005"])
        self.assertEqual([r["turning_direction"] for r in profile["rotors"]],["ccw","ccw","cw","cw"])


if __name__=="__main__":
    unittest.main()
