import importlib.util
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET

SCRIPT = Path(__file__).resolve().parents[2]/'scripts/prepare_x650_robustness_models.py'
SPEC = importlib.util.spec_from_file_location('robust_model_generator', SCRIPT)
module = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(module)


def fixture():
    root = ET.Element('sdf', version='1.9')
    model = ET.SubElement(root, 'model', name='amsim_urdf_nominal')
    for index, name in enumerate(sorted(module.LINKS)):
        link = ET.SubElement(model, 'link', name=name)
        ET.SubElement(link, 'pose').text = f'{index} 0 0 0 0 0'
        inertial = ET.SubElement(link, 'inertial')
        ET.SubElement(inertial, 'mass').text = str(index+1.)
        ET.SubElement(inertial, 'pose').text = '.01 0 .02 0 0 0'
        inertia = ET.SubElement(inertial, 'inertia')
        for key, value in zip(module.INERTIA_FIELDS, [2,.1,.2,3,.15,4]):
            ET.SubElement(inertia, key).text = str(value)
        visual = ET.SubElement(link, 'visual', name='mesh')
        ET.SubElement(visual, 'uri').text = 'model://amsim_urdf_nominal/meshes/original.stl'
        ET.SubElement(link, 'collision', name='same_collision')
    plugin = ET.SubElement(model, 'plugin', filename='/existing/lib.so', name='ts20interface::TS20Interface')
    for key,value in {'kp':'6','kd':'.25','bus_period_s':'.005','combined_interface_delay_s':'.01'}.items():
        ET.SubElement(plugin,key).text=value
    return ET.ElementTree(root)


class RobustnessModels(unittest.TestCase):
    def test_both_reviewed_scales_preserve_source_and_semantics(self):
        for scale in module.SCALES:
            source = fixture(); original = ET.tostring(source.getroot())
            output, ledger = module.scale_tree(source, scale=scale, model_name='amsim_urdf_new')
            self.assertEqual(ET.tostring(source.getroot()), original)
            self.assertEqual(len(ledger), 7)
            self.assertTrue(module.verify_scaled_tree(source, output, scale=scale))
            for link in ledger:
                for key,value in link['nominal'].items():
                    self.assertEqual(link['actual'][key],value*scale)

    def test_rejects_unreviewed_scale_or_reused_identity(self):
        for scale in (True,0,1.,float('nan'),1.2):
            with self.assertRaises(ValueError):
                module.scale_tree(fixture(),scale=scale,model_name='amsim_urdf_new')
        with self.assertRaises(ValueError):
            module.scale_tree(fixture(),scale=.85,model_name='amsim_urdf_nominal')

    def test_rejects_geometry_or_actuator_or_inertia_tamper(self):
        for path in ('model/link/pose','model/plugin/kd','model/link/inertial/inertia/ixy'):
            source=fixture(); output,_=module.scale_tree(source,scale=.85,model_name='amsim_urdf_new')
            output.find(path).text='0.7'
            with self.assertRaises(ValueError):
                module.verify_scaled_tree(source,output,scale=.85)

    def test_rejects_missing_link_component_and_nonphysical_tensor(self):
        for action in ('link','component','negative','indefinite'):
            source=fixture();model=source.find('model');inertia=model.find('link/inertial/inertia')
            if action=='link':model.remove(model.find('link'))
            if action=='component':inertia.remove(inertia.find('ixy'))
            if action=='negative':model.find('link/inertial/mass').text='-1'
            if action=='indefinite':inertia.find('ixy').text='100'
            with self.assertRaises(ValueError):
                module.scale_tree(source,scale=1.15,model_name='amsim_urdf_new')


if __name__=='__main__':
    unittest.main()
