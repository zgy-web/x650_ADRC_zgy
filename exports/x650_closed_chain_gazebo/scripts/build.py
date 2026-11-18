#!/usr/bin/env python3
"""Build locally and convert the distributable URDF; Python standard library only."""
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import xml.etree.ElementTree as ET

ROOT=Path(__file__).resolve().parents[1]
NAME='amsim_urdf_x650_portable_v1_flight'


def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()


def xml(root,path):
    ET.indent(root)
    ET.ElementTree(root).write(path,encoding='utf-8',xml_declaration=True)


def build():
    subprocess.run(['cmake','-S',str(ROOT),'-B',str(ROOT/'build/plugin'),'-DCMAKE_BUILD_TYPE=Release'],check=True)
    subprocess.run(['cmake','--build',str(ROOT/'build/plugin'),'-j2'],check=True)
    model=ROOT/'build/models'/NAME
    model.mkdir(parents=True,exist_ok=True)
    shutil.copytree(ROOT/'meshes',model/'meshes',dirs_exist_ok=True)
    tree=ET.parse(ROOT/'urdf/x650_closed_chain.urdf')
    for mesh in tree.findall('.//mesh'):
        uri=mesh.get('filename');prefix='package://x650_closed_chain_gazebo/'
        if not uri.startswith(prefix):raise ValueError('Unexpected mesh URI: '+uri)
        resource=(ROOT/uri[len(prefix):]).resolve(strict=True)
        if not resource.is_relative_to(ROOT):raise ValueError('Resource escapes package')
        mesh.set('filename',str(resource))
    xml(tree.getroot(),ROOT/'build/resolved.urdf')
    converted=subprocess.run(['gz','sdf','-p',str(ROOT/'build/resolved.urdf')],capture_output=True,text=True,check=True)
    (ROOT/'build/conversion.log').write_text(converted.stderr)
    sdf=ET.fromstring(converted.stdout);m=sdf.find('model')
    if m is None or m.get('name')!=NAME:raise RuntimeError('Converted model identity mismatch')
    for mesh in m.findall('.//mesh/uri'):
        mesh.text=f'model://{NAME}/meshes/'+Path(mesh.text).name
    required={'cad_arm::Closure','cad_arm::ImuAcceleration','gz::sim::systems::MulticopterMotorModel'}
    if not required.issubset({x.get('name') for x in m.findall('plugin')}):raise RuntimeError('Gazebo URDF extensions lost in conversion')
    mass=sum(float(x.text) for x in m.findall('link/inertial/mass'))
    if abs(mass-3.005081)>1e-9 or len(m.findall('joint'))!=9 or len(m.findall('link'))!=10:
        raise RuntimeError('Conversion lost physical links/joints/mass')
    xml(sdf,model/'model.sdf')
    conf=ET.Element('model');ET.SubElement(conf,'name').text=NAME
    ET.SubElement(conf,'version').text='1.0';ET.SubElement(conf,'sdf',version=sdf.get('version')).text='model.sdf'
    xml(conf,model/'model.config')
    env=os.environ.copy();env['GZ_SIM_RESOURCE_PATH']=str(model.parent)+os.pathsep+env.get('GZ_SIM_RESOURCE_PATH','')
    result=subprocess.run(['gz','sdf','-k',str(model/'model.sdf')],capture_output=True,text=True,env=env,check=True)
    if 'Valid.' not in result.stdout:raise RuntimeError(result.stdout+result.stderr)
    report=dict(urdf_sha256=sha(ROOT/'urdf/x650_closed_chain.urdf'),sdf_sha256=sha(model/'model.sdf'),
                plugin_sha256=sha(ROOT/'build/plugin/libx650_cad_arm.so'),mass_kg=mass,links=10,joints=9,native_validation=result.stdout)
    (ROOT/'build/build_report.json').write_text(json.dumps(report,indent=2)+'\n')
    print('Built model: '+str(model),flush=True)
    return model


if __name__=='__main__':build()
