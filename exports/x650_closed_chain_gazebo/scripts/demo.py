#!/usr/bin/env python3
"""Gazebo-only fixed-base demo; --check runs a bounded physical test."""
import argparse
import copy
from datetime import datetime,timezone
import json
import os
from pathlib import Path
import subprocess
import sys
import xml.etree.ElementTree as ET
from build import ROOT,NAME,build,xml


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--check',action='store_true')
    p.add_argument('--skip-build',action='store_true')
    args=p.parse_args()
    modeldir=ROOT/'build/models'/NAME if args.skip_build else build()
    out=ROOT/'runs'/datetime.now(timezone.utc).strftime('%Y%m%d_%H%M%S_%f');out.mkdir(parents=True)
    m=ET.parse(modeldir/'model.sdf').getroot().find('model')
    pose=m.find('pose')
    if pose is None:pose=ET.SubElement(m,'pose')
    pose.text='0 0 .8 0 0 0'
    j=ET.SubElement(m,'joint',name='bench_to_world',type='fixed')
    ET.SubElement(j,'parent').text='world';ET.SubElement(j,'child').text='base_link'
    closure=m.find("plugin[@name='cad_arm::Closure']")
    closure.find('exercise').text='true';closure.find('velocity_mode').text='false'
    ET.SubElement(closure,'csv').text=str(out/'motion.csv')
    w=ET.parse(ROOT/'config/world.sdf');w.getroot().find('world').append(m);xml(w.getroot(),out/'world.sdf')
    (out/'preparation.json').write_text(json.dumps(dict(exercise=True,closure_enabled=True,step_s=.0005,engine='bullet-featherstone')))
    env=os.environ.copy()
    env['GZ_SIM_RESOURCE_PATH']=str(modeldir.parent)+os.pathsep+env.get('GZ_SIM_RESOURCE_PATH','')
    env['GZ_SIM_SYSTEM_PLUGIN_PATH']=str(ROOT/'build/plugin')+os.pathsep+env.get('GZ_SIM_SYSTEM_PLUGIN_PATH','')
    env['GZ_PARTITION']='x650-portable-'+out.name
    print('Result directory: '+str(out),flush=True)
    if args.check:
        raise SystemExit(subprocess.call([sys.executable,str(ROOT/'scripts/check.py'),str(out)],env=env))
    os.execvpe('gz',['gz','sim','-r','--physics-engine','gz-physics-bullet-featherstone-plugin',str(out/'world.sdf')],env)


if __name__=='__main__':main()
