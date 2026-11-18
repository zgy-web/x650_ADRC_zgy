#!/usr/bin/env python3
"""Install the portable URDF into an existing matching AMSIM/PX4 environment.

No flight is started. Existing models/scenarios/airframes are never overwritten.
"""
import argparse
import json
from pathlib import Path
import shutil
import sys
import tempfile
import xml.etree.ElementTree as ET
from build import ROOT,NAME,build,sha,xml


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--repo',type=Path,required=True)
    p.add_argument('--px4',type=Path,required=True)
    args=p.parse_args()
    repo=args.repo.resolve(strict=True);px4=args.px4.resolve(strict=True)
    for dependency in ('src/amsim/modeling/manager.py','scripts/install_x650_airframe.py',
                       'simulations/x650_hover_v1/run_case.py','simulations/x650_hover_v1/parameter_evidence.py'):
        if not (repo/dependency).is_file():p.error('Matching AMSIM repository required: missing '+dependency)
    sys.path[:0]=[str(repo/'src'),str(repo/'scripts')]
    from install_x650_airframe import PINNED_PX4, install
    import subprocess
    commit=subprocess.check_output(['git','rev-parse','HEAD'],cwd=px4,text=True).strip()
    if commit!=PINNED_PX4:p.error('PX4 version mismatch: expected '+PINNED_PX4)
    dest=repo/'generated/imported_models'/NAME
    scenario=repo/'scenarios/x650_portable_flight.yaml'
    code=repo/'simulations/x650_portable_v1'
    world=px4/'Tools/simulation/gz/worlds/x650_portable_v1.sdf'
    airframe=ROOT/'config/4982_gz_x650_portable_v1'
    for target in (dest,scenario,code,world):
        if target.exists():p.error('Preserving existing destination: '+str(target))
    for d in (px4/'ROMFS/px4fmu_common/init.d-posix/airframes',px4/'build/px4_sitl_default/etc/init.d-posix/airframes'):
        if not d.is_dir():p.error('PX4 SITL must already be built: '+str(d))
        for f in d.glob('4982_*'):
            if f.name!=airframe.name or f.read_bytes()!=airframe.read_bytes():p.error('Airframe ID 4982 already in use: '+str(f))
    model=build()
    dest.parent.mkdir(parents=True,exist_ok=True)
    with tempfile.TemporaryDirectory(prefix='.x650-portable-',dir=dest.parent) as temp:
        stage=Path(temp)/NAME;shutil.copytree(model,stage)
        (stage/'source').mkdir();(stage/'plugins').mkdir()
        shutil.copy2(ROOT/'urdf/x650_closed_chain.urdf',stage/'model.urdf')
        shutil.copy2(ROOT/'urdf/x650_closed_chain.urdf',stage/'source/original.urdf')
        shutil.copy2(ROOT/'build/plugin/libx650_cad_arm.so',stage/'plugins/libx650_cad_arm.so')
        shutil.copy2(ROOT/'PROVENANCE.json',stage/'source_import_manifest.json')
        shutil.copy2(ROOT/'config/flight_profile.yaml',stage/'flight_profile.yaml')
        shutil.copy2(ROOT/'config/world.sdf',stage/'world.sdf')
        shutil.copy2(airframe,stage/airframe.name)
        tree=ET.parse(stage/'model.sdf')
        for plugin in tree.findall('model/plugin'):
            if plugin.get('name','').startswith('cad_arm::'):
                plugin.set('filename',str(dest/'plugins/libx650_cad_arm.so'))
        xml(tree.getroot(),stage/'model.sdf')
        joints=['upper_arm_joint','fore_crank_joint']
        manifest=dict(schema_version=1,kind='urdf_flight',model_name=NAME,root_link='base_link',base_link='base_link',
            flight_ready=True,flight_validated=False,flight_ready_meaning='Binding installed; run local validation before algorithm comparisons.',
            total_mass_kg=3.005081,px4_sys_autostart=4982,controlled_joints=joints,joints=joints,
            initial_joint_positions={j:0 for j in joints},rotors=[{'joint':f'M{i}_joint'} for i in range(1,5)],
            world_installed=str(world),profile_sha256=sha(stage/'flight_profile.yaml'),source_manifest_sha256=sha(stage/'source_import_manifest.json'),
            model_sha256=sha(stage/'model.sdf'),sdf_sha256=sha(stage/'model.sdf'),
            artifacts={x.relative_to(stage).as_posix():sha(x) for x in stage.rglob('*') if x.is_file()})
        (stage/'import_manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
        stage.rename(dest)
    # The portable model's local absolute plugin path is now resolved for this PC.
    from amsim.modeling.manager import resolve
    resolve(str(dest/'import_manifest.json'))
    install(px4,airframe)
    shutil.copy2(ROOT/'config/world.sdf',world)
    shutil.copytree(ROOT/'scenario',code)
    scenario.write_text(f'''schema_version: 1
name: x650_portable_flight
module: simulations/x650_portable_v1/run_flight.py
backend: px4_sitl
model: {NAME}
vehicle:
  takeoff_height_m: 1.5
record:
  rosbag: true
  px4_ulog: true
  ground_truth: true
visualization:
  default: gui
''')
    # Older copies mistakenly require a world named default when starting GUI.
    # Patch only that exact legacy check, preserving a backup; no broad rewrite.
    visual=repo/'src/amsim/visualization.py'
    old='if topics.returncode != 0 or "/world/default/" not in topics.stdout:'
    new='if topics.returncode != 0 or not any(t.startswith("/world/") and t.endswith("/clock") and len(t.split("/")) == 4 and t.split("/")[2] for t in topics.stdout.splitlines()):'
    patched=False
    if visual.is_file() and old in visual.read_text():
        backup=visual.with_suffix('.py.before_x650_portable')
        if backup.exists():raise RuntimeError('GUI backup already exists; inspect before patching '+str(backup))
        shutil.copy2(visual,backup);visual.write_text(visual.read_text().replace(old,new,1));patched=True
    print(json.dumps(dict(installed_model=str(dest),scenario=str(scenario),gui_legacy_check_fixed=patched,
        next_command='./scripts/amsim run x650_portable_flight --headless',flight_started=False),indent=2))


if __name__=='__main__':main()
