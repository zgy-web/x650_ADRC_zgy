#!/usr/bin/env python3
"""Package the confirmed CAD closed chain for the AMSIM/PX4 runtime.

Propulsion reuses the repository's explicitly approximate MN4014 KV330 model.
CAD mass/inertia are retained; this is not a calibrated hardware twin.
"""
import argparse
import copy
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import sys
import xml.etree.ElementTree as ET
import numpy as np
import yaml
from prepare import ROOT, pose, put, transform, values, write

sys.path.insert(0, str(ROOT/'scripts'))
from install_x650_airframe import install


def sha(p):
    return hashlib.sha256(Path(p).read_bytes()).hexdigest()


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--revision',default='v1')
    p.add_argument('--airframe-id',type=int,default=4985)
    p.add_argument('--px4',type=Path,default=ROOT.parent/'PX4-Autopilot')
    args=p.parse_args()
    if not args.revision.isalnum(): p.error('revision must be alphanumeric')
    name='amsim_urdf_x650_cad_arm_'+args.revision+'_flight'
    dest=ROOT/'generated/imported_models'/name
    if dest.exists(): p.error('Model exists; use a new revision')
    bench=Path(subprocess.check_output([sys.executable,str(Path(__file__).with_name('prepare.py'))],text=True).strip())
    tree=ET.parse(bench/'model.sdf'); model=tree.getroot().find('model')
    model.set('name',name);model.find('pose').text='0 0 0 0 0 0'
    model.remove(model.find("joint[@name='base_to_world']"))
    dest.mkdir(parents=True)
    shutil.copytree(bench/'meshes',dest/'meshes')
    (dest/'source').mkdir()
    shutil.copy2(bench/'source.urdf',dest/'source/original.urdf')
    shutil.copy2(bench/'source.urdf',dest/'model.urdf')
    shutil.copy2(bench/'preparation.json',dest/'source_import_manifest.json')
    (dest/'plugins').mkdir()
    shutil.copy2(ROOT/'generated/x650_cad_arm_build/libx650_cad_arm.so',dest/'plugins/libx650_cad_arm.so')
    for uri in model.findall('.//mesh/uri'):
        uri.text=f'model://{name}/meshes/{Path(uri.text).name}'
    closure=model.find("plugin[@name='cad_arm::Closure']")
    closure.set('filename',str(dest/'plugins/libx650_cad_arm.so'))
    ET.SubElement(model,'plugin',filename=str(dest/'plugins/libx650_cad_arm.so'),name='cad_arm::ImuAcceleration')
    closure.remove(closure.find('csv'))
    put(closure,'velocity_mode','true')
    state=model.find("plugin[@name='gz::sim::systems::JointStatePublisher']")
    state.remove(state.find('topic'))
    put(state,'update_rate',50)
    for j in ('upper_arm_joint','fore_crank_joint'):put(state,'joint_name',j)
    # Align rotor link frames to FLU; rotate visual and inertial frames to preserve CAD.
    for i in range(1,5):
        link=model.find(f"link[@name='M{i}_link']")
        old=np.array([float(x) for x in link.findtext('pose').split()])
        o=ET.Element('origin',xyz=values(old[:3]),rpy=values(old[3:]))
        rot=transform(o);rot[:3,3]=0
        for element in [link.find('inertial'),*link.findall('visual')]:
            pv=[float(x) for x in element.findtext('pose','0 0 0 0 0 0').split()]
            el=ET.Element('origin',xyz=values(pv[:3]),rpy=values(pv[3:]))
            element.find('pose').text=pose(rot@transform(el))
        link.find('pose').text=values([*old[:3],0,0,0])
        j=model.find(f"joint[@name='M{i}_joint']");j.set('type','revolute')
        put(j,'pose','0 0 0 0 0 0',relative_to=f'M{i}_link')
        a=ET.SubElement(j,'axis');put(a,'xyz','0 0 1')
        lim=ET.SubElement(a,'limit');put(lim,'lower',-1e16);put(lim,'upper',1e16);put(lim,'velocity',1000)
    # Ground contacts: approximate skids and body; arm boxes use CAD dimensions.
    def box(link,name,center,size):
        c=ET.SubElement(link,'collision',name=name);put(c,'pose',values([*center,0,0,0]))
        put(ET.SubElement(ET.SubElement(c,'geometry'),'box'),'size',values(size))
    base=model.find("link[@name='base_link']")
    for side in (-1,1):box(base,'skid_'+str(side),[0,side*.15,-.33550595],[.32,.025,.02])
    box(base,'body',[0,0,-.015],[.16,.16,.08])
    box(model.find("link[@name='upper_arm_Link']"),'arm_box',[.114,0,-.008],[.266,.038,.027])
    box(model.find("link[@name='fore_arm_Link']"),'forearm_box',[.141,0,0],[.298,.03,.019])
    sensorbase=ET.parse(ROOT/'assets/models/x500_underfold/model.sdf').getroot().find("model/link[@name='base_link']")
    for sensor in sensorbase.findall('sensor'):base.append(copy.deepcopy(sensor))
    profile=yaml.safe_load((ROOT/'configs/vehicles/x650_no_arm/flight_profile.yaml').read_text())
    profile.update(px4_sys_autostart=args.airframe_id,controlled_joints=['upper_arm_joint','fore_crank_joint'])
    profile['notes']='CAD combined 3.005081 kg; inherited MN4014 KV330/15x5CF/22.2V approximate simulation propulsion. New rotor names map by geometry, not hardware wiring. Force-servo closed chain; Bullet Featherstone required.'
    mapping={'jointName':'joint','linkName':'link','turningDirection':'turning_direction','motorConstant':'motor_constant','momentConstant':'moment_constant','maxRotVelocity':'max_rot_velocity','timeConstantUp':'time_constant_up','timeConstantDown':'time_constant_down','rotorDragCoefficient':'rotor_drag_coefficient','rollingMomentCoefficient':'rolling_moment_coefficient','rotorVelocitySlowdownSim':'rotor_velocity_slowdown_sim'}
    for i,rotor in enumerate(profile['rotors'],1):
        rotor['joint']=f'M{i}_joint';rotor['link']=f'M{i}_link'
        plugin=ET.SubElement(model,'plugin',filename='gz-sim-multicopter-motor-model-system',name='gz::sim::systems::MulticopterMotorModel')
        for tag,key in mapping.items():put(plugin,tag,rotor[key])
        put(plugin,'commandSubTopic','command/motor_speed');put(plugin,'motorNumber',i-1);put(plugin,'motorType','velocity')
    odom=ET.SubElement(model,'plugin',filename='gz-sim-odometry-publisher-system',name='gz::sim::systems::OdometryPublisher')
    for k,v in dict(dimensions=3,odom_publish_frequency=50,odom_topic='/ground_truth/odometry',odom_frame='world',robot_base_frame=name+'/model_frame',gaussian_noise=0).items():put(odom,k,v)
    write(tree.getroot(),dest/'model.sdf')
    config=ET.Element('model');put(config,'name',name);put(config,'version','1.0');put(config,'sdf','model.sdf',version='1.9');write(config,dest/'model.config')
    (dest/'flight_profile.yaml').write_text(yaml.safe_dump(profile,sort_keys=False))
    # Compute the initial aggregate COM from physical inertials, including proxy split.
    mass=0.; weighted=np.zeros(3)
    for link in model.findall('link'):
        pv=list(map(float,link.findtext('pose','0 0 0 0 0 0').split()));lv=transform(ET.Element('origin',xyz=values(pv[:3]),rpy=values(pv[3:])))
        iv=list(map(float,link.findtext('inertial/pose').split()));it=transform(ET.Element('origin',xyz=values(iv[:3]),rpy=values(iv[3:])))
        m=float(link.findtext('inertial/mass'));mass+=m;weighted+=m*(lv@it)[:3,3]
    com=weighted/mass
    worldname='x650_cad_arm_'+args.revision
    airframe_name=f'{args.airframe_id}_gz_x650_cad_arm_{args.revision}'
    airframe=dest/airframe_name
    lines=['#!/bin/sh','. ${R}etc/init.d/rc.mc_defaults','PX4_SIMULATOR=gz',f'PX4_GZ_WORLD={worldname}',f'PX4_SIM_MODEL={name}','PX4_GZ_MODEL_POSE=0,0,0.40,0,0,0']
    parameters={'SIM_GZ_EN':1,'CA_AIRFRAME':0,'CA_ROTOR_COUNT':4,'CA_R_REV':0,'SIM_GZ_EC_REV':0,'THR_MDL_FAC':1,'MPC_THR_HOVER':mass*9.80665/(4*profile['rotors'][0]['motor_constant']*628**2)}
    for i in range(4):
        loc=np.array(list(map(float,model.find(f"link[@name='M{i+1}_link']/pose").text.split()))[:3]);frd=(loc-com)*[1,-1,-1]
        for a,v in zip('XYZ',frd):parameters[f'CA_ROTOR{i}_P{a}']=float(v)
        for a,v in zip('XYZ',(0,0,-1)):parameters[f'CA_ROTOR{i}_A{a}']=v
        parameters[f'CA_ROTOR{i}_CT']=profile['rotors'][i]['motor_constant']*628**2
        parameters[f'CA_ROTOR{i}_KM']=.016 if i<2 else -.016
        for k,v in [('FUNC',101+i),('DIS',0),('MIN',0),('MAX',628)]:parameters[f'SIM_GZ_EC_{k}{i+1}']=v
    for i in range(5,9):parameters[f'SIM_GZ_EC_FUNC{i}']=0
    lines.extend(f'param set {key} {value}' for key,value in parameters.items())
    airframe.write_text('\n'.join(lines)+'\n')
    # A dedicated world preserves other scenarios' physics engines and step sizes.
    world=ET.parse(args.px4/'Tools/simulation/gz/worlds/default.sdf').getroot()
    w=world.find('world');w.set('name',worldname);w.find('physics/max_step_size').text='.0005'
    w.find('physics/real_time_update_rate').text='2000'
    server=ET.parse(args.px4/'Tools/simulation/gz/server.config').getroot()
    for plugin in server.findall('plugins/plugin'):
        if plugin.get('filename','').startswith('lib'):continue
        el=copy.deepcopy(plugin);el.attrib.pop('entity_name',None);el.attrib.pop('entity_type',None)
        if 'Physics'==el.get('name').split('::')[-1]:
            put(ET.SubElement(el,'engine'),'filename','gz-physics-bullet-featherstone-plugin')
        w.append(el)
    write(world,dest/'world.sdf')
    targetworld=args.px4/'Tools/simulation/gz/worlds'/(worldname+'.sdf')
    if targetworld.exists():raise FileExistsError(targetworld)
    shutil.copy2(dest/'world.sdf',targetworld)
    validation=subprocess.run(['gz','sdf','-k',str(dest/'model.sdf')],capture_output=True,text=True,check=True)
    if 'Valid.' not in validation.stdout:raise RuntimeError(validation.stdout+validation.stderr)
    manifest=dict(schema_version=1,kind='urdf_flight',model_name=name,root_link='base_link',base_link='base_link',flight_ready=True,flight_validated=False,
        flight_ready_meaning='Binding complete for commissioning; validation results are separate.',
        total_mass_kg=mass,initial_com_flu_m=com.tolist(),px4_sys_autostart=args.airframe_id,
        controlled_joints=profile['controlled_joints'],joints=profile['controlled_joints'],initial_joint_positions={j:0 for j in profile['controlled_joints']},
        rotors=profile['rotors'],profile_sha256=sha(dest/'flight_profile.yaml'),source_manifest_sha256=sha(dest/'source_import_manifest.json'),
        model_sha256=sha(dest/'model.sdf'),sdf_sha256=sha(dest/'model.sdf'),world_installed=str(targetworld),
        collision_approximation='Two landing skid boxes, body box, upper/small arm boxes; self collision disabled; not full CAD collision geometry.',
        artifacts={p.relative_to(dest).as_posix():sha(p) for p in dest.rglob('*') if p.is_file()})
    (dest/'import_manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    install(args.px4,airframe)
    scenario=dict(schema_version=1,name='x650_cad_arm_flight',module='simulations/x650_cad_arm_v1/run_flight.py',backend='px4_sitl',model=name,
                  vehicle={'takeoff_height_m':1.5},record={'rosbag':True,'px4_ulog':True,'ground_truth':True},visualization={'default':'gui'})
    (ROOT/'scenarios/x650_cad_arm_flight.yaml').write_text(yaml.safe_dump(scenario,sort_keys=False))
    print(json.dumps({'model':name,'directory':str(dest),'mass_kg':mass,'com_flu_m':com.tolist(),'airframe':args.airframe_id},indent=2))


if __name__=='__main__':main()
