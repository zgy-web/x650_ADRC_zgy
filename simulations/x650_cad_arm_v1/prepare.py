#!/usr/bin/env python3
"""Build a fixed-base dynamics experiment from the user's assembled CAD URDF."""
import argparse
import copy
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import shutil
import sys
import xml.etree.ElementTree as ET
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'src'))
from amsim.modeling.urdf_importer import _rotation


def put(parent, tag, text, **attrs):
    node = ET.SubElement(parent, tag, **attrs)
    node.text = str(text)
    return node


def values(v):
    return ' '.join(format(float(x), '.15g') for x in v)


def transform(origin):
    t = np.eye(4)
    if origin is not None:
        t[:3, :3] = _rotation([float(x) for x in origin.get('rpy', '0 0 0').split()])
        t[:3, 3] = [float(x) for x in origin.get('xyz', '0 0 0').split()]
    return t


def pose(t):
    r = t[:3, :3]
    return values([*t[:3, 3], math.atan2(r[2, 1], r[2, 2]),
                   math.atan2(-r[2, 0], math.hypot(r[0, 0], r[1, 0])),
                   math.atan2(r[1, 0], r[0, 0])])


def write(root, path):
    ET.indent(root)
    ET.ElementTree(root).write(path, encoding='utf-8', xml_declaration=True)


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--package', type=Path, default=Path('/home/ubuntu22/uav_arm/urdf/uav_arm_urdf'))
    p.add_argument('--exercise', action='store_true')
    p.add_argument('--disable-closure', action='store_true')
    p.add_argument('--step', type=float, default=.0005)
    p.add_argument('--engine', choices=['dartsim', 'bullet-featherstone'], default='bullet-featherstone')
    args = p.parse_args()
    if not 0 < args.step <= .001:
        p.error('step must be in (0, .001]')
    plugin = ROOT / 'generated/x650_cad_arm_build/libx650_cad_arm.so'
    if not plugin.is_file():
        p.error('Build the CMake plugin first')
    source = args.package / 'urdf/uav_arm_urdf.urdf'
    robot = ET.parse(source).getroot()
    stamp = datetime.now(timezone.utc).strftime('%Y%m%d_%H%M%S_%f')
    output = ROOT / 'runs' / ('x650_cad_arm_' + stamp)
    output.mkdir(parents=True)
    shutil.copy2(source, output / 'source.urdf')
    meshdir = output / 'meshes'
    meshdir.mkdir()
    original = {'base_link': np.eye(4)}
    for j in robot.findall('joint'):
        original[j.find('child').get('link')] = original[j.find('parent').get('link')] @ transform(j.find('origin'))
    calibrated = copy.deepcopy(original)
    # Exact parallel axes; remove rounded pi/2 and pi in the CAD export.
    calibrated['upper_arm_Link'][:3, :3] = _rotation([-math.pi/2, .66323, -math.pi])
    calibrated['fore_crank_Link'][:3, :3] = _rotation([-math.pi/2, 0, math.pi])
    for n in ('fore_long_link', 'fore_arm_Link'):
        calibrated[n][:3, :3] = _rotation([math.pi/2, 0, 0])
    calibrated['fore_long_link'][:3, 3] = [-.05, .00325, -.155]
    upper = calibrated['upper_arm_Link']
    # The closure anchor's Y is the small-arm layer, not the shoulder origin.
    b = (upper @ np.array([.23, 0, 0, 1]))[:3]
    b[1] = .00170
    d = b + np.array([-.05, 0, 0])
    calibrated['fore_arm_Link'][:3, 3] = d
    anchor_big = (np.linalg.inv(upper) @ np.r_[b, 1])[:3]
    anchor_small = (np.linalg.inv(calibrated['fore_arm_Link']) @ np.r_[b, 1])[:3]
    assert np.linalg.norm(upper[:3, :3] @ anchor_big + upper[:3, 3] - b) < 1e-12
    sdf = ET.Element('sdf', version='1.9')
    model = ET.SubElement(sdf, 'model', name='x650_cad_arm')
    put(model, 'pose', '0 0 .8 0 0 0')
    put(model, 'self_collide', 'false')
    hashes = {}
    for src in robot.findall('link'):
        name = src.get('name')
        if src.find('inertial') is None:
            continue  # Remove only the two nonphysical mimic helpers.
        link = ET.SubElement(model, 'link', name=name)
        put(link, 'pose', pose(calibrated[name]))
        inert = ET.SubElement(link, 'inertial')
        si = src.find('inertial')
        put(inert, 'pose', pose(transform(si.find('origin'))))
        fraction = .95 if name == 'upper_arm_Link' else 1.
        put(inert, 'mass', float(si.find('mass').get('value')) * fraction)
        tensor = ET.SubElement(inert, 'inertia')
        for key, val in si.find('inertia').attrib.items():
            put(tensor, key, float(val) * fraction)
        for i, visual in enumerate(src.findall('visual')):
            out = ET.SubElement(link, 'visual', name=f'visual_{i}')
            put(out, 'pose', pose(transform(visual.find('origin'))))
            mesh = visual.find('geometry/mesh')
            uri = mesh.get('filename')
            prefix = 'package://uav_arm_urdf/'
            if not uri.startswith(prefix):
                raise ValueError(uri)
            asset = (args.package / uri[len(prefix):]).resolve(strict=True)
            if not asset.is_relative_to(args.package.resolve()):
                raise ValueError('Mesh outside package')
            shutil.copy2(asset, meshdir / asset.name)
            hashes[asset.name] = hashlib.sha256(asset.read_bytes()).hexdigest()
            target = ET.SubElement(ET.SubElement(out, 'geometry'), 'mesh')
            put(target, 'uri', (meshdir / asset.name).as_uri())
            put(target, 'scale', mesh.get('scale', '1 1 1'))
            color = visual.find('material/color')
            if color is not None:
                material = ET.SubElement(out, 'material')
                for key in ('ambient', 'diffuse'):
                    put(material, key, color.get('rgba'))
    big = model.find("link[@name='upper_arm_Link']")
    proxy = ET.SubElement(model, 'link', name='B_closure_proxy')
    proxy.append(copy.deepcopy(big.find('pose')))
    inertia = copy.deepcopy(big.find('inertial'))
    for e in [inertia.find('mass'), *inertia.find('inertia')]:
        e.text = str(float(e.text) * .05/.95)
    proxy.append(inertia)

    def joint(name, parent, child, point=None, axis=(0, 1, 0), limits=None):
        j = ET.SubElement(model, 'joint', name=name, type='revolute' if point is not None else 'fixed')
        put(j, 'parent', parent)
        put(j, 'child', child)
        if point is not None:
            put(j, 'pose', values([*point, 0, 0, 0]), relative_to='__model__')
            a = ET.SubElement(j, 'axis')
            put(a, 'xyz', values(axis), expressed_in='__model__')
            lim = ET.SubElement(a, 'limit')
            put(lim, 'lower', limits[0] if limits else -math.pi)
            put(lim, 'upper', limits[1] if limits else math.pi)
            if limits:
                put(lim, 'effort', 10)
                put(lim, 'velocity', 1)
            put(ET.SubElement(a, 'dynamics'), 'damping', .002)
    joint('base_to_world', 'world', 'base_link')
    for i in range(1, 5):
        joint(f'M{i}_joint', 'base_link', f'M{i}_link')
    joint('upper_arm_joint', 'base_link', 'upper_arm_Link', upper[:3, 3], (0, -1, 0), (0, 1.95))
    joint('fore_crank_joint', 'base_link', 'fore_crank_Link', calibrated['fore_crank_Link'][:3, 3], (0, 1, 0), (0, .78))
    joint('C_passive', 'fore_crank_Link', 'fore_long_link', calibrated['fore_long_link'][:3, 3])
    joint('D_passive', 'fore_long_link', 'fore_arm_Link', d)
    joint('B_passive', 'fore_arm_Link', 'B_closure_proxy', b)
    closure = ET.SubElement(model, 'plugin', filename=str(plugin), name='cad_arm::Closure')
    put(closure, 'enabled', str(not args.disable_closure).lower())
    put(closure, 'exercise', str(args.exercise).lower())
    put(closure, 'csv', str(output / 'motion.csv'))
    put(closure, 'anchor_big', values(anchor_big))
    put(closure, 'anchor_small', values(anchor_small))
    publisher = ET.SubElement(model, 'plugin', filename='gz-sim-joint-state-publisher-system', name='gz::sim::systems::JointStatePublisher')
    put(publisher, 'topic', '/x650_cad_arm/joint_states')
    write(sdf, output / 'model.sdf')
    world_sdf = ET.Element('sdf', version='1.9')
    world = ET.SubElement(world_sdf, 'world', name='x650_cad_arm_bench')
    put(world, 'gravity', '0 0 -9.80665')
    physics = ET.SubElement(world, 'physics', name='bench', type='ignored')
    put(physics, 'max_step_size', args.step)
    put(physics, 'real_time_factor', 1)
    for filename, name in [('physics','Physics'),('user-commands','UserCommands'),('scene-broadcaster','SceneBroadcaster')]:
        ET.SubElement(world, 'plugin', filename=f'gz-sim-{filename}-system', name=f'gz::sim::systems::{name}')
    light = ET.SubElement(world, 'light', name='sun', type='directional')
    put(light, 'pose', '0 0 10 0 0 0');put(light, 'diffuse', '.8 .8 .8 1');put(light, 'direction', '-.5 .1 -.9')
    world.append(copy.deepcopy(model))
    write(world_sdf, output / 'world.sdf')
    mass = sum(float(x.text) for x in model.findall('link/inertial/mass'))
    source_mass = sum(float(x.get('value')) for x in robot.findall('link/inertial/mass'))
    assert abs(mass-source_mass) < 1e-12
    report = dict(source=str(source), source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
        mesh_sha256=hashes, plugin_sha256=hashlib.sha256(plugin.read_bytes()).hexdigest(),
        confirmed_axis_distances_m={'OB':.23,'CD':.23,'OC':.05,'BD':.05},
        geometry_basis='User confirmed OB=230 mm, BD=50 mm; OC=50 mm from URDF, CD=230 mm from CAD export rounded coordinates.',
        proxy_mass_fraction=.05,
        source_mass_kg=source_mass, generated_mass_kg=mass,
        anchor_big=anchor_big.tolist(), anchor_small=anchor_small.tolist(),
        link_origin_adjustment_m={n:float(np.linalg.norm(calibrated[n][:3,3]-original[n][:3,3])) for n in calibrated},
        exercise=args.exercise, closure_enabled=not args.disable_closure, step_s=args.step, engine=args.engine,
        limits='Fixed base, fixed rotors, no collisions; force PD is a simulation servo, not a validated TS20 model.',
        flight_ready=False, status='generated_unvalidated')
    (output / 'preparation.json').write_text(json.dumps(report,indent=2)+'\n')
    (output / 'open.sh').write_text('#!/usr/bin/env bash\nset -e\n'
        + 'export GZ_PARTITION=x650-cad-' + stamp + '\n'
        + 'exec gz sim -r --physics-engine gz-physics-' + args.engine + '-plugin '
        + '\"' + str(output / 'world.sdf') + '\"\n')
    print(output)


if __name__ == '__main__':
    main()
