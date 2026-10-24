#!/usr/bin/env python3
"""Derive immutable physical mass/inertia mismatch plants from the accepted X650.

The nominal preparation report, controller, PX4 parameters and TS20 response
parameters remain unchanged. This is a rigid-body robustness experiment, not a
new loaded-motor identification: the TS20 velocity-interface surrogate does not
derive its internal response from Gazebo joint inertia.
"""
from __future__ import annotations

import argparse
from copy import deepcopy
import hashlib
import json
import math
import os
from pathlib import Path
import re
import shutil
import subprocess
import sys
import tempfile
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
SCALES = (.85, 1.15)
LINKS = {'base_link', 'arm_link_1', 'arm_link_2', 'link_002', 'link_003', 'link_004', 'link_005'}
INERTIA_FIELDS = ('ixx', 'ixy', 'ixz', 'iyy', 'iyz', 'izz')


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def identity(path):
    path = Path(path).resolve(strict=True)
    return {'path': str(path), 'sha256': sha(path), 'bytes': path.stat().st_size}


def read(path):
    return json.loads(Path(path).read_text(encoding='utf-8'))


def write(path, value):
    Path(path).write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False)+'\n', encoding='utf-8')


def _xml_contract(node):
    """Ignore serialization whitespace only, preserving every semantic field."""
    return (node.tag, tuple(sorted(node.attrib.items())), (node.text or '').strip(),
            tuple(_xml_contract(child) for child in node))


def _rename(model, old_name, new_name):
    model.set('name', new_name)
    for node in model.iter():
        for key in ('relative_to', 'expressed_in', 'attached_to'):
            value = node.get(key)
            if value == old_name:
                node.set(key, new_name)
            elif value and value.startswith(old_name+'::'):
                node.set(key, new_name+value[len(old_name):])
        if node.tag == 'uri' and node.text and node.text.startswith('model://'+old_name+'/'):
            node.text = 'model://'+new_name+'/'+node.text[len('model://'+old_name+'/'):]


def single_model(tree):
    root = tree.getroot()
    models = root.findall('model')
    if root.tag != 'sdf' or len(models) != 1 or models[0].find('model') is not None:
        raise ValueError('Exactly one non-nested SDF model is required')
    return models[0]


def _inertial_values(link):
    inertial = link.find('inertial')
    if inertial is None or len(link.findall('inertial')) != 1:
        raise ValueError('Every original rigid link requires exactly one inertial')
    inertia = inertial.find('inertia')
    if inertia is None or set(x.tag for x in inertia) != set(INERTIA_FIELDS) or len(inertia) != 6:
        raise ValueError('All six original inertia tensor components must be explicit')
    nodes = {'mass': inertial.find('mass'), **{key: inertia.find(key) for key in INERTIA_FIELDS}}
    if any(node is None for node in nodes.values()):
        raise ValueError('Incomplete rigid inertia')
    values = {key: float(node.text) for key, node in nodes.items()}
    if not all(math.isfinite(x) for x in values.values()) or min(values[key] for key in ('mass','ixx','iyy','izz')) <= 0:
        raise ValueError('Original rigid mass and inertia must be finite and positive')
    a,b,c,d,e,f = (values[key] for key in INERTIA_FIELDS)
    if a*d-b*b <= 0 or a*d*f+2*b*c*e-a*e*e-d*c*c-f*b*b <= 0:
        raise ValueError('Original inertia must be positive definite')
    return nodes, values


def scale_tree(source_tree, *, scale, model_name):
    """Return a copy and a complete per-link ledger; source is never mutated."""
    if isinstance(scale, bool) or scale not in SCALES:
        raise ValueError('Only the reviewed 0.85 and 1.15 uniform rigid scales are permitted')
    if not isinstance(model_name, str) or re.fullmatch(r'amsim_urdf_[A-Za-z0-9_]+', model_name) is None:
        raise ValueError('An explicit safe imported model name is required')
    tree = deepcopy(source_tree)
    model = single_model(tree)
    old_name = model.get('name')
    if old_name == model_name:
        raise ValueError('Robustness model must have an independent identity')
    links = model.findall('link')
    if len(links) != 7 or {link.get('name') for link in links} != LINKS:
        raise ValueError('Expected exactly the seven accepted X650/two-link rigid links')
    ledger = []
    for link in links:
        nodes, before = _inertial_values(link)
        after = {key: value*scale for key, value in before.items()}
        for key, value in after.items():
            nodes[key].text = format(value, '.17g')
        ledger.append({'link': link.get('name'), 'nominal': before, 'actual': after,
                       'inertial_pose_preserved': True})
    _rename(model, old_name, model_name)
    verify_scaled_tree(source_tree, tree, scale=scale)
    return tree, ledger


def verify_scaled_tree(source_tree, derived_tree, *, scale):
    """Independent allow-list check: no collision, sensor, rotor or TS20 edit."""
    before, after = deepcopy(source_tree), deepcopy(derived_tree)
    original, modified = single_model(before), single_model(after)
    if original.get('name') == modified.get('name'):
        raise ValueError('Independent physical model identity is required')
    _rename(modified, modified.get('name'), original.get('name'))
    old_links, new_links = original.findall('link'), modified.findall('link')
    if len(old_links) != len(new_links):
        raise ValueError('Rigid topology changed')
    for left, right in zip(old_links, new_links):
        old_nodes, old_values = _inertial_values(left)
        new_nodes, new_values = _inertial_values(right)
        for key in old_nodes:
            if new_values[key] != old_values[key]*scale:
                raise ValueError('Unexpected physical scaling: '+str(left.get('name'))+'/'+key)
            new_nodes[key].text = old_nodes[key].text
    if _xml_contract(before.getroot()) != _xml_contract(after.getroot()):
        raise ValueError('Derived SDF changed something outside mass/inertia and model identity')
    return True


def validate_sdf(path):
    result = subprocess.run(['gz', 'sdf', '-k', str(path)], capture_output=True, text=True, timeout=60)
    output = (result.stdout+result.stderr).strip()
    if result.returncode or 'Valid.' not in output:
        raise RuntimeError('Native SDF validation failed: '+output[-4000:])
    return {'passed': True, 'command': ['gz', 'sdf', '-k', str(path)], 'output': output}


def derive(source_dir, source_preparation, destination, *, scale, model_name, source_airframe):
    source_dir, source_preparation = Path(source_dir).resolve(strict=True), Path(source_preparation).resolve(strict=True)
    source_airframe, destination = Path(source_airframe).resolve(strict=True), Path(destination).resolve()
    if (destination.exists() or destination.name != model_name or source_dir == destination
            or source_dir in destination.parents or destination in source_dir.parents):
        raise ValueError('New, independently named output must be outside the original model')
    sys.path.insert(0, str(ROOT/'src'))
    from amsim.modeling.manager import resolve
    source_spec = resolve(str(source_dir/'import_manifest.json'))
    original_manifest, original_plant = read(source_dir/'import_manifest.json'), read(source_dir/'plant_manifest.json')
    original_report = read(source_preparation)
    if (source_spec.model_name != 'amsim_urdf_x650_2link_ts20_damped_v1_flight'
            or source_spec.px4_sys_autostart != 4996
            or original_plant.get('schema') != 'x650_ts20_conditional_plant_v1'
            or original_plant.get('model_sha256') != source_spec.sha256
            or original_plant.get('preparation_report_sha256') != sha(source_preparation)
            or not math.isclose(original_report.get('total_mass_kg', -1), source_spec.total_mass_kg, rel_tol=0, abs_tol=1e-12)
            or any(original_plant.get('parameters', {}).get(k) != v for k,v in {'kp':6.,'kd':.25,'bus_period_s':.005}.items())):
        raise ValueError('The complete original accepted damped TS20 nominal plant is required')
    source_ids = {p.relative_to(source_dir).as_posix(): sha(p) for p in source_dir.rglob('*') if p.is_file()}
    source_tree = ET.parse(source_dir/'model.sdf')
    tree, ledger = scale_tree(source_tree, scale=scale, model_name=model_name)
    total_mass = sum(x['actual']['mass'] for x in ledger)
    if not math.isclose(total_mass, source_spec.total_mass_kg*scale, abs_tol=1e-12):
        raise ValueError('Aggregate mass scaling differs from its rigid ledger')
    destination.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix='.x650-robust-', dir=destination.parent) as temporary:
        staged = Path(temporary)/model_name
        shutil.copytree(source_dir, staged, ignore=shutil.ignore_patterns('.amsim_install.json'))
        (staged/'nominal_import_manifest.json').write_bytes((source_dir/'import_manifest.json').read_bytes())
        (staged/'nominal_plant_manifest.json').write_bytes((source_dir/'plant_manifest.json').read_bytes())
        ET.indent(tree, space='  ')
        tree.write(staged/'model.sdf', encoding='utf-8', xml_declaration=True)
        config = ET.parse(staged/'model.config')
        config.getroot().find('name').text = model_name
        description = config.getroot().find('description')
        if description is None:
            description = ET.SubElement(config.getroot(), 'description')
        description.text = 'Simulation rigid mass/inertia mismatch; nominal upper controller and TS20 interface unchanged.'
        config.write(staged/'model.config', encoding='utf-8', xml_declaration=True)
        validation = validate_sdf(staged/'model.sdf')
        validation['validated_sdf_sha256'] = sha(staged/'model.sdf')
        report = {'kind': 'x650_uniform_rigid_mismatch_v1', 'simulation_only': True,
            'model_name': model_name, 'model_sha256': sha(staged/'model.sdf'),
            'scale': scale, 'mass_bias_fraction': scale-1., 'total_mass_kg': total_mass,
            'nominal_total_mass_kg': source_spec.total_mass_kg, 'per_link': ledger,
            'nominal_model_manifest': identity(source_dir/'import_manifest.json'),
            'nominal_sdf': identity(source_dir/'model.sdf'),
            'nominal_plant_manifest': identity(source_dir/'plant_manifest.json'),
            'nominal_preparation_report': identity(source_preparation),
            'unchanged_airframe_source': identity(source_airframe),
            'generator': identity(Path(__file__)), 'px4_sys_autostart': 4996,
            'native_validation': validation,
            'unchanged': {'geometry': True, 'inertial_poses': True, 'joint_axes_limits': True,
                'collision': True, 'visual_meshes': True, 'sensor_configuration': True,
                'rotor_dynamics': True, 'ts20_parameters_and_library': True,
                'ts20_bus_and_internal_delay': True, 'px4_parameters': True,
                'controller_preparation_report': True, 'controller_weights_and_prediction': True,
                'source_model_files': True},
            'only_sdf_semantic_changes': ['model_identity_and_self_resource_references',
                'each_of_seven_link_masses_times_scale', 'each_of_six_link_inertia_components_times_scale'],
            'source_model_files_sha256': source_ids,
            'controller_model_policy': 'Read original nominal preparation bytes; actual physical total mass intentionally differs. Do not pass this report to ArmModel.from_report.',
            'prior_flight_gates_reused': False, 'new_physical_run_required': True,
            'claims': {'center_of_mass_preserved_at_every_joint_configuration': True,
                'loaded_motor_response_modeled_from_joint_inertia': False,
                '24v_hardware_qualified': False, 'delay_injection': False,
                'scope': 'All Gazebo rigid masses and COM inertias scaled uniformly. TS20 is the unchanged velocity-interface surrogate: internal actuator response is not a joint-load torque model.'}}
        write(staged/'robustness_manifest.json', report)
        plant = deepcopy(original_plant)
        plant.update(schema='x650_ts20_robustness_plant_v1', model_name=model_name,
            model_sha256=report['model_sha256'], total_mass_kg=total_mass,
            geometry_inertia_collision_preserved=False, rigid_geometry_collision_preserved=True,
            rigidity_mismatch_manifest_sha256=sha(staged/'robustness_manifest.json'),
            nominal_preparation_report_sha256=sha(source_preparation),
            native_validation=validation, prior_flight_gates_reused=False,
            derivation='Uniform physical rigid mass/inertia scaling; nominal controller/TS20/PX4 unchanged')
        plant['claims'] = {**plant.get('claims', {}), 'joint_inertias_changed': True, 'mass_inertias_preserved': False}
        plant['artifacts'] = {p.relative_to(staged).as_posix(): sha(p) for p in staged.rglob('*')
                              if p.is_file() and p.name not in ('plant_manifest.json','import_manifest.json')}
        write(staged/'plant_manifest.json', plant)
        artifacts = {p.relative_to(staged).as_posix(): sha(p) for p in staged.rglob('*')
                     if p.is_file() and p.name != 'import_manifest.json'}
        imported = deepcopy(original_manifest)
        imported.update(model_name=model_name, model_sha256=artifacts['model.sdf'], sdf_sha256=artifacts['model.sdf'],
            total_mass_kg=total_mass, resources=artifacts, artifacts=artifacts,
            native_validation=validation, flight_validated=False,
            conditional_actuator_plant_manifest_sha256=artifacts['plant_manifest.json'],
            rigidity_mismatch_manifest_sha256=artifacts['robustness_manifest.json'],
            parent_flight_manifest_sha256=sha(source_dir/'import_manifest.json'), prior_flight_gates_reused=False,
            flight_ready_meaning='Simulation binding only. New explicit rigid mismatch; historical physical flight certificates do not qualify this plant.')
        write(staged/'import_manifest.json', imported)
        resolve(str(staged/'import_manifest.json'))
        if source_ids != {p.relative_to(source_dir).as_posix(): sha(p) for p in source_dir.rglob('*') if p.is_file()}:
            raise ValueError('Original nominal model changed during derivation')
        staged.rename(destination)
    return {'model': model_name, 'model_manifest': identity(destination/'import_manifest.json'),
            'physical_report': identity(destination/'robustness_manifest.json'),
            'physical_plant_manifest': identity(destination/'plant_manifest.json'),
            'nominal_preparation_report': identity(source_preparation),
            'unchanged_airframe_source': identity(source_airframe),
            'model_sha256': sha(destination/'model.sdf'), 'scale': scale, 'total_mass_kg': total_mass,
            'source_files_unchanged': True, 'native_validation_passed': True}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source-model-dir', type=Path, required=True)
    parser.add_argument('--source-preparation', type=Path, required=True)
    parser.add_argument('--source-airframe', type=Path, required=True)
    parser.add_argument('--output-dir', type=Path, required=True)
    parser.add_argument('--model-name', required=True)
    parser.add_argument('--scale', type=float, choices=SCALES, required=True)
    args = parser.parse_args(argv)
    print(json.dumps(derive(args.source_model_dir, args.source_preparation, args.output_dir,
        scale=args.scale, model_name=args.model_name, source_airframe=args.source_airframe), indent=2, allow_nan=False))


if __name__ == '__main__':
    main()
