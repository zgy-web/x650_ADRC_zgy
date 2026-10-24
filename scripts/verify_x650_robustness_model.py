#!/usr/bin/env python3
"""Read-only admission check for a separately generated X650 rigid mismatch.

This binds a changed Gazebo plant to the original nominal controller inputs.
It never edits model resources, modifies PX4, installs models or starts flight.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(Path(__file__).resolve().parent))
from prepare_x650_robustness_models import INERTIA_FIELDS, LINKS, SCALES, verify_scaled_tree


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def identity(path):
    path = Path(path).resolve(strict=True)
    return {'path': str(path), 'sha256': sha(path), 'bytes': path.stat().st_size}


def read(path):
    return json.loads(Path(path).read_text(encoding='utf-8'))


def require(value, message):
    if not value:
        raise ValueError(message)


def bound_identity(binding, label):
    require(isinstance(binding, dict) and set(binding) == {'path','sha256','bytes'},
            'Complete identity required: '+label)
    require(isinstance(binding['path'], str) and Path(binding['path']).is_absolute(),
            'Absolute identity path required: '+label)
    path = Path(binding['path']).resolve(strict=True)
    require(identity(path) == binding, 'Bound file changed: '+label)
    return path


def _resolve_model(path):
    sys.path.insert(0, str(ROOT/'src'))
    from amsim.modeling.manager import resolve
    return resolve(str(path))


def _values(link):
    return {'mass': float(link.findtext('inertial/mass')),
            **{key: float(link.findtext('inertial/inertia/'+key)) for key in INERTIA_FIELDS}}


def verify_ledger(report, original_tree, actual_tree):
    scale = report['scale']
    require(not isinstance(scale, bool) and scale in SCALES, 'Unreviewed physical scale')
    require(report['mass_bias_fraction'] == scale-1., 'Mass bias declaration changed')
    verify_scaled_tree(original_tree, actual_tree, scale=scale)
    original = {x.get('name'): x for x in original_tree.findall('model/link')}
    actual = {x.get('name'): x for x in actual_tree.findall('model/link')}
    require(set(original) == set(actual) == LINKS, 'Original seven-link identity changed')
    ledger = report.get('per_link')
    require(isinstance(ledger, list) and len(ledger) == 7
            and {x.get('link') for x in ledger} == LINKS, 'Per-link ledger is incomplete or duplicated')
    for row in ledger:
        name = row['link']
        require(row == {'link': name, 'nominal': _values(original[name]),
                        'actual': _values(actual[name]), 'inertial_pose_preserved': True},
                'Per-link ledger disagrees with actual SDF: '+name)
    nominal_mass = sum(_values(link)['mass'] for link in original.values())
    actual_mass = sum(_values(link)['mass'] for link in actual.values())
    require(math.isclose(nominal_mass, report['nominal_total_mass_kg'], rel_tol=0, abs_tol=1e-12)
            and math.isclose(actual_mass, report['total_mass_kg'], rel_tol=0, abs_tol=1e-12)
            and math.isclose(actual_mass, nominal_mass*scale, rel_tol=0, abs_tol=1e-12),
            'Actual aggregate mass disagrees with scaled rigid ledger')
    return nominal_mass, actual_mass


def verify_ts20(nominal_plant, actual_plant, original_tree, actual_tree):
    require(nominal_plant.get('schema') == 'x650_ts20_conditional_plant_v1'
            and actual_plant.get('schema') == 'x650_ts20_robustness_plant_v1', 'Explicit original and mismatch TS20 plant schemas required')
    for key in ('parameters','plugin_library','plugin_library_sha256','plugin_source','plugin_source_sha256',
                'candidate_profile_sha256','source_manifest_sha256','clock_contract'):
        require(actual_plant.get(key) == nominal_plant.get(key), 'TS20 identity/response changed: '+key)
    library = Path(nominal_plant['plugin_library']).resolve(strict=True)
    source = Path(nominal_plant['plugin_source']).resolve(strict=True)
    require(sha(library) == nominal_plant['plugin_library_sha256'], 'Bound TS20 plugin library changed')
    require(sha(source) == nominal_plant['plugin_source_sha256'], 'Bound TS20 plugin source changed')
    for tree in (original_tree, actual_tree):
        plugins = [p for p in tree.findall('model/plugin') if p.get('name') == 'ts20interface::TS20Interface']
        require(len(plugins) == 1, 'Exactly one physical TS20 interface required')
        plugin = plugins[0]
        require(plugin.get('filename') == nominal_plant['plugin_library'], 'Actual TS20 plugin filename differs')
        for key,value in nominal_plant['parameters'].items():
            nodes = plugin.findall(key)
            require(len(nodes) == 1 and float(nodes[0].text) == value,
                    'Actual TS20 SDF parameter differs: '+key)
    return {'plugin_library': identity(library), 'plugin_source': identity(source),
            'parameters': nominal_plant['parameters']}


def verify_model(report_path, expected_scale=None):
    report_path = Path(report_path).resolve(strict=True)
    report = read(report_path)
    require(report_path.name == 'robustness_manifest.json'
            and report.get('kind') == 'x650_uniform_rigid_mismatch_v1'
            and report.get('simulation_only') is True, 'Explicit simulation rigidity mismatch report required')
    scale = report.get('scale')
    require(not isinstance(scale, bool) and scale in SCALES, 'Unreviewed physical scale')
    if expected_scale is not None:
        require(not isinstance(expected_scale, bool) and expected_scale in SCALES
                and scale == expected_scale, 'Physical scale differs from the selected case')
    directory = report_path.parent
    spec = _resolve_model(directory/'import_manifest.json')
    imported, plant = read(directory/'import_manifest.json'), read(directory/'plant_manifest.json')
    require(spec.model_name == report.get('model_name') and spec.sha256 == report.get('model_sha256')
            and directory.name == spec.model_name and spec.px4_sys_autostart == report.get('px4_sys_autostart') == 4996,
            'Derived model registry/SDF/report identity mismatch')
    require(imported.get('rigidity_mismatch_manifest_sha256') == sha(report_path)
            and plant.get('rigidity_mismatch_manifest_sha256') == sha(report_path)
            and plant.get('model_name') == spec.model_name and plant.get('model_sha256') == spec.sha256,
            'Mismatch report is not bound by the actual model and plant manifests')
    native = report.get('native_validation', {})
    require(native.get('passed') is True and native.get('validated_sdf_sha256') == spec.sha256,
            'Native validation does not bind actual derived SDF')
    source_manifest = bound_identity(report.get('nominal_model_manifest'), 'nominal model manifest')
    source_sdf = bound_identity(report.get('nominal_sdf'), 'nominal SDF')
    source_plant_path = bound_identity(report.get('nominal_plant_manifest'), 'nominal TS20 plant')
    preparation = bound_identity(report.get('nominal_preparation_report'), 'nominal controller preparation')
    airframe = bound_identity(report.get('unchanged_airframe_source'), 'original PX4 airframe')
    generator = bound_identity(report.get('generator'), 'original derivation source')
    nominal = _resolve_model(source_manifest)
    require(nominal.model_name == 'amsim_urdf_x650_2link_ts20_damped_v1_flight'
            and source_manifest.parent == source_sdf.parent == source_plant_path.parent
            and nominal.sdf_path.resolve() == source_sdf and nominal.sha256 == sha(source_sdf)
            and nominal.px4_sys_autostart == spec.px4_sys_autostart,
            'Explicit original nominal plant required')
    require((directory/'nominal_import_manifest.json').read_bytes() == source_manifest.read_bytes()
            and (directory/'nominal_plant_manifest.json').read_bytes() == source_plant_path.read_bytes(),
            'Archived nominal manifest bytes differ')
    original_tree, actual_tree = ET.parse(source_sdf), ET.parse(spec.sdf_path)
    nominal_mass, actual_mass = verify_ledger(report, original_tree, actual_tree)
    require(math.isclose(spec.total_mass_kg, actual_mass, rel_tol=0, abs_tol=1e-12), 'Registry physical mass differs')
    source_plant = read(source_plant_path)
    require(source_plant.get('preparation_report_sha256') == sha(preparation)
            and plant.get('nominal_preparation_report_sha256') == sha(preparation),
            'Original controller preparation binding changed')
    require(math.isclose(read(preparation).get('total_mass_kg', -1), nominal_mass, rel_tol=0, abs_tol=1e-12),
            'Controller report is no longer nominal')
    plugin = verify_ts20(source_plant, plant, original_tree, actual_tree)
    original_files = {p.relative_to(source_manifest.parent).as_posix(): sha(p)
                      for p in source_manifest.parent.rglob('*') if p.is_file()}
    require(original_files == report.get('source_model_files_sha256'), 'Nominal source file set or bytes changed')
    changed = {'model.sdf','model.config','plant_manifest.json','import_manifest.json','.amsim_install.json'}
    for relative,digest in original_files.items():
        if relative not in changed:
            require(sha(directory/relative) == digest, 'Physical resource changed: '+relative)
    require(report.get('prior_flight_gates_reused') is False
            and report.get('new_physical_run_required') is True
            and report.get('claims', {}).get('loaded_motor_response_modeled_from_joint_inertia') is False
            and report.get('claims', {}).get('delay_injection') is False,
            'Mismatch experiment scope or qualification was misrepresented')
    return {'verified': True, 'model_name': spec.model_name, 'model_sha256': spec.sha256,
            'scale': scale, 'nominal_total_mass_kg': nominal_mass, 'total_mass_kg': actual_mass,
            'report': identity(report_path), 'nominal_model_manifest': identity(source_manifest),
            'nominal_preparation_report': identity(preparation), 'unchanged_airframe_source': identity(airframe),
            'generator': identity(generator), 'verifier': identity(Path(__file__)), 'ts20': plugin,
            'all_nonmass_sdf_semantics_preserved': True, 'original_resource_bytes_preserved': True,
            'loaded_motor_response_qualified': False, 'hardware_qualified': False}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('report', type=Path)
    parser.add_argument('--expected-scale', type=float, choices=SCALES)
    args = parser.parse_args(argv)
    print(json.dumps(verify_model(args.report, args.expected_scale), indent=2, allow_nan=False))


if __name__ == '__main__':
    main()
