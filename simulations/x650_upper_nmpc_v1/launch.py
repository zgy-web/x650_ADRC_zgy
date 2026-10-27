#!/usr/bin/env python3
"""Admit one fresh upper-controller stage on actual PX4/Gazebo feedback.

--dry-run verifies the existing proofs and physical prerequisites only. A launch
still requires a new complete physical window and independent post-run audit.
"""
from pathlib import Path
import argparse
import hashlib
import json
import os
import subprocess
import sys
import time
import zipfile

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from simulations.x650_upper_nmpc_v1 import qualification as q
from simulations.x650_isolated_live_v1.launch import execution_resources
from simulations.x650_upper_nmpc_v1 import task
from simulations.pid_ff_wholebody_v1.launch import flight_lock, atomic_json

EXECUTION_SCENARIO = 'x650_cpp_live_v1'


def runtime_hashes(startup):
    from ament_index_python.packages import get_package_prefix
    result = {}
    for package, names in {'flyarm_control_gateway': ('state_ingress', 'live_dispatcher', 'consumption_source'),
                            'flyarm_nmpc_cpp': ('isolated_worker',), 'flyarm_state_cpp': ('state_source',)}.items():
        for name in names:
            path = (Path(get_package_prefix(package))/'lib'/package/name).resolve(strict=True)
            result[str(path)] = q.sha(path)
    for key in ('core_library', 'optimization_library', 'predictor_library'):
        path = Path(startup[key]['path']).resolve(strict=True)
        actual = q.sha(path)
        q.require(actual == startup[key]['sha256'], 'Bound library changed: ' + key)
        result[str(path)] = actual
    return result


def runtime_interfaces():
    from flyarm_control_interfaces import msg
    from rclpy.serialization import serialize_message, deserialize_message
    return q.verify_runtime_interfaces(msg, serialize_message, deserialize_message)


def source_hashes(extra_sources=()):
    paths = []
    for name in ('src/amsim', 'ros2_ws/src', 'control_packages/x650_wholebody_v1',
                 'simulations/x650_wholebody_mpc_v1', 'simulations/x650_ts20_hold_v1',
                 'simulations/x650_isolated_live_v1', 'simulations/x650_cpp_live_v1'):
        paths.extend(p for p in (ROOT/name).rglob('*') if p.is_file() and '__pycache__' not in p.parts
                     and p.suffix in ('.py', '.cpp', '.hpp', '.cc', '.hh', '.json', '.msg', '.xml', '.txt', '.npz'))
    paths.extend(ROOT/name for name in ('scripts/amsim', 'scripts/x650_isolated_diagnostic_recorder.py',
                 'scripts/validate_x650_cpp_pipeline.py', 'scripts/export_x650_cpp_inputs.py',
                 'scripts/x650_cpp_core_replay.cpp', 'configs/experiments/x650_cpp_live_v1.json',
                 'scenarios/x650_cpp_live_v1.yaml'))
    for name in extra_sources:
        path = ROOT/name
        q.require(path.exists() and path.resolve().is_relative_to(ROOT.resolve()), 'Missing/outside extra execution source')
        if path.is_dir():
            paths.extend(p for p in path.rglob('*') if p.is_file() and '__pycache__' not in p.parts)
        else:
            paths.append(path)
    return {p.relative_to(ROOT).as_posix(): q.sha(p) for p in sorted(set(paths))}


def archive_sources(out, identities):
    with zipfile.ZipFile(out/'sources.zip', 'x', zipfile.ZIP_DEFLATED) as archive:
        for rel, digest in identities.items():
            data = (ROOT/rel).read_bytes()
            q.require(hashlib.sha256(data).hexdigest() == digest, 'Source changed while archiving: ' + rel)
            archive.writestr(rel, data)


def archive_binaries(out, identities):
    manifest = []
    with zipfile.ZipFile(out/'runtime_binaries.zip', 'x', zipfile.ZIP_DEFLATED) as archive:
        for index, (path, digest) in enumerate(sorted(identities.items())):
            data = Path(path).read_bytes()
            q.require(hashlib.sha256(data).hexdigest() == digest, 'Runtime changed while archiving: ' + path)
            name = f'{index:02d}_'+Path(path).name
            archive.writestr(name, data)
            manifest.append({'path': path, 'sha256': digest, 'bytes': len(data), 'archive_entry': name})
    atomic_json(out/'runtime_binary_manifest.json', manifest)


def main(argv=None):
    task_api, qualification_api = task, q
    default_protocol = None
    launch_prefix = 'x650_upper_launch_'
    extra_sources = ('simulations/x650_upper_nmpc_v1', 'control_packages/x650_nmpc_controller_v1',
        'control_packages/x650_mpc_controller_v1', 'simulations/x650_cpp_multirate_v1',
        'scripts/audit_x650_upper_controller.py')
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--task', type=Path, required=True)
    parser.add_argument('--startup-manifest', type=Path, required=True)
    parser.add_argument('--startup-sha256', required=True)
    parser.add_argument('--qualification-manifest', type=Path, required=True)
    parser.add_argument('--protocol', type=Path, default=default_protocol or ROOT/'configs/experiments/x650_cpp_live_v1.json')
    parser.add_argument('--options', type=Path, required=True)
    parser.add_argument('--dry-run', action='store_true')
    args = parser.parse_args(argv)
    cfg = task.validate_task_config(q.read(args.task))
    scenario = cfg['scenario']
    extra_sources = (*extra_sources, 'simulations/'+scenario, 'scenarios/'+scenario+'.yaml')
    options = q.read(args.options)
    protocol = q.read(args.protocol)
    resources = execution_resources(protocol)
    interfaces = runtime_interfaces()
    startup = q.read(args.startup_manifest)
    binaries = runtime_hashes(startup)
    admission = qualification_api.verify(args.qualification_manifest, task_path=args.task,
                         startup_path=args.startup_manifest, startup_sha256=args.startup_sha256,
                         protocol_path=args.protocol, binaries=binaries, options_path=args.options)
    admission['runtime_interfaces'] = interfaces
    cfg = task_api.validate_task_config(q.read(args.task))
    _, provenance = task_api.preflight(cfg)
    from amsim.scenario_registry import get
    descriptor = get(scenario)
    q.require(descriptor.module == 'simulations/'+scenario+'/run_case.py'
              and descriptor.config['model'] == cfg['model'], 'Explicit C++ scenario physical model differs')
    q.require(not (ROOT/'.amsim/runtime.json').exists(), 'An active runtime already exists')
    if args.dry_run:
        print(json.dumps({k: v for k, v in admission.items() if k != 'verified_artifacts'}, allow_nan=False))
        return 0
    with flight_lock(ROOT):
        q.require(not (ROOT/'.amsim/runtime.json').exists(), 'An active runtime already exists')
        q.require(runtime_hashes(startup) == binaries, 'Runtime changed after admission')
        out = ROOT/'runs'/(launch_prefix+time.strftime('%Y%m%d_%H%M%S')+'_'+str(os.getpid()))
        out.mkdir(parents=True)
        logs = out/'actuator_live'; logs.mkdir()
        for name, source in [('task.json', args.task), ('protocol.json', args.protocol),
                             ('qualification_manifest.json', args.qualification_manifest),
                             ('original_startup_manifest.json', args.startup_manifest), ('options.json', args.options)]:
            (out/name).write_bytes(source.read_bytes())
        # Exact task bytes are copied, so only the startup task path changes.
        # The immutable tested configuration and all library/source bindings
        # remain byte-for-value equal; this derived manifest is archived too.
        owned_startup = dict(startup)
        owned_startup['task_config'] = {key: value for key, value in q.identity(out/'task.json').items() if key != 'bytes'}
        atomic_json(out/'startup_manifest.json', owned_startup)
        before = source_hashes(extra_sources)
        atomic_json(out/'source_manifest.json', before)
        archive_sources(out, before); archive_binaries(out, binaries)
        atomic_json(out/'ground_admission.json', admission)
        launch_inputs = [q.identity(out/name) for name in ('task.json', 'protocol.json',
            'qualification_manifest.json', 'original_startup_manifest.json', 'startup_manifest.json', 'options.json')]
        status = {'complete': False, 'physical_hold_qualified': False,
            'task': cfg, 'task_sha256': q.sha(args.task), 'provenance': provenance,
            'protocol': protocol, 'protocol_sha256': q.sha(args.protocol),
            'qualification_manifest': q.identity(args.qualification_manifest),
            'startup_manifest': q.identity(out/'startup_manifest.json'),
            'startup_derivation': 'Only task_config.path redirected to identical archived task bytes',
            'launch_inputs': launch_inputs,
            'execution_resources': resources, 'runtime_binaries_sha256': binaries,
            'execution_scenario': scenario, 'new_runtime_acceptance_required': True}
        atomic_json(out/'launch.json', status)
        env = os.environ.copy()
        env.update(X650_EXECUTION_ADAPTER='cpp_full_pipeline_v1',
            FLYARM_CPP_PROTOCOL=str(out/'protocol.json'),
            FLYARM_CPP_STARTUP_MANIFEST=str(out/'startup_manifest.json'),
            FLYARM_CPP_STARTUP_SHA256=q.sha(out/'startup_manifest.json'),
            FLYARM_UPPER_OPTIONS=str(out/'options.json'),
            FLYARM_UPPER_OPTIONS_SHA256=q.sha(out/'options.json'),
            FLYARM_EXECUTION_RESOURCES=json.dumps(resources, separators=(',', ':')),
            X650_TS20_TASK_CONFIG=str(out/'task.json'), AMSIM_TS20_LOG_DIR=str(logs),
            AMSIM_SCENARIO_PYTHON='/usr/bin/python3', AMSIM_CHECK_PYTHON='/usr/bin/python3',
            AMSIM_VENV='/root/autodl-tmp/amsim-venv', PYTHONUNBUFFERED='1',
            OPENBLAS_NUM_THREADS='1', OMP_NUM_THREADS='1', MKL_NUM_THREADS='1')
        env['PYTHONPATH'] = '/usr/lib/python3/dist-packages:/root/autodl-tmp/amsim-venv/lib/python3.10/site-packages:'+env.get('PYTHONPATH', '')
        env['PX4_GZ_MODEL_POSE'] = ','.join(str(x) for x in cfg['spawn_pose_enu_xyz_rpy'])
        old = set((ROOT/'runs').glob(scenario+'_*'))
        print('CPP_LAUNCH '+str(out), flush=True)
        try:
            with (out/'flight.log').open('x') as log:
                result = subprocess.run(['taskset', '--cpu-list', ','.join(map(str, resources['background'])),
                    str(ROOT/'scripts/amsim'), 'run', scenario, '--headless'],
                    cwd=ROOT, env=env, stdout=log, stderr=subprocess.STDOUT)
            new = sorted(set((ROOT/'runs').glob(scenario+'_*'))-old)
            status.update(exit_code=result.returncode, run_directories=[str(x) for x in new],
                          runtime_closed=not (ROOT/'.amsim/runtime.json').exists())
            status['complete'] = (result.returncode == 0 and len(new) == 1
                and q.read(new[0]/'result.json').get('success') is True and status['runtime_closed'])
        except BaseException as error:
            status['error'] = repr(error)
            raise
        finally:
            try:
                after = source_hashes(extra_sources)
                status['sources_unchanged'] = after == before
                atomic_json(out/'source_manifest_after.json', after)
                after_binaries = runtime_hashes(startup)
                status['runtime_binaries_unchanged'] = after_binaries == binaries
                atomic_json(out/'runtime_binary_manifest_after.json', after_binaries)
                status['launch_inputs_unchanged'] = all(q.sha(Path(a['path'])) == a['sha256']
                    for a in [admission['qualification_manifest'], *launch_inputs])
            except BaseException as error:
                status['final_identity_error'] = repr(error)
                status['sources_unchanged'] = status['runtime_binaries_unchanged'] = False
            status['complete'] = bool(status['complete'] and status.get('sources_unchanged')
                and status.get('runtime_binaries_unchanged') and status.get('launch_inputs_unchanged'))
            atomic_json(out/'launch.json', status)
        if not status['complete']:
            raise RuntimeError('Complete C++ simulation lifecycle did not finish: '+str(out))
        return 0


if __name__ == '__main__':
    raise SystemExit(main())
