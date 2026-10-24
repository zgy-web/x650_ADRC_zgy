"""Fail-closed ground qualification for the owning C++ simulation pipeline.

This module only reads existing evidence. It never invokes an optimizer, starts
ROS, publishes a command, or turns software replay into a flight certificate.
"""
from pathlib import Path
import argparse
import hashlib
import json
import math


def require(ok, message):
    if not ok:
        raise ValueError(message)


def sha(path):
    digest = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(chunk)
    return digest.hexdigest()


def identity(path):
    path = Path(path).resolve(strict=True)
    return {'path': str(path), 'sha256': sha(path), 'bytes': path.stat().st_size}


def _object(pairs):
    result = {}
    for key, value in pairs:
        require(key not in result, 'Duplicate JSON field: ' + key)
        result[key] = value
    return result


def read(path):
    def invalid(value):
        raise ValueError('Nonfinite JSON value: ' + value)
    return json.loads(Path(path).read_text(encoding='utf-8'), object_pairs_hook=_object,
                      parse_constant=invalid)


class Evidence:
    def __init__(self):
        self.checked = {}

    def artifact(self, value):
        require(isinstance(value, dict) and 'path' in value and 'sha256' in value,
                'Artifact identity requires path and SHA256')
        path = Path(value['path'])
        require(path.is_absolute() and path.is_file(), 'Missing absolute evidence: ' + str(path))
        path = path.resolve(strict=True)
        key = str(path)
        actual = identity(path) if key not in self.checked else self.checked[key]
        require(value['sha256'] == actual['sha256'], 'Artifact bytes changed: ' + key)
        if 'bytes' in value:
            require(type(value['bytes']) is int and value['bytes'] == actual['bytes'], 'Artifact size differs: ' + key)
        self.checked[key] = actual
        return path

    def tree(self, value):
        if isinstance(value, dict):
            if 'path' in value and 'sha256' in value:
                self.artifact(value)
            for child in value.values():
                self.tree(child)
        elif isinstance(value, list):
            for child in value:
                self.tree(child)

    def document(self, value):
        result = read(self.artifact(value))
        self.tree(result)
        return result


def same_artifact(left, right):
    return (Path(left['path']).resolve() == Path(right['path']).resolve()
            and left['sha256'] == right['sha256'])


def physical_task(value):
    result = dict(value)
    result.pop('numerical_backend', None)
    return result


def finite_bounded(value, bound):
    return type(value) in (float, int) and math.isfinite(value) and 0 <= value <= bound


def verify_numerical(report, total, startup, evidence, *, matrices):
    require(report.get('kind') == 'cpp_owning_core_equivalence_v1'
            and report.get('complete') is True and report.get('pass') is True
            and report.get('libraries_unchanged') is True
            and report.get('runtime_python_calls') == 0
            and report.get('passed') == report.get('total') == total,
            'Complete C++ numerical proof missing')
    for name, proof_name in [('core_library', 'core_library'), ('optimization_library', 'optimization_library'),
                             ('predictor_library', 'predictor')]:
        require(same_artifact(startup[name], report[proof_name]), 'Numerical library binding differs: ' + name)
    cases = evidence.document(report['cases'])
    require(cases.get('complete') is True and cases.get('source_files_unchanged') is True
            and len(cases.get('rows', [])) == total, 'Raw input/oracle manifest incomplete')
    require(len({str(Path(v['path']).resolve()) for v in cases['rows']}) == total, 'Raw numerical inputs repeated')
    rows = report.get('rows', [])
    require(len(rows) == total and [r.get('index') for r in rows] == list(range(total)), 'Numerical rows missing/repeated')
    for row in rows:
        require(row.get('pass_') is True and row.get('exit_code') == 0, 'Numerical row rejected')
        for key in ('normalized_plan_peak', 'predicted_state_peak', 'predicted_tip_peak'):
            require(finite_bounded(row.get(key), 1e-7), 'Numerical difference exceeds original tolerance: ' + key)
        require(finite_bounded(row.get('objective_relative'), 1e-8), 'Numerical objective exceeds original tolerance')
        quality = row.get('independently_recomputed_quality', {})
        require(finite_bounded(quality.get('primal_violation'), 2e-7)
                and finite_bounded(quality.get('kkt_stationarity_inf'), .002), 'Independent original-row/KKT proof missing')
        require(finite_bounded(row.get('native_compute_ms'), 50)
                and finite_bounded(row.get('core_step_wall_ms'), 50), 'Production core exceeded 50 ms')
        if matrices:
            errors = row.get('matrix_peak_errors', {})
            require(set(errors) == {'A', 'b', 'H', 'g', 'C', 'lo', 'hi'}
                    and all(finite_bounded(v, 1e-9) for v in errors.values())
                    and row.get('mit_wire_fields_equal') is True
                    and row.get('independent_matrix_oracle') is True, 'Full matrix/MIT oracle proof missing')


def verify_worker(report, startup, evidence, binaries):
    require(report.get('kind') == 'cpp_worker_actual_ros_software_replay_v1'
            and report.get('complete') is True and report.get('passed') is True
            and report.get('no_actuator_publishers') is True
            and report.get('physical_acceptance') is False
            and report.get('control_time_dt_and_actual_history_unchanged') is True,
            'Actual ROS worker software proof incomplete')
    require(same_artifact(report['core_library'], startup['core_library']), 'ROS worker core differs')
    checked_startup = evidence.document(report['startup_manifest'])
    require({k: v for k, v in startup.items() if k != 'task_config'} ==
            {k: v for k, v in checked_startup.items() if k != 'task_config'},
            'Live startup differs from tested worker configuration/libraries/sources')
    require(physical_task(evidence.document(startup['task_config'])) ==
            physical_task(evidence.document(checked_startup['task_config'])), 'Physical task differs from worker proof')
    require(binaries.get(str(Path(report['binary']['path']).resolve())) == report['binary']['sha256'], 'Installed worker differs from ROS proof')
    cases = report.get('cases', [])
    require(len(cases) == 2 and {c.get('name') for c in cases} == {'complete_history_sigint', 'invalid_hash_latched'}, 'ROS positive/negative cases missing')
    for case in cases:
        good = case['name'] == 'complete_history_sigint'
        count = 1056 if good else 3
        require(case.get('passed') is True and case.get('child_exit') == (0 if good else 1), 'ROS process lifecycle rejected')
        final = case.get('final', {})
        require(final.get('requests') == count and final.get('accepted') == (1056 if good else 2)
                and final.get('faulted') is (not good) and final.get('no_python_gc_or_pickle') is True,
                'ROS worker completion/latch counts differ')
        recorder = case.get('recorder', {})
        require(recorder.get('complete') is True and recorder.get('failed') is False
                and recorder.get('dropped') == 0 and recorder.get('written') == recorder.get('offered') == count
                and len(case.get('records', [])) == count, 'ROS raw evidence lost')
        if good:
            rows = case.get('rows', [])
            require(len(rows) == count and [r.get('sequence') for r in rows] == list(range(1, count+1)), 'ROS historical rows missing')
            for row in rows:
                require(row.get('status') == 0 and row.get('absolute_errors')
                        and all(finite_bounded(v, 1e-7) for v in row['absolute_errors'].values()), 'ROS command/PVA disagreement')
                require(finite_bounded(row.get('native_callback_ms'), 50), 'Complete worker callback exceeded 50 ms')
        for sequence, record_id in enumerate(case['records'], 1):
            record = evidence.document(record_id)
            require(record.get('kind') == 'cpp_full_pipeline_call_v1' and record.get('sequence') == sequence,
                    'Unknown, duplicated or reordered native record')
    return checked_startup


def verify(manifest_path, *, task_path, startup_path, startup_sha256, protocol_path, binaries):
    evidence = Evidence()
    manifest = read(manifest_path)
    evidence.tree(manifest)
    required = {'kind', 'simulation_only', 'original_task', 'task_config', 'startup_manifest', 'protocol',
                'matrix_proof', 'history_proof', 'worker_proof', 'source_parity_proof', 'source_build_proof', 'transport_proof'}
    require(set(manifest) == required and manifest['kind'] == 'x650_cpp_live_qualification_v1'
            and manifest['simulation_only'] is True, 'Explicit C++ qualification schema required')
    for key, path in [('task_config', task_path), ('startup_manifest', startup_path), ('protocol', protocol_path)]:
        require(same_artifact(manifest[key], identity(path)), 'Selected artifact differs: ' + key)
    require(manifest['startup_manifest']['sha256'] == startup_sha256, 'Selected startup SHA differs')
    task = evidence.document(manifest['task_config'])
    original_task = evidence.document(manifest['original_task'])
    require('numerical_backend' in original_task and 'numerical_backend' not in task
            and task == physical_task(original_task), 'Only numerical_backend removal is authorized')
    startup = evidence.document(manifest['startup_manifest'])
    require(startup.get('kind') == 'cpp_full_pipeline_v1' and startup.get('simulation_only') is True
            and startup.get('trajectory') == {'kind': 'hold'} and same_artifact(startup['task_config'], manifest['task_config']), 'Live startup/task mismatch')
    protocol = evidence.document(manifest['protocol'])
    require(protocol.get('kind') == 'x650_cpp_live_protocol_v1' and protocol.get('simulation_only') is True
            and protocol.get('trajectory_kind') == 'hold' and task.get('stage') == 'tip_hold'
            and task.get('duration_s') == protocol.get('tip_hold_duration_s') == 30., 'Fresh complete hold protocol required')
    for key, expected in [('output_period_s', .02), ('command_age_max_s', .05), ('control_interval_max_s', .05),
                          ('source_wall_age_max_s', .025), ('core_compute_max_s', .05), ('output_interval_max_s', .04)]:
        require(protocol.get(key) == expected, 'Original transport policy changed: ' + key)
    for key, count, matrices in [('matrix_proof', 30, True), ('history_proof', 1056, False)]:
        verify_numerical(evidence.document(manifest[key]), count, startup, evidence, matrices=matrices)
    verify_worker(evidence.document(manifest['worker_proof']), startup, evidence, binaries)
    source = evidence.document(manifest['source_parity_proof'])
    require(source.get('passed') is True and source.get('snapshots') == 1056
            and set(source.get('maximum_absolute_error', {})) == {'p', 'v', 'a', 'R', 'omega', 'state_time', 'joint_time', 'q', 'dq', 'r', 'reference_time', 'bus_origin', 'commands', 'packets'}
            and all(v == 0 for v in source['maximum_absolute_error'].values())
            and source.get('raw_bag_exact_matches') == {'position': 1056, 'attitude': 1056}
            and source.get('raw_bag_missing_by_topic') == {'position': 0, 'attitude': 0}, 'Actual raw-source parity incomplete')
    # Final dynamic-binding source build must explicitly bind the unchanged
    # mathematical parity proof and its new executable/lifecycle evidence.
    build = evidence.document(manifest['source_build_proof'])
    require(build.get('kind') == 'cpp_source_final_qualification_v1' and build.get('passed') is True
            and build.get('complete') is True and same_artifact(build['parity_proof'], manifest['source_parity_proof']), 'Final C++ source qualification missing')
    require(binaries.get(str(Path(build['binary']['path']).resolve())) == build['binary']['sha256'], 'Installed source binary differs from final proof')
    require(build.get('unit_tests_passed') is True and build.get('ros_lifecycle_passed') is True
            and build.get('no_actuator_publishers') is True, 'Final raw source lifecycle/tests missing')
    transport = evidence.document(manifest['transport_proof'])
    require(transport.get('passed') is True and transport.get('zero_consumed') is True
            and binaries.get(str(Path(transport['dispatcher_binary']).resolve())) == transport.get('dispatcher_sha256'), 'Current dispatcher actual-consumption proof required')
    return {'kind': 'cpp_live_ground_admission_v1', 'ready': True, 'physical_hold_qualified': False,
            'qualification_manifest': identity(manifest_path), 'task_sha256': manifest['task_config']['sha256'],
            'startup_sha256': startup_sha256, 'protocol_sha256': manifest['protocol']['sha256'],
            'numerical_cases': 30, 'historical_cases': 1056, 'ros_historical_cases': 1056,
            'source_parity_cases': 1056, 'verified_artifact_count': len(evidence.checked),
            'verified_artifacts': list(evidence.checked.values())}


def prepare(args):
    output = args.output.resolve()
    require(not output.exists(), 'Preparation output must be a new directory')
    old = read(args.old_task)
    require('numerical_backend' in old, 'Original task must explicitly contain the replaced backend binding')
    worker = read(args.worker_proof)
    evidence = Evidence()
    worker_startup = evidence.document(worker['startup_manifest'])
    require(physical_task(old) == physical_task(evidence.document(worker_startup['task_config'])), 'Original task differs from tested physical condition')
    output.mkdir(parents=True)
    new_task = output/'task.json'
    new_task.write_text(json.dumps(physical_task(old), indent=2, allow_nan=False)+'\n', encoding='utf-8')
    startup = dict(worker_startup)
    startup['task_config'] = {key: value for key, value in identity(new_task).items() if key != 'bytes'}
    startup_path = output/'startup_manifest.json'
    startup_path.write_text(json.dumps(startup, indent=2, allow_nan=False)+'\n', encoding='utf-8')
    manifest = {'kind': 'x650_cpp_live_qualification_v1', 'simulation_only': True,
                'original_task': identity(args.old_task), 'task_config': identity(new_task),
                'startup_manifest': identity(startup_path), 'protocol': identity(args.protocol)}
    for key in ('matrix_proof', 'history_proof', 'worker_proof', 'source_parity_proof', 'source_build_proof', 'transport_proof'):
        manifest[key] = identity(getattr(args, key))
    path = output/'qualification_manifest.json'
    path.write_text(json.dumps(manifest, indent=2, allow_nan=False)+'\n', encoding='utf-8')
    print(json.dumps({'prepared_not_admitted': True, 'manifest': identity(path), 'startup': identity(startup_path), 'task': identity(new_task)}))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest='command', required=True)
    p = sub.add_parser('prepare', help='Write new inputs only; never launch simulation or grant acceptance')
    for key in ('old_task', 'output', 'protocol', 'matrix_proof', 'history_proof', 'worker_proof', 'source_parity_proof', 'source_build_proof', 'transport_proof'):
        p.add_argument('--'+key.replace('_', '-'), type=Path, required=True)
    prepare(parser.parse_args())


if __name__ == '__main__':
    main()
