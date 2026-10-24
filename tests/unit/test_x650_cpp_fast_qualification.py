"""Complete historical normal gate and independent fast phase/dt checks."""
from copy import deepcopy
import hashlib
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch
import zipfile
import numpy as np

from simulations.x650_cpp_fast_v1 import qualification as q


def write(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, allow_nan=False), encoding='utf-8')
    return q.identity(path)


def normal_fixture(root):
    launch, run = root/'launch', root/'run'; launch.mkdir(); run.mkdir()
    baseline = dict(stage='normal', controller='mpc', duration_s=32., frozen_cpp_hold={}, cpp_hold_certificate={})
    task = write(launch/'task.json', baseline)
    protocol = q.read(Path(__file__).resolve().parents[2]/'configs/experiments/x650_cpp_normal_v1.json')
    write(launch/'protocol.json', protocol)
    startup = dict(trajectory=q.normal.normal_trajectory(), config={'frozen': True},
        **{k: {'path': str(root/k), 'sha256': 'a'*64} for k in ('core_library', 'optimization_library', 'predictor_library')})
    write(launch/'startup_manifest.json', startup)
    write(run/'controller_task.json', baseline)
    write(run/'controller_metadata.json', dict(task=baseline, complete=True, landed_and_disarmed=True,
        diagnostic_parameters_restored=True, diagnostic_restore_pending=False))
    for name in ('native_trace.csv', 'native_trace_manifest.json', 'px4.ulg', 'isolated_live/lifecycle.json',
                 'isolated_live/live_start.json', 'ts20_state.csv', 'diagnostic_parameters_restored.json'):
        write(run/name, {})
    source = {'worker.cpp': hashlib.sha256(b'original normal worker').hexdigest()}
    write(launch/'source_manifest.json', source); write(launch/'source_manifest_after.json', source)
    with zipfile.ZipFile(launch/'sources.zip', 'w') as z: z.writestr('worker.cpp', b'original normal worker')
    executable = root/'worker'; executable.write_bytes(b'original executable'); bid = q.identity(executable)
    binaries = {bid['path']: bid['sha256']}
    write(launch/'runtime_binary_manifest.json', [dict(bid, archive_entry='worker')])
    write(launch/'runtime_binary_manifest_after.json', binaries)
    with zipfile.ZipFile(launch/'runtime_binaries.zip', 'w') as z: z.writestr('worker', executable.read_bytes())
    write(launch/'launch.json', dict(complete=True, runtime_closed=True, sources_unchanged=True,
        runtime_binaries_unchanged=True, launch_inputs_unchanged=True, exit_code=0,
        execution_scenario='x650_cpp_normal_v1', run_directories=[str(run.resolve())], task=baseline,
        protocol=protocol, runtime_binaries_sha256=binaries))
    artifacts = [q.identity(p) for base in (launch, run) for p in sorted(base.rglob('*')) if p.is_file()]
    cert = dict(kind='x650_cpp_normal_stage_certificate_v1', stage='normal', passed=True, complete=True,
        certificate=True, physical_acceptance=True, full_window_ulog_verified=True, parameter_restore_verified=True,
        native_ground_verified=True, actuator_log_verified=True, window_sim_s=[10., 42.],
        launch_directory=str(launch), run_directory=str(run), artifact_identities=artifacts,
        artifacts={a['path']: a['sha256'] for a in artifacts})
    prior = {'startup': startup, 'protocol': q.read(Path(__file__).resolve().parents[2]/'configs/experiments/x650_cpp_live_v1.json')}
    return baseline, cert, write(root/'certificate.json', cert), {k: v for k, v in task.items() if k != 'bytes'}, prior


class HistoricalNormalTests(unittest.TestCase):
    def check(self, root, mutate=None):
        base, cert, bind, task, prior = normal_fixture(root)
        if mutate:
            mutate(cert, root)
            bind = write(root/'certificate.json', cert)
        with patch('simulations.x650_cpp_fast_v1.task.normal_config', return_value=base), \
                patch.object(q, 'verify_hold_gate', return_value=prior):
            return q.verify_normal_gate(bind, task)

    def test_original_normal_survives_intentional_active_worker_upgrade(self):
        with tempfile.TemporaryDirectory() as folder:
            result = self.check(Path(folder), lambda c, r: (r/'worker').write_bytes(b'new fast executable'))
            self.assertTrue(result['verified']); self.assertEqual(result['window_sim_s'], [10., 42.])

    def test_short_wrong_stage_and_missing_raw_evidence_rejected(self):
        mutations = [lambda c, r: c.update(complete=False), lambda c, r: c.update(window_sim_s=[10., 41.99]),
            lambda c, r: c.update(stage='tip_hold'), lambda c, r: c.update(actuator_log_verified=False),
            lambda c, r: (r/'run/px4.ulg').write_bytes(b'changed')]
        for i, mutation in enumerate(mutations):
            with self.subTest(defect=i), tempfile.TemporaryDirectory() as folder:
                with self.assertRaises(ValueError): self.check(Path(folder), mutation)

    def test_rehashing_corrupted_archive_cannot_change_internal_source_identity(self):
        def mutate(cert, root):
            with zipfile.ZipFile(root/'launch/sources.zip', 'w') as z: z.writestr('worker.cpp', b'new source')
            cert['artifact_identities'] = [q.identity(a['path']) for a in cert['artifact_identities']]
            cert['artifacts'] = {a['path']: a['sha256'] for a in cert['artifact_identities']}
        with tempfile.TemporaryDirectory() as folder:
            with self.assertRaisesRegex(ValueError, 'Archived source changed'): self.check(Path(folder), mutate)


class ReferenceContractTests(unittest.TestCase):
    def test_fast_kind_two_and_actual_dt_boundaries_checked_from_original_rows(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder); source = write(root/'source_core.hpp', {})
            startup = dict(config=dict(nodes=2, node_dt=.04), source_artifacts=[source])
            cases = [dict(kind=2, center=[1., 2., 3.], t=t, dt=dt, nodes=2, node_dt=.04)
                for t in (0., 2., 4., 18., 20., 24.) for dt in (.012, .016, .020, .024)]
            cases.append(dict(cases[0], kind=0))
            output = []
            for i, case in enumerate(cases):
                offset = 0.
                for node in (-1, 0, 1):
                    if node >= 0: offset += case['dt'] if node == 0 else case['node_dt']
                    at = case['t']+offset
                    value = np.concatenate(q.figure8(case['center']).sample(at)) if case['kind'] else np.r_[case['center'], np.zeros(6)]
                    output.append([i, node, at, *value])
            actual = root/'compiled.txt'; np.savetxt(actual, output, fmt='%.17g')
            cb = write(root/'cases.json', cases)
            report = dict(kind='cpp_fast_reference_preview_v1', passed=True, complete=True,
                trajectory=q.fast_trajectory(), maximum_absolute_error=dict.fromkeys((
                    'position', 'velocity', 'acceleration', 'preview_position', 'preview_velocity', 'preview_acceleration'), 0.),
                case_count=len(cases), target_sample_count=len(output), physical_acceptance=False,
                binary=source, source_artifacts=[source], cases=cb, actual_output=q.identity(actual),
                time_tolerance=0., pva_tolerance=2e-12)
            q.verify_reference(report, startup, q.Evidence())
            cases[0]['kind'] = 1
            report['cases'] = write(root/'cases.json', cases)
            with self.assertRaisesRegex(ValueError, 'Reference shape differs'):
                q.verify_reference(report, startup, q.Evidence())


if __name__ == '__main__': unittest.main()
