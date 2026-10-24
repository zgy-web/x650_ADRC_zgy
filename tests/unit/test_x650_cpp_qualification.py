"""Negative evidence gates: a pass flag cannot replace identities or full rows."""
from copy import deepcopy
from pathlib import Path
import json
import tempfile
import unittest

from simulations.x650_cpp_live_v1 import qualification as q


def artifact(name):
    return {'path': str(Path.cwd()/name), 'sha256': 'a'*64}


class NumericalEvidence:
    def document(self, value):
        return {'complete': True, 'source_files_unchanged': True,
                'rows': [artifact('input'+str(i)) for i in range(30)]}


def numerical_fixture():
    startup = {k: artifact(k) for k in ('core_library', 'optimization_library', 'predictor_library')}
    row = {'pass_': True, 'exit_code': 0, 'normalized_plan_peak': 0., 'predicted_state_peak': 0.,
           'predicted_tip_peak': 0., 'objective_relative': 0., 'native_compute_ms': 4., 'core_step_wall_ms': 4.,
           'independently_recomputed_quality': {'primal_violation': 0., 'kkt_stationarity_inf': 0.},
           'matrix_peak_errors': dict.fromkeys(('A', 'b', 'H', 'g', 'C', 'lo', 'hi'), 0.),
           'mit_wire_fields_equal': True, 'independent_matrix_oracle': True}
    report = {'kind': 'cpp_owning_core_equivalence_v1', 'complete': True, 'pass': True,
              'libraries_unchanged': True, 'runtime_python_calls': 0, 'passed': 30, 'total': 30,
              'core_library': startup['core_library'], 'optimization_library': startup['optimization_library'],
              'predictor': startup['predictor_library'], 'cases': artifact('cases'),
              'rows': [dict(deepcopy(row), index=i) for i in range(30)]}
    return startup, report


def worker_fixture():
    startup = {'core_library': artifact('core'), 'config': {'nodes': 15}, 'task_config': artifact('new_task')}
    tested = deepcopy(startup); tested['task_config'] = artifact('old_task')
    binary = artifact('worker')
    cases = []
    for name, count, good in [('complete_history_sigint', 1056, True), ('invalid_hash_latched', 3, False)]:
        cases.append({'name': name, 'passed': True, 'child_exit': 0 if good else 1,
            'final': {'requests': count, 'accepted': count if good else 2, 'faulted': not good,
                      'no_python_gc_or_pickle': True},
            'recorder': {'complete': True, 'failed': False, 'dropped': 0, 'written': count, 'offered': count},
            'records': [artifact('call_'+str(i)) for i in range(1, count+1)],
            'rows': [{'sequence': i, 'status': 0, 'absolute_errors': {'command': 0.}, 'native_callback_ms': 5.} for i in range(1, count+1)]})
    report = {'kind': 'cpp_worker_actual_ros_software_replay_v1', 'complete': True, 'passed': True,
        'no_actuator_publishers': True, 'physical_acceptance': False,
        'control_time_dt_and_actual_history_unchanged': True, 'core_library': startup['core_library'],
        'startup_manifest': artifact('tested_startup'), 'binary': binary, 'cases': cases}

    class Evidence:
        def document(self, value):
            name = Path(value['path']).name
            if name == 'tested_startup': return tested
            if name in ('old_task', 'new_task'): return {'stage': 'tip_hold'}
            if name.startswith('call_'): return {'kind': 'cpp_full_pipeline_call_v1', 'sequence': int(name[5:])}
            raise AssertionError(name)
    return startup, report, Evidence(), {binary['path']: binary['sha256']}


class CppQualificationTests(unittest.TestCase):
    def test_artifact_tamper_is_not_passed_by_existing_sha(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/'proof.json'; path.write_text('{}')
            expected = q.identity(path); q.Evidence().artifact(expected)
            path.write_text('{"passed":true}')
            with self.assertRaisesRegex(ValueError, 'bytes changed'): q.Evidence().artifact(expected)

    def test_duplicate_and_nonfinite_json_are_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/'proof.json'
            for text in ('{"passed":false,"passed":true}', '{"value":NaN}'):
                path.write_text(text)
                with self.assertRaises(ValueError): q.read(path)

    def test_all_matrix_rows_and_actual_mit_are_required(self):
        startup, report = numerical_fixture()
        q.verify_numerical(report, 30, startup, NumericalEvidence(), matrices=True)
        report['rows'][29]['mit_wire_fields_equal'] = False
        with self.assertRaisesRegex(ValueError, 'MIT'): q.verify_numerical(report, 30, startup, NumericalEvidence(), matrices=True)

    def test_numerical_quality_and_production_deadline_unchanged(self):
        for key, value in [('primal_violation', 2.0001e-7), ('kkt_stationarity_inf', .002001)]:
            startup, report = numerical_fixture(); report['rows'][5]['independently_recomputed_quality'][key] = value
            with self.assertRaisesRegex(ValueError, 'KKT'): q.verify_numerical(report, 30, startup, NumericalEvidence(), matrices=True)
        for key in ('native_compute_ms', 'core_step_wall_ms'):
            startup, report = numerical_fixture(); report['rows'][5][key] = 50.0001
            with self.assertRaisesRegex(ValueError, '50 ms'): q.verify_numerical(report, 30, startup, NumericalEvidence(), matrices=True)
        startup, report = numerical_fixture(); report['rows'][5]['objective_relative'] = 1.0001e-8
        with self.assertRaisesRegex(ValueError, 'objective'): q.verify_numerical(report, 30, startup, NumericalEvidence(), matrices=True)

    def test_wrong_current_library_does_not_inherit_old_pass(self):
        startup, report = numerical_fixture(); startup = deepcopy(startup); startup['core_library']['sha256'] = 'b'*64
        with self.assertRaisesRegex(ValueError, 'binding differs'): q.verify_numerical(report, 30, startup, NumericalEvidence(), matrices=True)

    def test_worker_full_positive_and_latched_negative_evidence(self):
        startup, report, evidence, binaries = worker_fixture()
        q.verify_worker(report, startup, evidence, binaries)
        report['cases'][1]['final']['faulted'] = False
        with self.assertRaisesRegex(ValueError, 'latch counts'): q.verify_worker(report, startup, evidence, binaries)

    def test_native_record_duplicate_and_recorder_loss_rejected(self):
        startup, report, evidence, binaries = worker_fixture()
        report['cases'][0]['records'][-1] = report['cases'][0]['records'][0]
        with self.assertRaisesRegex(ValueError, 'duplicated'): q.verify_worker(report, startup, evidence, binaries)
        startup, report, evidence, binaries = worker_fixture(); report['cases'][0]['recorder']['dropped'] = 1
        with self.assertRaisesRegex(ValueError, 'evidence lost'): q.verify_worker(report, startup, evidence, binaries)

    def test_config_change_cannot_reuse_worker_proof(self):
        startup, report, evidence, binaries = worker_fixture(); startup['config']['nodes'] = 14
        with self.assertRaisesRegex(ValueError, 'configuration'): q.verify_worker(report, startup, evidence, binaries)

    def test_worker_callback_keeps_50ms_gate(self):
        startup, report, evidence, binaries = worker_fixture(); report['cases'][0]['rows'][-1]['native_callback_ms'] = 50.001
        with self.assertRaisesRegex(ValueError, 'callback exceeded 50 ms'): q.verify_worker(report, startup, evidence, binaries)


if __name__ == '__main__':
    unittest.main()
