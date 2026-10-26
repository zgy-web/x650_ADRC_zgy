"""Synthetic admission counterexamples; no flight or optimizer is invoked."""
from copy import deepcopy
import json
from pathlib import Path
import tempfile
import unittest

from simulations.x650_cpp_recovery_v1 import qualification as q


def write(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, allow_nan=False), encoding='utf-8')
    return q.identity(path)


class ProtocolTests(unittest.TestCase):
    def setUp(self):
        self.original = dict(kind='x650_cpp_robustness_protocol_v1', output_period_s=.02,
            core_compute_max_s=.05, consumption_progress_budget_ms=40, delay_tests_enabled=False,
            robustness=dict(schedule=[dict(force_world_n=[1.5, 0., 0.])]),
            cpu_roles=dict(source=[19], cpp_source=[17], worker=[87, 89, 91, 97],
                dispatcher=[93], ingress=[94], consumption=[95]),
            worker_thread_roles=dict(executor=[87], auxiliary=[89, 91, 97]), cpu_scope='ordinary scheduler')
        self.new = deepcopy(self.original)
        self.new.update(cpu_roles=dict(source=[0], cpp_source=[1], worker=[2, 3, 4, 5],
            dispatcher=[6], ingress=[7], consumption=[8]),
            worker_thread_roles=dict(executor=[2], auxiliary=[3, 4, 5]), cpu_scope='new container, ordinary scheduler')

    def test_only_cpu_placement_may_change(self):
        q.validate_protocol(self.new, self.original)
        for mutate in (lambda v: v.update(core_compute_max_s=.06),
                       lambda v: v.update(output_period_s=.025),
                       lambda v: v.update(consumption_progress_budget_ms=60),
                       lambda v: v.update(delay_tests_enabled=True),
                       lambda v: v['robustness']['schedule'][0].update(force_world_n=[1., 0., 0.]),
                       lambda v: v.update(new_unqualified_option=True)):
            with self.subTest(mutate=mutate):
                value = deepcopy(self.new); mutate(value)
                with self.assertRaisesRegex(ValueError, 'protocol'):
                    q.validate_protocol(value, self.original)

    def test_cross_role_overlap_and_wrong_thread_partition_reject(self):
        mutations = (
            lambda v: v['cpu_roles'].update(source=[6]),
            lambda v: v['cpu_roles'].update(worker=[2, 3, 4]),
            lambda v: v['cpu_roles'].update(source=[False]),
            lambda v: v['cpu_roles'].update(unknown=[9]),
            lambda v: v['worker_thread_roles'].update(executor=[2, 3]),
            lambda v: v['worker_thread_roles'].update(auxiliary=[2, 3, 4]),
            lambda v: v['worker_thread_roles'].update(auxiliary=[3, 4, 9]),
            lambda v: v.update(cpu_scope=''))
        for mutate in mutations:
            with self.subTest(mutate=mutate):
                value = deepcopy(self.new); mutate(value)
                with self.assertRaises(ValueError): q.validate_protocol(value, self.original)


class EvidenceBindingTests(unittest.TestCase):
    def test_independent_timers_may_differ_but_branch_or_output_may_not(self):
        left = dict(output=[.1, .2], diagnostics=dict(centre_kind='slew_limited_brake',
            compute_ms=19., attempts=[dict(passed=True, elapsed_ms=8.)]))
        right = deepcopy(left); right['diagnostics']['compute_ms'] = 21.
        right['diagnostics']['attempts'][0]['elapsed_ms'] = 10.
        self.assertTrue(q.numerical_equal(left, right, ignored={'compute_ms', 'elapsed_ms'}))
        right['output'][0] += .001
        self.assertFalse(q.numerical_equal(left, right, ignored={'compute_ms', 'elapsed_ms'}))
        right = deepcopy(left); right['diagnostics']['centre_kind'] = 'unvalidated'
        self.assertFalse(q.numerical_equal(left, right, ignored={'compute_ms', 'elapsed_ms'}))

    def test_task_copy_allows_new_path_only_with_identical_hash_bound_bytes(self):
        folder = Path(__file__).resolve().parents[2]/'tmp'
        folder.mkdir(exist_ok=True)
        with tempfile.TemporaryDirectory(dir=folder) as temporary:
            a = write(Path(temporary)/'original.json', dict(frozen=True))
            b = write(Path(temporary)/'owned.json', dict(frozen=True))
            self.assertTrue(q.identical_task_copy(a, b, q.Evidence()))
            b = write(Path(temporary)/'changed.json', dict(frozen=False))
            self.assertFalse(q.identical_task_copy(a, b, q.Evidence()))


def software_fixture(root):
    """Build a clearly synthetic evidence graph to test admission rejection."""
    libraries = {}
    for name in ('core', 'optimization', 'predictor'):
        path = root/(name+'.so'); path.write_bytes(name.encode())
        libraries[name] = q.identity(path)
    worker = root/'isolated_worker'; worker.write_bytes(b'synthetic ABI2 worker')
    worker = q.identity(worker)
    sources = []
    for name in sorted(q.EXECUTION_SOURCES):
        path = root/name; path.parent.mkdir(parents=True, exist_ok=True); path.write_text(name)
        sources.append(dict(q.identity(path), relative_path=name))
    task = write(root/'task.json', {'synthetic': True})
    config = {'nodes': 15, 'synthetic': True}
    startup = dict(config=config, trajectory={'kind': 'fast_figure8'}, task_config=task,
        source_artifacts=[{k: v for k, v in s.items() if k != 'relative_path'} for s in sources],
        **{name+'_library': libraries[role] for name, role in
           [('core', 'core'), ('optimization', 'optimization'), ('predictor', 'predictor')]})
    startup_id = write(root/'startup.json', startup)
    reconstruction = write(root/'reconstruction.json', dict(
        kind='x650_cpp_original_sequence_reconstruction_v1', complete=True,
        all_original_outcomes_reproduced=True, stopped_at_original_rejection=True,
        memory_reconstructed_in_cpp=True, history_from_proposals=False,
        counterfactual_closed_loop_claimed=False, physical_validation_claimed=False,
        row_count=379, step_count_after=378, recording_sha256='a'*64))
    # Library bytes are synthetic; their identities still have to be consistent
    # all the way through summary, regression, raw calls and actual-worker proof.
    old_libraries = {k: v['sha256'] for k, v in libraries.items()}
    raw = write(root/'raw.json', dict(kind='x650_cpp_recovery_raw_replay_v1', complete=True,
                                     loaded_code=libraries))
    rows = []
    for seq, expectation in [(i, 'unchanged') for i in range(1, 379)]+[(379, 'recovered'), ('extra_rejected', 'rejected')]:
        fixture = write(root/'fixtures'/f'{seq}.json', dict(synthetic=True, sequence=seq, config=config,
            memory=dict(step_count=378)))
        call_raw = raw
        if seq == 379:
            recovered = dict(kind='x650_cpp_recovery_raw_replay_v1', complete=True, core_abi=2,
                returned=True, memory_commit_valid=True, input_sha256=fixture['sha256'], loaded_code=libraries,
                memory_before=dict(step_count=378), memory_after=dict(step_count=379), output=dict(command=[0., 0.]),
                core_step_wall_ms=19.2, diagnostics=dict(accepted=True, nonlinear_passed=True, actual_bounds=True,
                    previous_plan_fallback=False, recovery_triggered=True, recovery_trust_radius=.02,
                    compute_ms=19.1, recovery_attempts=[dict(centre_kind='slew_limited_brake', elapsed_ms=12.)]))
            call_raw = write(root/'raw_379.json', recovered)
        audit = write(root/'audits'/f'{seq}.json', dict(
            kind='x650_cpp_recovery_independent_replay_audit_v1', complete=True, passed=True,
            expectation=expectation, physical_validation_claimed=False, input_sha256=fixture['sha256'],
            artifacts=dict(input=fixture, old=call_raw, new=call_raw)))
        rows.append(dict(sequence=seq, expectation=expectation, passed=True, fixture=fixture,
                         audit=audit, old=dict(raw=call_raw), new=dict(raw=call_raw)))
    regression_value = dict(kind='x650_cpp_recovery_recording_regression_v1',
        complete=True, passed=True, replay_code_and_original_report_unchanged=True,
        original_source_and_fixtures_unchanged=True, closed_loop_or_physical_validation_claimed=False,
        injected_delay=False, configuration=dict(value=config), expected_case_count=380,
        passed_cases=380, failed_cases=0, rows=rows, original_reconstruction=reconstruction,
        expected_new_libraries=old_libraries, expected_old_libraries=old_libraries)
    regression = write(root/'regression.json', regression_value)
    worker_rows = [dict(sequence=i, passed=True, native_callback_ms=22., recovery_triggered=i == 379,
        absolute_errors=dict(command=0.), full_native_output_absolute_errors=dict(state=0.),
        diagnostic_absolute_errors=dict(plan=0.)) for i in range(1, 380)]
    independent = deepcopy(recovered)
    independent['core_step_wall_ms'] = 19.5
    independent['diagnostics']['compute_ms'] = 19.4
    independent['diagnostics']['recovery_attempts'][0]['elapsed_ms'] = 12.3
    independent = write(root/'independent_raw_379.json', independent)
    probe_value = dict(kind='x650_cpp_recovery_actual_worker_probe_v1', complete=True, passed=True,
        test_sequence_complete=True, no_actuator_publishers=True, physical_validation=False,
        physical_acceptance=False, counterfactual_closed_loop_claimed=False, injected_delay=False,
        rows_count=379, child_exit=0, worker=worker, startup=startup_id, libraries=libraries,
        recording=dict(path=str(root/'recording.json'), sha256='a'*64), recovered_report=independent,
        final=dict(requests=379, accepted=379, faulted=False),
        recorder=dict(complete=True, dropped=0, written=379, offered=379), rows=worker_rows,
        records=[raw]*379, wire_crosscheck=[dict(sequence=i, snapshot_all_fields_exact=True,
            proposal_all_fields_exact=True) for i in range(1, 380)])
    # The recording is only a schema marker in this synthetic test. Bind its
    # real identity in both reports rather than bypassing evidence verification.
    recording = write(root/'recording.json', dict(synthetic=True))
    original = q.read(reconstruction['path']); original['recording_sha256'] = recording['sha256']
    reconstruction = write(Path(reconstruction['path']), original)
    regression_value['original_reconstruction'] = reconstruction
    regression = write(Path(regression['path']), regression_value)
    probe_value['recording'] = recording
    probe = write(root/'probe.json', probe_value)
    summary = dict(kind='x650_trusted_centre_recovery_software_acceptance_v1', complete=True, passed=True,
        physical_validation=False, new_closed_loop_trajectory_claimed=False, hardware_qualified=False,
        injected_delay=False, core_abi=2, initial_trust_radius=.02, formal_recovery_trust_radius=.02,
        maximum_control_qp_attempts=3, maximum_auxiliary_projection_qp_attempts=1,
        maximum_centres=2, original_deadline_ms=50., code=dict(libraries=libraries, worker=worker, sources=sources),
        original_prefix_reconstruction=reconstruction, regression=regression, ros_worker_probe=probe)
    return summary, startup, {worker['path']: worker['sha256']}, regression_value, probe_value


class SoftwareTests(unittest.TestCase):
    def setUp(self):
        folder = Path(__file__).resolve().parents[2]/'tmp'
        folder.mkdir(exist_ok=True)
        self.temp = tempfile.TemporaryDirectory(dir=folder)
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.summary, self.startup, self.binaries, self.regression, self.probe = software_fixture(self.root)

    def verify(self, summary=None, startup=None, binaries=None):
        return q.verify_software(summary or self.summary, startup or self.startup,
                                q.Evidence(), self.binaries if binaries is None else binaries)

    def test_complete_synthetic_graph_admits_software_only(self):
        result = self.verify()
        self.assertTrue(result['verified']); self.assertFalse(result['physical_acceptance'])
        self.assertEqual(result['trust_radius'], .02)

    def test_changed_radius_deadline_abi_or_physical_claim_rejects(self):
        for key, value in [('formal_recovery_trust_radius', .03), ('original_deadline_ms', 60.),
                           ('core_abi', 1), ('physical_validation', True), ('injected_delay', True)]:
            with self.subTest(key=key):
                changed = deepcopy(self.summary); changed[key] = value
                with self.assertRaises(ValueError): self.verify(summary=changed)

    def test_installed_worker_or_missing_execution_source_rejects(self):
        with self.assertRaisesRegex(ValueError, 'worker'): self.verify(binaries={})
        changed = deepcopy(self.startup); changed['source_artifacts'] = changed['source_artifacts'][1:]
        with self.assertRaisesRegex(ValueError, 'source'): self.verify(startup=changed)

    def test_raw_evidence_bytes_cannot_be_changed_after_summary(self):
        Path(self.summary['regression']['path']).write_text('{}')
        with self.assertRaisesRegex(ValueError, 'bytes changed'): self.verify()

    def test_incomplete_or_reordered_regression_rejects_even_with_new_receipt(self):
        self.regression['rows'][0]['sequence'] = 2
        self.summary['regression'] = write(self.root/'regression_changed.json', self.regression)
        with self.assertRaisesRegex(ValueError, 'reordered'): self.verify()

    def test_probe_late_callback_or_missing_actual_receipt_rejects(self):
        self.probe['rows'][378]['native_callback_ms'] = 50.001
        self.summary['ros_worker_probe'] = write(self.root/'probe_changed.json', self.probe)
        with self.assertRaisesRegex(ValueError, 'deadline'): self.verify()

    def test_probe_cannot_omit_original_recovery_call(self):
        self.probe['rows'][378]['recovery_triggered'] = False
        self.summary['ros_worker_probe'] = write(self.root/'probe_changed.json', self.probe)
        with self.assertRaisesRegex(ValueError, 'coverage'): self.verify()


if __name__ == '__main__':
    unittest.main()
