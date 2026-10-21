"""Synthetic math/provenance tests only; these data are never flight evidence."""
from copy import deepcopy
from datetime import datetime, timezone
import importlib.util
import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT/'scripts'))
import identify_x650_response_protocol as ident
import audit_x650_response_protocol as audit
from simulations.x650_wholebody_mpc_v1.response_protocol import build_response_protocol, protocol_velocity


def analytic_response(t, commands_t, command, axis, parameters, delay=0.):
    """Independent step superposition, not the event-driven predictor under test."""
    answer = np.zeros(len(t))
    differences = np.r_[command[0], np.diff(command)]
    for index in np.flatnonzero(differences):
        elapsed = t-commands_t[index]-delay
        active = elapsed >= 0
        dt = elapsed[active]
        if axis < 2:
            wn, zeta = parameters['wn_rad_s'], parameters['zeta']
            wd = wn*np.sqrt(1-zeta*zeta)
            response = 1-np.exp(-zeta*wn*dt)*(np.cos(wd*dt)+zeta*wn/wd*np.sin(wd*dt))
        else:
            response = 1-np.exp(-dt/parameters['tau_s'])
        answer[active] += differences[index]*response
    return answer


def fixture(role='train', delay=0.):
    protocol = build_response_protocol(role)
    t = np.round(np.arange(0., 60.0001, .004), 9)
    ct = np.round(np.arange(0., 60.0001, .02), 9)
    u = np.array([protocol_velocity(float(x), protocol) for x in ct])
    parameters = {'x': {'wn_rad_s': 2.8, 'zeta': .707},
                  'y': {'wn_rad_s': 3.1, 'zeta': .78}, 'z': {'tau_s': .25}}
    v = np.column_stack([analytic_response(t, ct, u[:, axis], axis, parameters[name], delay)
                         for axis, name in enumerate('xyz')])
    run = {'protocol': protocol, 'truth_times': t, 'velocities': v, 'command_times': ct,
           'commands': u, 'joints': np.zeros((len(t), 4))}
    return run, parameters


class FrozenResponseMathematicsTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.sample_run, cls.parameters = fixture()
        cls.data = ident.prepare_run(cls.sample_run)

    def test_continuous_zoh_matches_independent_analytic_pulses(self):
        for axis, name in enumerate('xyz'):
            predicted = ident.predict(self.data, axis, self.parameters[name])
            np.testing.assert_allclose(predicted, self.data['velocities'][:, axis], atol=5e-14, rtol=0)

    def test_training_fits_known_second_and_first_order_parameters(self):
        for axis, name in enumerate('xyz'):
            parameters, fit = ident.fit_axis(self.data, axis)
            self.assertTrue(fit['identifiable'])
            for key, expected in self.parameters[name].items():
                self.assertAlmostEqual(parameters[key], expected, places=5)

    def test_both_frozen_sequences_pass_with_unmodified_parameters(self):
        for role in ('train', 'holdout'):
            run, parameters = fixture(role)
            data = ident.prepare_run(run)
            for axis, name in enumerate('xyz'):
                metrics, _ = ident.evaluate_axis(data, axis, parameters[name])
                self.assertTrue(metrics['passed'], metrics['reasons'])
                self.assertEqual(len(metrics['pulse_and_zero_rest_segments']), 8)
                self.assertFalse(metrics['delay_diagnostic']['applied_to_model'])

    def test_zero_rest_failure_is_not_diluted_by_long_axis_window(self):
        data = deepcopy(self.data)
        times = data['times']
        mask = (times >= 3.) & (times <= 5.1)
        data['velocities'][mask, 0] += .013
        metrics, _ = ident.evaluate_axis(data, 0, self.parameters['x'])
        self.assertTrue(metrics['whole_axis_window']['passed'])
        self.assertFalse(metrics['passed'])
        rest = metrics['pulse_and_zero_rest_segments'][1]
        self.assertEqual(rest['kind'], 'zero_velocity_rest')
        self.assertFalse(rest['criteria']['bias'])

    def test_material_delay_rejects_without_changing_output_model(self):
        run, parameters = fixture('holdout', delay=.09)
        metrics, _ = ident.evaluate_axis(ident.prepare_run(run), 0, parameters['x'])
        self.assertFalse(metrics['passed'])
        self.assertTrue(metrics['delay_diagnostic']['material_unmodeled_delay'])
        self.assertAlmostEqual(metrics['delay_diagnostic']['best_delay_s'], .09, places=6)
        self.assertEqual(parameters['x'], self.parameters['x'])

    def test_quiet_initial_acceleration_is_checked_not_fitted(self):
        run = deepcopy(self.sample_run)
        run['velocities'][run['truth_times'] <= 1., 0] = .02
        with self.assertRaisesRegex(ValueError, 'quiet'):
            ident.prepare_run(run)

    def test_active_command_during_zero_rest_is_rejected(self):
        run = deepcopy(self.sample_run)
        run['commands'][10, 0] = .01
        with self.assertRaisesRegex(ValueError, 'predeclared'):
            ident.prepare_run(run)

    def test_return_segment_is_excluded_from_prediction_and_scoring(self):
        run = deepcopy(self.sample_run)
        after = run['truth_times'] > 54.
        run['velocities'][after] = 100.
        data = ident.prepare_run(run)
        self.assertEqual(data['times'][-1], 54.)
        np.testing.assert_equal(data['velocities'], self.data['velocities'])


class ProtocolAuditTests(unittest.TestCase):
    def controls(self):
        protocol = build_response_protocol('train')
        t = np.round(np.arange(0., 60.0001, .02), 9)
        records = []
        for value in t:
            u = protocol_velocity(float(value), protocol)
            record = {'phase': 'measure', 't_s': value, 'sim_s': 10.+value}
            for axis, velocity in zip('xyz', u):
                record['base_velocity_target_'+axis] = velocity
                record['command_acceleration_'+axis] = 0.
                record['feedback_p'+axis] = 0.
            records.append(record)
        return protocol, records

    def test_complete_protocol_pulses_zero_rest_and_return(self):
        protocol, records = self.controls()
        result = audit.verify_protocol_commands(records, {'target_world_enu_m': [0., 0., 0.]}, [10., 70.], protocol)
        self.assertTrue(result['verified'])

    def test_rest_motion_rejected(self):
        protocol, records = self.controls()
        records[200]['base_velocity_target_x'] = .01
        with self.assertRaisesRegex(ValueError, 'zero-velocity'):
            audit.verify_protocol_commands(records, {'target_world_enu_m': [0., 0., 0.]}, [10., 70.], protocol)

    def test_active_acceleration_feedforward_rejected(self):
        protocol, records = self.controls()
        records[200]['command_acceleration_x'] = .001
        with self.assertRaisesRegex(ValueError, 'zero acceleration'):
            audit.verify_protocol_commands(records, {'target_world_enu_m': [0., 0., 0.]}, [10., 70.], protocol)


class FreezeProvenanceTests(unittest.TestCase):
    def test_plan_seals_source_and_templates(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            binary = root/'px4'; binary.write_bytes(b'synthetic binary, never executable')
            source = root/'controller.py'; source.write_text('# fixture\n')
            tasks = {role: {'response_probe_protocol': build_response_protocol(role), 'duration_s': 60.}
                     for role in ('train', 'holdout')}
            tasks['holdout'].update(frozen_response_candidate=None, frozen_response_candidate_sha256=None)
            from simulations.x650_wholebody_mpc_v1.task import source_files_for_task
            preflight = {'synthetic': True, 'source_files_sha256': {
                path.relative_to(ROOT).as_posix(): ident.sha(path)
                for path in source_files_for_task(tasks['train'])}}
            output = root/'preregister.json'
            plan = ident.freeze_plan(output, [source], tasks, binary, preflight)
            self.assertFalse(plan['eligible_as_stage_gate'])
            ident.verify_plan(output, task=tasks['train'])
            bad = deepcopy(tasks['train']); bad['duration_s'] = 61.
            with self.assertRaisesRegex(ValueError, 'template'):
                ident.verify_plan(output, task=bad)
            with self.assertRaises(FileExistsError):
                ident.freeze_plan(output, [source], tasks, binary, preflight)
            source.write_text('# changed\n')
            with self.assertRaisesRegex(ValueError, 'source changed'):
                ident.verify_plan(output)

    def test_created_times_reject_boolean_nonfinite_and_future(self):
        for value in (True, False, float('nan'), float('inf'), -1., 0., 1e30):
            self.assertFalse(ident.valid_created_time(value), value)
        self.assertTrue(ident.valid_created_time(10.))

    def test_holdout_requires_candidate_preflight_archive_and_chronology(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            candidate_path = root/'candidate.json'
            candidate = {'created_unix': 20.}
            candidate_path.write_text(json.dumps(candidate))
            archived = root/'frozen_response_candidate.json'
            archived.write_bytes(candidate_path.read_bytes())
            launch = root/'launch.json'
            launch.write_text(json.dumps({'started_utc': datetime.fromtimestamp(30., timezone.utc).isoformat(),
                'frozen_response_candidate_archive': {'path': str(archived), 'sha256': ident.sha(candidate_path),
                                                      'source_path': str(candidate_path)}}))
            protocol = build_response_protocol('holdout')
            template = {'response_probe_protocol': protocol}
            run = {'protocol': protocol, 'metadata': {'task': {**template,
                   'frozen_response_candidate': str(candidate_path), 'frozen_response_candidate_sha256': ident.sha(candidate_path)}},
                   'artifacts': {str(launch): ident.sha(launch), str(archived): ident.sha(archived)}}
            plan = {'protocols': {'holdout': protocol}, 'task_templates': {'holdout': template}, 'created_unix': 10.}
            self.assertTrue(ident.check_run_plan(run, plan, 'holdout', candidate, candidate_path))
            candidate['created_unix'] = 31.
            with self.assertRaisesRegex(ValueError, 'before its training'):
                ident.check_run_plan(run, plan, 'holdout', candidate, candidate_path)
            candidate['created_unix'] = 20.
            archived.write_text('{}')
            with self.assertRaisesRegex(ValueError, 'archive'):
                ident.check_run_plan(run, plan, 'holdout', candidate, candidate_path)

    def test_failed_fit_preserves_report_without_candidate_or_model(self):
        with tempfile.TemporaryDirectory() as tmp:
            output = Path(tmp)/'failure'
            with patch.object(ident, 'check_plan', side_effect=ValueError('synthetic source mismatch')):
                with self.assertRaises(ValueError):
                    ident.fit_training('absent', 'absent', 'absent', output)
            self.assertFalse(ident.json_read(output/'training_report.json')['passed'])
            self.assertFalse((output/'frozen_response_candidate.json').exists())
            self.assertFalse((output/'response_model.json').exists())

    def test_validation_has_no_optimizer_and_preserves_failed_evidence(self):
        with tempfile.TemporaryDirectory() as tmp:
            output = Path(tmp)/'failure'
            with patch.object(ident, 'check_plan', side_effect=ValueError('synthetic prerequisite mismatch')):
                with patch.object(ident, 'fit_axis', side_effect=AssertionError('must not fit holdout')):
                    with self.assertRaises(ValueError):
                        ident.validate_holdout('absent', 'absent', 'absent', 'absent', output)
            self.assertFalse(ident.json_read(output/'response_identification.json')['passed'])
            self.assertFalse((output/'response_model.json').exists())

    def test_validation_reaches_full_scoring_without_any_optimizer(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            run, parameters = fixture('holdout')
            run.update(folder='fresh_holdout', command_csv_sha256='new command',
                       normalized_measurement_sha256='new measure', identity={'synthetic': True}, artifacts={})
            training_report = root/'training_report.json'; training_report.write_text('{}')
            source_archive = root/'plan.sources.zip'; source_archive.write_bytes(b'synthetic immutable source fixture')
            preregistration = root/'preregistration.json'; preregistration.write_text('{}')
            candidate = {'parameters': parameters, 'identity': {'synthetic': True},
                'training': {'folder': 'fresh_training', 'command_csv_sha256': 'old command',
                             'normalized_measurement_sha256': 'old measure'},
                'training_report_path': str(training_report), 'training_report_sha256': ident.sha(training_report)}
            candidate_path = root/'candidate.json'; candidate_path.write_text(json.dumps(candidate))
            plan = {'protocols': {role: build_response_protocol(role) for role in ('train', 'holdout')},
                    'source_archive': {'path': str(source_archive), 'sha256': ident.sha(source_archive)}}
            training = {'axes': [{}, {}, {}], 'artifacts': {}}
            with patch.object(ident, 'check_plan', return_value=plan), \
                    patch.object(ident, 'check_candidate', return_value=training), \
                    patch.object(ident, 'load_protocol_run', return_value=run), \
                    patch.object(ident, 'check_run_plan', return_value=True), \
                    patch.object(ident, 'fit_axis', side_effect=AssertionError('cannot fit holdout')), \
                    patch.object(ident, 'least_squares', side_effect=AssertionError('cannot optimize holdout')), \
                    patch.object(ident, 'minimize_scalar', side_effect=AssertionError('cannot optimize holdout')):
                report = ident.validate_holdout(candidate_path, 'unused', 'unused', preregistration, root/'result')
            self.assertTrue(report['passed'], report['reasons'])
            self.assertEqual(report['parameters'], parameters)
            self.assertEqual(report['artifacts'][str(training_report)], ident.sha(training_report))
            self.assertTrue((root/'result/response_model.json').is_file())


if __name__ == '__main__':
    unittest.main()
