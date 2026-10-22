"""Synthetic diagnostics: no flight acceptance and no estimator/controller fit."""
import json
from pathlib import Path
import sys
import unittest
from unittest.mock import patch
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT/"scripts"))
sys.path.insert(0, str(Path(__file__).parent))
from test_x650_mpc_failure_plot import FailurePlotTests
from test_x650_mixed_stage_comparison_plot import write_csv, write_json
import analyze_x650_prediction_chains as chains
from mixed_mpc_protocol import build_prediction_chain_protocol


class DiagnosticTests(unittest.TestCase):
    def test_unique_causal_events_and_no_future_repeated_value_claim(self):
        t = np.arange(0., .12001, .004)
        values = np.where(t >= .064-1e-9, .2, np.where(t >= .044-1e-9, .1, 0.))
        report, rows = chains.command_effect_events([0, .02, .04], [0, .1, .2], t, values)
        self.assertEqual(report['status_counts'], {'unique_causal_value_change_witness': 2})
        self.assertAlmostEqual(report['effect_first_sample_minus_consumption_s']['median'], .024)
        report, _ = chains.command_effect_events([0, .02, .04], [0, .1, .1], t, values)
        self.assertNotIn('unique_causal_value_change_witness', report['status_counts'])
        self.assertIn('repeated_command_value_ambiguous', report['status_counts'])
        early = np.where(t >= .016-1e-9, .1, 0.)
        report, _ = chains.command_effect_events([0, .02, .04], [0, .1, .2], t, early)
        self.assertNotIn('unique_causal_value_change_witness', report['status_counts'])

    def test_native_integration_and_known_time_interpolation_preserve_bias(self):
        t = np.arange(0., .02401, .004)
        dq = np.column_stack((np.arange(len(t))*.1, np.arange(len(t))*-.2))
        q = np.vstack(([.2, -.3], [.2, -.3]+np.cumsum(np.diff(t)[:, None]*dq[1:], axis=0)))
        report = chains.integration_check(t, q, dq)
        self.assertLess(max(report['right_endpoint_dq_integral_minus_q_rad']['axis_peak_abs']), 1e-12)
        np.testing.assert_allclose(chains.interpolate(t, q, t), q)
        with self.assertRaisesRegex(ValueError, 'coverage'):
            chains.interpolate(t, q, [-.004])

    def make_fixture(self):
        fixture = FailurePlotTests('test_partial_failed_run_renders_without_task_end_or_certificate')
        fixture.setUp()
        self.addCleanup(fixture.tearDown)
        folder = fixture.folder
        native = chains.failure.display.csv_rows(folder/'native_trace.csv')
        for row in native:
            row.update(valid_accel=1, base_vx=0, base_vy=0, base_vz=0, base_ax=0, base_ay=0, base_az=0)
        write_csv(folder/'native_trace.csv', native)
        control = chains.failure.display.csv_rows(folder/'control_trace.csv')
        meta = chains.read(folder/'controller_metadata.json')
        model = chains.ArmModel(**meta['physical_profile'])
        tip = np.array([0, 0, 1])+chains.tip_body([0, 0], model)
        feedback, entries = [], []
        for row in control:
            row.update(q1=0, q2=0, qd1=0, qd2=0, joint_stamp_s=float(row['sim_s']))
            state = dict(q_rad=[0, 0], dq_rad_s=[0, 0], rotation_flu_to_enu=np.eye(3).tolist(), omega_flu_rad_s=[0, 0, 0])
            feedback.append(dict(sim_s=float(row['sim_s']), phase='measure', feedback=dict(joint_stamp_s=float(row['sim_s']), control_state=state)))
            predicted = np.zeros((3, 12)); predicted[:, 2] = 1
            entries.append(dict(sim_s=float(row['sim_s']), phase='measure', diagnostics=dict(solve_ms=2.,
                predicted_states=predicted.tolist(), predicted_tip_positions=np.tile(tip, (3, 1)).tolist(), node_intervals_s=[.02]*3)))
        write_csv(folder/'control_trace.csv', control)
        for name, rows in (('feedback_trace.jsonl', feedback), ('controller_diagnostics.jsonl', entries)):
            (folder/name).write_text('\n'.join(json.dumps(row) for row in rows), encoding='utf-8')
        return folder, fixture.root

    def test_failed_render_replays_prediction_without_optimization_or_time_fit(self):
        folder, root = self.make_fixture()
        with patch('scipy.optimize.minimize', side_effect=AssertionError('no optimization')), \
             patch('scipy.optimize.least_squares', side_effect=AssertionError('no fit')):
            report = chains.generate(folder, root/'diagnostic', gravity=9.80665)
        self.assertFalse(report['experiment_complete'])
        self.assertTrue(report['diagnostic_only'])
        self.assertFalse(report['eligible_as_stage_gate'])
        self.assertEqual(report['original_prediction_errors']['original_forecasts'], 11)
        self.assertGreater(report['original_prediction_errors']['unsupported_nodes_beyond_partial_end'], 0)
        self.assertEqual(report['relative_feedback_synchronization']['q_feedback_minus_native_at_header_rad']['axis_rms'], [0., 0.])
        for item in report['original_prediction_errors']['by_node']:
            self.assertLess(max(item['closure']['axis_peak_abs']), 1e-12)
        self.assertEqual(len(report['outputs_sha256']), 3)

    def test_completed_run_uses_declared_end_but_remains_diagnostic(self):
        folder, _ = self.make_fixture()
        meta = chains.read(folder/'controller_metadata.json')
        meta.update(complete=True, phase='complete', task_end_sim_s=10.2)
        write_json(folder/'controller_metadata.json', meta)
        report, *_ = chains.analyze(folder, gravity=9.80665)
        self.assertTrue(report['experiment_complete'])
        self.assertFalse(report['eligible_as_stage_gate'])
        self.assertEqual(report['partial_window_sim_s'], [10., 10.2])

    def enable_chain_fixture(self, folder):
        meta = chains.read(folder/'controller_metadata.json')
        task = meta['task']
        task['mixed_mpc_protocol'] = build_prediction_chain_protocol()
        write_json(folder/'controller_task.json', task)
        write_json(folder/'controller_metadata.json', meta)
        config = task['mixed_mpc_protocol']['mpc_controller_config']['prediction_chain']
        model = chains.ArmModel(**meta['physical_profile'])
        path = folder/'controller_diagnostics.jsonl'
        entries = [json.loads(line) for line in path.read_text().splitlines()]
        for entry in entries:
            d = entry['diagnostics']
            rotations = np.asarray([chains.rotation_increment(np.array([0., .02*(j+1), 0.])) for j in range(3)])
            predicted = np.asarray(d['predicted_states'])
            d.update(prediction_chain=config, predicted_rotations_flu_to_enu=rotations.tolist(),
                     prediction_initial_joint_position=[.001, -.002])
            d['predicted_tip_positions'] = [
                (x[:3]+R@chains.tip_body(x[6:8], model)).tolist() for x, R in zip(predicted, rotations)]
        path.write_text('\n'.join(json.dumps(row) for row in entries), encoding='utf-8')
        return path, entries

    def test_new_chain_uses_saved_rotations_and_reports_initial_joint_propagation(self):
        folder, _ = self.make_fixture()
        self.enable_chain_fixture(folder)
        # A saved rotation really differs from legacy omega=0 reconstruction.
        with patch.object(chains, 'rotation_increment', side_effect=AssertionError('no legacy reconstruction')):
            report, *_ = chains.analyze(folder, gravity=9.80665)
        forecast = report['original_prediction_errors']
        self.assertEqual(forecast['attitude_source'], 'original saved predicted_rotations_flu_to_enu')
        self.assertLess(forecast['maximum_stored_tip_rebuild_error_m'], 1e-12)
        initial = forecast['prediction_initial_joint_position']
        self.assertTrue(initial['available'])
        np.testing.assert_allclose(initial['propagated_minus_native_at_control_rad']['axis_mean'], [.001, -.002])
        np.testing.assert_allclose(initial['raw_feedback_minus_native_at_control_rad']['axis_rms'], [0., 0.])
        self.assertFalse(report['eligible_as_stage_gate'])

    def test_new_chain_rejects_missing_improper_or_fk_inconsistent_rotations(self):
        for fault in ('missing', 'improper', 'proper_but_wrong', 'missing_initial_q', 'config_mismatch'):
            with self.subTest(fault=fault):
                folder, _ = self.make_fixture()
                path, entries = self.enable_chain_fixture(folder)
                d = entries[0]['diagnostics']
                if fault == 'missing': del d['predicted_rotations_flu_to_enu']
                elif fault == 'improper': d['predicted_rotations_flu_to_enu'][0][0][0] *= 1.1
                elif fault == 'proper_but_wrong': d['predicted_rotations_flu_to_enu'][0] = np.eye(3).tolist()
                elif fault == 'missing_initial_q': del d['prediction_initial_joint_position']
                else: d['prediction_chain'] = {**d['prediction_chain'], 'joint_delay_s': .025}
                path.write_text('\n'.join(json.dumps(row) for row in entries), encoding='utf-8')
                with self.assertRaises(ValueError):
                    chains.analyze(folder, gravity=9.80665)


if __name__ == '__main__': unittest.main()
