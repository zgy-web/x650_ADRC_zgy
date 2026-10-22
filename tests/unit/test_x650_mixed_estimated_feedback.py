"""Synthetic acceleration source/frame/freshness contracts, never flight evidence."""
from copy import deepcopy
from pathlib import Path
import sys
import unittest
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
for path in (ROOT, ROOT/'scripts', Path(__file__).parent, ROOT/'control_packages/x650_wholebody_v1'):
    if str(path) not in sys.path: sys.path.insert(0, str(path))
from test_x650_estimated_feedback import fixture, advance
from test_x650_upper_stage_audit import stream_fixture
from simulations.x650_wholebody_mpc_v1.estimated_feedback import (
    Px4EstimatedFeedback, ACCELERATION_PROFILE, ACCELERATION_CONTRACT, NED_TO_ENU)
import audit_x650_mixed_feedback as audit


def acceleration_fixture():
    node, reference = fixture()
    node.data['position'].ax, node.data['position'].ay, node.data['position'].az = 1., 2., -3.
    return node, reference


class EstimatedAccelerationFeedbackTests(unittest.TestCase):
    def test_legacy_does_not_require_acceleration_or_add_diagnostics(self):
        node, reference = fixture()
        legacy = Px4EstimatedFeedback(reference, node)
        state, _, diagnostics = legacy.read(node, 10., wall_now=100.)
        self.assertIsNone(state.acceleration_enu)
        self.assertEqual(legacy.document['schema_version'], 1)
        self.assertEqual(diagnostics['feedback_profile'], 'px4_estimated_raw_ev_world_v1')
        self.assertNotIn('acceleration_source', diagnostics)
        self.assertNotIn('feedback_contract', diagnostics)
        node.data['position'].ax = float('nan')
        self.assertIsNone(legacy.read(node, 10., wall_now=100.)[0].acceleration_enu)

    def test_optin_maps_earth_derivative_without_attitude_rotation_or_gravity(self):
        node, reference = acceleration_fixture()
        adapter = Px4EstimatedFeedback(reference, node, require_acceleration=True)
        state, _, diagnostics = adapter.read(node, 10., wall_now=100.)
        np.testing.assert_array_equal(state.acceleration_enu, [2., 1., 3.])
        self.assertFalse(state.acceleration_enu.flags.writeable)
        self.assertEqual(diagnostics['px4_snapshot']['acceleration_ned_m_s2'], [1., 2., -3.])
        self.assertEqual(diagnostics['control_state']['acceleration_enu_m_s2'], [2., 1., 3.])
        self.assertEqual(diagnostics['acceleration_source']['timestamp_sample'], 499000)
        self.assertEqual(adapter.document['feedback_contract'], ACCELERATION_CONTRACT)
        self.assertFalse(adapter.document['acceleration_fallback'])
        self.assertEqual(adapter.document['max_age_s'], {'state': .1, 'flags': 1.25, 'joint': .25, 'wall': 4., 'flags_wall': 10.})

    def test_missing_invalid_acceleration_never_falls_back_to_truth_or_difference(self):
        for fault in ('missing', 'nan', 'inf', 'bool'):
            node, reference = acceleration_fixture()
            if fault == 'missing': del node.data['position'].ax
            else: node.data['position'].ax = {'nan': float('nan'), 'inf': float('inf'), 'bool': True}[fault]
            node.data['truth'] = object()
            node.acceleration = [0., 0., 0.]
            with self.subTest(fault=fault), self.assertRaises((ValueError, RuntimeError)):
                Px4EstimatedFeedback(reference, node, require_acceleration=True).read(node, 10., wall_now=100.)
        for option in (1, None, 'true'):
            with self.subTest(option=option), self.assertRaises(ValueError):
                Px4EstimatedFeedback(reference, require_acceleration=option)

    def test_new_position_acceleration_does_not_advance_angular_derivative_stamp(self):
        node, reference = acceleration_fixture()
        adapter = Px4EstimatedFeedback(reference, node, require_acceleration=True)
        adapter.read(node, 10., wall_now=100.)
        advance(node, 10.02, angular=False)
        node.data['position'].ax = 4.
        state, _, diagnostics = adapter.read(node, 10.02, wall_now=100.02)
        self.assertEqual(state.stamp_s, 10.)
        self.assertEqual(diagnostics['acceleration_stamp_s'], 10.02)
        self.assertEqual(diagnostics['acceleration_source']['callback_sequence'], 2)
        np.testing.assert_array_equal(state.acceleration_enu, [2., 4., 3.])

    def test_held_position_keeps_its_acceleration_stamp_and_value(self):
        node, reference = acceleration_fixture()
        adapter = Px4EstimatedFeedback(reference, node, require_acceleration=True)
        adapter.read(node, 10., wall_now=100.)
        previous = (node.received_sim['position'], node.received['position'], node.callback_sequences['position'])
        advance(node, 10.02)
        node.received_sim['position'], node.received['position'], node.callback_sequences['position'] = previous
        state, _, diagnostics = adapter.read(node, 10.02, wall_now=100.02)
        self.assertEqual(state.stamp_s, 10.02)
        self.assertEqual(diagnostics['acceleration_stamp_s'], 10.)
        np.testing.assert_array_equal(state.acceleration_enu, [2., 1., 3.])

    def test_stale_reset_invalid_fusion_and_missing_stream_still_fail(self):
        for fault in ('stale', 'reset', 'invalid', 'fusion', 'missing'):
            node, reference = acceleration_fixture()
            adapter = Px4EstimatedFeedback(reference, node, require_acceleration=True)
            if fault == 'stale': node.received_sim['position'] -= .101
            elif fault == 'reset': node.data['position'].vxy_reset_counter += 1
            elif fault == 'invalid': node.data['position'].v_xy_valid = False
            elif fault == 'fusion': node.data['estimator_flags'].cs_ev_vel = False
            else: del node.data['attitude']
            with self.subTest(fault=fault), self.assertRaises(RuntimeError):
                adapter.read(node, 10., wall_now=100.)

    def test_diagnostics_and_state_are_copies_not_live_message_views(self):
        node, reference = acceleration_fixture()
        state, _, diagnostic = Px4EstimatedFeedback(reference, node, require_acceleration=True).read(node, 10., wall_now=100.)
        node.data['position'].ax = 99.
        self.assertEqual(diagnostic['px4_snapshot']['acceleration_ned_m_s2'], [1., 2., -3.])
        np.testing.assert_array_equal(state.acceleration_enu, [2., 1., 3.])


def audit_fixture():
    ulog, native, reference, window, pid, control, feedback = stream_fixture()
    local = audit.precision.dataset(ulog, 'vehicle_local_position')
    source_t = audit.precision.stamp(local)
    source_a = np.column_stack((.01*np.sin(source_t), .02*np.cos(source_t), .03*np.sin(2*source_t)))
    for index, field in enumerate(('ax', 'ay', 'az')): local[field] = source_a[:, index]
    if 'timestamp' not in local: local['timestamp'] = source_t*1e6
    for row, entry in zip(control, feedback):
        d = entry['feedback']
        now = float(entry['sim_s'])
        an = np.array([np.interp(now, source_t, source_a[:, index]) for index in range(3)])
        ae = NED_TO_ENU@an
        callback = d['callbacks']['position']
        d.update(feedback_profile=ACCELERATION_PROFILE, feedback_contract=ACCELERATION_CONTRACT,
                 acceleration_stamp_s=callback['received_sim_s'],
                 acceleration_source={'topic': 'vehicle_local_position', 'callback_sequence': callback['sequence'],
                     'received_sim_s': callback['received_sim_s'], 'received_wall_s': callback['received_wall_s'],
                     **d['px4_raw_timestamps']['position'], 'quantity': 'earth_fixed_velocity_derivative',
                     'input_frame': 'NED', 'output_frame': 'ENU', 'numerical_differentiation': False, 'truth_feedback': False})
        d['px4_snapshot']['acceleration_ned_m_s2'] = an.tolist()
        d['control_state']['acceleration_enu_m_s2'] = ae.tolist()
        row.update({f'feedback_a{axis}': float(ae[index]) for index, axis in enumerate('xyz')})
        row.update(feedback_acceleration_position_stamp_s=callback['received_sim_s'],
                   feedback_acceleration_position_sequence=callback['sequence'])
    native_acceleration = {'t': native['t'], 'acceleration': np.column_stack(
        [np.interp(native['t'], source_t, source_a[:, index]) for index in range(3)])@NED_TO_ENU.T}
    return ulog, native_acceleration, reference, window, control, feedback


class MixedAccelerationAuditTests(unittest.TestCase):
    @staticmethod
    def timed_consumption(receipt_minus_publication, use_minus_publication):
        ulog, native, _, window, control, feedback = audit_fixture()
        local = audit.precision.dataset(ulog, 'vehicle_local_position')
        source_t = np.asarray(local['timestamp'], float)*1e-6
        # Unique synthetic payloads prevent adjacent near-static publications
        # from serving as alternative tolerance witnesses.
        local['ax'] = np.arange(len(source_t), dtype=float)
        selected_control, selected_feedback = [], []
        for original_index in (100, 101):
            row, entry = deepcopy(control[original_index]), deepcopy(feedback[original_index])
            index = int(np.argmin(abs(source_t-float(row['sim_s']))))
            pub = source_t[index]
            receipt, use = pub+receipt_minus_publication, pub+use_minus_publication
            row['sim_s'], entry['sim_s'] = use, use
            d = entry['feedback']
            callback = d['callbacks']['position']
            callback.update(received_sim_s=receipt, sim_age_s=use-receipt)
            d['callbacks']['clock']['received_wall_s'] = callback['received_wall_s']+.001
            d['acceleration_source']['received_sim_s'] = receipt
            d['acceleration_stamp_s'] = receipt
            row['feedback_acceleration_position_stamp_s'] = receipt
            an = np.asarray([local[key][index] for key in ('ax', 'ay', 'az')])
            d['px4_snapshot']['position_ned_m'] = [float(local[key][index]) for key in ('x', 'y', 'z')]
            d['px4_snapshot']['velocity_ned_m_s'] = [float(local[key][index]) for key in ('vx', 'vy', 'vz')]
            d['px4_snapshot']['acceleration_ned_m_s2'] = an.tolist()
            d['control_state']['acceleration_enu_m_s2'] = (NED_TO_ENU@an).tolist()
            row.update({f'feedback_a{axis}': float(value) for axis, value in zip('xyz', NED_TO_ENU@an)})
            selected_control.append(row)
            selected_feedback.append(entry)
        window = [selected_control[0]['sim_s'], selected_control[-1]['sim_s']]
        for row, entry in zip(selected_control, selected_feedback):
            row['t_s'] = entry['t_s'] = row['sim_s']-window[0]
        return ulog, native, window, selected_control, selected_feedback

    def test_known_cached_clock_schedules_use_actual_consumption_time(self):
        for receipt, publication, use in ((41.656, 41.688, 41.700), (44.636, 44.648, 44.664), (69.860, 69.872, 69.880)):
            with self.subTest(receipt=receipt, publication=publication, use=use):
                ulog, native, window, control, feedback = self.timed_consumption(receipt-publication, use-publication)
                report = audit.audit_mixed_acceleration_feedback(control, feedback, ulog, native, window)
                self.assertTrue(report['verified'])
                np.testing.assert_allclose(report['consumption_age_range_s'], [use-publication]*2, atol=1e-9)
                np.testing.assert_allclose(report['callback_cached_clock_minus_publication_range_s'], [receipt-publication]*2, atol=1e-9)
                self.assertEqual(report['consumption_age_samples_checked'], 2)

    def test_actual_future_or_stale_consumption_source_is_rejected(self):
        for receipt, use in ((-.020, -.009), (.02, .101)):
            with self.subTest(receipt=receipt, use=use):
                ulog, native, window, control, feedback = self.timed_consumption(receipt, use)
                with self.assertRaisesRegex(ValueError, 'bounded-causal'):
                    audit.audit_mixed_acceleration_feedback(control, feedback, ulog, native, window)

    def test_cached_publication_age_is_rechecked_at_every_held_use(self):
        ulog, native, window, control, feedback = self.timed_consumption(.095, .095)
        control[1], feedback[1] = deepcopy(control[0]), deepcopy(feedback[0])
        control[1]['sim_s'] += .02
        feedback[1]['sim_s'] += .02
        control[1]['t_s'] += .02
        feedback[1]['t_s'] += .02
        feedback[1]['feedback']['callbacks']['position']['sim_age_s'] = .02
        with self.assertRaisesRegex(ValueError, 'stale or future at actual control use'):
            audit.audit_mixed_acceleration_feedback(control, feedback, ulog, native, window)

    def test_full_v3_feedback_guards_and_same_payload_ulog_matching(self):
        ulog, native, reference, window, control, feedback = audit_fixture()
        report, _ = audit.feedback_check_mixed(control, feedback, reference, window)
        self.assertEqual(report['feedback_contract'], ACCELERATION_CONTRACT)
        checked = audit.audit_mixed_acceleration_feedback(control, feedback, ulog, native, window)
        self.assertTrue(checked['verified'])
        self.assertEqual(checked['distinct_position_callbacks'], len(control))
        self.assertFalse(checked['transport_delay_identified'])
        self.assertEqual(checked['physical_acceleration_proxy']['acceptance_policy'], 'report_only')

    def test_legacy_label_cannot_enter_v3_audit(self):
        _, _, reference, window, control, feedback = audit_fixture()
        feedback[0]['feedback']['feedback_profile'] = 'px4_estimated_raw_ev_world_v1'
        with self.assertRaisesRegex(ValueError, 'reviewed PX4 receipt'):
            audit.feedback_check_mixed(control, feedback, reference, window)

    def test_frame_or_source_callback_mismatch_is_rejected(self):
        for fault in ('frame', 'sequence', 'stamp', 'truth', 'difference'):
            ulog, native, _, window, control, feedback = audit_fixture()
            d = feedback[10]['feedback']
            if fault == 'frame': d['control_state']['acceleration_enu_m_s2'][2] += 9.81
            elif fault == 'sequence': d['acceleration_source']['callback_sequence'] += 1
            elif fault == 'stamp': control[10]['feedback_acceleration_position_stamp_s'] += .02
            elif fault == 'truth': d['acceleration_source']['truth_feedback'] = True
            else: d['acceleration_source']['numerical_differentiation'] = True
            with self.subTest(fault=fault), self.assertRaises(ValueError):
                audit.audit_mixed_acceleration_feedback(control, feedback, ulog, native, window)

    def test_epoch_raw_times_are_diagnostic_not_a_fitted_clock_alignment(self):
        ulog, native, _, window, control, feedback = audit_fixture()
        for entry in feedback:
            d = entry['feedback']
            for key in ('timestamp', 'timestamp_sample'):
                d['px4_raw_timestamps']['position'][key] += 1700000000000000
                d['acceleration_source'][key] += 1700000000000000
        report = audit.audit_mixed_acceleration_feedback(control, feedback, ulog, native, window)
        self.assertTrue(report['verified'])
        self.assertIsNone(report['fitted_time_offset_s'])

    def test_missing_or_delayed_source_cannot_be_backfilled_from_truth(self):
        for fault in ('missing', 'delayed'):
            ulog, native, _, window, control, feedback = audit_fixture()
            local = audit.precision.dataset(ulog, 'vehicle_local_position')
            if fault == 'missing': del local['ax']
            else: local['timestamp'] += .2e6
            with self.subTest(fault=fault), self.assertRaises(ValueError):
                audit.audit_mixed_acceleration_feedback(control, feedback, ulog, native, window)

    def test_large_physical_proxy_error_is_reported_without_changing_model_gate(self):
        ulog, native, _, window, control, feedback = audit_fixture()
        native['acceleration'] += [1., -2., 3.]
        report = audit.audit_mixed_acceleration_feedback(control, feedback, ulog, native, window)
        self.assertTrue(report['verified'])
        proxy = report['physical_acceleration_proxy']
        self.assertIsNone(proxy['acceptance_thresholds'])
        self.assertFalse(proxy['changes_response_model_acceptance'])
        self.assertGreater(proxy['held_controller_snapshot_minus_native_at_control_time']['rmse_3d'], 3.)

    def test_native_file_edge_sliver_is_reported_without_truth_backfill(self):
        ulog, native, _, window, control, feedback = audit_fixture()
        native['t'] = native['t'][1:-1]
        native['acceleration'] = native['acceleration'][1:-1]
        report = audit.audit_mixed_acceleration_feedback(control, feedback, ulog, native, window)
        proxy = report['physical_acceleration_proxy']
        self.assertEqual(proxy['control_samples_omitted_at_native_file_edges'], 2)
        self.assertEqual(proxy['held_controller_snapshot_minus_native_at_control_time']['sample_count'], len(control)-2)

    def test_held_callback_payload_cannot_mutate_without_a_new_sequence(self):
        ulog, native, _, window, control, feedback = audit_fixture()
        before, d = feedback[9]['feedback'], feedback[10]['feedback']
        now = float(feedback[10]['sim_s'])
        d['callbacks']['position'] = deepcopy(before['callbacks']['position'])
        d['callbacks']['position'].update(sim_age_s=now-before['callbacks']['position']['received_sim_s'])
        d['acceleration_source'] = deepcopy(before['acceleration_source'])
        d['acceleration_stamp_s'] = before['acceleration_stamp_s']
        d['px4_raw_timestamps']['position'] = deepcopy(before['px4_raw_timestamps']['position'])
        control[10]['feedback_acceleration_position_stamp_s'] = before['acceleration_stamp_s']
        control[10]['feedback_acceleration_position_sequence'] = before['acceleration_source']['callback_sequence']
        with self.assertRaisesRegex(ValueError, 'Held position callback payload'):
            audit.audit_mixed_acceleration_feedback(control, feedback, ulog, native, window)


if __name__ == '__main__':
    unittest.main()
