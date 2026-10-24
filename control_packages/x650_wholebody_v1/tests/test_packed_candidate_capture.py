"""No-library tests of the qualification observer, not native qualification."""
from copy import deepcopy
import importlib.util
from pathlib import Path
import unittest
import numpy as np

PATH = Path(__file__).resolve().parents[3]/'scripts/packed_candidate_capture.py'
SPEC = importlib.util.spec_from_file_location('candidate_capture_under_test', PATH)
capture = importlib.util.module_from_spec(SPEC); SPEC.loader.exec_module(capture)


def inputs(packed=False):
    packet = dict(send_time_s=-.005, q_ref=np.array([.1, -.2]), dq_ref=np.array([.03, -.04]),
                  tau_ff=np.array([.01, .02]), kp=np.array([6., 6.]), kd=np.array([.25, .25]))
    return dict(_native_packed=packed, _context=dict(r_current=np.array([.1, -.2]),
                node_times=np.array([.02, .06]), packets=[packet])), np.array([[0.,0.,0.,.05,-.04]]*2)


def result(times):
    return dict(time_s=np.array(times, copy=True), q=np.tile([.1, -.2], (len(times), 1)),
                dq=np.tile([.03, -.04], (len(times), 1)), modes=np.ones((len(times), 2), dtype=np.int32),
                envelope=dict(q_lower=np.array([.09, -.21]), q_upper=np.array([.11, -.19]),
                              dq_lower=np.array([.02, -.05]), dq_upper=np.array([.04, -.03]),
                              events=[dict(time_s=.01, joint=0, kind='velocity_zero', q=.105, dq=0.)],
                              collected=True, substeps=12))


class Public:
    def __init__(self):
        self.calls = 0; self.kp = np.array([6., 6.]); self.kd = np.array([.25, .25])

    def _rollout_known(self, start, q, dq, packets, times, *, track_extrema):
        self.calls += 1
        return result(times)

    def candidate_rollout(self, prediction, plan):
        context = prediction['_context']
        out = self._rollout_known(0., np.array([0., 0.]), np.array([.01, -.01]),
                                 context['packets'], context['node_times'], track_extrema=True)
        out['reference_nodes'] = context['r_current'] + np.cumsum(
            np.diff(np.r_[0., context['node_times']])[:, None]*plan[:, 3:], axis=0)
        out['packets'] = context['packets']
        return out


class Packed(Public):
    def _rollout_native_arrays(self, start, q, dq, sends, positions, velocities, torques, times,
                               *, track_extrema, materialize_events=True):
        self.calls += 1
        return result(times)

    def _rollout_known(self, start, q, dq, packets, times, *, track_extrema):
        arrays = [np.array([p['send_time_s'] for p in packets])]
        arrays += [np.array([p[key] for p in packets]) for key in ('q_ref','dq_ref','tau_ff')]
        return self._rollout_native_arrays(start, q, dq, *arrays, times, track_extrema=track_extrema)

    def _candidate_packed(self, prediction, plan):
        if not prediction.get('_native_packed'): return self.candidate_rollout(prediction, plan)
        context = prediction['_context']; packets = context['packets']
        arrays = [np.array([p['send_time_s'] for p in packets])]
        arrays += [np.array([p[key] for p in packets]) for key in ('q_ref','dq_ref','tau_ff')]
        out = self._rollout_native_arrays(0., np.array([0., 0.]), np.array([.01, -.01]), *arrays,
                                         context['node_times'], track_extrema=True, materialize_events=False)
        out['reference_nodes'] = context['r_current'] + np.cumsum(
            np.diff(np.r_[0., context['node_times']])[:, None]*plan[:, 3:], axis=0)
        return out


class CandidateCaptureTests(unittest.TestCase):
    def test_actual_single_execution_and_full_results_compare_without_replay(self):
        public, packed = Public(), Packed()
        with capture.CandidateExecutionCapture(public) as a:
            expected = public.candidate_rollout(*inputs())
        with capture.CandidateExecutionCapture(packed) as b:
            actual = packed._candidate_packed(*inputs(True))
        evidence = capture.compare_execution_records(a.records, b.records)
        self.assertEqual(public.calls, 1); self.assertEqual(packed.calls, 1)
        self.assertEqual(evidence[0]['extra_rollouts'], 0)
        self.assertEqual(evidence[0]['public_inputs_sha256'], evidence[0]['native_inputs_sha256'])
        self.assertEqual(evidence[0]['public_result_sha256'], evidence[0]['native_result_sha256'])
        self.assertNotIn('_candidate_packed', packed.__dict__)
        self.assertNotIn('_rollout_native_arrays', packed.__dict__)
        self.assertNotIn('candidate_rollout', public.__dict__)
        capture.exact(expected['envelope'], actual['envelope'])

    def test_exact_local_fallback_is_captured_once_at_native_boundary(self):
        packed = Packed()
        with capture.CandidateExecutionCapture(packed) as observer:
            packed._candidate_packed(*inputs(False))
        self.assertEqual(packed.calls, 1)
        self.assertTrue(observer.records[0]['exact_local_fallback'])
        self.assertTrue(observer.records[0]['materialize_events'])

    def test_nominal_propagation_outside_candidate_is_not_mislabelled(self):
        packed = Packed(); prediction, plan = inputs(True)
        with capture.CandidateExecutionCapture(packed) as observer:
            packet = prediction['_context']['packets'][0]
            packed._rollout_native_arrays(0., np.zeros(2), np.zeros(2), np.array([-.005]),
                packet['q_ref'][None, :], packet['dq_ref'][None, :], packet['tau_ff'][None, :],
                np.array([.02, .06]), track_extrema=False)
            self.assertEqual(observer.records, [])
            packed._candidate_packed(prediction, plan)
        self.assertEqual(packed.calls, 2)
        self.assertEqual(len(observer.records), 1)

    def test_actual_packet_query_or_extremum_difference_fails_comparison(self):
        public, packed = Public(), Packed()
        with capture.CandidateExecutionCapture(public) as a: public.candidate_rollout(*inputs())
        with capture.CandidateExecutionCapture(packed) as b: packed._candidate_packed(*inputs(True))
        for field in ('packet', 'query', 'extremum'):
            altered = deepcopy(b.records)
            if field == 'packet': altered[0]['inputs']['packets'][0]['q_ref'][0] += .001
            elif field == 'query': altered[0]['inputs']['query_times_s'][0] += .001
            else: altered[0]['result']['envelope']['events'][0]['q'] += .001
            with self.subTest(field=field), self.assertRaises(AssertionError):
                capture.compare_execution_records(a.records, altered)

    def test_inputs_results_and_plan_are_owned_after_original_call(self):
        packed = Packed(); prediction, plan = inputs(True)
        with capture.CandidateExecutionCapture(packed) as observer:
            actual = packed._candidate_packed(prediction, plan)
        before = deepcopy(observer.records)
        actual['q'][:] = 999.; actual['envelope']['q_upper'][:] = 999.
        prediction['_context']['packets'][0]['tau_ff'][:] = 999.
        prediction['_context']['r_current'][:] = 999.; plan[:] = 999.
        capture.exact(observer.records, before)

    def test_packed_expansion_uses_owned_boundary_arrays_not_mutated_sources(self):
        packed = Packed(); prediction, plan = inputs(True)
        with capture.CandidateExecutionCapture(packed) as observer:
            packed._candidate_packed(prediction, plan)
            raw = observer.records[0]['inputs']
            self.assertIn('packet_arrays', raw); self.assertNotIn('packets', raw)
            expected = deepcopy(raw['packet_arrays'])
            prediction['_context']['packets'][0]['q_ref'][:] = 999.
            prediction['_context']['packets'][0]['tau_ff'][:] = 999.
            packed.kp[:] = 999.; packed.kd[:] = 999.
        packet = observer.records[0]['inputs']['packets'][0]
        for key, name in (('q_ref','positions'),('dq_ref','velocities'),('tau_ff','torques')):
            np.testing.assert_array_equal(packet[key],expected[name][0])
        np.testing.assert_array_equal(packet['kp'],expected['kp'])
        np.testing.assert_array_equal(packet['kd'],expected['kd'])
        self.assertEqual(packed.calls,1)

    def test_missing_or_double_propagation_cannot_be_promoted_to_evidence(self):
        for count in (0, 2):
            packed = Packed(); original = packed._candidate_packed
            def wrong(prediction, plan):
                if count == 0:
                    out = result(prediction['_context']['node_times']); out['reference_nodes'] = np.zeros((2, 2))
                    return out
                original(prediction, plan)
                return original(prediction, plan)
            packed._candidate_packed = wrong
            with self.subTest(count=count), self.assertRaisesRegex(AssertionError, 'Expected one actual'):
                with capture.CandidateExecutionCapture(packed) as observer:
                    packed._candidate_packed(*inputs(True))
            self.assertIs(packed.__dict__['_candidate_packed'], wrong)
            self.assertNotIn('_rollout_native_arrays', packed.__dict__)
            self.assertFalse(observer.records)

    def test_exception_restores_original_instance_overrides(self):
        packed = Packed()
        def fail(*args, **kwargs): raise ValueError('native rejected')
        packed._rollout_native_arrays = fail
        with self.assertRaisesRegex(ValueError, 'native rejected'):
            with capture.CandidateExecutionCapture(packed) as observer:
                packed._candidate_packed(*inputs(True))
        self.assertIs(packed.__dict__['_rollout_native_arrays'], fail)
        self.assertNotIn('_candidate_packed', packed.__dict__)
        self.assertFalse(observer.records)

    def test_altered_candidate_result_or_missing_extrema_are_rejected(self):
        for alter in ('after_return', 'missing_extrema'):
            packed = Packed(); original = packed._candidate_packed
            if alter == 'after_return':
                def changed(prediction, plan):
                    out = original(prediction, plan); out['q'][0, 0] += .01; return out
                packed._candidate_packed = changed
            else:
                boundary = packed._rollout_native_arrays
                def changed(*args, **kwargs):
                    out = boundary(*args, **kwargs); out['envelope']['collected'] = False; return out
                packed._rollout_native_arrays = changed
            with self.subTest(alter=alter), self.assertRaises(AssertionError):
                with capture.CandidateExecutionCapture(packed) as observer:
                    packed._candidate_packed(*inputs(True))
                capture.verify_records(observer.records)


if __name__ == '__main__': unittest.main()
