"""Qualification-only observation of a candidate's actual propagation call.

Never performs an additional propagation or substitutes a candidate. Install
only around offline qualification. Every instance override is removed/restored
on exit, including exceptions, so the verifier introduces no retained cycle.
"""
import hashlib
import json
import numpy as np

_ABSENT = object()
RESULT_KEYS = ('time_s', 'q', 'dq', 'modes', 'envelope')


def snapshot(value):
    """Own every mutable numeric/container field at the actual call boundary."""
    if isinstance(value, np.ndarray):
        if value.dtype.hasobject: raise TypeError('Object arrays are not numerical evidence')
        return value.copy()
    if isinstance(value, dict): return {key: snapshot(item) for key, item in value.items()}
    if isinstance(value, list): return [snapshot(item) for item in value]
    if isinstance(value, tuple): return tuple(snapshot(item) for item in value)
    if value is None or isinstance(value, (bool, int, float, str, np.generic)): return value
    raise TypeError('Unsupported mutable evidence type: '+type(value).__name__)


def plain(value):
    if isinstance(value, np.ndarray): return value.tolist()
    if isinstance(value, np.generic): return value.item()
    if isinstance(value, dict): return {key: plain(item) for key, item in value.items()}
    if isinstance(value, (tuple, list)): return [plain(item) for item in value]
    return value


def digest(value):
    return hashlib.sha256(json.dumps(plain(value), sort_keys=True, separators=(',', ':'),
                                     allow_nan=False).encode()).hexdigest()


def exact(left, right, path='candidate'):
    if isinstance(left, dict):
        if not isinstance(right, dict) or set(left) != set(right):
            raise AssertionError(path + ': key set differs')
        for key in left: exact(left[key], right[key], path+'.'+key)
    elif isinstance(left, (tuple, list)):
        if len(left) != len(right): raise AssertionError(path + ': lengths differ')
        for index, (a, b) in enumerate(zip(left, right)): exact(a, b, path+f'[{index}]')
    elif isinstance(left, np.ndarray):
        np.testing.assert_array_equal(left, right, err_msg=path)
    elif left != right:
        raise AssertionError(path + ': values differ')


def restore(instance, name, previous):
    if previous is _ABSENT: instance.__dict__.pop(name, None)
    else: setattr(instance, name, previous)


class CandidateExecutionCapture:
    def __init__(self, predictor):
        self.predictor = predictor
        self.packed = callable(getattr(predictor, '_candidate_packed', None))
        self.entry_name = '_candidate_packed' if self.packed else 'candidate_rollout'
        self.boundary_name = '_rollout_native_arrays' if self.packed else '_rollout_known'
        self.records = []
        self._active = False

    def __enter__(self):
        predictor = self.predictor
        self._entry_previous = predictor.__dict__.get(self.entry_name, _ABSENT)
        original_entry = getattr(predictor, self.entry_name)
        def observed_entry(prediction, candidate):
            if self._active: raise AssertionError('Nested candidate capture is not permitted')
            self._active = True
            calls = []
            boundary_previous = predictor.__dict__.get(self.boundary_name, _ABSENT)
            original_boundary = getattr(predictor, self.boundary_name)
            def observed_boundary(*args, **kwargs):
                if kwargs.get('track_extrema') is not True:
                    raise AssertionError('Actual candidate propagation did not request all extrema')
                if self.packed:
                    if len(args) != 8: raise AssertionError('Packed propagation input signature changed')
                    start, q, dq, sends, positions, velocities, torques, times = args
                    if not (len(sends) == len(positions) == len(velocities) == len(torques)):
                        raise AssertionError('Packed packet-array row count differs')
                    # Clone six typed arrays now. Expand per-packet dictionaries
                    # only after the original step, using these owned copies.
                    packet_input = dict(packet_arrays=dict(sends=sends, positions=positions,
                        velocities=velocities, torques=torques, kp=predictor.kp, kd=predictor.kd))
                    representation = 'packed_arrays_at_actual_native_wrapper_entry'
                else:
                    if len(args) != 5: raise AssertionError('Public propagation input signature changed')
                    start, q, dq, packets, times = args
                    packet_input = dict(packets=packets)
                    representation = 'public_packets_at_actual_rollout_known_entry'
                observed = dict(representation=representation,
                                inputs=snapshot(dict(start=start, initial_q=q, initial_dq=dq,
                                                     query_times_s=times, **packet_input)),
                                track_extrema=True,
                                materialize_events=kwargs.get('materialize_events', True))
                # This is the one original execution, not a reconstruction.
                result = original_boundary(*args, **kwargs)
                observed['result'] = snapshot({key: result[key] for key in RESULT_KEYS})
                if result['envelope'].get('collected') is not True:
                    raise AssertionError('Actual propagation lacks a collected safety envelope')
                calls.append(observed)
                return result
            setattr(predictor, self.boundary_name, observed_boundary)
            try:
                result = original_entry(prediction, candidate)
                if len(calls) != 1:
                    raise AssertionError(f'Expected one actual candidate propagation, observed {len(calls)}')
                observed = calls[0]
                # Snapshot now; perform all expensive array/extremum assertions
                # only after the original controller step has returned. This
                # observer must not charge its verification to the 50 ms gate.
                observed['candidate_return'] = snapshot({key: result[key] for key in RESULT_KEYS})
                observed['reference_nodes'] = snapshot(result['reference_nodes'])
                observed['candidate_plan'] = np.array(candidate, copy=True)
                context = prediction['_context']
                observed['reference_context'] = snapshot({key: context[key] for key in ('r_current', 'node_times')})
                observed['exact_local_fallback'] = not bool(prediction.get('_native_packed')) if self.packed else None
                observed['actual_propagation_calls'] = 1
                self.records.append(observed)
                return result
            finally:
                restore(predictor, self.boundary_name, boundary_previous)
                self._active = False
        setattr(predictor, self.entry_name, observed_entry)
        return self

    def __exit__(self, *exception):
        restore(self.predictor, self.entry_name, self._entry_previous)
        for record in self.records:
            data = record['inputs'].pop('packet_arrays', None)
            if data is not None:
                record['inputs']['packets'] = [dict(send_time_s=float(when), q_ref=p.copy(),
                    dq_ref=v.copy(), tau_ff=f.copy(), kp=data['kp'].copy(), kd=data['kd'].copy())
                    for when,p,v,f in zip(data['sends'],data['positions'],data['velocities'],data['torques'])]


def verify_records(records):
    for record in records:
        exact(record['result'], record['candidate_return'], 'candidate return vs actual propagation')
        context = record['reference_context']
        expected = context['r_current'] + np.cumsum(
            np.diff(np.r_[0., context['node_times']])[:, None]*record['candidate_plan'][:, 3:], axis=0)
        exact(record['reference_nodes'], expected, 'actual reference integration')


def compare_execution_records(reference, native):
    if not reference or len(reference) != len(native):
        raise AssertionError('Missing or mismatched actual candidate executions')
    verify_records(reference); verify_records(native)
    evidence = []
    for index, (old, new) in enumerate(zip(reference, native)):
        # The QP plans may differ below the pre-existing solve tolerance, but
        # their quantized actual MIT inputs and resulting propagation must not.
        exact(old['inputs'], new['inputs'], f'actual_candidate[{index}].inputs')
        exact(old['result'], new['result'], f'actual_candidate[{index}].result')
        exact(old['reference_context'], new['reference_context'], f'actual_candidate[{index}].reference_context')
        if old['actual_propagation_calls'] != 1 or new['actual_propagation_calls'] != 1:
            raise AssertionError('Candidate capture is not an actual single execution')
        evidence.append(dict(index=index, captured_during_actual_call=True, extra_rollouts=0,
                             public_inputs_sha256=digest(old['inputs']), native_inputs_sha256=digest(new['inputs']),
                             public_result_sha256=digest(old['result']), native_result_sha256=digest(new['result']),
                             reference_nodes_each_match_own_actual_candidate=True,
                             reference_nodes_peak_abs_difference=float(np.max(np.abs(
                                 old['reference_nodes']-new['reference_nodes']))),
                             native=plain(new)))
    return evidence
