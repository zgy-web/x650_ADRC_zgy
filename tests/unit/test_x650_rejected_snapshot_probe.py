"""Evidence-verifier tests; actual C++/ROS coverage requires the private probe."""
from copy import deepcopy
import importlib.util
from pathlib import Path
from types import SimpleNamespace
import unittest

ROOT = Path(__file__).resolve().parents[2]
SPEC = importlib.util.spec_from_file_location('rejected_probe', ROOT/'scripts/probe_x650_rejected_snapshot.py')
PROBE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(PROBE)


class Snapshot(SimpleNamespace):
    def get_fields_and_field_types(self):
        return {name: 'fixture' for name in vars(self)}


def fixture():
    original = Snapshot(sequence=42, session_id=17, epoch=3, configuration_sha256=list(range(32)),
                        issued_monotonic_ns=100, control_time_s=10.02, task_time_s=.02, dt_s=.007,
                        packet_q_ref=[.1, .2, .3, .4], command_velocity=[.04, -.05], reference_time_s=.019)
    decoded = deepcopy(original); decoded.sequence = 1; decoded.dt_s = original.control_time_s-10.
    event = SimpleNamespace(snapshot_sequence=1, session_id=17, epoch=3, reason=15,
                            sequence=1, received_monotonic_ns=120)
    metadata = dict(kind='isolated_rejected_snapshot_v1', complete=True,
                    snapshot_semantics='exact_gate_input_after_admission_sequence_and_actual_dt_assignment',
                    ros_type='flyarm_control_interfaces/msg/ControlSnapshot', payload='rejected_snapshot.cdr',
                    bytes=8192, forward=False, reason=15, session_id=17, epoch=3,
                    sequence=1, event_sequence=1, source_sequence=42,
                    configuration_sha256=bytes(range(32)).hex(), source_issued_monotonic_ns=100,
                    source_received_monotonic_ns=110, checked_monotonic_ns=120,
                    gate_return_monotonic_ns=130, event_received_monotonic_ns=120)
    return metadata, decoded, original, event


class RejectedSnapshotProbeTests(unittest.TestCase):
    def test_all_fields_match_with_only_real_admission_rewrites(self):
        metadata, decoded, original, event = fixture()
        result = PROBE.check_evidence(metadata, decoded, original, 10., event, 8192)
        self.assertTrue(result['all_decoded_fields_equal'])
        self.assertEqual(result['source_sequence'], 42)
        self.assertEqual(result['admission_sequence'], 1)
        self.assertEqual(result['field_count'], len(vars(original)))
        self.assertEqual(original.dt_s, .007)

    def test_no_source_field_may_change_or_disappear(self):
        for key in vars(fixture()[1]):
            metadata, decoded, original, event = fixture()
            value = getattr(decoded, key)
            if isinstance(value, list):
                value[0] += 1
            else:
                setattr(decoded, key, value+1)
            with self.subTest(key=key), self.assertRaisesRegex(RuntimeError, 'fields differ'):
                PROBE.check_evidence(metadata, decoded, original, 10., event, 8192)
        metadata, decoded, original, event = fixture()
        del decoded.packet_q_ref
        with self.assertRaisesRegex(RuntimeError, 'field set'):
            PROBE.check_evidence(metadata, decoded, original, 10., event, 8192)

    def test_foreign_identity_wrong_reason_and_causal_clock_tampering_fail(self):
        for key, value in [('session_id', 99), ('epoch', 9), ('source_sequence', 99),
                           ('sequence', 42), ('reason', 4), ('bytes', 8191),
                           ('source_issued_monotonic_ns', 101), ('source_received_monotonic_ns', 121),
                           ('checked_monotonic_ns', 131), ('event_received_monotonic_ns', 119),
                           ('configuration_sha256', '00'*32), ('complete', False)]:
            metadata, decoded, original, event = fixture(); metadata[key] = value
            with self.subTest(key=key), self.assertRaises(RuntimeError):
                PROBE.check_evidence(metadata, decoded, original, 10., event, 8192)


if __name__ == '__main__':
    unittest.main()
