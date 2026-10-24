"""Worker phase observations preserve bytes/order; no ROS or flight required."""
import json
from pathlib import Path
import pickle
import sys
from types import SimpleNamespace as NS
import unittest
import uuid
from unittest.mock import Mock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from flyarm_mpc_reference.isolated_node import WorkerTimingTrace, process_snapshot
from flyarm_mpc_reference.diagnostic_channel import DiagnosticSender, HEADER, MAGIC


class Clock:
    def __init__(self, wall=100):
        self.wall, self.cpu = wall, 0

    def wait(self, ns, cpu=False):
        self.wall += ns
        self.cpu += ns if cpu else 0

    def now(self):
        return self.wall

    def thread(self):
        return self.cpu


def make_case(*, publish_ns=0, serialize_ns=0, offer_ns=0, clock=None):
    clock = Clock() if clock is None else clock
    capture = NS(diagnostic=None)
    snapshot = NS(sequence=211, issued_monotonic_ns=19_714_370)
    proposal = NS(compute_end_monotonic_ns=0, accepted=True, payload=[1., 2.])
    order, packets = [], []
    def engine_process(s, proposal_type):
        assert s is snapshot and proposal_type is NS
        order.append('engine')
        clock.wait(14_368_327, cpu=True)
        capture.diagnostic = {'owned': [3., 4.]}
        proposal.compute_end_monotonic_ns = clock.now()
        return proposal
    engine = NS(process=engine_process, last_error=None)
    def publish(p):
        order.append('publish')
        assert p is proposal
        clock.wait(publish_ns)
    publisher = NS(publish=publish)
    def serialize(value):
        order.append('snapshot_cdr' if value is snapshot else 'proposal_cdr')
        clock.wait(serialize_ns, cpu=True)
        return pickle.dumps(value)
    sender = NS(last_timing_ns=None)
    def offer(sequence, value):
        order.append('offer')
        packets.append((sequence, pickle.dumps(value)))
        begin = clock.now()
        clock.wait(offer_ns, cpu=True)
        sender.last_timing_ns = (begin, clock.now(), clock.now(), clock.now(), clock.now())
        return True
    sender.offer = offer
    trace = WorkerTimingTrace()
    args = engine, capture, publisher, sender, snapshot, NS, serialize, trace
    return args, clock, order, packets, proposal


def row_dict(trace):
    return dict(zip(trace.columns, trace.rows[-1]))


class WorkerTimingTests(unittest.TestCase):
    def test_executor_cpu_samples_append_without_changing_payload_or_phase_order(self):
        args, clock, order, packets, proposal = make_case()
        observer = Mock(side_effect=[(101, 87), (101, 87)])
        process_snapshot(*args, clock_ns=clock.now, thread_clock_ns=clock.thread, cpu_observer=observer)
        row = row_dict(args[-1])
        self.assertEqual((row['callback_native_tid_begin'], row['callback_native_tid_end']), (101, 101))
        self.assertEqual((row['callback_actual_cpu_begin'], row['callback_actual_cpu_end']), (87, 87))
        self.assertEqual(row['executor_affinity_checked'], 3)
        self.assertEqual(order, ['engine', 'publish', 'snapshot_cdr', 'proposal_cdr', 'offer'])
        self.assertEqual(pickle.loads(packets[0][1])['proposal_cdr'], pickle.dumps(proposal))

    def test_bad_executor_entry_prevents_solve_and_still_records_failed_boundary(self):
        args, clock, order, packets, _ = make_case()
        observer = Mock(side_effect=[RuntimeError('wrong executor'), (101, 87)])
        with self.assertRaisesRegex(RuntimeError, 'wrong executor'):
            process_snapshot(*args, clock_ns=clock.now, thread_clock_ns=clock.thread, cpu_observer=observer)
        self.assertEqual(order, []); self.assertEqual(packets, [])
        self.assertTrue(args[0].faulted)
        self.assertEqual(row_dict(args[-1])['executor_affinity_checked'], 2)

    def test_bad_exit_fails_recording_after_publication_without_rewriting_command(self):
        args, clock, order, packets, proposal = make_case()
        observer = Mock(side_effect=[(101, 87), RuntimeError('placement drift')])
        with self.assertRaisesRegex(RuntimeError, 'placement drift'):
            process_snapshot(*args, clock_ns=clock.now, thread_clock_ns=clock.thread, cpu_observer=observer)
        self.assertIn('publish', order); self.assertTrue(args[0].faulted)
        self.assertEqual(pickle.loads(packets[0][1])['proposal_cdr'], pickle.dumps(proposal))
        self.assertEqual(row_dict(args[-1])['executor_affinity_checked'], 1)

    def test_existing_publish_before_diagnostic_order_and_exact_record_preserved(self):
        args, clock, order, packets, proposal = make_case(publish_ns=50, serialize_ns=20, offer_ns=30)
        engine, capture, publisher, sender, snapshot, kind, serialize, trace = args
        process_snapshot(*args, clock_ns=clock.now, thread_clock_ns=clock.thread)
        self.assertEqual(order, ['engine', 'publish', 'snapshot_cdr', 'proposal_cdr', 'offer'])
        value = pickle.loads(packets[0][1])
        self.assertEqual(packets[0][0], 211)
        self.assertEqual(value, {'snapshot_cdr': pickle.dumps(snapshot),
            'proposal_cdr': pickle.dumps(proposal), 'diagnostic': capture.diagnostic, 'error': None})
        row = row_dict(trace)
        self.assertEqual(row['engine_return_ns'], proposal.compute_end_monotonic_ns)
        self.assertEqual(row['publish_end_ns']-row['publish_begin_ns'], 50)
        self.assertEqual(row['snapshot_cdr_end_ns']-row['snapshot_cdr_begin_ns'], 20)
        self.assertEqual(row['proposal_cdr_end_ns']-row['proposal_cdr_begin_ns'], 20)
        self.assertEqual(row['offer_end_ns']-row['offer_begin_ns'], 30)
        self.assertEqual(row['publish_thread_cpu_end_ns']-row['publish_thread_cpu_begin_ns'], 0)
        self.assertEqual(row['callback_thread_cpu_end_ns']-row['callback_thread_cpu_begin_ns'], 14_368_397)
        self.assertEqual(row['diagnostic_offered'], 1)

    def test_hold20_tail_can_be_distinguished_as_publish_or_post_publish_delivery(self):
        # Recorded hold20 candidate 211: compute end 39.173783 ms, gate receipt
        # 46.395216 ms. These alternative injections explain the same old
        # record; neither is asserted to be the actual, still unobserved cause.
        actual_gate_ns = 46_395_216
        tail_ns = 7_221_433
        observations = []
        for publish_delay in (tail_ns, 89_181):
            args, clock, _, _, _ = make_case(publish_ns=publish_delay,
                clock=Clock(24_805_456))
            process_snapshot(*args, clock_ns=clock.now, thread_clock_ns=clock.thread)
            row = row_dict(args[-1])
            self.assertEqual(row['proposal_compute_end_ns'], 39_173_783)
            observed_publish = row['publish_end_ns']-row['publish_begin_ns']
            after_publish = actual_gate_ns-row['publish_end_ns']
            self.assertEqual(observed_publish+after_publish, tail_ns)
            observations.append((observed_publish, after_publish))
        self.assertEqual(observations, [(7_221_433, 0), (89_181, 7_132_252)])

    def test_post_publish_recording_delays_next_callback_not_current_publish(self):
        args, clock, _, _, _ = make_case(serialize_ns=200_000, offer_ns=659_663)
        process_snapshot(*args, clock_ns=clock.now, thread_clock_ns=clock.thread)
        first = row_dict(args[-1])
        process_snapshot(*args, clock_ns=clock.now, thread_clock_ns=clock.thread)
        second = row_dict(args[-1])
        self.assertEqual(first['publish_end_ns'], first['proposal_compute_end_ns'])
        self.assertEqual(first['callback_exit_ns']-first['publish_end_ns'], 1_059_663)
        self.assertGreaterEqual(second['callback_entry_ns'], first['callback_exit_ns'])

    def test_hold20_measured_tail_misses_slot_without_relaxing_original_lease(self):
        # Software timeline replay only: preserve 50 ms issue-based validity
        # and the recorded candidate 212 not yet available at the fault tick.
        commands = [(210, 0, 21_884_233), (211, 19_714_370, 46_395_216)]
        ticks = [44_023_981, 64_012_920, 84_014_652]
        observed = []
        for tick in ticks:
            sequence, issued, _ = max((r for r in commands if r[2] <= tick), key=lambda r: r[0])
            age = tick-issued
            observed.append((sequence, age, age <= 50_000_000))
        self.assertEqual(observed, [(210, 44_023_981, True),
            (211, 44_298_550, True), (211, 64_300_282, False)])
        # At 211's compute end, the immediate next slot had 4.85 ms margin.
        # Removing a 0.66 ms pickle cannot fix a 7.22 ms pre-receipt delay:
        # it is after publish, and even that optimistic subtraction misses.
        self.assertEqual(ticks[0]-39_173_783, 4_850_198)
        self.assertGreater(46_395_216-659_663, ticks[0])

    def test_failed_publish_records_unreached_phases_and_does_not_offer(self):
        args, clock, _, packets, _ = make_case()
        args[2].publish = Mock(side_effect=RuntimeError('publish rejected'))
        with self.assertRaisesRegex(RuntimeError, 'publish rejected'):
            process_snapshot(*args, clock_ns=clock.now, thread_clock_ns=clock.thread)
        row = row_dict(args[-1])
        self.assertGreater(row['publish_begin_ns'], 0)
        self.assertEqual(row['publish_end_ns'], 0)
        self.assertEqual(row['offer_begin_ns'], 0)
        self.assertEqual(row['diagnostic_offered'], 0)
        self.assertEqual(packets, [])

    def test_bounded_trace_overflow_is_reported_without_control_changes(self):
        trace = WorkerTimingTrace(capacity=1)
        row = [0]*len(trace.columns)
        trace.append(row)
        trace.append(list(row))
        self.assertEqual(len(trace.rows), 1)
        self.assertEqual(trace.dropped, 1)
        path = Path(__file__).resolve().parent/('_worker_timing_'+uuid.uuid4().hex+'.json')
        self.addCleanup(lambda: path.unlink(missing_ok=True))
        trace.write(path)
        saved = json.loads(path.read_text())
        self.assertFalse(saved['complete'])
        self.assertEqual(saved['columns'], list(trace.columns))

    def test_sender_timestamps_do_not_change_wire_payload(self):
        clock = Clock()
        packets = []
        sock = Mock()
        def send(packet):
            clock.wait(25)
            packets.append(packet)
            return len(packet)
        sock.send.side_effect = send
        with patch('flyarm_mpc_reference.diagnostic_channel.socket.socket', return_value=sock), \
             patch('flyarm_mpc_reference.diagnostic_channel.socket.AF_UNIX', 1, create=True):
            sender = DiagnosticSender('unused', clock_ns=clock.now)
        value = {'array': [1., 2.], 'bytes': b'exact'}
        self.assertTrue(sender.offer(211, value))
        self.assertEqual(packets, [HEADER.pack(MAGIC, 211)+pickle.dumps(value, protocol=5)])
        self.assertEqual(sender.last_timing_ns, (100, 100, 100, 125, 125))
        self.assertEqual(sender.timings_ns, [25])
        sock.send.side_effect = BlockingIOError('full')
        self.assertFalse(sender.offer(212, value))
        self.assertEqual(sender.last_timing_ns, (125, 125, 125, 0, 125))
        self.assertEqual(sender.dropped, 1)


if __name__ == '__main__':
    unittest.main()
