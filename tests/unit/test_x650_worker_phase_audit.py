import json
from pathlib import Path
import sys
from types import SimpleNamespace as NS
import unittest
import uuid

sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'scripts'))
from x650_worker_phase_audit import verify_worker_phases


class PhaseAuditTests(unittest.TestCase):
    def setUp(self):
        self.path=Path(__file__).resolve().parent/('_phase_audit_'+uuid.uuid4().hex)
        self.path.mkdir()
        def cleanup():
            for name in ('worker_timing.json','ingress_timing.json'):
                (self.path/name).unlink(missing_ok=True)
            self.path.rmdir()
        self.addCleanup(cleanup)
        names=('callback_entry_ns', 'engine_return_ns', 'publish_begin_ns', 'publish_end_ns',
            'snapshot_cdr_begin_ns', 'snapshot_cdr_end_ns', 'proposal_cdr_begin_ns', 'proposal_cdr_end_ns',
            'offer_begin_ns', 'pickle_begin_ns', 'pickle_end_ns', 'socket_send_begin_ns',
            'socket_send_end_ns', 'sender_end_ns', 'offer_end_ns', 'callback_exit_ns')
        self.row=dict(zip(names,range(100,260,10)))
        self.row.update(sequence=1,snapshot_issued_ns=90,proposal_compute_end_ns=109,
            publish_thread_cpu_begin_ns=10,publish_thread_cpu_end_ns=11,diagnostic_offered=1)
        self.calls=[(NS(sequence=1,session_id=7,epoch=3,issued_monotonic_ns=90,control_time_s=10.02,dt_s=.02),
            NS(sequence=1,snapshot_sequence=1,session_id=7,epoch=3,
                compute_start_monotonic_ns=101,compute_end_monotonic_ns=109),{})]
        # Gate may receive asynchronously before publish returns.
        self.events=[dict(sequence=1,proposal_sequence=1,snapshot_sequence=1,session_id=7,
            epoch=3,accepted=True,received_monotonic_ns=125,roundtrip_ns=35)]
        ingress=dict(kind=4,sequence=1,forward=True,source_issued_ns=90,
            latest_control_s=10.02,last_control_s=10.,released_sequence=0,last_admitted_sequence=0)
        (self.path/'ingress_timing.json').write_text(json.dumps(dict(
            kind='isolated_ingress_timing_v1',complete=True,overflow=0,
            columns=list(ingress),rows=[list(ingress.values())])))

    def write(self, **updates):
        value=dict(kind='isolated_worker_phase_timing_v1',complete=True,dropped=0,
            columns=list(self.row),rows=[list(self.row.values())])
        value.update(updates)
        (self.path/'worker_timing.json').write_text(json.dumps(value))

    def test_complete_original_timestamps_and_asynchronous_gate(self):
        self.write()
        report=verify_worker_phases(self.path,self.calls,self.events)
        self.assertTrue(report['verified'])
        self.assertEqual(report['durations']['compute_end_to_gate_ms']['max'],16/1e6)

    def test_original_timestamp_or_sequence_tamper_is_rejected(self):
        for field,value in [('snapshot_issued_ns',91),('proposal_compute_end_ns',108),('sequence',2)]:
            original=self.row[field]
            self.row[field]=value; self.write()
            with self.assertRaises(ValueError):verify_worker_phases(self.path,self.calls,self.events)
            self.row[field]=original

    def test_loss_overflow_and_impossible_phase_order_are_rejected(self):
        for update in [dict(complete=False,dropped=1),dict(rows=[]),dict(rows=[[]])]:
            self.write(**update)
            with self.assertRaises(ValueError):verify_worker_phases(self.path,self.calls,self.events)
        self.row['socket_send_end_ns']=self.row['pickle_begin_ns']-1
        self.write()
        with self.assertRaises(ValueError):verify_worker_phases(self.path,self.calls,self.events)

    def test_unrelated_duplicate_gate_or_non_tail_missing_event_is_rejected(self):
        self.write()
        for field,value in [('proposal_sequence',999),('snapshot_sequence',999),
                ('session_id',8),('epoch',4),('roundtrip_ns',36)]:
            event=dict(self.events[0]);event[field]=value
            with self.assertRaises(ValueError):verify_worker_phases(self.path,self.calls,[event])
        with self.assertRaises(ValueError):verify_worker_phases(self.path,self.calls,self.events*2)
        self.add_pending_tail()
        self.write(rows=[list(self.row.values()),list(self.second.values())])
        wrong=dict(self.events[0],proposal_sequence=2,snapshot_sequence=2)
        with self.assertRaises(ValueError):verify_worker_phases(self.path,self.calls,[wrong])
        with self.assertRaises(ValueError):verify_worker_phases(self.path,self.calls,[])

    def add_pending_tail(self):
        self.second=dict(self.row)
        for key in self.second:
            if key.endswith('_ns'):self.second[key]+=1000
        self.second['sequence']=2
        self.calls.append((NS(sequence=2,session_id=7,epoch=3,issued_monotonic_ns=1090,
                control_time_s=10.04,dt_s=.02),
            NS(sequence=2,snapshot_sequence=2,session_id=7,epoch=3,
                compute_start_monotonic_ns=1101,compute_end_monotonic_ns=1109),{}))
        path=self.path/'ingress_timing.json';document=json.loads(path.read_text())
        row=dict(zip(document['columns'],document['rows'][0]))
        row.update(sequence=2,source_issued_ns=1090,latest_control_s=10.04,last_control_s=10.02,
            released_sequence=1,last_admitted_sequence=1)
        document['rows'].append([row[name] for name in document['columns']])
        path.write_text(json.dumps(document))

    def test_one_successful_pending_tail_is_allowed(self):
        self.add_pending_tail()
        self.write(rows=[list(self.row.values()),list(self.second.values())])
        report=verify_worker_phases(self.path,self.calls,self.events)
        self.assertEqual(report['pairs'],2)
        self.assertEqual(report['durations']['engine_ms']['count'],2)
        self.assertEqual(report['durations']['compute_end_to_gate_ms']['count'],1)

    def test_duplicate_or_malformed_ingress_trace_is_rejected(self):
        self.write()
        path=self.path/'ingress_timing.json';original=json.loads(path.read_text())
        malformed=[dict(original,columns=original['columns']+[original['columns'][0]]),
            dict(original,rows=[original['rows'][0]+[999]]),
            dict(original,rows=original['rows']*2)]
        for document in malformed:
            path.write_text(json.dumps(document))
            with self.assertRaises(ValueError):verify_worker_phases(self.path,self.calls,self.events)


if __name__=='__main__':unittest.main()
