"""Physics adoption and an asynchronous observation are different evidence."""
import copy
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]/'scripts'))
from audit_x650_streaming_consumption import audit_consumption, audit_async_ledger, physics_rows


def fixture(adoptions=None):
    sends = []
    for i, sim in enumerate((.010, .030, .050)):
        sends.append(dict(session_id=1, epoch=2, command_sequence=i+1,
            dispatch_sequence=10+i, sent_monotonic_ns=100_000_000+i*20_000_000,
            sim_s=sim, repeated=False, stop_zero=i==2, expected_counter=[11+i,21+i],
            joint_velocity=[.1*(i+1), -.1*(i+1)] if i<2 else [0.,0.]))
    if adoptions is None:
        adoptions = [(12,0,0),(12,1,0),(32,0,1),(32,1,1),(52,0,2),(52,1,2)]
    held = [10,20]
    velocity = [0.,0.]
    receipt = [0.,0.]
    rows = []
    for millisecond in range(1,61):
        for at,j,index in adoptions:
            if at == millisecond:
                held[j] = sends[index]['expected_counter'][j]
                velocity[j] = sends[index]['joint_velocity'][j]
                receipt[j] = (millisecond-1)/1000
        r = {'sim_s':millisecond/1000, 'interval_begin_s':(millisecond-1)/1000, 'step_dt_s':.001}
        for j in range(2):
            r.update({f'upper_sequence{j+1}':held[j],f'u{j+1}':velocity[j],f'receipt{j+1}_s':receipt[j]})
        rows.append(r)
    receipts = []
    for i,s in enumerate(sends):
        match = next((r for r in rows if [r['upper_sequence1'],r['upper_sequence2']]==s['expected_counter']), None)
        if match:
            receipts.append(dict(session_id=1,epoch=2,command_sequence=i+1,dispatch_sequence=10+i,
                observed_monotonic_ns=s['sent_monotonic_ns']+3_000_000,
                observation_sim_s=match['sim_s'],joint_consumed=True,pva_send_returned=i!=2,failed=False,
                consumed_counter=s['expected_counter'][:],consumed_sim_s=[match['receipt1_s'],match['receipt2_s']]))
    return dict(sends=sends,receipts=receipts,source=rows,
        start=dict(session_id=1,epoch=2,issued_monotonic_ns=90_000_000),
        release=dict(zero_consumed=True,writer_exited=True,wall_monotonic_ns=150_000_000),publications=[])


class StreamingConsumptionAuditTest(unittest.TestCase):
    def test_complete_adoption_can_have_no_intermediate_ros_receipts(self):
        f=fixture();f['receipts']=f['receipts'][-1:]
        r=audit_consumption(**f)
        self.assertTrue(r['passed']);self.assertTrue(r['all_sends_applied'])
        self.assertEqual(r['observed_receipt_count'],1)
        self.assertEqual(r['joint_counts'][0],{'adopted':3})
        self.assertFalse(r['complete_physical_hold_qualified'])

    def test_overwritten_commands_are_not_reported_as_applied(self):
        f=fixture([(32,0,1),(32,1,1),(52,0,2),(52,1,2)])
        r=audit_consumption(**f)
        self.assertTrue(r['passed']);self.assertFalse(r['all_sends_applied'])
        self.assertEqual(r['joint_counts'][0],{'superseded_before_physics_adoption':1,'adopted':2})
        self.assertFalse(r['sends'][0]['receipt_observed'])

    def test_asynchronous_joints_do_not_claim_same_boundary_adoption(self):
        f=fixture([(12,0,0),(14,1,0),(32,0,1),(34,1,1),(52,0,2),(54,1,2)])
        r=audit_consumption(**f)
        self.assertEqual(r['same_physics_boundary_adoptions'],0)
        self.assertEqual(r['both_joint_values_observed_together'],3)

    def test_mixed_batches_preserve_individual_adoption(self):
        f=fixture([(12,0,0),(32,0,1),(34,1,0),(36,1,1),(52,0,2),(54,1,2)])
        r=audit_consumption(**f)
        self.assertTrue(r['all_sends_applied'])
        self.assertFalse(r['sends'][0]['both_joint_values_observed_together'])
        self.assertFalse(r['sends'][0]['receipt_observed'])

    def test_missing_csv_row_cannot_prove_supersession(self):
        f=fixture();del f['source'][11]
        with self.assertRaisesRegex(ValueError,'Missing/overlapping'):audit_consumption(**f)

    def test_jump_cannot_acknowledge_skipped_send(self):
        f=fixture([(32,0,1),(32,1,1),(52,0,2),(52,1,2)])
        fake=copy.deepcopy(f['receipts'][0]);fake.update(command_sequence=1,dispatch_sequence=10)
        f['receipts'].insert(0,fake)
        with self.assertRaisesRegex(ValueError,'both exact counters'):audit_consumption(**f)

    def test_cross_row_joint_receipt_is_not_one_observation(self):
        f=fixture([(12,0,0),(32,0,1),(34,1,0),(36,1,1),(52,0,2),(54,1,2)])
        fake=copy.deepcopy(f['receipts'][0]);fake.update(command_sequence=1,dispatch_sequence=10,
            consumed_counter=[11,21],consumed_sim_s=[.011,.033],observation_sim_s=.034)
        f['receipts'].insert(0,fake)
        with self.assertRaisesRegex(ValueError,'both exact counters'):audit_consumption(**f)

    def test_unknown_writer_value_and_epoch_fail(self):
        for field in ('u1','upper_sequence1','session_id'):
            f=fixture()
            if field=='session_id':f['sends'][1][field]=3
            else:f['source'][31][field]+=1
            with self.subTest(field=field),self.assertRaises(ValueError):audit_consumption(**f)

    def test_zero_requires_actual_receipt_and_owner_release(self):
        f=fixture();f['receipts'].pop()
        with self.assertRaisesRegex(ValueError,'Final zero'):audit_consumption(**f)
        f=fixture();f['release']['writer_exited']=False
        with self.assertRaisesRegex(ValueError,'release unverified'):audit_consumption(**f)

    def test_wrong_initial_counter_or_missing_send_rejected(self):
        f=fixture();f['sends'][0]['expected_counter'][0]+=1
        with self.assertRaisesRegex(ValueError,'baseline'):audit_consumption(**f)
        f=fixture();f['sends'].pop(1)
        with self.assertRaisesRegex(ValueError,'counter gap'):audit_consumption(**f)

    def test_no_nonfinite_or_inconsistent_counter_allowed(self):
        f=fixture();f['source'][15]['u1']=float('nan')
        with self.assertRaisesRegex(ValueError,'Nonfinite'):audit_consumption(**f)
        f=fixture();f['source'][15]['receipt1_s']+=.001
        with self.assertRaisesRegex(ValueError,'Unchanged counter'):audit_consumption(**f)

    def test_no_overlapping_legacy_publications(self):
        f=fixture();f['publications']=[{'publish_before':{'wall_monotonic_s':.13}}]
        with self.assertRaisesRegex(ValueError,'Overlapping'):audit_consumption(**f)

    def test_counter_parse_does_not_round_large_uint64(self):
        f=fixture();f['source'][0]['upper_sequence1']='18446744073709551615'
        self.assertEqual(physics_rows(f['source'][:1])[0]['counter'][0],2**64-1)


def online_fixture():
    f=fixture()
    baseline=dict(session_id=1,epoch=2,observed_monotonic_ns=98_000_000,sim_s=.010,
                  consumed_counter=[10,20],consumed_velocity=[0.,0.],consumed_sim_s=[0.,0.])
    ledger=dict(schema='latest_value_ledger_v1',complete=True,overflow=False,consumption_policy='latest_value_v1',
                consumption_progress_budget_ms=40,capacity=16,have_epoch_baseline=True,initial_counter=[10,20],
                baseline_received_ns=99_000_000,ignored_old_states=0,rows=[])
    for s,r in zip(f['sends'],f['receipts']):
        ledger['rows'].append(dict(command_sequence=s['command_sequence'],dispatch_sequence=s['dispatch_sequence'],
            sent_ns=s['sent_monotonic_ns'],sim_s=s['sim_s'],stop_zero=s['stop_zero'],paired=True,
            expected_counter=s['expected_counter'][:],velocity=s['joint_velocity'][:],joint_status=[1,1],
            consumed_sim_s=r['consumed_sim_s'][:],observation_sim_s=[r['observation_sim_s']]*2,
            observed_ns=[s['sent_monotonic_ns']+1_000_000]*2,received_ns=[s['sent_monotonic_ns']+2_000_000]*2))
    return {k:v for k,v in f.items() if k!='publications'} | dict(ledger=ledger,baseline=baseline)


class StreamingOnlineLedgerTest(unittest.TestCase):
    def test_complete_original_baseline_and_ledger(self):
        r=audit_async_ledger(**online_fixture())
        self.assertTrue(r['passed']);self.assertEqual(r['rows'],3)
        self.assertEqual(r['per_joint_progress_observation_ms']['max'],2.)

    def test_every_send_must_exist_and_in_order(self):
        f=online_fixture();f['ledger']['rows'].pop(1)
        with self.assertRaisesRegex(ValueError,'every actual send'):audit_async_ledger(**f)

    def test_wrong_baseline_cannot_relabel_receive_counters(self):
        f=online_fixture();f['baseline']['consumed_counter'][0]+=1
        with self.assertRaisesRegex(ValueError,'baseline'):audit_async_ledger(**f)

    def test_status_cannot_invent_adoption(self):
        f=online_fixture();f['ledger']['rows'][0]['joint_status'][0]=2
        with self.assertRaisesRegex(ValueError,'contradicts'):audit_async_ledger(**f)

    def test_late_progress_not_renewed_by_late_observation(self):
        f=online_fixture();e=f['ledger']['rows'][0]
        e['received_ns']=[141_000_000]*2;e['observed_ns']=[140_000_000]*2
        with self.assertRaisesRegex(ValueError,'40 ms'):audit_async_ledger(**f)

    def test_state_age_stays_25ms(self):
        f=online_fixture();f['ledger']['rows'][1]['observed_ns']=[95_000_000]*2
        with self.assertRaisesRegex(ValueError,'freshness'):audit_async_ledger(**f)

    def test_paired_observation_receipt_loss_is_evidence_failure(self):
        f=online_fixture();f['receipts'].pop(0)
        with self.assertRaisesRegex(ValueError,'receipt set'):audit_async_ledger(**f)

    def test_ledger_overflow_cannot_qualify(self):
        f=online_fixture();f['ledger']['overflow']=True
        with self.assertRaisesRegex(ValueError,'overflow'):audit_async_ledger(**f)

    def test_skipped_online_observation_is_not_falsely_relabelled(self):
        f=online_fixture();first,second=f['ledger']['rows'][:2]
        for name in ('consumed_sim_s','observation_sim_s','observed_ns','received_ns'):
            first[name]=second[name][:]
        first['joint_status']=[2,2];first['paired']=False;f['receipts'].pop(0)
        # Native CSV proves this command was adopted, although latest-value
        # ROS missed it. Online status2 must retain its weaker exact meaning.
        self.assertTrue(audit_async_ledger(**f)['passed'])

    def test_stop_uses_release_timeout_not_tracking_progress_budget(self):
        f=online_fixture();e=f['ledger']['rows'][-1]
        e['observed_ns']=[181_000_000]*2;e['received_ns']=[182_000_000]*2
        f['receipts'][-1]['observed_monotonic_ns']=183_000_000
        f['release']['wall_monotonic_ns']=184_000_000
        r=audit_async_ledger(**f)
        self.assertEqual(r['stop_progress_observation_ms'],[42.,42.])


if __name__ == '__main__':unittest.main()
