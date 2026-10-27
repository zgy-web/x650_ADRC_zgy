import copy
import hashlib
from pathlib import Path
import sys
import unittest

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'scripts'))
from x650_pil_transport import encode, validate_candidate


class PilTransportTests(unittest.TestCase):
    def setUp(self):
        self.source='a'*64
        s=dict(schema_version=1,session_id=7,epoch=3,sequence=1,configuration_sha256=[9]*32,
            issued_monotonic_ns=1_000_000_000,control_time_s=10.04,task_time_s=.04,
            position_gains=[1.,1.,1.],position_enu=[0.,0.,2.])
        self.request=dict(op='step',snapshot=s)
        p={k:s[k] for k in ('schema_version','session_id','epoch','sequence','configuration_sha256','control_time_s','task_time_s')}
        p.update(snapshot_sequence=1,replay_only=False,status=0,accepted_optimum=True,
            nonlinear_check_passed=True,subproblems=1,primal_violation=0.,kkt_residual=0.,
            compute_start_monotonic_ns=9_000_000_000_000,compute_end_monotonic_ns=9_000_001_000_000,
            position_reference_enu=[0.,0.,2.],velocity_feedforward_enu=[.1,0.,0.],
            acceleration_feedforward_enu=[0.,0.,0.],base_velocity_target=[.1,0.,0.],joint_velocity=[0.,0.])
        self.response=dict(request_sha256=hashlib.sha256(encode(self.request)).hexdigest(),source_sha256=self.source,
            status='candidate',source_issue_ns=s['issued_monotonic_ns'],proposal_nx_clock=p,
            hold_guard_passed=True,hold_guard_s=.16,memory_step_count=1,core_wall_ns=1_000_000,
            core_thread_ns=900_000,nx_received_ns=p['compute_start_monotonic_ns'],nx_service_ns=1_500_000)

    def check(self,response=None,**kw):
        times=dict(sent_ns=1_001_000_000,received_ns=1_010_000_000,issue_ns=1_000_000_000)
        times.update(kw)
        return validate_candidate(self.request,self.response if response is None else response,
            source_sha256=self.source,**times)

    def test_unsynchronized_clocks_are_not_subtracted(self):
        self.assertEqual(self.check()['original_issue_age_ns'],10_000_000)

    def test_unchanged_50ms_boundary(self):
        self.check(received_ns=1_050_000_000)
        with self.assertRaises(ValueError):self.check(received_ns=1_050_000_001)

    def test_issue_age_cannot_be_renewed_at_send(self):
        with self.assertRaises(ValueError):self.check(sent_ns=1_049_000_000,received_ns=1_051_000_000)
        with self.assertRaises(ValueError):self.check(issue_ns=1_002_000_000)

    def test_bound_identity_and_nonfinite_payloads(self):
        for key,value in [('source_issue_ns',1),('memory_step_count',2),('request_sha256','b'*64),('source_sha256','b'*64)]:
            v=copy.deepcopy(self.response);v[key]=value
            with self.subTest(key=key),self.assertRaises(ValueError):self.check(v)
        v=copy.deepcopy(self.response);v['proposal_nx_clock']['joint_velocity'][0]=float('nan')
        with self.assertRaises(ValueError):self.check(v)

    def test_nx_duration_and_pva_checked_independently(self):
        v=copy.deepcopy(self.response);v['core_wall_ns']=50_000_001
        with self.assertRaises(ValueError):self.check(v)
        v=copy.deepcopy(self.response);v['proposal_nx_clock']['velocity_feedforward_enu'][0]=0.
        with self.assertRaises(ValueError):self.check(v)

    def test_recorded_scope_does_not_grant_transport_admission(self):
        self.check(received_ns=2_000_000_000,check_deadline=False)
        with self.assertRaises(ValueError):self.check(received_ns=2_000_000_000)
        with self.assertRaises(ValueError):encode(dict(bad=float('nan')))


if __name__=='__main__':unittest.main()
