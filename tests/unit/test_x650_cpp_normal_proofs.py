"""Offline proof-verifier negatives; no ROS dependency or controller execution."""
from copy import deepcopy
import importlib.util
from pathlib import Path
import sys
import unittest
import numpy as np

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'scripts'))
import validate_x650_cpp_normal_reference as reference
import probe_x650_cpp_normal_worker as worker


class ReferenceProofTests(unittest.TestCase):
    def cases(self):
        return [dict(kind=1,t=t,dt=dt,nodes=15,node_dt=.04,center=[1.2,-.3,2.1])
                for t in (1.99,3.99,25.99,27.99) for dt in (.012,.016,.020,.024)]

    def test_cumulative_actual_step_and_c2_boundaries(self):
        cases=self.cases();actual=reference.expected(cases)
        self.assertEqual(max(reference.compare(actual,cases).values()),0.)
        self.assertEqual(len(reference.c2_checks()),4)
        self.assertEqual(actual[1,2],cases[0]['t']+cases[0]['dt'])
        self.assertNotEqual(actual[1,2],cases[0]['t']+.02)

    def test_missing_duplicate_reordered_or_wrong_pva_is_rejected(self):
        cases=self.cases();base=reference.expected(cases)
        bad=[base[:-1],base[::-1],np.vstack([base[:1],base[:1],base[2:]])]
        for col in (0,1,2,3,6,9):
            x=base.copy();x[3,col]+=1e-6;bad.append(x)
        x=base.copy();x[4,6]=float('nan');bad.append(x)
        for i,value in enumerate(bad):
            with self.subTest(i=i),self.assertRaises(ValueError):reference.compare(value,cases)

    def test_original_hold_stays_constant(self):
        cases=[dict(kind=0,t=4.,dt=.012,nodes=15,node_dt=.04,center=[1.,2.,3.])]
        expected=reference.expected(cases)
        np.testing.assert_array_equal(expected[:,3:6],np.tile([1.,2.,3.],(16,1)))
        self.assertTrue(np.all(expected[:,6:]==0.))


class RawSnapshotTests(unittest.TestCase):
    def fixture(self):
        state=dict(control_time=12.4,stamp=12.396,joint_stamp=12.396,position=[1.,2.,3.],velocity=[.1,0.,0.],
            omega_body=[0.]*3,acceleration=[0.]*3,rotation=np.eye(3).ravel().tolist(),q=[.1,.2],dq=[.01,.02])
        packet=dict(send_time=2.395,q_ref=[.1,.2],dq_ref=[.01,.02],tau_ff=[0.,0.],kp=[6.,6.],kd=[.25,.25])
        goal=dict(position=[1.,2.,3.],velocity=[.1,0.,0.],acceleration=[.02,0.,.03])
        return dict(sequence=120,input=dict(t=2.4,dt=.02,state=state,target=goal,
            context=dict(reference_time=2.396,bus_origin=0.,reference=[.1,.2],
                commands=[dict(time=2.38,velocity=[.01,.02])],packets=[packet])))

    def test_raw_context_and_current_dynamic_target_are_not_recomputed(self):
        row=self.fixture();before=deepcopy(row);out=worker.snapshot_fields(row,dict(base_position_gain=[.95,.95,1.]))
        self.assertEqual(row,before)
        self.assertEqual(out['reference_time_s'],2.396);self.assertEqual(out['target_acceleration_enu'],[.02,0.,.03])
        self.assertEqual(out['packet_send_time_s'][0],2.395)
        self.assertEqual(out['packet_q_ref'][:2],[.1,.2]);self.assertEqual(out['packet_kp'][:2],[6.,6.])
        self.assertEqual(out['command_time_s'][1:],[0.]*63)
        self.assertEqual(out['command_velocity'][2:],[0.]*126)
        self.assertEqual(out['packet_q_ref'][2:],[0.]*126)

    def test_overflow_or_empty_history_is_rejected(self):
        for name,count in (('commands',0),('packets',0),('commands',65),('packets',65)):
            row=self.fixture();values=row['input']['context'][name];row['input']['context'][name]=values*count
            with self.subTest(name=name,count=count),self.assertRaises(ValueError):
                worker.snapshot_fields(row,dict(base_position_gain=[1.,1.,1.]))

    def test_wrong_shape_or_nonfinite_evidence_cannot_match(self):
        for a,b in (([1.],[1.,2.]),([float('nan')],[1.]),([1.],[float('inf')])):
            with self.assertRaises(ValueError):worker.array_error(a,b)

    def test_preorigin_or_future_packet_fixture_rejected_before_ros(self):
        for time in (-.005,2.4+2e-10):
            row=self.fixture();row['input']['context']['packets'][0]['send_time']=time
            with self.subTest(time=time),self.assertRaisesRegex(ValueError,'bus boundary'):
                worker.snapshot_fields(row,dict(base_position_gain=[1.,1.,1.]))
        row=self.fixture();row['input']['context']['packets'][0]['send_time']=-.005
        row['input']['context']['bus_origin']=-.025
        self.assertEqual(worker.snapshot_fields(row,dict(base_position_gain=[1.,1.,1.]))['packet_send_time_s'][0],-.005)

    def test_exact_message_fields_reject_changed_scalar_or_array(self):
        class Message:
            def __init__(self):self.sequence=1;self.position=np.array([.1,.2,.3])
            def get_fields_and_field_types(self):return dict(sequence='uint64',position='double[3]')
        left=Message();right=Message();worker.require_identical_message_fields(left,right)
        for field in ('sequence','position'):
            right=Message()
            if field=='sequence':right.sequence=2
            else:right.position[0]=np.nextafter(right.position[0],1.)
            with self.subTest(field=field),self.assertRaisesRegex(ValueError,'CDR field differs'):
                worker.require_identical_message_fields(left,right)


if __name__=='__main__':unittest.main()
