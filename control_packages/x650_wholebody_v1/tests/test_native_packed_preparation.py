"""Native packed preparation: mandatory with the two explicitly bound libraries."""
import hashlib
import os
from pathlib import Path
import unittest
from copy import deepcopy
from unittest.mock import patch
import numpy as np
from test_ts20_mpc import setup
from test_ts20_prediction import prediction,packet,state
from x650_wholebody.native_optimization import NativeOptimization
from x650_wholebody.native_joint_prediction import NativeJointMatrices
from x650_wholebody.native_mpc import NativeTs20MpcController
from x650_wholebody.ts20 import POSITION_STEP_RAD,VELOCITY_STEP_RAD_S

LIB=os.environ.get("X650_NATIVE_OPTIMIZATION_LIBRARY")
PREDICTOR=os.environ.get("X650_NATIVE_PREDICTION_LIBRARY")


@unittest.skipUnless(LIB and PREDICTOR,"explicit bound native libraries required")
class NativePackedPreparation(unittest.TestCase):
    def setUp(self):
        self.controller,self.reference,self.state,self.target=setup()
        self.reference.enable_native(PREDICTOR,
            expected_sha256=hashlib.sha256(Path(PREDICTOR).read_bytes()).hexdigest())
        self.kernel=NativeOptimization(LIB,hashlib.sha256(Path(LIB).read_bytes()).hexdigest())
        self.native=NativeJointMatrices(self.reference,self.kernel)

    @staticmethod
    def compare_tree(test,left,right):
        if isinstance(left,dict):
            test.assertEqual(set(left),set(right))
            for key in left:test.compare_tree(test,left[key],right[key])
        elif isinstance(left,(tuple,list)):
            test.assertEqual(len(left),len(right))
            for a,b in zip(left,right):test.compare_tree(test,a,b)
        elif isinstance(left,np.ndarray):
            np.testing.assert_array_equal(left,right)
        else:test.assertEqual(left,right)

    def packed(self,**kwargs):
        # Keep the fixture's real input handling but select the private method.
        with patch.object(self.native,'predict',self.native._predict_packed):
            return prediction(self.native,**kwargs)

    def test_packed_rows_match_public_nonuniform_and_changed_candidate(self):
        for first in (.012,.024):
            nodes=np.cumsum([first,.04,.04,.04])
            nominal=np.tile([.03,-.04,.01,.09,-.07],(len(nodes),1))
            kwargs=dict(nodes=nodes,nominal=nominal,
                        value=state(q=(.001,-.002),dq=(.02,-.03),stamp=9.996))
            public=prediction(self.native,**kwargs)
            packed=self.packed(**kwargs)
            self.assertTrue(packed['_native_packed'])
            expected=np.array([np.r_[q,dq,Q.ravel(),D.ravel()] for _,q,dq,Q,D in public['nodes']])
            references=np.array([np.r_[r,R.ravel()] for _,r,R in public['reference_nodes']])
            np.testing.assert_array_equal(packed['native_joint_rows'],expected)
            np.testing.assert_array_equal(packed['native_reference_rows'],references)
            for key in ('q_current','dq_current','r_current','nominal_q','nominal_dq','nominal_modes'):
                np.testing.assert_array_equal(packed[key],public[key],err_msg=key)
            candidate=-nominal
            expected=self.native.candidate_rollout(public,candidate)
            actual=self.native._candidate_packed(packed,candidate)
            for key in ('time_s','q','dq','modes','reference_nodes'):
                np.testing.assert_array_equal(actual[key],expected[key],err_msg=key)
            self.compare_tree(self,actual['envelope'],expected['envelope'])
            self.assertNotIn('events',actual)
            self.assertNotIn('packets',actual)
            self.assertTrue(actual['envelope']['collected'])

    def test_private_packet_arrays_exact_at_half_lsb_and_sent_now(self):
        nodes=np.cumsum([.012,.04,.04])
        for toward in (-np.inf,0.,np.inf):
            q=.5*POSITION_STEP_RAD;v=.5*VELOCITY_STEP_RAD_S
            if toward:q,v=np.nextafter(q,toward),np.nextafter(v,toward)
            nominal=np.tile([0.,0.,0.,v,-v],(len(nodes),1));r=np.array([q,-q])
            sent=[packet(.02,q=r)]
            for sensitivities in (False,True):
                expected,P,V=self.native._future_packets(.02,nodes,nominal,np.array([.45,.3]),
                    r,deepcopy(sent),0.,sensitivities=sensitivities)
                actual,aP,aV=self.native._future_packed(.02,nodes,nominal,np.array([.45,.3]),
                    r,self.native._packet_arrays(sent),0.,sensitivities=sensitivities)
                for left,right in zip(actual,self.native._packet_arrays(expected)):
                    np.testing.assert_array_equal(left,right)
                self.assertGreater(actual[0][1],.02)
                if sensitivities:
                    np.testing.assert_array_equal(P,aP);np.testing.assert_array_equal(V,aV)
                    np.testing.assert_array_equal(aP[0],0.);np.testing.assert_array_equal(aV[0],0.)

    def test_packed_candidates_recompute_without_mutating_measurement_or_nominal(self):
        nominal=np.tile([0.,0.,0.,.05,-.04],(3,1))
        packed=self.packed(nominal=nominal)
        before=deepcopy(packed)
        first=self.native._candidate_packed(packed,-nominal)
        expected=deepcopy(first)
        self.native._candidate_packed(packed,nominal*2)
        self.compare_tree(self,packed,before)
        self.compare_tree(self,first,expected)
        for result in (first['q'],first['dq'],first['reference_nodes']):
            self.assertFalse(np.shares_memory(result,packed['native_joint_rows']))
        first['q'][:]=999.
        self.compare_tree(self,packed,before)

    def test_full_matrix_assembly_and_candidate_check_match_public_path(self):
        c=self.controller
        c.__class__=NativeTs20MpcController
        c.native_optimization=self.kernel;c.actuator_predictor=self.native
        actual=c._problem(0.,.02,self.state,self.target)
        with patch.object(self.native,'_predict_packed',self.native.predict):
            expected=c._problem(0.,.02,self.state,self.target)
        for key in ('A','b','H','g','C','lo','hi','offsets','jacobians','first_lower','first_upper'):
            np.testing.assert_array_equal(actual[key],expected[key],err_msg=key)
        z,_,_=self.kernel.solve(actual,c.max_iterations)
        plan=(z*actual['scale']).reshape(c.nodes,5)
        predicted=actual['offsets']+np.einsum('nij,j->ni',actual['jacobians'],z)
        result,diag=c._validate_candidate(0.,self.state,actual,plan,predicted)
        other,other_diag=c._validate_candidate(0.,self.state,expected,plan,predicted)
        np.testing.assert_array_equal(result,other)
        self.compare_tree(self,diag,other_diag)

    def test_exact_local_uses_complete_existing_event_path(self):
        nodes=np.array([.02,.06]);h=np.diff(np.r_[0.,nodes]);nominal=np.zeros((2,5))
        args=(0.,nodes,h,state(),[(-.01,np.zeros(2))],[.45,.35],nominal)
        kwargs=dict(reference_state={'time_s':-.01,'r':np.zeros(2)},
                    packet_history=[packet(-.01),packet(-.005)],exact_local=True)
        actual=self.native._predict_packed(*args,**kwargs)
        expected=self.native.predict(*args,**kwargs)
        self.assertNotIn('_native_packed',actual)
        self.compare_tree(self,actual,expected)

    def test_private_prediction_rejects_bad_grid_future_packets_and_stale_measurement(self):
        for kwargs in (dict(packet_history=[packet(.005)]),
                       dict(packet_history=[packet(-.01),packet(-.006)]),
                       dict(value=state(stamp=9.7))):
            with self.assertRaises(ValueError):self.packed(**kwargs)


if __name__=='__main__':unittest.main()
