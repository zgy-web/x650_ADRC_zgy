"""Mandatory native negative/independent tests when the two libraries are set."""
import hashlib
import os
from pathlib import Path
import unittest
import ctypes
from unittest.mock import patch
from copy import deepcopy
import numpy as np
from test_ts20_mpc import setup
from x650_wholebody.native_optimization import NativeOptimization
from x650_wholebody.native_mpc import NativeTs20MpcController
from x650_wholebody.native_joint_prediction import NativeJointMatrices
from x650_wholebody.mpc import MpcController,MpcSolveError
from x650_wholebody.ts20 import POSITION_STEP_RAD,VELOCITY_STEP_RAD_S
from test_ts20_prediction import packet,prediction

LIB=os.environ.get("X650_NATIVE_OPTIMIZATION_LIBRARY")
PREDICTOR=os.environ.get("X650_NATIVE_PREDICTION_LIBRARY")


@unittest.skipUnless(LIB and PREDICTOR,"explicit native libraries required")
class NativeGates(unittest.TestCase):
    def setUp(self):
        self.native=NativeOptimization(LIB,hashlib.sha256(Path(LIB).read_bytes()).hexdigest())

    @staticmethod
    def problem():
        return {"H":np.array([[4.,1.],[1.,2.]]),"g":np.array([-5.,-3.]),
                "C":np.array([[1.,1.],[1.,0.],[1.,1.]]),
                "lo":np.array([-2.,-.8,-2.]),"hi":np.array([.4,.8,.4])}

    def test_independent_active_qp_and_duplicate_rows(self):
        p=self.problem();z,q,center=self.native.solve(p,100)
        # Solve KKT analytically for x+y=.4; no reference optimizer required.
        K=np.block([[p["H"],np.ones((2,1))],[np.ones((1,2)),np.zeros((1,1))]])
        expected=np.linalg.solve(K,np.r_[-p["g"],.4])[:2]
        np.testing.assert_allclose(z,expected,atol=1e-12,rtol=0)
        self.assertFalse(center)
        self.assertLess(MpcController._quality(p,z)["kkt_stationarity_inf"],1e-10)
        self.assertLess(q["primal_violation"],1e-12)

    def test_interior_solution(self):
        p=self.problem();p["g"]=np.array([-.1,.1])
        z,q,center=self.native.solve(p,100)
        np.testing.assert_allclose(z,-np.linalg.solve(p["H"],p["g"]),atol=1e-14)
        self.assertTrue(center)

    def test_reject_infeasible(self):
        p=self.problem();p["lo"][0]=3.;p["hi"][0]=4.
        with self.assertRaises(ValueError):self.native.solve(p,100)

    def test_reject_bad_hessian(self):
        for h in (np.zeros((2,2)),np.array([[1.,2.],[0.,1.]])):
            p=self.problem();p["H"]=h
            with self.assertRaises(ValueError):self.native.solve(p,100)

    def test_reject_bad_shapes_and_nonfinite(self):
        for key,value in (("g",[1.,float("nan")]),("C",np.ones((3,3))),
                          ("lo",[1.,2.]),("hi",[float("inf"),1.,1.])):
            p=self.problem();p[key]=value
            with self.assertRaises(ValueError):self.native.solve(p,100)
        with self.assertRaises(ValueError):NativeOptimization(LIB,"0"*64)

    def test_ffi_pointers_have_no_owner_dictionary_cycle(self):
        observed=[]
        def inspect_call(*args):
            for value in args:
                if isinstance(value,(ctypes.POINTER(ctypes.c_double),ctypes.POINTER(ctypes.c_int))):
                    observed.append(value._objects)
            return 0
        with patch.object(self.native.lib,"x650_assemble",inspect_call):
            self.native.call("x650_assemble",2,np.zeros(2),2,np.zeros(4),4,np.zeros(3,dtype=np.int32))
        self.assertEqual(observed,[None,None,None])

    def test_bounded_active_set(self):
        with self.assertRaises(ValueError):self.native.solve(self.problem(),1)

    def test_per_row_precision_survives_dimension_and_row_scaling(self):
        # In the original 75-variable/230-row solver, 6.54e-8 normalized
        # violation was below m*eps*n*n*100, although 5x that in original
        # row units exceeded the unchanged 2e-7 acceptance gate.
        for n in (5,75):
            for factor in (.005,5.,5000.):
                for sign in (-1.,1.):
                    with self.subTest(n=n,factor=factor,sign=sign):
                        C=np.zeros((230,n));C[:,0]=sign*factor
                        bound=6.541115840363876e-8
                        p={"H":np.eye(n),"g":np.zeros(n),"C":C,
                           "lo":np.full(230,factor*bound),
                           "hi":np.full(230,10.*factor)}
                        z,q,center=self.native.solve(p,100)
                        expected=np.zeros(n);expected[0]=sign*bound
                        np.testing.assert_allclose(z,expected,atol=1e-13,rtol=0)
                        self.assertFalse(center)
                        self.assertLess(q["primal_violation"],1e-10)

    def test_recorded_near_active_row_is_actually_converged(self):
        fixture=Path(__file__).resolve().parents[1]/"native/regressions/hold16_round1416_qp.npz"
        with np.load(fixture) as saved:
            p={key:saved[key] for key in ("H","g","C","lo","hi")}
            old=saved["solution"]
            original=MpcController._quality(p,old)
            self.assertAlmostEqual(original["primal_violation"],3.270557920181938e-7,places=14)
            z,q,_=self.native.solve(p,int(saved["max_iterations"]))
        # Recompute against all original rows, independently of the C++ report.
        independent=MpcController._quality(p,z)
        self.assertLess(independent["primal_violation"],1e-10)
        self.assertLess(independent["kkt_stationarity_inf"],1e-8)
        self.assertLess(q["primal_violation"],1e-10)

    def test_joint_recurrence_matches_reference_at_packet_boundaries(self):
        _,p,_,_=setup()
        rng=np.random.default_rng(20261023)
        start=-.008
        bounds=np.array([start,0.,.002,.005,.01,.013,.02,.025])
        queries=bounds[[0,1,3,5,7]]
        sends=np.array([-.02,.005,.013,.022])
        for n in (5,75,300):
            P,V=rng.normal(size=(2,len(sends),2,n))
            expected=[];S=np.zeros((2,2,n));cursor=start
            index=max(0,int(np.searchsorted(sends,start+1e-11,side='right')-1))
            for end in bounds:
                if end>cursor:
                    A,B=p._transition(end-cursor)
                    force=p.kp[:,None]*P[index]+p.kd[:,None]*V[index]
                    S=np.einsum('jik,jkn->jin',A,S)+B[:,:,None]*force[:,None,:]
                if len(expected)<len(queries) and abs(queries[len(expected)]-end)<1e-9:
                    expected.append(S.copy())
                cursor=end;index=max(0,int(np.searchsorted(sends,end+1e-11,side='right')-1))
            actual=self.native.joint_sensitivities(p,start,bounds,queries,sends,P,V)
            np.testing.assert_allclose(actual,expected,atol=1e-12,rtol=1e-12)

    def test_joint_recurrence_rejects_bad_times_and_shapes(self):
        _,p,_,_=setup();P=np.zeros((2,2,5))
        for bounds,queries,sends,positions in (
            ([0.,.01],[.005],[-.02,0.],P),
            ([0.,.01],[0.,0.],[-.02,0.],P),
            ([0.,0.],[0.],[-.02,0.],P),
            ([0.,.01],[0.],[0.,-.02],P),
            ([0.,.01],[0.],[-.02,0.],np.zeros((2,3,5))),
        ):
            with self.assertRaises(ValueError):
                self.native.joint_sensitivities(p,0.,bounds,queries,sends,positions,P)

    def test_joint_matrix_kernel_preserves_complete_problem(self):
        c,p,s,t=setup()
        p.enable_native(PREDICTOR,expected_sha256=hashlib.sha256(Path(PREDICTOR).read_bytes()).hexdigest())
        a=c._problem(0.,.02,s,t)
        c.actuator_predictor=NativeJointMatrices(p,self.native)
        b=c._problem(0.,.02,s,t)
        for key in ('A','b','H','g','C','lo','hi','offsets','jacobians'):
            np.testing.assert_allclose(a[key],b[key],atol=1e-9,rtol=1e-9,err_msg=key)

    def test_batched_future_packets_preserve_bus_boundaries_and_sensitivities(self):
        _,p,_,_=setup()
        p.enable_native(PREDICTOR,expected_sha256=hashlib.sha256(Path(PREDICTOR).read_bytes()).hexdigest())
        native=NativeJointMatrices(p,self.native)
        rng=np.random.default_rng(20261024)
        limits=np.array([.45,.3]);r=np.array([-.21,.17])
        for count in (1,15,60):
            nodes=np.cumsum(np.full(count,.02));nominal=rng.uniform(-.2,.2,(count,5))
            for phase in (0.,.003):
                for delta in (-1e-8,0.,1e-8):
                    with self.subTest(count=count,phase=phase,delta=delta):
                        t=phase+.04+delta
                        sent=[{'send_time_s':phase+.02,'q_ref':r.copy(),'dq_ref':np.zeros(2),
                               'tau_ff':np.zeros(2),'kp':p.kp.copy(),'kd':p.kd.copy()}]
                        for sensitivity in (True,False):
                            expected=p._future_packets(t,nodes,nominal,limits,r,deepcopy(sent),phase,
                                                       sensitivities=sensitivity)
                            actual=native._future_packets(t,nodes,nominal,limits,r,deepcopy(sent),phase,
                                                          sensitivities=sensitivity)
                            self.assertEqual(len(actual[0]),len(expected[0]))
                            for a,b in zip(actual[0],expected[0]):
                                self.assertEqual(a['send_time_s'],b['send_time_s'])
                                for key in ('q_ref','dq_ref','tau_ff','kp','kd'):
                                    np.testing.assert_array_equal(a[key],b[key],err_msg=key)
                            if sensitivity:
                                np.testing.assert_array_equal(actual[1],expected[1])
                                np.testing.assert_array_equal(actual[2],expected[2])
                            else:
                                self.assertEqual(actual[1:],(None,None))

    def test_batched_packets_keep_independent_mutable_fields(self):
        _,p,_,_=setup()
        p.enable_native(PREDICTOR,expected_sha256=hashlib.sha256(Path(PREDICTOR).read_bytes()).hexdigest())
        native=NativeJointMatrices(p,self.native)
        packets,P,V=native._future_packets(0.,np.array([.02,.04]),np.zeros((2,5)),
                                           np.array([.45,.3]),np.zeros(2),[],0.)
        saved=deepcopy(packets[1]);gains=p.kp.copy();second=P[1].copy()
        for key in ('q_ref','dq_ref','tau_ff','kp','kd'):
            packets[0][key][:]=99.
            np.testing.assert_array_equal(packets[1][key],saved[key])
        P[0][:]=99.;V[0][:]=99.
        np.testing.assert_array_equal(P[1],second)
        np.testing.assert_array_equal(p.kp,gains)

    def test_nonuniform_horizon_and_half_lsb_packet_boundaries_are_exact(self):
        _,p,_,_=setup()
        p.enable_native(PREDICTOR,expected_sha256=hashlib.sha256(Path(PREDICTOR).read_bytes()).hexdigest())
        native=NativeJointMatrices(p,self.native)
        for first in (.012,.024):
            nodes=np.cumsum([first,.04,.04,.04])
            for toward in (-np.inf,0.,np.inf):
                q=.5*POSITION_STEP_RAD;v=.5*VELOCITY_STEP_RAD_S
                if toward:
                    q,v=np.nextafter(q,toward),np.nextafter(v,toward)
                nominal=np.tile([0.,0.,0.,v,-v],(len(nodes),1));r=np.array([q,-q])
                sent=[packet(.02,q=r)]
                for sensitivity in (False,True):
                    a=p._future_packets(.02,nodes,nominal,np.array([.45,.3]),r,deepcopy(sent),0.,
                                        sensitivities=sensitivity)
                    b=native._future_packets(.02,nodes,nominal,np.array([.45,.3]),r,deepcopy(sent),0.,
                                             sensitivities=sensitivity)
                    self.assertGreater(b[0][1]['send_time_s'],.02)
                    for pa,pb in zip(a[0],b[0]):
                        self.assertEqual(pa['send_time_s'],pb['send_time_s'])
                        for key in ('q_ref','dq_ref','tau_ff','kp','kd'):
                            np.testing.assert_array_equal(pa[key],pb[key])
                    if sensitivity:
                        np.testing.assert_array_equal(a[1],b[1]);np.testing.assert_array_equal(a[2],b[2])
                        np.testing.assert_array_equal(b[1][0],0.);np.testing.assert_array_equal(b[2][0],0.)

    def test_batched_prediction_and_changed_candidate_preserve_complete_results(self):
        _,p,_,_=setup()
        p.enable_native(PREDICTOR,expected_sha256=hashlib.sha256(Path(PREDICTOR).read_bytes()).hexdigest())
        native=NativeJointMatrices(p,self.native)
        for first in (.012,.024):
            nodes=np.cumsum([first,.04,.04,.04]);nominal=np.tile([0.,0.,0.,.09,-.07],(len(nodes),1))
            a=prediction(p,nodes=nodes,nominal=nominal)
            b=prediction(native,nodes=nodes,nominal=nominal)
            for key in ('nodes','events','reference_nodes'):
                self.assertEqual(len(a[key]),len(b[key]))
                for ra,rb in zip(a[key],b[key]):
                    for va,vb in zip(ra,rb):
                        np.testing.assert_allclose(va,vb,atol=1e-12,rtol=1e-12)
            candidate=-nominal
            expected=p.candidate_rollout(a,candidate);actual=native.candidate_rollout(b,candidate)
            for key in ('q','dq','modes','time_s','reference_nodes'):
                np.testing.assert_array_equal(expected[key],actual[key])
            for pa,pb in zip(expected['packets'],actual['packets']):
                for key in pa:np.testing.assert_array_equal(pa[key],pb[key])

    def test_batched_rollout_events_do_not_alias_public_arrays_or_inputs(self):
        _,p,_,_=setup()
        p.enable_native(PREDICTOR,expected_sha256=hashlib.sha256(Path(PREDICTOR).read_bytes()).hexdigest())
        native=NativeJointMatrices(p,self.native)
        q0,dq0=np.array([.01,-.01]),np.array([.1,-.1])
        packets=[packet(-.005,q=(.02,-.03)),packet(0.,q=(.03,-.02))]
        result=native.rollout(0.,q0,dq0,packets,[0.,.02,.04])
        public=[q0,dq0,result['q'],result['dq'],result['modes']]
        public.extend(event[key] for event in result['events'] for key in ('q','dq','modes'))
        before=[value.copy() for value in public]
        for i,left in enumerate(public):
            for right in public[i+1:]:self.assertFalse(np.shares_memory(left,right))
        result['events'][0]['q'][:]=99.
        for value,original in zip(public,before):
            if value is not result['events'][0]['q']:np.testing.assert_array_equal(value,original)

    def candidate_case(self):
        c,_,_,_=setup();N=c.nodes
        predicted=np.zeros((N,12));q=np.zeros((N,2));dq=q.copy()
        actual={"reference_nodes":q.copy(),"envelope":{"collected":True,
            "q_lower":[0.,0.],"q_upper":[0.,0.],"dq_lower":[0.,0.],"dq_upper":[0.,0.]}}
        return c,predicted,q,dq,actual

    def test_candidate_error_thresholds(self):
        c,p,q,dq,a=self.candidate_case()
        self.assertTrue(self.native.candidate(c,p,q,dq,a)[3])
        q[0,0]=.006000001
        self.assertFalse(self.native.candidate(c,p,q,dq,a)[3])
        q[0,0]=0.;dq[-1,1]=.100000001
        self.assertFalse(self.native.candidate(c,p,q,dq,a)[3])

    def test_candidate_full_envelope_and_reference_limits(self):
        for key,value in (("q_upper",[.46,0.]),("dq_upper",[.46,0.]),
                          ("q_lower",[-.76,0.]),("dq_lower",[-.46,0.])):
            c,p,q,dq,a=self.candidate_case();a["envelope"][key]=value
            self.assertFalse(self.native.candidate(c,p,q,dq,a)[2])
        c,p,q,dq,a=self.candidate_case();a["reference_nodes"][0,0]=c.q_upper[0]+1e-5
        self.assertFalse(self.native.candidate(c,p,q,dq,a)[2])

    def test_candidate_missing_or_nonfinite_envelope(self):
        for env in (None,{"collected":False}):
            c,p,q,dq,a=self.candidate_case();a["envelope"]=env
            with self.assertRaises(ValueError):self.native.candidate(c,p,q,dq,a)
        c,p,q,dq,a=self.candidate_case();a["envelope"]["q_upper"][0]=float("nan")
        with self.assertRaises(ValueError):self.native.candidate(c,p,q,dq,a)

    def test_rejection_rolls_back_plan_and_command(self):
        c,p,s,target=setup()
        p.enable_native(PREDICTOR,expected_sha256=hashlib.sha256(Path(PREDICTOR).read_bytes()).hexdigest())
        c.__class__=NativeTs20MpcController;c.native_optimization=self.native
        before={name:deepcopy(getattr(c,name)) for name in
                ("_plan","_plan_t","_plan_h","previous_command","last_t","step_count","_joint_history")}
        def rejected(*args):raise MpcSolveError("injected candidate rejection",{"accepted_optimum":False})
        c._validate_candidate=rejected
        with self.assertRaises(MpcSolveError):c.step(0.,.02,s,target)
        for name,value in before.items():
            if isinstance(value,np.ndarray):np.testing.assert_array_equal(getattr(c,name),value)
            else:self.assertEqual(getattr(c,name),value)


if __name__=="__main__":unittest.main()
