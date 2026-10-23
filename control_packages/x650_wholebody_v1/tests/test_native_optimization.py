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
from x650_wholebody.mpc import MpcController,MpcSolveError

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
