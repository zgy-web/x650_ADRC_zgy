"""QP-node safety margins and unchanged independent full-interval checks.

Native comparisons are mandatory when X650_NATIVE_OPTIMIZATION_LIBRARY is
provided. These software tests do not start ROS/Gazebo or qualify a flight.
"""
from copy import deepcopy
import hashlib
import os
from pathlib import Path
import sys
from types import MethodType
import unittest
from unittest.mock import patch

import numpy as np
from scipy.optimize import Bounds, LinearConstraint, minimize

ROOT=Path(__file__).resolve().parents[2]
PACKAGE=ROOT/'control_packages/x650_wholebody_v1'
sys.path[:0]=[str(PACKAGE),str(PACKAGE/'tests')]
from test_ts20_mpc import setup
from x650_wholebody import prediction_chain as chain
from x650_wholebody.mpc import MpcSolveError
from x650_wholebody.native_optimization import NativeOptimization
from x650_wholebody.ts20_mpc import CANDIDATE_POLICY

LIB=os.environ.get('X650_NATIVE_OPTIMIZATION_LIBRARY')
QPAD=.0003
VPAD=.03
TOL=2e-7


def fake_prediction(controller, *, age=.007, q=None, dq=None):
    """Affine predictor fixture; no fake controller/safety implementation."""
    n=5*controller.nodes
    q=np.array([.12,-.08] if q is None else q,dtype=float)
    dq=np.array([.03,-.02] if dq is None else dq,dtype=float)
    nodes=[];references=[]
    times=np.cumsum([.02,*([controller.node_dt]*(controller.nodes-1))])
    for i,t in enumerate(times):
        Q=np.zeros((2,n));D=np.zeros((2,n));R=np.zeros((2,n))
        Q[0,3]=.025+.003*i;Q[1,4]=.021+.002*i
        D[0,3]=.17+.01*i;D[1,4]=.14+.01*i
        R[0,3]=.01+.001*i;R[1,4]=.008+.001*i
        nodes.append((t,q.copy(),dq.copy(),Q,D))
        references.append((t,np.array([.02,-.01]),R))
    return dict(nodes=nodes,reference_nodes=references,q_current=np.zeros(2),
        dq_current=np.zeros(2),measurement_age_s=age,r_current=np.zeros(2))


def joint_rows(controller,prediction,state):
    controller._actuator_context={'command_history':[(-.01,np.zeros(2))]}
    rows=[]
    def collect(C,lo,hi):rows.append((np.asarray(C).copy(),np.asarray(lo).copy(),np.asarray(hi).copy()))
    warm=np.zeros((controller.nodes,5))
    lo,hi,guard=controller._joint_constraints(collect,prediction,.02,state,0.,warm)
    return rows,lo,hi,guard


def qp_solution(lower,upper,target):
    problem={'H':np.eye(1)*2.,'g':np.array([-2*target]),'C':np.ones((1,1)),
             'lo':np.array([lower]),'hi':np.array([upper])}
    answer=minimize(lambda z:float((z[0]-target)**2),np.array([np.clip(target,lower,upper)]),
        jac=lambda z:2*(z-target),bounds=Bounds([-1.],[1.]),
        constraints=[LinearConstraint(problem['C'],problem['lo'],problem['hi'])],method='SLSQP',
        options={'ftol':1e-13,'maxiter':100})
    if not answer.success:raise AssertionError(answer.message)
    return float(answer.x[0]),problem


class JointSafetyMarginPython(unittest.TestCase):
    def test_policy_values_match_preexisting_candidate_check(self):
        self.assertEqual(CANDIDATE_POLICY['numerical_q_envelope_padding_rad'],QPAD)
        self.assertEqual(CANDIDATE_POLICY['numerical_dq_envelope_padding_rad_s'],VPAD)
        self.assertEqual(CANDIDATE_POLICY['actual_bound_tolerance'],TOL)
        self.assertEqual(CANDIDATE_POLICY['q_linearization_error_max_rad'],.006)
        self.assertEqual(CANDIDATE_POLICY['dq_linearization_error_max_rad_s'],.10)

    def test_only_node_q_and_dq_limits_are_tightened(self):
        c,_,s,_=setup();p=fake_prediction(c)
        rows,firstlo,firsthi,guard=joint_rows(c,p,s)
        reserve=c.velocity_limit[3:]/(2*c.acceleration_limit[3:])+c.max_dt+.05+p['measurement_age_s']
        self.assertEqual(len(rows),3*c.nodes+c.nodes+1)
        for i,(_,q,dq,Q,D) in enumerate(p['nodes']):
            expected=((Q,c.q_lower+QPAD-q,c.q_upper-QPAD-q),
                      (D,-c.velocity_limit[3:]+VPAD-dq,c.velocity_limit[3:]-VPAD-dq),
                      (Q+reserve[:,None]*D,c.q_lower-q-reserve*dq,c.q_upper-q-reserve*dq))
            for got,want in zip(rows[3*i:3*i+3],expected):
                for actual,value in zip(got,want):np.testing.assert_array_equal(actual,value)
        for got,(_,r,R) in zip(rows[3*c.nodes:4*c.nodes],p['reference_nodes']):
            for actual,value in zip(got,(R,c.q_lower-r,c.q_upper-r)):np.testing.assert_array_equal(actual,value)
        legacylo,legacyhi,legacyguard=chain.conservative_first_bounds(c,.02,s,p,
            c._actuator_context['command_history'],0.,{'joint_delay_guard_s':.05})
        np.testing.assert_array_equal(firstlo,legacylo);np.testing.assert_array_equal(firsthi,legacyhi)
        np.testing.assert_array_equal(firstlo,-c.acceleration_limit*.02)
        np.testing.assert_array_equal(firsthi,c.acceleration_limit*.02)
        np.testing.assert_array_equal(guard['q_guard_lower'],legacyguard['q_guard_lower'])
        np.testing.assert_array_equal(guard['q_guard_upper'],legacyguard['q_guard_upper'])
        indices=np.array([5*i+j for i in range(c.nodes) for j in (3,4)])
        np.testing.assert_array_equal(rows[-1][0],np.eye(5*c.nodes)[indices])
        np.testing.assert_array_equal(rows[-1][1],np.full(2*c.nodes,-.02))
        np.testing.assert_array_equal(rows[-1][2],np.full(2*c.nodes,.02))

    def test_command_velocity_can_still_reach_original_unpadded_limit(self):
        c,_,_,_=setup()
        # Commands and predicted mechanical state are different quantities.
        # At a safe interior q, an already-running command may retain its full
        # original limit even though predicted dq nodes reserve .03 rad/s.
        c.previous_command=np.r_[np.zeros(3),c.velocity_limit[3],-c.velocity_limit[4]]
        lo,hi=c._bounds(.02,np.zeros(2))
        self.assertEqual(hi[3],c.velocity_limit[3])
        self.assertEqual(lo[4],-c.velocity_limit[4])

    def test_old_node_boundary_is_now_rejected_by_actual_qp_rows(self):
        c,_,s,_=setup();p=fake_prediction(c,q=[0.,0.],dq=[0.,0.]);rows,*_=joint_rows(c,p,s)
        for kind,row_index,limits,padding in (('q',0,c.q_upper,QPAD),('dq',1,c.velocity_limit[3:],VPAD)):
            C,lo,hi=rows[row_index]
            lower=c.q_lower if kind=='q' else -c.velocity_limit[3:]
            for j in range(2):
                column=3+j
                for sign in (-1,1):
                    old_limit=limits[j] if sign>0 else lower[j]
                    physical=old_limit-sign*padding*.5
                    z=np.zeros(5*c.nodes);z[column]=physical/C[j,column]
                    oldlo=lower[j];oldhi=limits[j]
                    value=float(C[j]@z)
                    self.assertGreaterEqual(value,oldlo);self.assertLessEqual(value,oldhi)
                    self.assertTrue(value<lo[j] or value>hi[j],(kind,j,sign))
                    # Solve the actual 1D convex QP: the old optimum was legal;
                    # new row moves it onto the original candidate-safe limit.
                    answer,_=qp_solution(float(lo[j]),float(hi[j]),physical)
                    self.assertAlmostEqual(answer,float(hi[j] if sign>0 else lo[j]),places=12)

    def candidate(self,c,s,envelope,refs=None):
        n=c.nodes;time=np.cumsum([.02,*([c.node_dt]*(n-1))]);q=np.zeros((n,2));dq=q.copy()
        actual=dict(time_s=time,q=q,dq=dq,reference_nodes=q.copy() if refs is None else refs,
                    envelope=deepcopy(envelope))
        prediction={'r_current':np.zeros(2)}
        with patch.object(c.actuator_predictor,'candidate_rollout',return_value=actual):
            return c._validate_candidate(0.,s,{'joint_prediction':prediction,'h':np.diff(np.r_[0.,time])},
                                         np.zeros((n,5)),np.zeros((n,12)))

    def test_full_interval_extrema_still_reject_even_with_safe_nodes(self):
        c,_,s,_=setup()
        base=dict(collected=True,q_lower=[0.,0.],q_upper=[0.,0.],dq_lower=[0.,0.],dq_upper=[0.,0.])
        self.candidate(c,s,base)
        for field,j,value in (('q_lower',0,c.q_lower[0]+QPAD*.5),
                              ('q_upper',1,c.q_upper[1]-QPAD*.5),
                              ('dq_lower',1,-c.velocity_limit[4]+VPAD*.5),
                              ('dq_upper',0,c.velocity_limit[3]-VPAD*.5)):
            with self.subTest(field=field):
                envelope=deepcopy(base);envelope[field][j]=value
                with self.assertRaises(MpcSolveError):self.candidate(c,s,envelope)

    def test_candidate_tolerance_boundary_and_unpadded_reference_bounds(self):
        c,_,s,_=setup()
        base=dict(collected=True,q_lower=[0.,0.],q_upper=[0.,0.],dq_lower=[0.,0.],dq_upper=[0.,0.])
        for field,j,limit,padding,sign in (
            ('q_lower',0,c.q_lower[0],QPAD,-1),('q_upper',1,c.q_upper[1],QPAD,1),
            ('dq_lower',1,-c.velocity_limit[4],VPAD,-1),('dq_upper',0,c.velocity_limit[3],VPAD,1)):
            for outside,accepted in ((.5*TOL,True),(1.5*TOL,False)):
                envelope=deepcopy(base);envelope[field][j]=limit-sign*padding+sign*outside
                with self.subTest(field=field,outside=outside):
                    if accepted:self.candidate(c,s,envelope)
                    else:
                        with self.assertRaises(MpcSolveError):self.candidate(c,s,envelope)
        refs=np.zeros((c.nodes,2));refs[:,0]=c.q_upper[0]
        self.candidate(c,s,base,refs)  # references keep the original unpadded q bound
        refs[:,0]=c.q_upper[0]+1.5*TOL
        with self.assertRaises(MpcSolveError):self.candidate(c,s,base,refs)


@unittest.skipUnless(LIB,'Set X650_NATIVE_OPTIMIZATION_LIBRARY for mandatory native parity checks')
class JointSafetyMarginNative(JointSafetyMarginPython):
    def setUp(self):
        self.native=NativeOptimization(LIB,hashlib.sha256(Path(LIB).read_bytes()).hexdigest())

    def test_native_and_python_full_assembled_problem_match(self):
        c,_,s,target=setup();p=fake_prediction(c)
        def prediction(this,*args):
            this._actuator_context={'command_history':[(-.01,np.zeros(2))]}
            return deepcopy(p)
        c._joint_prediction=MethodType(prediction,c)
        expected=c._problem(0.,.02,s,target)
        actual=self.native.assemble(c,0.,.02,s,target)
        for field in ('A','b','H','g','C','lo','hi','offsets','jacobians','first_lower','first_upper'):
            np.testing.assert_allclose(actual[field],expected[field],rtol=1e-10,atol=1e-10,err_msg=field)

    def test_native_qp_moves_old_legal_optimum_inside_same_safety_margins(self):
        for lower,upper,padding in ((-.75,.45,QPAD),(-.35,.85,QPAD),(-.45,.45,VPAD),(-.35,.35,VPAD)):
            for sign in (-1,1):
                old_target=(upper-padding*.5) if sign>0 else (lower+padding*.5)
                expected,p=qp_solution(lower+padding,upper-padding,old_target)
                z,quality,_=self.native.solve(p,100)
                self.assertAlmostEqual(float(z[0]),expected,places=11)
                self.assertLess(quality['primal_violation'],1e-10)

    def test_native_final_gate_preserves_interval_padding_and_reference_tolerance(self):
        c,_,_,_=setup();n=c.nodes;q=np.zeros((n,2));pred=np.zeros((n,12))
        base=dict(collected=True,q_lower=[0.,0.],q_upper=[0.,0.],dq_lower=[0.,0.],dq_upper=[0.,0.])
        for field,j,limit,padding,sign in (
            ('q_lower',0,c.q_lower[0],QPAD,-1),('q_upper',1,c.q_upper[1],QPAD,1),
            ('dq_lower',1,-c.velocity_limit[4],VPAD,-1),('dq_upper',0,c.velocity_limit[3],VPAD,1)):
            for outside,accepted in ((.5*TOL,True),(1.5*TOL,False)):
                envelope=deepcopy(base);envelope[field][j]=limit-sign*padding+sign*outside
                out=self.native.candidate(c,pred,q,q,dict(envelope=envelope,reference_nodes=q.copy()))
                self.assertEqual(out[2:],(accepted,accepted))
        refs=q.copy();refs[:,0]=c.q_upper[0]
        self.assertTrue(self.native.candidate(c,pred,q,q,dict(envelope=base,reference_nodes=refs))[3])
        refs[:,0]+=1.5*TOL
        self.assertFalse(self.native.candidate(c,pred,q,q,dict(envelope=base,reference_nodes=refs))[2])


if __name__=='__main__':unittest.main()
