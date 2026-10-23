"""Synthetic integration: estimated control, independent truth, direct PVA."""
import csv
import json
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace as NS
import unittest
from unittest.mock import patch
import numpy as np

ROOT = Path(__file__).resolve().parents[3]
for folder in (ROOT/'tests/unit', ROOT/'simulations/x650_wholebody_mpc_v1',
               ROOT/'control_packages/x650_wholebody_v1'):
    sys.path.insert(0, str(folder))
import run_case as runner
from test_x650_estimated_feedback import fixture
from estimated_feedback import Px4EstimatedFeedback
from x650_wholebody.api import ControlState


class UpperLoopTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.path = Path(self.temp.name)
        self.node, reference = fixture()
        self.node.received.update(land=99., status=100., truth=100.)
        self.node.data.update(land=NS(timestamp=1, landed=False),
                              status=NS(timestamp=1, failsafe=False, nav_state=14))
        self.node.spin_once = lambda **kwargs: None
        self.sent = []
        self.ctx = NS(node=self.node, vehicle=NS(_pump=lambda: None, target=(1,1),
            link=NS(mav=NS(set_position_target_local_ned_send=lambda *v: self.sent.append(v)))),
            arm=NS(set_velocity=lambda *args: None))
        self.cfg = dict(research_profile={'name':'precision_ev_upper_v2'}, stage='response_probe',
                        duration_s=30., control_period_s=.02)
        self.truth = ControlState([1.8,2.7,1.4], np.eye(3), [.2,.1,0.], [0.,0.,0.], [.3,.4], [.1,.2], 10.)
        self.capture = NS(now=lambda:10., phase='measure', check=lambda:None)
        self.loop = runner.ControlLoop(self.ctx,self.cfg,NS(),self.capture,{'MPC_XY_P':.95,'MPC_Z_P':1.},self.path)
        self.addCleanup(self.loop.close)
        self.loop.fixed_reference = NS(document=reference)
        self.loop.feedback_adapter = Px4EstimatedFeedback(reference,self.node)
        self.loop.anchor = np.array([2.,3.,1.5])
        self.loop.yaw_ned=0.
        self.loop.task_start=7.5
        self.loop.send_reference=True
        self.clockpatch = patch.object(runner.time,'monotonic',return_value=100.)
        self.clockpatch.start(); self.addCleanup(self.clockpatch.stop)
        self.truthpatch = patch.object(runner,'state_from_node',return_value=(self.truth,10.))
        self.truthpatch.start(); self.addCleanup(self.truthpatch.stop)

    def test_control_uses_estimate_and_csv_preserves_truth(self):
        with patch.dict(sys.modules,{'pymavlink':NS(mavutil=NS(mavlink=NS(MAV_FRAME_LOCAL_NED=1)))}):
            self.loop.pump()
        self.assertEqual(len(self.sent),1)
        self.loop.control.flush(); self.loop.feedback_stream.flush()
        with (self.path/'control_trace.csv').open() as stream:
            row = next(csv.DictReader(stream))
        self.assertEqual(float(row['base_px']),1.8)
        self.assertEqual(float(row['feedback_px']),2.)
        self.assertEqual(row['fixed_world_position_sent'],'False')
        self.assertEqual(row['pva_sent'],'True')
        sp=self.sent[0]
        np.testing.assert_allclose(sp[5:8],[3.,2.,486.5])
        np.testing.assert_allclose(sp[8:11],[0.,.08,0.],atol=1e-14)
        feedback=json.loads((self.path/'feedback_trace.jsonl').read_text())['feedback']
        self.assertFalse(feedback['truth_feedback'])
        self.assertEqual(feedback['control_state']['position_enu_m'],[2.,3.,1.5])

    def test_stale_estimate_never_falls_back_to_truth(self):
        self.node.received_sim['angular_velocity']=9.8
        with self.assertRaisesRegex(RuntimeError,'angular_velocity feedback is stale'):
            self.loop.read()
        self.assertEqual(self.sent,[])

    def test_clock_disagreement_rejects_control(self):
        self.capture.now=lambda:10.06
        with self.assertRaisesRegex(RuntimeError,'clock progress differ'):
            self.loop.read()

    def test_control_clock_is_estimator_clock_not_recorder_arrival(self):
        self.capture.now=lambda:10.004
        self.assertEqual(self.loop.read()[0],10.)
        self.assertEqual(self.loop.control_now(),10.)

    def test_recovery_explicitly_ends_upper_control(self):
        self.loop.controller=object()
        self.loop.disable_upper_feedback()
        self.assertFalse(self.loop.send_reference)
        self.assertIsNone(self.loop.controller)
        self.assertIsNone(self.loop.feedback_adapter)
        # Truth remains available to safety/joint-home recovery, with no PVA.
        self.assertIs(self.loop.read()[1],self.truth)

    def test_ts20_normal_restore_clears_stale_feedback_and_keeps_recovery_outputs(self):
        self.loop.read()
        self.assertIsNotNone(self.loop.feedback_diagnostics)
        self.assertIsNone(self.truth.acceleration_enu)
        self.loop.extension = NS(
            select_joint_observation=lambda *args: None,
            after_state_read=lambda loop,state,joint_stamp,sim: (state,joint_stamp),
            on_pump=lambda *args: None,
            before_publish=lambda *args: {"active":False,"reason":"recovery"})
        self.loop.recovery=True
        self.loop.task_start=None
        self.capture.phase='restore_folded'
        joint=[]
        self.ctx.arm.set_velocity=lambda *args: joint.append(args)
        with patch.object(self.loop.feedback_adapter,'read',side_effect=AssertionError('Inactive estimator read')), \
                patch.dict(sys.modules,{'pymavlink':NS(mavutil=NS(mavlink=NS(MAV_FRAME_LOCAL_NED=1)))}):
            self.loop.pump()
        self.assertIsNone(self.loop.feedback_diagnostics)
        self.assertEqual(len(joint),2)
        self.assertEqual(len(self.sent),1)  # Existing recovery hold behavior.
        self.loop.control.flush();self.loop.feedback_stream.flush()
        with (self.path/'control_trace.csv').open() as stream:
            row=next(csv.DictReader(stream))
        self.assertEqual(row['phase'],'restore_folded')
        self.assertTrue(np.isnan(float(row['feedback_ax'])))
        self.assertTrue(np.isnan(float(row['feedback_px'])))
        self.assertIsNone(json.loads((self.path/'feedback_trace.jsonl').read_text())['feedback'])

    def test_unbound_upper_loop_cannot_publish(self):
        self.loop.feedback_adapter=None
        with patch.dict(sys.modules,{'pymavlink':NS(mavutil=NS(mavlink=NS(MAV_FRAME_LOCAL_NED=1)))}):
            with self.assertRaisesRegex(RuntimeError,'requires validated PX4'):
                self.loop.pump()
        self.assertEqual(self.sent,[])

    def test_ground_probe_defers_only_final_heading_and_never_enables_control(self):
        self.node.data['status'].arming_state=1
        self.node.data['land'].landed=True
        self.node.data['position'].heading_good_for_control=False
        self.node.data['estimator_flags'].cs_tilt_align=True
        self.node.data['estimator_flags'].cs_yaw_align=True
        result=runner.estimated_ground_probe(self.node,10.)
        self.assertTrue(result['ground_streams_verified'])
        self.assertFalse(result['control_eligible'])
        self.assertFalse(result['heading_good_for_control'])
        with self.assertRaisesRegex(RuntimeError,'heading_good_for_control'):
            self.loop.read()
        self.node.data['estimator_flags'].cs_yaw_align=False
        with self.assertRaisesRegex(RuntimeError,'initial estimator alignment'):
            runner.estimated_ground_probe(self.node,10.)


if __name__=='__main__':
    unittest.main()
