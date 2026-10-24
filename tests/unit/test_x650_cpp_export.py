"""The offline math exporter must not modify production timing or inputs."""
import sys
import unittest
from pathlib import Path
import numpy as np

ROOT=Path(__file__).resolve().parents[2]
sys.path[:0]=[str(ROOT/'scripts'),str(ROOT/'control_packages/x650_wholebody_v1/tests')]
from export_x650_cpp_inputs import configuration,memory,raw_input,offline_math_reference
from test_ts20_mpc import setup
from x650_wholebody.ts20_mpc import RELINEARIZATION_POLICY


class CppRawExportTests(unittest.TestCase):
    def test_reference_deadline_restored_even_after_failure(self):
        before=dict(RELINEARIZATION_POLICY)
        with self.assertRaisesRegex(RuntimeError,'oracle failure'):
            with offline_math_reference():
                self.assertEqual(RELINEARIZATION_POLICY['total_deadline_ms'],3_600_000.)
                raise RuntimeError('oracle failure')
        self.assertEqual(RELINEARIZATION_POLICY,before)

    def test_unexpected_reference_policy_is_rejected_without_mutation(self):
        before=dict(RELINEARIZATION_POLICY)
        try:
            RELINEARIZATION_POLICY['total_deadline_ms']=51.
            with self.assertRaises(ValueError):
                with offline_math_reference():self.fail('must not enter')
            self.assertEqual(RELINEARIZATION_POLICY['total_deadline_ms'],51.)
        finally:
            RELINEARIZATION_POLICY.clear();RELINEARIZATION_POLICY.update(before)

    def test_raw_export_owns_inputs_and_does_not_supply_qp_matrices(self):
        c,p,s,t=setup();value=raw_input(c,s,t,0.,.02)
        self.assertNotIn('H',value);self.assertNotIn('jacobians',value)
        self.assertEqual(len(value['preview']),c.nodes)
        self.assertEqual(value['context']['packets'][0]['send_time'],-.01)
        original=s.position.copy();value['state']['position'][0]=999.
        np.testing.assert_array_equal(s.position,original)
        self.assertEqual(c.step_count,0)

    def test_warm_replay_preserves_plan_without_claiming_a_prior_step(self):
        c,p,s,t=setup();c._plan=np.zeros((c.nodes,5));c._plan_t=-.02
        c._plan_h=np.r_[.02,np.full(c.nodes-1,c.node_dt)]
        value=memory(c)
        self.assertFalse(value['has_last_t']);self.assertEqual(value['step_count'],0)
        self.assertEqual(len(value['plan']),c.nodes*5)
        value['plan'][0]=12.;self.assertEqual(c._plan[0,0],0.)
        config=configuration(c)
        self.assertEqual(config['motion'][4],p.delay_s)
        np.testing.assert_array_equal(config['kp'],p.kp)


if __name__=='__main__':unittest.main()
