"""Clock/cutoff regressions: a partial failed flight must not use landing data."""
import csv
from pathlib import Path
import struct
import sys
import tempfile
import unittest

import numpy as np

ROOT=Path(__file__).resolve().parents[2]
sys.path[:0]=[str(ROOT/'src'),str(ROOT/'scripts'),str(ROOT/'control_packages/x650_wholebody_v1')]
import plot_x650_recovery_disturbance as plot


class RecoveryPlotTest(unittest.TestCase):
    def packet(self,order='<'):
        raw=bytearray(7628);raw[:4]=b'\x00\x01\x00\x00' if order=='<' else b'\x00\x00\x00\x00'
        struct.pack_into(order+'H',raw,4,1)
        struct.pack_into(order+'QQQ',raw,12,123,4,379)
        raw[37:69]=bytes(range(32))
        struct.pack_into(order+'Qddd',raw,76,100000000,53.42,7.544,.02)
        return raw

    def test_original_cdr_prefix_alignment_and_endian(self):
        # Encapsulation is outside the alignment base: offset mistakes would
        # produce plausible but wrong physical cutoffs and include landing.
        for order in ('<','>'):
            got=plot.snapshot_identity(self.packet(order))
            self.assertEqual((got['session_id'],got['epoch'],got['sequence']),(123,4,379))
            self.assertEqual(got['configuration_sha256'],list(range(32)))
            self.assertAlmostEqual(got['control_time_s'],53.42)
            self.assertAlmostEqual(got['task_time_s'],7.544)

    def test_unknown_cdr_or_truncated_snapshot_rejected(self):
        for raw in (self.packet()[:-1],b'\x00\x07\x00\x00'+self.packet()[4:]):
            with self.assertRaises(ValueError):plot.snapshot_identity(raw)

    def test_failed_snapshot_wins_over_later_landing_finish(self):
        calls=[dict(sequence=378,task_time_s=7.524,error=''),
               dict(sequence=379,task_time_s=7.544,error='QP infeasible')]
        cutoff,source=plot.controlled_cutoff({'complete':False},
            {'complete':False,'task_end_sim_s':90.}, {'complete':False,'passed':False},calls,24.)
        self.assertEqual(cutoff,7.544)
        self.assertIn('rejected',source)

    def test_clean_finish_selects_declared_window(self):
        cutoff,_=plot.controlled_cutoff({'complete':True},{'complete':True},
            {'complete':True,'passed':True},[dict(task_time_s=23.98,error='')],24.)
        self.assertEqual(cutoff,24.)

    def test_incomplete_without_core_rejection_uses_last_actual_snapshot(self):
        cutoff,_=plot.controlled_cutoff({'complete':False},{'complete':False},
            {'complete':False,'passed':False},[dict(task_time_s=3.24,error='')],24.)
        self.assertEqual(cutoff,3.24)

    def test_future_landing_rows_do_not_enter_native_prefix(self):
        fields=['sim_s','valid_base','valid_joint_state','valid_tip','tip_x','tip_y','tip_z',
                'arm_joint_1_q','arm_joint_2_q','arm_joint_1_qd','arm_joint_2_qd']
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/'native.csv'
            with path.open('w',newline='') as stream:
                writer=csv.DictWriter(stream,fieldnames=fields);writer.writeheader()
                for elapsed in (0.,.004,.008,.012):
                    row={name:0 for name in fields}
                    row.update(sim_s=10.+elapsed,valid_base=1,valid_joint_state=1,valid_tip=1,
                               tip_x=1000. if elapsed>.008 else .001)
                    writer.writerow(row)
            t,values=plot.native_values(path,10.,.008)
            self.assertEqual(len(t),3)
            self.assertLess(values[:,1].max(),.01)

    def test_irregular_samples_use_time_weighted_rms(self):
        t=np.array([0.,.001,1.]);error=np.array([[0.,0.,0.],[1.,0.,0.],[1.,0.,0.]])
        norm=np.linalg.norm(error,axis=1)
        got=plot.metric(t,norm,error)
        self.assertAlmostEqual(got['position_rmse_m'],np.sqrt(.9995))
        self.assertNotAlmostEqual(got['position_rmse_m'],np.sqrt(np.mean(norm**2)))


if __name__=='__main__':unittest.main()
