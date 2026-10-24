"""Physical-prefix metrics must retain truncation and actual sample timing."""
import csv
import io
import math
import unittest
from unittest.mock import patch

from scripts.report_x650_cpp_live import physical_prefix


def samples(times):
    rows = []
    for t, x in times:
        row = dict(sim_s=t, valid_base=1, valid_joint_state=1, valid_tip=1,
                   tip_x=x, tip_y=0, tip_z=0, base_px=0, base_py=0, base_pz=0,
                   base_qw=1, base_qx=0, base_qy=0, base_qz=0,
                   arm_joint_1_q=0, arm_joint_1_qd=0, arm_joint_2_q=0, arm_joint_2_qd=0)
        rows.append(row)
    stream = io.StringIO()
    writer = csv.DictWriter(stream, list(rows[0])); writer.writeheader(); writer.writerows(rows)
    stream.seek(0)
    return stream


class PrefixReportTests(unittest.TestCase):
    def test_nonuniform_samples_are_time_weighted_and_not_full_acceptance(self):
        with patch('pathlib.Path.open', return_value=samples([(10, 0), (11, 2), (13, 2), (14, 99)])):
            out = physical_prefix('unused.csv', 10, 13, [0, 0, 0], [0, 0, 0], 30)
        self.assertAlmostEqual(out['endpoint_rmse_m'], math.sqrt(10/3))
        self.assertEqual(out['endpoint_peak_m'], 2)
        self.assertEqual(out['window_sim_s'], [10, 13])
        self.assertEqual(out['sample_count'], 3)
        self.assertFalse(out['covers_requested_duration'])

    def test_frozen_target_is_used_without_alignment_or_offset_fit(self):
        with patch('pathlib.Path.open', return_value=samples([(2, 3), (2.004, 3)])):
            out = physical_prefix('unused.csv', 2, 2.004, [1, 0, 0], [0, 0, 0], 30)
        self.assertEqual(out['endpoint_rmse_m'], 2)
        self.assertEqual(out['target_enu_m'], [1, 0, 0])
        self.assertTrue(out['no_phase_alignment'])

    def test_repeated_physical_time_is_rejected(self):
        with patch('pathlib.Path.open', return_value=samples([(2, 0), (2, 1)])):
            with self.assertRaisesRegex(ValueError, 'regressed/repeated'):
                physical_prefix('unused.csv', 2, 3, [0, 0, 0], [0, 0, 0], 30)


if __name__ == '__main__':
    unittest.main()
