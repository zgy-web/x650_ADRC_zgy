import importlib.util
from pathlib import Path
import sys
import unittest

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'scripts'))
from prepare_x650_multirate_recording import select_rows


class MultirateInputTest(unittest.TestCase):
    def test_uses_measured_intervals_without_rounding(self):
        rows=[{'snapshot':{'task_time_s':t}} for t in [.021,.042,.061,.083,.103,.124]]
        got=select_rows(rows,count=3)
        self.assertEqual([x['snapshot']['task_time_s'] for x,_ in got],[.042,.083,.124])
        self.assertEqual([dt for _,dt in got],[.042,.083-.042,.124-.083])

    def test_does_not_hide_missing_source_gap(self):
        with self.assertRaises(ValueError):
            select_rows([{'snapshot':{'task_time_s':.04}},{'snapshot':{'task_time_s':.20}}],count=2)

    def test_requires_complete_declared_prefix(self):
        with self.assertRaises(ValueError):
            select_rows([{'snapshot':{'task_time_s':.04}}],count=2)


if __name__=='__main__':unittest.main()
