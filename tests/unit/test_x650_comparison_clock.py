"""Exercise reception gaps that a receipt-age sample alone would miss."""
import importlib.util
from pathlib import Path
import unittest


MODULE = Path(__file__).resolve().parents[2] / 'simulations/x650_control_comparison_v1/clock_observation.py'
spec = importlib.util.spec_from_file_location('comparison_clock', MODULE)
clock_observation = importlib.util.module_from_spec(spec)
spec.loader.exec_module(clock_observation)


class ClockTimingTests(unittest.TestCase):
    def test_repeated_stamps_and_catch_up_do_not_hide_a_stall(self):
        clock = clock_observation.ClockObservation()
        self.assertIsNone(clock.snapshot(100.)['clock_no_progress_max_wall_s'])
        clock.receive(5., 100.)
        clock.receive(5., 102.)
        self.assertEqual(clock.snapshot(102.), {
            'clock_receipt_age_s': 0., 'clock_no_progress_max_wall_s': 2.})
        clock.reset_window()
        clock.receive(5.2, 103.)
        clock.receive(5.3, 103.5)
        self.assertEqual(clock.snapshot(103.5)['clock_no_progress_max_wall_s'], 3.)
        clock.reset_window()
        self.assertEqual(clock.snapshot(103.75)['clock_no_progress_max_wall_s'], .25)
