"""A fixed logging patch must preserve all unrelated default topics."""
import importlib.util
from pathlib import Path
import unittest

ROOT=Path(__file__).resolve().parents[2]
spec=importlib.util.spec_from_file_location('upper_logging',ROOT/'scripts/configure_px4_upper_logging.py')
module=importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class LoggingTests(unittest.TestCase):
    def fixture(self):
        return '\n'.join('\tadd_topic("'+key+'", '+str(value)+');' for key,value in module.DEFAULT_INTERVAL_MS.items())+'\n\tadd_topic("vehicle_status");\n'

    def test_changes_only_four_intervals_and_is_idempotent(self):
        original=self.fixture()
        result=module.reviewed_text(original)
        self.assertIn('add_topic("vehicle_status");',result)
        self.assertEqual(result.count(', 0);'),4)
        self.assertEqual(module.reviewed_text(result),result)

    def test_unknown_interval_is_not_silently_changed(self):
        with self.assertRaises(ValueError):
            module.reviewed_text(self.fixture().replace('200','199'))

    def test_missing_or_duplicate_topic_rejected(self):
        for source in (self.fixture().replace('trajectory_setpoint','another_topic'),
                       self.fixture()+'add_topic("trajectory_setpoint", 200);\n'):
            with self.assertRaises(ValueError):
                module.reviewed_text(source)


if __name__=='__main__': unittest.main()
