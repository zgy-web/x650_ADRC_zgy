"""Preflight setup-cycle cleanup is observable and never changes runtime GC policy."""
import gc
import json
from pathlib import Path
import sys
import unittest
from unittest.mock import Mock
import uuid
import weakref

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from flyarm_mpc_reference.isolated_node import collect_preflight_garbage


class PreflightGcTests(unittest.TestCase):
    def output(self):
        path=Path(__file__).resolve().parent/('_preflight_gc_'+uuid.uuid4().hex+'.json')
        self.addCleanup(lambda: path.unlink(missing_ok=True))
        return path

    def test_real_setup_cycle_is_collected_with_callbacks_and_policy_preserved(self):
        class SetupOnly:
            pass
        def construction():
            item=SetupOnly();item.loop=item
            return weakref.ref(item)
        # No temporary strong reference to the setup-only cycle survives the
        # simulated constructor return; the helper must not retain its heap.
        pending=construction()
        policy=(gc.isenabled(),gc.get_threshold(),gc.get_freeze_count())
        callbacks=tuple(gc.callbacks)
        path=self.output()
        report=collect_preflight_garbage(path)
        self.assertIsNone(pending())
        self.assertEqual(policy,(gc.isenabled(),gc.get_threshold(),gc.get_freeze_count()))
        self.assertEqual(callbacks,tuple(gc.callbacks))
        self.assertTrue(report['gc_policy_unchanged'])
        self.assertEqual(report['generation'],2)
        self.assertEqual(report,json.loads(path.read_text()))
        self.assertGreaterEqual(report['duration_ns'],0)
        self.assertGreaterEqual(report['after']['stats'][2]['collections'],
                                report['before']['stats'][2]['collections']+1)

    def test_explicit_call_records_duration_counts_without_disabling_or_freezing(self):
        collector=Mock()
        collector.isenabled.return_value=True
        collector.get_count.side_effect=[(5,6,7),(1,0,0)]
        collector.get_threshold.return_value=(700,10,10)
        collector.get_stats.return_value=[{'collections':1,'collected':2,'uncollectable':0}]*3
        collector.get_freeze_count.return_value=0
        collector.garbage=[];collector.collect.return_value=17
        report=collect_preflight_garbage(self.output(),collector=collector,
                                        clock_ns=Mock(side_effect=[100,5100]))
        collector.collect.assert_called_once_with(2)
        for method in ('disable','enable','freeze','unfreeze','set_threshold'):
            getattr(collector,method).assert_not_called()
        self.assertEqual(report['duration_ns'],5000)
        self.assertEqual(report['collected'],17)
        self.assertEqual(report['before']['counts'],[5,6,7])
        self.assertEqual(report['after']['counts'],[1,0,0])


if __name__=='__main__':unittest.main()
