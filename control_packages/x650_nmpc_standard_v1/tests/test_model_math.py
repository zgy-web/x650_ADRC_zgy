"""Lightweight numerical-input and time-schedule contracts (no optimizer)."""
import copy
import json
import os
from pathlib import Path
import unittest

import numpy as np

from x650_nmpc_standard.config import ModelConfig, SurrogateConfig
from x650_nmpc_standard.model import build_case, _schedule, parameter_layout


class ModelMath(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        root=Path(os.environ.get('X650_STANDARD_FIXTURES',Path(__file__).resolve().parents[3]/'tmp/standard_nmpc_20261027'))
        cls.documents=[json.loads(p.read_text()) for p in sorted(root.glob('*selected.json'))]
        if len(cls.documents)!=4:raise RuntimeError('Four original selected-fixture files required')
        cls.config=ModelConfig(cls.documents[0]['config'])

    def test_all_real_cases_restore_and_preserve_input(self):
        count=0;moving=0;pending=0;first_steps=set()
        for doc in self.documents:
            for wrapper in doc['cases']:
                original=copy.deepcopy(wrapper);case=build_case(doc['config'],wrapper)
                self.assertEqual(wrapper,original);self.assertEqual(case.x0.shape,(23,))
                self.assertEqual(case.parameters.shape,(16,161));self.assertEqual(case.initial_plan.shape,(15,5))
                self.assertEqual(case.horizon[0],wrapper['case']['input']['dt'])
                self.assertTrue(np.isfinite(case.parameters).all());count+=1
                first_steps.add(round(case.horizon[0],6));moving+=max(np.abs(wrapper['case']['input']['state']['dq']))>.05
                pending+=case.metadata['initial_pending_packet_count']
        self.assertEqual(count,16);self.assertGreater(moving,0);self.assertGreater(pending,0)
        self.assertIn(.052,first_steps)

    def test_fixed_schedule_matches_global_event_times(self):
        c=self.config;slots=SurrogateConfig().event_slots
        for phase in [0.,.0001,.001,.001653620907423576,.003,.0049]:
            for first_h in [.02,.031,.04,.052,.1]:
                first_future=phase;start=0.;pending=phase-c.bus_period+c.motion[4]
                pending=pending if pending>1e-10 else None
                for h in [first_h,.04,.04]:
                    schedule,next_pending=_schedule(start,h,first_future,c,pending,slots)
                    elapsed=start;actual=[]
                    for duration,isnew,ispending,rel,mask in zip(*[schedule[k] for k in ('durations','apply_new','apply_pending','send_relative','sample_mask')]):
                        elapsed+=duration
                        if mask:self.assertLessEqual(duration,c.bus_period+1e-10)
                        if isnew:actual.append((elapsed,1,rel))
                        if ispending:actual.append((elapsed,0,0.))
                    self.assertAlmostEqual(elapsed,start+h,places=12)
                    all_sends=[first_future+k*c.bus_period for k in range(100)
                               if start-1e-10<=first_future+k*c.bus_period<start+h-1e-10]
                    expected=([(pending,0,0.)] if pending is not None else [])+[
                        (send+c.motion[4],1,send-start) for send in all_sends if send+c.motion[4]<=start+h+1e-10]
                    expected.sort(key=lambda v:v[0])
                    np.testing.assert_allclose(actual,expected,atol=2e-12,rtol=0)
                    expected_pending=[send+c.motion[4] for send in all_sends if send+c.motion[4]>start+h+1e-10]
                    self.assertLessEqual(len(expected_pending),1)
                    if expected_pending:self.assertAlmostEqual(next_pending,expected_pending[0],places=12)
                    else:self.assertIsNone(next_pending)
                    start+=h;pending=next_pending

    def test_actual_history_rejection_not_synthetic_replacement(self):
        source=self.documents[0]['cases'][0]['case']
        mutations=[lambda c:c['memory'].update(history_primed=False),
                   lambda c:c['input']['context'].update(packets=[]),
                   lambda c:c['input']['context']['packets'].reverse(),
                   lambda c:c['input']['context']['packets'][-1]['kp'].__setitem__(0,7.),
                   lambda c:c['input'].update(dt=.101),
                   lambda c:c['input']['state'].update(joint_stamp=c['input']['state']['control_time']-.03)]
        for mutation in mutations:
            case=copy.deepcopy(source);mutation(case)
            with self.assertRaises(ValueError):build_case(self.config,case)

    def test_parameter_layout_has_no_aliases(self):
        layout=parameter_layout();used=[]
        for key,slice_ in layout.items():
            if key!='size':used.extend(range(slice_.start,slice_.stop))
        self.assertEqual(used,list(range(161)))


if __name__=='__main__':unittest.main()
