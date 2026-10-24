"""Ground preparation must precede takeoff and preserve actual handoff semantics."""
import ast
from copy import deepcopy
from pathlib import Path
import unittest

ROOT=Path(__file__).resolve().parents[2]
tree=ast.parse((ROOT/'scripts/audit_x650_isolated_live.py').read_text())
function=next(n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name=='check_preflight_transport')


def require(ok,message):
    if not ok:raise ValueError(message)


class GroundAudit(unittest.TestCase):
    def setUp(self):
        self.record={'start_monotonic_ns':1000000000,'end_monotonic_ns':2000000000,
            'ground_landed':True,'ground_disarmed':True,'loop_strict_false':True,
            'before_live_start':True,'allocator_constructed_no_step':True,'transport_ready':True,
            'allocator_step_count':0,'normal_output_count':0,'initial_task_origin_s':0.,
            'allocator_prepare_ns':1000000,'transport_launch_ns':2000000}
        self.start={'issued_monotonic_ns':5000000000}
        self.meta={'isolated_preflight_transport':deepcopy(self.record),
            'takeoff_state_transition':{'before':{'command_boundary_wall_monotonic_s':3.}}}
        ns={'read':lambda p:self.record,'require':require}
        exec(compile(ast.Module(body=[function],type_ignores=[]),'<ground-audit>','exec'),ns)
        self.check=lambda:ns['check_preflight_transport'](Path('live'),self.meta,self.start)

    def test_ground_complete(self):
        self.assertTrue(self.check()['verified'])

    def test_airborne_or_post_takeoff_preparation_rejected(self):
        for key,value in [('ground_landed',False),('ground_disarmed',False),('loop_strict_false',False),
                          ('end_monotonic_ns',3500000000)]:
            with self.subTest(key=key):
                self.setUp();self.record[key]=value;self.meta['isolated_preflight_transport']=deepcopy(self.record)
                with self.assertRaises(ValueError):self.check()

    def test_preflight_control_advancement_rejected(self):
        for key in ['allocator_step_count','normal_output_count','initial_task_origin_s']:
            with self.subTest(key=key):
                self.setUp();self.record[key]=1;self.meta['isolated_preflight_transport']=deepcopy(self.record)
                with self.assertRaises(ValueError):self.check()

    def test_tampered_record_rejected(self):
        self.record['allocator_prepare_ns']+=1
        with self.assertRaises(ValueError):self.check()


if __name__=='__main__':unittest.main()
