"""Ground-only construction and transport setup; no ROS or actuator calls."""
import ast
from copy import deepcopy
from pathlib import Path
from types import SimpleNamespace as NS
import time
import unittest
from unittest.mock import Mock
import numpy as np

ROOT=Path(__file__).resolve().parents[2]
tree=ast.parse((ROOT/'simulations/x650_isolated_live_v1/run_case.py').read_text(encoding='utf-8'))
definition=next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name=='LiveExtension')


class FakeBase:
    def build_allocator(self,cfg,model,pid,trajectory,loop):
        self.constructed.append((deepcopy(cfg),model,dict(pid),trajectory,loop))
        return self.allocator_fixture


class ConstantTarget:
    def __init__(self,center):self.center=center


namespace={'Ts20Extension':FakeBase,'deepcopy':deepcopy,'time':time,'np':np,
           'shared':NS(ConstantTarget=ConstantTarget),'write':Mock()}
exec(compile(ast.Module(body=[definition],type_ignores=[]),'<live-ground-adapter>','exec'),namespace)
LiveExtension=namespace['LiveExtension']


class GroundPreparation(unittest.TestCase):
    def fixture(self):
        ext=LiveExtension()
        ext.active=ext.started=False;ext.failure=None;ext.session=12;ext.task_sha='a'*64
        ext.latest={'dispatch':NS(status=0,output_valid=False)};ext.message_counts={'dispatch':3}
        ext.out=Path('/unused');ext.constructed=[]
        ext.allocator_fixture=NS(step_count=0,step=Mock(side_effect=AssertionError('no ground solve')),
                                 reset=Mock(side_effect=AssertionError('no synthetic reset')))
        ext.runtime=NS(states=NS(path=Path('/actual/ts20_state.csv')))
        node=NS(data={'status':NS(arming_state=1),'land':NS(landed=True)})
        ctx=NS(node=node,vehicle=NS(link=NS(clients=[('127.0.0.1',18570)],port=NS(fileno=lambda:7)),target=(1,1)))
        cfg={'stage':'normal','mpc_config':{'nodes':15},'numerical_backend':{'library_sha256':'b'*64},
             'trajectory_parameters':{'period':12}}
        loop=NS(ctx=ctx,cfg=cfg,model=object(),pid={'MPC_XY_P':1.4,'MPC_Z_P':1.1},
                strict=False,offboard=False,recovery=False,task_start=None,pump=Mock())
        ext.loop=loop;ext._launch=Mock();ext._ros=lambda package,executable,params:(package,executable,params)
        ext._transport_ready=Mock(side_effect=[False,True]);ext._alive=Mock()
        return ext,ctx,loop

    def test_prepares_before_arming_without_solve_or_synthetic_reset(self):
        ext,ctx,loop=self.fixture();meta={}
        ext._prepare_ground_transport(ctx,loop,meta)
        self.assertEqual(len(ext.constructed),1)
        self.assertIs(ext.constructed[0][1],loop.model)
        self.assertIsInstance(ext.constructed[0][3],ConstantTarget)
        ext.allocator_fixture.step.assert_not_called();ext.allocator_fixture.reset.assert_not_called()
        self.assertEqual([c.args[0] for c in ext._launch.call_args_list],['ingress','dispatcher','consumption'])
        ingress=ext._launch.call_args_list[0].args[1][2]
        self.assertEqual(ingress['task_origin_s'],0.)
        self.assertTrue(ingress['simulation_live'])
        consumption=ext._launch.call_args_list[2].args[1][2]
        self.assertEqual(consumption['configuration_sha256'],ext.task_sha)
        self.assertEqual(consumption['expected_session_id'],ext.session)
        record=meta['isolated_preflight_transport']
        for field in ('ground_landed','ground_disarmed','loop_strict_false','before_live_start',
                      'transport_ready','allocator_constructed_no_step'):
            self.assertIs(record[field],True)
        self.assertEqual(record['normal_output_count'],0)
        self.assertEqual(record['allocator_step_count'],0)
        self.assertLessEqual(record['start_monotonic_ns'],record['end_monotonic_ns'])
        loop.pump.assert_called_once()

    def test_ground_guards_precede_every_constructor_and_launch(self):
        for bad in ('armed','airborne','strict','started','offboard','recovery'):
            ext,ctx,loop=self.fixture()
            if bad=='armed':ctx.node.data['status'].arming_state=2
            elif bad=='airborne':ctx.node.data['land'].landed=False
            elif bad=='started':loop.task_start=1.
            else:setattr(loop,bad,True)
            with self.subTest(bad=bad),self.assertRaises(RuntimeError):
                ext._prepare_ground_transport(ctx,loop,{})
            self.assertEqual(ext.constructed,[]);ext._launch.assert_not_called()

    def test_ground_state_checked_again_while_waiting(self):
        ext,ctx,loop=self.fixture()
        loop.pump.side_effect=lambda:setattr(ctx.node.data['status'],'arming_state',2)
        with self.assertRaisesRegex(RuntimeError,'disarmed ground'):
            ext._prepare_ground_transport(ctx,loop,{})

    def test_any_cpp_actuator_send_or_non_idle_dispatch_rejects_readiness(self):
        for bad in ('send','fault','output'):
            ext,ctx,loop=self.fixture();ext._transport_ready=lambda:True
            if bad=='send':ext.message_counts['sends']=1
            elif bad=='fault':ext.latest['dispatch'].status=3
            else:ext.latest['dispatch'].output_valid=True
            with self.subTest(bad=bad),self.assertRaises(RuntimeError):
                ext._prepare_ground_transport(ctx,loop,{})

    def test_airborne_build_only_claims_cached_controller_with_actual_trajectory(self):
        ext,ctx,loop=self.fixture();ext._prepare_ground_transport(ctx,loop,{})
        actual=object();loop.strict=True;ctx.node.data['status'].arming_state=2
        result=ext.build_allocator(loop.cfg,loop.model,loop.pid,actual,loop)
        self.assertIs(result,ext.allocator_fixture);self.assertIs(result.trajectory,actual)
        self.assertEqual(len(ext.constructed),1)
        result.step.assert_not_called();result.reset.assert_not_called()
        with self.assertRaises(RuntimeError):ext.build_allocator(loop.cfg,loop.model,loop.pid,actual,loop)

    def test_changed_identity_configuration_or_state_cannot_reuse_cache(self):
        for bad in ('model','pid','config','loop','step'):
            ext,ctx,loop=self.fixture();ext._prepare_ground_transport(ctx,loop,{})
            cfg,model,pid,chosen=loop.cfg,loop.model,loop.pid,loop
            if bad=='model':model=object()
            elif bad=='pid':pid={**pid,'MPC_XY_P':2.}
            elif bad=='config':cfg['mpc_config']['nodes']=16
            elif bad=='loop':chosen=object()
            else:ext.allocator_fixture.step_count=1
            with self.subTest(bad=bad),self.assertRaises(RuntimeError):
                ext.build_allocator(cfg,model,pid,object(),chosen)

    def test_activation_contains_no_construct_launch_or_graph_discovery(self):
        activate=next(n for n in definition.body if isinstance(n,ast.FunctionDef) and n.name=='activate')
        called={n.func.attr for n in ast.walk(activate) if isinstance(n,ast.Call) and isinstance(n.func,ast.Attribute)}
        self.assertTrue({'_launch','_ros','build_allocator','_transport_ready','get_subscription_count'}.isdisjoint(called))


if __name__=='__main__':unittest.main()
