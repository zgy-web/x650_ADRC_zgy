"""Actual live hook/helper lifecycle, with no ROS runtime or flight endpoints."""
import ast
from pathlib import Path
import sys
import time
from types import ModuleType, SimpleNamespace as NS
import unittest
from unittest.mock import Mock, patch

from simulations.x650_isolated_live_v1.supervision import FaultSupervisor
from simulations.x650_isolated_live_v1.source_activity import SourceActivityTrace, Phase

ROOT=Path(__file__).resolve().parents[2]


def source_class(path,name,namespace):
    parsed=ast.parse((ROOT/path).read_text(encoding='utf-8'))
    definition=next(n for n in parsed.body if isinstance(n,ast.ClassDef) and n.name==name)
    exec(compile(ast.Module(body=[definition],type_ignores=[]),str(path),'exec'),namespace)
    return namespace[name]


class FakeNode:
    def __init__(self,name,context):
        self.name,self.context=name,context
        self.subs=[]
        self.destroy_calls=0

    def create_subscription(self,typ,topic,callback,qos):
        subscription=NS(typ=typ,topic=topic,callback=callback,qos=qos)
        self.subs.append(subscription)
        return subscription

    def destroy_node(self):self.destroy_calls+=1


class FakeExecutor:
    def __init__(self,context):
        self.context=context;self.nodes=[];self.calls=[];self.closed=[]

    def add_node(self,node):self.nodes.append(node)
    def remove_node(self,node):self.nodes.remove(node)
    def spin_once(self,timeout_sec):self.calls.append(timeout_sec)
    def shutdown(self,timeout_sec):self.closed.append(timeout_sec)


def fake_ros_modules():
    modules={name:ModuleType(name) for name in ['rclpy','rclpy.node','rclpy.executors','rclpy.qos']}
    modules['rclpy.node'].Node=FakeNode
    modules['rclpy.executors'].SingleThreadedExecutor=FakeExecutor
    modules['rclpy.qos'].QoSProfile=lambda **kw:NS(**kw)
    modules['rclpy.qos'].ReliabilityPolicy=NS(BEST_EFFORT='best_effort')
    return modules


class SupervisorLifecycleTests(unittest.TestCase):
    def helper(self):
        context=object()
        with patch.dict(sys.modules,fake_ros_modules()):
            helper=FaultSupervisor(context,NS(DispatchTick=object,GateEvent=object),7,2)
        return helper,context

    def test_real_helper_context_depth_identity_latch_and_idempotent_close(self):
        helper,context=self.helper()
        self.assertIs(helper.node.context,context)
        self.assertIs(helper.executor.context,context)
        self.assertEqual([s.qos.depth for s in helper.subs],[1,1])
        dispatch=NS(session_id=8,epoch=2,sequence=1,status=3,reason=1,actual_monotonic_ns=10)
        helper.subs[0].callback(dispatch)
        self.assertIsNone(helper.poll())
        self.assertEqual(helper.latch.counts['ignored_identity'],1)
        dispatch.session_id=7
        helper.subs[0].callback(dispatch)
        self.assertIn('reason 1',helper.poll())
        first=dict(helper.latch.first)
        dispatch.status=4
        helper.subs[0].callback(dispatch)
        self.assertEqual(helper.latch.first,first)
        self.assertEqual(helper.executor.calls,[0.,0.,0.,0.])
        helper.close();helper.close()
        self.assertEqual(helper.node.destroy_calls,1)
        self.assertEqual(helper.executor.closed,[0.])
        self.assertEqual(helper.executor.nodes,[])
        self.assertIn('reason 1',helper.poll())
        self.assertEqual(helper.executor.calls,[0.,0.,0.,0.])

    def live(self):
        namespace={'Ts20Extension':object,'time':time,'write':Mock()}
        cls=source_class(Path('simulations/x650_isolated_live_v1/run_case.py'),'LiveExtension',namespace)
        extension=cls()
        extension.active=True;extension.failure=None;extension.supervisor=NS(poll=Mock(return_value=None))
        extension.pump_times=[]
        extension.loop=NS(recovery=False,ctx=NS(node=NS(spin_once=Mock())),monitor_now=lambda:1.,pump=Mock())
        extension._release=Mock()
        extension.handle_active_pump=Mock()
        return extension

    def test_active_uses_actual_vehicle_parameter_and_stored_original_binding(self):
        extension=self.live()
        vehicle_cls=source_class(Path('src/amsim/api/vehicle.py'),'VehicleAPI',{'ApiNode':object,'time':time})
        vehicle=vehicle_cls.__new__(vehicle_cls)
        vehicle.node=extension.loop.ctx.node
        vehicle.link=NS(recv_match=lambda **kwargs:None)
        vehicle.target=None;vehicle.heartbeat_time=0.
        # Same two captures used by ControlLoop then LoopTimingObserver.
        original=vehicle._pump
        extension.loop.original_pump=original
        extension.loop_timing=NS(original={'original_pump':extension.loop.original_pump})
        vehicle._pump=Mock(side_effect=AssertionError('must not call replaced public pump recursively'))
        extension.pump_actual()
        self.assertTrue(vehicle.node.spin_once.call_count)
        self.assertTrue(all(call.kwargs=={'timeout_sec':0.} for call in vehicle.node.spin_once.call_args_list))
        extension.supervisor.poll.assert_called_once()
        extension.handle_active_pump.assert_called_once()
        vehicle.node.spin_once.reset_mock()
        original()
        vehicle.node.spin_once.assert_called_once_with(timeout_sec=.02)

    def test_fault_checked_before_main_pump(self):
        extension=self.live()
        extension.supervisor.poll.return_value='C++ output latched fault reason 1'
        original=Mock(side_effect=AssertionError('main callback must not run after fault'))
        extension.loop_timing=NS(original={'original_pump':original})
        with self.assertRaisesRegex(RuntimeError,'reason 1'):extension.pump_actual()
        original.assert_not_called()
        extension.handle_active_pump.assert_not_called()
        self.assertEqual(extension.failure,'C++ output latched fault reason 1')

    def test_first_supervision_exception_keeps_entry_failure_and_finally_trace(self):
        extension=self.live()
        trace=extension.source_activity=SourceActivityTrace(capacity=4,gc_capacity=8)
        extension.activity_phase=Phase
        trace.start_gc()
        extension.supervisor.poll.side_effect=RuntimeError('source fault')
        extension.loop_timing=NS(original={'original_pump':Mock()})
        try:
            with self.assertRaisesRegex(RuntimeError,'source fault'):extension.pump_actual()
            self.assertEqual(len(extension.pump_times),0)
            self.assertTrue(trace.frozen)
            self.assertEqual(extension.activity_token,0)
        finally:trace.close()
        report=trace.report()
        row=dict(zip(report['columns'],report['rows'][0]))
        self.assertEqual(row['failed'],1)
        self.assertEqual(row['failure_phase'],Phase.SUPERVISION)
        self.assertGreaterEqual(row['end_ns'],row['failure_ns'])
        self.assertGreaterEqual(row['failure_ns'],row['supervision_begin_ns'])
        self.assertEqual(row['mavlink_ros_begin_ns'],0)
        self.assertEqual(trace.begin(),0)
        extension.loop_timing.original['original_pump'].assert_not_called()

    def test_success_trace_extends_past_legacy_pump_append(self):
        extension=self.live()
        trace=extension.source_activity=SourceActivityTrace(capacity=4,gc_capacity=8)
        extension.activity_phase=Phase
        extension.loop_timing=NS(original={'original_pump':Mock()})
        extension.pump_actual()
        trace.close()
        report=trace.report()
        row=dict(zip(report['columns'],report['rows'][0]))
        self.assertEqual(row['failed'],0)
        self.assertEqual(row['completed'],1)
        self.assertGreaterEqual(row['end_ns'],extension.pump_times[-1][2])
        self.assertGreaterEqual(row['handle_done_begin_ns'],row['monitor_begin_ns'])
        self.assertEqual(extension.activity_token,0)

    def test_recovery_releases_writer_before_legacy_and_skips_fault_rethrow(self):
        extension=self.live();extension.loop.recovery=True
        extension.supervisor.poll.side_effect=AssertionError('latched fault must not block recovery')
        order=[]
        extension._release.side_effect=lambda loop:order.append('release')
        extension.loop.pump.side_effect=lambda:order.append('legacy')
        extension.pump_actual()
        self.assertEqual(order,['release','legacy'])
        extension.supervisor.poll.assert_not_called()

    def test_supervision_report_write_failure_occurs_after_all_children_stopped(self):
        extension=self.live()
        extension.source_sender=None
        order=[]
        extension._stop_process=lambda name:order.append('stop:'+name)
        extension.logs=[NS(closed=False,close=lambda:order.append('close-log'))]
        def failed_report():
            order.append('supervision-report')
            raise OSError('synthetic report disk failure')
        extension._close_supervision=failed_report
        with self.assertRaisesRegex(OSError,'report disk failure'):extension.cleanup()
        self.assertEqual(order,['stop:'+name for name in
            ('dispatcher','consumption','ingress','worker','recorder','source_recorder')]
            +['close-log','supervision-report'])


if __name__=='__main__':unittest.main()
