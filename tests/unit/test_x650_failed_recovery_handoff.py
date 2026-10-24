"""Failed-flight ownership/mode handoff with deterministic clocks, no endpoints."""
import ast
from pathlib import Path
import subprocess
from types import SimpleNamespace as NS
import unittest
from unittest.mock import Mock

ROOT=Path(__file__).resolve().parents[2]
SIGNALS=NS(SIGINT=2,SIGKILL=9)


def definition(path,name,namespace):
    tree=ast.parse((ROOT/path).read_text(encoding='utf-8'))
    selected=next(item for item in tree.body if getattr(item,'name',None)==name)
    exec(compile(ast.Module(body=[selected],type_ignores=[]),str(path),'exec'),namespace)
    return namespace[name]


class Clock:
    def __init__(self):self.now=100.
    def monotonic(self):return self.now
    def monotonic_ns(self):return int(self.now*1e9)
    def sleep(self,seconds):self.now+=seconds


class FailedRecoveryHandoff(unittest.TestCase):
    def fixture(self):
        clock=Clock();kills=[]
        shared=NS(receipt_time=lambda node,key:node.received[key])
        namespace={'Ts20Extension':object,'time':clock,'shared':shared,'signal':SIGNALS,
                   'subprocess':subprocess,'os':NS(killpg=lambda pid,sig:kills.append((pid,sig)))}
        cls=definition(Path('simulations/x650_isolated_live_v1/run_case.py'),'LiveExtension',namespace)
        ext=cls();ext.started=ext.active=True;ext.session=7;ext.epoch=2;ext.failure='original fault'
        ext.release={};ext.latest={};ext.procs={};ext.meta={}
        node=NS(data={'status':NS(timestamp=10,nav_state=14,arming_state=2,failsafe=False)},
                received={'status':99.},callback_sequences={'status':4},spin_once=Mock(),zero_all=Mock())
        vehicle=NS(target=(1,1),ack={176:(99.,0)},_send_command=Mock(),land=Mock())
        ctx=NS(node=node,vehicle=vehicle)
        loop=NS(ctx=ctx,recovery=False,offboard=True,send_reference=False,strict=False,
                monitor_now=lambda:42.,control_now=lambda:42.,pump=Mock(),
                land_watchdog=NS(observe=Mock()),status_watchdog=NS(observe=Mock()))
        ext.loop=loop;ext.loop_timing=NS(original={'original_pump':Mock()})
        ext.msg=NS(RuntimeStop=lambda:NS());ext.stop_pub=NS(publish=Mock())
        actual_release=ext._release
        def release(*args,**kwargs):
            ext.release={'writer_exited':True,'zero_consumed':True};ext.active=False
            vehicle._pump=loop.pump
        ext._release=Mock(side_effect=release)
        def receive():clock.sleep(.01)
        ext._pump_recovery_receive_only=Mock(side_effect=receive)
        return ext,ctx,loop,clock,kills,actual_release

    def new_status(self,ctx,clock,*,stamp=11,sequence=5,nav=4,failsafe=False):
        ctx.node.data['status']=NS(timestamp=stamp,nav_state=nav,arming_state=2,failsafe=failsafe)
        ctx.node.callback_sequences['status']=sequence;ctx.node.received['status']=clock.monotonic()

    def test_confirmed_hold_requires_new_ack_and_new_source_state_no_outputs(self):
        ext,ctx,loop,clock,kills,_=self.fixture();meta={};order=[]
        release=ext._release.side_effect
        def released(*args,**kwargs):order.append('release');release(*args,**kwargs)
        ext._release.side_effect=released
        def send(command,params):
            order.append('mode');self.assertNotIn(176,ctx.vehicle.ack)
            self.assertTrue(ext.release['writer_exited']);self.assertFalse(loop.send_reference)
        ctx.vehicle._send_command.side_effect=send
        def receive():
            clock.sleep(.01);ctx.vehicle.ack[176]=(clock.monotonic(),0);self.new_status(ctx,clock)
        ext._pump_recovery_receive_only.side_effect=receive
        self.assertTrue(ext.before_failed_fold(ctx,loop,meta))
        ext._release.assert_called_once_with(loop,bounded_failed_recovery=True)
        self.assertEqual(order,['release','mode'])
        ctx.vehicle._send_command.assert_called_once_with(176,[1.,4.,3.,0.,0.,0.,0.])
        self.assertTrue(meta['failed_recovery_handoff']['confirmed'])
        self.assertEqual(meta['failed_recovery_handoff']['decision'],'fold_then_land')
        loop.pump.assert_not_called();ctx.node.zero_all.assert_not_called();ctx.vehicle.land.assert_not_called()

    def test_reject_immediately_returns_direct_land_without_retry(self):
        ext,ctx,loop,clock,_,_=self.fixture();meta={}
        def receive():clock.sleep(.01);ctx.vehicle.ack[176]=(clock.monotonic(),2)
        ext._pump_recovery_receive_only.side_effect=receive
        self.assertFalse(ext.before_failed_fold(ctx,loop,meta))
        self.assertLess(clock.now-100.,.02)
        self.assertEqual(meta['failed_recovery_handoff']['reason'],'mode_command_rejected')
        self.assertEqual(ctx.vehicle._send_command.call_count,1)

    def test_old_ack_cached_status_old_source_or_identity_never_confirm(self):
        for invalid in ('old_ack','no_ack','old_callback','old_timestamp','old_receipt','wrong_nav','failsafe','wrong_target'):
            with self.subTest(invalid=invalid):
                ext,ctx,loop,clock,_,_=self.fixture();meta={}
                def receive():
                    clock.sleep(.01)
                    if invalid!='no_ack':ctx.vehicle.ack[176]=(99. if invalid=='old_ack' else clock.monotonic(),0)
                    self.new_status(ctx,clock,stamp=10 if invalid=='old_timestamp' else 11,
                                    sequence=4 if invalid=='old_callback' else 5,
                                    nav=14 if invalid=='wrong_nav' else 4,failsafe=invalid=='failsafe')
                    if invalid=='old_receipt':ctx.node.received['status']=99.
                    if invalid=='wrong_target':ctx.vehicle.target=(2,1)
                ext._pump_recovery_receive_only.side_effect=receive
                self.assertFalse(ext.before_failed_fold(ctx,loop,meta))
                self.assertLess(clock.now-100.,.37)
                self.assertFalse(meta['failed_recovery_handoff']['confirmed'])
                loop.pump.assert_not_called();ctx.node.zero_all.assert_not_called()

    def test_send_or_receive_exception_records_failure_and_does_not_block_land(self):
        for component in ('send','receive'):
            with self.subTest(component=component):
                ext,ctx,loop,_,_,_=self.fixture();meta={}
                (ctx.vehicle._send_command if component=='send' else ext._pump_recovery_receive_only).side_effect=OSError('synthetic')
                self.assertFalse(ext.before_failed_fold(ctx,loop,meta))
                self.assertEqual(meta['failed_recovery_handoff']['reason'],'handoff_exception')
                ext.land_after_failed_recovery(ctx,loop,meta)
                ctx.vehicle.land.assert_called_once()

    def test_missing_zero_or_unverified_exit_skips_mode_and_fold(self):
        for missing in ('writer_exited','zero_consumed'):
            ext,ctx,loop,_,_,_=self.fixture();meta={}
            def release(*args,**kwargs):
                ext.release={'writer_exited':True,'zero_consumed':True};ext.release[missing]=False;ext.active=False
            ext._release.side_effect=release
            self.assertFalse(ext.before_failed_fold(ctx,loop,meta))
            ctx.vehicle._send_command.assert_not_called()
            self.assertEqual(meta['failed_recovery_handoff']['reason'],'writer_exit_or_zero_consumption_unverified')

    def test_unstarted_control_skips_fold_and_does_not_touch_idle_cpp(self):
        ext,ctx,loop,_,_,_=self.fixture();ext.started=ext.active=False;meta={}
        self.assertFalse(ext.before_failed_fold(ctx,loop,meta))
        ext._release.assert_not_called();ctx.vehicle._send_command.assert_not_called()

    def process_fixture(self,mode='interrupt'):
        ext,ctx,loop,clock,kills,actual=self.fixture();ext._release=actual
        state={'returncode':None}
        def wait(timeout):
            if mode=='interrupt' or (mode=='kill' and kills[-1][1]==SIGNALS.SIGKILL):
                state['returncode']=0;return 0
            clock.sleep(timeout);raise subprocess.TimeoutExpired('owned dispatcher',timeout)
        proc=NS(pid=1234,poll=lambda:state['returncode'],wait=Mock(side_effect=wait))
        ext.procs['dispatcher']=proc
        sent=NS(session_id=7,epoch=2,command_sequence=12,dispatch_sequence=44,stop_zero=True,
                joint_velocity=[0.,0.],expected_counter=[8,8])
        receipt=NS(session_id=7,epoch=2,command_sequence=12,dispatch_sequence=44,joint_consumed=True,failed=False,
                   consumed_counter=[8,8],pva_send_returned=False)
        ext.latest={'sends':sent,'receipts':receipt}
        return ext,ctx,loop,clock,kills,proc

    def test_bounded_release_verifies_identity_and_restores_legacy_only_after_death(self):
        for mode in ('interrupt','kill'):
            ext,ctx,loop,clock,kills,proc=self.process_fixture(mode)
            ext._release(loop,bounded_failed_recovery=True)
            self.assertTrue(ext.release['writer_exited']);self.assertTrue(ext.release['zero_consumed'])
            self.assertFalse(ext.recovery_owner_unverified);self.assertFalse(ext.active)
            self.assertIs(ctx.vehicle._pump,loop.pump)
            self.assertEqual(kills[0],(1234,SIGNALS.SIGINT))
            if mode=='kill':self.assertEqual(kills[-1],(1234,SIGNALS.SIGKILL))
            self.assertLess(clock.now-100.,.25)

    def test_bad_zero_identity_times_out_without_claiming_consumption(self):
        for field,value in [('session_id',8),('epoch',3),('command_sequence',11),('dispatch_sequence',43),
                            ('failed',True),('joint_consumed',False),('consumed_counter',[7,8]),('pva_send_returned',True)]:
            ext,ctx,loop,clock,_,_=self.process_fixture()
            setattr(ext.latest['receipts'],field,value)
            ext._release(loop,bounded_failed_recovery=True)
            self.assertFalse(ext.release['zero_consumed']);self.assertTrue(ext.release['writer_exited'])
            self.assertLess(clock.now-100.,.18)

    def test_unconfirmed_killed_owner_keeps_receive_only_and_suppresses_land_zero_all(self):
        ext,ctx,loop,clock,kills,proc=self.process_fixture('unknown');meta={}
        ext._release(loop,bounded_failed_recovery=True)
        self.assertFalse(ext.release['writer_exited']);self.assertTrue(ext.recovery_owner_unverified)
        self.assertIs(ctx.vehicle._pump,ext._pump_recovery_receive_only)
        original_zero=ctx.node.zero_all
        def land():ctx.node.zero_all();ctx.vehicle._pump();ctx.vehicle._send_command(21,[0.]*7)
        ctx.vehicle.land.side_effect=land
        ext.land_after_failed_recovery(ctx,loop,meta)
        original_zero.assert_not_called();loop.pump.assert_not_called()
        self.assertIs(ctx.node.zero_all,original_zero)
        ctx.vehicle._send_command.assert_called_once_with(21,[0.]*7)
        self.assertTrue(meta['unverified_owner_land_zero_publish_suppressed'])
        self.assertEqual(kills,[(1234,SIGNALS.SIGINT),(1234,SIGNALS.SIGKILL)])

    def test_real_receive_only_pump_never_calls_legacy_or_arm(self):
        ext,ctx,loop,clock,_,_=self.fixture()
        del ext._pump_recovery_receive_only
        ext._pump_recovery_receive_only()
        ext.loop_timing.original['original_pump'].assert_called_once_with(ros_timeout_s=0.)
        self.assertEqual(ctx.node.spin_once.call_count,32)
        loop.land_watchdog.observe.assert_called_once_with(ctx.node,42.)
        loop.status_watchdog.observe.assert_called_once_with(ctx.node,42.)
        loop.pump.assert_not_called();ctx.node.zero_all.assert_not_called()

    def test_shared_opt_in_hook_failure_never_prevents_land_branch(self):
        hook=definition(Path('simulations/x650_wholebody_mpc_v1/run_case.py'),'prepare_failed_fold',{})
        self.assertTrue(hook(None,None,{},None))
        for result in (False,None,OSError('synthetic evidence failure')):
            meta={};extension=NS(before_failed_fold=Mock())
            if isinstance(result,Exception):extension.before_failed_fold.side_effect=result
            else:extension.before_failed_fold.return_value=result
            self.assertFalse(hook(None,None,meta,extension))
            self.assertTrue(meta['fold_skipped_for_direct_land'])
            if isinstance(result,Exception):self.assertIn('synthetic',meta['failed_recovery_handoff_error'])


if __name__=='__main__':unittest.main()
