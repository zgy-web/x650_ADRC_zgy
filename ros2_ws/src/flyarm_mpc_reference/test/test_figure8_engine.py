"""Trajectory admission and one-shot actual-anchor contracts; no physical claims."""
from copy import deepcopy
import json
from pathlib import Path
import sys
import shutil
import uuid
from contextlib import contextmanager
from types import SimpleNamespace as NS
import unittest
from unittest.mock import Mock, patch
import numpy as np
from test_engine import engine, snapshot, REPO
sys.path.insert(0,str(REPO))
from flyarm_mpc_reference.figure8_factory import FigureEightEngine
from simulations.x650_isolated_figure8_v1 import task
from simulations.x650_wholebody_mpc_v1.run_case import FigureEight, ConstantTarget


@contextmanager
def temporary_fixture():
    root=(REPO/'tmp'/('normal_gate_contract_'+uuid.uuid4().hex)).resolve()
    root.relative_to(REPO.resolve())
    root.mkdir(parents=True)
    try:yield root
    finally:
        root.relative_to(REPO.resolve())
        shutil.rmtree(root)


class FigureEightEngineTests(unittest.TestCase):
    def make(self):
        base, adapter=engine()
        adapter.reset=Mock()
        adapter.allocator=NS(prime_command_history=Mock(),trajectory=ConstantTarget([9.,9.,9.]))
        params=dict(amplitude_x=.25,amplitude_z=.1,period_s=12.,cycles=2,start_s=2.,ramp_s=2.)
        live=FigureEightEngine(base,params,bytes(range(32)),FigureEight)
        start=NS(session_id=7,epoch=1,configuration_sha256=list(range(32)),origin_s=3.,history_count=2,
            history_time_s=[2.98,3.]+[0.]*62,history_command=[.01,-.02,.03,.04,-.05]*2+[0.]*310,
            target_position_enu=[.2,-.3,1.6],anchor_position_enu=[.2,-.3,2.],
            anchor_rotation_row_major=np.eye(3).reshape(-1),anchor_q=[0.,0.])
        return live,adapter,start

    def test_one_handoff_installs_same_actual_center_and_preview_trajectory(self):
        live,adapter,start=self.make()
        live.begin_live(start)
        self.assertIs(live.trajectory,adapter.allocator.trajectory)
        np.testing.assert_array_equal(live.trajectory.center,start.target_position_enu)
        self.assertEqual(adapter.calls,[])
        self.assertEqual(live.trajectory.end_s,28.)
        history,origin=adapter.allocator.prime_command_history.call_args.args
        self.assertEqual(origin,3.)
        np.testing.assert_array_equal(history[-1][1],[.01,-.02,.03,.04,-.05])
        with self.assertRaises(ValueError):live.begin_live(start)

    def test_moving_target_is_verified_at_same_task_time_without_phase_fit(self):
        live,adapter,start=self.make();live.begin_live(start)
        s=snapshot();s.replay_only=False;s.task_time_s=s.reference_time_s=7.02
        live.last_control_time=10.
        for field,value in zip(('target_position_enu','target_velocity_enu','target_acceleration_enu'),
                               live.trajectory.sample(s.task_time_s)):
            setattr(s,field,value.tolist())
        self.assertGreater(np.linalg.norm(s.target_velocity_enu),.001)
        live._decode(s,1_000_000_000)
        s.target_position_enu[0]+=.001
        with self.assertRaises(ValueError):live._decode(s,1_000_000_000)
        self.assertEqual(adapter.calls,[])

    def test_before_live_start_is_not_executable(self):
        live,adapter,_=self.make();s=snapshot();s.replay_only=False
        self.assertNotEqual(live.process(s,NS).status,0)
        self.assertEqual(adapter.calls,[])


class NormalAdmissionTests(unittest.TestCase):
    def test_declared_revision_selects_baseline_and_old_binding_is_rejected(self):
        with temporary_fixture() as root:
            protocol_file=root/'configs/experiments/x650_isolated_live_v1.json'
            protocol_file.parent.mkdir(parents=True)
            old=root/'old.json';new=root/'new.json'
            old.write_text(json.dumps({'mpc_config':{'horizon':15},'numerical_backend':{'version':1}}))
            new.write_text(json.dumps({'mpc_config':{'horizon':15},'numerical_backend':{'version':2}}))
            old_binding={'path':str(old),'sha256':task.sha(old)}
            new_binding={'path':str(new),'sha256':task.sha(new)}
            gate={'path':str(root/'certificate.json'),'sha256':'a'*64}
            with patch.object(task,'ROOT',root),patch.object(task.hold,'validate_task_config',side_effect=deepcopy):
                protocol_file.write_text(json.dumps({'frozen_hold_task_sha256':old_binding['sha256']}))
                old_cfg=task.build_task_config(old_binding,gate)
                self.assertEqual(old_cfg['figure8_protocol']['baseline_task_sha256'],old_binding['sha256'])
                protocol_file.write_text(json.dumps({'frozen_hold_task_sha256':new_binding['sha256']}))
                self.assertEqual(task.baseline_sha256(),new_binding['sha256'])
                with self.assertRaisesRegex(ValueError,'accepted frozen version'):
                    task.validate_task_config(old_cfg)
                new_cfg=task.build_task_config(new_binding,gate)
                self.assertEqual(new_cfg['numerical_backend']['version'],2)
                self.assertEqual(task.validate_task_config(new_cfg),new_cfg)
                new_cfg['mpc_config']['horizon']=14
                with self.assertRaises(ValueError):task.validate_task_config(new_cfg)

    def test_controller_change_and_fast_are_not_admitted(self):
        with patch.object(task,'baseline_config',side_effect=lambda _: {'mpc_config':{'horizon':15}}):
            cfg=task.build_task_config({'path':'baseline','sha256':'a'*64},{'path':'hold','sha256':'b'*64})
            self.assertEqual(task.validate_task_config(cfg),cfg)
            for key,value in [('stage','fast'),('duration_s',24.)]:
                changed=deepcopy(cfg);changed[key]=value
                with self.assertRaises(ValueError):task.validate_task_config(changed)
            cfg['mpc_config']['horizon']=14
            with self.assertRaises(ValueError):task.validate_task_config(cfg)

    def test_complete_certificate_checks_original_artifacts_and_numerical_sources(self):
        with temporary_fixture() as temp:
            root=Path(temp);launch=root/'launch';run=root/'run'
            launch.mkdir();run.mkdir()
            def write(path,value):
                path.parent.mkdir(parents=True,exist_ok=True)
                path.write_text(json.dumps(value),encoding='utf-8');return task.sha(path)
            baseline=write(launch/'task.json',{'test_fixture_only':True})
            protocol={'frozen_numerical_library_sha256':'f'*64,'tip_hold_duration_s':30.,
                      'frozen_hold_task_sha256':baseline}
            write(root/'configs/experiments/x650_isolated_live_v1.json',protocol)
            names=['control_packages/x650_wholebody_v1/source.py']+[
                'ros2_ws/src/flyarm_mpc_reference/flyarm_mpc_reference/'+n
                for n in ('factory.py','engine.py','isolated_node.py','diagnostic_channel.py')]
            before={n:write(root/n,{'fixture':n}) for n in names}
            write(launch/'source_manifest.json',before);write(launch/'source_manifest_after.json',before)
            write(launch/'sources.zip',{'fixture_only':True})
            write(run/'controller_metadata.json',{'complete':True})
            binary=root/'dispatcher';binary_hash=write(binary,{'fixture':True})
            write(launch/'launch.json',dict(protocol=protocol,complete=True,task_sha256=baseline,
                run_directories=[str(run)],runtime_closed=True,sources_unchanged=True,
                runtime_binaries_unchanged=True,runtime_binaries_sha256={str(binary):binary_hash}))
            cert=dict(kind='x650_isolated_live_stage_certificate_v1',stage='tip_hold',passed=True,
                complete=True,full_window_ulog_verified=True,parameter_restore_verified=True,
                native_ground_verified=True,actuator_log_verified=True,
                source_binding={'verified':True,'task_sha256':baseline,'native_library_sha256':'f'*64},
                window_sim_s=[42.,72.],launch_directory=str(launch),source_run_directory=str(run),
                artifacts={str(p):task.sha(p) for p in [*launch.iterdir(),*run.iterdir()]})
            path=root/'cert.json';write(path,cert)
            cfg={'isolated_live_hold':{'path':str(path),'sha256':task.sha(path)}}
            with patch.object(task,'ROOT',root):
                self.assertTrue(task.validate_live_hold(cfg)['verified'])
                write(root/names[0],{'changed':True})
                with self.assertRaisesRegex(ValueError,'source changed'):task.validate_live_hold(cfg)
                write(root/names[0],{'fixture':names[0]})
                write(run/'controller_metadata.json',{'complete':False})
                with self.assertRaisesRegex(ValueError,'evidence changed'):task.validate_live_hold(cfg)

    def test_old_or_partial_hold_pass_flag_is_insufficient(self):
        for doc in ({'kind':'x650_ts20_conditional_stage_certificate_v1','passed':True},
                    {'kind':'x650_isolated_live_stage_certificate_v1','stage':'tip_hold','passed':True}):
            with patch.object(task,'bound_document',return_value=doc):
                with self.assertRaisesRegex(ValueError,'complete actual isolated'):
                    task.validate_live_hold({'isolated_live_hold':{}})


if __name__=='__main__':unittest.main()
