"""NX computes; original PX4/Gazebo plant advances only at explicit barriers.

The server Python orchestration is intentionally NOT a real-time replacement
for the C++ dispatcher. Wall time is diagnostic, simulation time is the control
clock. The existing estimated feedback, actuator plant and NAV recovery remain.
"""
from copy import deepcopy
import json
import os
from pathlib import Path
import select
import subprocess
import sys
import time
import numpy as np

ROOT=Path(__file__).resolve().parents[2]
for path in (ROOT,ROOT/'src',ROOT/'scripts',ROOT/'control_packages/x650_wholebody_v1'):
    sys.path.insert(0,str(path))
from simulations.x650_ts20_hold_v1.run_case import Ts20Extension,shared
from simulations.x650_cpp_multirate_v1 import task as source_task
from simulations.x650_cpp_live_v1.run_case import expected_core_config
from simulations.x650_nx_sync_pil_v1.protocol import Client,clock_barrier
from simulations.x650_nx_sync_pil_v1.cases import compose
from x650_pil_transport import validate_candidate


def read(path):return json.loads(Path(path).read_bytes())
def sha(path):return source_task.sha(path)


def prepare(plan):
    binding=plan['source_task']
    if sha(binding['path'])!=binding['sha256']:raise ValueError('Source task changed')
    original=read(binding['path'])
    model,provenance=source_task.preflight(original)
    if original['multirate']['case_id']!='tip_hold':raise ValueError('Quiet hold preparation required')
    if plan['kind']!='x650_nx_sync_pil_v1' or plan['physics_tick_ns']!=4_000_000 or plan['solve_ticks']!=10 or plan['dispatch_ticks']!=5:
        raise ValueError('Explicit fixed physics/control grid required')
    clock=plan['clock_probe']
    if sha(clock['path'])!=clock['sha256'] or not read(clock['path']).get('px4_clock_frozen_verified') or not read(clock['path']).get('passed'):
        raise ValueError('New physical/PX4 clock proof is missing')
    if sha(plan['startup_manifest']['path'])!=plan['startup_manifest']['sha256']:raise ValueError('Core config changed')
    manifest=read(plan['startup_manifest']['path'])
    cfg,trajectory=compose(original,plan)
    provenance=deepcopy(provenance)
    provenance['sync_pil']={'kind':plan['kind'],'source_task':binding,'clock':'paused fixed-step Gazebo',
        'wall_clock_qualified':False,'hardware_flight_qualified':False,'physical_acceptance_inherited':False,
        'case':plan['case'],'trajectory':trajectory,'prior_sync_hold':plan.get('prior_sync_hold')}
    provenance['stage']=cfg['stage']
    return cfg,model,provenance,manifest


class NoLocalOptimizer:
    def __init__(self,model,pid):
        self.model=model;self.base_position_gain=np.array([pid['MPC_XY_P'],pid['MPC_XY_P'],pid['MPC_Z_P']])
    def reset(self,*args):pass
    def metadata(self):return {'kind':'NX_ABI4_sync_simulation_only','python_optimizer':False}
    def step(self,*args):raise RuntimeError('Server optimizer must never execute')


class SyncExtension(Ts20Extension):
    def __init__(self,cfg,provenance,plan,manifest):
        # The legacy plant selector uses the exact original motor scenario.
        # This is only the TS20 observation adapter, never the new controller.
        adapter_cfg=source_task.baseline_config(read(plan['source_task']['path']))
        super().__init__(adapter_cfg,provenance)
        self.sync_cfg=cfg
        self.plan,self.manifest=plan,manifest
        self.active=False;self.paused=False;self.client=None;self.helper=None
        self.events=[];self.nx_records=[];self.sent=[];self.sequence=0;self.release={}
        self.previous_command=None
        self.session=int.from_bytes(os.urandom(4),'little') or 1
        self.config_sha=source_task.canonical_hash({'task':cfg,'sync_plan':plan})

    def validate_runtime(self,cfg,model,provenance,out):
        checked,_,p,m=prepare(self.plan)
        if cfg!=checked or p!=provenance or m!=self.manifest:raise ValueError('Sync inputs changed')
        self.log_dir=Path(os.environ['AMSIM_TS20_LOG_DIR']);self.out=Path(out)
        if not self.log_dir.is_dir():raise ValueError('Unique actuator log directory missing')

    def stage_gates(self,*args):
        return {'new_clock_probe':self.plan['clock_probe'],'scope':'new synchronous experiment; no live timing certificate inherited'}

    def on_loop_ready(self,ctx,loop,meta,out):
        try:
            super().on_loop_ready(ctx,loop,meta,out)
            # Establish Gazebo discovery on the ground. Starting this helper
            # at airborne handoff stalls the still-live prestream scheduler.
            self.helper=subprocess.Popen([self.plan['stepper'],'default','4000000'],stdin=subprocess.PIPE,stdout=subprocess.PIPE,text=True)
            meta['sync_stepper_ground_ready']=self.barrier('status')
        except BaseException:
            # Also works with the source-pinned historical lifecycle on server:
            # a failed setup must leave a functioning ground recovery receiver.
            ctx.vehicle._pump=loop.original_pump
            raise

    def build_allocator(self,cfg,model,pid,trajectory,loop):
        prior=read(self.provenance['base_response_prior']['path'])
        if expected_core_config(cfg,model,pid,self.predictor,prior)!=self.manifest['config']:
            raise ValueError('Actual model/PID/predictor differs from NX core config')
        return NoLocalOptimizer(model,pid)

    def barrier(self,command):
        self.helper.stdin.write(command+'\n');self.helper.stdin.flush()
        if not select.select([self.helper.stdout],[],[],6.)[0]:raise TimeoutError('Gazebo barrier helper timeout')
        row=json.loads(self.helper.stdout.readline());self.events.append(row)
        if not row['ok']:raise RuntimeError(row)
        self.paused=row['owns_pause']
        return row

    def drain(self,clock_ns):
        # No state values or timestamps are projected. Wait for actual ROS
        # callbacks at the stopped world boundary, including queued PX4 data.
        end=time.monotonic()+1.5;stable=None
        receiver=self.loop_timing.original['original_pump']
        while time.monotonic()<end:
            receiver(ros_timeout_s=0.)
            for _ in range(48):self.loop.ctx.node.spin_once(timeout_sec=0.)
            current=self.loop.ctx.node.estimator_clock_s
            if current is not None and abs(current-clock_ns/1e9)<1e-9:
                if stable is None:stable=time.monotonic()
                if time.monotonic()-stable>=.02:return
            else:stable=None
            time.sleep(.001)
        raise TimeoutError('Actual ROS clock did not reach frozen physics boundary')

    def activate(self,ctx,loop,cfg,meta,rotation):
        if self.helper is None or self.helper.poll() is not None:raise RuntimeError('Ground-prepared stepper is unavailable')
        boundary=self.barrier('pause')['sim_ns'];self.clock_ns=boundary;self.drain(boundary)
        # Finish the old owner's prestream at this actual paused timestamp.
        loop.next_control=loop.control_now();loop.pump()
        history=loop.sent_command_history[-64:]
        if abs(history[-1][0]-boundary/1e9)>1e-9:raise RuntimeError('No actual prestream send at frozen handoff boundary')
        self.last_solve=history[-1][0];self.origin=self.last_solve;self.clock_ns=boundary;self.previous_command=None
        self.gains=self.manifest['config']['base_position_gain']
        handoff={'session_id':self.session,'epoch':1,'configuration_sha256':list(bytes.fromhex(self.config_sha)),
            'issued_monotonic_ns':time.monotonic_ns(),'origin_s':self.last_solve,
            'anchor_position_enu':loop.anchor.tolist(),'anchor_rotation_row_major':np.asarray(rotation).reshape(-1).tolist(),
            'anchor_q':[0.,0.],'target_position_enu':loop.trajectory.sample(0.)[0].tolist(),
            'history_count':len(history),'history_time_s':[float(x[0]) for x in history]+[0.]*(64-len(history)),
            'history_command':np.asarray([x[1] for x in history]).reshape(-1).tolist()+[0.]*(320-5*len(history)),
            'ref_alt':float(loop.fixed_reference.document['px4_reference_at_bind']['ref_alt']),'yaw_ned':float(loop.yaw_ned)}
        self.client=Client(self.plan['forward_port'])
        request={'op':'start','scope':'simulation_pil','config':self.manifest['config'],
            'trajectory':self.provenance['sync_pil']['trajectory'],'handoff':handoff}
        response,sent,received=self.client.request(request)
        self.nx_records.append({'request':request,'response':response,'sent_ns':sent,'received_ns':received})
        if response['status']!='ready' or response['source_sha256']!=self.plan['nx_source_sha256']:raise RuntimeError('NX did not admit handoff')
        clock_barrier(boundary,self.barrier('status')['sim_ns'])
        self.active=True;loop.next_control=float('inf');ctx.vehicle._pump=self.pump
        loop.task_start=self.last_solve;loop.recorder.phase='measure'
        meta.update(phase='measure',task_start_sim_s=loop.task_start,reference_base_enu_m=loop.anchor,
            reference_rotation=rotation,requested_duration_s=cfg['duration_s'],
            synchronous_pil=self.provenance['sync_pil'],activation_command_history=shared.json_value(history))
        self.meta=meta
        print('NX_SYNC_ACTIVE',cfg['stage'],loop.task_start,flush=True)
        return loop.task_start

    def snapshot(self,now,state,joint_stamp):
        loop=self.loop;t=now-loop.task_start;context=self.runtime.context(state,t)
        s={'schema_version':1,'session_id':self.session,'epoch':1,'sequence':self.sequence+1,'replay_only':False,
            'configuration_sha256':list(bytes.fromhex(self.config_sha)),'issued_monotonic_ns':time.monotonic_ns(),
            'control_time_s':now,'task_time_s':t,'dt_s':now-self.last_solve,'state_time_s':state.stamp_s,'joint_time_s':joint_stamp}
        for k,v in [('position_enu',state.position),('velocity_enu',state.velocity),('omega_body_flu',state.omega_body),
            ('acceleration_enu',state.acceleration_enu),('rotation_row_major',state.rotation.reshape(-1)),('q',state.q),('dq',state.dq)]:s[k]=np.asarray(v).tolist()
        desired=loop.trajectory.sample(t);loop.recorder.tip_target=desired[0].copy()
        for k,v in zip(('target_position_enu','target_velocity_enu','target_acceleration_enu'),desired):s[k]=v.tolist()
        s.update(position_gains=self.gains,reference_time_s=float(context['reference_state']['time_s']),
            reference_r=np.asarray(context['reference_state']['r']).tolist(),bus_origin_s=float(context['bus_origin_s']))
        commands,packets=context['command_history'],context['packet_history']
        if not 1<=len(commands)<=64 or not 1<=len(packets)<=64:raise ValueError('History bounds')
        s.update(command_count=len(commands),packet_count=len(packets),
            command_time_s=[float(x[0]) for x in commands]+[0.]*(64-len(commands)),
            command_velocity=np.asarray([x[1] for x in commands]).reshape(-1).tolist()+[0.]*(128-2*len(commands)),
            packet_send_time_s=[float(x['send_time_s']) for x in packets]+[0.]*(64-len(packets)))
        for field in ('q_ref','dq_ref','tau_ff','kp','kd'):
            s['packet_'+field]=np.asarray([x[field] for x in packets]).reshape(-1).tolist()+[0.]*(128-2*len(packets))
        return {'op':'step','snapshot':s}

    def send(self,now,proposal,*,zero=False):
        from pymavlink import mavutil
        loop=self.loop
        joint=[0.,0.] if zero else proposal['joint_velocity']
        for name,value in zip(shared.JOINTS,joint):loop.ctx.arm.set_velocity(name,float(value))
        if proposal is not None:
            p,v,a=proposal['position_reference_enu'],proposal['velocity_feedforward_enu'],proposal['acceleration_feedforward_enu']
            ref=loop.fixed_reference.document['px4_reference_at_bind']['ref_alt']
            pn=[p[1],p[0],ref-p[2]];vn=[v[1],v[0],-v[2]];an=[a[1],a[0],-a[2]]
            loop.ctx.vehicle.link.mav.set_position_target_local_ned_send(int(now*1000)&0xffffffff,
                *loop.ctx.vehicle.target,mavutil.mavlink.MAV_FRAME_LOCAL_NED,2048,*pn,*vn,*an,loop.yaw_ned,0.)
        self.sent.append({'sim_s':now,'wall_ns':time.monotonic_ns(),'joint':joint,'proposal_sequence':None if proposal is None else proposal['sequence'],
            'position_ned':None if proposal is None else pn,'velocity_ned':None if proposal is None else vn,'type_mask':2048,'zero':zero})
        loop.max_joint_cmd=np.maximum(loop.max_joint_cmd,np.abs(joint))

    def consumed(self,now):
        if not self.sent:return
        row=self.runtime._past_row(now);sent=self.sent[-1]
        checks=[]
        for j in (1,2):
            checks.append(abs(row[f'u{j}']-sent['joint'][j-1])<1e-10 and row[f'receipt{j}_s']>=sent['sim_s']-1e-9)
        sent['consumption']={'verified':all(checks),'sim_s':row['sim_s'],
            'sequences':[int(row[f'upper_sequence{j}']) for j in (1,2)],'receipt_s':[row[f'receipt{j}_s'] for j in (1,2)]}
        if not all(checks):raise RuntimeError('Actual TS20 plugin did not consume last synchronous dispatch')

    def pump(self):
        loop=self.loop
        if loop.recovery or not self.active:
            if self.active or self.paused:self.release_owner()
            return loop.pump()
        before=self.clock_ns;step=self.barrier('step 5');self.clock_ns=step['sim_ns'];clock_barrier(before,self.clock_ns,ticks=5)
        self.drain(self.clock_ns)
        now,state,joint_stamp=loop.read();self.consumed(now)
        if abs(now-self.clock_ns/1e9)>1e-9:raise RuntimeError('Feedback clock differs from barrier')
        # At the final observation boundary, consume the previous command and
        # return. Do not publish a fresh candidate that recovery would replace
        # before the next physics iteration could consume it.
        if now-self.origin>=self.sync_cfg['duration_s']-1e-9:return
        if now-self.last_solve>=.04-1e-9:
            request=self.snapshot(now,state,joint_stamp);response,sent,received=self.client.request(request)
            record={'request':request,'response':response,'sent_ns':sent,'received_ns':received,
                'feedback':shared.json_value(loop.feedback_diagnostics)}
            self.nx_records.append(record)
            clock_barrier(self.clock_ns,self.barrier('status')['sim_ns'])
            record['validation']=validate_candidate(request,response,sent_ns=sent,received_ns=received,
                issue_ns=request['snapshot']['issued_monotonic_ns'],source_sha256=self.plan['nx_source_sha256'],check_deadline=False)
            record['clock_scope']='synchronous_simulation; WAN wall time excluded; NX local 50 ms gate unchanged'
            self.previous_command=response['proposal_nx_clock'];self.sequence+=1;self.last_solve=now
        if self.previous_command is not None:
            if now-self.previous_command['control_time_s']>.16+1e-9:raise RuntimeError('Simulation command lease expired')
            self.send(now,self.previous_command)
        # First 20 ms uses the actual prestream hold; first MPC is at 40 ms.

    def release_owner(self):
        loop=self.loop
        if not self.paused:return
        error=None
        try:
            self.send(self.clock_ns/1e9,self.previous_command,zero=True)
            before=self.clock_ns;self.clock_ns=self.barrier('step 5')['sim_ns'];clock_barrier(before,self.clock_ns,ticks=5)
            self.drain(self.clock_ns);self.consumed(self.clock_ns/1e9)
            self.release['zero_consumed']=True
        except Exception as e:error=e;self.release['zero_consumed']=False;self.release['error']=repr(e)
        finally:
            if self.client:
                try:self.client.request({'op':'stop','session_id':self.session,'epoch':1})
                except Exception as e:self.release['nx_stop_error']=repr(e)
                self.client.close();self.client=None
            self.active=False
            self.barrier('resume');self.barrier('stop');self.helper.wait(timeout=3)
            loop.ctx.vehicle._pump=loop.pump;loop.last=None;loop.next_control=loop.control_now()
            self.release['resumed_for_original_recovery']=True
        if error is not None:raise error

    def before_failed_fold(self,ctx,loop,meta):
        if self.paused:self.release_owner()
        return True

    def finalize(self,meta,stage,out):
        if self.paused:
            try:self.release_owner()
            except Exception as e:meta['sync_cleanup_error']=repr(e)
        if self.helper is not None and self.helper.poll() is None:
            try:self.barrier('stop');self.helper.wait(timeout=3)
            except Exception as e:
                meta['sync_helper_stop_error']=repr(e);self.helper.terminate();self.helper.wait(timeout=3)
        super().finalize(meta,stage,out)
        for name,rows in [('sync_clock.jsonl',self.events),('nx_exchange.jsonl',self.nx_records),('sync_dispatch.jsonl',self.sent)]:
            with (Path(out)/name).open('x') as f:
                for row in rows:f.write(json.dumps(shared.json_value(row),allow_nan=False)+'\n')
            stage['artifacts'][name]=sha(Path(out)/name)
        meta['sync_release']=self.release;meta['nx_steps']=self.sequence
        for document in (meta,stage):document.update(synchronous_simulation_only=True,wall_clock_qualified=False,
            hardware_qualified=False,eligible_as_stage_gate=False,physical_acceptance_inherited=False)


def scenario():
    plan=read(os.environ['FLYARM_NX_SYNC_PLAN'])
    cfg,model,provenance,manifest=prepare(plan)
    out=Path(os.environ['AMSIM_RUN_DIR']);extension=SyncExtension(cfg,provenance,plan,manifest)
    return shared.execute(cfg,model,provenance,out,extension=extension)


if __name__=='__main__':scenario()
