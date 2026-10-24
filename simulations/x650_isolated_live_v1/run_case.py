"""Opt-in actual Gazebo/PX4 adapter for the isolated frozen MPC runtime.

The shared ScenarioContext still owns takeoff, prestream, supervision and
recovery. During active control it receives/validates feedback only; it never
calls the optimizer or sends a joint/PVA command. No simulated plant is replaced.
"""
from pathlib import Path
from copy import deepcopy
import base64
import json
import os
import signal
import subprocess
import sys
import tempfile
import time
import numpy as np
ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT))
from simulations.x650_ts20_hold_v1.run_case import Ts20Extension, ROOT, task, shared


def sha(p):
    return task.sha(p)


def write(p,value):
    shared.atomic_json(p,value)


class LiveExtension(Ts20Extension):
    def pump_actual(self):
        loop=self.loop
        if loop.recovery or not self.active:
            if self.active:self._release(loop)
            return loop.pump()
        begin=time.monotonic_ns()
        self._check_supervision()
        # Keep the original MAVLink parser/heartbeat, then a time-bounded ROS
        # drain instead of completing 32 callbacks before every source sample.
        # Mixing .02 and 0 on this executor repeatedly resets rclpy's ready
        # callback iterator; late-created evidence subscriptions can starve.
        self.loop_timing.original['original_pump'](ros_timeout_s=0.)
        until=time.monotonic_ns()+250_000
        count=0
        while count<32 and time.monotonic_ns()<until:
            loop.ctx.node.spin_once(timeout_sec=0.);count+=1
        drained=time.monotonic_ns()
        self.handle_active_pump(loop,loop.monitor_now())
        self.pump_times.append((begin,drained,time.monotonic_ns(),count))

    def on_pump(self, loop, sim):
        super().on_pump(loop, sim)
        if self.handle_active_pump(loop, sim):
            # The existing scheduler already skips legacy control until
            # next_control. Keep reception and lifecycle watchdogs running,
            # while granting the C++ dispatcher exclusive output ownership.
            # _release restores the legacy schedule before recovery proceeds.
            loop.next_control = float('inf')

    def prepare(self,out):
        # Construct and validate the worker BEFORE the shared lifecycle opens
        # its reversible parameter transaction. Never relax the pending-journal
        # guard to create a second worker while a flight transaction is active.
        self.resources=json.loads(os.environ['FLYARM_EXECUTION_RESOURCES'])
        protocol=task.read(ROOT/'configs/experiments/x650_isolated_live_v1.json')
        if self.resources['roles']!=protocol['cpu_roles']:raise ValueError('Execution CPU plan differs')
        # Limit only our own source threads. Child roles explicitly override
        # this affinity before imports, including both background recorders.
        for thread in Path('/proc/self/task').iterdir():
            os.sched_setaffinity(int(thread.name),set(self.resources['roles']['source']))
        from ament_index_python.packages import get_package_prefix
        from rclpy.serialization import serialize_message
        from flyarm_control_interfaces import msg
        self.msg,self.serialize=msg,serialize_message
        self.prefix=get_package_prefix
        self.active=self.stopping=self.started=self.cleaned=False
        self.out=Path(out)/"isolated_live";self.out.mkdir()
        write(self.out/'execution_resources.json',self.resources)
        self.session=int.from_bytes(os.urandom(4),'little')&0x7fffffff or 1
        self.epoch=1
        self.task_path=Path(os.environ[task.CONFIG_ENV])
        self.task_sha=sha(self.task_path)
        self.procs,self.logs={},[]
        self.sources=[]
        self.latest={};self.message_counts={};self.evidence_sequence=0
        self.source_sender=None
        self.supervisor=None
        self.pump_times=[]
        self.failure=None;self.last_source=-1.;self.source_sequence=0
        init={'schema_version':1,'synthetic_software_fixture_only':True,'simulation_live':True,
            'deferred_actual_handoff':True,'task_origin_sim_s':0.,'anchor_position_enu':[0.,0.,0.],
            'anchor_rotation_row_major':np.eye(3).reshape(-1).tolist(),'anchor_q':[0.,0.],
            'target_position_enu':[0.,0.,0.],
            'published_prestream_history':[{'control_time_s':0.,'command':[0.]*5}],
            'scope':'constructor-only template; zero solves; all references/history replaced by actual LiveStart before admission'}
        write(self.out/'initialization.json',init)
        self.private=Path(tempfile.mkdtemp(prefix='flyarm-live-'));self.private.chmod(0o700)
        sock=self.private/'diag.sock'
        self._launch('recorder',[sys.executable,str(ROOT/'scripts/x650_isolated_diagnostic_recorder.py'),
            '--socket',str(sock),'--output',str(self.out/'recording')])
        deadline=time.monotonic()+10
        while not sock.exists():
            self._alive()
            if time.monotonic()>deadline:raise TimeoutError('Diagnostic process not ready')
            time.sleep(.01)
        source_sock=self.private/'source.sock'
        self._launch('source_recorder',[sys.executable,str(ROOT/'scripts/x650_isolated_diagnostic_recorder.py'),
            '--socket',str(source_sock),'--output',str(self.out/'source_recording')])
        deadline=time.monotonic()+10
        while not source_sock.exists():
            self._alive()
            if time.monotonic()>deadline:raise TimeoutError('Source recorder not ready')
            time.sleep(.01)
        from flyarm_mpc_reference.diagnostic_channel import DiagnosticSender
        self.source_sender=DiagnosticSender(source_sock)
        self._launch('worker',self._ros('flyarm_mpc_reference','isolated_node',{
            'expected_session_id':self.session,'expected_epoch':1,'simulation_live':True,'repo_root':ROOT,
            'task_config':self.task_path,'task_config_sha256':self.task_sha,
            'initialization_manifest':self.out/'initialization.json','initialization_sha256':sha(self.out/'initialization.json'),
            'output_dir':self.out,'diagnostic_socket':sock}))
        deadline=time.monotonic()+60
        while not (self.out/'worker_ready').exists():
            self._alive()
            if time.monotonic()>deadline:raise TimeoutError('Preflight worker not ready')
            time.sleep(.01)

    def on_loop_ready(self,ctx,loop,meta,out):
        super().on_loop_ready(ctx,loop,meta,out)
        from rclpy.qos import QoSProfile, ReliabilityPolicy
        msg=self.msg
        from simulations.x650_isolated_live_v1.supervision import FaultSupervisor
        self.supervisor=FaultSupervisor(ctx.node.context,msg,self.session,self.epoch)
        self.source_pub=ctx.node.create_publisher(msg.ControlSnapshot,"/flyarm/isolated/v1/source",QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT))
        self.start_pub=ctx.node.create_publisher(msg.LiveStart,"/flyarm/isolated/v1/live_start",1)
        self.stop_pub=ctx.node.create_publisher(msg.RuntimeStop,"/flyarm/isolated/v1/stop",1)
        qos=QoSProfile(depth=4000,reliability=ReliabilityPolicy.BEST_EFFORT)
        self.message_types={'dispatch':msg.DispatchTick,'sends':msg.ActuatorSend,
                            'receipts':msg.ExecutionReceipt,'events':msg.GateEvent}
        def record(m,name):
            self.latest[name]=m
            self.message_counts[name]=self.message_counts.get(name,0)+1
            self._evidence({'kind':'ros','topic':name,'cdr':self.serialize(m)})
        self.subs=[ctx.node.create_subscription(typ,"/flyarm/isolated/v1/"+topic,
            lambda m,name=name:record(m,name),qos) for typ,topic,name in (
                (msg.DispatchTick,"dispatch",'dispatch'),(msg.ActuatorSend,"actuator_send",'sends'),
                (msg.ExecutionReceipt,"consumed",'receipts'),(msg.GateEvent,"event",'events'))]
        meta["isolated_execution"]={"kind":"isolated_simulation_v1","session_id":self.session,
            "configuration_sha256":self.task_sha,"numerical_backend_unchanged":True,
            "pva_ack_semantics":"MAVLink local send plus independent postflight PX4 ULog; no per-message ACK",
            "joint_ack_semantics":"actual plugin consumed counters, values and times; not ROS publish return",
            "ownership":"legacy prestream -> C++ only -> zero consumed -> legacy recovery",
            "source_adapter":"independent of algorithm/output/diagnostic processes; existing PX4 estimated and causal TS20 validators"}
        self._prepare_ground_transport(ctx,loop,meta)

    @staticmethod
    def _ground_state(ctx,loop):
        status,land=ctx.node.data['status'],ctx.node.data['land']
        result={'ground_landed':bool(land.landed),'ground_disarmed':int(status.arming_state)!=2,
                'loop_strict_false':not loop.strict,'before_live_start':loop.task_start is None}
        if not all(result.values()) or loop.offboard or loop.recovery:
            raise RuntimeError('Isolated preparation requires disarmed ground before strict flight control')
        return result

    def _transport_ready(self):
        return ((self.out/'worker_ready').exists() and self.start_pub.get_subscription_count()>=4
                and all((self.out/(name+'_ready')).exists() for name in ('ingress','dispatcher','consumption'))
                and self.source_pub.get_subscription_count()>=1 and self.message_counts.get('dispatch',0)>2)

    def _assert_waiting_no_output(self):
        self._check_supervision()
        if self.started or self.active or self.message_counts.get('sends',0):
            raise RuntimeError('Isolated actuator output occurred before actual LiveStart')
        tick=self.latest.get('dispatch')
        if tick is None or tick.status!=0 or tick.output_valid:
            raise RuntimeError('Dispatcher must remain idle without actuator output before handoff')
        if self.failure:raise RuntimeError(self.failure)

    def _prepare_ground_transport(self,ctx,loop,meta):
        # The shared airborne build hook previously repeated software-proof
        # hashing/library setup, followed by three process launches. Those
        # synchronous operations left a measured 34 ms hole between pumps.
        # Complete all of them before arming. No controller step or synthetic
        # reset occurs; the shared lifecycle still supplies the actual anchor.
        record={'kind':'isolated_ground_preparation_v1','start_monotonic_ns':time.monotonic_ns(),
                **self._ground_state(ctx,loop),'initial_task_origin_s':0.}
        start_cpu=time.thread_time_ns()
        self.prepared_configuration=deepcopy(loop.cfg)
        self.prepared_model=loop.model
        self.prepared_pid=dict(loop.pid)
        before=time.monotonic_ns()
        self.prepared_allocator=super().build_allocator(loop.cfg,loop.model,loop.pid,
                                                       shared.ConstantTarget(np.zeros(3)),loop)
        record['allocator_prepare_ns']=time.monotonic_ns()-before
        record['allocator_step_count']=int(self.prepared_allocator.step_count)
        if record['allocator_step_count']!=0:raise RuntimeError('Ground constructor unexpectedly advanced controller')
        self.prepared_allocator_claimed=False
        common={'expected_session_id':self.session,'expected_epoch':1,'configuration_sha256':self.task_sha,'simulation_live':True}
        gains=[loop.pid['MPC_XY_P'],loop.pid['MPC_XY_P'],loop.pid['MPC_Z_P']]
        clients=list(ctx.vehicle.link.clients)
        if len(clients)!=1:raise RuntimeError('Exactly one actual MAVLink peer required')
        address=clients[0]
        if not isinstance(address,tuple) or address[0]!='127.0.0.1':raise RuntimeError('Only discovered loopback SITL allowed')
        before=time.monotonic_ns()
        # Live ingress refuses source messages before LiveStart and replaces
        # this construction-only origin with its real recorded send boundary.
        self._launch('ingress',self._ros('flyarm_control_gateway','state_ingress',{**common,'position_gains':gains,'task_origin_s':0.,'ready_file':self.out/'ingress_ready'}))
        self._launch('dispatcher',self._ros('flyarm_control_gateway','live_dispatcher',{**common,
            'sitl_udp_port':int(address[1]),'sitl_socket_fd':ctx.vehicle.link.port.fileno(),
            'sitl_system_id':int(ctx.vehicle.target[0]),'sitl_component_id':int(ctx.vehicle.target[1]),'ready_file':self.out/'dispatcher_ready'}))
        self._launch('consumption',self._ros('flyarm_control_gateway','consumption_source',{**common,
            'state_csv':self.runtime.states.path,'ready_file':self.out/'consumption_ready'}))
        record['transport_launch_ns']=time.monotonic_ns()-before
        deadline=time.monotonic()+60
        while not self._transport_ready():
            loop.pump()
            self._alive()
            self._ground_state(ctx,loop)
            if self.message_counts.get('sends',0):raise RuntimeError('Ground transport sent an actuator command')
            if time.monotonic()>deadline:raise TimeoutError('Ground live nodes not ready')
        self._assert_waiting_no_output()
        record.update(self._ground_state(ctx,loop),transport_ready=True,
            allocator_constructed_no_step=self.prepared_allocator.step_count==0,
            normal_output_count=self.message_counts.get('sends',0),
            end_monotonic_ns=time.monotonic_ns(),thread_cpu_ns=time.thread_time_ns()-start_cpu)
        self.preflight_transport=record
        write(self.out/'preflight_transport.json',record)
        meta['isolated_preflight_transport']=record

    def build_allocator(self,cfg,model,pid,trajectory,loop):
        # No hashing, imports, native construction or process startup at this
        # airborne hook. Only attach the declared actual task reference.
        if (not hasattr(self,'prepared_allocator') or self.prepared_allocator_claimed
                or cfg!=self.prepared_configuration or model is not self.prepared_model
                or dict(pid)!=self.prepared_pid or loop is not self.loop
                or self.prepared_allocator.step_count!=0):
            raise RuntimeError('Ground-prepared allocator identity/state differs at handoff')
        self.prepared_allocator.trajectory=trajectory
        self.prepared_allocator_claimed=True
        return self.prepared_allocator

    def _launch(self,name,args):
        f=(self.out/(name+'.log')).open('x');self.logs.append(f)
        env=os.environ.copy();env.update(OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1',MKL_NUM_THREADS='1')
        cpus=self.resources['roles'].get(name,self.resources['background'])
        self.procs[name]=subprocess.Popen(['taskset','--cpu-list',','.join(map(str,cpus)),*args],cwd=ROOT,env=env,stdin=subprocess.DEVNULL,
            stdout=f,stderr=subprocess.STDOUT,start_new_session=True,
            pass_fds=(self.loop.ctx.vehicle.link.port.fileno(),) if name=='dispatcher' else ())

    def _ros(self,package,executable,params):
        path=Path(self.prefix(package))/'lib'/package/executable
        if not path.is_file():raise RuntimeError('Missing isolated executable '+str(path))
        command=[str(path),'--ros-args']
        for k,v in params.items():command+=['-p',k+':='+str(v).lower() if isinstance(v,bool) else k+':='+str(v)]
        return command

    def validate_activation_stage(self,cfg):
        if cfg['stage']!='tip_hold':raise ValueError('Live stage currently requires fresh complete tip_hold first')

    def activate(self,ctx,loop,cfg,meta,rotation):
        self.validate_activation_stage(cfg)
        if not loop.sent_command_history or loop.task_start is not None:raise RuntimeError('Actual prestream required')
        if not getattr(self,'preflight_transport',{}).get('transport_ready') or not self.prepared_allocator_claimed:
            raise RuntimeError('Ground preparation must complete before actual handoff')
        self._alive()
        self._assert_waiting_no_output()
        gains=[loop.pid['MPC_XY_P'],loop.pid['MPC_XY_P'],loop.pid['MPC_Z_P']]
        # Exact legacy send boundary, then one-way transfer. No legacy joint
        # or PVA writes occur until the explicit C++ stop/consumption handshake.
        previous=loop.sent_command_history[-1][0]
        ctx.vehicle.wait(lambda:loop.sent_command_history[-1][0]>previous,8.,'last legacy prestream send')
        history=loop.sent_command_history[-64:]
        observed={'source':self._affinity(os.getpid())}
        observed.update({name:self._affinity(p.pid) for name,p in self.procs.items()})
        for name,threads in observed.items():
            expected=self.resources['roles'].get(name,self.resources['background'])
            if not threads or any(cpus!=expected for cpus in threads.values()):
                raise RuntimeError('Owned process CPU isolation differs: '+name)
        write(self.out/'execution_affinity_at_handoff.json',observed)
        self._assert_waiting_no_output()
        start=float(history[-1][0])
        s=self.msg.LiveStart();s.session_id=self.session;s.epoch=1;s.configuration_sha256=list(bytes.fromhex(self.task_sha))
        s.issued_monotonic_ns=time.monotonic_ns();s.origin_s=start
        s.anchor_position_enu=np.asarray(loop.anchor).tolist();s.anchor_rotation_row_major=np.asarray(rotation).reshape(-1).tolist()
        s.anchor_q=[0.,0.];s.target_position_enu=loop.trajectory.sample(0.)[0].tolist()
        s.history_count=len(history);s.history_time_s=[float(t) for t,v in history]+[0.]*(64-len(history))
        s.history_command=np.asarray([v for t,v in history]).reshape(-1).tolist()+[0.]*(320-5*len(history))
        s.ref_alt=float(loop.fixed_reference.document['px4_reference_at_bind']['ref_alt']);s.yaw_ned=float(loop.yaw_ned)
        self.start_message=s;self.gains=gains
        self.active=self.started=True
        loop.next_control=float('inf')
        ctx.vehicle._pump=self.pump_actual
        loop.task_start=start;loop.recorder.phase='measure'
        meta.update(phase='measure',task_start_sim_s=start,reference_base_enu_m=loop.anchor,
            reference_rotation=rotation,requested_duration_s=cfg['duration_s'])
        meta['activation_command_history']=[{'control_time_s':t,'task_time_s':t-start,'command':v.tolist()} for t,v in history]
        meta['ts20_activation']={'task_start_sim_s':start,'reference_reset_to_measured_q':False,
            'command_source':'actual plugin-consumed upper commands','packet_source':'actual fixed-grid MIT sends','delay_applied_once':True}
        self.start_pub.publish(s)
        return start

    def _alive(self):
        for name,p in self.procs.items():
            if p.poll() is not None:raise RuntimeError('Isolated '+name+' exited '+str(p.returncode))

    @staticmethod
    def _affinity(pid):
        return {thread.name:sorted(os.sched_getaffinity(int(thread.name)))
                for thread in (Path('/proc')/str(pid)/'task').iterdir()}

    def _evidence(self,value):
        self.evidence_sequence+=1
        if not self.source_sender.offer(self.evidence_sequence,value):
            # A slow recorder never blocks source reception. Evidence loss is
            # explicit and disqualifies the run, including after recovery.
            self.failure='Source evidence queue rejected a record'

    def handle_active_pump(self,loop,sim):
        if not self.active:return False
        if loop.recovery:
            self._release(loop)
            return False
        self._alive()
        self._check_supervision()
        if self.failure:raise RuntimeError(self.failure)
        tick,event=self.latest.get('dispatch'),self.latest.get('events')
        if tick is not None and tick.status==3:raise RuntimeError('C++ output latched fault reason '+str(tick.reason))
        if event is not None and not event.accepted:raise RuntimeError('C++ candidate gate rejected reason '+str(event.reason))
        now=float(loop.ctx.node.estimator_clock_s)
        if now<=self.last_source:return True
        begin=time.monotonic_ns()
        now,state,joint_stamp=loop.read()
        t=now-loop.task_start
        if t<=0:return True
        context=self.runtime.context(state,t)
        s=self.msg.ControlSnapshot();s.schema_version=1;s.session_id=self.session;s.epoch=1
        self.source_sequence+=1;s.sequence=self.source_sequence;s.replay_only=False
        s.configuration_sha256=list(bytes.fromhex(self.task_sha));s.issued_monotonic_ns=begin
        s.control_time_s=float(now);s.task_time_s=float(t);s.dt_s=.02
        s.state_time_s=float(state.stamp_s);s.joint_time_s=float(joint_stamp)
        for field,value in [('position_enu',state.position),('velocity_enu',state.velocity),('omega_body_flu',state.omega_body),
            ('acceleration_enu',state.acceleration_enu),('rotation_row_major',state.rotation.reshape(-1)),('q',state.q),('dq',state.dq)]:
            setattr(s,field,np.asarray(value).tolist())
        desired=loop.trajectory.sample(t);loop.recorder.tip_target=desired[0].copy()
        s.target_position_enu=desired[0].tolist();s.target_velocity_enu=desired[1].tolist();s.target_acceleration_enu=desired[2].tolist()
        s.position_gains=self.gains;s.reference_time_s=float(context['reference_state']['time_s'])
        s.reference_r=np.asarray(context['reference_state']['r']).tolist();s.bus_origin_s=float(context['bus_origin_s'])
        commands=context['command_history'];packets=context['packet_history']
        if not 1<=len(commands)<=64 or not 1<=len(packets)<=64:raise RuntimeError('Actual history exceeds bounded wire contract')
        s.command_count=len(commands);s.packet_count=len(packets)
        s.command_time_s=[float(x[0]) for x in commands]+[0.]*(64-len(commands))
        s.command_velocity=np.asarray([x[1] for x in commands]).reshape(-1).tolist()+[0.]*(128-2*len(commands))
        s.packet_send_time_s=[float(x['send_time_s']) for x in packets]+[0.]*(64-len(packets))
        for field in ('q_ref','dq_ref','tau_ff','kp','kd'):
            setattr(s,'packet_'+field,np.asarray([x[field] for x in packets]).reshape(-1).tolist()+[0.]*(128-2*len(packets)))
        self.source_pub.publish(s)
        self.sources.append((int(s.sequence),int(s.issued_monotonic_ns),time.monotonic_ns(),float(now),float(joint_stamp)))
        self._evidence({'kind':'feedback','value':{'sequence':int(s.sequence),'sim_s':float(now),'t_s':float(t),
            'phase':'measure','feedback':loop.feedback_diagnostics}})
        self.last_source=now
        return True

    def _check_supervision(self):
        if self.supervisor is not None:
            error=self.supervisor.poll()
            if error:
                if self.failure is None:self.failure=error
                raise RuntimeError(error)

    def _close_supervision(self):
        if self.supervisor is not None:
            self.supervisor.close()
            write(self.out/'fault_supervision.json',self.supervisor.latch.report())

    def _pump_recovery_receive_only(self):
        """Receive navigation/ACK evidence without either actuator writer."""
        loop=self.loop
        self.loop_timing.original['original_pump'](ros_timeout_s=0.)
        until=time.monotonic_ns()+250_000
        for _ in range(32):
            if time.monotonic_ns()>=until:break
            loop.ctx.node.spin_once(timeout_sec=0.)
        sim=loop.monitor_now()
        loop.land_watchdog.observe(loop.ctx.node,sim)
        if loop.status_watchdog is not None:
            loop.status_watchdog.observe(loop.ctx.node,sim)

    def _release_for_failed_recovery(self,loop):
        # An unconfirmed exit must never enable the old joint/PVA publisher.
        # NAV mode commands can still be sent while this receive-only pump
        # obtains their confirmation. No stale MPC command is republished.
        self.recovery_owner_unverified=True
        loop.ctx.vehicle._pump=self._pump_recovery_receive_only
        record={'zero_consumed':False,'writer_exited':False,'bounded_failed_recovery':True,
                'receipt_budget_s':.15,'sigint_budget_s':.15,'sigkill_budget_s':.05,
                'signals':[],'errors':[],'begin_monotonic_ns':time.monotonic_ns()}
        self.release=record
        try:
            stop=self.msg.RuntimeStop();stop.session_id=self.session;stop.epoch=self.epoch
            self.stop_pub.publish(stop)
            deadline=time.monotonic()+.15
            while time.monotonic()<deadline:
                self._pump_recovery_receive_only()
                sent,receipt=self.latest.get('sends'),self.latest.get('receipts')
                if (sent is not None and receipt is not None and sent.stop_zero
                        and sent.session_id==receipt.session_id==self.session
                        and sent.epoch==receipt.epoch==self.epoch
                        and receipt.dispatch_sequence==sent.dispatch_sequence
                        and receipt.command_sequence==sent.command_sequence
                        and list(sent.joint_velocity)==[0.,0.]
                        and list(receipt.consumed_counter)==list(sent.expected_counter)
                        and receipt.joint_consumed and not receipt.pva_send_returned and not receipt.failed):
                    record['zero_consumed']=True;break
                time.sleep(.001)
        except Exception as exc:
            record['errors'].append('zero receipt: '+repr(exc))
        process=self.procs.get('dispatcher')
        try:
            if process is not None and process.poll() is None:
                os.killpg(process.pid,signal.SIGINT);record['signals'].append('SIGINT')
                try:process.wait(timeout=.15)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid,signal.SIGKILL);record['signals'].append('SIGKILL')
                    process.wait(timeout=.05)
        except Exception as exc:
            record['errors'].append('writer exit: '+repr(exc))
        record['writer_exited']=process is not None and process.poll() is not None
        record['wall_monotonic_ns']=time.monotonic_ns()
        self.active=False
        self.recovery_owner_unverified=not record['writer_exited']
        if record['writer_exited']:
            loop.last=None;loop.next_control=loop.control_now()
            loop.ctx.vehicle._pump=loop.pump
        if not record['zero_consumed'] or not record['writer_exited']:
            self.failure=self.failure or 'Failed-recovery output release was not fully verified'

    @staticmethod
    def _recovery_status(node):
        status=node.data.get('status')
        if status is None:return None
        return {'callback_sequence':int(node.callback_sequences.get('status',0)),
                'received_wall_monotonic_s':shared.receipt_time(node,'status'),
                'timestamp_us':int(status.timestamp),'nav_state':int(status.nav_state),
                'arming_state':int(status.arming_state),'failsafe':bool(status.failsafe)}

    def before_failed_fold(self,ctx,loop,meta):
        """Confirm PX4 autonomous hold before waiting for arm recovery."""
        record={'kind':'isolated_failed_recovery_handoff_v1','confirmed':False,
                'decision':'direct_land','confirmation_budget_s':.35,
                'command':176,'parameters':[1.,4.,3.,0.,0.,0.,0.],
                'expected_nav_state':4,'acks':[],
                'clock_semantics':'host monotonic callback/ACK receipts; PX4 timestamp only compared for source progression',
                'source_semantics':'VehicleAPI._pump accepts ACK only from its fixed PX4 target',
                'scope':'failed experiment recovery only; no MPC continuation or hardware qualification'}
        meta['failed_recovery_handoff']=record
        loop.recovery=True
        loop.offboard=loop.send_reference=loop.strict=False
        if not self.started:
            record['reason']='live_control_never_started'
            return False
        try:
            if self.active:self._release(loop,bounded_failed_recovery=True)
            record['release']=dict(getattr(self,'release',{}))
            if not record['release'].get('writer_exited') or not record['release'].get('zero_consumed'):
                record['reason']='writer_exit_or_zero_consumption_unverified'
                return False
            before=self._recovery_status(ctx.node)
            target=tuple(ctx.vehicle.target or ())
            if before is None or before['callback_sequence']<=0 or before['timestamp_us']<=0 or len(target)!=2:
                record['reason']='missing_initial_status_or_target'
                return False
            record.update(before=before,target_system=int(target[0]),target_component=int(target[1]))
            # Clear the old Offboard DO_SET_MODE ACK before this new request.
            ctx.vehicle.ack.pop(176,None)
            boundary=time.monotonic()
            record['command_boundary_wall_monotonic_s']=boundary
            ctx.vehicle._send_command(176,record['parameters'])
            record['command_send_returned_wall_monotonic_s']=time.monotonic()
            deadline=boundary+.35
            last_ack=None;accepted=False
            while time.monotonic()<deadline:
                self._pump_recovery_receive_only()
                observed=time.monotonic()
                if observed>=deadline:break
                if tuple(ctx.vehicle.target or ())!=target:
                    record['reason']='px4_target_changed';return False
                ack=ctx.vehicle.ack.get(176)
                if ack is not None and ack!=last_ack:
                    last_ack=ack
                    record['acks'].append({'command':176,'received_wall_monotonic_s':float(ack[0]),
                        'result':int(ack[1]),'target_system':target[0],'target_component':target[1]})
                    if boundary<=float(ack[0])<=observed:
                        if int(ack[1]) not in (0,5):
                            record['reason']='mode_command_rejected';return False
                        accepted=True
                current=self._recovery_status(ctx.node)
                record['last_status']=current
                if (accepted and current is not None
                        and current['callback_sequence']>before['callback_sequence']
                        and boundary<current['received_wall_monotonic_s']<=observed
                        and current['timestamp_us']>before['timestamp_us']
                        and current['nav_state']==4 and current['arming_state']==2 and not current['failsafe']):
                    record.update(confirmed=True,decision='fold_then_land',reason='new_ack_and_autonomous_hold_state',
                                  confirmed_wall_monotonic_s=observed)
                    return True
                time.sleep(.001)
            record['reason']='mode_confirmation_timeout'
        except Exception as exc:
            record['reason']='handoff_exception';record['error']=repr(exc)
        finally:
            record['finished_wall_monotonic_s']=time.monotonic()
        return False

    def land_after_failed_recovery(self,ctx,loop,meta):
        if not getattr(self,'recovery_owner_unverified',False):
            return ctx.vehicle.land()
        # VehicleAPI.land normally starts with zero_all(). If writer death is
        # unverified that would create a second joint writer. Suppress only
        # this convenience publish; keep the original NAV_LAND/ACK/state path.
        loop.ctx.vehicle._pump=self._pump_recovery_receive_only
        original_zero=ctx.node.zero_all
        def suppressed_zero():
            meta['unverified_owner_land_zero_publish_suppressed']=True
        ctx.node.zero_all=suppressed_zero
        try:return ctx.vehicle.land()
        finally:ctx.node.zero_all=original_zero

    def _release(self,loop,*,bounded_failed_recovery=False):
        if bounded_failed_recovery:return self._release_for_failed_recovery(loop)
        if not self.active:return
        stop=self.msg.RuntimeStop();stop.session_id=self.session;stop.epoch=1
        self.stop_pub.publish(stop)
        deadline=time.monotonic()+1.
        released=False
        while time.monotonic()<deadline:
            for _ in range(24):loop.ctx.node.spin_once(timeout_sec=0.)
            sent,receipt=self.latest.get('sends'),self.latest.get('receipts')
            if sent is not None and sent.stop_zero and receipt is not None and receipt.dispatch_sequence==sent.dispatch_sequence and receipt.joint_consumed:
                released=True;break
            time.sleep(.001)
        # Stop the only new writer before allowing the legacy recovery owner.
        self._stop_process('dispatcher')
        self.active=False
        loop.ctx.vehicle._pump=loop.pump
        loop.last=None;loop.next_control=loop.control_now()
        self.release={'zero_consumed':released,'writer_exited':self.procs['dispatcher'].poll() is not None,
                      'wall_monotonic_ns':time.monotonic_ns()}
        if not released:self.failure='C++ writer stopped, but zero-consumption receipt missing'

    def _stop_process(self,name):
        p=self.procs.get(name)
        if p is None or p.poll() is not None:return
        os.killpg(p.pid,signal.SIGINT)
        try:p.wait(timeout=10.)
        except subprocess.TimeoutExpired:
            os.killpg(p.pid,signal.SIGTERM)
            try:p.wait(timeout=3.)
            except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGKILL);p.wait()

    def finalize(self,meta,stage,out):
        if self.active:self._release(self.loop)
        self._close_supervision()
        for name in ('ingress','worker','dispatcher','consumption','recorder'):self._stop_process(name)
        if self.source_sender is not None:
            self.source_sender.close(self.out/'source_sender.json');self.source_sender=None
        self._stop_process('source_recorder')
        for f in self.logs:f.close()
        from rosidl_runtime_py.convert import message_to_ordereddict
        if self.started:write(self.out/'live_start.json',message_to_ordereddict(self.start_message))
        write(self.out/'source_timing.json',{'columns':['sequence','issued_ns','publish_return_ns','control_s','joint_s'],'rows':self.sources})
        from rclpy.serialization import deserialize_message
        from simulations.x650_isolated_live_v1.source_recording import materialize
        try:
            materialize(self.out,self.message_types,deserialize_message,message_to_ordereddict,write)
        except Exception as error:
            self.failure='Source recording reconciliation failed: '+repr(error)
        write(self.out/'source_pump_timing.json',{'columns':['begin_ns','drained_ns','end_ns','extra_callbacks'],'rows':self.pump_times})
        report={'started':self.started,'release':getattr(self,'release',None),'error':self.failure,
            'processes':{name:{'pid':p.pid,'returncode':p.returncode} for name,p in self.procs.items()},
            'source_samples':len(self.sources),'physical_acceptance_pending':True}
        write(self.out/'lifecycle.json',report)
        meta['isolated_runtime']=report
        if self.failure:meta['complete']=False;stage.update(success=False,complete=False,passed=False)
        super().finalize(meta,stage,out)

    def cleanup(self):
        # Also reached when startup fails before shared.execute can finalize.
        if self.source_sender is not None:
            self.source_sender.close(self.out/'source_sender.json');self.source_sender=None
        for name in ('dispatcher','consumption','ingress','worker','recorder','source_recorder'):self._stop_process(name)
        for f in self.logs:
            if not f.closed:f.close()
        # A report-write failure must not prevent stopping owned processes.
        self._close_supervision()


def scenario():
    if os.environ.get('X650_EXECUTION_ADAPTER')!='isolated_simulation_v1':
        raise ValueError('Explicit simulation execution selection required')
    if os.environ.get('AMSIM_SCENARIO_NAME')!='x650_isolated_live_v1':
        raise ValueError('Dedicated live platform scenario required')
    cfg=task.selected_task_config()
    model,provenance=task.preflight(cfg)
    out=Path(os.environ['AMSIM_RUN_DIR']).resolve()
    extension=LiveExtension(cfg,provenance)
    try:
        extension.prepare(out)
        return shared.execute(cfg,model,provenance,out,extension=extension)
    finally:
        if hasattr(extension,'procs'):extension.cleanup()


if __name__=='__main__':scenario()
