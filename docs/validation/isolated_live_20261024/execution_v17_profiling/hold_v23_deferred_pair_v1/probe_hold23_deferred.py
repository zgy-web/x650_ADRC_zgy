"""Scratch-only paired preparation timing, preserving the qualified live constructor."""
from pathlib import Path
from types import SimpleNamespace
import argparse,base64,gc,hashlib,importlib.util,json,os,socket,sys,tempfile,time
import numpy as np
from rclpy.serialization import deserialize_message,serialize_message
from flyarm_control_interfaces.msg import ControlSnapshot,ControlProposal

p=argparse.ArgumentParser();p.add_argument('--variant',choices=('packed','packed_deferred'),required=True)
p.add_argument('--output',type=Path,required=True);p.add_argument('--scratch',type=Path,required=True)
p.add_argument('--deferred',type=Path,required=True)
args=p.parse_args();args.output.mkdir(parents=True,exist_ok=False)
R=Path('/root/autodl-tmp/safe-aerial-manipulator-testbed')
sys.path[:0]=[str(R),str(R/'ros2_ws/src/flyarm_mpc_reference'),str(R/'control_packages/x650_wholebody_v1')]
from flyarm_mpc_reference.live_factory import create_live_engine
from flyarm_mpc_reference.diagnostic_channel import CaptureAdapter,DiagnosticSender
from flyarm_mpc_reference.isolated_node import process_snapshot,WorkerTimingTrace
from x650_wholebody import adapter as adapter_module

live=R/'runs/x650_isolated_live_v1_20261024_121409_13665/isolated_live'
task=R/'runs/x650_isolated_launch_20261024_121323_13664/task.json'
sha=lambda path:hashlib.sha256(path.read_bytes()).hexdigest()
start=json.loads((live/'live_start.json').read_text());calls=[]
for line in (live/'recording/diagnostics.jsonl').read_text().splitlines():
    d=json.loads(line)
    calls.append((deserialize_message(base64.b64decode(d['snapshot_cdr']['base64']),ControlSnapshot),
                  deserialize_message(base64.b64decode(d['proposal_cdr']['base64']),ControlProposal)))
e=create_live_engine(R,task,sha(task),live/'initialization.json',sha(live/'initialization.json'),start['session_id'],start['epoch'])
e.begin_live(SimpleNamespace(**start))
# All real task/source/preflight checks above run against the qualified source.
# The following in-memory class replacement is explicitly NOT qualification.
if args.variant.startswith('packed'):
    def load(name):
        qualified='x650_wholebody._scratch_'+name
        spec=importlib.util.spec_from_file_location(qualified,args.scratch/(name+'.py'))
        module=importlib.util.module_from_spec(spec);sys.modules[qualified]=module;spec.loader.exec_module(module)
        return module
    joint=load('native_joint_prediction');opt=load('native_optimization');mpc=load('native_mpc')
    mpc.NativeJointMatrices=joint.NativeJointMatrices;mpc.NativeOptimization=opt.NativeOptimization
    allocator=e.adapter.allocator
    allocator.__class__=mpc.NativeTs20MpcController
    allocator.native_optimization.__class__=opt.NativeOptimization
    allocator.actuator_predictor.__class__=joint.NativeJointMatrices

adapter=e.adapter
deferred=None
if args.variant=='packed_deferred':
    spec=importlib.util.spec_from_file_location('_scratch_deferred',args.deferred)
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    deferred=module.DeferredLiveDiagnostics(adapter);e.adapter=deferred
capture=CaptureAdapter(e.adapter);e.adapter=capture
allocator=adapter.allocator;kernel=allocator.native_optimization
totals={}
def wrap(obj,name,label):
    original=getattr(obj,name)
    def measured(*a,**kw):
        wall=time.perf_counter_ns();cpu=time.thread_time_ns()
        try:return original(*a,**kw)
        finally:
            v=totals.setdefault(label,[0.,0.,0]);v[0]+=(time.perf_counter_ns()-wall)/1e6
            v[1]+=(time.thread_time_ns()-cpu)/1e6;v[2]+=1
    setattr(obj,name,measured)
for obj,name,label in ((e,'_decode','decode'),(adapter,'step','adapter'),(allocator,'step','allocator'),
                      (adapter,'_snapshot_diagnostics','clone'),(adapter_module,'arm_reaction_feedforward','arm_ff'),
                      (kernel,'assemble','assemble'),(kernel,'solve','qp'),
                      (allocator,'_joint_prediction','joint_prediction'),(allocator,'_validate_candidate','candidate')):
    wrap(obj,name,label)
if deferred is not None:
    wrap(deferred,'step','deferred_step');wrap(deferred,'finish_diagnostics','finish_diagnostics')

def plain(value):
    if isinstance(value,np.ndarray):return value.tolist()
    if isinstance(value,np.generic):return value.item()
    if isinstance(value,dict):return {str(k):plain(v) for k,v in value.items()}
    if isinstance(value,(tuple,list)):return [plain(v) for v in value]
    return value

class Publisher:
    def publish(self,proposal):self.proposal=proposal

fields=('position_reference_enu','velocity_feedforward_enu','acceleration_feedforward_enu','base_velocity_target','joint_velocity')
numeric_keys=('predicted_states','predicted_commands','predicted_tip_positions','predicted_tip_error_norm',
    'predicted_rotations_flu_to_enu','ts20_candidate_envelope','ts20_candidate_q_linearization_error_rad',
    'ts20_candidate_dq_linearization_error_rad_s','ts20_candidate_actual_bounds_passed',
    'ts20_nonlinear_check_passed','primal_violation','kkt_stationarity_inf','objective',
    'ts20_relinearization_attempt_count','ts20_relinearization_attempts')
rows=[];numeric=[];publisher=Publisher();trace=WorkerTimingTrace(capacity=100)
def serialize_after_publish(message):
    # process_snapshot invokes this only after publisher.publish returned.
    # Complete all diagnostics before its normal synchronous pickle handoff.
    if isinstance(message,ControlSnapshot) and deferred is not None:
        capture.diagnostic=deferred.finish_diagnostics()
    return serialize_message(message)
with tempfile.TemporaryDirectory(prefix='flyarm-packed-') as td:
    socket_path=str(Path(td)/'d.sock');receiver=socket.socket(socket.AF_UNIX,socket.SOCK_DGRAM)
    receiver.setsockopt(socket.SOL_SOCKET,socket.SO_RCVBUF,393216);receiver.bind(socket_path)
    sender=DiagnosticSender(socket_path)
    gc.collect(2);time.sleep(.2);due=time.monotonic()
    for index,(snapshot,original) in enumerate(calls,1):
        remaining=due-time.monotonic()
        if remaining>0:time.sleep(remaining)
        due+=.02;snapshot.issued_monotonic_ns=time.monotonic_ns();totals.clear()
        process_snapshot(e,capture,publisher,sender,snapshot,ControlProposal,serialize_after_publish,trace)
        result=publisher.proposal
        receiver.recv(196608)
        times=dict(zip(trace.columns,trace.rows[-1]))
        d=capture.diagnostic or {}
        rows.append({'sequence':index,'status':int(result.status),
            'engine_wall_ms':(result.compute_end_monotonic_ns-result.compute_start_monotonic_ns)/1e6,
            'callback_proxy_wall_ms':(times['callback_exit_ns']-times['callback_entry_ns'])/1e6,
            'callback_proxy_thread_cpu_ms':(times['callback_thread_cpu_end_ns']-times['callback_thread_cpu_begin_ns'])/1e6,
            'phases_wall_cpu_count':dict(totals),
            'command_max_abs_error':max(float(np.max(abs(np.asarray(getattr(result,k))-getattr(original,k)))) for k in fields),
            'allocator_timing':plain(d.get('ts20_timing_observation'))})
        numeric.append({'sequence':index,'command':{k:list(getattr(result,k)) for k in fields},
                        'diagnostics':plain(d)})
        if result.status:rows[-1]['error']=e.last_error;break
    sender.close(args.output/'diagnostic_sender.json');receiver.close()
trace.write(args.output/'worker_timing.json')
source_paths=[Path(adapter_module.__file__),R/'ros2_ws/src/flyarm_mpc_reference/flyarm_mpc_reference/engine.py']
source_paths += [args.scratch/(n+'.py')
                 for n in ('native_joint_prediction','native_optimization','native_mpc')]
source_paths.append(args.deferred)
report={'kind':'hold23_scratch_deferred_offline_pair_v1','variant':args.variant,'physical_run':False,'qualification':False,
    'injection':'Original constructor/begin_live validations finish, then scratch classes replace only three numerical objects in memory; no constructor/model/parameter changes',
    'timing':'Phase-only, no cProfile; real process_snapshot, ROS CDR and synchronous Unix diagnostic send; no-op publisher excludes DDS. In deferred variant finish_diagnostics executes on first serializer entry, after publish returns and before any CDR/pickle.',
    'pacing':'All 26 original snapshots in order, new process each pass, one preflight gen2 collection then .2 s idle; absolute 20 ms schedule without reset; only wall issue rebased',
    'affinity':sorted(os.sched_getaffinity(0)),
    'environment':{k:os.environ.get(k) for k in ('OPENBLAS_NUM_THREADS','OMP_NUM_THREADS','MKL_NUM_THREADS')},
    'sources':{str(path):sha(path) for path in source_paths},
    'inputs':{str(path):sha(path) for path in (task,live/'initialization.json',live/'live_start.json',live/'recording/diagnostics.jsonl')},
    'rows':rows}
(args.output/'report.json').write_text(json.dumps(report,indent=2)+'\n')
(args.output/'numeric.json').write_text(json.dumps(numeric,separators=(',',':'),allow_nan=False)+'\n')
print(json.dumps({'variant':args.variant,'report':str(args.output/'report.json'),'count':len(rows),
    'max_command_error':max(x['command_max_abs_error'] for x in rows),
    'engine_median_ms':float(np.median([x['engine_wall_ms'] for x in rows])),
    'engine_max_ms':max(x['engine_wall_ms'] for x in rows),
    'callback_median_ms':float(np.median([x['callback_proxy_wall_ms'] for x in rows])),
    'tail':rows[-2:]},indent=2))
