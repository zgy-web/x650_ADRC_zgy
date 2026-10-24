"""Bounded original-input offline replay; no ROS node or actuator output."""
from pathlib import Path
from types import SimpleNamespace
import argparse,base64,cProfile,gc,hashlib,json,os,pstats,sys,time
import numpy as np
from rclpy.serialization import deserialize_message
from flyarm_control_interfaces.msg import ControlSnapshot,ControlProposal

parser=argparse.ArgumentParser();parser.add_argument('--mode',choices=('phases','profile'),required=True)
parser.add_argument('--output',type=Path,required=True);args=parser.parse_args()
args.output.mkdir(parents=True,exist_ok=False)
R=Path('/root/autodl-tmp/safe-aerial-manipulator-testbed')
sys.path[:0]=[str(R),str(R/'ros2_ws/src/flyarm_mpc_reference'),
              str(R/'control_packages/x650_wholebody_v1')]
from flyarm_mpc_reference.live_factory import create_live_engine
from flyarm_mpc_reference.diagnostic_channel import CaptureAdapter
from flyarm_mpc_reference import engine as engine_module
from x650_wholebody import adapter as adapter_module
from x650_wholebody.api import ControlCommand

live=R/'runs/x650_isolated_live_v1_20261024_121409_13665/isolated_live'
task=R/'runs/x650_isolated_launch_20261024_121323_13664/task.json'
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
start=json.loads((live/'live_start.json').read_text());calls=[]
for line in (live/'recording/diagnostics.jsonl').read_text().splitlines():
    d=json.loads(line)
    calls.append((deserialize_message(base64.b64decode(d['snapshot_cdr']['base64']),ControlSnapshot),
                  deserialize_message(base64.b64decode(d['proposal_cdr']['base64']),ControlProposal)))
e=create_live_engine(R,task,sha(task),live/'initialization.json',sha(live/'initialization.json'),
                     start['session_id'],start['epoch'])
e.begin_live(SimpleNamespace(**start));capture=CaptureAdapter(e.adapter);e.adapter=capture
adapter=capture.adapter;allocator=adapter.allocator;kernel=allocator.native_optimization
totals={}

def wrap(obj,name,label,*,as_static=False):
    original=getattr(obj,name)
    def measure(*a,**kw):
        begin=time.perf_counter_ns();cpu=time.thread_time_ns()
        try:return original(*a,**kw)
        finally:
            row=totals.setdefault(label,[0.,0.,0])
            row[0]+=(time.perf_counter_ns()-begin)/1e6
            row[1]+=(time.thread_time_ns()-cpu)/1e6;row[2]+=1
    setattr(obj,name,staticmethod(measure) if as_static else measure)

for obj,name,label in ((e,'_decode','engine_decode'),(e,'_empty_proposal','empty_ros_proposal'),
                      (e,'state_factory','state_construction'),(e,'target_factory','target_construction'),
                      (e.context,'bind','context_bind'),(e.context,'clear','context_clear'),
                      (adapter,'step','adapter_step'),(allocator,'step','allocator_outer'),
                      (adapter,'_snapshot_diagnostics','owned_diagnostic_clone'),
                      (adapter,'observe','motion_filter'),(adapter,'_motion_sample','motion_validation'),
                      (adapter_module,'arm_reaction_feedforward','arm_reaction_feedforward'),
                      (adapter_module,'clip_norm','feedforward_saturation'),
                      (kernel,'assemble','assemble'),(kernel,'solve','qp'),
                      (allocator,'_validate_candidate','candidate_validation'),
                      (engine_module,'vector','engine_vector_conversion'),
                      (engine_module,'finite_scalar','engine_scalar_validation')):
    wrap(obj,name,label)
wrap(ControlCommand,'from_executed_velocity','command_construction',as_static=True)

gc.collect(2);time.sleep(.2)
rows=[];next_due=time.monotonic();fields=('position_reference_enu','velocity_feedforward_enu',
    'acceleration_feedforward_enu','base_velocity_target','joint_velocity')
for i,(snapshot,original) in enumerate(calls,1):
    remaining=next_due-time.monotonic()
    if remaining>0:time.sleep(remaining)
    next_due+=.02
    snapshot.issued_monotonic_ns=time.monotonic_ns()
    totals.clear();cpu=time.thread_time_ns();wall=time.perf_counter_ns()
    profile=cProfile.Profile() if args.mode=='profile' and i>=25 else None
    if profile:profile.enable()
    result=e.process(snapshot,ControlProposal)
    if profile:profile.disable()
    row={'sequence':i,'status':int(result.status),
         'engine_wall_ms':(result.compute_end_monotonic_ns-result.compute_start_monotonic_ns)/1e6,
         'call_wall_ms':(time.perf_counter_ns()-wall)/1e6,
         'call_thread_cpu_ms':(time.thread_time_ns()-cpu)/1e6,
         'phases_wall_cpu_count':dict(totals),
         'command_max_abs_error':max(float(np.max(abs(np.asarray(getattr(result,f))-getattr(original,f))))
                                     for f in fields),
         'allocator_timing':capture.diagnostic.get('ts20_timing_observation') if capture.diagnostic else None}
    if profile:
        profile.dump_stats(str(args.output/f'round_{i}.prof'))
        functions=[]
        for (file,line,name),(cc,nc,tt,ct,callers) in pstats.Stats(profile).stats.items():
            functions.append({'file':file,'line':line,'name':name,'calls':nc,'self_ms':tt*1000,'cumulative_ms':ct*1000})
        row['top_self']=sorted(functions,key=lambda x:x['self_ms'],reverse=True)[:30]
        row['top_cumulative']=sorted(functions,key=lambda x:x['cumulative_ms'],reverse=True)[:35]
    rows.append(row)
    if result.status:row['error']=e.last_error;break

report={'kind':'hold23_original_input_engine_profile_v1','physical_run':False,'mode':args.mode,
    'instrumentation':'phase wrappers measure wall/thread CPU; cProfile only rounds25/26 in profile mode; no timing acceptance claim',
    'input_policy':'all 26 original CDR snapshots replayed sequentially from actual LiveStart, only wall issue time rebased',
    'pacing':'20 ms absolute wall schedule after constructor/one preflight gen2 collection and .2 s idle; no cadence reset on overrun',
    'affinity':sorted(os.sched_getaffinity(0)),
    'environment':{key:os.environ.get(key) for key in ('OPENBLAS_NUM_THREADS','OMP_NUM_THREADS','MKL_NUM_THREADS')},
    'sources':{str(p):sha(p) for p in (Path(adapter_module.__file__),Path(engine_module.__file__),
        R/'control_packages/x650_wholebody_v1/x650_wholebody/native_joint_prediction.py',
        R/'control_packages/x650_wholebody_v1/x650_wholebody/native_optimization.py')},
    'inputs':{str(p):sha(p) for p in (task,live/'live_start.json',live/'initialization.json',live/'recording/diagnostics.jsonl')},
    'rows':rows}
(args.output/'report.json').write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps({'report':str(args.output/'report.json'),'count':len(rows),
    'max_error':max(x['command_max_abs_error'] for x in rows),'tail':rows[-2:]},indent=2))
