"""Original hold25 CDR paced software replay; no nodes, publishers or actuators.

All decoding and storage allocation precedes timing. The original 50 ms gate,
candidate checks, state history and physical dt remain unchanged. Wall issue
timestamps alone are rebased so this offline run can enter the existing gate.
"""
from pathlib import Path
from types import SimpleNamespace
import argparse,base64,ctypes,gc,hashlib,json,os,sys,time
import numpy as np

parser=argparse.ArgumentParser()
parser.add_argument('--backend',choices=('original','identity'),required=True)
parser.add_argument('--output',type=Path,required=True)
args=parser.parse_args();args.output.mkdir(parents=True,exist_ok=False)
R=Path('/root/autodl-tmp/safe-aerial-manipulator-testbed')
sys.path[:0]=[str(R),str(R/'ros2_ws/src/flyarm_mpc_reference'),str(R/'control_packages/x650_wholebody_v1')]
from rclpy.serialization import deserialize_message
from flyarm_control_interfaces.msg import ControlSnapshot,ControlProposal
from flyarm_mpc_reference.live_factory import create_live_engine
from flyarm_mpc_reference.diagnostic_channel import CaptureAdapter
from x650_wholebody.native_optimization import NativeOptimization

live=R/'runs/x650_isolated_live_v1_20261024_131702_20177/isolated_live'
task=R/'runs/x650_isolated_launch_20261024_131614_20176/task.json'
old_library=Path('/root/autodl-tmp/x650_native_optimization_build_v3/libx650_optimization.so')
new_library=R/'artifacts/isolated_live_20261024/identity_qp_scratch_v1/build/libx650_optimization.so'
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
fields=('position_reference_enu','velocity_feedforward_enu','acceleration_feedforward_enu','base_velocity_target','joint_velocity')
snapshots=[];expected=[]
with (live/'recording/diagnostics.jsonl').open() as stream:
    for line in stream:
        row=json.loads(line)
        snapshots.append(deserialize_message(base64.b64decode(row['snapshot_cdr']['base64']),ControlSnapshot))
        original=deserialize_message(base64.b64decode(row['proposal_cdr']['base64']),ControlProposal)
        expected.append(np.concatenate([np.asarray(getattr(original,key)) for key in fields]))
del row,line,original
expected=np.asarray(expected)
start=json.loads((live/'live_start.json').read_text())
engine=create_live_engine(R,task,sha(task),live/'initialization.json',sha(live/'initialization.json'),start['session_id'],start['epoch'])
engine.begin_live(SimpleNamespace(**start));capture=CaptureAdapter(engine.adapter);engine.adapter=capture
if args.backend=='identity':
    # Explicit scratch override after production identity/initialization checks.
    # It is a new implementation under test, never a rewritten task hash.
    native=NativeOptimization(new_library,sha(new_library))
    capture.adapter.allocator.native_optimization=native
    capture.adapter.allocator.actuator_predictor._affine_kernel=native
selected_library=old_library if args.backend=='original' else new_library
cpu=ctypes.CDLL(None).sched_getcpu;cpu.restype=ctypes.c_int
columns=('sequence','scheduled_start_ns','callback_entry_ns','engine_return_ns',
         'diagnostic_begin_ns','diagnostic_end_ns','callback_cpu_begin_ns','engine_cpu_end_ns',
         'diagnostic_cpu_end_ns','cpu_begin','cpu_after_engine','cpu_after_diagnostic',
         'lateness_ns','original_issue_ns','rebased_issue_ns','proposal_compute_start_ns',
         'proposal_compute_end_ns','status')
trace=np.zeros((len(snapshots),len(columns)),dtype=np.uint64)
errors=np.zeros(len(snapshots));completed=0;failure=None
gc_before={'enabled':gc.isenabled(),'thresholds':gc.get_threshold(),'counts':gc.get_count(),
           'collections':[s['collections'] for s in gc.get_stats()]}
# Only touching the already allocated arrays; no collect/disable/threshold change.
trace.fill(0);errors.fill(0)
period_ns=20_000_000;due=time.monotonic_ns()+500_000_000
for index,snapshot in enumerate(snapshots):
    wait_ns=due-time.monotonic_ns()
    if wait_ns>0:time.sleep(wait_ns/1e9)
    original_issue=int(snapshot.issued_monotonic_ns)
    c0=cpu();cb=time.thread_time_ns();begin=time.monotonic_ns()
    snapshot.issued_monotonic_ns=begin
    proposal=engine.process(snapshot,ControlProposal)
    end=time.monotonic_ns();ce=time.thread_time_ns();c1=cpu()
    diagnostic_begin=time.monotonic_ns()
    try:
        capture.complete_diagnostic(int(snapshot.sequence))
    except BaseException as error:
        failure=repr(error)
    diagnostic_end=time.monotonic_ns();cd=time.thread_time_ns();c2=cpu()
    trace[index]=(snapshot.sequence,due,begin,end,diagnostic_begin,diagnostic_end,
                  cb,ce,cd,c0,c1,c2,max(0,begin-due),original_issue,begin,
                  proposal.compute_start_monotonic_ns,proposal.compute_end_monotonic_ns,proposal.status)
    actual=np.concatenate([np.asarray(getattr(proposal,key)) for key in fields])
    errors[index]=np.max(np.abs(actual-expected[index]));completed=index+1
    due+=period_ns
    if proposal.status or failure:
        failure=failure or engine.last_error;break

gc_after={'enabled':gc.isenabled(),'thresholds':gc.get_threshold(),'counts':gc.get_count(),
          'collections':[s['collections'] for s in gc.get_stats()]}
used=trace[:completed];w=(used[:,3]-used[:,2])/1e6;c=(used[:,7]-used[:,6])/1e6
full=(used[:,5]-used[:,2])/1e6;diag=(used[:,5]-used[:,4])/1e6
def stats(values):
    return {'min_ms':float(np.min(values)),'median_ms':float(np.median(values)),
            'p95_ms':float(np.percentile(values,95)),'p99_ms':float(np.percentile(values,99)),
            'max_ms':float(np.max(values))}
report={'kind':'hold25_paced_predecoded_software_replay_v1','physical_run':False,
        'backend':args.backend,'completed':completed,'expected_count':len(snapshots),
        'all_accepted':completed==len(snapshots) and not failure,'failure':failure,
        'max_command_error':float(errors[:completed].max()),'affinity':sorted(os.sched_getaffinity(0)),
        'cpu_begin_values':sorted(set(map(int,used[:,9]))),'cpu_end_values':sorted(set(map(int,used[:,11]))),
        'library':{'path':str(selected_library),'sha256':sha(selected_library)},
        'environment':{k:os.environ.get(k) for k in ('OPENBLAS_NUM_THREADS','OMP_NUM_THREADS','MKL_NUM_THREADS')},
        'gc_before':gc_before,'gc_after':gc_after,
        'policy':'Predecoded original 1056 snapshots; original state, measured dt, causal history and original 50 ms gate. Wall issue rebased only. 20 ms absolute schedule after .5 s initial idle, no catch-up schedule reset. No phase wrappers, QP copy, cProfile, publisher, socket or file I/O inside engine interval. Full deferred diagnostic completion retained before next step. Command comparison outside timed engine interval. Natural GC unchanged.',
        'statistics':{'engine_wall':stats(w),'engine_thread_cpu':stats(c),'engine_plus_diagnostic_wall':stats(full),
                      'diagnostic_wall':stats(diag),'start_lateness':stats(used[:,12]/1e6)},
        'engine_above_20ms':int(np.sum(w>20)),'engine_plus_diagnostic_above_20ms':int(np.sum(full>20)),
        'columns':columns,'rows':used.tolist(),'command_max_errors':errors[:completed].tolist(),
        'sources':{str(p):sha(p) for p in (Path(__file__),task,live/'live_start.json',live/'initialization.json',live/'recording/diagnostics.jsonl')}}
(args.output/'report.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
print(json.dumps({k:report[k] for k in ('backend','completed','all_accepted','failure','max_command_error','affinity','statistics','engine_above_20ms')},indent=2))
