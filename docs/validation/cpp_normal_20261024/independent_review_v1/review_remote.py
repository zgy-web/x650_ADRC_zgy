"""Read-only, compact independent review of one original normal execution."""
from pathlib import Path
import base64,csv,hashlib,json,os,sys
import numpy as np

R=Path('/root/autodl-tmp/safe-aerial-manipulator-testbed')
L=R/'runs/x650_cpp_normal_launch_20261024_172532_46633'
D=R/'runs/x650_cpp_normal_v1_20261024_172622_46636'
B=R/'artifacts/cpp_normal_20261024'
sys.path[:0]=[str(R),str(R/'scripts')]
os.sched_setaffinity(0,{94})
def read(path):return json.loads(path.read_text())
def ident(path):
    b=path.read_bytes()
    return dict(path=str(path),bytes=len(b),sha256=hashlib.sha256(b).hexdigest())
live=D/'isolated_live';p=B/'normal_audit_v1/report.json';report=read(p)
launch,meta,stage,start,life,worker,sender,source=[read(x) for x in (
    L/'launch.json',D/'controller_metadata.json',D/'stage_result.json',live/'live_start.json',
    live/'lifecycle.json',live/'worker_final.json',live/'diagnostic_sender.json',live/'cpp_source/source_final.json')]
assert report['passed'] and launch['exit_code']==0 and launch['runtime_closed']
assert meta['complete'] and meta['landed_and_disarmed'] and meta['diagnostic_parameters_restored']
assert meta['diagnostic_restore_pending'] is False and all(x['returncode']==0 for x in life['processes'].values())
assert stage['complete'] and stage['passed'] and stage['success'] and life['error'] is None
assert report['window_sim_s']==[start['origin_s'],start['origin_s']+32.]
assert launch['execution_scenario']=='x650_cpp_normal_v1' and launch['task']['stage']=='normal'
paths=[L/'launch.json',L/'task.json',L/'protocol.json',D/'controller_metadata.json',D/'stage_result.json',
    live/'live_start.json',live/'lifecycle.json',live/'worker_final.json',live/'diagnostic_sender.json',
    live/'cpp_source/source_final.json',live/'cpp_source/source_selections.csv',live/'worker_timing.json',
    D/'native_trace.csv',D/'ts20_state.csv',D/'ts20_packets.csv',D/'px4.ulg',D/'diagnostic_parameters_restored.json',
    live/'source_recording/diagnostics.jsonl',live/'source_recording/manifest.json']
identities=[ident(x) for x in paths]
assert all(report['artifacts'][x['path']]==x['sha256'] for x in identities)
with (D/'native_trace.csv').open(newline='') as f:
    physical=[r for r in csv.DictReader(f) if -1e-9<=float(r['sim_s'])-start['origin_s']<=32.+1e-9]
t=np.array([float(r['sim_s'])-start['origin_s'] for r in physical])
assert len(t)==8001 and np.all(np.diff(t)>0) and np.max(np.diff(t))<.00400001
assert all(r['valid_tip']=='1' for r in physical)
tip=np.array([[float(r['tip_'+a]) for a in 'xyz'] for r in physical])
# Independent vector expression of the original frozen C2 phase law. No fitting.
s=t-2.;u=np.clip(s/2.,0,1);v=np.clip((26.-s)/2.,0,1)
ramp=lambda x:2.5*x**4-3*x**5+x**6
phi=np.where(s<=0.,0.,np.where(s>=26.,4*np.pi,np.where(s<2.,np.pi/3*ramp(u),
    np.where(s>24.,4*np.pi-np.pi/3*ramp(v),np.pi/6*(s-1.)))))
target=np.array(start['target_position_enu'])+np.column_stack((.25*np.sin(phi),np.zeros(len(t)),.1*np.sin(2*phi)))
err2=np.sum((tip-target)**2,axis=1)
metrics={}
for name,lo,hi in [('endpoint',0.,32.),('tracking_endpoint',2.,28.),('tail_endpoint',28.,32.)]:
    mask=(t>=lo-1e-9)&(t<=hi+1e-9);tt=t[mask];ee=err2[mask]
    m=dict(samples=len(tt),duration_s=float(tt[-1]-tt[0]),rmse_m=float(np.sqrt(np.trapz(ee,tt)/(tt[-1]-tt[0]))),peak_m=float(np.sqrt(ee.max())))
    assert abs(m['rmse_m']-report[name]['position_rmse_m'])<1e-13 and abs(m['peak_m']-report[name]['position_max_m'])<1e-13
    metrics[name]=m
from audit_x650_streaming_consumption import original_messages
messages=original_messages(live)
sends=messages['sends'];normal=[x for x in sends if not x['stop_zero']];receipts=messages['receipts'];ticks=messages['dispatch'];events=messages['events']
assert len(sends)==1600 and len(normal)==1599 and len(receipts)==1600 and len(events)==1598
assert all(x['accepted'] for x in events) and not any(x['status']==3 for x in ticks)
with (D/'ts20_state.csv').open(newline='') as f:ts20=list(csv.DictReader(f))
adopt=[{},{}]
for r in ts20:
    for j in range(2):adopt[j].setdefault(int(r[f'upper_sequence{j+1}']),r)
joint_counts=[0,0];same_boundary=0
for sent,receipt in zip(sends,receipts):
    assert sent['dispatch_sequence']==receipt['dispatch_sequence'] and sent['expected_counter']==receipt['consumed_counter']
    at=[]
    for j in range(2):
        row=adopt[j][sent['expected_counter'][j]];at.append(float(row[f'receipt{j+1}_s']))
        assert abs(float(row[f'u{j+1}'])-sent['joint_velocity'][j])<=1e-12
        assert abs(at[-1]-receipt['consumed_sim_s'][j])<=1e-9
        joint_counts[j]+=1
    same_boundary+=int(abs(at[0]-at[1])<=1e-9)
assert sends[-1]['stop_zero'] and sends[-1]['joint_velocity']==[0.,0.]
normal_ns=np.array([x['sent_monotonic_ns'] for x in normal],dtype=np.int64)
actual_gap=float(np.max(np.diff(normal_ns))/1e6)
assert abs(actual_gap-report['timing']['actual_send_gap_ms']['maximum'])<1e-12
phase=read(live/'worker_timing.json');pp=[dict(zip(phase['columns'],r)) for r in phase['rows']]
assert len(pp)==worker['requests']==worker['accepted']==sender['written']==1598 and sender['dropped']==0
core_ms=np.array([(r['core_end_ns']-r['core_begin_ns'])/1e6 for r in pp]);callback_ms=np.array([(r['callback_exit_ns']-r['callback_entry_ns'])/1e6 for r in pp])
assert abs(float(core_ms.max())-report['worker']['durations']['core_ms']['maximum'])<1e-12
with (live/'cpp_source/source_selections.csv').open(newline='') as f:selections=list(csv.DictReader(f))
assert len(selections)==report['feedback']['source_snapshots']
assert report['feedback']['original_worker_snapshots_verified']==len(pp) and report['feedback']['clean_source_stop']
assert [int(v['sequence']) for v in selections]==list(range(1,len(selections)+1))
sys.path.insert(0,'/root/autodl-tmp/amsim-venv/lib/python3.10/site-packages')
from pyulog import ULog
ulog=ULog(str(D/'px4.ulg'),message_name_filter_list=['vehicle_status','vehicle_land_detected'])
status=next(v.data for v in ulog.data_list if v.name=='vehicle_status')
land=next(v.data for v in ulog.data_list if v.name=='vehicle_land_detected')
st=status['timestamp']/1e6;lt=land['timestamp']/1e6
flight=(st>=start['origin_s'])
assert status['arming_state'][-1]==1 and land['landed'][-1]==1 and st[-1]>start['origin_s']+32
assert not np.any(status['failsafe'][flight])
review=dict(kind='independent_cpp_normal_review_v1',passed=True,scope='Independent original-byte, metric, CDR-count, physics-adoption and ULog lifecycle cross-check; one simulated run, no hardware claim',
    report=ident(p),launch=str(L),run=str(D),window_sim_s=report['window_sim_s'],trajectory=report['figure8_protocol']['trajectory'],
    metrics=metrics,source=dict(snapshots=len(selections),worker_link_count=len(pp),clean_stop=True),
    worker=dict(calls=len(pp),accepted=worker['accepted'],core_median_ms=float(np.median(core_ms)),core_max_ms=float(core_ms.max()),callback_max_ms=float(callback_ms.max())),
    execution=dict(normal_sends=len(normal),all_sends=len(sends),receipts=len(receipts),stop_zero=1,repeated_normal=sum(bool(x['repeated']) for x in normal),
        joint_adopt_counts=joint_counts,same_physics_boundary=same_boundary,gate_events=len(events),dispatch_ticks=len(ticks),faults=0,max_send_gap_ms=actual_gap),
    recovery=dict(runtime_closed=True,landed_and_disarmed=True,parameters_restored=True,child_exit_codes={k:v['returncode'] for k,v in life['processes'].items()},
        last_status_s=float(st[-1]),last_land_s=float(lt[-1]),failsafe_samples_active_and_recovery=int(np.count_nonzero(status['failsafe'][flight]))),
    limitations=['Precise simulated EV localization; not GPS or hardware accuracy',
        'No per-publication PX4 acknowledgement is inferred from ULog value matching',
        'Float64-to-counter adoption mapping uses declared exclusive ordered bridge assumption',
        'Only 1598/1600 paired commands share exactly the same physics adoption boundary',
        'C++ compute maximum above 20ms is allowed by latest-valid periodic output within unchanged 50ms age budget; no worst-case guarantee'],
    original_identities=identities)
print(json.dumps(dict(review=review,original_report_base64=base64.b64encode(p.read_bytes()).decode())))
