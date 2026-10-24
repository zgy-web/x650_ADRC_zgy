"""Read-only second review of one full fast execution; no fitting or simulation."""
from pathlib import Path
from collections import Counter
import bisect,csv,hashlib,json,os,sys,zipfile
import numpy as np

R=Path('/root/autodl-tmp/safe-aerial-manipulator-testbed')
L=R/'runs/x650_cpp_fast_launch_20261024_200610_52114'
D=R/'runs/x650_cpp_fast_v1_20261024_200653_52118'
B=R/'artifacts/cpp_fast_20261024'
O=B/'independent_review_v1'
sys.path[:0]=[str(R),str(R/'scripts')]
os.sched_setaffinity(0,{94})
def read(path):return json.loads(path.read_text())
def ident(path):
    data=path.read_bytes()
    return dict(path=str(path),bytes=len(data),sha256=hashlib.sha256(data).hexdigest())
def write(name,value):(O/name).write_text(json.dumps(value,indent=2,allow_nan=False)+'\n')

p=B/'fast_audit_v1/report.json';report=read(p);live=D/'isolated_live'
launch,meta,stage,start,life,worker,sender,source=[read(x) for x in (
    L/'launch.json',D/'controller_metadata.json',D/'stage_result.json',live/'live_start.json',
    live/'lifecycle.json',live/'worker_final.json',live/'diagnostic_sender.json',live/'cpp_source/source_final.json')]
assert all(report[k] is True for k in ('passed','complete','certificate','physical_acceptance'))
assert launch['exit_code']==0 and launch['runtime_closed'] and launch['complete']
assert meta['complete'] and meta['landed_and_disarmed'] and meta['diagnostic_parameters_restored']
assert meta['diagnostic_restore_pending'] is False and all(x['returncode']==0 for x in life['processes'].values())
assert stage['complete'] and stage['passed'] and stage['success'] and life['error'] is None
assert report['window_sim_s']==[start['origin_s'],start['origin_s']+24.]
assert abs(meta['task_end_sim_s']-start['origin_s']-24.)<=.05
assert launch['execution_scenario']=='x650_cpp_fast_v1' and launch['task']['stage']=='fast'
assert report['figure8_protocol']['trajectory']==dict(kind='fast_figure8',amplitude_x=.25,amplitude_z=.10,period_s=8.,cycles=2,start_s=2.,ramp_s=2.,duration_s=24.)
paths=[L/'launch.json',L/'task.json',L/'protocol.json',L/'sources.zip',L/'source_manifest.json',L/'source_manifest_after.json',
    L/'runtime_binaries.zip',L/'runtime_binary_manifest.json',L/'runtime_binary_manifest_after.json',
    D/'controller_metadata.json',D/'stage_result.json',live/'live_start.json',live/'lifecycle.json',
    live/'worker_final.json',live/'diagnostic_sender.json',live/'cpp_source/source_final.json',
    live/'cpp_source/source_selections.csv',live/'worker_timing.json',D/'native_trace.csv',D/'ts20_state.csv',
    D/'ts20_packets.csv',D/'px4.ulg',D/'diagnostic_parameters_restored.json',
    live/'source_recording/diagnostics.jsonl',live/'source_recording/manifest.json']
identities=[ident(x) for x in paths]
assert all(report['artifacts'][x['path']]==x['sha256'] for x in identities)
source_manifest=read(L/'source_manifest.json');assert source_manifest==read(L/'source_manifest_after.json')
with zipfile.ZipFile(L/'sources.zip') as archive:
    assert set(archive.namelist())==set(source_manifest)
    assert all(hashlib.sha256(archive.read(n)).hexdigest()==h for n,h in source_manifest.items())
binaries=read(L/'runtime_binary_manifest.json');assert {x['path']:x['sha256'] for x in binaries}==read(L/'runtime_binary_manifest_after.json')==launch['runtime_binaries_sha256']
with zipfile.ZipFile(L/'runtime_binaries.zip') as archive:
    assert set(archive.namelist())=={x['archive_entry'] for x in binaries}
    for x in binaries:
        data=archive.read(x['archive_entry']);assert len(data)==x['bytes'] and hashlib.sha256(data).hexdigest()==x['sha256']
with (D/'native_trace.csv').open(newline='') as f:
    physical=[r for r in csv.DictReader(f) if -1e-9<=float(r['sim_s'])-start['origin_s']<=24.+1e-9]
t=np.array([float(r['sim_s'])-start['origin_s'] for r in physical])
assert len(t)==6001 and np.all(np.diff(t)>0) and np.max(np.diff(t))<.00400001
assert all(r['valid_tip']=='1' and r['valid_base']=='1' and r['valid_joint_state']=='1' for r in physical)
tip=np.array([[float(r['tip_'+a]) for a in 'xyz'] for r in physical])
# Independent closed vector expression: 2 s start, 18 s moving, 4 s final hold.
# omega=2pi/8; the acceleration/deceleration phase ramps each last 2 s.
s=t-2.;u=np.clip(s/2.,0,1);v=np.clip((18.-s)/2.,0,1)
ramp=lambda x:2.5*x**4-3*x**5+x**6
phi=np.where(s<=0.,0.,np.where(s>=18.,4*np.pi,np.where(s<2.,np.pi/2*ramp(u),
    np.where(s>16.,4*np.pi-np.pi/2*ramp(v),np.pi/4*(s-1.)))))
target=np.array(start['target_position_enu'])+np.column_stack((.25*np.sin(phi),np.zeros(len(t)),.1*np.sin(2*phi)))
err2=np.sum((tip-target)**2,axis=1);metrics={}
for name,lo,hi in [('endpoint',0.,24.),('tracking_endpoint',2.,20.),('tail_endpoint',20.,24.)]:
    mask=(t>=lo-1e-9)&(t<=hi+1e-9);tt=t[mask];ee=err2[mask]
    m=dict(samples=len(tt),duration_s=float(tt[-1]-tt[0]),rmse_m=float(np.sqrt(np.trapz(ee,tt)/(tt[-1]-tt[0]))),peak_m=float(np.sqrt(ee.max())))
    assert abs(m['rmse_m']-report[name]['position_rmse_m'])<1e-13 and abs(m['peak_m']-report[name]['position_max_m'])<1e-13
    metrics[name]=m
q=np.array([[float(r['base_q'+a]) for a in 'wxyz'] for r in physical]);q/=np.linalg.norm(q,axis=1)[:,None]
tilt=float(np.degrees(np.arccos(np.clip(1-2*(q[:,1]**2+q[:,2]**2),-1,1))).max())
assert abs(tilt-report['tilt_peak_deg'])<1e-9
ranges={}
for key in ('base_px','base_py','base_pz','arm_joint_1_q','arm_joint_2_q','tip_x','tip_y','tip_z'):
    values=np.array([float(r[key]) for r in physical]);ranges[key]=dict(min=float(values.min()),max=float(values.max()),peak_to_peak=float(np.ptp(values)))
from audit_x650_streaming_consumption import original_messages
messages=original_messages(live);sends=messages['sends'];normal=[x for x in sends if not x['stop_zero']]
receipts=messages['receipts'];ticks=messages['dispatch'];events=messages['events']
assert len(sends)==report['consumption']['send_count'] and len(receipts)==report['consumption']['observed_receipt_count']
assert len(sends)==len(normal)+1 and len(normal)>1100 and all(x['accepted'] for x in events)
assert not any(x['status']==3 for x in ticks)
with (D/'ts20_state.csv').open(newline='') as f:ts20=list(csv.DictReader(f))
assert all(abs(float(b['interval_begin_s'])-float(a['sim_s']))<1e-9 for a,b in zip(ts20,ts20[1:]))
adopt=[{},{}];pairs={}
for index,r in enumerate(ts20):
    pairs.setdefault(tuple(int(r[f'upper_sequence{j+1}']) for j in range(2)),r)
    for j in range(2):adopt[j].setdefault(int(r[f'upper_sequence{j+1}']),(index,r))
counts=[Counter(),Counter()];same_boundary=0;adopt_order=[sorted(x) for x in adopt]
for sent in sends:
    ats=[]
    for j in range(2):
        counter=sent['expected_counter'][j]
        if counter in adopt[j]:
            _,row=adopt[j][counter];at=float(row[f'receipt{j+1}_s']);ats.append(at)
            assert abs(float(row[f'u{j+1}'])-sent['joint_velocity'][j])<=1e-12
            counts[j]['adopted']+=1
        else:
            pos=bisect.bisect_right(adopt_order[j],counter);assert pos<len(adopt_order[j])
            nextcounter=adopt_order[j][pos];index,row=adopt[j][nextcounter];assert index>0
            assert int(ts20[index-1][f'upper_sequence{j+1}'])<counter<nextcounter
            counts[j]['superseded_before_physics_adoption']+=1
    same_boundary+=int(len(ats)==2 and abs(ats[0]-ats[1])<=1e-9 and tuple(sent['expected_counter']) in pairs)
assert [dict(x) for x in counts]==report['consumption']['joint_counts']
assert same_boundary==report['consumption']['same_physics_boundary_adoptions']
assert report['consumption']['all_sends_applied'] is all(x['adopted']==len(sends) for x in counts)
by_dispatch={x['dispatch_sequence']:x for x in sends}
for receipt in receipts:
    sent=by_dispatch[receipt['dispatch_sequence']]
    assert sent['expected_counter']==receipt['consumed_counter'] and receipt['joint_consumed'] and not receipt['failed']
    for j in range(2):
        _,row=adopt[j][sent['expected_counter'][j]]
        assert abs(float(row[f'receipt{j+1}_s'])-receipt['consumed_sim_s'][j])<=1e-9
assert sends[-1]['stop_zero'] and sends[-1]['joint_velocity']==[0.,0.] and life['release']['zero_consumed']
normal_ns=np.array([x['sent_monotonic_ns'] for x in normal],dtype=np.int64);gaps=np.diff(normal_ns)/1e6
assert np.min(gaps)>0 and np.max(gaps)<=40. and abs(float(np.max(gaps))-report['timing']['actual_send_gap_ms']['maximum'])<1e-12
valid=[x for x in ticks if x['output_valid']]
assert len(valid)==len(normal) and np.all(np.diff([x['scheduled_monotonic_ns'] for x in valid])==20_000_000)
assert all(0<=x['command_age_ns']<=50_000_000 for x in valid)
phase=read(live/'worker_timing.json');pp=[dict(zip(phase['columns'],row)) for row in phase['rows']]
assert len(pp)==worker['requests']==worker['accepted']==sender['written']==len(events) and sender['dropped']==0
core_ms=np.array([(x['core_end_ns']-x['core_begin_ns'])/1e6 for x in pp]);callback_ms=np.array([(x['callback_exit_ns']-x['callback_entry_ns'])/1e6 for x in pp])
assert abs(float(core_ms.max())-report['worker']['durations']['core_ms']['maximum'])<1e-12 and float(core_ms.max())<=50.
with (live/'cpp_source/source_selections.csv').open(newline='') as f:selections=list(csv.DictReader(f))
assert len(selections)==report['feedback']['source_snapshots'] and report['feedback']['original_worker_snapshots_verified']==len(pp)
assert report['feedback']['clean_source_stop'] and [int(x['sequence']) for x in selections]==list(range(1,len(selections)+1))
sys.path.insert(0,'/root/autodl-tmp/amsim-venv/lib/python3.10/site-packages')
from pyulog import ULog
ulog=ULog(str(D/'px4.ulg'),message_name_filter_list=['vehicle_status','vehicle_land_detected'])
status=next(x.data for x in ulog.data_list if x.name=='vehicle_status');land=next(x.data for x in ulog.data_list if x.name=='vehicle_land_detected')
st=status['timestamp']/1e6;lt=land['timestamp']/1e6;flight=st>=start['origin_s']
assert status['arming_state'][-1]==1 and land['landed'][-1]==1 and st[-1]>start['origin_s']+24.
assert not np.any(status['failsafe'][flight])
assert identities==[ident(x) for x in paths]
unique={x['command_sequence'] for x in normal}
review=dict(kind='independent_cpp_fast_review_v1',passed=True,
    scope='Independent raw-byte, closed phase-law/native error, CDR, physics-adoption and ULog lifecycle cross-check; no new physics, time/space fitting or hardware claim',
    report=ident(p),launch=str(L),run=str(D),window_sim_s=report['window_sim_s'],trajectory=report['figure8_protocol']['trajectory'],metrics=metrics,tilt_peak_deg=tilt,
    source=dict(snapshots=len(selections),worker_link_count=len(pp),clean_stop=True),
    worker=dict(calls=len(pp),accepted=worker['accepted'],core_median_ms=float(np.median(core_ms)),core_max_ms=float(core_ms.max()),callback_max_ms=float(callback_ms.max())),
    execution=dict(continuous_sends=len(normal),all_sends=len(sends),receipts=len(receipts),stop_zero=1,repeated_sends=sum(bool(x['repeated']) for x in normal),
        joint_counts=[dict(x) for x in counts],same_physics_boundary=same_boundary,gate_events=len(events),dispatch_ticks=len(ticks),faults=0,max_send_gap_ms=float(gaps.max()),command_age_max_ms=max(x['command_age_ns'] for x in valid)/1e6),
    recovery=dict(runtime_closed=True,landed_and_disarmed=True,parameters_restored=True,child_exit_codes={k:v['returncode'] for k,v in life['processes'].items()},last_status_s=float(st[-1]),last_land_s=float(lt[-1]),failsafe_samples_active_and_recovery=int(np.count_nonzero(status['failsafe'][flight]))),
    limitations=['Precise simulated EV localization; no GPS or hardware accuracy claim','No per-publication PX4 ACK is inferred from ULog value matching','Float64-to-counter mapping uses declared exclusive ordered exactly-once bridge assumption','Independent per-joint adoption is recorded honestly, without claiming all pairs share the same physics boundary','One ordinary-scheduler execution is not a worst-case timing guarantee'],original_identities=identities)
extra=dict(unique_continuous_candidates=len(unique),not_sent_accepted_candidates=sorted(set(range(1,worker['accepted']+1))-unique),
    repeated_sends=[{k:x[k] for k in ('dispatch_sequence','command_sequence','sim_s','sent_monotonic_ns')} for x in normal if x['repeated']],
    last_sends=[{k:x[k] for k in ('dispatch_sequence','command_sequence','sim_s','stop_zero')} for x in sends[-3:]],ranges=ranges,
    joint_peak_to_peak_deg=[float(np.degrees(ranges[f'arm_joint_{j}_q']['peak_to_peak'])) for j in (1,2)],
    scope='Original full24s physical world ranges; these are not control-contribution allocation ratios')
write('review.json',review);write('ranges_and_candidate_counts.json',extra)
write('manifest.json',dict(files=[ident(O/n) for n in ('review_remote.py','review.json','ranges_and_candidate_counts.json')]))
print(json.dumps(dict(passed=True,metrics=metrics,tilt=tilt,worker=review['worker'],execution=review['execution'],recovery=review['recovery'],extra=extra,files=[ident(O/n) for n in ('review.json','ranges_and_candidate_counts.json','manifest.json')]),indent=2))
