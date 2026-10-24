"""Read-only hold22 context reconstruction, explicitly not rejected raw CDR."""
from pathlib import Path
from types import SimpleNamespace
from dataclasses import dataclass
import base64,hashlib,json,sys
import numpy as np
from rclpy.serialization import deserialize_message
from flyarm_control_interfaces.msg import ControlSnapshot

R=Path('/root/autodl-tmp/safe-aerial-manipulator-testbed')
sys.path.insert(0,str(R/'control_packages/x650_wholebody_v1'))
from x650_wholebody.ts20_runtime import Ts20Runtime
r=R/'runs/x650_isolated_live_v1_20261024_120535_11859';live=r/'isolated_live'
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
start=json.loads((live/'live_start.json').read_text());origin=start['origin_s']
events=json.loads((live/'events.json').read_text())
trace=json.loads((live/'ingress_timing.json').read_text())
trace=[dict(zip(trace['columns'],row)) for row in trace['rows']]
failed_admission=[row for row in trace if row['kind']==4 and row['gate_end_ns'] and not row['forward']][-1]
feedback=json.loads((live/'feedback_sources.json').read_text())[-1]
calls=[]
for line in (live/'recording/diagnostics.jsonl').read_text().splitlines():
    d=json.loads(line)
    calls.append(deserialize_message(base64.b64decode(d['snapshot_cdr']['base64']),ControlSnapshot))

@dataclass
class State:
    q:object
    dq:object
    control_time_s:float
    joint_stamp_s:float

runtime=Ts20Runtime(r/'ts20_state.csv',r/'ts20_packets.csv',
                   SimpleNamespace(bus_period_s=.005,delay_s=.003346379092576424))
results=[]
for now,saved in ((calls[-1].control_time_s,calls[-1]),(feedback['sim_s'],None)):
    obs=runtime.select_joint_observation(now)
    state=State(np.asarray(obs.q),np.asarray(obs.dq),now,obs.source_stamp_s)
    state,_=runtime.measured_state(state,now,observation=obs)
    t=now-origin;context=runtime.context(state,t)
    commands,packets,reference=(context[k] for k in ('command_history','packet_history','reference_state'))
    violations=[]
    if not 1<=len(commands)<=64 or not 1<=len(packets)<=64:violations.append({'predicate':'counts'})
    if abs(reference['time_s']-(state.joint_stamp_s-(now-t)))>1e-8:
        violations.append({'predicate':'reference_time'})
    for i,(at,u) in enumerate(commands):
        if at>t:violations.append({'predicate':'command_future','index':i,'delta_s':at-t})
        if i and at<commands[i-1][0]:violations.append({'predicate':'command_order','index':i})
        if any(abs(u)>[.45,.35]):violations.append({'predicate':'command_limit','index':i})
    for i,p in enumerate(packets):
        at=p['send_time_s']
        if at>t:violations.append({'predicate':'packet_future','index':i,'delta_s':at-t})
        if at<context['bus_origin_s']:violations.append({'predicate':'packet_before_origin','index':i})
        if i and at<=packets[i-1]['send_time_s']:violations.append({'predicate':'packet_order','index':i})
        if any(p['kp']<0) or any(p['kd']<0):violations.append({'predicate':'negative_gains','index':i})
    row={'original_rejected_snapshot_cdr':False,'now':now,'joint':state.joint_stamp_s,
         'task_time_s':t,'origin_s':origin,'reference_time_s':reference['time_s'],
         'command_time_s':[x[0] for x in commands],'packet_time_s':[x['send_time_s'] for x in packets],
         'bus_origin_s':context['bus_origin_s'],'strict_gate_context_violations':violations,
         'padding':'reconstructed serialization uses zero padding; rejected original padding was not captured'}
    if saved is not None:
        matches={'counts':len(commands)==saved.command_count and len(packets)==saved.packet_count,
            'joint_stamp':state.joint_stamp_s==saved.joint_time_s,
            'reference_time':reference['time_s']==saved.reference_time_s,
            'reference_r':bool(np.array_equal(reference['r'],saved.reference_r)),
            'bus_origin':context['bus_origin_s']==saved.bus_origin_s,
            'command_times':bool(np.array_equal([x[0] for x in commands],saved.command_time_s[:saved.command_count])),
            'command_values':bool(np.array_equal(np.asarray([x[1] for x in commands]).ravel(),saved.command_velocity[:2*saved.command_count])),
            'packet_times':bool(np.array_equal([p['send_time_s'] for p in packets],saved.packet_send_time_s[:saved.packet_count]))}
        for field in ('q_ref','dq_ref','tau_ff','kp','kd'):
            matches[field]=bool(np.array_equal(np.asarray([p[field] for p in packets]).ravel(),
                                              getattr(saved,'packet_'+field)[:2*saved.packet_count]))
        row['actual_accepted18_context_matches']=matches
        assert all(matches.values())
    else:
        control=feedback['feedback']['control_state']
        row['actual_feedback_joint_matches']={
            'q':bool(np.array_equal(state.q,control['q_rad'])),
            'dq':bool(np.array_equal(state.dq,control['dq_rad_s'])),
            'joint_stamp':state.joint_stamp_s==feedback['feedback']['joint_stamp_s']}
    results.append(row)
runtime.close()
report={'kind':'hold22_context_roundoff_reconstruction_v1','physical_run':str(r),
    'original_worker_requests':len(calls),'last_events':events[-3:],
    'rejected_admission':failed_admission,'reconstructed_contexts':results,
    'limitation':'Original source77/rejected admission19 CDR was not retained. Reconstruction reads complete postflight logs using the unchanged causal provider; this does not recover the original CSV read cursor. Accepted18 matches its original CDR exactly.',
    'comparison':'Existing provider and Python worker accept history <=task+1e-10; old C++ gate used <=task with no epsilon.',
    'inputs_sha256':{str(p):sha(p) for p in (r/'ts20_state.csv',r/'ts20_packets.csv',
        live/'live_start.json',live/'events.json',live/'ingress_timing.json',live/'feedback_sources.json',
        live/'recording/diagnostics.jsonl',R/'control_packages/x650_wholebody_v1/x650_wholebody/ts20_runtime.py')}}
print(json.dumps(report,indent=2))
