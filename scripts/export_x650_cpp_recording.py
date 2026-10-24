"""Export original hold CDR and recorded warm plans for owning-C++ replay.

No ROS node is created and no actuator command is sent. Historical rejection,
publication and timing outcomes are preserved separately from offline math.
"""
from __future__ import annotations
import argparse
import base64
from copy import deepcopy
import json
from pathlib import Path
import sys
import numpy as np
from export_x650_cpp_inputs import ROOT, configuration, memory, raw_input, plain, file_identity


def controller_from_recording(run):
    from x650_wholebody.model import ArmModel
    from x650_wholebody.ts20 import FrozenTs20Model
    from x650_wholebody.ts20_prediction import Ts20PredictionModel
    from x650_wholebody.ts20_mpc import Ts20MpcController
    read=lambda p:json.loads(Path(p).read_text())
    meta=read(run/'controller_metadata.json');cfg=read(run/'controller_task.json')
    model=ArmModel(**meta['physical_profile'])
    inputs=cfg['ts20_inputs'];plant=read(inputs['plant_manifest'])['parameters']
    source=FrozenTs20Model(inputs['source_dir'],read(inputs['source_manifest']))
    predictor=Ts20PredictionModel(source.metadata()['motion_model'],kp=plant['kp'],kd=plant['kd'],bus_period_s=plant['bus_period_s'])
    config=deepcopy(cfg['mpc_config'])
    config['response_model']=meta['controller_details']['allocator']['response_model']
    controller=Ts20MpcController(model,meta['actual_pid_parameters'],predictor=predictor,
        context_provider=lambda s,t:None,config=config)
    return controller,meta,cfg


def decode(snapshot):
    from x650_wholebody.api import ControlState, EndpointTarget
    s=snapshot
    state=ControlState(np.asarray(s.position_enu),np.asarray(s.rotation_row_major).reshape(3,3),
        np.asarray(s.velocity_enu),np.asarray(s.omega_body_flu),np.asarray(s.q),np.asarray(s.dq),s.state_time_s,
        acceleration_enu=np.asarray(s.acceleration_enu),control_time_s=s.control_time_s,joint_stamp_s=s.joint_time_s)
    target=EndpointTarget(np.asarray(s.target_position_enu),np.asarray(s.target_velocity_enu),np.asarray(s.target_acceleration_enu))
    commands=[(float(when),np.asarray(s.command_velocity)[2*i:2*i+2].copy())
        for i,when in enumerate(s.command_time_s[:s.command_count])]
    packets=[dict(send_time_s=float(s.packet_send_time_s[i]),**{field:
        np.asarray(getattr(s,'packet_'+field))[2*i:2*i+2].copy() for field in ('q_ref','dq_ref','tau_ff','kp','kd')})
        for i in range(s.packet_count)]
    context=dict(reference_state=dict(time_s=s.reference_time_s,r=np.asarray(s.reference_r)),
        bus_origin_s=s.bus_origin_s,command_history=commands,packet_history=packets)
    return state,target,context


def main(argv=None):
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--run',type=Path,required=True);p.add_argument('--recording-sha256',required=True)
    p.add_argument('--output',type=Path,required=True)
    a=p.parse_args(argv);a.output.mkdir(parents=True,exist_ok=False)
    from rclpy.serialization import deserialize_message
    from flyarm_control_interfaces.msg import ControlSnapshot, ControlProposal
    live=a.run/'isolated_live';recording=live/'recording/diagnostics.jsonl'
    identity=file_identity(recording)
    if identity['sha256']!=a.recording_sha256:raise ValueError('Original recording SHA differs')
    c,meta,cfg=controller_from_recording(a.run)
    start=json.loads((live/'live_start.json').read_text())
    c.reset(start['anchor_position_enu'],np.asarray(start['anchor_rotation_row_major']).reshape(3,3),start['anchor_q'])
    count=start['history_count'];commands=np.asarray(start['history_command']).reshape(64,5)[:count]
    c.prime_command_history(list(zip(start['history_time_s'][:count],commands)),start['origin_s'])
    manifest=dict(kind='cpp_raw_historical_inputs_v1',complete=False,rows=[],no_flight=True,
        original_recording=identity,reference_timing_is_not_a_gate=True,
        historical_acceptance_unchanged=True,source_files_unchanged=True)
    sources=[recording,live/'live_start.json',a.run/'controller_metadata.json',a.run/'controller_task.json']
    manifest['source_artifacts']=[file_identity(f) for f in sources]
    for index,line in enumerate(recording.open()):
        row=json.loads(line)
        snapshot=deserialize_message(base64.b64decode(row['snapshot_cdr']['base64']),ControlSnapshot)
        proposal=deserialize_message(base64.b64decode(row['proposal_cdr']['base64']),ControlProposal)
        state,target,context=decode(snapshot)
        c.context_provider=lambda s,t:context
        c.trajectory=lambda t:target
        d=row['diagnostic']
        item=dict(schema_version=1,label=dict(index=index,sequence=int(snapshot.sequence),historical=True),
            config=configuration(c),memory=memory(c),input=raw_input(c,state,target,snapshot.task_time_s,snapshot.dt_s))
        item['oracle']=plain(dict(historical_record=True,command=np.r_[proposal.base_velocity_target,proposal.joint_velocity],
            position_reference=np.asarray(proposal.position_reference_enu),velocity_feedforward=np.asarray(proposal.velocity_feedforward_enu),
            plan=np.asarray(d['predicted_commands']).ravel(),predicted_states=np.asarray(d['predicted_states']).ravel(),
            predicted_tip=np.asarray(d['predicted_tip_positions']).ravel(),objective=d['objective'],
            subproblems=len(d['ts20_relinearization_attempts']),attempts=d['ts20_relinearization_attempts'],
            historical_proposal_status=int(proposal.status),historical_issue_ns=int(snapshot.issued_monotonic_ns),
            scope='recorded mathematical results; not a new historical timing/publication outcome'))
        path=a.output/f'case_{index:04d}.json';path.write_text(json.dumps(item,separators=(',',':'),allow_nan=False))
        manifest['rows'].append(file_identity(path))
        c._plan=np.asarray(d['predicted_commands']);c._plan_t=float(snapshot.task_time_s);c._plan_h=np.asarray(d['node_intervals_s'])
        c.previous_command=c._plan[0].copy();c.last_t=c._plan_t;c._previous_stamp=state.stamp_s;c.step_count=index+1
        c._joint_history.append((c.last_t,c.previous_command[3:].copy()))
        c._joint_history=[(when,u) for when,u in c._joint_history if when>=c.last_t-.75]
        c._chain_clock_origin=state.control_time_s-snapshot.task_time_s
    manifest.update(complete=True,source_files_unchanged=manifest['source_artifacts']==[file_identity(f) for f in sources])
    (a.output/'manifest.json').write_text(json.dumps(manifest,indent=2,allow_nan=False))
    print(json.dumps(dict(total=len(manifest['rows']),source_files_unchanged=manifest['source_files_unchanged'])))
    return 0 if manifest['source_files_unchanged'] else 1


if __name__=='__main__':raise SystemExit(main())
