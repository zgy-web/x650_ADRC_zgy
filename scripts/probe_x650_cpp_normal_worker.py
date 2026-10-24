"""Private DDS-domain dynamic-reference test of the actual C++ worker.

The input sequence is explicitly synthetic software evidence. No ingress,
dispatcher, Gazebo, PX4 client or actuator publisher is created.
"""
from __future__ import annotations
import argparse
import copy
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time
import numpy as np
from validate_x650_cpp_normal_reference import TRAJECTORY, identity


def require(ok,why):
    if not ok:raise ValueError(why)


def read_bound(item):
    actual=identity(item['path'])
    require(all(actual[k]==item[k] for k in item),'Bound input changed')
    return json.loads(Path(item['path']).read_text())


def array_error(a,b):
    a,b=np.asarray(a),np.asarray(b)
    require(a.shape==b.shape and np.isfinite(a).all() and np.isfinite(b).all(),'Invalid numerical evidence')
    return float(np.max(abs(a-b))) if a.size else 0.


def require_identical_message_fields(left,right):
    """CDR alignment padding is not a field; every declared field must match."""
    require(type(left) is type(right),'Original CDR message type differs')
    fields=left.get_fields_and_field_types()
    require(fields==right.get_fields_and_field_types(),'Original CDR field schema differs')
    for key in fields:
        require(np.array_equal(getattr(left,key),getattr(right,key)),f'Original CDR field differs: {key}')


def snapshot_fields(row,config):
    """Pure raw input -> wire conversion; all unused fixed slots stay zero."""
    i=row['input'];s=i['state'];c=i['context'];goal=i['target']
    result=dict(schema_version=1,replay_only=False,sequence=row['sequence'],
        control_time_s=s['control_time'],task_time_s=i['t'],dt_s=i['dt'],state_time_s=s['stamp'],joint_time_s=s['joint_stamp'],
        position_enu=s['position'],velocity_enu=s['velocity'],omega_body_flu=s['omega_body'],acceleration_enu=s['acceleration'],
        rotation_row_major=s['rotation'],q=s['q'],dq=s['dq'],target_position_enu=goal['position'],
        target_velocity_enu=goal['velocity'],target_acceleration_enu=goal['acceleration'],position_gains=config['base_position_gain'],
        reference_time_s=c['reference_time'],bus_origin_s=c['bus_origin'],reference_r=c['reference'],
        command_count=len(c['commands']),packet_count=len(c['packets']))
    require(0<result['command_count']<=64 and 0<result['packet_count']<=64,'Bounded nonempty histories required')
    require(all(c['bus_origin']<=x['send_time']<=i['t']+1e-10 for x in c['packets']),
            'Synthetic packet history lies outside the unchanged live bus boundary')
    result['command_time_s']=[x['time'] for x in c['commands']]+[0.]*(64-len(c['commands']))
    result['command_velocity']=[v for x in c['commands'] for v in x['velocity']]+[0.]*(128-2*len(c['commands']))
    result['packet_send_time_s']=[x['send_time'] for x in c['packets']]+[0.]*(64-len(c['packets']))
    for name in ('q_ref','dq_ref','tau_ff','kp','kd'):
        result['packet_'+name]=[v for x in c['packets'] for v in x[name]]+[0.]*(128-2*len(c['packets']))
    return result


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    for name in ('sequence','startup','worker','output'):ap.add_argument('--'+name,type=Path,required=True)
    ap.add_argument('--domain-id',type=int,default=223);ap.add_argument('--probe-cpu',type=int,default=94)
    a=ap.parse_args();require(1<=a.domain_id<=232,'Private domain required')
    a.output.mkdir(parents=True,exist_ok=False);os.sched_setaffinity(0,{a.probe_cpu})
    os.environ.update(ROS_DOMAIN_ID=str(a.domain_id),ROS_LOCALHOST_ONLY='1')
    startup=json.loads(a.startup.read_text());sequence=json.loads(a.sequence.read_text())
    require(sequence['passed'] and sequence['complete'] and sequence['trajectory']==startup['trajectory']==TRAJECTORY,'Frozen dynamic sequence required')
    require(sequence['startup_sha256']==identity(a.startup)['sha256'] and sequence['config']==startup['config'],'Sequence/startup identity differs')
    baseline=[read_bound(row) for row in sequence['rows']]
    require(len(baseline)==160 and [r['sequence'] for r in baseline]==list(range(1,161)),'Complete 160-step software sequence required')
    start_values=read_bound(dict(path=sequence['live_start_path'],sha256=sequence['live_start_sha256']))
    require(baseline[0]['input']['t']==.02 and baseline[-1]['input']['t']==3.2,'Expected moving startup phase coverage differs')
    from rclpy.serialization import serialize_message,deserialize_message
    from rclpy.qos import QoSProfile,ReliabilityPolicy
    import rclpy
    from flyarm_control_interfaces.msg import ControlSnapshot,ControlProposal,LiveStart
    sources=[identity(__file__),identity(a.sequence),identity(a.startup),identity(a.worker)]
    hash_bytes=list(bytes.fromhex(startup['task_config']['sha256']))
    report=dict(kind='cpp_normal_worker_actual_ros_v1',complete=False,passed=False,trajectory=TRAJECTORY,
        physical_acceptance=False,no_actuator_publishers=False,source_artifacts=sources,binary=identity(a.worker),
        startup_manifest=identity(a.startup),core_library=identity(startup['core_library']['path']),
        optimization_library=identity(startup['optimization_library']['path']),predictor=identity(startup['predictor_library']['path']),
        sequence=identity(a.sequence),software_task_time_s=[.02,3.2],continuous_positive_steps=160,
        scope='Synthetic continuous moving startup through actual ROS; full 32-second phases are separately checked by independent trajectory and matrix proofs, not a physical run',cases=[])
    rclpy.init();node=rclpy.create_node('cpp_normal_private_software_probe')
    qos=QoSProfile(depth=10,reliability=ReliabilityPolicy.BEST_EFFORT)
    snapshots=node.create_publisher(ControlSnapshot,'/flyarm/isolated/v1/admitted/snapshot',qos)
    starts=node.create_publisher(LiveStart,'/flyarm/isolated/v1/live_start',1)
    received=[];node.create_subscription(ControlProposal,'/flyarm/isolated/v1/output/proposal',received.append,qos)
    child=None;log=None
    def spin_until(condition,timeout,label):
        end=time.monotonic()+timeout
        while not condition():
            if time.monotonic()>=end:raise TimeoutError(label)
            rclpy.spin_once(node,timeout_sec=.001)
            if child is not None and child.poll() is not None and not condition():raise RuntimeError(label+': worker exited')
    try:
        for case_index,name in enumerate(('dynamic_sequence_sigint','wrong_target','wrong_phase','invalid_hash_latched')):
            out=a.output/name;out.mkdir();received.clear();session=250000000+case_index
            log=(out/'child.log').open('wb')
            child=subprocess.Popen(['taskset','-c','87,89,91,97',str(a.worker),'--ros-args',
                '-p',f'startup_manifest:={a.startup.resolve()}','-p',f'startup_sha256:={identity(a.startup)["sha256"]}',
                '-p',f'output_dir:={out.resolve()}','-p',f'expected_session_id:={session}','-p','expected_epoch:=1',
                '-p','simulation_live:=true','-p','executor_cpu:=87','-p','auxiliary_cpus:=[89,91,97]'],
                stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
            spin_until(lambda:(out/'worker_ready').exists() and snapshots.get_subscription_count()==1 and starts.get_subscription_count()==1,20.,'READY discovery')
            pubs=node.get_publisher_names_and_types_by_node('cpp_full_pipeline_mpc','/')
            require(set(x[0] for x in pubs)<= {'/flyarm/isolated/v1/output/proposal','/rosout','/parameter_events'},'Worker has unexpected actuator/output publisher')
            require(any(x[0]=='/flyarm/isolated/v1/output/proposal' for x in pubs),'Actual proposal publisher not discovered')
            start=LiveStart()
            for key,value in start_values.items():setattr(start,key,value)
            start.session_id=session;start.epoch=1;start.configuration_sha256=list(hash_bytes);start.issued_monotonic_ns=time.monotonic_ns()
            starts.publish(start)
            until=time.monotonic()+.04
            while time.monotonic()<until:rclpy.spin_once(node,timeout_sec=.001)
            count=160 if case_index==0 else 120
            case=dict(name=name,passed=False,rows=[],publisher_names=[x[0] for x in pubs]);report['cases'].append(case)
            sent=[];due=time.monotonic()
            for index,reference in enumerate(baseline[:count]):
                while time.monotonic()<due:rclpy.spin_once(node,timeout_sec=min(.001,max(0.,due-time.monotonic())))
                snapshot=ControlSnapshot()
                for key,value in snapshot_fields(reference,startup['config']).items():setattr(snapshot,key,value)
                snapshot.session_id=session;snapshot.epoch=1;snapshot.configuration_sha256=list(hash_bytes)
                invalid=case_index>0 and index==count-1
                if invalid:
                    if name=='wrong_target':snapshot.target_position_enu[0]+=1e-6
                    elif name=='wrong_phase':
                        later=baseline[index+1]['input']['target']
                        snapshot.target_position_enu=later['position'];snapshot.target_velocity_enu=later['velocity'];snapshot.target_acceleration_enu=later['acceleration']
                    else:snapshot.configuration_sha256[0]^=1
                snapshot.issued_monotonic_ns=time.monotonic_ns();sent.append(bytes(serialize_message(snapshot)));snapshots.publish(snapshot)
                spin_until(lambda:len(received)>index,1.,f'proposal {index+1}')
                got=received[index]
                require(got.sequence==index+1 and got.snapshot_sequence==index+1 and got.session_id==session and got.epoch==1,'Actual proposal identity differs')
                require(list(got.configuration_sha256)==hash_bytes,'Output hash differs')
                if invalid:
                    require(got.status!=0 and not got.accepted_optimum and not any(got.base_velocity_target) and not any(got.joint_velocity),'Invalid input not rejected with zero output')
                    case['rows'].append(dict(sequence=index+1,status=int(got.status),intentionally_invalid=True))
                    # Re-submit a valid input after the rejection; the worker must exit/latch.
                    snapshot.configuration_sha256=list(hash_bytes)
                    for field,key in (('target_position_enu','position'),('target_velocity_enu','velocity'),('target_acceleration_enu','acceleration')):setattr(snapshot,field,reference['input']['target'][key])
                    snapshot.issued_monotonic_ns=time.monotonic_ns();snapshots.publish(snapshot);child.wait(timeout=5.)
                    until=time.monotonic()+.05
                    while time.monotonic()<until:rclpy.spin_once(node,timeout_sec=.001)
                    require(child.returncode==1 and len(received)==count,'Rejected worker resumed or did not exit')
                else:
                    if got.status!=0:
                        report['failed_proposal']=dict(sequence=int(got.sequence),status=int(got.status),
                            accepted_optimum=bool(got.accepted_optimum),nonlinear_check_passed=bool(got.nonlinear_check_passed),
                            subproblems=int(got.subproblems),compute_start_monotonic_ns=int(got.compute_start_monotonic_ns),
                            compute_end_monotonic_ns=int(got.compute_end_monotonic_ns))
                        raise ValueError(f'Actual worker rejected synthetic snapshot {index+1}; status={got.status}; inspect original native record')
                    expected={'base_velocity_target':reference['command'][:3],'joint_velocity':reference['command'][3:],
                        'position_reference_enu':reference['position_reference'],'velocity_feedforward_enu':reference['velocity_feedforward'],
                        'acceleration_feedforward_enu':reference['acceleration_feedforward']}
                    errors={key:array_error(getattr(got,key),value) for key,value in expected.items()}
                    require(got.status==0 and got.accepted_optimum and got.nonlinear_check_passed and got.subproblems==reference['diagnostics']['subproblems'] and max(errors.values())<=1e-8,'Actual dynamic command/branch differs')
                    case['rows'].append(dict(sequence=index+1,status=int(got.status),task_time_s=snapshot.task_time_s,
                        target_position=list(snapshot.target_position_enu),target_velocity=list(snapshot.target_velocity_enu),target_acceleration=list(snapshot.target_acceleration_enu),
                        absolute_errors=errors,native_callback_ms=(got.compute_end_monotonic_ns-got.compute_start_monotonic_ns)/1e6))
                due+=.02
            if case_index==0:child.send_signal(signal.SIGINT);child.wait(timeout=8.);require(child.returncode==0,'Normal SIGINT failed')
            final=json.loads((out/'worker_final.json').read_text());sender=json.loads((out/'diagnostic_sender.json').read_text());timing=json.loads((out/'worker_timing.json').read_text())
            require(final['requests']==count and final['accepted']==count-(case_index>0) and final['faulted']==(case_index>0),'Worker final counters differ')
            require(sender['complete'] and sender['written']==count and timing['complete'] and len(timing['rows'])==count,'Missing actual worker evidence')
            records=[]
            wire_crosscheck=[];probe_records=out/'probe_records';probe_records.mkdir()
            for i in range(count):
                path=out/'native_records'/f'{i+1:06d}.json';record=json.loads(path.read_text())
                require(record['sequence']==i+1 and record['diagnostic_mode']=='compact','Record identity differs')
                # Store each endpoint's original serialization independently.
                # ROS C++/Python serializers may leave different alignment
                # bytes; compare all actual fields after verified decoding.
                sent_path=probe_records/f'{i+1:06d}.sent_snapshot.cdr';sent_path.write_bytes(sent[i])
                received_path=probe_records/f'{i+1:06d}.received_proposal.cdr';received_path.write_bytes(serialize_message(received[i]))
                require(identity(record['snapshot_cdr']['path'])==record['snapshot_cdr'],'Recorded snapshot CDR hash differs')
                require(identity(record['proposal_cdr']['path'])==record['proposal_cdr'],'Recorded proposal CDR hash differs')
                require_identical_message_fields(deserialize_message(Path(record['snapshot_cdr']['path']).read_bytes(),ControlSnapshot),deserialize_message(sent[i],ControlSnapshot))
                require_identical_message_fields(deserialize_message(Path(record['proposal_cdr']['path']).read_bytes(),ControlProposal),received[i])
                wire_crosscheck.append(dict(sequence=i+1,sent_snapshot=identity(sent_path),received_proposal=identity(received_path),
                    snapshot_all_fields_exact=True,proposal_all_fields_exact=True))
                if case_index==0 or i<count-1:
                    d=record['result']['diagnostics'];expected=baseline[i]['diagnostics']
                    errors={key:array_error(d[key],expected[key]) for key in ('plan','predicted_states','predicted_tip','predicted_rotations','candidate_envelope','horizon')}
                    require(max(errors.values())<=1e-7 and d['accepted'] and d['nonlinear_passed'] and d['actual_bounds'] and not d['previous_plan_fallback'],'Actual moving native state/plan/candidate differs')
                    case['rows'][i]['diagnostic_absolute_errors']=errors
                    require(case['rows'][i]['native_callback_ms']<=50.,'Actual native worker deadline exceeded')
                records.append(identity(path))
            case.update(passed=True,child_exit=child.returncode,final=final,recorder=sender,records=records,
                wire_crosscheck=wire_crosscheck,cdr_comparison='All declared fields exactly equal; independent raw CDR bytes and SHA retained; serializer alignment padding is not a message field',
                timing=identity(out/'worker_timing.json'),affinity=identity(out/'worker_affinity_final.json'))
            log.close();log=None;child=None
            print(json.dumps(dict(case=name,passed=True,rows=count)),flush=True)
        require(sources==[identity(__file__),identity(a.sequence),identity(a.startup),identity(a.worker)],'Software proof inputs changed')
        report.update(passed=True,complete=True,no_actuator_publishers=True)
    except Exception as error:report['error']=repr(error)
    finally:
        if child is not None and child.poll() is None:
            child.send_signal(signal.SIGINT)
            try:child.wait(timeout=8.)
            except subprocess.TimeoutExpired:child.terminate();child.wait(timeout=3.)
        if log is not None:log.close()
        node.destroy_node();rclpy.shutdown()
        (a.output/'report.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
    return 0 if report['passed'] else 1


if __name__=='__main__':raise SystemExit(main())
