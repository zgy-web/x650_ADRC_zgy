#!/usr/bin/env python3
"""Check the actual ABI4 worker against a new sequential low-rate oracle.

From the explicitly derived input, only issue clocks refresh; state and actual packet
histories are preserved. Each implementation advances its own fresh memory.
There is no simulation, ingress, dispatcher, PX4 client or actuator publisher.
This recorded-state test cannot certify a new closed-loop flight.
"""
from __future__ import annotations
import argparse
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time

from audit_x650_recovery_replay import OUTPUT_DIMENSIONS, strict_json
from probe_x650_recovery_worker import (require, identity, bound, receipt, write_new,
    array_error, same_fields, message_matches_json, proposal_errors, cpu_list)


def structural_error(actual, expected, *, path='', ignored=('elapsed_ms','compute_ms')):
    """Compare owning diagnostics while excluding independently measured time."""
    if isinstance(expected,dict):
        require(isinstance(actual,dict), 'Diagnostic object differs: '+path)
        keys=set(expected)-set(ignored)
        require(keys <= set(actual), 'Diagnostic fields missing: '+path)
        return max([structural_error(actual[k],expected[k],path=path+'/'+k,ignored=ignored) for k in keys]+[0.])
    if isinstance(expected,list):
        require(isinstance(actual,list) and len(actual)==len(expected),'Diagnostic array differs: '+path)
        return max([structural_error(a,b,path=path+'/'+str(i),ignored=ignored)
                    for i,(a,b) in enumerate(zip(actual,expected))]+[0.])
    if isinstance(expected,bool) or isinstance(expected,str) or expected is None:
        require(type(actual) is type(expected) and actual==expected,'Diagnostic branch differs: '+path)
        return 0.
    require(isinstance(actual,(int,float)) and not isinstance(actual,bool)
        and math.isfinite(actual) and math.isfinite(expected),'Nonfinite/malformed diagnostic: '+path)
    return abs(float(actual)-float(expected))


def validate_oracle(recording, startup, oracle):
    require(recording.get('kind')=='x650_cpp_multirate_derived_recording_v1'
        and recording.get('originals_verified_unchanged') is True,'Verified original recording required')
    require(recording['config']==startup['config'] and recording['trajectory']==startup['trajectory'],
        'Original controller config/trajectory changed')
    require(oracle.get('kind')=='x650_cpp_multirate_sequence_oracle_v1' and oracle.get('core_abi')==4
        and oracle.get('passed') is True and oracle.get('complete') is True and oracle.get('row_count')==len(recording['rows'])
        and oracle.get('step_count_after')==len(recording['rows']) and oracle.get('memory_sequentially_reconstructed_in_cpp') is True
        and oracle.get('original_proposals_used_as_output_oracle') is False
        and oracle.get('recorded_states_and_actual_history_preserved') is True,
        'Fresh ABI4 sequential finite-prefix-call oracle required')
    require(len(recording['rows'])==len(oracle['rows'])>=20
        and [r['sequence'] for r in recording['rows']]==[r['sequence'] for r in oracle['rows']]==list(range(1,len(recording['rows'])+1)),
        'Original/oracle prefix must include all 1..finite-prefix records')
    require(startup['task_config']['sha256']==bytes(recording['live_start']['configuration_sha256']).hex(),
        'Original task hash differs from startup')


def expected_calls(recording, oracle, inputs):
    rows=[];previous=None
    for sequence,(source_id,oracle_id) in enumerate(zip(recording['rows'],oracle['rows']),1):
        inputs.extend((receipt(source_id),receipt(oracle_id)))
        source,expected=(strict_json(v['path']) for v in (source_id,oracle_id))
        require(source['sequence']==expected['sequence']==sequence and expected.get('returned') is True,
                'Sequential oracle row failed or identity differs')
        require(expected['source_row']==source_id,'Oracle used a different original recorded snapshot')
        for key in ('record_identity','snapshot_identity','proposal_identity'):inputs.append(receipt(source[key]))
        before,after=expected['memory_before'],expected['memory_after']
        require(before['step_count']==sequence-1 and after['step_count']==sequence
            and (previous is None or before==previous),'Oracle memory was reset/substituted between calls')
        previous=after
        for message in (source['snapshot'],source['proposal']):
            for key in ('session_id','epoch','configuration_sha256'):
                require(message[key]==recording['live_start'][key],'Original session identity changed')
        d=expected['diagnostics']
        require(d['accepted'] and d['actual_bounds'] and d['nonlinear_passed'] and not d['previous_plan_fallback']
            and 1<=d['subproblems']<=3 and d['compute_ms']<=50.,'New sequential oracle candidate is not qualified')
        rows.append((source,expected))
    return rows


def main(argv=None):
    p=argparse.ArgumentParser(description=__doc__)
    for name in ('recording','startup','worker','oracle'):
        p.add_argument('--'+name,type=Path,required=True);p.add_argument('--'+name+'-sha256',required=True)
    p.add_argument('--output',type=Path,required=True);p.add_argument('--domain-id',type=int,required=True)
    p.add_argument('--executor-cpu',type=int,required=True);p.add_argument('--auxiliary-cpus',type=cpu_list,required=True)
    p.add_argument('--probe-cpu',type=int,required=True)
    a=p.parse_args(argv);require(a.output.is_absolute() and not a.output.exists(),'New absolute output directory required')
    a.output.mkdir(parents=True);out=a.output
    report=dict(kind='x650_cpp_multirate_actual_worker_probe_v1',schema_version=1,core_abi=4,
        complete=False,passed=False,test_sequence_complete=False,physical_validation=False,physical_acceptance=False,
        counterfactual_closed_loop_claimed=False,injected_delay=False,no_actuator_publishers=False,
        only_refreshed_input_field_from_derived_recording='issued_monotonic_ns',memory_oracle='fresh sequential ABI4 Core; each row inherits prior newly computed memory',
        scope='Explicitly derived 25 Hz snapshots with original physical states/history; real ABI4 worker; no plant execution',
        rows=[],wire_crosscheck=[],started_unix=time.time())
    inputs=[];child=node=log=ros=None
    try:
        require(1<=a.domain_id<=232,'Private DDS domain must be 1..232')
        cpus=[a.executor_cpu,*a.auxiliary_cpus,a.probe_cpu]
        require(min(cpus)>=0 and len(set(cpus))==len(cpus) and set(cpus)<=os.sched_getaffinity(0),
                'Distinct available probe/executor/auxiliary CPU IDs required')
        os.sched_setaffinity(0,{a.probe_cpu});os.environ.update(ROS_DOMAIN_ID=str(a.domain_id),ROS_LOCALHOST_ONLY='1')
        for name in ('recording','startup','worker','oracle'):
            value=bound(getattr(a,name),getattr(a,name+'_sha256'));setattr(a,name,Path(value['path']));inputs.append(value);report[name]=value
        for path in (Path(__file__),Path(__file__).with_name('probe_x650_recovery_worker.py')):inputs.append(identity(path))
        report['probe']=inputs[-2]
        recording,startup,oracle=(strict_json(getattr(a,k)) for k in ('recording','startup','oracle'))
        validate_oracle(recording,startup,oracle)
        require(oracle['recording']==report['recording'],'Oracle binds a different original recording')
        libraries={}
        for role,key in (('core','core_library'),('optimization','optimization_library'),('predictor','predictor_library')):
            libraries[role]=bound(startup[key]['path'],startup[key]['sha256']);inputs.append(libraries[role])
            require(all(libraries[role][k]==oracle['loaded_code'][role][k] for k in ('path','sha256')),
                    'Actual worker and new sequential oracle code differ: '+role)
        report['libraries']=libraries
        from simulations.x650_cpp_multirate_v1.profile import effective
        report['effective_profile']=effective();report['timing_profile']=startup['timing_profile']
        for item in (startup['task_config'],*startup['source_artifacts']):inputs.append(bound(item['path'],item['sha256']))
        pairs=expected_calls(recording,oracle,inputs);inputs.append(receipt(recording['live_start_cdr_identity']))
        import rclpy
        from rclpy.serialization import serialize_message,deserialize_message
        from rclpy.qos import QoSProfile,ReliabilityPolicy
        from flyarm_control_interfaces.msg import ControlSnapshot,ControlProposal,LiveStart
        ros=rclpy;ros.init();node=ros.create_node('x650_interval_private_recorded_probe')
        qos=QoSProfile(depth=10,reliability=ReliabilityPolicy.BEST_EFFORT)
        snapshots=node.create_publisher(ControlSnapshot,'/flyarm/isolated/v1/admitted/snapshot',qos)
        starts=node.create_publisher(LiveStart,'/flyarm/isolated/v1/live_start',1);received=[]
        node.create_subscription(ControlProposal,'/flyarm/isolated/v1/output/proposal',received.append,qos)
        worker_dir,wire_dir=out/'worker',out/'probe_records';worker_dir.mkdir();wire_dir.mkdir()
        log=(out/'child.log').open('xb')
        command=['taskset','-c',','.join(map(str,cpus[:-1])),str(a.worker),'--ros-args',
            '-p',f'startup_manifest:={a.startup}','-p',f'startup_sha256:={a.startup_sha256}',
            '-p',f'output_dir:={worker_dir}','-p',f"expected_session_id:={recording['live_start']['session_id']}",
            '-p',f"expected_epoch:={recording['live_start']['epoch']}",'-p','simulation_live:=true','-p','timing_profile:=mpc25_dispatch50_v1',
            '-p',f'executor_cpu:={a.executor_cpu}','-p','auxiliary_cpus:=['+','.join(map(str,a.auxiliary_cpus))+']']
        report['command']=command;child=subprocess.Popen(command,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        def wait(condition,timeout,label):
            end=time.monotonic()+timeout
            while not condition():
                if time.monotonic()>=end:raise TimeoutError(label)
                ros.spin_once(node,timeout_sec=.001)
                if child.poll() is not None and not condition():raise RuntimeError(label+': worker exited')
        wait(lambda:(worker_dir/'worker_ready').exists() and snapshots.get_subscription_count()==1
            and starts.get_subscription_count()==1,20.,'ABI4 worker READY/discovery')
        ws=strict_json(worker_dir/'worker_startup.json')
        require(ws['core_abi']==4 and ws['diagnostic_mode']=='compact' and ws['startup_controller_steps']==0
            and ws['startup_manifest']==startup and ws['python_in_worker'] is False,'Actual ABI4 startup differs')
        require(ws['timing_profile']==startup['timing_profile']
            and ws['effective_profile']==effective(), 'Actual worker did not select the bound timing profile')
        for role,key,loaded in (('core','core_library','loaded_core_path'),('optimization','optimization_library','loaded_optimization_path'),('predictor','predictor_library','loaded_predictor_path')):
            require(ws[loaded]==libraries[role]['path'],'Actually loaded worker library path differs')
        wait(lambda:('cpp_full_pipeline_mpc','/') in node.get_node_names_and_namespaces(),
            5., 'Worker ROS graph discovery')
        pubs={name for name,_ in node.get_publisher_names_and_types_by_node('cpp_full_pipeline_mpc','/')}
        graph=node.get_node_names_and_namespaces()
        require(pubs<={'/flyarm/isolated/v1/output/proposal','/rosout','/parameter_events'}
            and '/flyarm/isolated/v1/output/proposal' in pubs
            and set(graph)=={('cpp_full_pipeline_mpc','/'),(node.get_name(),'/')},'Unexpected private-domain node/actuator publisher')
        report.update(no_actuator_publishers=True,publisher_names=sorted(pubs),node_graph=graph,ros_domain_id=a.domain_id,
            executor_cpu=a.executor_cpu,auxiliary_cpus=a.auxiliary_cpus,probe_cpu=a.probe_cpu,worker_startup=identity(worker_dir/'worker_startup.json'))
        original_start=deserialize_message(Path(recording['live_start_cdr_identity']['path']).read_bytes(),LiveStart)
        message_matches_json(original_start,recording['live_start']);start=deserialize_message(serialize_message(original_start),LiveStart)
        start.issued_monotonic_ns=time.monotonic_ns();same_fields(original_start,start,ignored=('issued_monotonic_ns',))
        start_path=wire_dir/'live_start.sent.cdr';start_path.write_bytes(serialize_message(start));starts.publish(start)
        until=time.monotonic()+.04
        while time.monotonic()<until:ros.spin_once(node,timeout_sec=.001)
        sent=[];due=time.monotonic();first_task=pairs[0][0]['snapshot']['task_time_s']
        for i,(source,expected) in enumerate(pairs):
            sequence=i+1;target_due=due+source['snapshot']['task_time_s']-first_task
            while time.monotonic()<target_due:ros.spin_once(node,timeout_sec=min(.001,max(0.,target_due-time.monotonic())))
            original=deserialize_message(Path(source['snapshot_identity']['path']).read_bytes(),ControlSnapshot);message_matches_json(original,source['snapshot'])
            old_proposal=deserialize_message(Path(source['proposal_identity']['path']).read_bytes(),ControlProposal);message_matches_json(old_proposal,source['proposal'])
            message=deserialize_message(serialize_message(original),ControlSnapshot);message.issued_monotonic_ns=time.monotonic_ns();same_fields(original,message,ignored=('issued_monotonic_ns',))
            raw=bytes(serialize_message(message));sent.append(raw);path=wire_dir/f'{sequence:06d}.sent_snapshot.cdr';path.write_bytes(raw);snapshots.publish(message)
            wait(lambda:len(received)>i,1.,f'proposal {sequence}');got=received[i]
            got_path=wire_dir/f'{sequence:06d}.received_proposal.cdr';got_path.write_bytes(serialize_message(got))
            current=dict(sequence=sequence,passed=False,task_time_s=message.task_time_s,status=int(got.status),
                sent_snapshot=identity(path),received_proposal=identity(got_path),original_snapshot=source['snapshot_identity'])
            report['rows'].append(current)
            require(got.sequence==got.snapshot_sequence==sequence and got.session_id==start.session_id and got.epoch==start.epoch
                and list(got.configuration_sha256)==list(start.configuration_sha256) and got.control_time_s==message.control_time_s
                and got.task_time_s==message.task_time_s and got.schema_version==1 and not got.replay_only,'Proposal input identity changed')
            callback_ms=(got.compute_end_monotonic_ns-got.compute_start_monotonic_ns)/1e6
            require(got.status==0 and got.accepted_optimum and got.nonlinear_check_passed and 0<=callback_ms<=50.
                and 0<=got.primal_violation<=2e-7 and 0<=got.kkt_residual<=.002,'ABI4 worker rejected/late/nonoptimal input '+str(sequence))
            errors=proposal_errors(got,expected['output'])
            require(max(errors.values())<=1e-8 and got.subproblems==expected['diagnostics']['subproblems'],'Worker command differs from fresh sequential oracle')
            current.update(passed=True,absolute_errors=errors,native_callback_ms=callback_ms,subproblems=int(got.subproblems))
        require(len(received)==len(pairs),'Unexpected extra/missing actual proposals')
        child.send_signal(signal.SIGINT);child.wait(timeout=8.);require(child.returncode==0,'Worker normal finish failed')
        final,sender,timing=(strict_json(worker_dir/n) for n in ('worker_final.json','diagnostic_sender.json','worker_timing.json'))
        require(final['requests']==final['accepted']==len(pairs) and not final['faulted'] and final['live_started']
            and sender['complete'] and not sender['failed'] and sender['offered']==sender['written']==len(pairs) and sender['dropped']==0
            and timing['complete'] and timing['dropped']==0 and len(timing['rows'])==len(pairs),'Actual worker counters/recording/timing incomplete')
        same_fields(deserialize_message((worker_dir/'worker_live_start.cdr').read_bytes(),LiveStart),start)
        report['live_start_crosscheck']=dict(sent=identity(start_path),received=identity(worker_dir/'worker_live_start.cdr'),original=recording['live_start_cdr_identity'],all_fields_exact=True)
        records=[]
        for i,(_,expected) in enumerate(pairs):
            sequence=i+1;path=worker_dir/'native_records'/f'{sequence:06d}.json';record=strict_json(path)
            require(record['sequence']==sequence and record['diagnostic_mode']=='compact' and not record['full_matrix_packet_capture'] and not record['error'],'Actual owning native record differs')
            snap,prop=receipt(record['snapshot_cdr']),receipt(record['proposal_cdr'])
            same_fields(deserialize_message(Path(snap['path']).read_bytes(),ControlSnapshot),deserialize_message(sent[i],ControlSnapshot))
            same_fields(deserialize_message(Path(prop['path']).read_bytes(),ControlProposal),received[i])
            result=record['result'];errors={k:array_error(result[k],expected['output'][k]) for k in OUTPUT_DIMENSIONS}
            require(max(errors.values())<=1e-8,'Complete owning native output differs from new sequential memory')
            actual,wanted=result['diagnostics'],expected['diagnostics'];diagnostic_error=0.
            for key in ('plan','predicted_states','predicted_tip','predicted_rotations','candidate_envelope','horizon',
                'accepted','actual_bounds','nonlinear_passed','previous_plan_fallback','subproblems','recovery_triggered','recovery_reason',
                'recovery_trust_radius','recovery_stopping_violation','recovery_attempts','interval_rows','interval_repairs','interval_constraints',
                'interval_canonical_envelope','interval_canonical_substeps','hold_guard'):
                expected_value=wanted[key]
                if key=='hold_guard':
                    # The standalone replay writer calls this field send_time;
                    # the ROS recorder calls the same seconds value send_time_s.
                    packets=[]
                    for packet in expected_value['packets']:
                        require('send_time' in packet and 'send_time_s' not in packet,
                            'Unexpected oracle packet schema')
                        packet=dict(packet);packet['send_time_s']=packet.pop('send_time');packets.append(packet)
                    expected_value=dict(expected_value,packets=packets)
                diagnostic_error=max(diagnostic_error,structural_error(actual[key],expected_value,path=key))
            require(diagnostic_error<=1e-8,'Actual prediction/interval/recovery branch differs from sequential oracle')
            report['rows'][i].update(full_native_output_absolute_errors=errors,diagnostic_absolute_error=diagnostic_error,
                recovery_triggered=actual['recovery_triggered'],interval_rows=actual['interval_rows'])
            report['wire_crosscheck'].append(dict(sequence=sequence,snapshot=snap,proposal=prop,snapshot_all_fields_exact=True,proposal_all_fields_exact=True));records.append(identity(path))
        for item in inputs:require(identity(item['path'])==item,'Bound input/code changed during probe')
        report.update(passed=True,test_sequence_complete=True,rows_count=len(pairs),records=records,final=final,recorder=sender,
            timing=identity(worker_dir/'worker_timing.json'),affinity=identity(worker_dir/'worker_affinity_final.json'),
            cdr_comparison='All declared fields exact; serializer padding excluded; original CDR SHA retained')
    except Exception as error:report['error']=repr(error)
    finally:
        if child is not None and child.poll() is None:
            try:child.send_signal(signal.SIGINT);child.wait(timeout=8.)
            except subprocess.TimeoutExpired:
                child.terminate()
                try:child.wait(timeout=3.)
                except subprocess.TimeoutExpired:child.kill();child.wait(timeout=2.)
            except Exception as error:report['cleanup_error']=repr(error);report['passed']=False
        if child is not None:report['child_exit']=child.poll()
        if log is not None:log.close()
        if node is not None:
            try:node.destroy_node()
            except Exception as error:report['node_cleanup_error']=repr(error);report['passed']=False
        if ros is not None and ros.ok():
            try:ros.shutdown()
            except Exception as error:report['ros_cleanup_error']=repr(error);report['passed']=False
        report.update(complete=True,finished_unix=time.time());write_new(out/'report.json',report)
    return 0 if report['passed'] else 1


if __name__=='__main__':raise SystemExit(main())
