#!/usr/bin/env python3
"""Private-domain ROS probe of the real state_ingress output-ACK cadence.

SOFTWARE FIXTURE ONLY. Starts state_ingress, never a dispatcher, Gazebo, PX4,
worker, joint publisher or actuator. The ActuatorSend messages are synthetic.
Only the one child PID created here is signalled. Run in a sourced Linux ROS
workspace with an explicit reviewed binary hash and fresh output directory.
"""
from __future__ import annotations
import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import signal
import subprocess
import sys
import time
import traceback
import uuid

TOPIC='/flyarm/isolated/v1/'

def require(value,message):
    if not value:raise RuntimeError(message)

def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()

def save(path,value):Path(path).write_text(json.dumps(value,indent=2,allow_nan=False)+'\n',encoding='utf-8')

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--binary',type=Path,required=True)
    parser.add_argument('--binary-sha256',required=True)
    parser.add_argument('--domain-id',type=int,required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--timeout-s',type=float,default=25.)
    args=parser.parse_args()
    require(sys.platform.startswith('linux'),'Linux ROS workspace required')
    require(1<=args.domain_id<=232,'Explicit private nondefault domain required')
    require(10<=args.timeout_s<=120,'Bounded timeout10..120s required')
    require(re.fullmatch('[0-9a-f]{64}',args.binary_sha256) is not None,'Lowercase binary SHA256 required')
    require(args.binary.is_absolute(),'Absolute state_ingress binary required')
    binary=args.binary.resolve(strict=True)
    require(binary.name=='state_ingress' and os.access(binary,os.X_OK),'Only executable state_ingress may be launched')
    require(sha(binary)==args.binary_sha256,'Binary differs from reviewed hash')
    args.output=args.output.resolve();args.output.mkdir(parents=True,exist_ok=False)
    report={'kind':'x650_ingress_cadence_synthetic_probe_v1','passed':False,'complete':False,
      'synthetic_software_fixture_only':True,'physical_validation':False,'flight_accepted':False,
      'hardware_qualified':False,'eligible_as_stage_gate':False,'domain_id':args.domain_id,
      'binary':{'path':str(binary),'sha256':args.binary_sha256},'probe_sha256':sha(__file__),
      'events':[],'admitted':[],'source_published':[],'ack_published':[]}
    os.environ.update(ROS_DOMAIN_ID=str(args.domain_id),ROS_LOCALHOST_ONLY='1')
    deadline=time.monotonic()+args.timeout_s
    child=node=log=None;initialized=False;error=None
    try:
        import rclpy
        from rclpy.qos import QoSProfile,ReliabilityPolicy
        from flyarm_control_interfaces import msg
        rclpy.init();initialized=True
        node=rclpy.create_node('ingress_cadence_probe_'+uuid.uuid4().hex[:10])
        best=QoSProfile(depth=64,reliability=ReliabilityPolicy.BEST_EFFORT)
        reliable=QoSProfile(depth=8,reliability=ReliabilityPolicy.RELIABLE)
        observed={'admitted':[],'accepted':[],'status':[],'events':[]}
        subscriptions=[]
        for key,typ,topic in [('admitted',msg.ControlSnapshot,'admitted/snapshot'),
                              ('accepted',msg.AcceptedCommand,'accepted'),
                              ('status',msg.IngressStatus,'source_status'),('events',msg.GateEvent,'event')]:
            subscriptions.append(node.create_subscription(typ,TOPIC+topic,
                lambda value,key=key:observed[key].append(value),best))
        subscriptions.append(node.create_subscription(msg.GateEvent,TOPIC+'event',lambda value:None,best))
        publishers={key:node.create_publisher(typ,TOPIC+topic,qos) for key,typ,topic,qos in [
          ('source',msg.ControlSnapshot,'source',best),('proposal',msg.ControlProposal,'output/proposal',best),
          ('start',msg.LiveStart,'live_start',reliable),('stop',msg.RuntimeStop,'stop',reliable),
          ('ack',msg.ActuatorSend,'actuator_send',best),('consumed',msg.ExecutionReceipt,'consumed',best)]}
        def pump_for(seconds):
            until=time.monotonic()+seconds
            while time.monotonic()<until:
                require(time.monotonic()<deadline,'Global probe timeout')
                if child is not None:require(child.poll() is None,'Ingress exited unexpectedly')
                rclpy.spin_once(node,timeout_sec=min(.0003,max(0.,until-time.monotonic())))
        def until(predicate,label,seconds=.030):
            end=min(deadline,time.monotonic()+seconds)
            while not predicate():
                require(time.monotonic()<end,'Timeout: '+label)
                pump_for(.0003)
            require(not any(not e.accepted for e in observed['events']),'Ingress rejected a fixture round')
        def event(name,**values):
            report['events'].append({'event':name,'monotonic_ns':time.monotonic_ns(),**values})
        pump_for(.15)
        require(len(node.get_node_names())==1,'Private DDS domain is not empty')
        session=int.from_bytes(os.urandom(4),'little')&0x3fffffff or 1
        epoch=1
        digest=hashlib.sha256(('SYNTHETIC ingress cadence '+uuid.uuid4().hex).encode()).hexdigest()
        report.update(session_id=session,epoch=epoch,configuration_sha256=digest)
        ready=args.output/'ingress_ready'
        command=[str(binary),'--ros-args','-p','simulation_live:=true','-p',f'expected_session_id:={session}',
            '-p',f'expected_epoch:={epoch}','-p',f'configuration_sha256:={digest}',
            '-p','position_gains:=[0.95,0.95,1.0]','-p','task_origin_s:=10.0','-p',f'ready_file:={ready}']
        log=(args.output/'ingress.log').open('wb')
        child=subprocess.Popen(command,stdout=log,stderr=subprocess.STDOUT,env=os.environ.copy())
        report['own_pid']=child.pid
        until(lambda:ready.exists() and all(p.get_subscription_count()>=1 for p in publishers.values()),'DDS ready',8.)
        start=msg.LiveStart();start.session_id=session;start.epoch=epoch
        start.configuration_sha256=list(bytes.fromhex(digest));start.origin_s=10.;start.issued_monotonic_ns=time.monotonic_ns()
        publishers['start'].publish(start);pump_for(.003)
        source_sequence=0
        def source(task_time):
            nonlocal source_sequence
            source_sequence+=1;s=msg.ControlSnapshot();s.schema_version=1
            s.session_id=session;s.epoch=epoch;s.sequence=source_sequence;s.replay_only=False
            s.configuration_sha256=list(bytes.fromhex(digest));s.control_time_s=10.+task_time;s.task_time_s=task_time
            s.dt_s=.02;s.state_time_s=s.joint_time_s=s.control_time_s
            s.position_enu=[0.,0.,2.];s.rotation_row_major=[1.,0.,0.,0.,1.,0.,0.,0.,1.]
            s.q=[.1,.2];s.dq=[0.,0.];s.target_position_enu=[0.,0.,1.6];s.position_gains=[.95,.95,1.]
            s.reference_time_s=task_time;s.reference_r=[.1,.2];s.bus_origin_s=-.02
            s.command_count=1;s.packet_count=2;s.command_time_s=[-.02]+[0.]*63
            s.packet_send_time_s=[-.02,-.015]+[0.]*62
            s.packet_q_ref=[.1,.2,.1,.2]+[0.]*124;s.packet_kp=[6.]*4+[0.]*124;s.packet_kd=[.25]*4+[0.]*124
            s.issued_monotonic_ns=time.monotonic_ns();publishers['source'].publish(s)
            report['source_published'].append({'sequence':int(s.sequence),'issued_ns':int(s.issued_monotonic_ns),
                'publish_return_ns':time.monotonic_ns(),'control_s':float(s.control_time_s),'task_s':task_time})
            until(lambda:observed['status'] and observed['status'][-1].source_sequence==s.sequence,'source reception')
            return s
        def proposal(s):
            p=msg.ControlProposal();p.schema_version=1;p.session_id=session;p.epoch=epoch
            p.sequence=p.snapshot_sequence=s.sequence;p.replay_only=False;p.configuration_sha256=list(bytes.fromhex(digest))
            p.control_time_s=s.control_time_s;p.task_time_s=s.task_time_s
            p.position_reference_enu=[0.,0.,2.];p.accepted_optimum=p.nonlinear_check_passed=True;p.subproblems=1
            p.compute_start_monotonic_ns=p.compute_end_monotonic_ns=time.monotonic_ns()
            publishers['proposal'].publish(p)
            until(lambda:observed['accepted'] and observed['accepted'][-1].proposal.sequence==p.sequence,'accepted proposal')
        def ack(sequence,*,wrong_session=False,stop=False):
            a=msg.ActuatorSend();a.session_id=session+1 if wrong_session else session;a.epoch=epoch
            a.command_sequence=sequence;a.dispatch_sequence=len(report['ack_published'])+1
            a.sim_s=10.1;a.stop_zero=stop;a.sent_monotonic_ns=time.monotonic_ns()
            publishers['ack'].publish(a)
            report['ack_published'].append({'sequence':sequence,'session_id':int(a.session_id),'stop_zero':stop,
                'sent_monotonic_ns':int(a.sent_monotonic_ns),'publish_return_ns':time.monotonic_ns()})
        def no_extra(count,label):
            pump_for(.0015);require(len(observed['admitted'])==count,label+' admitted without a matching actual-send grant')
        source(.016);no_extra(0,'first16ms');event('first16ms_waited')
        source(.020);until(lambda:len(observed['admitted'])==1,'first20ms admission')
        proposal(observed['admitted'][0]);s36=source(.036)
        no_extra(1,'accepted but not sent')
        ack(1,wrong_session=True);no_extra(1,'wrong session')
        ack(999);no_extra(1,'wrong sequence')
        ack(1,stop=True);no_extra(1,'stop acknowledgement')
        ack(1);until(lambda:len(observed['admitted'])==2,'actual ACK plus16ms source')
        second=observed['admitted'][1]
        require(second.issued_monotonic_ns==s36.issued_monotonic_ns,'Source issue time was rewritten')
        require(abs(second.dt_s-.016)<1e-12,'16ms source interval was normalized')
        event('actual_ACK_released16ms',dt_s=float(second.dt_s),source_issued_ns=int(second.issued_monotonic_ns))
        proposal(second);source(.056)
        ack(1);no_extra(2,'duplicate old acknowledgement')
        ack(2);until(lambda:len(observed['admitted'])==3,'next20ms admission')
        third=observed['admitted'][2];require(abs(third.dt_s-.020)<1e-12,'20ms dt changed')
        proposal(third);source(.080);ack(3)
        until(lambda:len(observed['admitted'])==4,'next24ms admission')
        require(abs(observed['admitted'][3].dt_s-.024)<1e-12,'24ms dt changed')
        stop=msg.RuntimeStop();stop.session_id=session;stop.epoch=epoch;publishers['stop'].publish(stop);pump_for(.003)
        report['admitted']=[{'sequence':int(s.sequence),'issued_ns':int(s.issued_monotonic_ns),
            'control_s':float(s.control_time_s),'dt_s':float(s.dt_s),'command_count':int(s.command_count),
            'packet_count':int(s.packet_count)} for s in observed['admitted']]
        require([s['sequence'] for s in report['admitted']]==[1,2,3,4],'Duplicate or skipped admission')
        event('fixture_contract_completed')
    except BaseException as exc:
        error=exc;report.update(error=repr(exc),traceback=traceback.format_exc())
    finally:
        if child is not None and child.poll() is None:
            child.send_signal(signal.SIGINT)
            try:child.wait(timeout=3.)
            except subprocess.TimeoutExpired:
                child.terminate()
                try:child.wait(timeout=2.)
                except subprocess.TimeoutExpired:child.kill();child.wait(timeout=2.)
        if log is not None:log.close()
        if node is not None:node.destroy_node()
        if initialized and rclpy.ok():rclpy.shutdown()
    try:
        if error is None:
            require(child.returncode==0,'Ingress did not exit cleanly')
            trace_path=args.output/'ingress_timing.json';trace=json.loads(trace_path.read_text())
            require(trace['complete'] is True and trace['overflow']==0,'Ingress timing evidence incomplete')
            rows=[dict(zip(trace['columns'],r)) for r in trace['rows']]
            forwards=[r for r in rows if r['kind']==4 and r['forward']]
            releases=[r for r in rows if r['kind']==5 and r['forward']]
            require([r['sequence'] for r in forwards]==[1,2,3,4],'Trace does not cover exact admissions')
            require([r['sequence'] for r in releases]==[1,2,3],'Trace contains missing/duplicate output release')
            for row,s in zip(forwards,report['admitted']):
                require(row['source_issued_ns']==s['issued_ns'],'Trace source issue differs from admitted wire')
            report.update(passed=True,complete=True,trace_sha256=sha(trace_path),
                negative_ack_cases_passed=True,actual_dt_ms=[s['dt_s']*1000 for s in report['admitted']])
        require(sha(binary)==args.binary_sha256,'Binary changed during probe')
    except BaseException as exc:
        report.update(passed=False,complete=False,error=repr(exc),traceback=traceback.format_exc())
    report['child_returncode']=None if child is None else child.returncode
    save(args.output/'report.json',report)
    print(json.dumps({'passed':report['passed'],'report':str(args.output/'report.json'),'error':report.get('error')},indent=2))
    return 0 if report['passed'] else 1

if __name__=='__main__':raise SystemExit(main())
