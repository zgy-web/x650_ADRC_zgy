#!/usr/bin/env python3
"""Private-domain ROS lifecycle probe; no algorithm, actuator, or flight output."""
import argparse
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time


def load_helper(path):
    spec=importlib.util.spec_from_file_location('placement_probe_helper',path)
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    return module


def write(path,value):
    path.write_text(json.dumps(value,indent=2,allow_nan=False)+'\n')


def child(args):
    helper=load_helper(args.helper)
    import rclpy
    from rclpy.node import Node
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.signals import SignalHandlerOptions
    from std_msgs.msg import String
    roles={'executor':[args.cpus[0]],'auxiliary':args.cpus[1:]}
    rclpy.init(args=[],domain_id=args.domain_id,signal_handler_options=SignalHandlerOptions.NO)
    stop=helper.WorkerStopRequest();stop.install()
    node=executor=None;report={};placement=None
    try:
        node=Node('worker_placement_software_probe')
        # Entities are private and never published to. Timer callbacks exercise
        # the actual single-threaded executor with the same CPU observer.
        node.create_publisher(String,'/worker_placement_probe/unused',1)
        node.create_subscription(String,'/worker_placement_probe/unused',lambda msg:None,1)
        executor=SingleThreadedExecutor();executor.add_node(node)
        placement=helper.WorkerAffinity(args.cpus,roles)
        placement.bind_before_ready()
        observer=helper.ExecutorCpuObserver(os.getpid(),roles['executor'])
        callback_rows=[]
        def observe_callback():
            begin=time.monotonic_ns();tid,cpu=observer()
            end_tid,end_cpu=observer();end=time.monotonic_ns()
            if len(callback_rows)>=1000:raise RuntimeError('Probe callback trace overflow')
            callback_rows.append([begin,end,tid,cpu,end_tid,end_cpu])
        node.create_timer(.05,observe_callback)
        placement.check('ready');placement.write(args.output/'ready.json')
        placement.check('live_start')
        stop.spin(executor,node.context)
        placement.check('final')
        report={'passed':stop.requested and len(callback_rows)>=2,
            'stop_requested':stop.requested,'context_ok_before_shutdown':node.context.ok(),
            'callbacks':callback_rows,'placement':placement.document()}
    except BaseException as error:
        report={'passed':False,'error':repr(error),'placement':None if placement is None else placement.document()}
    finally:
        # Same order as isolated_node: final evidence, then executor/node/RMW.
        try:
            write(args.output/'child_final.json',report)
        finally:
            try:
                if executor is not None:executor.shutdown()
                if node is not None:node.destroy_node()
                if rclpy.ok():rclpy.shutdown()
            finally:
                stop.restore()
    return 0 if report.get('passed') else 2


def parent(args):
    helper=load_helper(args.helper)
    roles={'executor':[args.cpus[0]],'auxiliary':args.cpus[1:]}
    helper.validate_thread_roles(args.cpus,roles)
    if args.domain_id==0 or not 1<=args.domain_id<=232:
        raise ValueError('Require an explicit nonzero private ROS domain')
    args.output.mkdir(parents=True,exist_ok=False)
    command=['taskset','--cpu-list',','.join(map(str,args.cpus)),sys.executable,str(Path(__file__).resolve()),
        '--helper',str(args.helper.resolve()),'--output',str(args.output.resolve()),
        '--domain-id',str(args.domain_id),'--cpus',','.join(map(str,args.cpus)),'--child']
    report={'kind':'worker_thread_placement_ros_software_v1','physical_validation':False,
        'scope':'real ROS participant/executor and production placement/signal helper; no optimizer or physical execution validation',
        'commands_published':False,'helper_sha256':hashlib.sha256(args.helper.read_bytes()).hexdigest(),
        'probe_sha256':hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),'passed':False}
    process=None;identity=None
    try:
        with (args.output/'child.log').open('w') as log:
            process=subprocess.Popen(command,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
            deadline=time.monotonic()+20
            while not (args.output/'ready.json').exists():
                if process.poll() is not None:raise RuntimeError('Child exited before READY')
                if time.monotonic()>deadline:raise TimeoutError('READY timeout')
                time.sleep(.02)
            ready=json.loads((args.output/'ready.json').read_text())
            identity=ready['phases']['ready']['threads'][str(process.pid)]['start_ticks']
            helper.verify_worker_evidence(ready,ready['phases']['ready']['threads'],args.cpus,roles)
            time.sleep(.25)
            begin=time.monotonic_ns();cpu_begin=time.thread_time_ns()
            observed=helper.thread_snapshot(process.pid)
            helper.verify_snapshot(observed,process.pid,args.cpus,roles,ready['phases']['bound_before_ready']['threads'])
            cpu_end=time.thread_time_ns();end=time.monotonic_ns()
            report['dynamic_handoff_check']={'wall_ns':end-begin,'thread_cpu_ns':cpu_end-cpu_begin,'threads':observed}
            report['sigint_ns']=time.monotonic_ns();os.kill(process.pid,signal.SIGINT)
            process.wait(timeout=10)
            final=json.loads((args.output/'child_final.json').read_text())
            helper.verify_worker_evidence(final['placement'],observed,args.cpus,roles,require_final=True)
            report.update(child_final=final,returncode=process.returncode,
                passed=bool(process.returncode==0 and final['passed'] and final['context_ok_before_shutdown']))
    except BaseException as error:
        report['error']=repr(error)
    finally:
        if process is not None and process.poll() is None:
            # Only this child, and require its READY identity when available.
            current=helper.thread_snapshot(process.pid)
            if identity is None or current[str(process.pid)]['start_ticks']==identity:
                process.terminate()
                try:process.wait(timeout=3)
                except subprocess.TimeoutExpired:process.kill();process.wait(timeout=3)
            else:report['cleanup_error']='Owned child identity changed; no signal sent'
        report['child_exited']=process is not None and process.poll() is not None
        if not report['child_exited']:report['passed']=False
        write(args.output/'report.json',report)
    print(json.dumps({'passed':report['passed'],'report':str(args.output/'report.json')}))
    return 0 if report['passed'] else 2


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--helper',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--domain-id',type=int,default=217)
    p.add_argument('--cpus',type=lambda s:[int(n) for n in s.split(',')],default=[87,89,91,97])
    p.add_argument('--child',action='store_true')
    args=p.parse_args()
    return child(args) if args.child else parent(args)


if __name__=='__main__':raise SystemExit(main())
