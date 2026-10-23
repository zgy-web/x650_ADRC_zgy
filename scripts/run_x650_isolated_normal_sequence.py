#!/usr/bin/env python3
"""One ordinary figure eight after a new passed hold under a pinned protocol.

Source ROS/workspace environment before invocation. Default behavior performs
gate checks and launcher --dry-run only. --execute authorizes exactly one
normal run and its independent audit/plots; failure never triggers a retry,
new hold, fast trajectory, source change or numerical parameter adjustment.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import time
import traceback


def sha(path):
    digest=hashlib.sha256()
    with Path(path).open('rb') as stream:
        for chunk in iter(lambda:stream.read(1024*1024),b''):digest.update(chunk)
    return digest.hexdigest()


def read(path):
    return json.loads(Path(path).read_text(encoding='utf-8'))


def require(condition,message):
    if not condition:raise ValueError(message)


def require_protocol(protocol):
    revision=protocol.get('execution_revision')
    require(type(revision) is int and revision>=9 and protocol.get('simulation_only') is True,
            'A caller-pinned simulation execution protocol revision >= 9 is required')


def bound(path,expected):
    path=Path(path).resolve(strict=True)
    require(isinstance(expected,str) and len(expected)==64 and not set(expected)-set('0123456789abcdef'),
            'Explicit lowercase SHA256 required')
    require(path.is_file() and sha(path)==expected,'Pinned input bytes differ: '+str(path))
    return {'path':str(path),'sha256':expected}


def write_new(path,value):
    encoded=json.dumps(value,indent=2,allow_nan=False)+'\n'
    with Path(path).open('x',encoding='utf-8') as stream:stream.write(encoded)


def status_file(path,value):
    # Only this driver's own fresh status is updated; original evidence is
    # never rewritten. Exclusive event files retain every state transition.
    temporary=Path(str(path)+'.tmp')
    encoded=json.dumps(value,indent=2,allow_nan=False)+'\n'
    with temporary.open('x',encoding='utf-8') as stream:
        stream.write(encoded);stream.flush();os.fsync(stream.fileno())
    os.replace(temporary,path)


def inside(path,root):
    result=Path(path).resolve()
    result.relative_to(root.resolve())
    return result


def exact_launch(log,repo):
    entries=[line[len('ISOLATED_LAUNCH '):].strip()
             for line in Path(log).read_text(errors='replace').splitlines()
             if line.startswith('ISOLATED_LAUNCH ')]
    require(len(entries)==1,'Cannot identify exactly one launcher result; never select latest directory')
    folder=inside(entries[0],repo/'runs')
    require(folder.name.startswith('x650_isolated_figure8_launch_') and folder.is_dir(),
            'Reported launch directory has wrong identity')
    return folder


def exact_run(launch,repo,task_path):
    document=read(launch/'launch.json')
    require(document.get('execution_scenario')=='x650_isolated_figure8_v1','Wrong launch scenario')
    require(document.get('task_sha256')==sha(task_path) and document.get('task')==read(task_path)
            and sha(launch/'task.json')==sha(task_path),'Executed task differs from this driver')
    runs=document.get('run_directories',[])
    require(isinstance(runs,list) and len(runs)==1,'One uniquely bound physical run is required')
    folder=inside(runs[0],repo/'runs')
    require(folder.name.startswith('x650_isolated_figure8_v1_') and folder.is_dir(),
            'Wrong physical run identity')
    return folder,document


def require_normal_certificate(report,launch,run):
    require(report.get('kind')=='x650_isolated_normal_stage_certificate_v1' and report.get('stage')=='normal',
            'Audit is not an ordinary figure-eight certificate')
    require(all(report.get(k) is True for k in ('passed','complete','full_window_ulog_verified',
            'parameter_restore_verified','native_ground_verified','actuator_log_verified')),
            'Normal audit did not pass: '+str(report.get('error')))
    require(Path(report['launch_directory']).resolve()==launch.resolve()
            and Path(report['source_run_directory']).resolve()==run.resolve(),'Audit belongs to another run')
    window=report.get('window_sim_s',[])
    require(len(window)==2 and abs(window[1]-window[0]-32.)<=1e-9,'Audit does not cover the entire 32 s')


def verify_plots(folder,report_path):
    document=read(folder/'figure_manifest.json')
    require(document.get('kind')=='x650_isolated_validation_figure_v1' and document.get('complete') is True
            and document.get('stage')=='normal' and document.get('metric_match_verified_before_write') is True,
            'Plot generation did not produce a complete matched normal record')
    require(document.get('inputs',{}).get('report')=={'path':str(report_path.resolve()),'sha256':sha(report_path)},
            'Plot manifest belongs to another report')
    outputs=document.get('outputs',{})
    require({'endpoint_validation.png','endpoint_validation.pdf','endpoint_chart_data.csv'}<=set(outputs),
            'Expected plots/data are missing')
    for name,entry in outputs.items():
        p=inside(folder/name,folder)
        require(p.is_file() and p.stat().st_size==entry['bytes'] and sha(p)==entry['sha256'],
                'Plot output bytes differ: '+name)
    return document


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--repo',type=Path,required=True)
    for key in ('hold-task','hold-report','transport-proof'):
        parser.add_argument('--'+key,type=Path,required=True)
        parser.add_argument('--'+key+'-sha256',required=True)
    parser.add_argument('--protocol-sha256',required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--execute',action='store_true')
    args=parser.parse_args()
    repo=args.repo.resolve(strict=True)
    output=inside(args.output,repo/'artifacts')
    require(not output.exists(),'Fresh driver output required; there is no automatic resume or rerun')
    output.mkdir(parents=True,exist_ok=False)
    state={'kind':'x650_isolated_normal_sequence_v1','complete':False,'normal_passed':False,
        'execute_requested':args.execute,'fast_started':False,'retry_enabled':False,
        'created_unix':time.time(),'pid':os.getpid(),'host':os.uname().nodename if hasattr(os,'uname') else None,
        'repo':str(repo),'output':str(output),'phase':'created','commands':[]}
    event=0
    def update(phase,**values):
        nonlocal event
        state.update(phase=phase,**values);event+=1
        write_new(output/f'event_{event:03d}.json',state)
        status_file(output/'status.json',state)
        print(json.dumps({'phase':phase,**values},allow_nan=False),flush=True)
    def call(name,command):
        log=output/(name+'.log')
        env=os.environ.copy()
        env.update(OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1',MKL_NUM_THREADS='1',PYTHONUNBUFFERED='1')
        item={'name':name,'command':command,'log':str(log),'started_unix':time.time()}
        state['commands'].append(item)
        with log.open('x') as stream:
            child=subprocess.Popen(command,cwd=repo,env=env,stdout=stream,stderr=subprocess.STDOUT,
                                   stdin=subprocess.DEVNULL,start_new_session=True)
            item['pid']=child.pid
            update(name+'_running')
            # No timeout/kill: the launcher owns flight recovery. An externally
            # interrupted driver records this PID and leaves the lifecycle to
            # finish. A reviewer must inspect it before any later experiment.
            item['returncode']=child.wait()
        item.update(ended_unix=time.time(),log_sha256=sha(log))
        update(name+'_finished',last_returncode=item['returncode'])
        return item['returncode'],log
    try:
        script=Path(__file__).resolve()
        (output/'driver_source.py').write_bytes(script.read_bytes())
        state['driver']={'path':str(script),'sha256':sha(script)}
        protocol_path=repo/'configs/experiments/x650_isolated_live_v1.json'
        bindings={'protocol':bound(protocol_path,args.protocol_sha256),
            'hold_task':bound(args.hold_task,args.hold_task_sha256),
            'hold_report':bound(args.hold_report,args.hold_report_sha256),
            'transport_proof':bound(args.transport_proof,args.transport_proof_sha256)}
        protocol=read(protocol_path)
        require_protocol(protocol)
        require(protocol.get('frozen_hold_task_sha256')==args.hold_task_sha256,
                'New hold task is not the currently declared baseline')
        baseline=read(args.hold_task)
        require(baseline.get('stage')=='tip_hold' and baseline['numerical_backend']['library_sha256']
                ==protocol['frozen_numerical_library_sha256'],'New hold task/library identity differs')
        for folder in (repo,repo/'src',repo/'scripts'):
            sys.path.insert(0,str(folder))
        from simulations.x650_isolated_figure8_v1 import task
        require(task.ROOT.resolve()==repo,'Imported task is from another repository')
        cfg=task.build_task_config(bindings['hold_task'],bindings['hold_report'])
        cfg=task.validate_task_config(cfg)
        gate=task.validate_live_hold(cfg)
        require(cfg['stage']=='normal' and cfg['figure8_protocol']['fast_admitted'] is False,
                'Only ordinary figure eight may be selected')
        normal_task=output/'normal_task.json';write_new(normal_task,cfg)
        state.update(bindings=bindings,normal_task={'path':str(normal_task),'sha256':sha(normal_task)},
                     hold_gate=gate,protocol=protocol)
        # Audit and renderer files are bound by this driver's own evidence too.
        code_paths=[repo/'simulations/x650_isolated_figure8_v1/launch.py',
                    repo/'scripts/audit_x650_isolated_figure8.py',repo/'scripts/audit_x650_isolated_live.py',
                    repo/'scripts/plot_x650_isolated_validation.py']
        state['driver_step_sources']={str(p):sha(p) for p in code_paths}
        update('gates_verified')
        command=[sys.executable,str(code_paths[0]),'--task',str(normal_task),
                 '--transport-proof',str(args.transport_proof.resolve())]
        code,_=call('normal_preflight',command+['--dry-run'])
        require(code==0,'Normal launcher preflight failed; no flight started')
        if not args.execute:
            update('prepared_only',complete=True,normal_passed=False,flight_started=False)
            return 0
        require(not (repo/'.amsim/runtime.json').exists(),'An active runtime exists; do not launch another')
        for entry in bindings.values():bound(entry['path'],entry['sha256'])
        update('normal_launch_requested',flight_started=True)
        code,log=call('normal_launch',command)
        launch=exact_launch(log,repo)
        run,launch_document=exact_run(launch,repo,normal_task)
        state.update(launch_directory=str(launch),run_directory=str(run),
                     launch_complete=launch_document.get('complete') is True)
        # Even an unsuccessful completed lifecycle can receive an audit-failure
        # record; it cannot reach the certified plotting stage below.
        require(not (repo/'.amsim/runtime.json').exists(),'Runtime remains active; stop progression and inspect owned lifecycle')
        audit_dir=output/'normal_audit'
        audit_code,_=call('normal_audit',[sys.executable,str(code_paths[1]),'--launch-dir',str(launch),
            '--run-dir',str(run),'--output',str(audit_dir)])
        report_path=audit_dir/'report.json'
        require(report_path.is_file(),'Independent audit did not produce its report')
        state['normal_report']={'path':str(report_path),'sha256':sha(report_path)}
        report=read(report_path)
        require(code==0 and launch_document.get('complete') is True and audit_code==0,
                'Normal lifecycle or independent audit failed; no fast progression: '+str(report.get('error')))
        require_normal_certificate(report,launch,run)
        update('normal_accepted',normal_passed=True,endpoint=report['endpoint'])
        plot_dir=output/'normal_plots'
        plot_code,_=call('normal_plots',[sys.executable,str(code_paths[3]),'--report',str(report_path),'--output',str(plot_dir)])
        require(plot_code==0,'Normal passed but plot creation failed; keep original acceptance evidence')
        plots=verify_plots(plot_dir,report_path)
        require(all(sha(p)==digest for p,digest in state['driver_step_sources'].items()),
                'Driver/audit/renderer source changed while running')
        update('normal_complete_fast_not_started',complete=True,normal_passed=True,
            plot_manifest={'path':str(plot_dir/'figure_manifest.json'),'sha256':sha(plot_dir/'figure_manifest.json')},
            plot_metrics=plots['metrics'])
        return 0
    except BaseException as error:
        update('stopped_no_progression',complete=False,error=repr(error),traceback=traceback.format_exc(),
               runtime_present=(repo/'.amsim/runtime.json').exists(),
               interruption_policy='Do not kill simulation or rerun automatically; inspect recorded owned PID and lifecycle first')
        return 130 if isinstance(error,KeyboardInterrupt) else 1


if __name__=='__main__':raise SystemExit(main())
