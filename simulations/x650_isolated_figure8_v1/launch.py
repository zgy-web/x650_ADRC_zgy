#!/usr/bin/env python3
"""Normal figure eight after complete actual isolated hold; unchanged execution and numerics."""
import argparse,hashlib,json,os,subprocess,sys,time,zipfile
from pathlib import Path
ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT))
from simulations.x650_isolated_figure8_v1 import task
from simulations.pid_ff_wholebody_v1.launch import flight_lock,atomic_json
EXECUTION_SCENARIO='x650_isolated_figure8_v1'


def execution_resources(protocol):
    roles=protocol['cpu_roles'];allowed=set(os.sched_getaffinity(0));reserved=set()
    for cpus in roles.values():
        if not cpus or any(type(c) is not int or c not in allowed for c in cpus):
            raise ValueError('Declared execution CPU unavailable; qualify a new resource plan')
        for cpu in cpus:
            siblings=Path(f'/sys/devices/system/cpu/cpu{cpu}/topology/thread_siblings_list').read_text().strip()
            for part in siblings.split(','):
                span=list(map(int,part.split('-')))
                reserved.update(range(span[0],span[-1]+1))
    background=sorted(allowed-reserved)
    if not background:raise ValueError('No background simulation CPUs remain')
    return {'roles':roles,'background':background,'reserved_with_smt_siblings':sorted(reserved),
            'scope':protocol['cpu_scope'],'scheduler_changed':False}


def source_hashes():
    paths=[]
    for name in ('src/amsim','ros2_ws/src','control_packages/x650_wholebody_v1','simulations/x650_wholebody_mpc_v1',
                 'simulations/x650_ts20_hold_v1','simulations/x650_isolated_live_v1','simulations/x650_isolated_figure8_v1',
                 'simulations/x650_ts20_figure8_v1'):
        paths += [p for p in (ROOT/name).rglob('*') if p.is_file() and p.suffix in ('.py','.cpp','.hpp','.cc','.hh','.json','.msg','.xml','.txt','.npz') and '__pycache__' not in p.parts]
    paths += [ROOT/'scripts/amsim',ROOT/'scripts/x650_isolated_diagnostic_recorder.py',ROOT/'configs/experiments/x650_isolated_live_v1.json',ROOT/'scenarios/x650_isolated_live_v1.yaml',ROOT/'scenarios/x650_isolated_figure8_v1.yaml',
              ROOT/'scripts/audit_x650_isolated_figure8.py']
    return {p.relative_to(ROOT).as_posix():task.sha(p) for p in sorted(set(paths))}


def runtime_hashes():
    from ament_index_python.packages import get_package_prefix
    result={}
    for package,names in {'flyarm_control_gateway':('state_ingress','live_dispatcher','consumption_source'),
                           'flyarm_mpc_reference':('isolated_node',)}.items():
        for name in names:
            path=Path(get_package_prefix(package))/'lib'/package/name
            result[str(path)]=task.sha(path)
    return result


def main():
    p=argparse.ArgumentParser();p.add_argument('--task',type=Path,required=True)
    p.add_argument('--transport-proof',type=Path,required=True);p.add_argument('--dry-run',action='store_true');a=p.parse_args()
    protocol=task.read(ROOT/'configs/experiments/x650_isolated_live_v1.json')
    resources=execution_resources(protocol)
    cfg=task.validate_task_config(task.read(a.task));_,provenance=task.preflight(cfg)
    from amsim.scenario_registry import get
    descriptor=get(EXECUTION_SCENARIO)
    if descriptor.module!='simulations/x650_isolated_figure8_v1/run_case.py' or descriptor.config['model']!=cfg['model']:
        raise ValueError('Explicit live scenario must use the frozen physical model')
    proof=task.read(a.transport_proof)
    if proof.get('passed') is not True or not proof.get('zero_consumed'):raise ValueError('Transport proof required')
    binaries=runtime_hashes()
    if proof.get('dispatcher_sha256') != binaries.get(proof.get('dispatcher_binary')):
        raise ValueError('Transport proof does not bind the current dispatcher binary')
    if a.dry_run:print(json.dumps({'ready':True,'task_sha256':task.sha(a.task)}));return
    with flight_lock(ROOT):
        if (ROOT/'.amsim/runtime.json').exists():raise RuntimeError('Active runtime exists')
        out=ROOT/'runs'/('x650_isolated_figure8_launch_'+time.strftime('%Y%m%d_%H%M%S')+'_'+str(os.getpid()))
        out.mkdir(parents=True);logs=out/'actuator_live';logs.mkdir()
        (out/'task.json').write_bytes(a.task.read_bytes());before=source_hashes()
        atomic_json(out/'source_manifest.json',before)
        with zipfile.ZipFile(out/'sources.zip','x',zipfile.ZIP_DEFLATED) as z:
            for rel,h in before.items():
                b=(ROOT/rel).read_bytes();assert hashlib.sha256(b).hexdigest()==h;z.writestr(rel,b)
        env=os.environ.copy();env.update(X650_EXECUTION_ADAPTER='isolated_simulation_v1',
            FLYARM_EXECUTION_RESOURCES=json.dumps(resources,separators=(',',':')),
            X650_TS20_TASK_CONFIG=str(out/'task.json'),AMSIM_TS20_LOG_DIR=str(logs),
            AMSIM_SCENARIO_PYTHON='/usr/bin/python3',AMSIM_CHECK_PYTHON='/usr/bin/python3',
            AMSIM_VENV='/root/autodl-tmp/amsim-venv',PYTHONUNBUFFERED='1',
            OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1',MKL_NUM_THREADS='1')
        env['PYTHONPATH']='/usr/lib/python3/dist-packages:/root/autodl-tmp/amsim-venv/lib/python3.10/site-packages:'+env.get('PYTHONPATH','')
        env['PX4_GZ_MODEL_POSE']=','.join(str(x) for x in cfg['spawn_pose_enu_xyz_rpy'])
        status={'complete':False,'task':cfg,'task_sha256':task.sha(a.task),'provenance':provenance,
            'protocol':protocol,'protocol_sha256':task.sha(ROOT/'configs/experiments/x650_isolated_live_v1.json'),
            'execution_resources':resources,
            'transport_proof':{'path':str(a.transport_proof.resolve()),'sha256':task.sha(a.transport_proof)},
            'runtime_binaries_sha256':binaries,
            'execution_scenario':EXECUTION_SCENARIO,'new_runtime_acceptance_required':True}
        atomic_json(out/'launch.json',status);print('ISOLATED_LAUNCH '+str(out),flush=True)
        old=set((ROOT/'runs').glob(EXECUTION_SCENARIO+'_*'))
        try:
            with (out/'flight.log').open('x') as log:
                result=subprocess.run(['taskset','--cpu-list',','.join(map(str,resources['background'])),
                    str(ROOT/'scripts/amsim'),'run',EXECUTION_SCENARIO,'--headless'],cwd=ROOT,env=env,stdout=log,stderr=subprocess.STDOUT)
            new=sorted(set((ROOT/'runs').glob(EXECUTION_SCENARIO+'_*'))-old)
            status.update(exit_code=result.returncode,run_directories=[str(x) for x in new],runtime_closed=not (ROOT/'.amsim/runtime.json').exists())
            status['complete']=result.returncode==0 and len(new)==1 and task.read(new[0]/'result.json').get('success') is True and status['runtime_closed']
        except BaseException as e:status['error']=repr(e);raise
        finally:
            after=source_hashes();status['sources_unchanged']=after==before
            status['runtime_binaries_unchanged']=runtime_hashes()==binaries
            status['complete']=status['complete'] and status['runtime_binaries_unchanged']
            status['complete']=status['complete'] and status['sources_unchanged']
            atomic_json(out/'source_manifest_after.json',after);atomic_json(out/'launch.json',status)
        if not status['complete']:raise RuntimeError('Live simulation lifecycle did not complete: '+str(out))

if __name__=='__main__':main()
