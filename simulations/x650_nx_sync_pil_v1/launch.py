#!/usr/bin/env python3
"""Launch exactly one owned synchronous PIL simulation, never real hardware."""
import argparse
import json
import os
from pathlib import Path
import subprocess
import sys
import zipfile

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT));sys.path.insert(0,str(ROOT/'src'))
from simulations.pid_ff_wholebody_v1.launch import flight_lock
from simulations.x650_nx_sync_pil_v1.run_case import prepare,read,sha


def sources(provenance):
    paths=set(provenance['source_files_sha256'])
    paths.update(str(p.relative_to(ROOT)).replace('\\','/') for p in (ROOT/'simulations/x650_nx_sync_pil_v1').glob('*.py'))
    paths.update(('scripts/x650_pil_transport.py','scenarios/x650_nx_sync_pil_v1.yaml'))
    return {p:sha(ROOT/p) for p in sorted(paths)}


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--plan',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--dry-run',action='store_true')
    a=p.parse_args();plan=read(a.plan);cfg,model,provenance,manifest=prepare(plan)
    if a.dry_run:
        print(json.dumps({'ready':True,'config':cfg,'provenance':provenance},indent=2));return
    with flight_lock(ROOT):
        if (ROOT/'.amsim/runtime.json').exists():raise RuntimeError('Existing simulation runtime')
        a.output.mkdir(parents=True,exist_ok=False);logs=a.output/'actuator';logs.mkdir()
        env=os.environ.copy();env.update(FLYARM_NX_SYNC_PLAN=str(a.plan.resolve()),AMSIM_TS20_LOG_DIR=str(logs.resolve()),
            PYTHONUNBUFFERED='1',AMSIM_SCENARIO_PYTHON='/usr/bin/python3',AMSIM_CHECK_PYTHON='/usr/bin/python3',
            PX4_GZ_MODEL_POSE=','.join(str(x) for x in cfg['spawn_pose_enu_xyz_rpy']),
            OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1',MKL_NUM_THREADS='1')
        before=set((ROOT/'runs').glob(cfg['scenario']+'_*'))
        document={'kind':'x650_nx_sync_pil_launch_v1','complete':False,'plan':plan,'provenance':provenance,'logs':str(logs)}
        before_sources=sources(provenance)
        (a.output/'source_manifest.json').write_text(json.dumps(before_sources,indent=2))
        with zipfile.ZipFile(a.output/'sources.zip','x',zipfile.ZIP_DEFLATED) as z:
            for rel in before_sources:z.write(ROOT/rel,rel)
        document['stepper_sha256']=sha(plan['stepper'])
        try:
            with (a.output/'flight.log').open('x') as log:
                r=subprocess.run([str(ROOT/'scripts/amsim'),'run',cfg['scenario'],'--headless'],env=env,cwd=ROOT,stdout=log,stderr=subprocess.STDOUT)
            created=set((ROOT/'runs').glob(cfg['scenario']+'_*'))-before
            document.update(exit_code=r.returncode,run_directories=[str(x) for x in sorted(created)])
            if len(created)==1:
                result=next(iter(created))/'result.json'
                if result.exists():document['result']=read(result)
            document['complete']=r.returncode==0 and len(created)==1 and document.get('result',{}).get('success') is True
            if not document['complete']:raise RuntimeError('New synchronous PIL did not complete; inspect retained flight.log')
        except BaseException as e:document['error']=repr(e);raise
        finally:
            after_sources=sources(provenance)
            (a.output/'source_manifest_after.json').write_text(json.dumps(after_sources,indent=2))
            document['sources_unchanged']=before_sources==after_sources
            document['stepper_unchanged']=sha(plan['stepper'])==document['stepper_sha256']
            document['complete']=bool(document['complete'] and document['sources_unchanged'] and document['stepper_unchanged'])
            document['runtime_closed']=not (ROOT/'.amsim/runtime.json').exists()
            (a.output/'launch.json').write_text(json.dumps(document,indent=2))
            print(json.dumps(document,indent=2),flush=True)
        if not document['complete']:raise RuntimeError('Synchronous launch incomplete or sources changed')


if __name__=='__main__':main()
