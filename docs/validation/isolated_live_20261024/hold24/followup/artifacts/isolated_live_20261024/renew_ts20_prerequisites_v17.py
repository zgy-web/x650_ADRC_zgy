"""One fresh hover20 -> audit -> probe30 -> audit; stop on any failure."""
import hashlib,json,os,subprocess,sys,time,traceback
from pathlib import Path

ROOT=Path('/root/autodl-tmp/safe-aerial-manipulator-testbed')
OUT=ROOT/'artifacts/isolated_live_20261024/prerequisites_v17'
sys.path.insert(0,str(ROOT))
from simulations.x650_ts20_hold_v1 import task


def ident(path):
    data=Path(path).read_bytes()
    return {'path':str(Path(path).resolve()),'bytes':len(data),'sha256':hashlib.sha256(data).hexdigest()}


def save(path,data):
    temporary=Path(str(path)+'.tmp')
    temporary.write_text(json.dumps(data,indent=2,allow_nan=False)+'\n')
    temporary.replace(path)


def diff(a,b,path=''):
    if isinstance(a,dict) and isinstance(b,dict):
        rows=[]
        for key in sorted(set(a)|set(b)):
            at=path+'/'+key
            if key not in a or key not in b:rows.append({'path':at,'old':a.get(key),'new':b.get(key)})
            else:rows.extend(diff(a[key],b[key],at))
        return rows
    return [] if a==b else [{'path':path,'old':a,'new':b}]


def current_sources(cfg):
    paths=set(task.source_files_for_task(cfg))
    paths.update((ROOT/'scripts').glob('*.py'))
    return {p.relative_to(ROOT).as_posix():task.sha(p) for p in sorted(paths)}


def assert_idle():
    if (ROOT/'.amsim/runtime.json').exists():
        raise RuntimeError('An existing simulation runtime must not be interrupted')


def run_checked(command,log,env):
    with log.open('x') as stream:
        result=subprocess.run(command,cwd=ROOT,env=env,stdout=stream,stderr=subprocess.STDOUT)
    return result.returncode


def main():
    assert_idle()
    OUT.mkdir(parents=True,exist_ok=False)
    status={'kind':'new_ts20_prerequisites_v17','complete':False,'started_unix':time.time(),
            'driver':ident(__file__),'stages':[],'no_automatic_retry':True,'starts_mpc_or_figure8':False}
    save(OUT/'status.json',status)
    env=os.environ.copy()
    env.pop('X650_EXECUTION_ADAPTER',None)
    env.update(OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1',MKL_NUM_THREADS='1',PYTHONUNBUFFERED='1')
    # Same system dependencies already selected by the canonical launcher.
    env['PYTHONPATH']='/usr/lib/python3/dist-packages:/root/autodl-tmp/amsim-venv/lib/python3.10/site-packages:'+env.get('PYTHONPATH','')
    baseline=ROOT/'artifacts/native_numerics_20261023'
    previous_hover=None
    first_sources=None
    try:
        for stage,old_name,duration in (('hover_folded','hover_audit_v3',20.),('joint_probe','probe_audit_v2',30.)):
            assert_idle()
            old_cert_path=baseline/old_name/'ts20_stage_certificate.json'
            old_cert=task.read(old_cert_path)
            template=Path(old_cert['source_run_directory'])/'controller_task.json'
            if old_cert['artifacts'][str(template)]!=task.sha(template):
                raise ValueError('Original stage task bytes differ from its archived certificate')
            old=task.read(template)
            cfg=json.loads(json.dumps(old))
            cfg['stage_evidence']={} if stage=='hover_folded' else {'hover_folded':str(previous_hover)}
            cfg=task.validate_task_config(cfg)
            changes=diff(old,cfg)
            expected=[] if stage=='hover_folded' else [{'path':'/stage_evidence/hover_folded',
                'old':old['stage_evidence']['hover_folded'],'new':str(previous_hover)}]
            if changes!=expected or cfg['stage']!=stage or cfg['controller']!='pid_ff' or cfg['duration_s']!=duration:
                raise ValueError('Only the new prerequisite certificate link may change')
            selected=OUT/(stage+'_task.json')
            with selected.open('x') as stream:json.dump(cfg,stream,indent=2);stream.write('\n')
            row={'stage':stage,'duration_s':duration,'complete':False,'old_task':ident(template),
                 'new_task':ident(selected),'old_certificate_lineage_only':ident(old_cert_path),
                 'config_differences':changes,'old_flight_evidence_reused':False}
            status['stages'].append(row);save(OUT/'config_manifest.json',status['stages'])
            _,provenance=task.preflight(cfg)
            save(OUT/(stage+'_preflight.json'),provenance)
            sources=current_sources(cfg)
            if first_sources is None:first_sources=sources;save(OUT/'sources_before.json',sources)
            elif sources!=first_sources:raise ValueError('Sources changed between prerequisite stages')
            launch_log=OUT/(stage+'_launch.log')
            code=run_checked([sys.executable,str(ROOT/'simulations/x650_ts20_hold_v1/launch.py'),
                              '--task',str(selected)],launch_log,env)
            row['launch_returncode']=code;row['launch_log']=ident(launch_log)
            lines=[line.split(' ',1)[1] for line in launch_log.read_text(errors='replace').splitlines()
                   if line.startswith('X650_TS20_LAUNCH ')]
            if len(lines)==1:
                launch=Path(lines[0]);row['launch_directory']=str(launch)
                launch_status=task.read(launch/'launch.json')
                row['run_directories']=launch_status.get('run_directories',[])
            save(OUT/'status.json',status)
            if code or len(lines)!=1:raise RuntimeError('Fresh '+stage+' launch failed: '+str(launch_log))
            if not launch_status.get('complete') or not launch_status.get('runtime_closed') or len(row['run_directories'])!=1:
                raise ValueError('Fresh stage lifecycle is incomplete')
            assert_idle()
            if current_sources(cfg)!=first_sources:raise ValueError('Sources changed during flight')
            run=Path(row['run_directories'][0]);audit_out=OUT/(stage+'_audit')
            audit_log=OUT/(stage+'_audit.log')
            row['audit_returncode']=run_checked([sys.executable,str(ROOT/'simulations/x650_ts20_hold_v1/audit.py'),
                '--run-dir',str(run),'--launch-dir',str(launch),'--output',str(audit_out)],audit_log,env)
            row['audit_log']=ident(audit_log)
            if row['audit_returncode']:raise RuntimeError('Full '+stage+' audit failed: '+str(audit_log))
            certificate=audit_out/'ts20_stage_certificate.json'
            document=task.read(certificate)
            if document['stage']!=stage or not document['complete'] or not document['passed']:
                raise ValueError('Incomplete fresh stage certificate')
            if document['ts20_operating_condition']!=provenance['ts20_operating_condition']:
                raise ValueError('Fresh certificate condition differs from preflight')
            if current_sources(cfg)!=first_sources:raise ValueError('Sources changed during audit')
            for path,sha in document['artifacts'].items():
                artifact=Path(path) if Path(path).is_absolute() else certificate.parent/path
                if task.sha(artifact)!=sha:raise ValueError('Fresh certificate artifact changed: '+path)
            row.update(complete=True,certificate=ident(certificate),window_sim_s=document['window_sim_s'],
                       timing=document['timing'],sources_unchanged=True)
            if stage=='hover_folded':previous_hover=certificate
            save(OUT/'status.json',status)
            print('PREREQUISITE_COMPLETE '+json.dumps(row),flush=True)
        status['complete']=True
    except BaseException as error:
        status.update(error=repr(error),traceback=traceback.format_exc())
        raise
    finally:
        status.update(finished_unix=time.time(),runtime_absent=not (ROOT/'.amsim/runtime.json').exists())
        if first_sources is not None:
            after=current_sources(cfg);save(OUT/'sources_after.json',after)
            status['sources_unchanged']=after==first_sources
            status['complete']=status['complete'] and status['sources_unchanged']
        save(OUT/'status.json',status)
    print('PREREQUISITES_PASSED '+str(OUT/'status.json'),flush=True)


if __name__=='__main__':main()
