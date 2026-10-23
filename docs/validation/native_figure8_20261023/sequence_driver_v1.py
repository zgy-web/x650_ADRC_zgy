import os,sys,pathlib,json,subprocess,traceback,time
r=pathlib.Path('/root/autodl-tmp/safe-aerial-manipulator-testbed');o=r/'artifacts/native_figure8_20261023';os.chdir(r);sys.path.insert(0,str(r))
from simulations.x650_ts20_figure8_v1 import task
from simulations.x650_ts20_figure8_v1.launch import verify_platform
e=os.environ.copy();e.update(OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1',MKL_NUM_THREADS='1',PYTHONPATH='/usr/lib/python3/dist-packages:/root/autodl-tmp/amsim-venv/lib/python3.10/site-packages')
proof=r/'artifacts/native_numerics_20261023/software_v4/software_acceptance.json';proof_sha='f12af2fadef68e8650972d909b0c6c3f164954b2910b7a37ea57a8d893842430'
status={'complete':False,'phase':'normal','stages':[],'started_unix':time.time()}
def save():
 (o/'sequence_status.json').write_text(json.dumps(status,indent=2)+'\n');print(json.dumps(status),flush=True)
def run(args,name):
 with (o/name).open('x') as f:p=subprocess.run(args,cwd=r,env=e,stdout=f,stderr=subprocess.STDOUT)
 if p.returncode:raise RuntimeError(name+' failed: '+str(p.returncode))
try:
 cfg=task.read(o/'normal_task_v1.json')
 for stage in ('normal','fast'):
  if stage=='fast':
   cfg=task.build_task_config(stage,cfg['frozen_hold'],{**cfg['trajectory_gates'],'normal':{'path':str(cert),'sha256':task.sha(cert)}})
   (o/'fast_task_v1.json').write_text(json.dumps(cfg,indent=2)+'\n')
  status['phase']=stage;save();config=o/(stage+'_task_v1.json')
  run(['/usr/bin/python3','simulations/x650_ts20_figure8_v1/launch.py','--task',str(config),'--software-proof',str(proof),'--software-sha256',proof_sha],stage+'_launch.log')
  launch=pathlib.Path(next(s.split(' ',1)[1] for s in (o/(stage+'_launch.log')).read_text().splitlines() if s.startswith('X650_TS20_LAUNCH ')))
  flight=pathlib.Path(task.read(launch/'launch.json')['run_directories'][0]);status.update(phase=stage+'_audit',active_launch=str(launch),active_run=str(flight));save()
  run(['/usr/bin/python3','simulations/x650_ts20_figure8_v1/audit.py','--run-dir',str(flight),'--launch-dir',str(launch),'--output',str(o/(stage+'_audit_v1'))],stage+'_audit.log')
  cert=o/(stage+'_audit_v1')/'figure8_stage_certificate.json';verified=verify_platform(launch,flight,expected_software_sha256=proof_sha)
  doc=task.read(cert);assert doc['passed'] is True
  status['stages'].append({'stage':stage,'run':str(flight),'launch':str(launch),'certificate':str(cert),'certificate_sha256':task.sha(cert),'platform_verification':verified,'endpoint':doc['result']['endpoint'],'tracking_endpoint':doc['result']['tracking_endpoint'],'timing':doc['timing']});save()
 status.update(complete=True,phase='complete',finished_unix=time.time());save()
except BaseException as exc:
 status.update(error=repr(exc),traceback=traceback.format_exc(),finished_unix=time.time());save();raise
