from pathlib import Path
import json,hashlib,subprocess,time,traceback
R=Path('/root/autodl-tmp/safe-aerial-manipulator-testbed');B=R/'artifacts/isolated_live_20261024'
status={'flight_started':False,'stage':'waiting_formal_qualification_v5'}
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def save(**kw):
 status.update(kw);(B/'prepare_hold_v24_status.json').write_text(json.dumps(status,indent=2))
try:
 save()
 while not (B/'preparation_qualify_v5.exit').exists():time.sleep(3)
 assert (B/'preparation_qualify_v5.exit').read_text().strip()=='0','Formal qualification failed'
 assert (B/'execution_v30_build.exit').read_text().strip()=='0'
 deployment=json.loads((B/'deployment_v30.json').read_text())
 for rel,item in deployment['files'].items():assert sha(R/rel)==item['sha256'],rel
 proof=B/'preparation_software_v5/native_acceptance.json';p=json.loads(proof.read_text());assert p['pass'] and p['no_flight']
 assert p['streamed_evidence']['matrix']['rows']==30 and p['streamed_evidence']['recorded']['rows']==388
 for item in p['streamed_evidence'].values():assert item['passed'] and item['complete_gc_trace_verified']
 old=json.loads((B/'quiet_task_cpp_v9.json').read_text());cfg=json.loads(json.dumps(old))
 cfg['numerical_backend']['software_proof']=str(proof);cfg['numerical_backend']['software_proof_sha256']=sha(proof)
 assert cfg['mpc_config']==old['mpc_config'] and cfg['ts20_inputs']==old['ts20_inputs']
 task=B/'quiet_task_cpp_v10.json'
 with task.open('x') as f:json.dump(cfg,f,indent=2);f.write('\n')
 protocol=R/'configs/experiments/x650_isolated_live_v1.json';previous=json.loads(protocol.read_text());assert previous['execution_revision']==16
 current=json.loads(json.dumps(previous));current.update(execution_revision=17,frozen_hold_task_sha256=sha(task),
  native_preparation='native-only packed future MIT arrays and matrix rows; actual candidate propagation and all extrema retained; same C++ physical libraries and independent reference predictor; new source-bound full regression/30 matrix/388 historical qualification',
  live_diagnostic_snapshot='private zero-FF live policy: candidate quality and actual command checked before publication; complete FF diagnostics and owned copy finish after publication and before synchronous immutable pickle or any next solve; completion failure locks worker and retains explicit incomplete-recording evidence',
  failed_recovery_handoff='failed runs release and verify the sole C++ writer and zero consumption, then require new PX4 ACK and fresh autonomous-Hold status within 350ms before folding; otherwise skip fold and use original LAND; unverified writer exit retains receive-only pumping with no legacy joint publisher; no hardware qualification')
 for key in ('output_period_s','command_age_max_s','control_interval_max_s','source_wall_age_max_s','output_interval_max_s','cpu_roles'):
  assert current[key]==previous[key]
 protocol.write_text(json.dumps(current,indent=2)+'\n');(B/'execution_protocol_v17.json').write_bytes(protocol.read_bytes())
 transport=json.loads((B/'transport_v11/report.json').read_text());assert transport['dispatcher_sha256']==sha(Path(transport['dispatcher_binary']))
 assert not (R/'.amsim/runtime.json').exists()
 source=(B/'hold_v23.sh').read_text().replace('quiet_task_cpp_v9','quiet_task_cpp_v10')
 (B/'hold_v24.sh').write_text(source);(B/'hold_v24_preflight.sh').write_text(source.replace(' --transport-proof',' --dry-run --transport-proof'))
 save(stage='ground_preflight',task_sha256=sha(task),protocol_sha256=sha(protocol),native_proof_sha256=sha(proof))
 with (B/'hold_v24_preflight.log').open('x') as log:subprocess.run(['/bin/bash',str(B/'hold_v24_preflight.sh')],stdout=log,stderr=subprocess.STDOUT,check=True)
 progress=(B/'progress_v7.py').read_text().replace('progress_v7','progress_v8').replace('hold_v23','hold_v24').replace('hold23','hold24').replace('Hold23','Hold24').replace('normal_sequence_v7','normal_sequence_v8').replace('quiet_task_cpp_v9','quiet_task_cpp_v10')
 (B/'progress_v8.py').write_text(progress);env=source.split('set -e')[0]
 (B/'progress_v8.sh').write_text(env+'\n/usr/bin/python3 '+str(B/'progress_v8.py')+' >'+str(B/'progress_v8.log')+' 2>&1\n')
 observer=(B/'hold_v23_resource_observer.py').read_text().replace('hold_v23','hold_v24');(B/'hold_v24_resource_observer.py').write_text(observer)
 subprocess.run(['screen','-dmS','flyarm-hold24-resources','/usr/bin/python3',str(B/'hold_v24_resource_observer.py')],check=True)
 runner=B/'hold_v24_runner.sh';runner.write_text('#!/bin/bash\n/bin/bash '+str(B/'hold_v24.sh')+' >'+str(B/'hold_v24.log')+' 2>&1\nprintf "%s\\n" "$?" >'+str(B/'hold_v24.exit')+'\n/bin/bash '+str(B/'progress_v8.sh')+'\n')
 subprocess.run(['screen','-dmS','flyarm-hold-v24','/bin/bash',str(runner)],check=True)
 save(stage='fresh_hold24_running',flight_started=True,ordinary_allowed_only_after_full_hold_audit=True,fast_started=False)
except BaseException as exc:
 save(stage='stopped',error=repr(exc),traceback=traceback.format_exc())
 raise
