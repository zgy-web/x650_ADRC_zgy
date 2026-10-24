from pathlib import Path
import json,subprocess,time,sys,hashlib
B=Path('/root/autodl-tmp/safe-aerial-manipulator-testbed/artifacts/isolated_live_20261024')
O=B/'unarmed_load_v5';S=B/'identity_qp_scratch_v1'
coord={'kind':'controlled_no_command_A_B_A2','background':str(O),'cpu':87,'backend':'original'}
try:
 deadline=time.monotonic()+180
 while not (O/'READY').exists():
  f=O/'status.json'
  if f.exists() and json.loads(f.read_text()).get('stage')=='failed':raise RuntimeError('Background failed before READY')
  if time.monotonic()>deadline:raise TimeoutError('Background did not reach READY')
  time.sleep(.5)
 status=json.loads((O/'status.json').read_text());ground=json.loads((O/'ground_before.json').read_text())
 assert status['stage']=='ready_for_replay' and ground['passed'] and ground['ground_disarmed_verified']
 coord['ground_before_sha256']=hashlib.sha256((O/'ground_before.json').read_bytes()).hexdigest()
 print('Background READY verified: beginning B',flush=True)
 output=S/'environment_B_unarmed_v5'
 with (S/'environment_B_unarmed_v5.log').open('x') as log:
  result=subprocess.run(['taskset','-c','87',sys.executable,str(S/'paced_uninstrumented.py'),'--backend','original','--output',str(output)],stdout=log,stderr=subprocess.STDOUT)
 coord['B_exit']=result.returncode;coord['B_output']=str(output/'report.json')
 (O/'RELEASE').write_text(str(output/'report.json')+'\n')
 print('B ended; RELEASE written; waiting for verified cleanup',flush=True)
 deadline=time.monotonic()+60
 while True:
  exitfile=B/'unarmed_load_v5.exit'
  state=json.loads((O/'status.json').read_text())
  if exitfile.exists() and state.get('ended_monotonic_ns'):
   assert exitfile.read_text().strip()=='0', 'Background nonzero exit'
   assert state.get('runtime_file_absent') is True and state.get('remaining_owned_processes')==[]
   coord['background_final']=state;break
  if time.monotonic()>deadline:raise TimeoutError('Background cleanup unconfirmed; A2 not started')
  time.sleep(.5)
 print('Background cleanup verified: beginning A2',flush=True)
 output=S/'environment_A2_no_sim'
 with (S/'environment_A2_no_sim.log').open('x') as log:
  result=subprocess.run(['taskset','-c','87',sys.executable,str(S/'paced_uninstrumented.py'),'--backend','original','--output',str(output)],stdout=log,stderr=subprocess.STDOUT)
 coord['A2_exit']=result.returncode;coord['A2_output']=str(output/'report.json')
 print('A2 ended',flush=True)
except BaseException as exc:
 coord['error']=repr(exc);print(repr(exc),flush=True)
finally:
 (S/'environment_coordinator_v5.json').write_text(json.dumps(coord,indent=2)+'\n')