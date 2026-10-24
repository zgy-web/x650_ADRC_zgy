from pathlib import Path
import json, hashlib, subprocess, traceback

R=Path('/root/autodl-tmp/safe-aerial-manipulator-testbed')
B=R/'artifacts/isolated_live_20261024'
status={'flight_started':False,'stage':'current_source_preflight','fast_started':False}
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def save(**kw):
    status.update(kw);(B/'prepare_hold_v25_status.json').write_text(json.dumps(status,indent=2))
def new(p,s):
    with p.open('x') as f:f.write(s)
try:
    assert not (R/'.amsim/runtime.json').exists()
    assert (B/'hold_v24.exit').read_text().strip()=='1'
    assert (B/'execution_v31_tests.exit').read_text().strip()=='0'
    assert (B/'execution_v31_ground_tests.exit').read_text().strip()=='0'
    tests=(B/'execution_v31_tests.log').read_text()
    for count in (11,7,3):assert f'Ran {count} tests' in tests
    for rel,entry in json.loads((B/'deployment_v31.json').read_text())['files'].items():
        assert sha(R/rel)==entry['sha256'],rel
    task=B/'quiet_task_cpp_v11.json'
    assert sha(task)=='c04a3bbf484b4770d3c0425ec4b2613dc12008761f02e429187979c07534a181'
    protocol=R/'configs/experiments/x650_isolated_live_v1.json'
    assert sha(protocol)=='49fe4e44f605c5ded701296bb7eeb1f2d867aae9ce561a205a2c10f72dc3234b'
    source=(B/'hold_v24_ready2.sh').read_text()
    new(B/'hold_v25.sh',source)
    new(B/'hold_v25_preflight.sh',source.replace(' --transport-proof',' --dry-run --transport-proof'))
    save(task_sha256=sha(task),protocol_sha256=sha(protocol),algorithm_parameters_and_deadlines_unchanged=True)
    with (B/'hold_v25_preflight.log').open('x') as log:
        result=subprocess.run(['/bin/bash',str(B/'hold_v25_preflight.sh')],stdout=log,stderr=subprocess.STDOUT)
    new(B/'hold_v25_preflight.exit',str(result.returncode)+'\n')
    assert result.returncode==0,'Preflight failed; no new flight'
    assert not (R/'.amsim/runtime.json').exists()
    progress=(B/'progress_v8.py').read_text().replace('progress_v8','progress_v9').replace('hold_v24','hold_v25').replace('hold24','hold25').replace('Hold24','Hold25').replace('normal_sequence_v8','normal_sequence_v9')
    needle="call('hold_v25_plot',"
    extra="call('hold_v25_source_activity_audit',[str(B/'verify_source_activity_v1.py'),'--run',str(run),'--output',str(B/'hold_v25_source_activity_audit.json')])\n    "
    assert needle in progress
    progress=progress.replace(needle,extra+needle)
    new(B/'progress_v9.py',progress)
    new(B/'progress_v9.sh',source.split('set -e')[0]+'\n/usr/bin/python3 '+str(B/'progress_v9.py')+' >'+str(B/'progress_v9.log')+' 2>&1\n')
    new(B/'hold_v25_resource_observer.py',(B/'hold_v24_resource_observer.py').read_text().replace('hold_v24','hold_v25'))
    runner=B/'hold_v25_runner.sh'
    new(runner,'#!/bin/bash\n/bin/bash '+str(B/'hold_v25.sh')+' >'+str(B/'hold_v25.log')+' 2>&1\nprintf "%s\\n" "$?" >'+str(B/'hold_v25.exit')+'\n/bin/bash '+str(B/'progress_v9.sh')+'\n')
    subprocess.run(['screen','-dmS','flyarm-hold25-resources','/usr/bin/python3',str(B/'hold_v25_resource_observer.py')],check=True)
    subprocess.run(['screen','-dmS','flyarm-hold-v25','/bin/bash',str(runner)],check=True)
    save(flight_started=True,stage='fresh_hold25_running',ordinary_requires_full_hold_and_source_activity_audits=True)
except BaseException as exc:
    save(stage='stopped',error=repr(exc),traceback=traceback.format_exc())
    raise
