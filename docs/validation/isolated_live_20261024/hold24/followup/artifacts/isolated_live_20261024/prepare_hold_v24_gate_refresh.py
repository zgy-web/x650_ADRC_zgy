from pathlib import Path
import hashlib
import json
import subprocess
import time
import traceback

R = Path('/root/autodl-tmp/safe-aerial-manipulator-testbed')
B = R / 'artifacts/isolated_live_20261024'
status = {'flight_started': False, 'stage': 'waiting_current_source_prerequisites', 'fast_started': False}

def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()

def save(**kw):
    status.update(kw)
    (B / 'prepare_hold_v24_gate_refresh_status.json').write_text(json.dumps(status, indent=2))

def exclusive(path, value):
    with path.open('x') as stream:
        stream.write(value)

try:
    save()
    prerequisite_file = B / 'prerequisites_v17/status.json'
    while True:
        if prerequisite_file.exists():
            prerequisites = json.loads(prerequisite_file.read_text())
            assert not prerequisites.get('error'), prerequisites.get('error')
            if prerequisites.get('complete'):
                break
        time.sleep(3)
    assert prerequisites['runtime_absent'] and prerequisites['sources_unchanged']
    stages = {item['stage']: item for item in prerequisites['stages']}
    assert set(stages) == {'hover_folded', 'joint_probe'}
    certificates = {}
    for name, item in stages.items():
        assert item['complete'] and item['sources_unchanged']
        assert item['launch_returncode'] == item['audit_returncode'] == 0
        cert = item['certificate']
        path = Path(cert['path'])
        assert sha(path) == cert['sha256']
        content = json.loads(path.read_text())
        assert content['complete'] and content['passed'] and content['stage'] == name
        certificates[name] = str(path)
    assert not (R / '.amsim/runtime.json').exists()
    assert not (B / 'hold_v24.log').exists()
    assert not (B / 'hold_v24.exit').exists()
    for rel, item in json.loads((B / 'deployment_v30.json').read_text())['files'].items():
        assert sha(R / rel) == item['sha256'], rel
    for name in ('execution_v30_build.exit', 'preparation_qualify_v5.exit'):
        assert (B / name).read_text().strip() == '0'
    old_path = B / 'quiet_task_cpp_v10.json'
    assert sha(old_path) == '21185ea26e345968777b1fc3206fb45f0a3e0bd98e79abaa85ee63732736b63e'
    old = json.loads(old_path.read_text())
    task_value = json.loads(json.dumps(old))
    task_value['stage_evidence'] = certificates
    assert {k: v for k, v in task_value.items() if k != 'stage_evidence'} == {k: v for k, v in old.items() if k != 'stage_evidence'}
    task = B / 'quiet_task_cpp_v11.json'
    exclusive(task, json.dumps(task_value, indent=2) + '\n')
    protocol = R / 'configs/experiments/x650_isolated_live_v1.json'
    assert sha(protocol) == 'bc5d25bd1097b9b0edeba837dbe5e91e29363206bdf781a437437c481ccd55ba'
    previous = json.loads(protocol.read_text())
    assert previous['execution_revision'] == 17
    current = json.loads(json.dumps(previous))
    current['frozen_hold_task_sha256'] = sha(task)
    assert {k: v for k, v in current.items() if k != 'frozen_hold_task_sha256'} == {k: v for k, v in previous.items() if k != 'frozen_hold_task_sha256'}
    protocol.write_text(json.dumps(current, indent=2) + '\n')
    exclusive(B / 'execution_protocol_v17_ready2.json', protocol.read_text())
    source = (B / 'hold_v23.sh').read_text().replace('quiet_task_cpp_v9', 'quiet_task_cpp_v11')
    exclusive(B / 'hold_v24_ready2.sh', source)
    exclusive(B / 'hold_v24_preflight_v2.sh', source.replace(' --transport-proof', ' --dry-run --transport-proof'))
    save(stage='current_source_ground_preflight', prerequisites_sha256=sha(prerequisite_file), stage_evidence=certificates,
         task_path=str(task), task_sha256=sha(task), protocol_sha256=sha(protocol), algorithm_or_parameter_changes=False)
    with (B / 'hold_v24_preflight_v2.log').open('x') as log:
        result = subprocess.run(['/bin/bash', str(B / 'hold_v24_preflight_v2.sh')], stdout=log, stderr=subprocess.STDOUT)
    exclusive(B / 'hold_v24_preflight_v2.exit', str(result.returncode) + '\n')
    assert result.returncode == 0, 'Fresh current-source preflight failed; no flight started'
    assert not (R / '.amsim/runtime.json').exists()
    progress = (B / 'progress_v7.py').read_text().replace('progress_v7', 'progress_v8').replace('hold_v23', 'hold_v24').replace('hold23', 'hold24').replace('Hold23', 'Hold24').replace('normal_sequence_v7', 'normal_sequence_v8').replace('quiet_task_cpp_v9', 'quiet_task_cpp_v11')
    exclusive(B / 'progress_v8.py', progress)
    env = source.split('set -e')[0]
    exclusive(B / 'progress_v8.sh', env + '\n/usr/bin/python3 ' + str(B / 'progress_v8.py') + ' >' + str(B / 'progress_v8.log') + ' 2>&1\n')
    observer = (B / 'hold_v23_resource_observer.py').read_text().replace('hold_v23', 'hold_v24')
    exclusive(B / 'hold_v24_resource_observer.py', observer)
    runner = B / 'hold_v24_runner.sh'
    exclusive(runner, '#!/bin/bash\n/bin/bash ' + str(B / 'hold_v24_ready2.sh') + ' >' + str(B / 'hold_v24.log') + ' 2>&1\nprintf "%s\\n" "$?" >' + str(B / 'hold_v24.exit') + '\n/bin/bash ' + str(B / 'progress_v8.sh') + '\n')
    subprocess.run(['screen', '-dmS', 'flyarm-hold24-resources', '/usr/bin/python3', str(B / 'hold_v24_resource_observer.py')], check=True)
    subprocess.run(['screen', '-dmS', 'flyarm-hold-v24', '/bin/bash', str(runner)], check=True)
    save(stage='fresh_hold24_running', flight_started=True, ordinary_allowed_only_after_full_hold_audit=True)
except BaseException as exc:
    save(stage='stopped', error=repr(exc), traceback=traceback.format_exc())
    raise
