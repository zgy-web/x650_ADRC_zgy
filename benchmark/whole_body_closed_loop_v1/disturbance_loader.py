"""Load the physical wrench plugin before arming its simulation-clock schedule."""
import json
import hashlib
import importlib.util
from pathlib import Path
import subprocess
import time

HERE=Path(__file__).resolve().parent


def _read_binding(path, model):
    spec = importlib.util.spec_from_file_location('_amsim_gz_entity_binding', HERE/'gz_world_entity_binding.py')
    verifier = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(verifier)
    return verifier.validate_binding_for_load(path, model)


def load_disturbance(out, model, schedule, duration, pump, *, binding=None):
    # Compatibility default retains the historical world/default + entity 1.
    # New physical acceptance must explicitly supply a fresh verified binding.
    verified = None
    world, entity = 'default', 1
    if binding is not None:
        verified = _read_binding(binding, model)
        world, entity = verified['world'], verified['world_id']
        for name in ('disturbance_config.txt','disturbance_load.log','disturbance_load_binding.json',
                     'applied_wrench.csv','applied_wrench.csv.status.json','disturbance_start.txt','disturbance_stop.txt'):
            if (out/name).exists():
                raise FileExistsError('Refusing to reuse existing verified disturbance evidence: '+name)
    config=out/'disturbance_config.txt'
    lines=[f'model={model}', 'link=base_link', f'output={out/"applied_wrench.csv"}',
           f'start_path={out/"disturbance_start.txt"}', f'stop_path={out/"disturbance_stop.txt"}',
           f'duration={duration:.17g}']
    for event in schedule:
        values=[event['start_s'],event['end_s'],*event['force_world_n'],*event['torque_world_nm']]
        lines.append('event='+','.join(format(float(v),'.17g') for v in values))
    config.write_text('\n'.join(lines)+'\n')
    request=('entity { id: '+str(entity)+' } plugins { name: "wbclosed::Disturbance" filename: '
             +json.dumps(str(HERE/'libdisturbance.so'))+' innerxml: '
             +json.dumps(f'<config_path>{config}</config_path>')+' }')
    with (out/'disturbance_load.log').open('w') as log:
        process=subprocess.Popen(['gz','service','--timeout','8000','-s',
            '/world/'+world+'/entity/system/add','--reptype','gz.msgs.Boolean',
            '--reqtype','gz.msgs.EntityPlugin_V','--req',request],stdout=log,stderr=subprocess.STDOUT)
        try:
            deadline=time.monotonic()+12
            while process.poll() is None:
                pump()
                if time.monotonic()>deadline:
                    raise RuntimeError('Physical disturbance plugin load timeout')
        finally:
            if process.poll() is None:
                process.terminate()
                try:
                    process.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    process.kill();process.wait(timeout=3)
    if process.returncode or 'data: true' not in (out/'disturbance_load.log').read_text():
        raise RuntimeError('Physical disturbance plugin did not load')
    if verified is not None:
        if _read_binding(binding, model) != verified:
            raise RuntimeError('Runtime binding changed while the load service was pending')
        # Acknowledgement is not a successful force/response experiment. The
        # caller must still verify READY, the full native wrench stream and
        # independent actual motion after the immutable schedule is armed.
        sha=lambda path: hashlib.sha256(Path(path).read_bytes()).hexdigest()
        result={'schema':'verified_disturbance_load_ack_v1', 'service_acknowledged':True,
                'force_experiment_complete':False, 'verified_binding':verified,
                'service':'/world/'+world+'/entity/system/add',
                'config_sha256':sha(config), 'library_sha256':sha(HERE/'libdisturbance.so'),
                'loader_source_sha256':sha(__file__), 'load_log_sha256':sha(out/'disturbance_load.log')}
        with (out/'disturbance_load_binding.json').open('x') as stream:
            stream.write(json.dumps(result,indent=2)+'\n')
        return result
