"""Load the physical wrench plugin before arming its simulation-clock schedule."""
import json
from pathlib import Path
import subprocess
import time

HERE=Path(__file__).resolve().parent


def load_disturbance(out, model, schedule, duration, pump):
    config=out/'disturbance_config.txt'
    lines=[f'model={model}', 'link=base_link', f'output={out/"applied_wrench.csv"}',
           f'start_path={out/"disturbance_start.txt"}', f'stop_path={out/"disturbance_stop.txt"}',
           f'duration={duration:.17g}']
    for event in schedule:
        values=[event['start_s'],event['end_s'],*event['force_world_n'],*event['torque_world_nm']]
        lines.append('event='+','.join(format(float(v),'.17g') for v in values))
    config.write_text('\n'.join(lines)+'\n')
    request=('entity { id: 1 } plugins { name: "wbclosed::Disturbance" filename: '
             +json.dumps(str(HERE/'libdisturbance.so'))+' innerxml: '
             +json.dumps(f'<config_path>{config}</config_path>')+' }')
    with (out/'disturbance_load.log').open('w') as log:
        process=subprocess.Popen(['gz','service','--timeout','8000','-s',
            '/world/default/entity/system/add','--reptype','gz.msgs.Boolean',
            '--reqtype','gz.msgs.EntityPlugin_V','--req',request],stdout=log,stderr=subprocess.STDOUT)
        deadline=time.monotonic()+12
        while process.poll() is None:
            pump()
            if time.monotonic()>deadline:
                process.terminate();process.wait(timeout=3)
                raise RuntimeError('Physical disturbance plugin load timeout')
    if process.returncode or 'data: true' not in (out/'disturbance_load.log').read_text():
        raise RuntimeError('Physical disturbance plugin did not load')
