#!/usr/bin/env python3
"""Unarmed PX4/Gazebo clock-barrier probe; start/stop only its owned runtime."""
import argparse
import json
import os
import re
from pathlib import Path
import subprocess
import time


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--stepper',type=Path,required=True)
    a=p.parse_args();a.output.mkdir(parents=True,exist_ok=False)
    (a.output/'actuator').mkdir()
    os.environ['AMSIM_TS20_LOG_DIR']=str(a.output/'actuator')
    from amsim.modeling.manager import resolve,install,env_for
    from amsim.runtime.process import start,stop,status
    from amsim.paths import px4_dir
    if status().get('running'):raise RuntimeError('Existing runtime, no ownership')
    spec=resolve('amsim_urdf_x650_2link_ts20_damped_v1_flight');install(spec)
    report={'kind':'unarmed_synchronous_clock_probe_v1','armed_commands_sent':0,'passed':False,'events':[]}
    owned=False;helper=None
    try:
        report['runtime']=start(env_overrides=env_for(spec));owned=True
        helper=subprocess.Popen([str(a.stepper),'default','4000000'],stdin=subprocess.PIPE,stdout=subprocess.PIPE,text=True)
        def call(command):
            helper.stdin.write(command+'\n');helper.stdin.flush()
            row=json.loads(helper.stdout.readline());report['events'].append(row)
            if not row['ok']:raise RuntimeError(row)
            return row
        def listener(topic):
            cmd=[str(px4_dir()/'build/px4_sitl_default/bin/px4-listener'),topic,'1']
            result=subprocess.run(cmd,capture_output=True,text=True,timeout=5)
            if result.returncode:raise RuntimeError(result.stderr)
            return result.stdout
        initial=call('pause')
        first=listener('sensor_combined');status_first=listener('vehicle_status')
        time.sleep(1.0)
        after=call('status');second=listener('sensor_combined');status_second=listener('vehicle_status')
        report['paused_observation']={'first':initial,'after_1_wall_second':after,'px4_first':first,'px4_second':second,
            'status_first':status_first,'status_second':status_second}
        if after['sim_ns']!=initial['sim_ns']:raise RuntimeError('Simulation advanced while waiting')
        def timestamp(text):
            found=re.search(r'^\s+timestamp: (\d+)',text,re.M)
            if not found:raise RuntimeError('PX4 sensor timestamp unavailable')
            return int(found[1])
        if timestamp(first)!=timestamp(second) or timestamp(second)*1000!=after['sim_ns']:
            raise RuntimeError('PX4 clock did not freeze at Gazebo boundary')
        report['px4_clock_frozen_verified']=True
        for _ in range(6):
            row=call('step 10')
            if row['sim_ns']-row['before_sim_ns']!=40000000:raise RuntimeError('Wrong tick interval')
            sensor=listener('sensor_combined')
            if timestamp(sensor)*1000!=row['sim_ns']:raise RuntimeError('PX4 sensor clock did not follow step')
            report['events'].append({'px4_sensor_after_step':sensor,'px4_clock_matches':True})
            time.sleep(.12)
            if call('status')['sim_ns']!=row['sim_ns']:raise RuntimeError('Clock drifted between steps')
        report['paused_final_status']=listener('vehicle_status')
        call('resume');time.sleep(.15);resumed=call('status')
        if resumed['sim_ns']<=row['sim_ns']:raise RuntimeError('Resume failed')
        call('stop');helper.wait(timeout=3)
        report['passed']=True
    finally:
        if helper and helper.poll() is None:
            helper.terminate();helper.wait(timeout=5)
        if owned:report['cleanup']=stop()
        (a.output/'report.json').write_text(json.dumps(report,indent=2))
        print(json.dumps(report,indent=2),flush=True)


if __name__=='__main__':main()
