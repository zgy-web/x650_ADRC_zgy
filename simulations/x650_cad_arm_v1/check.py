#!/usr/bin/env python3
"""Run a 10-second fixed-base physical test and audit motion and loop closure.

Usage: python check.py RUN_DIRECTORY
Requires a world generated with prepare.py --exercise. Does not start PX4.
"""
import argparse
import csv
import hashlib
import json
import math
import os
from pathlib import Path
import re
import signal
import subprocess
import uuid


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('directory', type=Path)
    args = p.parse_args()
    directory = args.directory.resolve(strict=True)
    metadata = json.loads((directory / 'preparation.json').read_text())
    if not metadata['exercise']:
        p.error('Use prepare.py --exercise for the automated motion test')
    if (directory / 'motion.csv').exists() or (directory / 'check.json').exists():
        p.error('Generate a new run directory; previous evidence must not be overwritten')
    world = directory / 'world.sdf'
    validation = subprocess.run(['gz','sdf','-k',str(world)],capture_output=True,text=True,timeout=30)
    result = dict(ok=False, sdf_valid=validation.returncode == 0 and 'Valid.' in validation.stdout,
                  validation_output=validation.stdout+validation.stderr,
                  world_sha256=hashlib.sha256(world.read_bytes()).hexdigest(),
                  limits={'closure_m':.0005,'axis_rad':.005,'settled_tracking_rad':.05},
                  scope='10 s fixed-base force-driven arm test; no collision or flight validation')
    if not result['sdf_valid']:
        (directory / 'check.json').write_text(json.dumps(result,indent=2)+'\n')
        raise SystemExit(1)
    env = os.environ.copy()
    env['GZ_PARTITION'] = 'x650-cad-check-' + uuid.uuid4().hex
    with (directory / 'gazebo.log').open('x') as log:
        proc = subprocess.Popen(['gz','sim','-s','-r','--physics-engine',f"gz-physics-{metadata.get('engine','dartsim')}-plugin",'--iterations',str(round(10/metadata['step_s'])),str(world)],
            env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        try:
            result['returncode'] = proc.wait(timeout=55)
        except subprocess.TimeoutExpired:
            os.killpg(proc.pid, signal.SIGTERM)
            try:
                proc.wait(timeout=3)
            except subprocess.TimeoutExpired:
                os.killpg(proc.pid, signal.SIGKILL)
                proc.wait()
            result['timed_out'] = True
    log = (directory / 'gazebo.log').read_text(errors='replace')
    result['loader_errors'] = [x for x in log.splitlines() if re.search(r'\[(?:Err|Error)\]|Failed to load|Error Code',x)]
    rows = []
    if (directory / 'motion.csv').exists():
        with (directory / 'motion.csv').open() as f:
            rows = [{k:float(v) for k,v in row.items()} for row in csv.DictReader(f)]
    finite = bool(rows) and all(math.isfinite(v) for row in rows for v in row.values())
    result.update(samples=len(rows), finite=finite)
    if finite:
        result.update(duration_s=rows[-1]['t'],max_closure_m=max(r['closure_m'] for r in rows),
                      max_axis_rad=max(r['axis_rad'] for r in rows),
                      joint_motion_rad=[max(r['q'+str(i)] for r in rows)-min(r['q'+str(i)] for r in rows) for i in range(2)])
        settled = [r for r in rows if any(a<=r['t']<=b for a,b in ((1.5,2),(3.5,4),(5.5,6),(7.5,8),(9.5,10)))]
        result['settled_max_error_rad'] = [max(abs(r[f'q{i}']-r[f'target{i}']) for r in settled) for i in range(2)]
        result['joint_bounds_ok'] = all(-.005<=r['q0']<=1.955 and -.005<=r['q1']<=.785 for r in rows)
        result['ok'] = bool(metadata['closure_enabled'] and result.get('returncode')==0 and not result['loader_errors']
            and len(rows)>=800 and rows[-1]['t']>=9.98 and result['max_closure_m']<=.0005
            and result['max_axis_rad']<=.005 and min(result['joint_motion_rad'])>.1
            and max(result['settled_max_error_rad'])<=.05 and result['joint_bounds_ok'])
    (directory / 'check.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result,indent=2))
    raise SystemExit(0 if result['ok'] else 1)


if __name__ == '__main__':
    main()
