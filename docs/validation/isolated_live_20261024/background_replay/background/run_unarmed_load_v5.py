"""Owned, bounded PX4/Gazebo background only; never runs a flight scenario."""
from pathlib import Path
import json,os,subprocess,sys,time,traceback
R=Path('/root/autodl-tmp/safe-aerial-manipulator-testbed')
B=R/'artifacts/isolated_live_20261024'
O=B/'unarmed_load_v5'
sys.path[:0]=[str(R),str(R/'src')]
from simulations.pid_ff_wholebody_v1.launch import flight_lock
from amsim.runtime.process import status,stop

def save(name,data):
    (O/name).write_text(json.dumps(data,indent=2,allow_nan=False)+'\n')
def identity(pid):
    try:
        fields=Path(f'/proc/{pid}/stat').read_text().rsplit(')',1)[1].split()
        return {'pid':pid,'state':fields[0],'ppid':int(fields[1]),'starttime':fields[19]}
    except FileNotFoundError:return None
def children(root):
    rows={}
    for p in Path('/proc').iterdir():
        if p.name.isdigit():
            v=identity(int(p.name))
            if v:rows[v['pid']]=v
    keep={root};changed=True
    while changed:
        more={p for p,v in rows.items() if v['ppid'] in keep}-keep
        changed=bool(more);keep|=more
    return [rows[p] for p in sorted(keep) if p in rows]
def call(cmd,name,env,timeout):
    with (O/name).open('x') as log:
        return subprocess.run(cmd,cwd=R,env=env,stdout=log,stderr=subprocess.STDOUT,timeout=timeout,check=True)

O.mkdir(exist_ok=False)
(O/'plant_logs').mkdir(exist_ok=False)
state=None;owner=None;observer=None;known=[];report={'kind':'unarmed_background_load_v1','flight_run':False,'stage':'starting'}
with flight_lock(R):
    assert not (R/'.amsim/runtime.json').exists(), 'Existing runtime; do not take ownership'
    source=R/'runs/x650_isolated_launch_20261024_131614_20176/launch.json'
    original=json.loads(source.read_text())
    bg=','.join(map(str,original['execution_resources']['background']))
    env=os.environ.copy()
    env.update(AMSIM_VENV='/root/autodl-tmp/amsim-venv',AMSIM_PX4_DIR='/root/autodl-tmp/PX4-Autopilot',
        AMSIM_ROS_WS='/root/autodl-tmp/ros2_px4_ws',AMSIM_CHECK_PYTHON='/usr/bin/python3',
        HEADLESS='1',PX4_SIM_SPEED_FACTOR='1',PX4_GZ_MODEL_POSE='0,0,0.375,0,0,0',
        AMSIM_TS20_LOG_DIR=str(O/'plant_logs'),OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1',MKL_NUM_THREADS='1',
        ROS_DOMAIN_ID='0',GZ_PARTITION='x650_unarmed_load_v5_20261024')
    probe=['/usr/bin/python3',str(B/'probe_unarmed_background_ready.py'),'--repo-root',str(R)]
    try:
        call(['taskset','--cpu-list',bg,str(R/'scripts/amsim'),'start','--model',original['task']['model'],'--wait','75'],'start.log',env,90)
        state=status();assert state['running'] and state['ready']
        owner=identity(int(state['pid']));assert owner
        known=children(owner['pid'])
        report.update(owner=owner,runtime=state,owned_processes=known,background_cpu_list=bg,
            domain=0,partition=env['GZ_PARTITION'],startup_commands_only=True)
        call(probe+['--minimum-wall-s','10','--timeout-wall-s','60','--output',str(O/'ground_before.json')],'ground_before.log',env,70)
        ground=json.loads((O/'ground_before.json').read_text());assert ground['ground_disarmed_verified'] and ground['passed']
        with (O/'ground_window.log').open('x') as log:
            observer=subprocess.Popen(probe+['--minimum-wall-s','40','--timeout-wall-s','55','--output',str(O/'ground_window.json')],cwd=R,env=env,stdout=log,stderr=subprocess.STDOUT)
        report.update(stage='ready_for_replay',ready_monotonic_ns=time.monotonic_ns())
        save('status.json',report);(O/'READY').write_text('Ground verified; no flight scenario or command publisher invoked\n')
        deadline=time.monotonic()+75
        samples=[]
        while time.monotonic()<deadline and not (O/'RELEASE').exists():
            assert identity(owner['pid']) and status()['pid']==owner['pid']
            samples.append({'monotonic_ns':time.monotonic_ns(),'cpu_stat':Path('/sys/fs/cgroup/cpu.stat').read_text(),
                'cpu87_khz':Path('/sys/devices/system/cpu/cpu87/cpufreq/scaling_cur_freq').read_text().strip()})
            time.sleep(.5)
        save('resource_samples.json',samples)
        report['released']= (O/'RELEASE').exists()
        if observer is not None:
            observer.wait(timeout=60)
            assert observer.returncode==0
            report['ground_window_verified']=json.loads((O/'ground_window.json').read_text())['ground_disarmed_verified']
        report['stage']='completed_background_observation'
    except BaseException as exc:
        report.update(stage='failed',error=repr(exc),traceback=traceback.format_exc())
        raise
    finally:
        if observer is not None and observer.poll() is None:
            observer.terminate()
            try:observer.wait(timeout=5)
            except subprocess.TimeoutExpired:observer.kill();observer.wait(timeout=5)
        if state is not None:
            current=status();actual=identity(owner['pid']) if owner else None
            same=(actual and actual['starttime']==owner['starttime'] and current.get('pid')==state['pid'] and current.get('started_at')==state['started_at'])
            report['cleanup_owner_matches']=bool(same)
            if same:report['stop_result']=stop()
            else:report['cleanup_error']='Runtime identity changed; did not signal another owner'
        remaining=[]
        for previous in known:
            current=identity(previous['pid'])
            if current and current['starttime']==previous['starttime'] and current['state']!='Z':remaining.append(current)
        report.update(remaining_owned_processes=remaining,runtime_file_absent=not (R/'.amsim/runtime.json').exists(),ended_monotonic_ns=time.monotonic_ns())
        save('status.json',report)
