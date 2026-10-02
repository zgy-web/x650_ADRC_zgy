"""Linux-only supervisor. Owns only processes it starts; no global pkill."""
import os
from pathlib import Path
import shutil
import signal
import socket
import subprocess
import time


def main():
    px4 = Path(os.environ.get('PX4_DIR', str(Path.home()/'PX4-Autopilot')))
    models = px4/'Tools/simulation/gz/models'
    binary = px4/'build/px4_sitl_default/bin/px4'
    for path in (binary, models/'x500_arm2dof/model.sdf'):
        if not path.is_file():
            raise RuntimeError(f'Missing: {path}')
    for cmd in ('MicroXRCEAgent', 'ros2', 'gz'):
        if not shutil.which(cmd):
            raise RuntimeError(f'Not found: {cmd}')
    for port in (8888, 18570):
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
            sock.bind(('0.0.0.0', port))
    for process in ('px4', 'MicroXRCEAgent'):
        if subprocess.run(['pgrep', '-x', process], stdout=subprocess.DEVNULL).returncode == 0:
            raise RuntimeError(f'{process} already running. Close the old session first.')
    existing = subprocess.run(['pgrep', '-af', 'gz sim|gz-sim'], capture_output=True, text=True)
    if existing.returncode == 0:
        raise RuntimeError('Old Gazebo processes exist; close them before starting:\n'+existing.stdout)
    env = os.environ.copy()
    env.update(PX4_SYS_AUTOSTART='4001', PX4_SIM_MODEL='x500_arm2dof')
    env['GZ_SIM_RESOURCE_PATH'] = str(models)+':'+str(px4/'Tools/simulation/gz/worlds')+':'+env.get('GZ_SIM_RESOURCE_PATH', '')
    logs = Path(os.environ.get('RESULTS_DIR', str(Path.home()/'px4_baseline_bags')))/('startup_'+time.strftime('%Y%m%d_%H%M%S'))
    logs.mkdir(parents=True, exist_ok=False)
    children, files = [], []

    def launch(name, args, cwd=None):
        log = open(logs/(name+'.log'), 'w')
        files.append(log)
        process = subprocess.Popen(args, cwd=cwd, env=env, stdin=subprocess.DEVNULL,
                                   stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        children.append((name, process))

    try:
        launch('agent', ['MicroXRCEAgent', 'udp4', '-p', '8888'])
        time.sleep(1)
        launch('px4', [str(binary), '-d'], px4)
        prefix = '/model/x500_arm2dof_0/joint/'
        state = '/world/default/model/x500_arm2dof_0/joint_state'
        launch('bridge', ['ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
                         prefix+'arm_joint_1/cmd_vel@std_msgs/msg/Float64]gz.msgs.Double',
                         prefix+'arm_joint_2/cmd_vel@std_msgs/msg/Float64]gz.msgs.Double',
                         state+'@sensor_msgs/msg/JointState[gz.msgs.Model',
                         '/ground_truth/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
                         '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                         '--ros-args', '-r', state+':=/arm/joint_states'])
        print(f'Services started. Logs: {logs}', flush=True)
        print('Open terminal 2: bash run_experiment.sh\nFlight readiness is checked there.\nStop this terminal ONLY after landing and experiment exit.', flush=True)
        while True:
            for name, process in children:
                if process.poll() is not None:
                    raise RuntimeError(f'{name} exited ({process.returncode}); see {logs/name}.log')
            time.sleep(1)
    except KeyboardInterrupt:
        print('Stopping owned simulation processes.')
    finally:
        for _, process in reversed(children):
            try:
                os.killpg(process.pid, signal.SIGINT)
            except ProcessLookupError:
                pass
        time.sleep(2)
        for _, process in reversed(children):
            try:
                os.killpg(process.pid, signal.SIGTERM)
            except ProcessLookupError:
                pass
        for log in files:
            log.close()


if __name__ == '__main__':
    main()
