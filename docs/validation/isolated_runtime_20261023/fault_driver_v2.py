from pathlib import Path
import subprocess,json,os
root=Path('/root/autodl-tmp/safe-aerial-manipulator-testbed');out=root/'artifacts/isolated_runtime_20261023'
status={'complete':False,'cases':[]}
for i,case in enumerate(['worker_pause','worker_short_pause','source_pause','recorder_pause']):
 command='source /opt/ros/humble/setup.bash && source /root/autodl-tmp/ros2_px4_ws/install/setup.bash && source /root/autodl-tmp/flyarm_ros2_isolated_v1_20261023/install/setup.bash && env OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 /usr/bin/python3 scripts/run_x650_isolated_runtime.py --task-config artifacts/native_numerics_20261023/quiet_task_cpp_v5.json --output artifacts/isolated_runtime_20261023/'+case+'_v2 --case '+case+' --domain-id '+str(202+i)
 with (out/(case+'_v2.log')).open('x') as log:
  result=subprocess.run(['/bin/bash','-c',command],cwd=root,stdin=subprocess.DEVNULL,stdout=log,stderr=subprocess.STDOUT)
 status['cases'].append({'case':case,'exit_code':result.returncode})
 (out/'fault_sequence_status_v2.json').write_text(json.dumps(status,indent=2))
status['complete']=True
(out/'fault_sequence_status_v2.json').write_text(json.dumps(status,indent=2))
