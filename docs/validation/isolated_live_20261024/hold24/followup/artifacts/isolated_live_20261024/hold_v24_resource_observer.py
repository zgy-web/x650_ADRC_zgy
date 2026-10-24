from pathlib import Path
import os,time,json
B=Path(__file__).parent
os.sched_setaffinity(0,{0})
start=time.monotonic();end=start+300
with (B/'hold_v24_resources.jsonl').open('x') as f:
 while time.monotonic()<end:
  row={'monotonic_ns':time.monotonic_ns()}
  for name in ('cpu.stat','cpu.max','cpu.pressure'):
   p=Path('/sys/fs/cgroup')/name
   if p.exists():row[name]=p.read_text()
  row['frequency_khz']={}
  for cpu in (87,89,91,97):
   p=Path(f'/sys/devices/system/cpu/cpu{cpu}/cpufreq/scaling_cur_freq')
   if p.exists():row['frequency_khz'][str(cpu)]=p.read_text().strip()
  f.write(json.dumps(row)+'\n');f.flush()
  if (B/'hold_v24.exit').exists():break
  time.sleep(.05)
