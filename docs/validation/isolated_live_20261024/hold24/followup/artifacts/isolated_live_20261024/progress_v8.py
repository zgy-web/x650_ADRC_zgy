from pathlib import Path
import sys,json,hashlib,subprocess,time,traceback
R=Path.cwd();B=R/'artifacts/isolated_live_20261024';A=R/'artifacts/isolated_live_20261023'
state={'complete':False,'stage':'waiting_for_hold24','fast_started':False}
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def save(**kw):
 state.update(kw);(B/'progress_v8.json').write_text(json.dumps(state,indent=2))
def call(label,args):
 save(stage=label)
 with (B/(label+'.log')).open('x') as f:r=subprocess.run([sys.executable,*args],stdout=f,stderr=subprocess.STDOUT)
 (B/(label+'.exit')).write_text(str(r.returncode)+'\n')
 if r.returncode:raise RuntimeError(label+' failed; original evidence retained')
try:
 save()
 while not (B/'hold_v24.exit').exists():time.sleep(3)
 assert (B/'hold_v24.exit').read_text().strip()=='0','Hold24 did not complete'
 rows=[s[len('ISOLATED_LAUNCH '):] for s in (B/'hold_v24.log').read_text().splitlines() if s.startswith('ISOLATED_LAUNCH ')];assert len(rows)==1
 launch=Path(rows[0]);j=json.loads((launch/'launch.json').read_text());assert j['complete'] and j['runtime_closed'] and len(j['run_directories'])==1
 run=Path(j['run_directories'][0]);save(hold_launch=str(launch),hold_run=str(run))
 audit=B/'hold_v24_audit_v1';call('hold_v24_audit_v1',['scripts/audit_x650_isolated_live.py','--launch-dir',str(launch),'--run-dir',str(run),'--output',str(audit)])
 report=audit/'report.json';assert json.loads(report.read_text())['passed'];save(hold_passed=True,hold_report=str(report),hold_report_sha256=sha(report))
 call('hold_v24_plot',['scripts/plot_x650_isolated_validation.py','--report',str(report),'--output',str(B/'hold_v24_plot')])
 task=B/'quiet_task_cpp_v11.json';transport=B/'transport_v11/report.json';protocol=R/'configs/experiments/x650_isolated_live_v1.json'
 call('normal_sequence_v8',['scripts/run_x650_isolated_normal_sequence.py','--repo',str(R),'--hold-task',str(task),'--hold-task-sha256',sha(task),'--hold-report',str(report),'--hold-report-sha256',sha(report),'--transport-proof',str(transport),'--transport-proof-sha256',sha(transport),'--protocol-sha256',sha(protocol),'--output',str(B/'normal_sequence_v8'),'--execute'])
 result=json.loads((B/'normal_sequence_v8/status.json').read_text());assert result['complete'] and result['normal_passed'];save(complete=True,stage='normal_complete',normal_report=result['normal_report'],normal_endpoint=result['endpoint'],fast_started=False)
except BaseException as e:
 save(error=repr(e),traceback=traceback.format_exc())
