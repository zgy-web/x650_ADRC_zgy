
import csv,hashlib,json
from pathlib import Path
repo=Path('/root/autodl-tmp/safe-aerial-manipulator-testbed')
root=repo/'runs/x650_isolated_live_v1_20261024_124523_18157'
base=repo/'artifacts/isolated_live_20261024'
start=json.loads((root/'isolated_live/live_start.json').read_text())['issued_monotonic_ns']
fault=json.loads((root/'isolated_live/fault_supervision.json').read_text())['first_fault']['producer_monotonic_ns']
result={}
for key,p in [('resources',base/'hold_v24_resources.jsonl'),('flight_tail',root/'flight_samples.csv')]:
 if not p.is_file():result[key]={'missing':str(p)};continue
 raw=p.read_bytes();source={'source_path':str(p),'bytes':len(raw),'sha256':hashlib.sha256(raw).hexdigest()}
 if key=='resources':
  rows=[json.loads(line) for line in raw.splitlines() if line.strip()]
  pre=[i for i,r in enumerate(rows) if r['monotonic_ns']<=start]
  post=[i for i,r in enumerate(rows) if r['monotonic_ns']>=fault]
  if not pre or not post:result[key]={'source':source,'coverage':False,'first':rows[0],'last':rows[-1]};continue
  lo=max(pre);hi=min(post)
  result[key]={'source':source,'derived_analysis':True,'selection':'nearest outside bracket of LiveStart and first fault','original_row_indices_inclusive':[lo,hi],'complete_original_rows':len(rows),'rows':rows[lo:hi+1]}
 else:
  rows=list(csv.DictReader(raw.decode().splitlines()))
  result[key]={'source':source,'derived_analysis':True,'selection':'last 5 original parsed CSV rows','complete_original_rows':len(rows),'rows':rows[-5:]}
print(json.dumps(result,indent=2))
