from pathlib import Path
import argparse,bisect,hashlib,json
p=argparse.ArgumentParser();p.add_argument('--run',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
folder=a.run/'isolated_live';path=folder/'source_activity.json'
j=json.loads(path.read_text());sources=json.loads((folder/'source_timing.json').read_text())
assert j['kind']=='isolated_source_activity_v1' and j['complete']
assert j['recorded']>0 and j['recorded']==j['attempted']==len(j['rows'])
assert j['gc']['policy_before'] is not None and j['gc']['policy_unchanged']
assert not any(j[k] for k in ('overflow','invalid_marks','overlapping_pumps','unfinished_records','duplicate_marks'))
assert j['gc']['overflow']==0 and j['gc']['unknown_phases']==0
rows=[dict(zip(j['columns'],r)) for r in j['rows']]
marks=[n for n in j['columns'] if n.endswith('_begin_ns')]
previous_end=0
for index,row in enumerate(rows,1):
    assert row['pump_id']==index and row['failed']==0 and row['completed']==1
    assert row['begin_ns']>=previous_end and row['end_ns']>=row['begin_ns']
    phases=[row[n] for n in marks if row[n]]
    assert phases==sorted(phases) and row['begin_ns']<=phases[0]<=phases[-1]<=row['end_ns']
    assert row['end_thread_cpu_ns']>=row['begin_thread_cpu_ns']
    previous_end=row['end_ns']
begins=[r['begin_ns'] for r in rows]
for source in sources['rows']:
    sequence,issued,published,_,_=source
    index=bisect.bisect_right(begins,issued)-1
    assert index>=0
    row=rows[index]
    assert row['begin_ns']<=issued<=row['read_begin_ns']<=row['publish_begin_ns']<=published<=row['end_ns'],sequence
gc_open={}
for raw in j['gc']['rows']:
    g=dict(zip(j['gc']['columns'],raw));key=(g['thread_id'],g['generation'])
    if g['phase']==1:
        assert key not in gc_open;gc_open[key]=g
    else:
        assert g['phase']==2 and key in gc_open
        first=gc_open.pop(key);assert g['monotonic_ns']>=first['monotonic_ns'] and g['thread_cpu_ns']>=first['thread_cpu_ns']
assert not gc_open
result={'kind':'independent_source_activity_audit_v1','passed':True,'run':str(a.run),'source_file':str(path),'source_sha256':hashlib.sha256(path.read_bytes()).hexdigest(),'pump_rows':len(rows),'source_rows':len(sources['rows']),'gc_rows':j['gc']['recorded'],'all_success_and_phase_order_verified':True,'original_source_issue_within_measured_read_verified':True,'gc_policy_and_complete_pairs_verified':True,'does_not_replace_full_flight_audit':True}
with a.output.open('x') as f:json.dump(result,f,indent=2)
print(json.dumps(result))
