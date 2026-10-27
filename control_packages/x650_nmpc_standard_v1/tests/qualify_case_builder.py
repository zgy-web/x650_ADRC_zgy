"""Cross-language original-feedback mapping; no optimizer or simulation."""
import argparse
import copy
import hashlib
import json
from pathlib import Path
import subprocess

import numpy as np

from x650_nmpc_standard.model import build_case


def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--selection',action='append',required=True)
    parser.add_argument('--cpp-builder',required=True);parser.add_argument('--output',required=True)
    args=parser.parse_args();out=Path(args.output);out.mkdir(parents=True,exist_ok=False)
    records=[];maximum={key:0. for key in ('x0','parameters','initial_plan','horizon')}
    documents=[]
    for index,path in enumerate(args.selection):
        doc=json.loads(Path(path).read_text());documents.append(doc);destination=out/f'cpp_{index:02}.json'
        run=subprocess.run([args.cpp_builder,path,str(destination)],capture_output=True,text=True)
        if run.returncode:raise RuntimeError(run.stderr)
        result=json.loads(destination.read_text());assert result['all_built'],result
        assert len(result['records'])==len(doc['cases'])
        for wrapped,row in zip(doc['cases'],result['records']):
            case=build_case(doc['config'],wrapped);assert wrapped['case']['id']==row['id'];errors={}
            for key in maximum:
                expected=np.asarray(getattr(case,key)).ravel();actual=np.asarray(row[key])
                assert expected.shape==actual.shape
                errors[key]=float(np.max(abs(expected-actual)));maximum[key]=max(maximum[key],errors[key])
                assert errors[key]<=1e-10,(row['id'],key,errors[key])
            records.append(dict(id=row['id'],source=path,errors=errors))
    base=documents[0]['cases'][0]['case'];bad=[]
    def add(label,change):
        case=copy.deepcopy(base);case['id']=label;change(case);bad.append(case)
    add('unprimed_history',lambda c:c['memory'].update(history_primed=False))
    add('empty_packets',lambda c:c['input']['context'].update(packets=[]))
    add('reverse_actual_packet_order',lambda c:c['input']['context']['packets'].reverse())
    add('different_actual_gains',lambda c:c['input']['context']['packets'][-1]['kp'].__setitem__(0,7.))
    add('dt_outside_contract',lambda c:c['input'].update(dt=.101))
    add('stale_joint_state',lambda c:c['input']['state'].update(joint_stamp=c['input']['state']['control_time']-.03))
    add('future_joint_state',lambda c:c['input']['state'].update(joint_stamp=c['input']['state']['control_time']+.001))
    add('inconsistent_epoch',lambda c:c['memory'].update(origin=c['memory']['origin']+.01))
    add('future_actual_packet',lambda c:c['input']['context']['packets'][-1].update(send_time=c['input']['t']+.001))
    add('invalid_rotation',lambda c:c['input']['state']['rotation'].__setitem__(0,2.))
    add('missing_command_history',lambda c:c['input']['context'].update(commands=[]))
    add('missing_history_coverage',lambda c:c['input']['context'].update(packets=[c['input']['context']['packets'][-1]]))
    (out/'rejected_inputs.json').write_text(json.dumps(dict(config=documents[0]['config'],cases=bad),allow_nan=False))
    process=subprocess.run([args.cpp_builder,str(out/'rejected_inputs.json'),str(out/'cpp_rejections.json')],capture_output=True,text=True)
    if process.returncode:raise RuntimeError(process.stderr)
    rejected=json.loads((out/'cpp_rejections.json').read_text());rejections=[]
    for case,result in zip(bad,rejected['records']):
        reason=None
        try:build_case(documents[0]['config'],case)
        except (ValueError,KeyError) as error:reason=str(error)
        assert reason and not result['built'],(case['id'],reason,result)
        rejections.append(dict(id=case['id'],python_reason=reason,cpp_reason=result['reason']))
    source_root=Path(__file__).resolve().parents[1]
    report=dict(kind='x650_standard_cpp_mapping_qualification_v1',passed=True,count=len(records),
                tolerance_absolute=1e-10,maximum_absolute_errors=maximum,records=records,rejections=rejections,
                input_sources=[dict(path=p,sha256=sha(p)) for p in args.selection],cpp_builder_sha256=sha(args.cpp_builder),
                sources={str(p.relative_to(source_root)):sha(p) for p in [source_root/'native/case_builder.hpp',source_root/'native/case_builder.cpp',
                    source_root/'native/case_builder_replay.cpp',source_root/'python/x650_nmpc_standard/model.py',source_root/'python/x650_nmpc_standard/config.py']},
                scope='Numerical mapping only; no solve, memory commit, command publication or physics')
    (out/'report.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
    print(json.dumps(dict(passed=True,count=len(records),rejections=len(rejections),maximum_absolute_errors=maximum,output=str(out/'report.json'))))


if __name__=='__main__':main()
