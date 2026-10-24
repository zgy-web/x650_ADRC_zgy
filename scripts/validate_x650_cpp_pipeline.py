"""Compare the independent Python oracle to the owning C++ controller.

Only offline math uses Python. C++ timing is measured within the native driver,
excluding process launch/JSON I/O; it does not certify ROS or physical timing.
"""
from __future__ import annotations
import argparse
import json
from pathlib import Path
import subprocess
import sys
import traceback
import numpy as np
from export_x650_cpp_inputs import ROOT, file_identity
sys.path.insert(0, str(ROOT/'control_packages/x650_wholebody_v1'))
from x650_wholebody.mpc import MpcController

PROTOCOL = json.loads((ROOT/'control_packages/x650_wholebody_v1/native/validation_protocol.json').read_text())


def compare(source, result):
    if result.get('pass') is not True:
        raise AssertionError('Native rejection: '+str(result.get('error')))
    oracle, d = source['oracle'], result['diagnostics']
    assert d['accepted'] and d['nonlinear_passed'] and d['actual_bounds']
    assert not d['previous_plan_fallback']
    assert d['subproblems'] == oracle['subproblems']
    assert [a['passed'] for a in d['attempts']] == [a['passed'] for a in oracle['attempts']]
    historical=oracle.get('historical_record') is True
    problem = ({k: np.asarray(v) for k,v in oracle['problems'][-1].items()} if not historical else None)
    errors = {}
    for key, native in ([] if historical else [('A','A'),('b','b'),('H','H'),('g','g'),('C','C'),('lo','lower'),('hi','upper')]):
        a = problem[key].ravel(); b = np.asarray(d[native])
        np.testing.assert_allclose(a,b,rtol=1e-9,atol=1e-9,err_msg=key)
        errors[key] = float(np.max(abs(a-b)))
    scale = np.tile(source['config']['velocity_limit'],source['config']['nodes'])
    plan_error = float(np.max(abs((np.asarray(oracle['plan'])-d['plan'])/scale)))
    state_error = float(np.max(abs(np.asarray(oracle['predicted_states'])-d['predicted_states'])))
    tip_error = float(np.max(abs(np.asarray(oracle['predicted_tip'])-d['predicted_tip'])))
    objective_error = abs(oracle['objective']-d['objective'])/(1+abs(oracle['objective']))
    assert plan_error <= PROTOCOL['normalized_plan_peak_max'], ('plan',plan_error)
    assert state_error <= PROTOCOL['predicted_state_peak_max'], ('state',state_error)
    assert tip_error <= PROTOCOL['predicted_state_peak_max'], ('tip',tip_error)
    assert objective_error <= PROTOCOL['objective_relative_scale_max'], ('objective',objective_error)
    np.testing.assert_allclose(result['command'],oracle['command'],rtol=0,atol=1e-7)
    execution = None if historical else oracle['candidate_executions'][-1]
    old_packets = [] if historical else execution['inputs']['packets']
    if not historical:assert len(old_packets)==len(d['candidate_packets'])
    for old,new in zip(old_packets,d['candidate_packets']):
        # Bus-grid arithmetic may differ by a floating ULP; wire fields must not.
        np.testing.assert_allclose(old['send_time_s'],new['send_time_s'],rtol=0,atol=1e-12)
        for field in ('q_ref','dq_ref','tau_ff','kp','kd'):
            np.testing.assert_array_equal(old[field],new[field],err_msg='MIT '+field)
    if historical:
        n=len(scale);m=len(d['lower'])
        problem={'H':np.asarray(d['H']).reshape(n,n),'g':np.asarray(d['g']),
            'C':np.asarray(d['C']).reshape(m,n),'lo':np.asarray(d['lower']),'hi':np.asarray(d['upper'])}
        np.testing.assert_allclose(result['position_reference'],oracle['position_reference'],rtol=0,atol=1e-9)
        np.testing.assert_allclose(result['velocity_feedforward'],oracle['velocity_feedforward'],rtol=0,atol=1e-7)
    q = MpcController._quality(problem,np.asarray(d['plan'])/scale)
    assert q['primal_violation'] <= PROTOCOL['original_full_constraint_primal_max'],q
    assert q['kkt_stationarity_inf'] <= PROTOCOL['original_full_constraint_stationarity_max'],q
    assert result['core_step_wall_ms'] <= 50.,result['core_step_wall_ms']
    return dict(matrix_peak_errors=errors,normalized_plan_peak=plan_error,
        predicted_state_peak=state_error,predicted_tip_peak=tip_error,objective_relative=objective_error,
        independently_recomputed_quality=q,core_step_wall_ms=result['core_step_wall_ms'],
        native_compute_ms=d['compute_ms'],mit_wire_fields_equal=None if historical else True,
        independent_matrix_oracle=not historical,
        timing_scope='owning Core only; does not include ROS feedback/admission/dispatch')


def main(argv=None):
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--cases',type=Path,required=True)
    p.add_argument('--binary',type=Path,required=True)
    p.add_argument('--binary-sha256',required=True)
    p.add_argument('--core-library',type=Path,required=True)
    p.add_argument('--core-sha256',required=True)
    p.add_argument('--optimization-library',type=Path,required=True)
    p.add_argument('--optimization-sha256',required=True)
    p.add_argument('--predictor',type=Path,required=True)
    p.add_argument('--predictor-sha256',required=True)
    p.add_argument('--output',type=Path,required=True)
    a=p.parse_args(argv);a.output.mkdir(parents=True,exist_ok=False)
    manifest=json.loads(a.cases.read_text())
    assert manifest['complete'] and manifest['source_files_unchanged']
    assert file_identity(a.binary)['sha256']==a.binary_sha256
    assert file_identity(a.core_library)['sha256']==a.core_sha256
    assert file_identity(a.optimization_library)['sha256']==a.optimization_sha256
    assert file_identity(a.predictor)['sha256']==a.predictor_sha256
    report=dict(kind='cpp_owning_core_equivalence_v1',cases=file_identity(a.cases),
        binary=file_identity(a.binary),predictor=file_identity(a.predictor),rows=[],complete=False,
        core_library=file_identity(a.core_library),optimization_library=file_identity(a.optimization_library),
        validator=file_identity(__file__),
        runtime_python_calls=0,ros_timing_qualified=False,physical_hold_qualified=False)
    for index,item in enumerate(manifest['rows']):
        source_path=Path(item['path']);assert file_identity(source_path)==item
        result_path=a.output/f'case_{index:03d}.json'
        process=subprocess.run([str(a.binary),str(a.predictor),a.predictor_sha256,
            str(source_path),str(result_path)],capture_output=True,text=True,timeout=30)
        row=dict(index=index,pass_=False,exit_code=process.returncode)
        try:
            if process.returncode:raise RuntimeError(process.stderr or result_path.read_text())
            result=json.loads(result_path.read_text());assert result['input_sha256']==item['sha256']
            for key,path in (('loaded_core',a.core_library),('loaded_optimization',a.optimization_library)):
                assert Path(result[key]['path']).resolve()==path.resolve(),(key,'loaded path differs')
                assert result[key]['sha256']==file_identity(path)['sha256'],(key,'loaded hash differs')
            row.update(compare(json.loads(source_path.read_text()),result));row['pass_']=True
        except Exception as e:
            row.update(error=repr(e),traceback=traceback.format_exc())
        report['rows'].append(row)
    report.update(complete=True,passed=sum(x['pass_'] for x in report['rows']),total=len(report['rows']))
    report['pass']=report['passed']==report['total']
    report['libraries_unchanged']=all(report[key]==file_identity(path) for key,path in
        (('binary',a.binary),('core_library',a.core_library),('optimization_library',a.optimization_library),('predictor',a.predictor)))
    report['pass']=report['pass'] and report['libraries_unchanged'] and report['validator']==file_identity(__file__)
    (a.output/'report.json').write_text(json.dumps(report,indent=2,allow_nan=False))
    print(json.dumps({k:report[k] for k in ('pass','passed','total')}))
    for row in report['rows']:
        if not row['pass_']:print(json.dumps(dict(index=row['index'],error=row['error'])))
    return 0 if report['pass'] else 1


if __name__=='__main__':raise SystemExit(main())
