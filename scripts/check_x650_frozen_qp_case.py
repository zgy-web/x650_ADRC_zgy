#!/usr/bin/env python3
"""Replay an archived matrix-level QP without flight, retuning or fallback.

This is diagnostic regression evidence, not a full-controller or flight pass.
Raw constraints and normalized box bounds remain authoritative; row-normalized
distances are reported only to explain the numerical scale of a violation.
"""
import argparse
import hashlib
import json
from pathlib import Path
import sys
import time
import traceback
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'control_packages/x650_wholebody_v1'))
from x650_wholebody.native_optimization import NativeOptimization


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def pinned_task(path,expected,library_sha):
    path=Path(path).resolve(strict=True)
    if sha(path)!=expected:raise ValueError('Caller-pinned task SHA256 differs')
    value=json.loads(path.read_text(encoding='utf-8'))
    if value.get('numerical_backend',{}).get('library_sha256')!=library_sha:
        raise ValueError('Library SHA256 differs from the pinned frozen task')
    config=value['mpc_config']
    for key in ('feasibility_tolerance','kkt_tolerance','max_iterations'):
        if key not in config:raise ValueError('Explicit frozen task setting required: '+key)
    limits={key:float(config[key]) for key in ('feasibility_tolerance','kkt_tolerance')}
    if any(not np.isfinite(x) or x<=0 for x in limits.values()):
        raise ValueError('Invalid pinned quality limits')
    iterations=config['max_iterations']
    if type(iterations) is not int or not 1<=iterations<=10000:
        raise ValueError('Invalid pinned max_iterations')
    return value,limits,iterations


def load_case(path,iterations):
    required=('H','g','C','lo','hi','warm','scale','solution','max_iterations')
    with np.load(path,allow_pickle=False) as stored:
        if any(key not in stored.files for key in required):
            raise ValueError('NPZ is missing required frozen QP fields')
        case={key:np.array(stored[key],copy=True) for key in required}
    count=case.pop('max_iterations')
    if count.size!=1 or count.dtype.kind not in 'iu' or int(count.item())!=iterations:
        raise ValueError('Captured iteration limit differs from pinned task')
    n=case['g'].size;m=case['lo'].size
    if not n or not m:raise ValueError('Nonempty frozen problem required')
    for key,shape in {'H':(n,n),'g':(n,),'C':(m,n),'lo':(m,),'hi':(m,),
                      'warm':(n,),'scale':(n,),'solution':(n,)}.items():
        value=case[key]
        if value.shape!=shape or value.dtype.kind not in 'fiu' or not np.isfinite(value).all():
            raise ValueError('Invalid captured finite numeric array: '+key)
        case[key]=np.ascontiguousarray(value,dtype=np.float64)
    if np.any(case['lo']>case['hi']) or np.any(case['scale']<=0):
        raise ValueError('Captured bounds or variable scale invalid')
    asymmetry=float(np.max(abs(case['H']-case['H'].T)))
    if asymmetry>1e-12*max(1.,float(np.max(abs(case['H'])))):
        raise ValueError('Captured Hessian is not symmetric')
    np.linalg.cholesky(case['H'])
    return case


def independent_residuals(problem,z):
    """Direct original-row evaluation; no solver-reported primal quality used."""
    z=np.asarray(z,dtype=float)
    if z.shape!=problem['g'].shape or not np.isfinite(z).all():
        raise ValueError('Invalid finite native solution')
    C,lo,hi=problem['C'],problem['lo'],problem['hi']
    values=C@z
    def row(side,residual):
        index=int(np.argmax(residual));coeff=C[index]
        norm=float(np.linalg.norm(coeff))
        violation=max(0.,float(residual[index]))
        return {'side':side,'row_index_zero_based':index,'value':float(values[index]),
                'lower':float(lo[index]),'upper':float(hi[index]),
                'signed_residual':float(residual[index]),'positive_violation':violation,
                'coefficient_l2_norm':norm,'coefficient_linf_norm':float(np.max(abs(coeff))),
                'nonzero_coefficients':int(np.count_nonzero(coeff)),
                'distance_in_normalized_z_coordinates':violation/norm if norm else None,
                'distance_is_acceptance_metric':False,
                'coefficients':coeff.tolist()}
    lower,upper=row('lower',lo-values),row('upper',values-hi)
    box_index=int(np.argmax(abs(z)-1.));box_value=float(z[box_index])
    box={'side':'upper' if box_value>=0 else 'lower','variable_index_zero_based':box_index,
         'value':box_value,'lower':-1.,'upper':1.,'signed_residual':abs(box_value)-1.,
         'positive_violation':max(0.,abs(box_value)-1.),'coefficient_l2_norm':1.}
    worst=max((lower,upper,box),key=lambda entry:entry['positive_violation'])
    gradient=problem['H']@z+problem['g']
    return {'primal_violation':worst['positive_violation'],'worst_source':
                ('box' if worst is box else 'C_'+worst['side']),
            'lower':lower,'upper':upper,'box':box,
            'solution_l2_norm':float(np.linalg.norm(z)),'solution_linf_norm':float(np.max(abs(z))),
            'unconstrained_gradient_linf':float(np.max(abs(gradient))),
            'objective_without_constant':float(.5*z@problem['H']@z+problem['g']@z),
            'scope':'original raw C rows and normalized decision box; gradient is not constrained KKT residual'}


def expectation_satisfied(expectation, *, inputs_unchanged, identical_solution,
                          identical_quality, reproduced, results, feasibility_tolerance):
    if expectation not in ('reproduce','accept'):
        raise ValueError('Expected outcome must be reproduce or accept')
    common=(inputs_unchanged and identical_solution and identical_quality and bool(results)
            and all(item['independent_primal_gate_matches_native'] for item in results))
    if expectation=='reproduce':
        return bool(common and reproduced)
    return bool(common and all(item['accepted_by_frozen_qp_quality']
        and item['independent_residual']['primal_violation']<=feasibility_tolerance for item in results))


def run(case_path,task_path,task_sha,library,library_sha,repeats,expectation='reproduce'):
    if not Path(library).is_absolute():raise ValueError('Explicit absolute library path required')
    if not 2<=repeats<=100:raise ValueError('Use 2 to 100 identical independent solve calls')
    cfg,limits,iterations=pinned_task(task_path,task_sha,library_sha)
    case=load_case(case_path,iterations)
    before={key:value.copy() for key,value in case.items()}
    native=NativeOptimization(library,library_sha)
    results=[];solutions=[]
    for index in range(repeats):
        begin=time.monotonic_ns()
        z,quality,exact_center=native.solve(case,iterations)
        elapsed=time.monotonic_ns()-begin
        residual=independent_residuals(case,z)
        if any(not np.isfinite(value) for value in quality.values()):
            raise ValueError('Nonfinite native quality')
        accepted=(quality['primal_violation']<=limits['feasibility_tolerance']
                  and quality['kkt_stationarity_inf']<=limits['kkt_tolerance'])
        primal_agrees=((quality['primal_violation']<=limits['feasibility_tolerance'])
                       ==(residual['primal_violation']<=limits['feasibility_tolerance']))
        results.append({'repeat':index+1,'solve_ns':elapsed,'exact_interior_center':exact_center,
            'quality':quality,'accepted_by_frozen_qp_quality':bool(accepted),
            'independent_primal_gate_matches_native':bool(primal_agrees),
            'independent_residual':residual,
            'native_vs_independent_primal_abs_difference':abs(quality['primal_violation']-residual['primal_violation']),
            'solution_sha256':hashlib.sha256(np.asarray(z,dtype='<f8').tobytes()).hexdigest(),
            'solution':z.tolist(),'max_abs_difference_from_recorded_solution':float(np.max(abs(z-case['solution']))),
            'identical_to_recorded_solution':bool(np.array_equal(z,case['solution']))})
        solutions.append(z.copy())
    unchanged=all(np.array_equal(case[key],value) for key,value in before.items())
    identical=all(np.array_equal(solutions[0],z) for z in solutions[1:])
    same_quality=all(item['quality']==results[0]['quality'] for item in results[1:])
    reproduced=all(item['identical_to_recorded_solution'] for item in results)
    satisfied=expectation_satisfied(expectation,inputs_unchanged=unchanged,identical_solution=identical,
        identical_quality=same_quality,reproduced=reproduced,results=results,
        feasibility_tolerance=limits['feasibility_tolerance'])
    return {'kind':'x650_frozen_matrix_qp_regression_v1','complete':True,
        'expectation':expectation,'expectation_satisfied':satisfied,'regression_consistent':satisfied,
        'offline_only':True,'controller_or_flight_accepted':False,
        'case':{'path':str(Path(case_path).resolve()),'sha256':sha(case_path)},
        'task':{'path':str(Path(task_path).resolve()),'sha256':task_sha},'library':native.receipt,
        'checker':{'path':str(Path(__file__).resolve()),'sha256':sha(__file__)},
        'dimensions':{'variables':len(case['g']),'linear_rows':len(case['lo']),'implicit_box_rows':len(case['g'])},
        'limits':limits,'max_iterations':iterations,'repeat_count':repeats,
        'inputs_unchanged':unchanged,'repeats_identical_solution':identical,'repeats_identical_quality':same_quality,
        'archived_solution_reproduced':reproduced,
        'captured_solution_independent_residual':independent_residuals(case,case['solution']),
        'hessian_condition_number_2':float(np.linalg.cond(case['H'])),
        'hessian_symmetry_max_abs':float(np.max(abs(case['H']-case['H'].T))),
        'scale':case['scale'].tolist(),'warm_norm_2':float(np.linalg.norm(case['warm'])),
        'warm_semantics':'archived for provenance; frozen NativeOptimization.solve does not consume warm or scale',
        'numpy_version':np.__version__,'results':results,
        'interpretation':('reproduce requires the recorded solution unchanged, and may reproduce rejection; '
            'accept permits a different solution but requires repeatability and original native/independent quality gates; '
            'neither certifies nonlinear candidate acceptance, timing integration, or a successful flight.')}


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    for name in ('case','task','library','output'):parser.add_argument('--'+name,type=Path,required=True)
    parser.add_argument('--task-sha256',required=True);parser.add_argument('--library-sha256',required=True)
    parser.add_argument('--repeats',type=int,default=5)
    parser.add_argument('--expect',choices=('reproduce','accept'),default='reproduce')
    args=parser.parse_args()
    if args.output.exists():raise FileExistsError('Do not overwrite prior evidence: '+str(args.output))
    try:
        report=run(args.case,args.task,args.task_sha256,args.library,args.library_sha256,args.repeats,args.expect)
    except Exception as error:
        report={'kind':'x650_frozen_matrix_qp_regression_v1','complete':False,
                'expectation':args.expect,'expectation_satisfied':False,
                'regression_consistent':False,'offline_only':True,'controller_or_flight_accepted':False,
                'error':repr(error),'traceback':traceback.format_exc()}
    encoded=json.dumps(report,indent=2,allow_nan=False)+'\n'
    with args.output.open('x',encoding='utf-8') as stream:stream.write(encoded)
    print(json.dumps({key:report.get(key) for key in ('complete','expectation','expectation_satisfied','regression_consistent','error',
        'limits','repeats_identical_solution','archived_solution_reproduced')},indent=2))
    if report.get('complete'):
        print(json.dumps({'quality':report['results'][0]['quality'],
            'independent_primal':report['results'][0]['independent_residual']['primal_violation'],
            'worst_source':report['results'][0]['independent_residual']['worst_source'],
            'accepted_by_frozen_qp_quality':report['results'][0]['accepted_by_frozen_qp_quality']},indent=2))
    return 0 if report['regression_consistent'] else 1


if __name__=='__main__':raise SystemExit(main())
