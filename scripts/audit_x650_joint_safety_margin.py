#!/usr/bin/env python3
"""Read-only paired fixed-input audit of TS20 planning-node safety padding.

Passing this audit verifies the numerical change and explains a recorded safety
rejection. It never means that the rejected controller call or a flight passed.
No controller, predictor, optimizer, actuator or simulator is executed here.
"""
from pathlib import Path
import argparse
import json
import sys
import traceback

import numpy as np

sys.path.insert(0,str(Path(__file__).resolve().parent))
import audit_x650_recovery_replay as replay

Q_PADDING=.0003
DQ_PADDING=.03
TOLERANCE=2e-7


def require(value,message):
    if not value:raise ValueError(message)


def array(value,shape):
    out=np.asarray(value,dtype=float)
    require((-1 in shape or out.size==int(np.prod(shape))) and np.isfinite(out).all(),'Malformed finite matrix/vector')
    return out.reshape(shape)


def compare_problem(old,new,nodes):
    """Assert that exactly the 2q+2dq node bound rows were tightened."""
    n,m=5*nodes,15*nodes+5
    require(old['qp_columns']==new['qp_columns']==n and old['qp_rows']==new['qp_rows']==m,
            'Unexpected original QP dimensions or row layout')
    unchanged={}
    for key,shape in [('A',(21*nodes,n)),('b',(21*nodes,)),('H',(n,n)),('g',(n,)),('C',(m,n))]:
        a,b=array(old[key],shape),array(new[key],shape)
        require(np.array_equal(a,b),'Unreviewed matrix/objective change: '+key)
        unchanged[key]=dict(elements=int(a.size),maximum_absolute_difference=0.)
    expected=np.zeros(m);labels={}
    for i in range(nodes):
        first=5*nodes+6*i
        for j in range(2):
            expected[first+j]=Q_PADDING;labels[first+j]=dict(node=i,joint=j+1,kind='q',padding=Q_PADDING)
            expected[first+2+j]=DQ_PADDING;labels[first+2+j]=dict(node=i,joint=j+1,kind='dq',padding=DQ_PADDING)
    a_lo,a_hi=(array(old[key],(m,)) for key in ('lower','upper'))
    b_lo,b_hi=(array(new[key],(m,)) for key in ('lower','upper'))
    require(np.all(a_lo<=a_hi) and np.all(b_lo<=b_hi),'QP bound ordering changed')
    require(np.max(abs((b_lo-a_lo)-expected))<=1e-12
        and np.max(abs((b_hi-a_hi)+expected))<=1e-12,'Only the reviewed node padding may change')
    unmodified=expected==0
    require(np.array_equal(a_lo[unmodified],b_lo[unmodified])
        and np.array_equal(a_hi[unmodified],b_hi[unmodified]),'Non-node guard/reference/trust bound changed')
    return dict(verified=True,unchanged=unchanged,changed_bound_rows=[dict(row=i,**labels[i],
        lower_delta=float(b_lo[i]-a_lo[i]),upper_delta=float(b_hi[i]-a_hi[i])) for i in labels],
        changed_lower_count=int(np.count_nonzero(a_lo!=b_lo)),changed_upper_count=int(np.count_nonzero(a_hi!=b_hi)),
        unchanged_rows=dict(command_slew=list(range(5*nodes)),
            stopping=[5*nodes+6*i+4+j for i in range(nodes) for j in range(2)],
            integrated_reference=list(range(11*nodes,13*nodes)),trust=list(range(13*nodes,15*nodes)),
            first_command_pending=list(range(15*nodes,15*nodes+5))),
        scope='Only q/dq surrogate node bounds change; objective, sensitivities, command normalization and all other original rows remain exact')


def check_metadata(old,new,fixture,input_sha):
    for value in (old,new):
        require(value.get('kind')=='x650_cpp_recovery_raw_replay_v1' and value.get('schema_version')==1
            and value.get('complete') is True and value.get('core_abi')==2
            and value.get('input_sha256')==input_sha,'Raw replay metadata or exact input identity differs')
        require(value.get('returned') is False and value.get('core_error') is True
            and value.get('unexpected_exception') is False and value.get('output') is None
            and value.get('error')=='TS20 nonlinear candidate rejected before command publication',
            'This audit requires the original and padded rejected candidate calls')
        require(value.get('memory_unchanged') is True and value.get('memory_commit_valid') is False
            and value['memory_before']==value['memory_after']==fixture['memory'],
            'Rejected fixed-input call changed original controller memory')
        expected=dict(offline_fixed_snapshot=True,actuator_io=False,closed_loop_or_physical_validation=False,
            injected_delay=False,original_core_deadline_ms=50.,initial_trust_radius=.02)
        require(all(value.get('scope',{}).get(k)==v for k,v in expected.items()),'Raw scope or original gates changed')
        d=value['diagnostics']
        require(d['accepted'] is False and d['actual_bounds'] is False and d['nonlinear_passed'] is False
            and d['previous_plan_fallback'] is False and d['recovery_triggered'] is False
            and d['recovery_attempts']==[] and d['subproblems']==1,'Unexpected call/branch status')
    require(old['loaded_code']['predictor']==new['loaded_code']['predictor'],'TS20 prediction library changed')
    require(old['memory_before']==new['memory_before'],'Paired calls used different original memory')
    require(fixture['config']['feasibility_tolerance']==TOLERANCE
        and fixture['config']['kkt_tolerance']==.002,'Original external quality tolerances changed')
    return dict(verified=True,exact_same_input_sha256=input_sha,rejected_memory_unchanged=True,
        predictor_unchanged=True,original_deadline_ms=50.,initial_trust_radius=.02,
        rejected_plan_never_returned=True)


def inspect_candidate(fixture,value,padding):
    d=value['diagnostics'];c=fixture['config'];N=c['nodes'];n=5*N;m=d['qp_rows']
    C=array(d['C'],(m,n));lo=array(d['lower'],(m,));hi=array(d['upper'],(m,));z=array(d['solution'],(n,))
    plan=array(d['plan'],(N,5));limits=array(c['velocity_limit'],(5,))
    require(np.max(abs(plan-z.reshape(N,5)*limits))<=1e-10,'Plan differs from original normalized solution')
    primal=max(0.,float((lo-C@z).max()),float((C@z-hi).max()),float((abs(z)-1).max()))
    require(primal<=TOLERANCE and abs(primal-d['primal_violation'])<=1e-10,'Original full-row QP quality failed')
    require(0<=d['kkt_stationarity']<=c['kkt_tolerance'] and 0<=d['compute_ms']<=50.,'Original KKT or deadline failed')
    refs=replay.node_references(fixture,d['plan'])
    replay.check_plan(fixture,d['plan'],refs)
    replay.check_packets(fixture,d['candidate_packets'],d['plan'])
    times=np.asarray(replay.node_times(fixture));queries=array(d['query_times'],(len(d['query_times']),))
    states=array(d['candidate_states'],(len(queries),4));extrema=array(d['candidate_extrema'],(-1,5))
    require(np.all(np.diff(queries)>0),'Native candidate query times regressed')
    # Reconstruct each affine node prediction from its own unchanged q/dq row,
    # original physical bound and known padding. No between-node affine value
    # is invented: that sensitivity is not present in this rejected capture.
    records=[];native_nodes=[];affine_nodes=[]
    for i,t in enumerate(times):
        indices=np.flatnonzero(abs(queries-t)<=1e-9)
        require(len(indices)==1,'Missing exact native horizon node')
        first=5*N+6*i;native=states[indices[0]];affine=np.empty(4)
        for j in range(2):
            affine[j]=c['q_lower'][j]+(Q_PADDING if padding else 0)-lo[first+j]+float(C[first+j]@z)
            affine[2+j]=-limits[3+j]+(DQ_PADDING if padding else 0)-lo[first+2+j]+float(C[first+2+j]@z)
        native_nodes.append(native);affine_nodes.append(affine)
        records.append(dict(node=i,task_time_s=float(t),relative_horizon_s=float(t-fixture['input']['t']),
            linearized_q=affine[:2].tolist(),native_q=native[:2].tolist(),
            linearized_dq=affine[2:].tolist(),native_dq=native[2:].tolist(),
            native_minus_linearized_dq=(native[2:]-affine[2:]).tolist()))
    native_nodes=np.asarray(native_nodes);affine_nodes=np.asarray(affine_nodes)
    qe=float(abs(native_nodes[:,:2]-affine_nodes[:,:2]).max());ve=float(abs(native_nodes[:,2:]-affine_nodes[:,2:]).max())
    require(abs(qe-d['q_linearization_error'])<=1e-10 and abs(ve-d['dq_linearization_error'])<=1e-10,
            'Native/linearized discrepancy differs from actual QP rows')
    require(qe<=.006 and ve<=.10,'Original nonlinear consistency threshold exceeded')
    envelope=array(d['candidate_envelope'],(8,));violations=[]
    for j in range(2):
        for label,violation in [('q_lower',c['q_lower'][j]-(envelope[j]-Q_PADDING)),
            ('q_upper',envelope[j+2]+Q_PADDING-c['q_upper'][j]),
            ('dq_lower',-limits[j+3]-(envelope[j+4]-DQ_PADDING)),
            ('dq_upper',envelope[j+6]+DQ_PADDING-limits[j+3])]:
            if violation>TOLERANCE:violations.append(dict(joint=j+1,bound=label,
                violation_before_tolerance=float(violation),violation_after_tolerance=float(violation-TOLERANCE)))
    require(violations,'Recorded bounds rejection has no independently observed envelope violation')
    min_candidates=extrema[(extrema[:,1]==0)&(extrema[:,2]==2)]
    require(len(min_candidates)>0,'No original acceleration-root evidence for joint 1')
    minimum=min_candidates[int(np.argmin(min_candidates[:,4]))]
    require(abs(minimum[4]-envelope[4])<=1e-12,'Recorded dq1 minimum disagrees with native acceleration root')
    at=float(minimum[0]);right=int(np.searchsorted(times,at));left=right-1
    require(0<=left<right<N and times[left]<at<times[right], 'dq1 minimum is not strictly between horizon nodes')
    dqnode_low=native_nodes[:,2:].min(axis=0);dqnode_high=native_nodes[:,2:].max(axis=0)
    node_violation=max(0.,float(np.max(-limits[3:]+DQ_PADDING-dqnode_low)),
        float(np.max(dqnode_high+DQ_PADDING-limits[3:])))
    reference_qp=C[11*N:13*N]@z
    require(np.all(reference_qp>=lo[11*N:13*N]-TOLERANCE)
        and np.all(reference_qp<=hi[11*N:13*N]+TOLERANCE),'Integrated reference QP rows failed')
    trust=[]
    for i in range(N):
        for j in range(2):
            row=13*N+2*i+j;column=5*i+3+j
            require(np.array_equal(C[row],np.eye(n)[column]) and abs(hi[row]-lo[row]-.04)<=1e-12,
                    'Original .02 trust radius/row identity changed')
            trust.append(max(0.,lo[row]-z[column],z[column]-hi[row]))
    require(max(trust)<=TOLERANCE,'Original normalized trust radius failed')
    return dict(verified=True,core_ms=d['compute_ms'],qp_primal_independently_recomputed=primal,
        qp_kkt_stationarity_recorded=d['kkt_stationarity'],same_original_qp_and_command_limits=True,
        command_slew_reference_history_and_future_packets_verified=True,trust_radius=.02,
        q_linearization_error_independently_recomputed=qe,dq_linearization_error_independently_recomputed=ve,
        candidate_envelope=envelope.tolist(),candidate_gate_violations=violations,
        native_node_dq_min=dqnode_low.tolist(),native_node_dq_max=dqnode_high.tolist(),
        native_nodes_dq_padding_violation=node_violation,
        node_comparison=records,minimum=dict(task_time_s=at,relative_horizon_s=at-fixture['input']['t'],
            joint=1,kind='native_acceleration_zero',q_rad=float(minimum[3]),dq_rad_s=float(minimum[4]),
            strictly_between_horizon_nodes=True,left_node=records[left],right_node=records[right],
            same_time_affine_value_available=False,
            affine_capture_scope='Only horizon-node Q/dq rows are available; no affine value is interpolated at the between-node acceleration root'),
        controller_call_passed=False,physical_validation=False)


def audit(fixture,old,new,input_sha):
    metadata=check_metadata(old,new,fixture,input_sha)
    comparison=compare_problem(old['diagnostics'],new['diagnostics'],fixture['config']['nodes'])
    before=inspect_candidate(fixture,old,False);after=inspect_candidate(fixture,new,True)
    require(after['native_nodes_dq_padding_violation']<=TOLERANCE,
            'New native nodes do not satisfy padded dq limits; cannot attribute solely to between-node extremum')
    return dict(kind='x650_joint_safety_margin_fixed_input_audit_v1',passed=True,complete=True,
        metadata=metadata,problem_change=comparison,baseline=before,padded=after,
        original_candidate_thresholds_preserved=True,margin_fix_solved_429=False,flight_passed=False,
        conclusion='Reviewed node padding is applied exactly. The remaining padded dq rejection is a native between-node acceleration-root extremum while all native horizon-node speeds satisfy the padded bound. Local surrogate differences are reported separately; no same-time affine value at the root was captured.',
        scope='Read-only fixed-input numerical audit, no optimizer/predictor rerun, no controller command or memory commit, no claim of recursive/robust feasibility')


def main(argv=None):
    p=argparse.ArgumentParser(description=__doc__)
    for name in ('fixture','baseline','padded','output'):p.add_argument('--'+name,type=Path,required=True)
    a=p.parse_args(argv);require(not a.output.exists(),'Refusing to overwrite audit evidence')
    inputs={k:replay.identity(getattr(a,k)) for k in ('fixture','baseline','padded')}
    try:
        result=audit(replay.strict_json(a.fixture),replay.strict_json(a.baseline),replay.strict_json(a.padded),inputs['fixture']['sha256'])
    except Exception as error:
        result=dict(kind='x650_joint_safety_margin_fixed_input_audit_v1',passed=False,complete=False,
            margin_fix_solved_429=False,flight_passed=False,error=repr(error),traceback=traceback.format_exc())
    require(all(replay.identity(v['path'])==v for v in inputs.values()),'Audit original inputs changed')
    result['artifacts']=dict(**inputs,auditor=replay.identity(__file__))
    a.output.write_text(json.dumps(result,indent=2,allow_nan=False)+'\n',encoding='utf-8')
    print(json.dumps({k:result.get(k) for k in ('passed','error','conclusion','margin_fix_solved_429')},indent=2))
    return 0 if result['passed'] else 1


if __name__=='__main__':raise SystemExit(main())
