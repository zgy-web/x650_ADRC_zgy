#!/usr/bin/env python3
"""Independent arithmetic audit of canonical-grid TS20 interval hard cuts.

This does not run the predictor, optimizer, simulator or actuator. The caller
must separately require canonical native repropagation for model consistency.
A verified rejection is rollback evidence, never an accepted control result.
"""
from copy import deepcopy
from pathlib import Path
import argparse
import json
import math
import sys
import traceback

import numpy as np
from scipy.optimize import nnls

sys.path.insert(0,str(Path(__file__).resolve().parent))
import audit_x650_recovery_replay as replay

Q_PAD=.0003
DQ_PAD=.03
TOL=2e-7


def require(condition,message):
    if not condition:raise ValueError(message)


def finite(value,shape=None):
    result=np.asarray(value,dtype=float)
    require(np.isfinite(result).all(),'Nonfinite arithmetic evidence')
    if shape is not None:
        require(-1 in shape or result.size==int(np.prod(shape)),'Arithmetic evidence shape differs')
        result=result.reshape(shape)
    return result


def close(actual,expected,tolerance=1e-12,label='array'):
    a,b=finite(actual),finite(expected)
    require(a.shape==b.shape and (not a.size or float(abs(a-b).max())<=tolerance),
            'Original '+label+' differs')


def canonical_grid(fixture,packets):
    """Recreate production query and integration grids without any cut time."""
    c,inp=fixture['config'],fixture['input'];t=inp['t'];h=[inp['dt']]+[c['node_dt']]*(c['nodes']-1)
    nodes=[];offset=0.
    for step in h:offset+=step;nodes.append(t+offset)
    end=nodes[-1];start=t-(inp['state']['control_time']-inp['state']['joint_stamp'])
    require(start<=t and packets,'Missing actual packet history or invalid source age')
    applied=[replay.number(p['send_time'])+c['motion'][4] for p in packets]
    queries=sorted(set([t,*nodes,*[at for at in applied if t<=at<=end]]))
    boundaries=sorted(set([start,*queries,*[at for at in applied if start<at<=end]]))
    return h,nodes,queries,boundaries


def point_limits(fixture,point):
    j,component=point['joint'],point['component'];c=fixture['config']
    require(type(j) is int and j in (0,1) and type(component) is int and component in (0,1),
            'Invalid hard-cut joint/component')
    require(type(point['source_kind']) is int and point['source_kind'] in (0,1,2,3),
            'Invalid hard-cut native point kind')
    if component==0:limits=(c['q_lower'][j]+Q_PAD,c['q_upper'][j]-Q_PAD)
    else:limits=(-c['velocity_limit'][3+j]+DQ_PAD,c['velocity_limit'][3+j]-DQ_PAD)
    close([point['lower'],point['upper']],limits,label='hard-cut padding/limit')
    replay.number(point['value']);replay.number(point['time'])
    return limits


def sliding_transition(c,duration,j):
    """Exact force-to-state kernel for the declared smooth sliding surrogate."""
    require(duration>=0,'Negative sliding-kernel interval')
    k=c['kp'][j]/c['motion'][0];damping=(c['motion'][1]+c['kd'][j])/c['motion'][0]
    half=-damping/2;disc=half*half-k
    if abs(disc)*duration*duration<1e-14:co,si=1.,duration
    else:
        w=math.sqrt(abs(disc));co=math.cosh(w*duration) if disc>0 else math.cos(w*duration)
        si=(math.sinh(w*duration) if disc>0 else math.sin(w*duration))/w
    A=np.asarray([[co-half*si,si],[-k*si,co+half*si]])*math.exp(half*duration)
    require(c['kp'][j]>0,'Audit sliding derivative requires original positive MIT Kp')
    return A,np.asarray([(1-A[0,0])/c['kp'][j],-A[1,0]/c['kp'][j]])


def encoding_guard(fixture,packets,plan,point):
    """Bound packet rounding through the smooth kernel, including centre bias.

    This is an encoding error bound for the sliding surrogate only. Friction,
    stiction, and branch changes still require the final canonical native gate.
    Actual immutable packet history has no hypothetical rounding perturbation.
    """
    c,inp=fixture['config'],fixture['input'];N=c['nodes'];values=finite(plan,(N,5))
    j,component=point['joint'],point['component'];time=replay.number(point['time'])
    h=np.asarray([inp['dt']]+[c['node_dt']]*(N-1));starts=np.r_[0.,np.cumsum(h)[:-1]]
    prefix=np.vstack((np.zeros(2),np.cumsum(h[:,None]*values[:,3:],axis=0)))
    reference=np.asarray(replay.reference_now(fixture));delay=c['motion'][4]
    epsilon=.5*(c['kp'][j]*(2*math.pi/(2**24))+c['kd'][j]/512)
    kernel_l1=0.;bias=0.;actual_count=len(inp['context']['packets'])
    for index in range(actual_count,len(packets)):
        packet=packets[index];begin=packet['send_time']+delay
        if begin>=time:continue
        end=packets[index+1]['send_time']+delay if index+1<len(packets) else time
        kernel=sliding_transition(c,time-begin,j)[1][component]
        if end<time:kernel-=sliding_transition(c,time-end,j)[1][component]
        relative=packet['send_time']-inp['t']
        active=max(0,min(N-1,int(np.searchsorted(starts,relative+1e-10,side='right'))-1))
        unquantized_q=reference[j]+prefix[active,j]+(relative-starts[active])*values[active,3+j]
        centre_error=c['kp'][j]*(packet['q_ref'][j]-unquantized_q)+c['kd'][j]*(packet['dq_ref'][j]-values[active,3+j])
        require(abs(centre_error)<=epsilon+1e-12,'Centre packet error exceeds the declared half-LSB encoding range')
        kernel_l1+=abs(kernel);bias+=kernel*centre_error
    radius=kernel_l1*epsilon
    return dict(encoding_lower_remainder=-radius-bias,encoding_upper_remainder=radius-bias,
        encoding_kernel_l1=kernel_l1,encoding_anchor_bias=bias,encoding_effort_half_lsb=epsilon)


def fast_branch_gradient(fixture,packets,boundaries,point):
    """Recompute the declared unquantized sliding-branch sensitivity only.

    This is linear algebra on the actual fixed packet grid. It is deliberately
    not presented as an event/quantizer derivative or a native model rerun.
    """
    c,inp=fixture['config'],fixture['input'];N=c['nodes'];n=5*N
    h=[inp['dt']]+[c['node_dt']]*(N-1);ends=np.cumsum(h);starts=np.r_[0.,ends[:-1]]
    actual_count=len(inp['context']['packets']);forcing=[]
    for i,p in enumerate(packets):
        F=np.zeros((2,n))
        if i>=actual_count:
            relative=p['send_time']-inp['t'];active=max(0,min(N-1,int(np.searchsorted(starts,relative+1e-10,side='right'))-1))
            for j in range(2):
                limit=c['velocity_limit'][3+j]
                for k in range(active):F[j,5*k+3+j]=c['kp'][j]*h[k]*limit
                F[j,5*active+3+j]=(c['kp'][j]*(relative-starts[active])+c['kd'][j])*limit
        forcing.append(F)
    knots=np.asarray([p['send_time']+c['motion'][4] for p in packets]);S=np.zeros((4,n));cursor=boundaries[0]
    for end in boundaries:
        packet=max(0,int(np.searchsorted(knots,cursor+1e-11,side='right'))-1)
        found=cursor-1e-12<=point['time']<=end+1e-12
        duration=float(np.clip(point['time']-cursor,0.,end-cursor)) if found else end-cursor
        if duration>0:
            for j in range(2):
                A,B=sliding_transition(c,duration,j)
                update=A@S[[j,j+2]]+B[:,None]*forcing[packet][j]
                S[j],S[j+2]=update[0],update[1]
        if found:return S[point['joint']+2*point['component']]
        cursor=end
    raise ValueError('Hard-cut sensitivity time is outside canonical horizon')


def centre_packets(fixture,plan):
    """Build only hypothetical quantized packets from actual immutable history."""
    c,inp=fixture['config'],fixture['input'];N=c['nodes'];values=finite(plan,(N,5))
    h=np.asarray([inp['dt']]+[c['node_dt']]*(N-1));ends=np.cumsum(h);starts=np.r_[0.,ends[:-1]]
    prefix=np.vstack((np.zeros(2),np.cumsum(h[:,None]*values[:,3:],axis=0)))
    reference=np.asarray(replay.reference_now(fixture));packets=deepcopy(inp['context']['packets'])
    period=c['bus_period'];phase=inp['context']['bus_origin'];t=inp['t']
    first=phase+math.ceil((t-phase)/period-1e-9)*period
    if first<=packets[-1]['send_time']+1e-10:first+=period
    stop=t+float(ends[-1])+1e-10;count=max(0,int(math.ceil((stop-first)/period)));increment=(first+period)-first
    def quantize(value,step):return math.copysign(math.floor(abs(value/step)+.5),value)*step
    for b in range(count):
        send=first+b*increment;relative=send-t
        active=max(0,min(N-1,int(np.searchsorted(starts,relative+1e-10,side='right'))-1))
        q=reference+prefix[active]+(relative-starts[active])*values[active,3:]
        packets.append(dict(send_time=send,q_ref=[quantize(v,2*math.pi/(2**24)) for v in q],
            dq_ref=[quantize(v,1/512) for v in values[active,3:]],tau_ff=[0.,0.],kp=c['kp'],kd=c['kd']))
    return packets


def check_interval_evidence(fixture,d):
    """Check copied cut anchors and exact appended QP inequalities."""
    c=fixture['config'];N=c['nodes'];n=5*N;base=15*N+5
    require(isinstance(d['interval_constraints'],list) and isinstance(d['interval_repairs'],list),
            'Missing interval cut/repair evidence')
    cuts=d['interval_constraints'];repairs=d['interval_repairs']
    require(type(d['interval_rows']) is int and d['interval_rows']==len(cuts)<=512,
            'Interval hard-cut row count/cap differs')
    require(d['qp_rows']==base+len(cuts) and d['qp_columns']==n,
            'Original base rows or appended hard-cut dimensions changed')
    require(len(repairs)<=2,'Interval repair control budget exceeded')
    cumulative=0;last_problem=0;generated=[];repair_summary=[]
    def match(point,prior):return point[1:]==prior[1:] and abs(point[0]-prior[0])<=1e-10
    for attempt in repairs:
        problem=attempt['control_subproblem']
        require(type(problem) is int and last_problem<problem<d['subproblems']<=3,
                'Interval repair not followed by an original-budget control solve')
        last_problem=problem
        points=attempt['points'];triggers=attempt['trigger_points']
        require(isinstance(points,list) and 1<=len(points)<=512
            and isinstance(triggers,list) and 1<=len(triggers)<=8,
                'Missing or excessive original violation points')
        require(0<=attempt['elapsed_ms']<=50.
            and attempt['canonical_substeps']>0,'Invalid interval generation budget or canonical evidence')
        envelope=finite(attempt['envelope'],(8,))
        require(isinstance(attempt['rejection'],str),'Missing interval generation status')
        require(attempt['risk_radius']==.02 and min(c['trust_radius'],.02)==.02,
                'Original risk-search radius changed')
        margins=finite(attempt['risk_margin'],(len(points),))
        encoding_lower=finite(attempt['risk_encoding_lower'],(len(points),))
        encoding_upper=finite(attempt['risk_encoding_upper'],(len(points),))
        require(np.all(margins>=0),'Negative hard-cut risk range')
        centre=finite(attempt['centre_plan'],(N,5));packets=centre_packets(fixture,centre)
        replay.check_packets(fixture,packets,centre.ravel().tolist())
        _,_,_,repair_boundaries=canonical_grid(fixture,packets)
        directions=set()
        for point in triggers:
            lo,hi=point_limits(fixture,point);value=point['value']
            require(value<lo-TOL or value>hi+TOL,'Interval batch lacks an actual native violation trigger')
            directions.add((point['joint'],point['component'],'lower' if value<lo-TOL else 'upper'))
            require(any(abs(p['time']-point['time'])<=1e-10 and p['joint']==point['joint']
                and p['component']==point['component'] and abs(p['value']-value)<=1e-12 for p in points),
                'Risk batch omitted an actual violation trigger')
        added=0
        for point,margin,encoding_lo,encoding_hi in zip(points,margins,encoding_lower,encoding_upper):
            lo,hi=point_limits(fixture,point);value=point['value']
            j,component=point['joint'],point['component']
            gradient=fast_branch_gradient(fixture,packets,repair_boundaries,point)
            expected_margin=.02*sum(abs(gradient[k]) for k in range(n) if k%5 in (3,4))
            require(abs(margin-expected_margin)<=1e-10,'Risk-point sliding search range differs')
            guard=encoding_guard(fixture,packets,centre,point)
            close([encoding_lo,encoding_hi],[guard['encoding_lower_remainder'],guard['encoding_upper_remainder']],
                1e-12,'risk-point encoding range')
            require(((j,component,'lower') in directions and value-margin+encoding_lo<=lo+TOL)
                or ((j,component,'upper') in directions and value+margin+encoding_hi>=hi-TOL),
                'Risk point lies outside a triggered hard-bound search range')
            require(point['source_kind'] in (0,2,3) or any(abs(p['time']-point['time'])<=1e-10
                and p['joint']==j and p['component']==component for p in triggers),
                'Untriggered integration-substep point entered the risk batch')
            lower_value=envelope[(0 if component==0 else 4)+j]
            upper_value=envelope[(2 if component==0 else 6)+j]
            require(lower_value-1e-12<=value<=upper_value+1e-12,'Generated point outside original candidate envelope')
            key=(point['time'],j,component)
            if not any(match(key,prior) for prior in generated):generated.append(key);added+=1
        cumulative+=added
        require(attempt['added_points']==added and attempt['total_points']==cumulative<=512,
                'Merged interval-point count differs')
        repair_summary.append(dict(control_subproblem=problem,added_points=added,total_points=cumulative,
            actual_violation_triggers=len(triggers),risk_radius=.02,elapsed_ms=attempt['elapsed_ms']))
    require(cumulative==len(cuts),'Final hard-cut list omits generated points')
    if not cuts:
        require(not repairs,'Interval repair occurred without hard cuts')
        return dict(verified=True,rows=0,repairs=[],canonical_grid_unchanged=True)
    plan=finite(d['interval_centre_plan'],(N,5));scale=finite(c['velocity_limit'],(5,));z=(plan/scale).ravel()
    packets=d['nominal_packets']
    replay.check_packets(fixture,packets,plan.ravel().tolist())
    _,_,centre_queries,boundaries=canonical_grid(fixture,packets)
    close(d['interval_canonical_boundaries'],boundaries,label='interval canonical integration grid')
    states=finite(d['interval_canonical_states'],(len(boundaries),4))
    close(states[0],fixture['input']['state']['q']+fixture['input']['state']['dq'],label='interval canonical initial state')
    nominal=finite(d['nominal_states'],(len(centre_queries),4))
    for i,t in enumerate(centre_queries):
        index=np.flatnonzero(abs(np.asarray(boundaries)-t)<=1e-12)
        require(len(index)==1,'Canonical centre query absent from integration grid')
        close(states[index[0]],nominal[i],1e-12,'canonical centre/actual nominal state')
    centre_envelope=finite(d['interval_canonical_envelope'],(8,))
    require(np.all(centre_envelope[:2]<=centre_envelope[2:4])
        and np.all(centre_envelope[4:6]<=centre_envelope[6:8]),'Canonical centre envelope is unordered')
    require(np.all(states.min(axis=0)>=centre_envelope[[0,1,4,5]]-1e-12)
        and np.all(states.max(axis=0)<=centre_envelope[[2,3,6,7]]+1e-12),
        'Canonical centre envelope omits a captured boundary state')
    require(d['interval_canonical_substeps']==d['candidate_substeps']>0,
            'Canonical centre integration substep count changed')
    # The centre used to construct the last QP can itself violate its new cuts;
    # it is a linearization anchor, not a trusted/executable fallback.
    C=finite(d['C'],(d['qp_rows'],n));lo=finite(d['lower']);hi=finite(d['upper']);keys=set()
    for i,cut in enumerate(cuts):
        point=cut['point'];point_limits(fixture,point);row=base+i
        key=(point['time'],point['joint'],point['component'])
        require(not any(match(key,prior) for prior in keys),'Repeated final hard-cut source identity')
        keys.add(key)
        require(any(match(key,prior) for prior in generated) and cut['qp_row']==row,
                'Hard-cut source identity or original appended row differs')
        require(boundaries[0]<=point['time']<=boundaries[-1],'Hard cut is outside canonical prediction horizon')
        gradient=finite(cut['gradient'],(n,));offset=replay.number(cut['affine_offset'])
        close(gradient,fast_branch_gradient(fixture,packets,boundaries,point),1e-10,
              'unquantized sliding-branch cut sensitivity')
        j,component=point['joint'],point['component']
        require(centre_envelope[(0 if component==0 else 4)+j]-1e-12<=point['value']
            <=centre_envelope[(2 if component==0 else 6)+j]+1e-12,
            'Hard-cut anchor is outside copied canonical centre envelope')
        close(C[row],gradient,label='appended hard-cut gradient')
        guard=encoding_guard(fixture,packets,plan,point)
        for name,expected in guard.items():close([cut[name]],[expected],1e-12,'hard-cut '+name)
        close([lo[row],hi[row]],
            [point['lower']-offset-guard['encoding_lower_remainder'],point['upper']-offset-guard['encoding_upper_remainder']],
            label='appended hard-cut encoding bounds')
        require(abs(offset+float(gradient@z)-point['value'])<=1e-10,
                'Hard-cut affine anchor changed the canonical centre value')
        require(np.max(abs(gradient))>1e-14 or point['lower']-TOL<=point['value']<=point['upper']+TOL,
                'Uncontrollable violating history was hidden in a constant hard cut')
    require(len(keys)==len(cuts),'Repeated final hard-cut source identity')
    return dict(verified=True,rows=len(cuts),repairs=repair_summary,canonical_grid_unchanged=True,
        encoding_scope='Offset-corrected half-LSB bound for the sliding surrogate only; final native envelope still mandatory',
        dense_anchor_scope='Copied canonical observer anchor with exact QP row identity; separate native repropagation remains required')


def check_qp(fixture,d):
    c=fixture['config'];N=c['nodes'];n=5*N;m=d['qp_rows'];base=15*N+5
    H=finite(d['H'],(n,n));g=finite(d['g'],(n,));C=finite(d['C'],(m,n))
    lo=finite(d['lower'],(m,));hi=finite(d['upper'],(m,));z=finite(d['solution'],(n,))
    require(np.all(lo<=hi),'QP bounds unordered')
    close(H,H.T,1e-8,'Hessian symmetry');np.linalg.cholesky(H)
    primal=max(0.,float((lo-C@z).max()),float((C@z-hi).max()),float((abs(z)-1).max()))
    require(primal<=TOL and 0<=d['primal_violation']<=TOL,'Full original/cut/box QP residual exceeded')
    normals=np.vstack((C,-C,np.eye(n),-np.eye(n)));limits=np.r_[hi,-lo,np.ones(2*n)]
    slack=limits-normals@z;gradient=H@z+g;active=slack<1e-6
    if active.any():multipliers,_=nnls(normals[active].T,-gradient,maxiter=max(1000,30*int(active.sum())))
    else:multipliers=np.empty(0)
    stationarity=float(abs(gradient+normals[active].T@multipliers).max())
    require(stationarity<=.002 and 0<=d['kkt_stationarity']<=.002,'Independent full-row KKT stationarity exceeded')
    # New cuts are appended after all original rows. Never locate the trust
    # region relative to the new matrix tail.
    for i in range(N):
        for j in range(2):
            row=13*N+2*i+j;column=5*i+3+j
            close(C[row],np.eye(n)[column],0.,'original trust row')
            require(abs(hi[row]-lo[row]-.04)<=1e-12,'Original normalized .02 trust width changed')
    require(base<=m and m-base==d['interval_rows'],'Unreviewed QP variable/slack count or original row layout')
    plan=finite(d['plan'],(N,5));close(plan,z.reshape(N,5)*finite(c['velocity_limit']),1e-10,'normalized plan')
    return dict(verified=True,columns=n,rows=m,no_slack_variables=True,
        primal_violation=primal,kkt_stationarity_independent=stationarity,original_trust_radius=.02)


def audit(fixture,raw,input_sha,expected_libraries):
    require(raw.get('kind')=='x650_cpp_recovery_raw_replay_v1' and raw.get('schema_version')==1
        and raw.get('complete') is True and raw.get('core_abi')==3,'Unsupported interval replay schema/ABI')
    require(raw.get('input_sha256')==input_sha,'Raw replay input hash differs')
    require(set(expected_libraries)=={'core','optimization','predictor'},'Three expected library hashes required')
    for key,digest in expected_libraries.items():
        require(isinstance(digest,str) and len(digest)==64 and not set(digest)-set('0123456789abcdef'),
                'Malformed expected library SHA256: '+key)
        require(raw['loaded_code'][key]['sha256']==digest,'Loaded expected library differs: '+key)
    scope=raw['scope'];expected=dict(offline_fixed_snapshot=True,actuator_io=False,
        closed_loop_or_physical_validation=False,injected_delay=False,original_core_deadline_ms=50.,initial_trust_radius=.02)
    require(all(scope.get(k)==v for k,v in expected.items()),'Interval replay scope or original hard policy changed')
    require(raw['memory_before']==fixture['memory'],'Original fixed-input memory differs')
    require(raw['unexpected_exception'] is False and raw['returned'] is not raw['core_error'],
            'Unexpected native return/error classification')
    same=raw['memory_before']==raw['memory_after']
    require(raw['memory_unchanged'] is same,'Reported memory rollback differs from complete values')
    result=dict(kind='x650_interval_constraint_independent_audit_v1',passed=True,complete=True,
        input_sha256=input_sha,controller_call_accepted=raw['returned'],physical_validation=False,
        native_repropagation_independent=False,
        scope='Read-only arithmetic and original evidence audit; parent driver separately requires canonical native repropagation')
    if not raw['returned']:
        require(same and raw['output'] is None and raw['memory_commit_valid'] is False
            and isinstance(raw['error'],str) and raw['error'],'Rejected call changed memory or invented command')
        result.update(memory_rollback_verified=True,controller_call_accepted=False,error=raw['error'],
            accepted_control_validated=False,rejection_scope='Rollback only; rejection is not an accepted control/flight result')
        return result
    require(raw['error'] is None and raw['memory_commit_valid'] is True,'Accepted call reports error/noncommit')
    c=fixture['config'];d=raw['diagnostics'];N=c['nodes']
    require(N==15 and c['feasibility_tolerance']==TOL and c['kkt_tolerance']==.002,'Original numerical policy differs')
    require(all(d.get(k) is True for k in ('accepted','nonlinear_passed','actual_bounds'))
        and d['previous_plan_fallback'] is False,'Accepted call lacks original candidate gates')
    require(0<=raw['core_step_wall_ms']<=50. and 0<=d['compute_ms']<=50.
        and 1<=d['subproblems']<=3 and len(d['attempts'])==d['subproblems'],
        'Accepted interval call exceeded original control QP/deadline budget')
    out=raw['output']
    for key,size in replay.OUTPUT_DIMENSIONS.items():replay.vector(out[key],size)
    close(out['command'],d['plan'][:5],0.,'actual first command')
    close(out['acceleration_feedforward'],[0.,0.,0.],0.,'zero acceleration feedforward')
    packets=d['candidate_packets'];replay.check_packets(fixture,packets,d['plan'])
    h,nodes,queries,boundaries=canonical_grid(fixture,packets)
    close(d['horizon'],h,1e-12,'actual horizon')
    close(d['query_times'],queries,1e-12,'canonical query grid')
    close(d['candidate_boundary_times'],boundaries,1e-12,'canonical integration grid')
    states=finite(d['candidate_states'],(len(queries),4));predicted=finite(d['predicted_states'],(N,12))
    linear=finite(d['linearized_states'],(N,12));actual=[]
    for t in nodes:
        indices=np.flatnonzero(abs(np.asarray(queries)-t)<=1e-9)
        require(len(indices)==1,'Canonical node state missing');actual.append(states[indices[0]])
    actual=np.asarray(actual)
    close(predicted[:,6:10],actual,1e-13,'committed actual nonlinear nodes')
    qe=float(abs(actual[:,:2]-linear[:,6:8]).max());ve=float(abs(actual[:,2:]-linear[:,8:10]).max())
    require(qe<=.006 and ve<=.10 and abs(qe-d['q_linearization_error'])<=1e-10
        and abs(ve-d['dq_linearization_error'])<=1e-10,'Independent nonlinear surrogate discrepancy differs')
    envelope=replay.check_envelope(fixture,d['candidate_envelope'])
    refs=replay.node_references(fixture,d['plan']);replay.check_plan(fixture,d['plan'],refs)
    expected_memory=deepcopy(fixture['memory']);inp=fixture['input']
    expected_memory.update(plan=d['plan'],plan_h=h,plan_t=inp['t'],previous_command=out['command'],
        last_t=inp['t'],has_last_t=True,previous_stamp=inp['state']['stamp'],has_stamp=True,
        origin=inp['state']['control_time']-inp['t'],has_origin=True,step_count=expected_memory['step_count']+1)
    history=expected_memory['joint_history']+[dict(time=inp['t'],velocity=out['joint_velocity'])]
    expected_memory['joint_history']=[v for v in history if v['time']>=inp['t']-.75]
    require(raw['memory_after']==expected_memory,'Accepted call committed unexpected memory/history')
    interval=check_interval_evidence(fixture,d);qp=check_qp(fixture,d)
    if d['recovery_triggered']:
        replay.check_recovery(fixture,d)
        require(replay.stopping_violation(fixture,actual.ravel().tolist())<=TOL,
                'Recovery accepted candidate violates native stopping reserve')
    else:require(d['recovery_attempts']==[] and d['recovery_reason']=='','Untriggered recovery evidence differs')
    result.update(accepted_control_validated=True,memory_commit_verified=True,qp=qp,interval=interval,
        canonical_grid_verified=True,nonlinear_envelope_violation=envelope,q_error=qe,dq_error=ve,
        hard_margin_preserved=dict(q_padding=Q_PAD,dq_padding=DQ_PAD,tolerance=TOL),
        core_step_wall_ms=raw['core_step_wall_ms'],core_ms=d['compute_ms'])
    return result


def main(argv=None):
    p=argparse.ArgumentParser(description=__doc__)
    for name in ('fixture','raw','expected-libraries','output'):p.add_argument('--'+name,type=Path,required=True)
    a=p.parse_args(argv);require(not a.output.exists(),'Refusing to overwrite interval evidence')
    paths={k:replay.identity(getattr(a,k)) for k in ('fixture','raw','expected_libraries')}
    try:result=audit(replay.strict_json(a.fixture),replay.strict_json(a.raw),paths['fixture']['sha256'],replay.strict_json(a.expected_libraries))
    except Exception as error:result=dict(kind='x650_interval_constraint_independent_audit_v1',passed=False,complete=False,
        controller_call_accepted=False,accepted_control_validated=False,physical_validation=False,
        error=repr(error),traceback=traceback.format_exc())
    require(all(replay.identity(v['path'])==v for v in paths.values()),'Original interval audit evidence changed')
    result['artifacts']=dict(**paths,auditor=replay.identity(__file__))
    a.output.write_text(json.dumps(result,indent=2,allow_nan=False)+'\n',encoding='utf-8')
    print(json.dumps({k:result.get(k) for k in ('passed','controller_call_accepted','accepted_control_validated','error')},indent=2))
    return 0 if result['passed'] else 1


if __name__=='__main__':raise SystemExit(main())
