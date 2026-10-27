"""Offline model qualification; no solver, controller or actuator publication.

The DOP853 oracle uses a globally reconstructed packet timeline, not the
fixed-size stage schedule. Native event-model disagreement is reported as a
model difference, never silently treated as an integration error or waived.
"""
from __future__ import annotations

import argparse
from dataclasses import replace
import hashlib
import json
from pathlib import Path
import subprocess

import casadi as ca
import numpy as np
from scipy.integrate import solve_ivp
from scipy.linalg import expm

from x650_nmpc_standard.config import ModelConfig, SurrogateConfig
from x650_nmpc_standard.model import build_case, build_model, rollout, _endpoint


def plain(v):
    if isinstance(v,np.ndarray):return v.tolist()
    if isinstance(v,np.generic):return v.item()
    if isinstance(v,dict):return {k:plain(x) for k,x in v.items()}
    if isinstance(v,(list,tuple)):return [plain(x) for x in v]
    return v


def write(path,value):
    path.write_text(json.dumps(plain(value),indent=2,allow_nan=False)+'\n')


def digest(path):return hashlib.sha256(path.read_bytes()).hexdigest()


def independent_joint(config,fixture,numerical,plan,surrogate):
    """Global event timeline + high-accuracy adaptive smooth ODE reference."""
    c=ModelConfig.from_dict(config);i=fixture['input'];t=i['t'];age=numerical.metadata['original_joint_age_s']
    start=t-age;ends=t+np.cumsum(numerical.horizon);reference=numerical.x0[12:14]
    packets=[]
    for packet in i['context']['packets']:
        force=np.array(packet['kp'])*packet['q_ref']+np.array(packet['kd'])*packet['dq_ref']+packet['tau_ff']
        packets.append((packet['send_time']+c.motion[4],force))
    # Independently compute the first global bus tick, including a send at t
    # only when the real history does not already include that tick.
    origin=i['context']['bus_origin']; tick=int(np.ceil((t-origin)/c.bus_period-1e-9))
    while origin+tick*c.bus_period<=i['context']['packets'][-1]['send_time']+1e-10:tick+=1
    while origin+tick*c.bus_period<ends[-1]-1e-10:
        send=origin+tick*c.bus_period
        k=min(np.searchsorted(ends,send+1e-10,side='right'),c.nodes-1)
        starts=np.r_[t,ends[:-1]];r=reference.copy()
        for j in range(k):r+=numerical.horizon[j]*plan[j,3:]
        r+=(send-starts[k])*plan[k,3:]
        packets.append((send+c.motion[4],c.kp*r+c.kd*plan[k,3:]))
        tick+=1
    packets.sort(key=lambda a:a[0]);eligible=[p for p in packets if p[0]<=start+1e-10]
    if not eligible:raise AssertionError('Global timeline lacks initial actual forcing')
    active=eligible[-1][1];qv=np.r_[i['state']['q'],i['state']['dq']];cursor=start
    events=sorted(set([t,*ends]+[p[0] for p in packets if start+1e-10<p[0]<=ends[-1]+1e-10]))
    outputs=[];aligned=None
    def rhs(_,y):
        q,v=y[:2],y[2:]
        friction=(c.motion[2]+c.motion[6]*np.exp(-(v/c.motion[7])**2))*np.tanh(v/surrogate.velocity_smoothing_rad_s)
        return np.r_[v,(active-c.kp*q-(c.kd+c.motion[1])*v-c.motion[3]-friction)/c.motion[0]]
    for event in events:
        if event>cursor+1e-14:
            solution=solve_ivp(rhs,(cursor,event),qv,method='DOP853',rtol=1e-11,atol=1e-13)
            if not solution.success:raise AssertionError(solution.message)
            qv=solution.y[:,-1]
        cursor=event
        if abs(event-t)<1e-9:aligned=qv.copy()
        for k,end in enumerate(ends):
            if abs(event-end)<1e-9:
                if not outputs or k>=len(outputs):outputs.append(qv.copy())
        changes=[p for p in packets if abs(p[0]-event)<1e-10]
        if changes:active=changes[-1][1]
    if len(outputs)!=c.nodes or aligned is None:raise AssertionError('Independent timeline output coverage')
    return aligned,np.array(outputs)


def independent_base(c,numerical,plan):
    x=numerical.x0.copy();rows=[x.copy()]
    for u,h in zip(plan,numerical.horizon):
        for j in range(2):
            wn,zeta=c.response[2*j:2*j+2]
            augmented=np.array([[0.,1.,0.,0.],[0.,0.,1.,0.],
                                [0.,-wn*wn,-2*wn*zeta,wn*wn],[0.,0.,0.,0.]])
            value=expm(h*augmented)@np.array([x[j],x[3+j],x[10+j],u[j]])
            x[j],x[3+j],x[10+j]=value[:3]
        tau=c.response[4]
        value=expm(h*np.array([[0.,1.,0.],[0.,-1/tau,1/tau],[0.,0.,0.]]))@np.array([x[2],x[5],u[2]])
        x[2],x[5]=value[:2];rows.append(x.copy())
    return np.array(rows)


def endpoint_velocity_check(c,s):
    x=ca.SX.sym('direct_x',23);u=ca.SX.sym('direct_u',5);ctx=ca.SX.sym('direct_ctx',20)
    tip,velocity,*_=_endpoint(x,u,ctx,c,s,ca);fn=ca.Function('direct_endpoint',[x,u,ctx],[tip,velocity])
    xv=np.zeros(23);xv[:3]=[.1,-.2,1.3];xv[3:6]=[.18,-.08,.03];xv[6:10]=[-.15,.2,.12,-.09];xv[10:12]=[.24,-.17]
    uv=np.array([.25,-.12,.09,.1,-.1]);yaw=.34;yr=.19
    z=.12;E=np.array([[np.cos(z),-np.sin(z),0],[np.sin(z),np.cos(z),0],[0,0,1]])
    Ed=E@np.array([[0,.09,.02],[-.09,0,-.04],[-.02,.04,0]])
    context=np.r_[yaw,yr,E.ravel(),Ed.ravel()]
    derivative=np.zeros(23);derivative[:3]=xv[3:6];derivative[3:6]=np.r_[xv[10:12],(uv[2]-xv[5])/c.response[4]]
    derivative[6:8]=xv[8:10]
    derivative[10:12]=[c.response[2*j]**2*(uv[j]-xv[3+j])-2*c.response[2*j]*c.response[2*j+1]*xv[10+j] for j in range(2)]
    ctxd=np.r_[yr,0.,Ed.ravel(),np.zeros(9)];eps=1e-6
    plus=np.array(fn(xv+eps*derivative,uv,context+eps*ctxd)[0]).ravel()
    minus=np.array(fn(xv-eps*derivative,uv,context-eps*ctxd)[0]).ravel()
    analytic=np.array(fn(xv,uv,context)[1]).ravel();fd=(plus-minus)/(2*eps)
    return dict(analytic=analytic,independent_kinematic_derivative=fd,max_abs_error=float(np.max(abs(analytic-fd))),
                nonzero_yaw_rate=yr,nonzero_residual_rotation_rate=Ed)


def derivatives_and_cost(model,numerical,plan,trajectory):
    """Finite-difference AD check plus explicit six-block objective arithmetic."""
    c=model.config;L=model.parameter_layout
    gradient=ca.Function('model_cost_gradient',[model.x,model.u,model.p],
                         [ca.jacobian(ca.vertcat(model.discrete,model.cost),ca.vertcat(model.x,model.u))])
    direction=np.random.default_rng(912).normal(size=28);direction/=np.linalg.norm(direction)
    x=numerical.x0;u=plan[0];p=numerical.parameters[0];eps=2e-6
    def values(x,u):return np.r_[np.array(model.functions['discrete'](x,u,p)).ravel(),float(model.functions['cost'](x,u,p))]
    fd=(values(x+eps*direction[:23],u+eps*direction[23:])-values(x-eps*direction[:23],u-eps*direction[23:]))/(2*eps)
    ad=np.array(gradient(x,u,p))@direction
    scaled_error=float(np.max(abs(fd-ad)/(1+abs(ad))))
    assert scaled_error<2e-7,'Discrete graph AD inconsistent with direct finite difference'
    components=np.zeros(6)
    for k,u in enumerate(plan):
        p=numerical.parameters[k];xn=trajectory['states'][k+1];xp=trajectory['states'][k];e=trajectory['endpoints'][k]
        residuals=[e[:3]-p[L['target_position']],e[3:]-p[L['target_velocity']],
                   xn[:3]-p[L['nominal_position']],xn[6:8]-p[L['nominal_q']],
                   (u-p[L['nominal_control']])/c.velocity_limit,
                   (u-xp[18:23])/(numerical.horizon[k]*c.acceleration_limit)]
        components+=numerical.horizon[k]*c.weights*np.array([float(r@r) for r in residuals])
    error=float(np.max(abs(components-trajectory['components'])))
    assert error<1e-12 and abs(float(components.sum())-trajectory['cost'])<1e-12,'Six-term objective arithmetic mismatch'
    return dict(ad_directional_relative_error=scaled_error,six_component_arithmetic_error=error,independent_components=components)


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--selection',action='append',required=True)
    parser.add_argument('--output',required=True);parser.add_argument('--event-replay',required=True)
    args=parser.parse_args();out=Path(args.output);out.mkdir(parents=True,exist_ok=False)
    selections=[json.loads(Path(p).read_text()) for p in args.selection]
    config=selections[0]['config'];assert all(a['config']==config for a in selections)
    c=ModelConfig(config);s=SurrogateConfig();fine=replace(s,rk4_substeps_per_bus=20,history_rk4_substeps_per_bus=40)
    model=build_model(c,s);model_fine=build_model(c,fine);rows=[];native_input={'config':config,'cases':[]}
    for selection_index,selection in enumerate(selections):
        for wrapper in selection['cases']:
            case=wrapper['case'];identity=json.dumps(case,sort_keys=True);numerical=build_case(c,wrapper,s)
            fine_case=build_case(c,wrapper,fine);plan=numerical.initial_plan
            coarse=rollout(model,numerical,plan);refined=rollout(model_fine,fine_case,plan)
            aligned,joint=independent_joint(c,case,numerical,plan,s)
            base=independent_base(c,numerical,plan);basecols=[0,1,2,3,4,5,10,11]
            qerr=coarse['states'][1:,6:8]-joint[:,:2];dqerr=coarse['states'][1:,8:10]-joint[:,2:]
            half=coarse['states'][:,6:10]-refined['states'][:,6:10]
            r=float(np.max(abs(coarse['states'][:,basecols]-base[:,basecols])))
            assert json.dumps(case,sort_keys=True)==identity,'Original case mutated'
            timeline_pass=max(np.max(abs(qerr)),np.max(abs(dqerr)))<2e-6
            half_pass=np.max(abs(half))<2e-6
            assert r<1e-11,'Base transition differs from independent augmented expm'
            alignment_error=float(np.max(abs(numerical.x0[6:10]-aligned)))
            fine_alignment_error=float(np.max(abs(fine_case.x0[6:10]-aligned)))
            fine_timeline_error=float(np.max(abs(refined['states'][1:,6:10]-joint)))
            native_input['cases'].append(dict(id=case['id'],memory=case['memory'],input=case['input'],plan=plan.ravel().tolist()))
            checks=derivatives_and_cost(model,numerical,plan,coarse) if not rows else None
            rows.append(dict(id=case['id'],source=wrapper.get('source'),source_index=wrapper.get('source_index'),
                             dt_s=numerical.horizon[0],initial_pending_packet_count=numerical.metadata['initial_pending_packet_count'],
                             maximum_recorded_joint_speed=np.max(np.abs(case['input']['state']['dq'])),
                             maximum_planned_joint_command=np.max(abs(plan[:,3:])),
                             global_timeline_q_error_max=float(np.max(abs(qerr))),global_timeline_dq_error_max=float(np.max(abs(dqerr))),
                             halfstep_q_error_max=float(np.max(abs(half[:,:2]))),halfstep_dq_error_max=float(np.max(abs(half[:,2:]))),
                             timeline_accuracy_passed=timeline_pass,step_halving_passed=half_pass,declared_integration_tolerance=2e-6,
                             actual_history_alignment_error_max=alignment_error,alignment_accuracy_passed=alignment_error<2e-6,
                             fine_actual_history_alignment_error_max=fine_alignment_error,fine_timeline_error_max=fine_timeline_error,
                             base_expm_error_max=r,smooth_states=coarse['states'],smooth_endpoints=coarse['endpoints'],
                             smooth_cost=coarse['cost'],case_metadata=numerical.metadata,independent_objective_and_ad_check=checks))
    velocity=endpoint_velocity_check(c,s);assert velocity['max_abs_error']<1e-7,'Rdot endpoint velocity mismatch'
    write(out/'event_inputs.json',native_input)
    process=subprocess.run([args.event_replay,str(out/'event_inputs.json'),str(out/'event_results.json')],capture_output=True,text=True)
    (out/'event_stdout.txt').write_text(process.stdout+process.stderr)
    if process.returncode:raise RuntimeError('Original event replay failed: '+process.stderr)
    native=json.loads((out/'event_results.json').read_text())
    assert len(native['records'])==len(rows)
    direct_tip,direct_velocity,*_=_endpoint(model.x,model.u,model.p[model.parameter_layout['attitude_context']],c,s,ca)
    direct_endpoint=ca.Function('native_joint_endpoint',[model.x,model.u,model.p],[ca.vertcat(direct_tip,direct_velocity)])
    fixtures=[wrapper for selection in selections for wrapper in selection['cases']]
    for row,result,fixture in zip(rows,native['records'],fixtures):
        assert row['id']==result['id'];row['event_plan_passed']=result['passed'];row['event_plan_reason']=result['reason']
        if len(result['node_states'])==4*c.nodes:
            event=np.array(result['node_states']).reshape(c.nodes,4);smooth=row['smooth_states'][1:,6:10]
            error=smooth-event;row['event_node_states']=event;row['smooth_minus_event_nodes']=error
            row['model_q_max_abs_error']=np.max(abs(error[:,:2]),axis=0)
            row['model_dq_max_abs_error']=np.max(abs(error[:,2:]),axis=0)
            row['model_q_rmse']=np.sqrt(np.mean(error[:,:2]**2,axis=0))
            row['model_dq_rmse']=np.sqrt(np.mean(error[:,2:]**2,axis=0))
            numerical=build_case(c,fixture,s);event_endpoints=[]
            for k,u in enumerate(numerical.initial_plan):
                xn=np.array(row['smooth_states'][k+1]).copy();xn[6:10]=event[k]
                event_endpoints.append(np.array(direct_endpoint(xn,u,numerical.parameters[k])).ravel())
            endpoint_difference=np.array(row['smooth_endpoints'])-event_endpoints
            row['event_endpoints_same_base']=event_endpoints
            row['model_endpoint_position_error_max_m']=float(np.max(np.linalg.norm(endpoint_difference[:,:3],axis=1)))
            row['model_endpoint_velocity_error_max_m_s']=float(np.max(np.linalg.norm(endpoint_difference[:,3:],axis=1)))
        else:row['event_prediction_unavailable']=True
    qualified=all(r['timeline_accuracy_passed'] and r['step_halving_passed'] and r['alignment_accuracy_passed'] for r in rows)
    report=dict(kind='x650_standard_model_qualification_v1',integration_and_schedule_passed=qualified,
                all_original_event_plans_accepted=native['all_accepted'],event_comparable_count=sum('model_q_rmse' in r for r in rows),
                physical_fit_acceptance_not_claimed=True,hardware_qualified=False,count=len(rows),
                model_metadata=model.metadata,endpoint_velocity_independent_check=velocity,rows=rows,
                source_selections=[dict(path=p,sha256=digest(Path(p))) for p in args.selection],
                event_replay_sha256=digest(Path(args.event_replay)),
                qualification_source_sha256=digest(Path(__file__)),
                model_source_sha256=digest(Path(__file__).resolve().parents[1]/'python/x650_nmpc_standard/model.py'),
                config_source_sha256=digest(Path(__file__).resolve().parents[1]/'python/x650_nmpc_standard/config.py'),
                limitations=['Friction and future quantization are new smooth assumptions, not reidentified hardware parameters.',
                             'Comparison uses unchanged committed-plan warm starts, not newly optimized plans.',
                             'DOP853/step halving validates the declared smooth equation and time schedule; event disagreement remains model mismatch.',
                             'Original native continuous envelope and held-command validation remains mandatory.'])
    write(out/'report.json',report)
    print(json.dumps(dict(count=len(rows),integration_and_schedule_passed=qualified,event_comparable=report['event_comparable_count'],
                          event_accepted=native['accepted_count'],output=str(out/'report.json'))))


if __name__=='__main__':main()
