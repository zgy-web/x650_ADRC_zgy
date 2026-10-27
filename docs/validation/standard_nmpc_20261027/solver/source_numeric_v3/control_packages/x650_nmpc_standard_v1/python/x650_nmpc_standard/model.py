"""One differentiable discrete OCP for both acados and IPOPT.

The 23-state model retains known bus times and one pending packet because the
identified combined delay is shorter than one 5 ms bus period.  The numerical
schedule is fixed before optimization; future forcing is smooth in x/u.  A
smooth friction/identity-quantizer surrogate is explicit.  The original event
model remains a separate final validator and is never represented as this ODE.
"""
from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Any

import numpy as np

from .config import ModelConfig, SurrogateConfig, canonical_sha256, validate_surrogate

STATE_LAYOUT = dict(position=slice(0,3), velocity=slice(3,6), q=slice(6,8),
                    dq=slice(8,10), horizontal_acceleration=slice(10,12),
                    reference=slice(12,14), active_force=slice(14,16),
                    pending_force=slice(16,18), previous_control=slice(18,23))


def parameter_layout(slots=22):
    sizes = [("h",1),("attitude_context",20),("target_position",3),
             ("target_velocity",3),("nominal_position",3),("nominal_q",2),
             ("nominal_control",5),("durations",slots),("apply_new",slots),
             ("apply_pending",slots),("send_relative",slots),("sample_mask",slots),
             ("pending_at_end",1),("pending_send_relative",1),
             ("stopping_reserve",2),("control_lower",5),("control_upper",5)]
    result={}; start=0
    for key,size in sizes:
        result[key]=slice(start,start+size);start+=size
    result["size"]=start
    return result


@dataclass
class StageModel:
    name: str
    x: Any
    u: Any
    p: Any
    discrete: Any
    cost: Any
    residual: Any
    constraints: Any
    constraint_lower: np.ndarray
    constraint_upper: np.ndarray
    control_lower: np.ndarray
    control_upper: np.ndarray
    terminal_cost: Any
    terminal_constraints: Any
    state_layout: dict
    parameter_layout: dict
    config: ModelConfig
    surrogate: SurrogateConfig
    functions: dict
    constraint_layout: dict
    metadata: dict


@dataclass
class NumericalCase:
    x0: np.ndarray
    parameters: np.ndarray
    initial_plan: np.ndarray
    horizon: np.ndarray
    metadata: dict


def _friction(v,c,s,exp,tanh):
    return (c.motion[2]+c.motion[6]*exp(-(v/c.motion[7])**2))*tanh(v/s.velocity_smoothing_rad_s)


def _joint_rhs(joint,force,c,s,vertcat,exp,tanh):
    q,v=joint[:2],joint[2:4]
    friction=_friction(v,c,s,exp,tanh)
    dv=(force-c.kp*q-(c.motion[1]+c.kd)*v-c.motion[3]-friction)/c.motion[0]
    return vertcat(v,dv)


def _rk4(joint, force, duration, c, s, *, symbolic=None, substeps=None):
    if symbolic is None:
        rhs=lambda y:_joint_rhs(y,force,c,s,lambda *a:np.concatenate(a),np.exp,np.tanh)
    else:
        rhs=lambda y:_joint_rhs(y,force,c,s,symbolic.vertcat,symbolic.exp,symbolic.tanh)
    steps=s.rk4_substeps_per_bus if substeps is None else substeps
    h=duration/steps
    for _ in range(steps):
        k1=rhs(joint);k2=rhs(joint+h*k1/2);k3=rhs(joint+h*k2/2);k4=rhs(joint+h*k3)
        joint=joint+h*(k1+2*k2+2*k3+k4)/6
    return joint


def _base_step(x,u,h,c,ca):
    """Exact old closed-loop horizontal/vertical ZOH transition."""
    pos=[None]*3;vel=[None]*3;acc=[]
    for j in range(2):
        wn,zeta=c.response[2*j:2*j+2];decay=zeta*wn;disc=wn*wn-decay*decay
        if abs(disc)<1e-14:
            co=1.;si=h
        elif disc>0:
            freq=math.sqrt(disc);co=ca.cos(freq*h);si=ca.sin(freq*h)/freq
        else:
            freq=math.sqrt(-disc);co=ca.cosh(freq*h);si=ca.sinh(freq*h)/freq
        damping=ca.exp(-decay*h)
        a00=damping*(co+decay*si);a01=damping*si
        a10=-wn*wn*a01;a11=damping*(co-decay*si)
        vn=u[j]+a00*(x[3+j]-u[j])+a01*x[10+j]
        an=a10*(x[3+j]-u[j])+a11*x[10+j]
        pos[j]=x[j]+u[j]*h-(an-x[10+j]+2*decay*(vn-x[3+j]))/(wn*wn)
        vel[j]=vn;acc.append(an)
    tau=c.response[4];alpha=ca.exp(-h/tau)
    pos[2]=x[2]+u[2]*h+(x[5]-u[2])*tau*(1-alpha)
    vel[2]=u[2]+alpha*(x[5]-u[2])
    return ca.vertcat(*pos),ca.vertcat(*vel),ca.vertcat(*acc)


def _endpoint(x,u,ctx,c,s,ca):
    a=ca.vertcat(x[10],x[11],(u[2]-x[5])/c.response[4])
    jerk=ca.vertcat(*[c.response[2*j]**2*(u[j]-x[3+j])
                     -2*c.response[2*j+1]*c.response[2*j]*a[j] for j in range(2)],
                   -a[2]/c.response[4])
    yaw,yr=ctx[0],ctx[1]
    # Context flattening is row-major, as in the native ABI.
    E=ca.reshape(ctx[2:11],3,3).T;Ed=ca.reshape(ctx[11:20],3,3).T
    force=a+ca.vertcat(0,0,s.gravity_m_s2)
    norm=ca.sqrt(ca.dot(force,force)+s.rotation_norm_regularizer**2)
    z=force/norm;zd=(jerk-z*ca.dot(z,jerk))/norm
    xc=ca.vertcat(ca.cos(yaw),ca.sin(yaw),0)
    xcd=yr*ca.vertcat(-ca.sin(yaw),ca.cos(yaw),0)
    cross=ca.cross(z,xc);cn=ca.sqrt(ca.dot(cross,cross)+s.rotation_norm_regularizer**2)
    y=cross/cn;cd=ca.cross(zd,xc)+ca.cross(z,xcd);yd=(cd-y*ca.dot(y,cd))/cn
    T=ca.horzcat(ca.cross(y,z),y,z)
    Td=ca.horzcat(ca.cross(yd,z)+ca.cross(y,zd),yd,zd)
    R=T@E;Rd=Td@E+T@Ed
    g=c.geometry;a1=g[2]+x[6];a2=g[3]+x[6]+x[7]
    d1=ca.vertcat(ca.cos(a1),0,-ca.sin(a1));d2=ca.vertcat(ca.cos(a2),0,-ca.sin(a2))
    t1=ca.vertcat(-ca.sin(a1),0,-ca.cos(a1));t2=ca.vertcat(-ca.sin(a2),0,-ca.cos(a2))
    r=ca.DM(g[4:7]+g[7:10])+g[0]*d1+g[1]*d2
    J=ca.horzcat(g[0]*t1+g[1]*t2,g[1]*t2)
    return x[:3]+R@r,x[3:6]+R@J@x[8:10]+Rd@r,R,force[2],cn


def build_model(config, surrogate: SurrogateConfig | None=None) -> StageModel:
    import casadi as ca
    c=ModelConfig.from_dict(config);s=surrogate or SurrogateConfig();screen=validate_surrogate(c,s)
    L=parameter_layout(s.event_slots);x=ca.SX.sym("x",23);u=ca.SX.sym("u",5);p=ca.SX.sym("p",L['size'])
    par=lambda k:p[L[k]]
    h=par('h')[0];joint=x[6:10];active=x[14:16];samples=[joint]
    masks=[1.]
    for slot in range(s.event_slots):
        joint=_rk4(joint,active,par('durations')[slot],c,s,symbolic=ca)
        samples.append(joint);masks.append(par('sample_mask')[slot])
        fresh=ca.DM(c.kp)*(x[12:14]+par('send_relative')[slot]*u[3:5])+ca.DM(c.kd)*u[3:5]
        isnew=par('apply_new')[slot];ispending=par('apply_pending')[slot]
        active=(1-isnew-ispending)*active+isnew*fresh+ispending*x[16:18]
    pos,vel,acc=_base_step(x,u,h,c,ca)
    reference=x[12:14]+h*u[3:5]
    pending=par('pending_at_end')[0]*(ca.DM(c.kp)*(x[12:14]+par('pending_send_relative')[0]*u[3:5])+ca.DM(c.kd)*u[3:5])
    xn=ca.vertcat(pos,vel,joint[:2],joint[2:4],acc,reference,active,pending,u)
    tip,tipv,R,fz,cn=_endpoint(xn,u,par('attitude_context'),c,s,ca)
    terms=[tip-par('target_position'),tipv-par('target_velocity'),
           xn[:3]-par('nominal_position'),xn[6:8]-par('nominal_q'),
           (u-par('nominal_control'))/ca.DM(c.velocity_limit),
           (u-x[18:23])/(h*ca.DM(c.acceleration_limit))]
    residual=ca.vertcat(*[math.sqrt(c.weights[j])*r for j,r in enumerate(terms)])
    components=ca.vertcat(*[h*c.weights[j]*ca.dot(r,r) for j,r in enumerate(terms)])
    cost=ca.sum1(components)
    rows=[];lower=[];upper=[];clayout={}
    def add(name,value,lo=-1.,hi=1.):
        start=len(lower);count=int(value.numel());rows.append(value)
        lower.extend([lo]*count);upper.extend([hi]*count);clayout[name]=slice(start,start+count)
    qcenter=(c.q_lower+c.q_upper)/2;qhalf=(c.q_upper-c.q_lower)/2-s.q_padding_rad
    dqlim=c.velocity_limit[3:]-s.dq_padding_rad_s
    for k,(state,mask) in enumerate(zip(samples,masks)):
        add(f'joint_sample_{k}',mask*ca.vertcat((state[:2]-ca.DM(qcenter))/ca.DM(qhalf),state[2:4]/ca.DM(dqlim)))
    half=ca.DM((c.q_upper-c.q_lower)/2);center=ca.DM(qcenter)
    add('integrated_reference',(reference-center)/half)
    add('stopping_nodes',(xn[6:8]+par('stopping_reserve')*xn[8:10]-center)/half)
    add('command_slew',(u-x[18:23])/(h*ca.DM(c.acceleration_limit)))
    # Source-history stopping bounds are numerical parameters, not reset states.
    # acados/HPIPM requires finite arrays.  1e15 represents the absent side
    # of these one-sided domain conditions; it does not relax a hard limit.
    add('first_guard_lower',(u-par('control_lower'))/ca.DM(c.velocity_limit),0.,1e15)
    add('first_guard_upper',(par('control_upper')-u)/ca.DM(c.velocity_limit),0.,1e15)
    add('upright_force',ca.vertcat(fz),s.gravity_m_s2*.25,1e15)
    add('heading_domain',ca.vertcat(cn),.25,1e15)
    constraints=ca.vertcat(*rows)
    name='x650_smooth_upper_v1'
    fun={key:ca.Function(name+'_'+key,[x,u,p],[value]) for key,value in
         [('discrete',xn),('cost',cost),('residual',residual),('constraints',constraints),
          ('components',components),('endpoint',ca.vertcat(tip,tipv)),
          ('joint_samples',ca.horzcat(*samples)),('rotation',R)]}
    return StageModel(name,x,u,p,xn,cost,residual,constraints,np.array(lower),np.array(upper),
                      -c.velocity_limit.copy(),c.velocity_limit.copy(),ca.SX(0),ca.SX.zeros(0,1),
                      STATE_LAYOUT.copy(),L,c,s,fun,clayout,
                      dict(kind='consistent_smooth_discrete_ocp_v1',states=23,controls=5,
                           physical_config_sha256=c.sha256,surrogate=s.as_dict(),integration=screen,
                           friction='smooth_Stribeck_tanh_no_static_sticking',future_quantization='identity',
                           actual_history_quantization='preserved_recorded_values',
                           inactive_one_sided_upper_bound=1e15,
                           intersample_safety='smooth samples only; original event continuous-envelope final check required',
                           derivatives='CasADi AD of this exact discrete graph',
                           physical_fit_verified=False,hardware_qualified=False))


def _array(value,shape,label):
    a=np.asarray(value,dtype=float)
    if a.shape!=shape or not np.isfinite(a).all():raise ValueError('Invalid '+label)
    return a


def _rotation(value):
    R=_array(value,(9,),'rotation').reshape(3,3)
    if np.max(abs(R.T@R-np.eye(3)))>1e-6 or abs(np.linalg.det(R)-1)>1e-6:
        raise ValueError('Invalid FLU to ENU rotation')
    return R


def _skew(v):
    return np.array([[0,-v[2],v[1]],[v[2],0,-v[0]],[-v[1],v[0],0]])


def _rotvec(v):
    a=float(np.linalg.norm(v));K=_skew(v)
    if a<1e-8:return np.eye(3)+K+.5*K@K
    return np.eye(3)+math.sin(a)/a*K+(1-math.cos(a))/(a*a)*K@K


def _rotation_log(R):
    angle=math.acos(float(np.clip((np.trace(R)-1)/2,-1,1)))
    if angle<1e-8:return .5*np.array([R[2,1]-R[1,2],R[0,2]-R[2,0],R[1,0]-R[0,1]])
    if math.pi-angle<1e-5:raise ValueError('Unsupported near-pi attitude residual')
    return angle/(2*math.sin(angle))*np.array([R[2,1]-R[1,2],R[0,2]-R[2,0],R[1,0]-R[0,1]])


def _thrust_rotation(acceleration,yaw,gravity):
    f=acceleration+np.array([0,0,gravity]);norm=np.linalg.norm(f)
    if norm<gravity*.25 or f[2]<=gravity*.25:raise ValueError('Upright attitude domain')
    z=f/norm;y=np.cross(z,[math.cos(yaw),math.sin(yaw),0]);yn=np.linalg.norm(y)
    if yn<.25:raise ValueError('Heading singularity')
    y/=yn;return np.column_stack((np.cross(y,z),y,z))


def body_tip(q,c):
    g=c.geometry;a=g[2]+q[0];b=g[3]+q.sum()
    return g[4:7]+g[7:10]+g[0]*np.array([math.cos(a),0,-math.sin(a)])+g[1]*np.array([math.cos(b),0,-math.sin(b)])


def nominal(c,anchor,target):
    """Same reference-only branch selection and nominal split as native Core."""
    R=_rotation(anchor['rotation']);qa=_array(anchor['q'],(2,),'anchor joints')
    pa=_array(anchor['position'],(3,),'anchor position');tp=_array(target['position'],(3,),'target position')
    tv=_array(target['velocity'],(3,),'target velocity');g=c.geometry;tip0=body_tip(qa,c)
    delta=R.T@(tp-pa-R@tip0);delta[1]=0
    goal=tip0+c.arm_share*delta-g[4:7]-g[7:10]
    if abs(goal[1])>1e-8:raise ValueError('Nominal arm plane')
    cosine=(goal[0]**2+goal[2]**2-g[0]**2-g[1]**2)/(2*g[0]*g[1])
    if abs(cosine)>1+1e-10:raise ValueError('Nominal unreachable')
    candidates=[]
    for sign in (-1,1):
        beta=sign*math.acos(float(np.clip(cosine,-1,1)))
        theta=math.atan2(-goal[2],goal[0])-math.atan2(g[1]*math.sin(beta),g[0]+g[1]*math.cos(beta))
        q=np.array([theta-g[2],beta+g[2]-g[3]])
        q=qa+(q-qa+math.pi)%(2*math.pi)-math.pi;candidates.append(q)
    q=min(candidates,key=lambda z:np.linalg.norm(z-qa))
    if np.any(q<c.physical_lower) or np.any(q>c.physical_upper):raise ValueError('Nominal physical joint bounds')
    a=g[2]+q[0];b=g[3]+q.sum();j2=g[1]*np.array([-math.sin(b),0,-math.cos(b)])
    J=np.column_stack((g[0]*np.array([-math.sin(a),0,-math.cos(a)])+j2,j2));Jxz=J[[0,2],:]
    if abs(np.linalg.det(Jxz))<1e-14:raise ValueError('Nominal IK singular')
    dqn=np.linalg.solve(Jxz,(c.arm_share*R.T@tv)[[0,2]])
    return tp-R@body_tip(q,c),q,np.r_[tv-R@J@dqn,dqn]


def _reference(in_,c):
    ctx=in_['context'];t=float(in_['t']);at=float(ctx['reference_time'])
    r=_array(ctx['reference'],(2,),'MIT reference').copy();commands=ctx['commands']
    if not commands or len(commands)>64 or at>t+1e-10 or commands[0]['time']>at+1e-10:
        raise ValueError('Actual command reference coverage')
    last=-math.inf
    for item in commands:
        time=float(item['time']);v=_array(item['velocity'],(2,),'actual command')
        if not math.isfinite(time) or time<=last or time>t+1e-10 or np.any(abs(v)>c.velocity_limit[3:]+1e-10):
            raise ValueError('Invalid actual command history')
        last=time
    active=np.array(commands[0]['velocity'],dtype=float)
    for item in commands:
        time=float(item['time']);v=np.array(item['velocity'],dtype=float)
        if time<=at+1e-10:active=v;continue
        if time>t:break
        r+=(time-at)*active;at=time;active=v
    return r+(t-at)*active


def _packets(in_,c):
    t=float(in_['t']);packets=in_['context']['packets'];result=[];last=-math.inf
    if not packets or len(packets)>64:raise ValueError('Actual packet history required')
    for p in packets:
        send=float(p['send_time']);q=_array(p['q_ref'],(2,),'packet q');v=_array(p['dq_ref'],(2,),'packet dq')
        kp=_array(p['kp'],(2,),'packet kp');kd=_array(p['kd'],(2,),'packet kd');tau=_array(p['tau_ff'],(2,),'packet feedforward')
        if (not math.isfinite(send) or send<=last or send>t+1e-10
                or not np.array_equal(kp,c.kp) or not np.array_equal(kd,c.kd)):
            raise ValueError('Actual packet identity/order/gains')
        if result and abs((send-result[0][0])/c.bus_period-round((send-result[0][0])/c.bus_period))>1e-7:
            raise ValueError('Actual packet bus grid changed')
        result.append((send,send+c.motion[4],kp*q+kd*v+tau));last=send
    return result


def _aligned_joint(in_,c,s,packets):
    state=in_['state'];t=float(in_['t']);age=float(state['control_time'])-float(state['joint_stamp'])
    if not math.isfinite(age) or age<0 or age>.025+1e-10:raise ValueError('Original joint source age')
    start=t-age;eligible=[p for p in packets if p[1]<=start+1e-10]
    if not eligible:raise ValueError('Actual delayed packet history does not cover joint measurement')
    joint=np.r_[_array(state['q'],(2,),'measured q'),_array(state['dq'],(2,),'measured dq')]
    active=eligible[-1][2].copy();cursor=start
    for packet in [p for p in packets if p[1]>start+1e-10 and p[1]<=t+1e-10]:
        duration=packet[1]-cursor
        # Historical gap may cover several bus periods; split without inventing packets.
        parts=max(1,math.ceil(duration/c.bus_period))
        for _ in range(parts):joint=_rk4(joint,active,duration/parts,c,s,substeps=s.history_rk4_substeps_per_bus)
        cursor=packet[1];active=packet[2].copy()
    duration=max(0.,t-cursor);parts=max(1,math.ceil(duration/c.bus_period))
    for _ in range(parts):joint=_rk4(joint,active,duration/parts,c,s,substeps=s.history_rk4_substeps_per_bus)
    pending=[p for p in packets if p[1]>t+1e-10]
    if len(pending)>1:raise ValueError('More than one already-sent pending packet')
    return joint,active,pending,age


def _schedule(start,h,first_future,c,pending_time,slots):
    end=start+h;period=c.bus_period;delay=c.motion[4]
    # Half-open sending intervals: a send exactly at the next node belongs to
    # its next control. Application at the node belongs to this transition.
    first=max(0,int(math.ceil((start-first_future)/period-1e-9)))
    sends=[];index=first
    while first_future+index*period<end-1e-10:
        send=first_future+index*period
        if send>=start-1e-10:sends.append(send)
        index+=1
    events=[]
    if pending_time is not None:
        if pending_time<start-1e-9 or pending_time>end+1e-9:raise ValueError('Pending packet outside next stage')
        events.append((max(start,pending_time),0.,1.,0.))
    for send in sends:
        if send+delay<=end+1e-10:events.append((min(end,send+delay),1.,0.,max(0.,send-start)))
    events.sort(key=lambda x:x[0]);dur=[];new=[];pending=[];rel=[];mask=[];cursor=start
    for at,n,b,r in events:
        if at<cursor-1e-10:raise ValueError('Fixed event schedule order')
        dur.append(max(0.,at-cursor));new.append(n);pending.append(b);rel.append(r);mask.append(1.);cursor=at
    if end-cursor>1e-10 or not dur:
        dur.append(max(0.,end-cursor));new.append(0.);pending.append(0.);rel.append(0.);mask.append(1.)
    if len(dur)>slots or max(dur)>period+1e-9:raise ValueError('Fixed event schedule capacity or integration span')
    next_pending=[send for send in sends if send+delay>end+1e-10]
    if len(next_pending)>1:raise ValueError('Multiple future pending packets')
    count=len(dur)
    for array in (dur,new,pending,rel,mask):array.extend([0.]*(slots-count))
    return dict(durations=dur,apply_new=new,apply_pending=pending,send_relative=rel,sample_mask=mask,
                pending_at_end=[float(bool(next_pending))],
                pending_send_relative=[next_pending[0]-start if next_pending else 0.]),(next_pending[0]+delay if next_pending else None)


def _first_bounds(in_,memory,c,current_q,current_dq,age):
    dt=float(in_['dt']);previous=_array(memory['previous_command'],(5,),'previous executed control')
    measured=_array(in_['state']['dq'],(2,),'measured dq');slew=c.acceleration_limit[3:]*dt
    vmin=np.minimum.reduce([measured,current_dq,previous[3:]-slew]);vmax=np.maximum.reduce([measured,current_dq,previous[3:]+slew])
    recent=[a for a in in_['context']['commands'] if a['time']>=in_['t']-c.command_hold_s-age-c.max_dt]
    for item in recent:
        vmin=np.minimum(vmin,item['velocity']);vmax=np.maximum(vmax,item['velocity'])
    qgl=current_q+(c.command_hold_s+age)*np.minimum(vmin,0)
    qgh=current_q+(c.command_hold_s+age)*np.maximum(vmax,0)
    if np.any(qgl<c.q_lower-1e-8) or np.any(qgh>c.q_upper+1e-8):
        raise ValueError('Original pending history stopping envelope infeasible')
    lo=np.maximum(-c.velocity_limit,previous-c.acceleration_limit*dt)
    hi=np.minimum(c.velocity_limit,previous+c.acceleration_limit*dt)
    a=c.acceleration_limit[3:];reserve=a*c.max_dt
    lo[3:]=np.maximum(lo[3:],-(np.sqrt(reserve**2+2*a*np.maximum(qgl-c.q_lower,0))-reserve))
    hi[3:]=np.minimum(hi[3:],np.sqrt(reserve**2+2*a*np.maximum(c.q_upper-qgh,0))-reserve)
    if np.any(lo>hi+1e-10):raise ValueError('Original first command history bounds infeasible')
    return lo,hi


def build_case(config,fixture_case,surrogate: SurrogateConfig | None=None) -> NumericalCase:
    """Restore original feedback, committed memory, actual packet history.

    Accepts a native {memory,input,...} case or a selected-fixture {case,...}
    envelope. No existing oracle output is used as the initial solution.
    """
    c=ModelConfig.from_dict(config);s=surrogate or SurrogateConfig();screen=validate_surrogate(c,s)
    wrapper=fixture_case;case=fixture_case.get('case',fixture_case);in_=case['input'];memory=case['memory']
    if not all(memory.get(k) is True for k in ('initialized','history_primed','has_origin')):
        raise ValueError('Primed original committed memory required')
    t=float(in_['t']);dt=float(in_['dt']);state=in_['state']
    if (not math.isfinite(t) or t<0 or not math.isfinite(dt) or not 0<dt<=c.max_dt+1e-10
            or abs(float(state['control_time'])-t-float(memory['origin']))>1e-8
            or (memory.get('has_last_t') and t<=float(memory['last_t']))):
        raise ValueError('Original control time/dt/epoch inconsistent')
    packets=_packets(in_,c);joint,active,pending,age=_aligned_joint(in_,c,s,packets)
    r=_reference(in_,c);previous=_array(memory['previous_command'],(5,),'previous executed control')
    x0=np.r_[_array(state['position'],(3,),'position'),_array(state['velocity'],(3,),'velocity'),
             joint[:2],joint[2:],_array(state['acceleration'],(3,),'acceleration')[:2],
             r,active,pending[0][2] if pending else np.zeros(2),previous]
    h=np.r_[dt,np.full(c.nodes-1,c.node_dt)];nodes=np.cumsum(h)
    targets=in_.get('preview',[])
    if targets and len(targets)!=c.nodes:raise ValueError('Original preview dimension')
    if not targets:
        target=in_['target'];tp=np.array(target['position']);tv=np.array(target['velocity']);ta=np.array(target['acceleration'])
        targets=[dict(position=(tp+offset*tv+.5*offset**2*ta).tolist(),velocity=(tv+offset*ta).tolist(),acceleration=ta.tolist()) for offset in nodes]
    R=_rotation(state['rotation']);yaw=math.atan2(R[1,0],R[0,0]);roll=math.atan2(R[2,1],R[2,2]);pitch=math.asin(float(np.clip(-R[2,0],-1,1)))
    omega=_array(state['omega_body'],(3,),'omega')
    if abs(math.cos(pitch))<1e-6:raise ValueError('Yaw rate singularity')
    yr=(math.sin(roll)*omega[1]+math.cos(roll)*omega[2])/math.cos(pitch)
    residual=_rotation_log(_thrust_rotation(np.array(state['acceleration']),yaw,s.gravity_m_s2).T@R)
    origin=float(in_['context']['bus_origin'])
    if not math.isfinite(origin):raise ValueError('Original bus origin nonfinite')
    first_future=origin+math.ceil((t-origin)/c.bus_period-1e-9)*c.bus_period
    if first_future<=packets[-1][0]+1e-10:first_future+=c.bus_period
    if first_future<t-1e-10:raise ValueError('Future packet before current time')
    ptime=pending[0][1] if pending else None;L=parameter_layout(s.event_slots);parameters=np.zeros((c.nodes+1,L['size']))
    firstlo,firsthi=_first_bounds(in_,memory,c,joint[:2],joint[2:],age)
    elapsed=0.;schedule_counts=[]
    for i in range(c.nodes):
        p=parameters[i];assign=lambda key,value:p.__setitem__(L[key],value)
        schedule,ptime=_schedule(t+elapsed,float(h[i]),first_future,c,ptime,s.event_slots)
        elapsed+=h[i];decay=math.exp(-elapsed/s.attitude_decay_s)
        E=_rotvec(decay*residual);Ed=E@_skew(-decay*residual/s.attitude_decay_s)
        assign('h',[h[i]]);assign('attitude_context',np.r_[yaw+s.attitude_decay_s*(1-decay)*yr,decay*yr,E.ravel(),Ed.ravel()])
        assign('target_position',targets[i]['position']);assign('target_velocity',targets[i]['velocity'])
        pn,qn,un=nominal(c,memory['anchor'],targets[i]);assign('nominal_position',pn);assign('nominal_q',qn);assign('nominal_control',un)
        for key,value in schedule.items():assign(key,value)
        assign('stopping_reserve',c.velocity_limit[3:]/(2*c.acceleration_limit[3:])+c.max_dt+c.command_hold_s+age)
        assign('control_lower',firstlo if i==0 else -c.velocity_limit);assign('control_upper',firsthi if i==0 else c.velocity_limit)
        schedule_counts.append(int(sum(schedule['sample_mask'])))
    parameters[-1]=parameters[-2]
    plan=np.tile(previous,(c.nodes,1))
    if memory.get('plan'):
        old=_array(memory['plan'],(c.nodes*5,),'original committed plan').reshape(c.nodes,5)
        oldh=_array(memory['plan_h'],(c.nodes,),'original plan steps')
        if np.any(oldh<=0):raise ValueError('Original plan horizon')
        ends=np.cumsum(oldh);offset=0.
        for i in range(c.nodes):
            at=t-float(memory['plan_t'])+offset;k=min(int(np.searchsorted(ends,at,side='right')),c.nodes-1)
            plan[i]=old[k];offset+=h[i]
    if not np.isfinite(x0).all() or not np.isfinite(parameters).all() or not np.isfinite(plan).all():
        raise ValueError('Nonfinite restored smooth OCP')
    metadata=dict(kind='restored_smooth_ocp_case_v1',case_id=case.get('id'),
                  source=wrapper.get('source'),source_sha256=wrapper.get('source_sha256'),source_index=wrapper.get('source_index'),
                  input_sha256=canonical_sha256(in_),memory_sha256=canonical_sha256(memory),
                  physical_config_sha256=c.sha256,surrogate=s.as_dict(),integration=screen,
                  original_task_time_s=t,original_dt_s=dt,original_joint_age_s=age,
                  measured_joint_state=np.r_[state['q'],state['dq']].tolist(),
                  smooth_aligned_joint_state=joint.tolist(),
                  initial_state_alignment='same smooth ODE with immutable actual sent packets; not a new measurement',
                  first_future_send_task_s=first_future,initial_pending_packet_count=len(pending),
                  actual_packet_history_sha256=canonical_sha256(in_['context']['packets']),
                  actual_packet_history_count=len(packets),schedule_active_slots=schedule_counts,
                  future_quantization='identity surrogate; actual packet values preserved',
                  initial_plan_source='shifted committed plan or previous executed control; oracle unused',
                  event_validation_required=True,continuous_safety_proven=False,hardware_qualified=False)
    return NumericalCase(x0,parameters,plan,h,metadata)


def rollout(model: StageModel,case: NumericalCase,plan=None):
    """Numeric evaluation of the same graph used by both NLP solvers."""
    U=np.asarray(case.initial_plan if plan is None else plan,dtype=float)
    if U.shape!=(model.config.nodes,5) or not np.isfinite(U).all():raise ValueError('Physical plan shape')
    states=[case.x0.copy()];cost=0.;components=np.zeros(6);constraints=[];endpoints=[]
    for i,u in enumerate(U):
        x=states[-1];p=case.parameters[i]
        constraints.append(np.asarray(model.functions['constraints'](x,u,p)).ravel())
        components+=np.asarray(model.functions['components'](x,u,p)).ravel()
        cost+=float(model.functions['cost'](x,u,p));endpoints.append(np.asarray(model.functions['endpoint'](x,u,p)).ravel())
        states.append(np.asarray(model.functions['discrete'](x,u,p)).ravel())
    return dict(states=np.array(states),cost=cost,components=components,
                constraints=np.array(constraints),endpoints=np.array(endpoints))
