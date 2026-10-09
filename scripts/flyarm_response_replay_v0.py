#!/usr/bin/env python3
"""Flyarm stage-0: replay PX4 SITL 6-DoF as commanded pose of an IDEALIZED surrogate.
This is not a Franka dynamic simulation or a claim of FR3 feasibility.
"""
import argparse,json,math,pathlib
import numpy as np
from pyulog import ULog

def euler(q):
    w,x,y,z=q
    return np.array([math.atan2(2*(w*x+y*z),1-2*(x*x+y*y)),
        math.asin(max(-1,min(1,2*(w*y-z*x)))),
        math.atan2(2*(w*z+x*y),1-2*(y*y+z*z))])

def main():
    ap=argparse.ArgumentParser()
    ap.add_argument('--ulog',required=True)
    ap.add_argument('--out',required=True)
    ap.add_argument('--duration',type=float,default=12)
    ap.add_argument('--tau',type=float,default=.04,help='hypothetical 1st-order lag in seconds')
    ap.add_argument('--linear-speed',type=float,default=.8)
    ap.add_argument('--angular-speed',type=float,default=1.5)
    a=ap.parse_args()
    u=ULog(a.ulog,message_name_filter_list=['vehicle_local_position','vehicle_attitude'])
    d={v.name:v.data for v in u.data_list}
    p=d['vehicle_local_position'];q=d['vehicle_attitude']
    t=np.asarray(p['timestamp'],dtype=np.float64)*1e-6
    tq=np.asarray(q['timestamp'],dtype=np.float64)*1e-6
    t0=max(t[0],tq[0]); t1=min(t[-1],tq[-1],t0+a.duration)
    grid=np.arange(t0,t1,.01)
    xyz=np.stack([np.interp(grid,t,p[k]) for k in ('x','y','z')],axis=1)
    qq=np.stack([np.interp(grid,tq,q[f'q[{i}]']) for i in range(4)],axis=1)
    qq/=np.maximum(np.linalg.norm(qq,axis=1,keepdims=True),1e-9)
    angles=np.unwrap(np.array([euler(x) for x in qq]),axis=0)
    target=np.concatenate((xyz-xyz[0],angles-angles[0]),axis=1)
    imitation=np.zeros_like(target)
    linear_saturated=angular_saturated=0
    dt=.01
    for i in range(1,len(grid)):
        requested=(target[i]-imitation[i-1])*min(dt/a.tau,1)
        lin=requested[:3];ang=requested[3:]
        lin_norm=np.linalg.norm(lin); ang_norm=np.linalg.norm(ang)
        if lin_norm>a.linear_speed*dt:
            lin*=a.linear_speed*dt/lin_norm; linear_saturated+=1
        if ang_norm>a.angular_speed*dt:
            ang*=a.angular_speed*dt/ang_norm; angular_saturated+=1
        imitation[i]=imitation[i-1]+requested
    err=imitation-target
    rms=lambda x:float(np.sqrt(np.mean(x*x)))
    summary={'type':'idealized_6dof_kinematic_surrogate_not_franka','ulog':a.ulog,
        'duration_s':float(grid[-1]-grid[0]),'samples':len(grid),'sample_dt_s':dt,
        'assumed_tau_s':a.tau,'assumed_linear_max_m_s':a.linear_speed,'assumed_angular_max_rad_s':a.angular_speed,
        'rms_position_m':rms(np.linalg.norm(err[:,:3],axis=1)),
        'rms_orientation_rad':rms(np.linalg.norm(err[:,3:],axis=1)),
        'max_position_m':float(np.max(np.linalg.norm(err[:,:3],axis=1))),
        'max_orientation_rad':float(np.max(np.linalg.norm(err[:,3:],axis=1))),
        'linear_saturation_steps':linear_saturated,'angular_saturation_steps':angular_saturated,
        'warning':'No Franka kinematics, workspace, joint limits, force/torque dynamics, or control loop simulated. Absolute pose anchored at first sample.'}
    out=pathlib.Path(a.out);out.parent.mkdir(parents=True,exist_ok=True)
    out.write_text(json.dumps(summary,indent=2))
    np.savetxt(out.with_suffix('.csv'),np.c_[grid-grid[0],target,imitation],delimiter=',',comments='',header='t_s,target_x,target_y,target_z,target_roll,target_pitch,target_yaw,sim_x,sim_y,sim_z,sim_roll,sim_pitch,sim_yaw')
    print(json.dumps(summary,indent=2))
if __name__=='__main__':main()
