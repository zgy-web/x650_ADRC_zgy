"""PX4 takeoff -> Offboard velocity hold + arm motion -> confirmed landing."""
import csv
import json
import hashlib
import math
import os
from pathlib import Path
import subprocess
import shutil
import sys
import time
import numpy as np
import yaml
from algorithm import State, Target, PositionBaseline

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'simulations/x650_hover_v1'))
from run_case import FlightTelemetry, euler, summarize
from parameter_evidence import ParameterEvidence
from amsim import ScenarioContext
from amsim.modeling.manager import resolve, validate_px4_airframe
from geometry_msgs.msg import Vector3
from pymavlink import mavutil

JOINTS=['upper_arm_joint','fore_crank_joint']


def enu(v):
    return np.array([v[1],v[0],-v[2]],dtype=float)


def main():
    cfg=yaml.safe_load(Path(os.environ['AMSIM_SCENARIO_CONFIG']).read_text())
    folder=Path(os.environ['AMSIM_RUN_DIR'])
    model=resolve(cfg['model'])
    for source in ('algorithm.py',):
        shutil.copy2(Path(__file__).with_name(source),folder/source)
    manifest=json.loads(model.manifest_path.read_text())
    if hashlib.sha256(Path(manifest['world_installed']).read_bytes()).hexdigest()!=manifest['artifacts']['world.sdf']:
        raise RuntimeError('Installed flight world differs from model manifest')
    result={'success':False,'model':model.model_name,'model_sha256':model.sha256,
            'control':'External position P -> PX4 velocity/attitude/rate loops; force-servo closed-chain arm',
            'acceptance':{'position_rmse_m':.25,'position_max_m':.5,'tilt_max_deg':15.}}
    rows=[];controls=[];events=[];telemetry=None;params=None;attempted=False
    closure=[None,0.]
    def receive(msg):closure[:]=[msg,time.monotonic()]
    bridge_log=(folder/'closure_bridge.log').open('w')
    bridge=subprocess.Popen(['ros2','run','ros_gz_bridge','parameter_bridge',
        '/x650_cad_arm/closure_metrics@geometry_msgs/msg/Vector3[gz.msgs.Vector3d'],stdout=bridge_log,stderr=subprocess.STDOUT)
    try:
        with ScenarioContext() as ctx:
            original_pump=ctx.vehicle._pump
            def event(name):
                events.append({'event':name,'sim_time_s':ctx.node.sim_time_s()});print(name,flush=True)
            try:
                if ctx.model['model_sha256']!=model.sha256 or ctx.arm.joints!=JOINTS:
                    raise RuntimeError('Active model does not match the CAD arm')
                ctx.node.create_subscription(Vector3,'/x650_cad_arm/closure_metrics',receive,10)
                ctx.vehicle.wait_ready()
                ctx.vehicle.wait(lambda:closure[0] is not None,15,'closure telemetry')
                airframe=validate_px4_airframe(model)
                params=ParameterEvidence(folder,airframe,model.sdf_path.parent/airframe.name,model.px4_sys_autostart,model.sha256)
                params.capture(ctx.vehicle,'before',enforce_contract=True)
                initial=ctx.observe(ground_truth=True)
                if initial.vehicle.armed or not initial.vehicle.landed:raise RuntimeError('Fresh grounded state required')
                height=cfg['vehicle']['takeoff_height_m'];gt=initial.ground_truth
                truth_target=(gt.x,gt.y,gt.z+height)
                yaw=euler(gt.qx,gt.qy,gt.qz,gt.qw)[2]
                telemetry=FlightTelemetry(folder,20,model.sha256,truth_target,yaw,rows);telemetry.attach(ctx)
                estimate=ctx.node.data['position']
                target_pos=enu([estimate.x,estimate.y,estimate.z-height])
                resets=(estimate.xy_reset_counter,estimate.z_reset_counter)
                telemetry.phase='takeoff';attempted=True;event('takeoff_begin')
                def guarded_takeoff_pump(ros_timeout_s=.02):
                    original_pump(ros_timeout_s)
                    g=ctx.node.observation(include_ground_truth=True).ground_truth
                    angles=euler(g.qx,g.qy,g.qz,g.qw)
                    if max(abs(angles[0]),abs(angles[1]))>math.radians(35) or g.z>truth_target[2]+1.5:
                        raise RuntimeError('Takeoff envelope exceeded')
                ctx.vehicle._pump=guarded_takeoff_pump
                ctx.vehicle.takeoff(height,stable_s=2.5)
                ctx.vehicle._pump=original_pump
                event('stable_hover');ctx.wait(2.)
                controller=PositionBaseline();yaw_ned=float(ctx.node.data['position'].heading)
                start=[None];last=[None];last_send=[time.monotonic()]
                def pump(ros_timeout_s=.02):
                    original_pump(ros_timeout_s)
                    sim=ctx.node.sim_time_s()
                    dt=.02 if last[0] is None else sim-last[0]
                    if dt<.019:return
                    if time.monotonic()-last_send[0]>.5:raise RuntimeError('Offboard wall-clock sending gap exceeded .5s')
                    last[0]=sim;last_send[0]=time.monotonic()
                    t=-1 if start[0] is None else sim-start[0]
                    est=ctx.node.data['position'];js=ctx.arm.state()
                    if (est.xy_reset_counter,est.z_reset_counter)!=resets:raise RuntimeError('Estimator reset')
                    if not (est.xy_valid and est.z_valid and est.v_xy_valid and est.v_z_valid):raise RuntimeError('Invalid estimate')
                    for key in ('position','truth','joints'):
                        if key in ctx.node.received and time.monotonic()-ctx.node.received[key]>1.:raise RuntimeError('Stale '+key)
                    if bridge.poll() is not None or time.monotonic()-closure[1]>1.:raise RuntimeError('Closure telemetry stale')
                    if not math.isfinite(closure[0].x) or closure[0].x>.0005 or closure[0].y>.005:raise RuntimeError('Closed-chain error exceeded limit')
                    target_q=np.array([0.,0.]) if t<5 else (np.array([.15,.10]) if t<13 else np.array([0.,0.]))
                    state=State(enu([est.x,est.y,est.z]),enu([est.vx,est.vy,est.vz]),
                        np.array([js[j].position_rad for j in JOINTS]),np.array([js[j].velocity_rad_s for j in JOINTS]),sim)
                    cmd=controller.step(t,dt,state,Target(target_pos,target_q))
                    v=np.asarray(cmd.velocity,dtype=float);qv=np.asarray(cmd.joint_velocity,dtype=float)
                    if v.shape!=(3,) or qv.shape!=(2,) or not np.all(np.isfinite(np.r_[v,qv])):raise RuntimeError('Invalid algorithm output')
                    v=np.clip(v,-.4,.4);qv=np.clip(qv,-.15,.15)
                    vn=enu(v)  # This ENU/NED swap-and-negate mapping is its own inverse.
                    mask=1|2|4|64|128|256|2048
                    ctx.vehicle.link.mav.set_position_target_local_ned_send(int(sim*1000)&0xffffffff,*ctx.vehicle.target,
                        mavutil.mavlink.MAV_FRAME_LOCAL_NED,mask,0,0,0,*vn,0,0,0,yaw_ned,0)
                    for j,speed in zip(JOINTS,qv):ctx.arm.set_velocity(j,float(speed))
                    obs=ctx.node.observation(include_ground_truth=True)
                    if obs.vehicle.failsafe or not obs.vehicle.armed:raise RuntimeError('Unexpected flight state')
                    if t>=0 and ctx.node.data['status'].nav_state!=14:raise RuntimeError('Lost Offboard mode')
                    g=obs.ground_truth;angles=euler(g.qx,g.qy,g.qz,g.qw)
                    if max(abs(angles[0]),abs(angles[1]))>math.radians(40) or np.linalg.norm(np.array([g.x,g.y,g.z])-truth_target)>2:raise RuntimeError('Trial envelope exceeded')
                    telemetry.check()
                    controls.append(dict(t=t,sim_time_s=sim,dt=dt,x=state.position[0],y=state.position[1],z=state.position[2],
                        vx_cmd=v[0],vy_cmd=v[1],vz_cmd=v[2],q0=state.q[0],q1=state.q[1],q0_target=target_q[0],q1_target=target_q[1],
                        dq0_cmd=qv[0],dq1_cmd=qv[1],closure_m=closure[0].x,axis_rad=closure[0].y))
                ctx.vehicle._pump=pump
                pre=time.monotonic()
                while time.monotonic()-pre<1.25:pump()
                ctx.vehicle._command(mavutil.mavlink.MAV_CMD_DO_SET_MODE,[1,6,0])
                ctx.vehicle.wait(lambda:ctx.node.data['status'].nav_state==14,10,'Offboard mode')
                start[0]=ctx.node.sim_time_s();telemetry.phase='measure';event('offboard_arm_trial_begin')
                ctx.vehicle.wait(lambda:ctx.node.sim_time_s()-start[0]>=21.,100,'CAD arm control trial')
                ctx.vehicle._pump=original_pump;ctx.arm.stop()
                telemetry.phase='landing';event('landing_begin');ctx.vehicle.land()
                result['landing_confirmed']=True;event('landed_and_disarmed')
                telemetry.phase='landed';ctx.wait(.5);telemetry.detach()
                params.capture(ctx.vehicle,'after',enforce_contract=False)
                result['metrics']=summarize(rows,result['acceptance'])
                measured=[r for r in controls if r['t']>=0]
                settled=[r for r in measured if 11<=r['t']<=12.5]
                result['arm_tracking_error_rad']=[max(abs(r[f'q{i}']-r[f'q{i}_target']) for r in settled) for i in range(2)]
                result['arm_motion_rad']=[max(r[f'q{i}'] for r in measured)-min(r[f'q{i}'] for r in measured) for i in range(2)]
                result['max_closure_m']=max(r['closure_m'] for r in measured)
                result['max_control_dt_s']=max(r['dt'] for r in measured)
                result['success']=bool(result['metrics']['accepted'] and max(result['arm_tracking_error_rad'])<.03 and min(result['arm_motion_rad'])>.07)
            except (Exception,KeyboardInterrupt) as exc:
                result['error']=str(exc);print('Trial error: '+str(exc),flush=True)
                ctx.vehicle._pump=original_pump
                try:ctx.arm.stop()
                except Exception as stop_error:result['arm_stop_error']=str(stop_error)
                if attempted:
                    try:
                        if telemetry:telemetry.phase='recovery_landing'
                        ctx.vehicle.land();result['recovery_landed']=True
                    except Exception as recovery:result['recovery_error']=str(recovery)
            finally:
                ctx.vehicle._pump=original_pump
                if telemetry:telemetry.detach()
    except (Exception,KeyboardInterrupt) as exc:result['error']=str(exc)
    finally:
        bridge.terminate()
        try:bridge.wait(timeout=5)
        except subprocess.TimeoutExpired:bridge.kill();bridge.wait()
        bridge_log.close()
        if telemetry:result['telemetry']=telemetry.close()
        if params:result['parameters']=params.reports
        if controls:
            with (folder/'control_trace.csv').open('w') as f:
                writer=csv.DictWriter(f,fieldnames=list(controls[0]));writer.writeheader();writer.writerows(controls)
        result['events']=events
        (folder/'result.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result,indent=2),flush=True)
    return 0 if result['success'] else 1


if __name__=='__main__':raise SystemExit(main())
