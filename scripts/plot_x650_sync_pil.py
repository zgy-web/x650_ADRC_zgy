#!/usr/bin/env python3
"""Plot independent same-time native endpoint data, keeping clock scopes separate."""
import argparse
import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


def load(folder,prefix):
    report=json.loads((folder/(prefix+'_report.json')).read_bytes())
    if not report['passed'] or report['wall_clock_qualified']:
        raise ValueError('Only audited synchronous physical data expected')
    position=np.genfromtxt(folder/(prefix+'_endpoint_series.csv'),delimiter=',',names=True)
    timing=np.genfromtxt(folder/(prefix+'_timing_series.csv'),delimiter=',',names=True)
    t=position['task_s'];error=position['error_3d_m']
    measured=float(np.sqrt(np.trapz(error**2,t)/(t[-1]-t[0])))
    if abs(measured-report['endpoint']['position_rmse_m'])>1e-12:
        raise ValueError('Plot samples do not reproduce the audited time-weighted metric')
    return report,position,timing


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--input',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    a=p.parse_args();a.output.mkdir(parents=True,exist_ok=True)
    hold,hp,ht=load(a.input,'hold_audit_v2');normal,np_,nt=load(a.input,'normal_audit_v1')
    plt.rcParams.update({'font.size':11,'axes.spines.top':False,'axes.spines.right':False})
    fig,axes=plt.subplots(2,2,figsize=(13.2,8.2),layout='constrained')
    fig.suptitle('Orin NX in the physical simulation loop\nSynchronous simulation time; not a wall-clock real-time qualification',fontsize=17)
    ax=axes[0,0];cx=np_['target_x_m'][0];cz=np_['target_z_m'][0]
    ax.plot(100*(np_['target_x_m']-cx),100*(np_['target_z_m']-cz),color='#e6a12a',lw=3,label='Target')
    ax.plot(100*(np_['actual_x_m']-cx),100*(np_['actual_z_m']-cz),color='#2473a6',lw=1.2,label='Actual native tip')
    ax.set(xlabel='World X from fixed target centre [cm]',ylabel='World Z from fixed target centre [cm]',title='Normal figure-eight: XZ projection')
    ax.set_aspect('equal');ax.legend(loc='upper right');ax.grid(alpha=.2)
    ax=axes[0,1]
    ax.plot(hp['task_s'],1000*hp['error_3d_m'],label=f"Hold: RMSE {1000*hold['endpoint']['position_rmse_m']:.2f} mm",color='#2473a6')
    ax.plot(np_['task_s'],1000*np_['error_3d_m'],label=f"Normal: full-window RMSE {1000*normal['endpoint']['position_rmse_m']:.2f} mm",color='#d87e24')
    ax.axvspan(2,28,color='#d87e24',alpha=.06)
    ax.set(xlabel='Simulation task time [s]',ylabel='Same-time 3D endpoint error [mm]',title='No phase or translation fitting',ylim=(0,None));ax.legend(fontsize=9);ax.grid(alpha=.2)
    ax=axes[1,0]
    for label,series,color in [('Hold',ht,'#2473a6'),('Normal',nt,'#d87e24')]:ax.plot(series['task_s'],series['nx_core_ms'],label=label,color=color,alpha=.8,lw=.8)
    ax.axhline(40,color='#596773',ls='--',lw=1,label='40 ms simulation solve period')
    ax.set(xlabel='Simulation task time [s]',ylabel='NX Core wall duration [ms]',title='Measured inside NX: matrix + QP + candidate checks',ylim=(0,45));ax.legend(fontsize=9);ax.grid(alpha=.2)
    ax=axes[1,1]
    for label,series,color in [('Hold',ht,'#2473a6'),('Normal',nt,'#d87e24')]:ax.plot(series['task_s'],series['server_roundtrip_ms'],label=label,color=color,alpha=.7,lw=.8)
    ax.set(xlabel='Simulation task time [s]',ylabel='Server-clock roundtrip [ms]',title='Physical time is frozen during these waits',ylim=(0,None));ax.legend();ax.grid(alpha=.2)
    fig.savefig(a.output/'nx_sync_pil.png',dpi=160);fig.savefig(a.output/'nx_sync_pil.pdf')
    plt.close(fig)
    print(json.dumps({'hold_rmse_mm':1000*hold['endpoint']['position_rmse_m'],
        'normal_full_rmse_mm':1000*normal['endpoint']['position_rmse_m'],
        'normal_tracking_rmse_mm':1000*normal['tracking']['position_rmse_m'],'metric_roundtrip_verified':True}))


if __name__=='__main__':main()
