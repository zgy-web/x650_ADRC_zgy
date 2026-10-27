#!/usr/bin/env python3
"""Plot complete audited 25 Hz robustness records, preserving failed outcomes."""
import argparse
import csv
import json
from pathlib import Path
import sys

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import plot_x650_recovery_disturbance as native
from plot_x650_cpp_robustness import identity, unchanged, read, require, _plt
from plot_x650_isolated_validation import checked_input

KINDS = {'x650_cpp_multirate_robustness_report_v1', 'x650_cpp_multirate_wrench_followup_report_v1'}
LABELS = {'mass085_fast': 'Mass and inertia x0.85', 'mass115_fast': 'Mass and inertia x1.15',
          'wrench_fast': 'Three external wrench pulses'}


def load(path):
    inputs = {'report': identity(path)}
    report = read(path)
    require(report.get('kind') in KINDS and report.get('complete') is True
            and report.get('integrity_and_execution_passed') is True
            and report.get('timing_profile') == 'mpc25_dispatch50_v1',
            'A complete independently audited execution is required; performance may fail')
    run = Path(report['run_directory']).resolve(strict=True)
    for key, filename in [('task','controller_task.json'),('start','isolated_live/live_start.json'),
                           ('native','native_trace.csv')]:
        inputs[key] = checked_input(report, run/filename)
    task, start = read(inputs['task']['path']), read(inputs['start']['path'])
    case = task['robustness']['case_id']
    require(case in LABELS and task['duration_s'] == 24. and task['stage'] == 'fast', 'Wrong full-window case')
    require(start['configuration_sha256'] == list(bytes.fromhex(inputs['task']['sha256'])),
            'Original LiveStart task identity differs')
    origin = float(start['origin_s'])
    require(np.allclose(report['window_sim_s'], [origin,origin+24.],rtol=0,atol=1e-9),
            'Audit window differs from original actual start')
    t, values = native.native_values(inputs['native']['path'], origin, 24.)
    ref = native.shared.FigureEight(np.asarray(start['target_position_enu']), **task['trajectory_parameters'])
    desired = np.asarray([ref.sample(at)[0] for at in t])
    error = values[:,1:4]-desired
    norm = np.linalg.norm(error,axis=1)
    metrics = native.metric(t,norm,error)
    expected = report['endpoint']
    require(metrics['sample_count'] == expected['sample_count'] and all(
        np.allclose(metrics[k],expected[k],rtol=1e-10,atol=1e-12)
        for k in ('position_rmse_m','position_max_m','axis_rmse_m','duration_s')),
        'Plotted full-window metrics do not match original audit')
    inputs.update(renderer=identity(__file__), native_helper=identity(native.__file__),
                  reference_implementation=identity(native.shared.__file__))
    unchanged(inputs)
    return dict(case=case,report=report,run=run,task=task,inputs=inputs,t=t,values=values,
                desired=desired,error=error,norm=norm,metrics=metrics)


def render(cases, output):
    output = Path(output).resolve()
    require(not output.exists(), 'New output directory required')
    require(len({x['case'] for x in cases}) == len(cases), 'Duplicate case')
    output.mkdir(parents=True)
    plt = _plt()
    manifests=[]
    with plt.rc_context({'font.size':10,'pdf.fonttype':42,'path.simplify':False}):
        for item in cases:
            t,v,d = item['t'],item['values'],item['desired']
            fig,axes=plt.subplots(2,2,figsize=(12,9))
            fig.subplots_adjust(top=.84,bottom=.17,hspace=.42,wspace=.27)
            status='PASS' if item['report']['physical_acceptance'] else 'FAIL'
            m=item['metrics']
            fig.suptitle('X650 + two-link arm | '+LABELS[item['case']],y=.975,fontsize=16)
            fig.text(.5,.925,f'Complete 24 s | Physical acceptance: {status} | RMS {m["position_rmse_m"]*1000:.3f} mm | Peak {m["position_max_m"]*1000:.3f} mm',ha='center')
            fig.text(.5,.892,'Same 25 Hz upper MPC / 50 Hz output; no injected delay',ha='center',fontsize=9)
            axes[0,0].plot(t,item['norm']*1000,color='#0072B2',lw=1)
            axes[0,0].set(title='Full-window 3D endpoint error',xlabel='Task time [s]',ylabel='Error [mm]',xlim=(0,24),ylim=(0,None))
            axes[0,1].plot(d[:,0],d[:,2],'--',color='#D55E00',label='Original reference',lw=1)
            axes[0,1].plot(v[:,1],v[:,3],color='#0072B2',label='Actual native endpoint',lw=1)
            axes[0,1].set(title='World XZ trajectory',xlabel='World X [m]',ylabel='World Z [m]')
            axes[0,1].set_aspect('equal',adjustable='box');axes[0,1].legend(fontsize=8)
            for j,color in enumerate(('#0072B2','#D55E00')):
                axes[1,0].plot(t,v[:,4+j],label=f'Joint {j+1}',color=color,lw=1)
                axes[1,1].plot(t,v[:,6+j],label=f'Joint {j+1}',color=color,lw=1)
            axes[1,0].set(title='Actual joint position',xlabel='Task time [s]',ylabel='q [rad]',xlim=(0,24))
            axes[1,1].set(title='Actual joint velocity',xlabel='Task time [s]',ylabel='dq [rad/s]',xlim=(0,24))
            for ax in axes.flat: ax.grid(color='#dddddd',lw=.6);ax.ticklabel_format(useOffset=False)
            axes[1,0].legend(fontsize=8);axes[1,1].legend(fontsize=8)
            fig.text(.07,.035,'Source: '+item['run'].name+'\nNative same-time data; no phase fit, position shift, or resampling. Each case has its own vertical scale.',fontsize=8)
            for extension in ('png','pdf'):fig.savefig(output/(item['case']+'.'+extension),dpi=170,facecolor='white')
            plt.close(fig)
            chart=np.column_stack((v[:,0],t,v[:,1:4],d,item['error'],item['norm'],v[:,4:]))
            with (output/(item['case']+'_data.csv')).open('x',newline='',encoding='utf-8') as stream:
                writer=csv.writer(stream);writer.writerow(['sim_s','task_s',*[k+'_'+a+'_m' for k in ('actual','reference','error') for a in 'xyz'],
                    'error_norm_m','q1_rad','q2_rad','dq1_rad_s','dq2_rad_s']);writer.writerows(chart.tolist())
            unchanged(item['inputs'])
            manifests.append(dict(case_id=item['case'],physical_acceptance=item['report']['physical_acceptance'],
                integrity_and_execution_passed=True,metrics=m,inputs=item['inputs']))
    record=dict(kind='x650_multirate_robustness_plots_v1',presentation_complete=True,
        new_physics_run=False,physical_acceptance_granted_by_plot=False,cases=manifests,
        outputs={p.name:identity(p) for p in output.iterdir() if p.is_file()})
    (output/'plot_manifest.json').write_text(json.dumps(record,indent=2,allow_nan=False)+'\n')
    return record


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--report',type=Path,action='append',required=True)
    p.add_argument('--output',type=Path,required=True)
    a=p.parse_args();result=render([load(path) for path in a.report],a.output)
    print(json.dumps({'presentation_complete':True,'cases':[{k:v for k,v in case.items() if k!='inputs'} for case in result['cases']]},indent=2))


if __name__=='__main__':main()
