#!/usr/bin/env python3
"""Plot only the original controlled prefix of a rejected wrench experiment.

The right boundary is decoded from the rejected original ControlSnapshot CDR,
never the later recovery/landing time. This is diagnostic presentation only and
cannot produce a passed certificate or full-24-second performance claim.
"""
import argparse
import csv
import json
from pathlib import Path
import sys

import numpy as np

HERE = Path(__file__).resolve().parent
if str(HERE) not in sys.path: sys.path.insert(0, str(HERE))
from plot_x650_cpp_robustness import identity, unchanged, read, sha, require, _plt
from plot_x650_isolated_validation import shared


def decode_cdr(path, *, proposal=False):
    from flyarm_control_interfaces.msg import ControlProposal, ControlSnapshot
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.convert import message_to_ordereddict
    return dict(message_to_ordereddict(deserialize_message(Path(path).read_bytes(),
        ControlProposal if proposal else ControlSnapshot)))


def _bound_record(binding, expected_path):
    actual = identity(expected_path)
    require(isinstance(binding, dict) and Path(binding['path']).resolve() == Path(actual['path'])
        and all(actual.get(k) == v for k,v in binding.items()),
        'Rejected-call CDR differs from original native record')
    return actual


def load_prefix(run, analysis_path, *, sequence=379):
    run, analysis_path = Path(run).resolve(strict=True), Path(analysis_path).resolve(strict=True)
    names = {'start': 'isolated_live/live_start.json', 'task': 'controller_task.json',
        'metadata': 'controller_metadata.json', 'stage': 'stage_result.json',
        'native': 'native_trace.csv', 'native_manifest': 'native_trace_manifest.json',
        'wrench': 'physical_disturbance/applied_wrench.csv',
        'wrench_status': 'physical_disturbance/applied_wrench.csv.status.json',
        'rejected_call': 'isolated_live/native_records/%06d.json' % sequence}
    inputs = {name: identity(run / relative) for name,relative in names.items()}
    inputs['failure_analysis'] = identity(analysis_path)
    analysis = read(analysis_path)
    require(analysis.get('kind') == 'x650_robust_wrench_failed_prefix_analysis_v1'
        and Path(analysis['failed_original_run']).resolve() == run
        and analysis.get('failed_sequence') == sequence
        and analysis.get('physical_acceptance') is False and analysis.get('full_experiment_complete') is False,
        'Explicit failed-prefix analysis for this original run is required')
    for index,item in enumerate(analysis.get('original_evidence', [])):
        source = Path(item['source_path']).resolve(strict=True)
        actual = identity(source)
        require(actual['sha256'] == item['sha256'] and actual['bytes'] == item['bytes'],
                'Failure-analysis original evidence changed')
        inputs['analysis_original_%03d' % index] = actual
    for key in ('qp_feasibility_probe', 'constraint_group_isolation'):
        if isinstance(analysis.get(key), str): inputs[key] = identity(analysis[key])
    conflict = analysis_path.parent / 'direct_trust_conflict.json'
    if conflict.is_file(): inputs['direct_trust_conflict'] = identity(conflict)
    start, task, metadata, stage, record = [read(inputs[key]['path']) for key in
        ('start', 'task', 'metadata', 'stage', 'rejected_call')]
    require(task['stage'] == 'fast' and task['duration_s'] == 24.
        and task['robustness']['case_id'] == 'wrench_fast' and task['robustness']['kind'] == 'external_wrench'
        and metadata.get('complete') is False and stage.get('passed') is False,
        'Only the original failed external-wrench fast condition is admitted')
    require(record.get('kind') == 'cpp_full_pipeline_call_v1' and record.get('sequence') == sequence
        and record.get('error') == 'QP infeasible', 'Original call does not record the stated rejection')
    for key,suffix in [('snapshot_cdr','snapshot.cdr'),('proposal_cdr','proposal.cdr')]:
        path = run / 'isolated_live/native_records' / ('%06d.' % sequence + suffix)
        inputs[key] = _bound_record(record[key], path)
    snapshot = decode_cdr(inputs['snapshot_cdr']['path'])
    proposal = decode_cdr(inputs['proposal_cdr']['path'], proposal=True)
    configuration = list(bytes.fromhex(inputs['task']['sha256']))
    require(all(document.get('session_id') == start['session_id'] and document.get('epoch') == start['epoch']
        and document.get('configuration_sha256') == start['configuration_sha256'] == configuration
        and document.get('sequence') == sequence for document in (snapshot,proposal)),
        'Rejected CDR identity differs from actual LiveStart/task')
    require(proposal.get('snapshot_sequence') == sequence and proposal.get('status') == 2
        and proposal.get('accepted_optimum') is False and proposal.get('nonlinear_check_passed') is False,
        'Rejected proposal was unexpectedly accepted or nonlinear-qualified')
    origin, cutoff = float(start['origin_s']), float(snapshot['control_time_s'])
    elapsed = cutoff-origin
    require(np.isfinite([origin,cutoff,elapsed]).all() and 7. < elapsed < 7.6
        and abs(snapshot['task_time_s']-elapsed) <= 1e-9
        and abs(proposal['control_time_s']-cutoff) <= 1e-9
        and abs(proposal['task_time_s']-elapsed) <= 1e-9
        and abs(analysis['failed_snapshot_task_s']-elapsed) <= 1e-9,
        'Diagnostic cutoff is not the original rejected snapshot time')
    require(read(inputs['native_manifest']['path'])['sha256'] == inputs['native']['sha256'],
            'Native trace differs from original capture manifest')
    with Path(inputs['native']['path']).open(newline='', encoding='utf-8') as stream:
        selected = [r for r in csv.DictReader(stream) if origin-1e-9 <= float(r['sim_s']) <= cutoff+1e-9]
    require(len(selected) > 1 and all(all(r[k] == '1' for k in ('valid_base','valid_joint_state','valid_tip'))
        for r in selected), 'Invalid native state in controlled failure prefix')
    fields = ['sim_s', *['tip_'+a for a in 'xyz'], *['base_q'+a for a in 'wxyz'],
              'arm_joint_1_q','arm_joint_2_q','arm_joint_1_qd','arm_joint_2_qd']
    native = np.asarray([[float(r[k]) for k in fields] for r in selected])
    t = native[:,0]-origin
    require(np.isfinite(native).all() and np.all(np.diff(t)>0) and np.max(np.diff(t)) <= .004+1e-8
        and t[0] <= .004+1e-9 and abs(t[-1]-elapsed) <= .004+1e-9,
        'Controlled native prefix is incomplete or contains a later recovery sample')
    quat = native[:,4:8]; norms = np.linalg.norm(quat, axis=1)
    require(np.max(abs(norms-1.)) <= .001, 'Native tilt quaternion is not unit length')
    quat = quat / norms[:,None]
    tilt = np.rad2deg(np.arccos(np.clip(1.-2.*(quat[:,1]**2+quat[:,2]**2),-1.,1.)))
    reference = shared.FigureEight(np.asarray(start['target_position_enu']), **task['trajectory_parameters'])
    desired = np.asarray([reference.sample(v)[0] for v in t]); error = native[:,1:4]-desired
    norm = np.linalg.norm(error,axis=1)
    integrate = getattr(np,'trapezoid',None) or np.trapz
    metrics = dict(scope='Failed controlled prefix only; not complete 24 s or recovery qualification',
        interval_task_s=[float(t[0]),float(t[-1])], sample_count=len(t),
        tip_3d_rmse_m=float(np.sqrt(integrate(norm**2,t)/(t[-1]-t[0]))),
        tip_3d_peak_m=float(norm.max()),tip_error_at_last_native_m=float(norm[-1]),
        tilt_peak_deg=float(tilt.max()))
    expected = analysis['prefix_native_metrics']
    require(expected['sample_count'] == metrics['sample_count'] and all(np.isclose(expected[k],metrics[k],
        rtol=1e-10,atol=1e-12) for k in ('tip_3d_rmse_m','tip_3d_peak_m','tip_error_at_last_native_m','tilt_peak_deg')),
        'Diagnostic prefix differs from independently analyzed original physical samples')
    with Path(inputs['wrench']['path']).open(newline='',encoding='utf-8') as stream:
        # Before the immutable marker is observed the original plugin labels
        # its clock -1. Do not relabel those rows as if it had already armed.
        selected = [r for r in csv.DictReader(stream) if origin-1e-9 <= float(r['sim_s']) <= cutoff+1e-9
                    and float(r['t_s']) >= 0.]
    wrench = np.asarray([[float(r[k]) for k in ('sim_s','t_s','fx_enqueued_N','ty_com_enqueued_Nm')]
                         for r in selected])
    require(wrench.ndim == 2 and wrench.shape[1] == 4 and np.isfinite(wrench).all()
        and np.max(abs(wrench[:,0]-origin-wrench[:,1])) <= 1e-9
        and np.all(np.diff(wrench[:,1])>0) and wrench[0,1] <= .05+1e-9
        and wrench[-1,1] <= elapsed+1e-9,
        'Enqueued-force prefix has a different clock or includes recovery')
    bounds = np.asarray([task['mpc_config']['q_lower'],task['mpc_config']['q_upper']],float)
    require(bounds.shape==(2,2) and np.isfinite(bounds).all() and np.all(bounds[0]<bounds[1]),
            'Actual controller joint limits are missing')
    unchanged(inputs)
    return dict(run=run,sequence=sequence,origin_s=origin,cutoff_sim_s=cutoff,cutoff_task_s=elapsed,
        inputs=inputs,analysis=analysis,task=task,snapshot=snapshot,proposal=proposal,native=native,
        t=t,desired=desired,error=error,norm=norm,tilt=tilt,wrench=wrench,bounds=bounds,metrics=metrics)


def render(data, output):
    out=Path(output).resolve();require(not out.exists(),'Diagnostic output must be a new directory')
    unchanged(data['inputs']);plt=_plt();t=data['t'];cutoff=data['cutoff_task_s']
    with plt.rc_context({'font.size':10,'pdf.fonttype':42,'path.simplify':False}):
        fig,axes=plt.subplots(5,1,figsize=(12,14),sharex=True,
            gridspec_kw={'height_ratios':(1.,1.5,1.,1.,1.)})
        fig.subplots_adjust(left=.1,right=.97,top=.85,bottom=.12,hspace=.34)
        axes[0].step(data['wrench'][:,1],data['wrench'][:,2],where='post',color='#0072B2')
        axes[0].set(ylabel='Enqueued Fx [N]',ylim=(-.15,1.8))
        axes[1].plot(t,data['norm']*1000,color='#0072B2',label='Actual same-time native tip error')
        axes[1].set(ylabel='3D tip error [mm]',ylim=(0.,max(1.,float(data['norm'].max())*1200.)))
        axes[1].legend(loc='upper left',fontsize=9)
        for joint,axis in enumerate(axes[2:4]):
            axis.plot(t,data['native'][:,8+joint],color='#0072B2',label='Actual joint %d'%(joint+1))
            for row in range(2):axis.axhline(data['bounds'][row,joint],color='#D55E00',ls='--',lw=.9,
                label='Controller q lower / upper' if row==0 else None)
            axis.set_ylabel('Joint %d q [rad]'%(joint+1));axis.legend(loc='upper left',ncol=2,fontsize=8)
        axes[4].plot(t,data['tilt'],color='#0072B2',label='Actual body tilt')
        tilt_limit=float(data['task']['ts20_protocol']['tilt_max_deg'])
        axes[4].axhline(tilt_limit,color='#D55E00',ls='--',lw=.9,label='Original tilt envelope')
        axes[4].set(ylabel='Body tilt [deg]',xlabel='Task time [s]',ylim=(0.,max(tilt_limit,float(data['tilt'].max()))*1.12))
        axes[4].legend(loc='upper left',ncol=2,fontsize=8)
        for axis in axes:
            axis.axvline(7.,color='#CC79A7',lw=1.,ls='--')
            axis.axvline(cutoff,color='#B00020',lw=1.3,ls='--')
            axis.set_xlim(0.,cutoff);axis.grid(color='#dddddd',lw=.6);axis.set_axisbelow(True)
        axes[0].text(7.,1.02,'Pulse starts 7.000 s',transform=axes[0].get_xaxis_transform(),
                     ha='right',color='#8B3970',fontsize=9)
        axes[0].text(cutoff,1.14,'Rejected snapshot %.3f s'%cutoff,transform=axes[0].get_xaxis_transform(),
                     ha='right',va='bottom',color='#B00020',fontsize=9)
        title='PARTIAL FAILED RUN | wrench_fast | NOT A COMPLETE 24 s EXPERIMENT'
        fig.suptitle(title,y=.975,fontsize=15,color='#B00020')
        fig.text(.5,.94,'First +1.5 N world-X pulse interrupted before its planned 7.600 s end. Original MPC parameters unchanged.',
                 ha='center',fontsize=10)
        fig.text(.5,.905,'Only 0-%.3f s through original rejected call %d | prefix peak %.3f mm | peak tilt %.3f deg'%
            (cutoff,data['sequence'],data['metrics']['tip_3d_peak_m']*1000,data['metrics']['tilt_peak_deg']),ha='center',fontsize=10)
        fig.text(.1,.047,'Red boundary: original rejected ControlSnapshot control_time_s, not the later gate receipt or NAV_LAND.\n'
            'Native physical samples after this boundary are excluded. QP rejection is not a passed nonlinear candidate.\n'
            'Shown q limits differ from predicted stopping-reserve / trust constraints; see bound failure analysis.\n'
            'Force is an ECS enqueue increment, not a force sensor. No full-24 s RMS or recovery-performance claim.',fontsize=8.5,linespacing=1.5)
        out.mkdir(parents=True,exist_ok=False)
        fig.savefig(out/'wrench_failed_prefix.png',dpi=180,facecolor='white')
        fig.savefig(out/'wrench_failed_prefix.pdf',facecolor='white',metadata={'Title':title,
            'Subject':'Original rejected-snapshot cutoff; partial diagnostic, no complete experiment acceptance'})
        plt.close(fig)
    columns=['sim_s','task_s',*[f'{kind}_{a}_m' for kind in ('actual_tip','reference_tip','tip_error') for a in 'xyz'],
        'tip_error_norm_m','q1_rad','q2_rad','dq1_rad_s','dq2_rad_s','tilt_deg']
    matrix=np.column_stack((data['native'][:,0],t,data['native'][:,1:4],data['desired'],data['error'],
        data['norm'],data['native'][:,8:12],data['tilt']))
    with (out/'controlled_prefix.csv').open('x',newline='',encoding='utf-8') as stream:
        writer=csv.writer(stream);writer.writerow(columns);writer.writerows(matrix.tolist())
    with (out/'enqueued_wrench_prefix.csv').open('x',newline='',encoding='utf-8') as stream:
        writer=csv.writer(stream);writer.writerow(['sim_s','task_s','fx_enqueued_N','ty_com_enqueued_Nm'])
        writer.writerows(data['wrench'].tolist())
    unchanged(data['inputs'])
    manifest=dict(kind='x650_failed_wrench_diagnostic_figure_v1',presentation_complete=True,
        physical_acceptance=False,full_experiment_complete=False,hardware_qualified=False,
        original_run=str(data['run']),failed_sequence=data['sequence'],rejected_snapshot=data['snapshot'],
        rejected_proposal=data['proposal'],cutoff_sim_s=data['cutoff_sim_s'],cutoff_task_s=cutoff,
        diagnostic_prefix_metrics=data['metrics'],prefix_metric_scope='Controlled prefix only; excludes later recovery/landing',
        no_offset_or_phase_fit=True,resampled=False,inputs=data['inputs'],renderer=identity(__file__),
        outputs={p.name:identity(p) for p in sorted(out.iterdir())})
    (out/'diagnostic_figure_manifest.json').write_text(json.dumps(manifest,indent=2,allow_nan=False)+'\n',encoding='utf-8')
    return manifest


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run',type=Path,required=True)
    parser.add_argument('--failure-analysis',type=Path,required=True)
    parser.add_argument('--sequence',type=int,default=379)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    result=render(load_prefix(args.run,args.failure_analysis,sequence=args.sequence),args.output)
    print(json.dumps({k:result[k] for k in ('presentation_complete','physical_acceptance','full_experiment_complete',
        'failed_sequence','cutoff_task_s','diagnostic_prefix_metrics')},indent=2))
    return 0


if __name__=='__main__':raise SystemExit(main())
