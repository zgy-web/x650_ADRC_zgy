#!/usr/bin/env python3
"""Plot one recorded wrench experiment, including honest controlled prefixes.

This is a renderer, not a physics run or a qualification gate. Reference values
are evaluated at the native physical timestamps using the same FigureEight as
plot_x650_cpp_robustness. No phase fit, position offset or future padding exists.
"""
from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path, PurePosixPath
import struct
import sys

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
from plot_x650_cpp_robustness import identity, unchanged, read, require, _plt
from plot_x650_isolated_validation import shared


def snapshot_identity(raw):
    """Read only the fixed primitive prefix of this version-1 CDR message.

    The 4-byte encapsulation is not part of CDR alignment. Both endian CDR1
    encodings are supported; XCDR2/parameter-list encodings are refused. Reading
    this prefix needs no ROS installation and never constructs a control input.
    The caller separately checks the complete original CDR SHA and byte count.
    """
    require(len(raw) == 7628 and raw[:4] in (b'\x00\x01\x00\x00', b'\x00\x00\x00\x00'),
            'Unsupported ControlSnapshot v1 CDR encoding/length')
    order = '<' if raw[1] == 1 else '>'
    version = struct.unpack_from(order+'H', raw, 4)[0]
    session, epoch, sequence = struct.unpack_from(order+'QQQ', raw, 12)
    issued, control, task, dt = struct.unpack_from(order+'Qddd', raw, 76)
    require(version == 1 and np.isfinite([control, task, dt]).all(), 'Invalid snapshot identity/clock')
    return dict(session_id=session, epoch=epoch, sequence=sequence,
                configuration_sha256=list(raw[37:69]), issued_monotonic_ns=issued,
                control_time_s=control, task_time_s=task, dt_s=dt)


def metric(t, norm, error):
    require(len(t) > 1 and np.all(np.diff(t) > 0), 'Metric requires increasing actual native times')
    integrate = getattr(np, 'trapezoid', None) or np.trapz
    return dict(position_rmse_m=float(np.sqrt(integrate(norm**2, t)/(t[-1]-t[0]))),
                position_max_m=float(norm.max()),
                axis_rmse_m=np.sqrt(integrate(error**2, t, axis=0)/(t[-1]-t[0])).tolist(),
                sample_count=len(t), interval_task_s=[float(t[0]), float(t[-1])],
                duration_s=float(t[-1]-t[0]), same_time=True, no_phase_alignment=True)


def controlled_cutoff(launch_record, metadata, stage, calls, duration):
    """Exclude recovery/landing; absent a clean finish use last real snapshot."""
    require(calls, 'No original worker snapshot establishes the controlled interval')
    errors = [c for c in calls if c['error']]
    full = (not errors and launch_record.get('complete') is True
            and metadata.get('complete') is True and stage.get('complete') is True
            and stage.get('passed') is True)
    if full:
        return duration, 'Clean original launch/metadata/stage finish; native coverage checked separately'
    last = errors[0] if errors else calls[-1]
    cutoff = min(duration, last['task_time_s'])
    require(0 < cutoff <= duration, 'No positive controlled prefix available')
    return cutoff, ('First rejected original ControlSnapshot boundary' if errors else
                    'Last recorded original ControlSnapshot; incomplete lifecycle, conservative cutoff')


def native_values(path, origin, cutoff):
    with Path(path).open(newline='', encoding='utf-8') as stream:
        selected = [r for r in csv.DictReader(stream)
                    if -1e-9 <= float(r['sim_s'])-origin <= cutoff+1e-9]
    require(len(selected) > 1 and all(all(r[k] == '1' for k in
        ('valid_base', 'valid_joint_state', 'valid_tip')) for r in selected), 'Invalid native physical prefix')
    fields = ['sim_s', *['tip_'+a for a in 'xyz'], 'arm_joint_1_q', 'arm_joint_2_q',
              'arm_joint_1_qd', 'arm_joint_2_qd']
    values = np.asarray([[float(r[k]) for k in fields] for r in selected], float)
    t = values[:, 0]-origin
    require(np.isfinite(values).all() and np.all(np.diff(t) > 0)
            and np.max(np.diff(t)) <= .008+1e-9 and t[0] <= .008+1e-9
            and t[-1] >= cutoff-.008-1e-9, 'Native physical prefix has a gap or misses its boundary')
    return t, values


def _locate_run(launch, record):
    values = record.get('run_directories', [])
    require(len(values) == 1, 'Exactly one original run must belong to this launch')
    recorded = Path(values[0])
    if recorded.is_absolute() and recorded.is_dir():
        return recorded.resolve(), False
    # Explicit portable archive layout: keep the immutable recorded path as
    # provenance, and only select its sibling directory name. Never search or
    # substitute an unrelated successful run.
    name = PurePosixPath(str(values[0]).replace('\\', '/')).name
    candidate = launch.parent/name
    require(candidate.is_dir(), 'Original run unavailable; extract sibling launch/run archive directories')
    return candidate.resolve(), True


def load(launch, audit=None):
    launch = Path(launch)
    require(launch.is_absolute(), '--launch must be an absolute directory')
    launch = launch.resolve(strict=True)
    inputs = {}

    def bind(name, path):
        inputs[name] = identity(path)
        return Path(inputs[name]['path'])

    launch_record = read(bind('launch', launch/'launch.json'))
    run, relocated = _locate_run(launch, launch_record)
    task = read(bind('task', launch/'task.json'))
    require(read(bind('actual_task', run/'controller_task.json')) == task == launch_record['task'],
            'Launch and actual task differ')
    duration = float(task['duration_s'])
    require(duration == 24. and task['stage'] == 'fast'
            and task['robustness']['kind'] == 'external_wrench'
            and len(task['robustness']['schedule']) == 3, 'Original 24 s three-pulse wrench task required')
    metadata = read(bind('metadata', run/'controller_metadata.json'))
    stage = read(bind('stage', run/'stage_result.json'))
    start = read(bind('live_start', run/'isolated_live/live_start.json'))
    origin = float(start['origin_s'])
    require(np.isfinite(origin) and metadata['task'] == task, 'Original task/start binding differs')
    require(start['configuration_sha256'] == list(bytes.fromhex(inputs['task']['sha256'])),
            'Actual LiveStart task SHA differs')
    calls = []
    for path in sorted((run/'isolated_live/native_records').glob('*.json')):
        item = read(bind('call:'+path.name, path))
        sequence = int(item['sequence'])
        raw = bind('snapshot:'+path.name, path.with_suffix('.snapshot.cdr'))
        actual = inputs['snapshot:'+path.name]
        require(actual['sha256'] == item['snapshot_cdr']['sha256']
                and actual['bytes'] == item['snapshot_cdr']['bytes'], 'Original snapshot SHA/length differs')
        snapshot = snapshot_identity(raw.read_bytes())
        require(snapshot['sequence'] == sequence and snapshot['session_id'] == start['session_id']
                and snapshot['epoch'] == start['epoch']
                and snapshot['configuration_sha256'] == start['configuration_sha256']
                and abs(snapshot['control_time_s']-origin-snapshot['task_time_s']) <= 1e-9,
                'Original snapshot session/task/time differs')
        d = item.get('result', {}).get('diagnostics', item.get('rejected_diagnostics', {}))
        calls.append(dict(sequence=sequence, task_time_s=snapshot['task_time_s'],
            error=item.get('error', ''), core_ms=d.get('compute_ms'),
            recovery_triggered=d.get('recovery_triggered'),
            recovery_attempts=len(d.get('recovery_attempts', [])),
            interval_repairs=len(d['interval_repairs']) if 'interval_repairs' in d else None,
            interval_rows=d.get('interval_rows'), subproblems=d.get('subproblems')))
    require(calls and all(b['sequence'] > a['sequence'] and b['task_time_s'] > a['task_time_s']
        for a,b in zip(calls, calls[1:])), 'Repeated/out-of-order actual worker calls')
    cutoff, cutoff_source = controlled_cutoff(launch_record, metadata, stage, calls, duration)
    native = bind('native_trace', run/'native_trace.csv')
    native_manifest = read(bind('native_manifest', run/'native_trace_manifest.json'))
    require(native_manifest['sha256'] == inputs['native_trace']['sha256'], 'Native capture SHA differs')
    t, values = native_values(native, origin, cutoff)
    reference = shared.FigureEight(np.asarray(start['target_position_enu'], float), **task['trajectory_parameters'])
    desired = np.asarray([reference.sample(at)[0] for at in t], float)
    error = values[:, 1:4]-desired
    norm = np.linalg.norm(error, axis=1)
    require(np.isfinite(desired).all(), 'Nonfinite original same-time reference')
    metrics = metric(t, norm, error)
    timing = read(bind('worker_timing', run/'isolated_live/worker_timing.json'))
    columns = timing['columns']
    require(len(columns) == len(set(columns)) and all(len(r) == len(columns) for r in timing['rows']),
            'Malformed worker timing columns')
    phases = {int(r[0]): dict(zip(columns, r)) for r in timing['rows']}
    require(len(phases) == len(timing['rows']), 'Repeated worker timing sequence')
    for call in calls:
        row = phases.get(call['sequence'])
        require(row is not None and int(row['sequence']) == call['sequence'], 'Call missing actual timing')
        begin, end = row['core_begin_ns'], row['core_end_ns']
        # A thrown native call has no core_end_ns. Do not relabel callback or
        # publish time as Core time to fill that missing measurement.
        call['core_wall_ms'] = (end-begin)/1e6 if end >= begin > 0 else None
        call['callback_ms'] = (row['callback_exit_ns']-row['callback_entry_ns'])/1e6
        require(np.isfinite(call['callback_ms']) and call['callback_ms'] >= 0, 'Invalid callback duration')
    full = cutoff == duration and t[-1] >= duration-.008-1e-9
    report = None
    if audit is not None:
        report = read(bind('audit', audit))
        require(report.get('kind') in ('x650_cpp_recovery_robustness_stage_certificate_v1',
            'x650_cpp_interval_robustness_stage_certificate_v1'), 'Wrong audit kind')
        require(PurePosixPath(str(report['source_run_directory']).replace('\\','/')).name == run.name,
                'Audit belongs to a different recorded run')
        for name in ('native_trace', 'actual_task', 'live_start'):
            original_path = str(inputs[name]['path'])
            expected = report.get('artifacts', {}).get(original_path)
            if relocated:
                matches = [v for p,v in report.get('artifacts',{}).items()
                           if p.endswith('/'+run.name+'/'+str(Path(original_path).relative_to(run)).replace('\\','/'))]
                require(len(matches) == 1, 'Relocated audit artifact identity is ambiguous')
                expected = matches[0]
            require(expected == inputs[name]['sha256'], 'Audit does not bind chart input: '+name)
        if report.get('passed') is True:
            require(full and report.get('physical_acceptance') is True, 'Passed audit lacks complete physical window')
            certified = report['endpoint']
            require(certified['sample_count'] == metrics['sample_count'] and all(
                np.allclose(metrics[k], certified[k], rtol=1e-10, atol=1e-12)
                for k in ('position_rmse_m','position_max_m','axis_rmse_m','duration_s')),
                'Recomputed chart metrics differ from independent audit')
    unchanged(inputs)
    return dict(launch=launch, run=run, relocated=relocated, original_run=launch_record['run_directories'][0],
        task=task, inputs=inputs, t=t, native=values, desired=desired, error=error, norm=norm, metrics=metrics,
        calls=[c for c in calls if -1e-9 <= c['task_time_s'] <= cutoff+1e-9], full=full,
        cutoff=cutoff, cutoff_source=cutoff_source, audit=report)


def render(data, output):
    out = Path(output).resolve()
    require(not out.exists(), 'Choose a new output directory; original results are never overwritten')
    unchanged(data['inputs'])
    plt = _plt()
    t, values, desired = data['t'], data['native'], data['desired']
    task = data['task']; schedule = task['robustness']['schedule']; duration = task['duration_s']
    accepted = bool(data['audit'] and data['audit'].get('passed') is True)
    status = ('COMPLETE RECORDED WINDOW' if data['full'] else 'INCOMPLETE CONTROLLED PREFIX')
    status += ' | ' + ('independent audit PASSED' if accepted else 'no physical acceptance granted by this plot')
    subtitle = ('Observed %.3f-%.3f s | %d native samples | %s RMS %.3f mm | peak %.3f mm' %
        (t[0], t[-1], len(t), '24 s' if data['full'] else 'PREFIX',
         data['metrics']['position_rmse_m']*1000, data['metrics']['position_max_m']*1000))

    def shade(axis):
        for i,event in enumerate(schedule,1):
            axis.axvspan(event['start_s'], event['end_s'], color='#CC79A7', alpha=.16)
            axis.text((event['start_s']+event['end_s'])/2, .97, 'P%d'%i,
                      transform=axis.get_xaxis_transform(), ha='center', va='top', fontsize=8, color='#873265')
        if not data['full']:
            axis.axvspan(data['cutoff'], duration, color='#777777', alpha=.10)
            axis.axvline(data['cutoff'], color='#B00020', ls='--', lw=1.)
        axis.set_xlim(0, duration); axis.grid(color='#dddddd', lw=.5); axis.set_axisbelow(True)

    def title(fig, heading):
        fig.suptitle('X650 + two-link arm | '+heading, y=.982, fontsize=14)
        fig.text(.5,.947,status,ha='center',fontsize=10,color='#B00020' if not data['full'] else '#333333')
        fig.text(.5,.919,subtitle,ha='center',fontsize=9)
        fig.text(.065,.025,'Recorded run: '+data['run'].name+'\n'
            'Native world ENU states; original same-time FigureEight. No alignment, resampling or future padding. Simulation only.\n'
            'Pink: prescribed pulse intervals. Grey/red: unobserved future after controlled cutoff; no curve is extended there.',
            fontsize=7.5,linespacing=1.45)

    def save(fig, stem):
        fig.savefig(out/(stem+'.png'),dpi=180,facecolor='white')
        fig.savefig(out/(stem+'.pdf'),facecolor='white',metadata={'Title':stem,'Subject':status})
        plt.close(fig)

    out.mkdir(parents=True,exist_ok=False)
    with plt.rc_context({'font.size':9,'pdf.fonttype':42,'path.simplify':False}):
        fig = plt.figure(figsize=(13,10))
        grid = fig.add_gridspec(2,2,left=.075,right=.97,top=.86,bottom=.13,hspace=.40,wspace=.24)
        axis = fig.add_subplot(grid[0,:])
        axis.plot(t,data['norm']*1000,color='#0072B2',lw=1.,label='Native same-time 3D error')
        axis.axhline(10,color='#009E73',ls='--',lw=.9,label='Original recovery band: 10 mm for 1 s')
        axis.set(ylabel='3D tip error [mm]',xlabel='Task time [s]',title='A  Actual end-effector position error',
                 ylim=(0,max(12.,data['norm'].max()*1150)))
        shade(axis);axis.legend(loc='upper left',fontsize=8)
        xz=fig.add_subplot(grid[1,0]);xyz=fig.add_subplot(grid[1,1],projection='3d')
        for ax in (xz,xyz):
            coords=(0,2) if ax is xz else (0,1,2)
            ax.plot(*[values[:,1+i] for i in coords],color='#0072B2',lw=1.,label='Native physical tip')
            ax.plot(*[desired[:,i] for i in coords],color='#D55E00',ls='--',lw=.9,label='Same-time reference (observed interval)')
            ax.set(xlabel='World X [m]',ylabel='World Z [m]' if ax is xz else 'World Y [m]')
            ax.legend(fontsize=7,loc='best');ax.ticklabel_format(useOffset=False)
        xz.set_title('B  Recorded XZ path');xz.set_aspect('equal',adjustable='box');xz.grid(color='#dddddd')
        xyz.set(zlabel='World Z [m]',title='C  Recorded XYZ path');xyz.view_init(elev=22,azim=-65)
        from matplotlib.ticker import MaxNLocator
        for direction in (xyz.xaxis,xyz.yaxis,xyz.zaxis):direction.set_major_locator(MaxNLocator(4))
        title(fig,'Single disturbed fast figure eight');save(fig,'endpoint_recovery')

        fig=plt.figure(figsize=(13,12))
        grid=fig.add_gridspec(4,2,left=.075,right=.97,top=.87,bottom=.12,hspace=.54,wspace=.24)
        qlo,qhi=np.asarray(task['mpc_config']['q_lower']),np.asarray(task['mpc_config']['q_upper'])
        vel=np.asarray(task['ts20_protocol']['joint_velocity_max_rad_s'])
        for j in range(2):
            qaxis=fig.add_subplot(grid[0,j]);vaxis=fig.add_subplot(grid[1,j])
            qaxis.plot(t,values[:,4+j],color='#0072B2',lw=.9,label='Native actual q')
            for limit in (qlo[j],qhi[j]):qaxis.axhline(limit,color='#D55E00',ls='--',lw=.8)
            qaxis.set(title='Joint %d actual angle and original q bounds'%(j+1),ylabel='q [rad]')
            vaxis.plot(t,values[:,6+j],color='#0072B2',lw=.9,label='Native actual dq')
            for limit in (-vel[j],vel[j]):vaxis.axhline(limit,color='#D55E00',ls='--',lw=.8)
            vaxis.set(title='Joint %d actual speed and original speed bounds'%(j+1),ylabel='dq [rad/s]')
            shade(qaxis);shade(vaxis)
        timeaxis=fig.add_subplot(grid[2,:]);eventaxis=fig.add_subplot(grid[3,:])
        calls=data['calls'];ct=np.asarray([c['task_time_s'] for c in calls])
        for field,label,color in (('core_wall_ms','Measured Core wall time','#0072B2'),
                                  ('callback_ms','Measured full worker callback','#E69F00')):
            keep=[i for i,c in enumerate(calls) if c[field] is not None]
            timeaxis.plot(ct[keep],[calls[i][field] for i in keep],'.-',ms=2,lw=.65,label=label,color=color)
        timeaxis.axhline(50,color='#B00020',ls='--',lw=.8,label='Original 50 ms deadline')
        timeaxis.axhline(20,color='#888888',ls=':',lw=.8,label='20 ms target output period')
        timeaxis.set(title='Native worker timing; a thrown Core call has no fabricated core_end',ylabel='Duration [ms]',ylim=(0,None))
        timeaxis.legend(ncol=2,fontsize=8);shade(timeaxis)
        known=[c for c in calls if c['recovery_triggered'] is not None]
        if known:
            for triggered,level,label,color in ((False,0,'Normal call','#999999'),(True,1,'Recovery invoked','#009E73')):
                chosen=[c for c in known if c['recovery_triggered'] is triggered
                        and (triggered or not c.get('interval_repairs'))]
                eventaxis.scatter([c['task_time_s'] for c in chosen],[level]*len(chosen),s=8,
                    label=label+' (%d calls)'%len(chosen),color=color)
        else:
            eventaxis.text(.02,.5,'Recovery telemetry absent in this recorded backend; no zero-recovery claim',
                           transform=eventaxis.transAxes,color='#555555')
        interval=[c for c in calls if c.get('interval_repairs')]
        if interval:
            eventaxis.scatter([c['task_time_s'] for c in interval],[1.5]*len(interval),marker='^',
                color='#0072B2',s=22,label='Interval hard-constraint repair (%d calls)'%len(interval))
        failed=[c for c in calls if c['error']]
        if failed:eventaxis.scatter([c['task_time_s'] for c in failed],[2]*len(failed),marker='x',color='#B00020',s=50,label='Rejected actual call')
        eventaxis.set(title='Actual native recovery events (internal centre generation is not an actuator command)',
                      ylabel='Event',xlabel='Task time [s]',ylim=(-.3,2.5),yticks=[0,1,1.5,2],yticklabels=['normal','centre','interval','rejected'])
        shade(eventaxis)
        if known or failed:eventaxis.legend(loc='upper left',fontsize=8,ncol=3)
        title(fig,'Actual joints and C++ worker timing');save(fig,'joints_and_backend')
    unchanged(data['inputs'])
    chart=np.column_stack((values[:,0],t,values[:,1:4],desired,data['error'],data['norm'],values[:,4:]))
    with (out/'native_chart_data.csv').open('x',newline='',encoding='utf-8') as stream:
        writer=csv.writer(stream);writer.writerow(['sim_s','task_s',*[k+'_'+a+'_m' for k in ('actual','reference','error') for a in 'xyz'],
            'error_norm_m','q1_rad','q2_rad','dq1_rad_s','dq2_rad_s']);writer.writerows(chart.tolist())
    with (out/'worker_chart_data.json').open('x',encoding='utf-8') as stream:json.dump(data['calls'],stream,indent=2,allow_nan=False)
    manifest=dict(kind='x650_recovery_disturbance_plots_v1',presentation_complete=True,
        full_recorded_window=data['full'],physical_acceptance=accepted,certificate_granted_by_renderer=False,
        original_launch=str(data['launch']),original_run=data['original_run'],local_run=str(data['run']),
        archive_relocated=data['relocated'],controlled_cutoff_task_s=data['cutoff'],cutoff_source=data['cutoff_source'],
        metric_scope='Complete recorded 24 s window' if data['full'] else 'Controlled prefix only; no full-experiment RMS or recovery qualification',
        metrics=data['metrics'],prescribed_schedule=schedule,inputs=data['inputs'],
        no_offset_or_phase_fit=True,resampled=False,future_data_padded=False,hardware_qualified=False,
        renderer=identity(__file__),reference_implementation=identity(shared.__file__),
        dependencies={name:identity(sys.modules[name].__file__) for name in
            ('plot_x650_cpp_robustness','plot_x650_isolated_validation')},
        snapshot_schema=identity(HERE.parent/'ros2_ws/src/flyarm_control_interfaces/msg/ControlSnapshot.msg'),
        outputs={p.name:identity(p) for p in sorted(out.iterdir())})
    with (out/'plot_manifest.json').open('x',encoding='utf-8') as stream:json.dump(manifest,stream,indent=2,allow_nan=False)
    return manifest


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--launch',type=Path,required=True)
    parser.add_argument('--audit',type=Path)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args();result=render(load(args.launch,args.audit),args.output)
    print(json.dumps({k:result[k] for k in ('full_recorded_window','physical_acceptance','controlled_cutoff_task_s','metrics')},indent=2))
    return 0


if __name__=='__main__':raise SystemExit(main())
