#!/usr/bin/env python3
"""Present three certified physical-robustness cases without running physics.

Every curve is recomputed from its original native physical record through the
existing strict verifier. Failed/missing inputs remain explicitly failed; no
interpolation, phase fit, substituted quiet run or synthetic result is used.
"""
import argparse
import csv
import json
from pathlib import Path
import sys

import numpy as np

HERE = Path(__file__).resolve().parent
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))
import plot_x650_isolated_validation as original

KIND = 'x650_cpp_robustness_stage_certificate_v1'
CASES = ('wrench_fast', 'mass085_fast', 'mass115_fast')
LABELS = {'wrench_fast': 'Prescribed world force / free couple',
          'mass085_fast': 'Actual rigid mass + inertia x0.85',
          'mass115_fast': 'Actual rigid mass + inertia x1.15'}
read, sha, require = original.read, original.sha, original.require


def identity(path):
    path = Path(path).resolve(strict=True)
    return dict(path=str(path), sha256=sha(path), bytes=path.stat().st_size)


def unchanged(inputs):
    require(all(sha(item['path']) == item['sha256'] for item in inputs.values()),
            'Robustness presentation inputs changed while reading or plotting')


def decode_failure_snapshot(path):
    """Optional ROS decode for a failed label only; never grants acceptance."""
    from flyarm_control_interfaces.msg import ControlSnapshot
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.convert import message_to_ordereddict
    return dict(message_to_ordereddict(deserialize_message(Path(path).read_bytes(), ControlSnapshot)))


def failure_context(report, report_identity):
    """Bind the actual failed task and, if available, rejected-snapshot time.

    An early audit failure need not have copied robustness fields to the report
    yet. Its artifact map still binds the original task and native call records.
    A missing decoder/time yields an explicitly incomplete label, never a guess.
    """
    require(report.get('kind') == KIND and report.get('stage') == 'fast'
        and report.get('passed') is False, 'A failed robustness certificate is required for a failed label')
    run = Path(report['source_run_directory']).resolve(strict=True)
    inputs = {'report': report_identity,
              'task': original.checked_input(report, run / 'controller_task.json')}
    task = read(inputs['task']['path'])
    case = task.get('robustness', {}).get('case_id')
    require(case in CASES and task.get('stage') == 'fast' and task.get('duration_s') == 24.,
            'Failed label requires the actual bound predeclared task')
    result = dict(case_id=case, failure_reason='Incomplete experiment', failure_task_s=None,
                  failure_sequence=None, failure_time_source=None)
    records = run / 'isolated_live/native_records'
    paths = sorted(Path(p) for p in report.get('artifacts', {})
                   if Path(p).parent.resolve() == records and Path(p).suffix == '.json')
    for path in reversed(paths):
        item = original.checked_input(report, path)
        record = read(path)
        if record.get('kind') != 'cpp_full_pipeline_call_v1' or not record.get('error'):
            continue
        inputs['rejected_call'] = item
        result.update(failure_reason=str(record['error']), failure_sequence=record['sequence'])
        binding = record.get('snapshot_cdr', {})
        raw = records / ('%06d.snapshot.cdr' % record['sequence'])
        inputs['rejected_snapshot'] = original.checked_input(report, raw)
        actual = identity(raw)
        require(Path(binding['path']).resolve() == raw.resolve()
            and all(actual.get(k) == v for k,v in binding.items()), 'Failed-label CDR binding differs')
        inputs['live_start'] = original.checked_input(report, run / 'isolated_live/live_start.json')
        start = read(inputs['live_start']['path'])
        try:
            snapshot = decode_failure_snapshot(raw)
        except (ImportError, ModuleNotFoundError):
            result['failure_time_source'] = 'Original CDR bound; ROS decoder unavailable, no time inferred'
            break
        elapsed = float(snapshot['control_time_s']) - float(start['origin_s'])
        require(snapshot['session_id'] == start['session_id'] and snapshot['epoch'] == start['epoch']
            and snapshot['configuration_sha256'] == start['configuration_sha256']
                == list(bytes.fromhex(inputs['task']['sha256']))
            and snapshot['sequence'] == record['sequence'] and np.isfinite(elapsed)
            and 0 <= elapsed < 24. and abs(snapshot['task_time_s']-elapsed) <= 1e-9,
            'Failed-label snapshot differs from actual session, task or clock')
        result.update(failure_task_s=elapsed, failure_time_source='Original rejected ControlSnapshot CDR')
        break
    unchanged(inputs)
    result['failure_proof_inputs'] = inputs
    return result


def load_case(path):
    path = Path(path).resolve(strict=True)
    report = read(path)
    require(report.get('kind') == KIND and report.get('stage') == 'fast'
        and all(report.get(k) is True for k in ('passed', 'complete', 'certificate', 'physical_acceptance')),
        'Complete passed robustness physical certificate required')
    data = original.verified_data(path, certificate_kinds={'fast': KIND}, stage_durations={'fast': 24.})
    task = read(data['inputs']['task']['path'])
    robust = task.get('robustness')
    require(isinstance(robust, dict) and robust == report.get('robustness')
        and robust.get('case_id') in CASES, 'Certified actual task robustness condition differs')
    case = robust['case_id']
    expected = {'wrench_fast': ('external_wrench', 1.),
                'mass085_fast': ('uniform_rigid_mismatch', .85),
                'mass115_fast': ('uniform_rigid_mismatch', 1.15)}[case]
    require((robust.get('kind'), robust.get('mass_inertia_scale')) == expected,
            'Robustness case label differs from physical condition')
    physical = report.get('physical_model', {})
    require(physical.get('verified') is True and physical.get('scale') == expected[1]
        and np.isfinite(physical.get('total_mass_kg', float('nan')))
        and physical['total_mass_kg'] > 0
        and physical.get('controller_preparation_unchanged') is True
        and physical.get('controller_pid_values_unchanged') is True,
        'Physical model or unchanged nominal controller declaration is incomplete')
    performance = report.get('robustness_performance', {})
    require(performance.get('case_id') == case and performance.get('same_time') is True
        and performance.get('no_phase_alignment') is True and performance.get('delay_injected') is False,
        'Robustness performance scope differs')
    inputs = dict(data['inputs'])
    run = Path(report['source_run_directory'])
    wrench = None
    if case == 'wrench_fast':
        require(report.get('external_wrench', {}).get('verified') is True
            and report['external_wrench'].get('enabled') is True,
            'Independent native wrench audit is missing')
        schedule, events = robust['schedule'], performance.get('events', [])
        require(len(schedule) == len(events) == 3 and robust['recovery_threshold_m'] == .01
            and robust['recovery_hold_s'] == 1., 'Fixed pulse and recovery protocol differs')
        for event, expected_event in zip(events, schedule):
            require(all(event.get(k) == expected_event[k] for k in ('start_s', 'end_s'))
                and event.get('threshold_m') == .01 and event.get('hold_s') == 1.,
                'Certified recovery marker belongs to a different pulse')
            if event.get('recovered') is True:
                require(event.get('recovery_after_end_s') is not None
                    and event.get('confirmed_at_task_s') is not None
                    and np.isfinite([event['recovery_after_end_s'], event['confirmed_at_task_s']]).all(),
                    'A recovered pulse lacks its actual confirmation marker')
            else:
                require(event.get('recovered') is False and event.get('recovery_after_end_s') is None
                    and event.get('confirmed_at_task_s') is None,
                    'An unrecovered pulse must not acquire a fabricated marker')
        inputs['wrench'] = original.checked_input(report, run / 'physical_disturbance/applied_wrench.csv')
        with Path(inputs['wrench']['path']).open(newline='', encoding='utf-8') as stream:
            rows = list(csv.DictReader(stream))
        origin = read(data['inputs']['live_start']['path'])['origin_s']
        values = np.asarray([[float(row[k]) for k in
            ('sim_s', 't_s', 'fx_enqueued_N', 'ty_com_enqueued_Nm')] for row in rows])
        require(values.ndim == 2 and values.shape[1] == 4 and np.isfinite(values).all()
            and np.all(np.diff(values[:, 0]) > 0.), 'Invalid original enqueued wrench display trace')
        keep = (values[:, 1] >= 0.) & (values[:, 1] <= 24. + 1e-9)
        wrench = values[keep]
        require(len(wrench) > 1 and abs(wrench[0, 1]) <= .05 + 1e-9
            and wrench[-1, 1] >= 24. - .004 - 1e-9
            and np.max(abs(wrench[:, 0] - origin - wrench[:, 1])) <= 1e-9,
            'Native wrench presentation lacks the complete same-origin physical window')
    else:
        require(report.get('external_wrench', {}).get('enabled') is False
            and robust.get('schedule') == [], 'Mass-only condition contains an external waveform')
    unchanged(inputs)
    return dict(case_id=case, data=data, report=report, task=task, robustness=robust,
                physical=physical, performance=performance, inputs=inputs, wrench=wrench)


def _plt():
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    return plt


def _condition(case):
    p = case['physical']
    return (LABELS[case['case_id']] + ' | actual mass %.5f kg | nominal controller unchanged'
            % p['total_mass_kg'])


def _save_pair(figure, out, stem, *, title):
    figure.savefig(out / (stem + '.png'), dpi=180, facecolor='white')
    figure.savefig(out / (stem + '.pdf'), facecolor='white', metadata={'Title': title,
        'Subject': 'Certified same-time native simulation record; no offset or phase fitting'})


def error_limit_mm(norm, *, recovery_band=False):
    """Keep even a constant nonzero error trace visibly inside the axes."""
    peak = float(np.max(np.asarray(norm, float))) * 1000.
    return 1.2 * max(1., peak, 10. if recovery_band else 0.)


def render_case(case, output):
    """Dedicated thin presentation: the title always names the actual case."""
    out = Path(output)
    require(not out.exists(), 'Per-case chart output already exists')
    unchanged(case['inputs'])
    data = case['data']; t, actual = data['values'][:, 1], data['values'][:, 2:5]
    desired, errors, norm, metrics = (data[k] for k in ('desired', 'error', 'norm', 'metrics'))
    plt = _plt()
    with plt.rc_context({'font.size': 9, 'pdf.fonttype': 42, 'path.simplify': False}):
        fig = plt.figure(figsize=(13, 7.7))
        grid = fig.add_gridspec(2, 2, width_ratios=(1., 1.25), hspace=.4, wspace=.29,
                               left=.075, right=.97, bottom=.16, top=.80)
        path = fig.add_subplot(grid[:, 0]); error = fig.add_subplot(grid[0, 1])
        axes = fig.add_subplot(grid[1, 1], sharex=error)
        path.plot(actual[:, 0], actual[:, 2], color='#0072B2', lw=1.1, label='Native physical tip')
        path.plot(desired[:, 0], desired[:, 2], color='#D55E00', lw=1.1, ls='--', label='Same-time reference')
        path.set(title='End-effector world XZ path', xlabel='World X [m]', ylabel='World Z [m]')
        path.set_aspect('equal', adjustable='box'); path.ticklabel_format(useOffset=False)
        path.legend(fontsize=8)
        error.plot(t, norm*1000, color='#0072B2', lw=1.1, label='3D error')
        error.axhline(metrics['position_rmse_m']*1000, color='#444444', ls='--', lw=.9, label='Full-window RMS')
        if case['case_id'] == 'wrench_fast':
            error.axhline(10., color='#009E73', ls=':', lw=1., label='Fixed recovery band: 10 mm')
            for event in case['performance']['events']:
                for axis in (error, axes):
                    axis.axvspan(event['start_s'], event['end_s'], color='#CC79A7', alpha=.17)
        error.set(title='Same-time 3D position error', ylabel='Error norm [mm]',
                  ylim=(0, error_limit_mm(norm, recovery_band=case['case_id']=='wrench_fast')))
        error.legend(fontsize=8)
        for i, color, name in zip(range(3), ('#0072B2', '#D55E00', '#009E73'), 'XYZ'):
            axes.plot(t, errors[:, i]*1000, color=color, lw=.9, label=name)
        axes.axhline(0., color='#888888', lw=.6)
        axes.set(title='Signed error: actual minus reference', xlabel='Task time [s]',
                 ylabel='Axis error [mm]', xlim=(0, 24.)); axes.legend(ncol=3, fontsize=8)
        for axis in (path, error, axes):
            axis.grid(color='#dddddd', lw=.6); axis.set_axisbelow(True)
        title = 'X650 + two-link arm | ' + case['case_id'] + ' | Fast figure eight'
        fig.suptitle(title, y=.965, fontsize=14)
        fig.text(.5, .915, _condition(case), ha='center', fontsize=10)
        fig.text(.5, .865, 'Complete 24 s | %d native samples | 3D RMS %.3f mm | peak %.3f mm' %
            (len(t), metrics['position_rmse_m']*1000, metrics['position_max_m']*1000), ha='center', fontsize=10)
        fig.text(.075, .065,
            'Original Gazebo physics timestamps; same-time world reference, no alignment or resampling. No injected delay.\n'
            'Actual plant varies; these are separate conditions, not paired controller-comparison trials. Simulation only.\n'
            'Native CSV SHA256: ' + data['inputs']['native_trace']['sha256'], fontsize=8, linespacing=1.5)
        out.mkdir(parents=True, exist_ok=False)
        _save_pair(fig, out, 'endpoint_validation', title=title); plt.close(fig)
    fields = ['sim_s', 'task_s'] + [f'{kind}_{axis}_m' for kind in ('actual', 'reference', 'error') for axis in 'xyz'] + ['error_norm_m']
    with (out / 'endpoint_chart_data.csv').open('x', newline='', encoding='utf-8') as stream:
        writer = csv.writer(stream); writer.writerow(fields)
        writer.writerows(np.column_stack((data['values'], desired, errors, norm)).tolist())
    unchanged(case['inputs'])
    manifest = dict(kind='x650_cpp_robustness_case_figure_v1', complete=True, case_id=case['case_id'],
        display_title=title, condition=_condition(case), metrics=metrics, physical_model=case['physical'],
        inputs=case['inputs'], no_offset_or_phase_fit=True, resampled=False, hardware_qualified=False,
        renderer=identity(__file__), outputs={p.name: identity(p) for p in sorted(out.iterdir())})
    (out / 'figure_manifest.json').write_text(json.dumps(manifest, indent=2, allow_nan=False)+'\n', encoding='utf-8')
    return manifest


def render_wrench(case, output):
    require(case['case_id'] == 'wrench_fast' and case['wrench'] is not None, 'Wrench case required')
    out = Path(output); data, raw = case['data'], case['wrench']; plt = _plt()
    t, norm = data['values'][:, 1], data['norm']*1000
    with plt.rc_context({'font.size': 10, 'pdf.fonttype': 42, 'path.simplify': False}):
        fig, axes = plt.subplots(3, 1, figsize=(12, 9), sharex=True,
                                 gridspec_kw={'height_ratios': (1., 1., 2.)})
        fig.subplots_adjust(left=.09, right=.97, top=.85, bottom=.16, hspace=.28)
        axes[0].step(raw[:, 1], raw[:, 2], where='post', color='#0072B2')
        axes[0].set(ylabel='Enqueued Fx [N]', title='World ENU X force at base_link own CoM')
        axes[1].step(raw[:, 1], raw[:, 3], where='post', color='#D55E00')
        axes[1].set(ylabel='Enqueued Ty [N m]', title='World ENU Y free couple at base_link own CoM')
        axes[2].plot(t, norm, color='#0072B2', lw=1., label='Native same-time 3D tip error')
        axes[2].axhline(10., color='#009E73', ls='--', lw=1., label='Predeclared 10 mm recovery band')
        for i, event in enumerate(case['performance']['events'], 1):
            for axis in axes:
                axis.axvspan(event['start_s'], event['end_s'], color='#CC79A7', alpha=.16)
            if event['recovered']:
                first = event['end_s'] + event['recovery_after_end_s']; end = event['confirmed_at_task_s']
                axes[2].axvspan(first, end, color='#009E73', alpha=.13)
                axes[2].axvline(end, color='#009E73', lw=.8, ls=':')
                axes[2].annotate('P%d: %.3f s\n1 s confirmed' % (i, event['recovery_after_end_s']),
                    xy=(end, 10.), xytext=(3, 12), textcoords='offset points', fontsize=8, color='#006B52')
            else:
                axes[2].text(event['end_s'], .94-i*.08, 'P%d: NOT RECOVERED' % i,
                    transform=axes[2].get_xaxis_transform(), color='#B00020', fontsize=9)
        for axis in axes:
            axis.set_xlim(0., 24.); axis.grid(color='#dddddd', lw=.6); axis.set_axisbelow(True)
        axes[2].set(xlabel='Task time [s]', ylabel='Error norm [mm]',
                    ylim=(0., error_limit_mm(data['norm'], recovery_band=True)))
        axes[2].legend(loc='upper right', fontsize=8)
        title = 'wrench_fast | Immutable physical waveform and endpoint recovery'
        fig.suptitle(title, y=.97, fontsize=14)
        fig.text(.5, .92, 'Original nominal controller | recovery = absolute 3D error <=10 mm continuously for 1 s',
                 ha='center', fontsize=10)
        fig.text(.09, .065, 'Pink: immutable pulse intervals. Green: actual native samples confirming continuous recovery.\n'
            'Force / couple are same-PreUpdate ECS increments, not sensor measurements or independent dynamics-consumption ACKs.\n'
            'No pre-pulse bias subtraction, phase fit or injected communication delay. Simulation only.', fontsize=8.5, linespacing=1.5)
        _save_pair(fig, out, 'wrench_recovery', title=title); plt.close(fig)
    with (out / 'wrench_chart_data.csv').open('x', newline='', encoding='utf-8') as stream:
        writer = csv.writer(stream); writer.writerow(['sim_s', 'task_s', 'fx_enqueued_N', 'ty_com_enqueued_Nm'])
        writer.writerows(raw.tolist())
    unchanged(case['inputs'])


def batch_summary(entries, output):
    """Separate axes and explicit failed slots; no pooled controller ranking."""
    out = Path(output); plt = _plt()
    ordered = sorted(entries, key=lambda e: CASES.index(e['case_id']) if e.get('case_id') in CASES else 9)
    with plt.rc_context({'font.size': 10, 'pdf.fonttype': 42, 'path.simplify': False}):
        fig, axes = plt.subplots(3, 1, figsize=(12, 10), sharex=True)
        fig.subplots_adjust(left=.09, right=.97, top=.87, bottom=.14, hspace=.5)
        for axis, entry in zip(axes, ordered):
            name = entry.get('case_id') or 'Unassigned failed certificate'
            if entry['verified']:
                case = entry['case']; data = case['data']; metrics = data['metrics']
                axis.plot(data['values'][:, 1], data['norm']*1000, color='#0072B2', lw=1.)
                axis.set_title(name + ' | ' + _condition(case), loc='left', fontsize=10)
                axis.text(.99, .92, 'RMS %.3f mm | peak %.3f mm' %
                    (metrics['position_rmse_m']*1000, metrics['position_max_m']*1000),
                    transform=axis.transAxes, ha='right', va='top', fontsize=9)
                if name == 'wrench_fast':
                    axis.axhline(10., color='#009E73', ls='--', lw=.8)
                    for event in case['performance']['events']:
                        axis.axvspan(event['start_s'], event['end_s'], color='#CC79A7', alpha=.15)
            else:
                reason = entry.get('failure_reason', 'Incomplete or unverified experiment')
                if entry.get('failure_task_s') is not None:
                    reason += ' at task %.3f s' % entry['failure_task_s']
                axis.set_title(name + ' | FAILED - ' + reason, loc='left', color='#B00020')
                axis.text(.03, .58, 'The 24 s experiment did not complete. No full-window curve or metric is shown.',
                          transform=axis.transAxes, color='#B00020', fontsize=10)
                axis.text(.03, .35, 'A partial record or later recovery is not full-experiment acceptance.',
                          transform=axis.transAxes, color='#555555', fontsize=9)
            axis.set(ylabel='3D error [mm]', xlim=(0., 24.),
                ylim=(0., error_limit_mm(entry['case']['data']['norm'], recovery_band=name=='wrench_fast')
                      if entry['verified'] else 1.))
            axis.grid(color='#dddddd', lw=.6); axis.set_axisbelow(True)
        axes[-1].set_xlabel('Task time [s]')
        title = 'X650 coordinated MPC | Separate physical robustness conditions'
        fig.suptitle(title, y=.97, fontsize=14)
        fig.text(.5, .925, 'Each axis has its own error scale; actual plant or wrench differs. No injected delay.', ha='center')
        fig.text(.09, .055, 'Native physical samples and original same-time reference; no offset / phase fit or pooled ranking.\n'
            'All controller coefficients, gains, limits and identified nominal response remain unchanged.\n'
            'These tests do not identify a loaded TS20 hardware model or qualify 24 V real-flight operation.', fontsize=8.5, linespacing=1.5)
        _save_pair(fig, out, 'robustness_summary', title=title); plt.close(fig)


def run(reports, output):
    require(len(reports) == 3, 'Exactly the three predeclared case certificates are required')
    out = Path(output).resolve()
    require(not out.exists(), 'Output directory must be new; existing evidence is never overwritten')
    entries = []
    for path in reports:
        entry = dict(report=str(Path(path).resolve()), verified=False, case_id=None)
        preliminary = None
        try:
            entry['report_identity'] = identity(path)
            preliminary = read(path)
            entry['case_id'] = preliminary.get('robustness', {}).get('case_id')
            entry['case'] = load_case(path)
            entry.update(verified=True, case_id=entry['case']['case_id'])
        except Exception as error:
            entry['error'] = repr(error)
            try:
                entry.update(failure_context(preliminary, entry['report_identity']))
            except Exception as context_error:
                entry['failure_context_error'] = repr(context_error)
        entries.append(entry)
    known = [e['case_id'] for e in entries if e['case_id'] in CASES]
    require(len(set(known)) == len(known), 'Repeated robustness case certificates are forbidden')
    if all(e['verified'] for e in entries):
        require(set(known) == set(CASES), 'The complete three-case protocol is required')
    out.mkdir(parents=True, exist_ok=False)
    for entry in entries:
        if entry['verified']:
            render_case(entry['case'], out / entry['case_id'])
            if entry['case_id'] == 'wrench_fast':
                render_wrench(entry['case'], out / entry['case_id'])
    batch_summary(entries, out)
    result = dict(kind='x650_cpp_robustness_presentation_batch_v1',
        complete=all(e['verified'] for e in entries), certificate_acceptance_granted=False,
        no_delay_injected=True, no_offset_or_phase_fit=True, no_resampling=True,
        hardware_qualified=False, renderer=identity(__file__), cases=[])
    for entry in entries:
        item = {k:v for k,v in entry.items() if k != 'case'}
        if entry['verified']:
            case = entry['case']; unchanged(case['inputs'])
            item.update(metrics=case['data']['metrics'], physical_model=case['physical'],
                robustness=case['robustness'], robustness_performance=case['performance'], inputs=case['inputs'])
        elif entry.get('failure_proof_inputs'):
            unchanged(entry['failure_proof_inputs'])
        result['cases'].append(item)
    result['all_predeclared_recoveries_passed'] = next((e['case']['performance'].get('performance_passed')
        for e in entries if e['verified'] and e['case_id'] == 'wrench_fast'), False)
    result['outputs'] = {p.relative_to(out).as_posix(): identity(p) for p in sorted(out.rglob('*')) if p.is_file()}
    (out / 'presentation_summary.json').write_text(json.dumps(result, indent=2, allow_nan=False)+'\n', encoding='utf-8')
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--reports', nargs=3, type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    result = run(args.reports, args.output)
    print(json.dumps(dict(output=str(args.output.resolve()), complete=result['complete'],
        all_predeclared_recoveries_passed=result['all_predeclared_recoveries_passed'],
        cases=[{k:c.get(k) for k in ('case_id','verified','error','metrics')} for c in result['cases']]), indent=2))
    return 0 if result['complete'] else 1


if __name__ == '__main__': raise SystemExit(main())
