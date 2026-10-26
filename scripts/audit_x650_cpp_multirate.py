#!/usr/bin/env python3
"""Independent full-window acceptance for the explicit multirate experiment.

Legacy auditors and their default thresholds are not modified. This version
retains physical/source/consumption/PX4 checks and makes changed timing explicit.
"""
from copy import deepcopy
from pathlib import Path
import argparse
import json
import sys
import traceback

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT),str(ROOT/'src'),str(ROOT/'scripts'),str(ROOT/'control_packages/x650_wholebody_v1')]
from simulations.x650_cpp_multirate_v1 import qualification, task, profile
from report_x650_cpp_live import identity, require, read
from audit_x650_cpp_hold import rows, same, summary, numeric_array, check_numeric_result, check_actual_send_barrier, check_recovery, ulog_value_view
from audit_x650_interval_robustness import interval_fixture, check_runtime_intervals, failure_context
from audit_x650_cpp_robustness import endpoint_windows, robustness_performance, check_wrench, check_physical_model
import audit_x650_recovery_replay as replay
import audit_x650_interval_constraints as interval
import audit_x650_upper_stage as upper

fields, trajectory_matrix, precision = upper.fields, upper.trajectory_matrix, upper.precision
NED_TO_ENU = upper.NED_TO_ENU
PVA_DELIVERY_MAX_S, CLOCK_BOUNDARY_S = upper.PVA_DELIVERY_MAX_S, upper.CLOCK_BOUNDARY_S
PVA_FLOAT_TOLERANCE, PROXY_AXIS_RMS_LIMIT = upper.PVA_FLOAT_TOLERANCE, upper.PROXY_AXIS_RMS_LIMIT


def check_archives(launch, run, meta):
    record = read(launch/'launch.json')
    require(all(record.get(k) is True for k in ('complete','runtime_closed','sources_unchanged',
        'runtime_binaries_unchanged','launch_inputs_unchanged')) and record.get('exit_code') == 0,
        'Incomplete or changing multirate launch')
    sources, binaries = qualification.archive_integrity(launch, record)
    cfg, protocol, startup = (read(launch/name) for name in ('task.json','protocol.json','startup_manifest.json'))
    require(task.validate_task_config(cfg) == cfg == record['task'] == meta['task'] == read(run/'controller_task.json')
        and record.get('execution_scenario') == cfg['scenario']
        and record.get('run_directories') == [str(run.resolve())]
        and protocol == record['protocol'] and cfg['duration_s'] == protocol['duration_s'],
        'Actual multirate task/reference/window differs')
    qualification.validate_protocol(protocol, cfg)
    for item in record['launch_inputs']:
        require(identity(item['path']) == item, 'Launch input changed')
    original = read(launch/'original_startup_manifest.json')
    require({k:v for k,v in startup.items() if k != 'task_config'}
        == {k:v for k,v in original.items() if k != 'task_config'}
        and startup['task_config']['sha256'] == original['task_config']['sha256'] == qualification.sha(launch/'task.json'),
        'Startup derivation changed something besides the identical task path')
    manifest = read(launch/'qualification_manifest.json')
    proof = qualification.verify(launch/'qualification_manifest.json', task_path=Path(manifest['task_config']['path']),
        startup_path=Path(manifest['startup_manifest']['path']), startup_sha256=manifest['startup_manifest']['sha256'],
        protocol_path=Path(manifest['protocol']['path']), binaries=binaries)
    require(proof.get('kind') == 'x650_cpp_multirate_admission_v1' and proof.get('ready') is True
        and proof.get('physical_acceptance') is False, 'New ground admission cannot substitute for physical acceptance')
    for key, name in [('task_config','task.json'),('protocol','protocol.json'),('startup_manifest','original_startup_manifest.json')]:
        require(manifest[key]['sha256'] == qualification.sha(launch/name), 'Actual archived admission input differs')
    required = qualification.EXECUTION_SOURCES | {
        'simulations/x650_cpp_multirate_v1/profile.py','simulations/x650_cpp_multirate_v1/task.py',
        'simulations/x650_cpp_multirate_v1/qualification.py','simulations/x650_cpp_multirate_v1/run_case.py',
        'simulations/x650_cpp_multirate_v1/launch.py','scripts/audit_x650_cpp_multirate.py'}
    require(required <= set(sources), 'Execution archive omits actual new timing/native implementation')
    return cfg, protocol, startup, record, proof


def check_timing_selection(live, actual_worker):
    reports = [('ingress_startup.json','isolated_ingress_startup_v1'),
        ('dispatcher_startup.json','isolated_dispatcher_startup_v1'),
        ('timing_profile.json','cpp_live_timing_selection_v1')]
    for name, kind in reports:
        value = read(live/name)
        require(value.get('kind') == kind and value.get('timing_profile') == profile.NAME,
            'Actual timing selection receipt differs: '+name)
        profile.validate(value['effective_profile'])
    require(actual_worker.get('timing_profile') == profile.NAME, 'Actual worker selected another timing profile')
    profile.validate(actual_worker['effective_profile'])
    return dict(verified=True, timing_profile=profile.NAME, effective_profile=profile.effective(),
        reports=[name for name,_ in reports])


def check_hold_guard(snapshot, config, diagnostics):
    """Check copied native hold evidence against actual history and encoding.

    This independently rebuilds the immutable/future packet schedule and query
    grid. It does not claim to re-propagate the nonlinear native model here;
    that implementation is bound by the independent native software tests.
    """
    g = diagnostics.get('hold_guard', {})
    require(all(g.get(k) is True for k in ('required','checked','passed'))
        and config['command_hold_s'] == profile.EFFECTIVE['command_hold_s'] == .16,
        'Missing complete first-command native ZOH hold guard')
    at, duration = float(snapshot.task_time_s), config['command_hold_s']
    require(same([g['start_task_s'],g['end_task_s'],g['duration_s'],g['source_control_time_s'],g['source_joint_time_s']],
        [at,at+duration,duration,snapshot.control_time_s,snapshot.joint_time_s])
        and g['actual_history_packet_count'] == snapshot.packet_count and g['substeps'] > 0,
        'Hold guard retimed source/history or shortened its declared interval')
    fixture = interval_fixture(snapshot, config, [0.]*5)
    command = numeric_array(g['command'], 2, 'hold command')
    require(same(command, np.asarray(diagnostics['plan']).reshape(config['nodes'],5)[0,3:]),
        'Hold guard validated a different first command')
    reference = np.asarray(replay.reference_now(fixture))
    require(same(g['reference_initial'],reference) and same(g['reference_final'],reference+duration*command),
        'Hold guard changed the actual integrated reference')
    lo, hi = np.asarray(config['q_lower']), np.asarray(config['q_upper'])
    require(np.all(reference >= lo-2e-7) and np.all(reference <= hi+2e-7)
        and np.all(reference+duration*command >= lo-2e-7) and np.all(reference+duration*command <= hi+2e-7),
        'Held first command would violate the reference hard limit')
    held = np.tile(np.asarray([0.,0.,0.,*command]),(config['nodes'],1))
    all_packets = interval.centre_packets(fixture, held)
    end = at+duration
    expected_packets = [p for i,p in enumerate(all_packets)
        if i<snapshot.packet_count or p['send_time']+config['motion'][4]<=end]
    require(len(g['packets']) == len(expected_packets), 'Hold guard packet history/grid count differs')
    for actual, expected in zip(g['packets'], expected_packets):
        for key in ('send_time','q_ref','dq_ref','tau_ff','kp','kd'):
            interval.close(actual['send_time_s' if key=='send_time' else key], expected[key],
                1e-12, 'held first-command packet '+key)
    # The real guard preserves original prediction-node boundaries within the
    # held interval, even though every plan row has the same command. Replacing
    # it with a one-node .16 s fixture would alter native integration pieces.
    offsets=np.cumsum([float(snapshot.dt_s)]+[config['node_dt']]*(config['nodes']-1))
    queries=sorted(set([at,end,*[at+v for v in offsets if v<duration],
        *[p['send_time']+config['motion'][4] for p in all_packets
            if at<=p['send_time']+config['motion'][4]<=end]]))
    start=at-(float(snapshot.control_time_s)-float(snapshot.joint_time_s))
    boundaries=sorted(set([start,*queries,*[p['send_time']+config['motion'][4] for p in all_packets
        if start<p['send_time']+config['motion'][4]<=end]]))
    interval.close(g['query_times'], queries, 1e-12, 'hold canonical query grid')
    interval.close(g['boundary_times'], boundaries, 1e-12, 'hold canonical boundary grid')
    states = numeric_array(g['boundary_states'],4*len(boundaries),'hold boundary states').reshape(-1,4)
    envelope = numeric_array(g['envelope'],8,'hold envelope')
    replay.check_envelope(fixture, envelope.tolist())
    require(np.all(states[:,:2] >= envelope[:2]-1e-12) and np.all(states[:,:2] <= envelope[2:4]+1e-12)
        and np.all(states[:,2:] >= envelope[4:6]-1e-12) and np.all(states[:,2:] <= envelope[6:8]+1e-12),
        'Native hold boundary observations lie outside its reported full envelope')
    expected_steps = sum(max(1,int(np.ceil(dt/config['integration_step']-1e-12)))
        for dt in np.diff(boundaries) if dt>0)
    require(g['substeps'] == expected_steps, 'Hold guard changed the original native integration grid')
    return dict(verified=True, duration_s=duration, substeps=g['substeps'], envelope=envelope.tolist(),
        scope='Copied native full envelope; independent real-history, quantization, query and reference checks; not a bound over an unknown future first-send phase')


def stopping_violation(fixture, states):
    c, inp = fixture['config'], fixture['input']
    values = np.asarray(replay.vector(states,4*c['nodes'])).reshape(c['nodes'],4)
    age = inp['state']['control_time']-inp['state']['joint_stamp']
    require(0<=age<=.025+1e-10, 'New recovery joint age differs')
    reserve = np.asarray(c['velocity_limit'])[3:]/(2*np.asarray(c['acceleration_limit'])[3:])
    reserve += c['max_dt']+c['command_hold_s']+age
    stopped = values[:,:2]+reserve*values[:,2:]
    return float(max(0.,np.max(np.asarray(c['q_lower'])-stopped),np.max(stopped-np.asarray(c['q_upper']))))



def pva_values(records):
    return np.column_stack([*[fields(records,'sent_'+part+'_target_')
        for part in ('position','velocity','acceleration')],
        np.asarray([float(row['sent_yaw_ned']) for row in records])])


def check_handoff_bridge(control, derived, start, ulog):
    """Prove the actual prestream PVA held during first-candidate startup.

    No legacy row is relabeled as MPC or moved to another time. The original
    received PVA must match real prestream sends over the complete prefix.
    """
    origin=float(start['origin_s'])
    require(derived and all(row['phase']=='measure' for row in derived), 'Missing actual C++ publication view')
    first=float(derived[0]['sim_s'])
    require(origin<=first<=origin+PVA_DELIVERY_MAX_S+1e-9,
        'Initial handoff is outside the existing bounded PVA context')
    prestream=[row for row in control if row['phase']!='measure'
        and row.get('pva_sent') in ('True','true','1') and float(row['sim_s'])<=origin+1e-9]
    require(prestream, 'No real prestream PVA covers the handoff boundary')
    legacy=max(prestream,key=lambda row:float(row['sim_s']))
    require(abs(float(legacy['sim_s'])-origin)<=1e-9 and start['history_count']>0
        and abs(start['history_time_s'][start['history_count']-1]-origin)<=1e-9,
        'LiveStart is not the original last prestream send boundary')
    require(all(int(float(row['sent_coordinate_frame']))==1 and int(float(row['sent_type_mask']))==2048
        for row in prestream), 'Initial bridge lacks complete original local-NED PVA')
    sent_times=np.asarray([float(row['sim_s']) for row in prestream]);sent=pva_values(prestream)
    require(np.isfinite(sent_times).all() and np.isfinite(sent).all()
        and np.all(np.diff(sent_times)>0), 'Original prestream PVA times/values are malformed')
    rt, received=trajectory_matrix(ulog)
    rt=np.asarray(rt,float);received=np.asarray(received,float)
    require(rt.ndim==1 and received.shape==(len(rt),10) and np.isfinite(rt).all()
        and np.all(np.diff(rt)>=0), 'Malformed original received PVA evidence')
    first_value=pva_values([derived[0]])[0]
    candidates=np.flatnonzero((rt>=first-CLOCK_BOUNDARY_S-1e-9)&(rt<=first+PVA_DELIVERY_MAX_S+1e-9))
    matches=candidates[np.max(abs(received[candidates]-first_value),axis=1)<=PVA_FLOAT_TOLERANCE]
    require(len(matches)>0, 'First actual C++ PVA is absent from the bounded original ULog')
    first_received=float(rt[matches[0]])
    # Include a real received context row at/before the handoff. A late first
    # measurement alone is not evidence over the unobserved startup prefix.
    context=np.flatnonzero(rt<=origin+1e-9)
    require(len(context)>0 and origin-rt[context[-1]]<=PVA_DELIVERY_MAX_S+1e-9,
        'ULog has no bounded received PVA context at the handoff origin')
    indices=np.flatnonzero((np.arange(len(rt))>=context[-1])&(rt<first_received-1e-9))
    require(len(indices)>0 and np.isfinite(received[indices]).all()
        and first_received-rt[indices[-1]]<=PVA_DELIVERY_MAX_S+1e-9
        and np.diff(rt[np.r_[indices,matches[0]]]).max()<=PVA_DELIVERY_MAX_S+1e-9,
        'Received PVA has an uncovered initial handoff prefix')
    for index in indices:
        choices=np.flatnonzero((sent_times<=rt[index]+CLOCK_BOUNDARY_S+1e-9)
            &(sent_times>=rt[index]-PVA_DELIVERY_MAX_S-1e-9))
        require(len(choices)>0 and np.any(np.max(abs(sent[choices]-received[index]),axis=1)<=PVA_FLOAT_TOLERANCE),
            'Initial received held PVA does not match a real bounded-causal prestream send')
    value=pva_values([legacy])[0]
    observed=np.flatnonzero((rt>=origin-CLOCK_BOUNDARY_S-1e-9)&(rt<=first_received+1e-9)
        &(np.max(abs(received-value),axis=1)<=PVA_FLOAT_TOLERANCE))
    require(len(observed)>0, 'Last actual prestream PVA was never observed during handoff')
    return dict(verified=True,origin_s=origin,first_cpp_writer_observation_sim_s=first,
        first_cpp_value_received_s=first_received,initial_writer_observation_span_s=first-origin,
        initial_received_value_span_s=first_received-origin,
        last_legacy_actual_send_sim_s=float(legacy['sim_s']),last_legacy_pva_received_s=float(rt[observed[0]]),
        initial_received_context_sim_s=float(rt[indices[0]]),prestream_received_context_rows=len(indices),
        legacy_rows_relabelled_as_mpc=False,timestamps_changed=False,
        scope='Original prestream PVA and actual bounded-causal received held values bridge startup; C++ starts at its separately reported first output/value observation')


def selected_rows(control, window, *, handoff):
    selected=[row for row in control if row['phase']=='measure'
        and window[0]-1e-9<=float(row['sim_s'])<=window[1]+1e-9]
    times=np.asarray([float(row['sim_s']) for row in selected])
    require(len(times)>=2 and np.isfinite(times).all() and np.all(np.diff(times)>0)
        and times[-1]>=window[1]-.05 and np.diff(times).max()<=.05+1e-9,
        'Actual C++ PVA view has an internal gap or incomplete final coverage')
    require(handoff.get('verified') is True and handoff.get('legacy_rows_relabelled_as_mpc') is False
        and handoff.get('timestamps_changed') is False and abs(handoff['origin_s']-window[0])<=1e-9
        and abs(handoff['first_cpp_writer_observation_sim_s']-times[0])<=1e-9,
        'Full-window initial coverage lacks the explicit original prestream bridge')
    require(np.max(abs(times-np.asarray([float(row['t_s']) for row in selected])-window[0]))<=1e-8,
        'Upper task/simulation time origin changed')
    return selected,times


# The worker/tick/full-window checks below preserve the original audit operations.
# Their changed time values are explicit; legacy auditor modules remain untouched.

def upper_commands(ulog, native, reference, control, window, pid, *, response, handoff):
    selected, times = selected_rows(control, window, handoff=handoff)
    require(all(row.get("pva_sent") in ("True", "true", "1") and int(float(row["sent_coordinate_frame"])) == 1
                and int(float(row["sent_type_mask"])) == 2048 for row in selected), "Upper stage must actually send complete local-NED PVA")
    p, v, a = (fields(selected, "sent_"+part+"_target_") for part in ("position", "velocity", "acceleration"))
    yaw = np.asarray([float(row["sent_yaw_ned"]) for row in selected])
    require(np.isfinite(yaw).all() and np.max(abs(yaw-float(reference["fixed_yaw_ned_rad"]))) <= 1e-9,
            "Upper commands changed the bound fixed heading")
    fb = fields(selected, "feedback_position_")
    logical = fields(selected, "base_velocity_target_", "xyz")
    gains = np.array([pid["MPC_XY_P"], pid["MPC_XY_P"], pid["MPC_Z_P"]])
    reconstructed = gains*(p-fb)+v
    require(np.max(abs(reconstructed-logical@NED_TO_ENU.T)) <= 1e-8,
            "Sent PVA does not reconstruct logical velocity from the same feedback snapshot")
    require(np.max(abs(a-fields(selected, "command_acceleration_", "xyz")@NED_TO_ENU.T)) <= 1e-9,
            "Sent acceleration differs from the logged world command")
    if response:
        require(np.max(abs(a)) <= 1e-12, "Response identification requires zero acceleration feedforward")
        from simulations.x650_wholebody_mpc_v1.run_case import response_probe_velocity
        excited = np.asarray([response_probe_velocity(float(row["t_s"]), 30.) for row in selected])
        active = np.asarray([float(row["t_s"]) < 14. for row in selected])
        require(np.max(abs(logical[active]-excited[active])) <= 1e-9, "Response input differs from frozen world-axis pulse schedule")
        returning = np.clip(1.5*(np.asarray(reference["target_world_enu_m"])-fields(selected, "feedback_p", "xyz")), -.12, .12)
        require(np.max(abs(logical[~active]-returning[~active])) <= 1e-9,
                "Response return segment does not hold its declared fixed world anchor")
    all_sent = [row for row in control if row.get("pva_sent") in ("True", "true", "1")]
    sent_t = np.asarray([float(row["sim_s"]) for row in all_sent])
    require(np.isfinite(sent_t).all() and np.all(np.diff(sent_t) > 0), "Actual send times must be strictly increasing")
    sent = np.column_stack([*[fields(all_sent, "sent_"+part+"_target_") for part in ("position", "velocity", "acceleration")],
                            np.asarray([float(row["sent_yaw_ned"]) for row in all_sent])])
    rt, received = trajectory_matrix(ulog)
    mask = (rt >= window[0]-1e-9)&(rt <= window[1]+1e-9)
    require(np.isfinite(received[mask]).all(), "Measurement received PVA cannot contain ignored/nonfinite fields")
    require(mask.sum() >= 2 and np.diff(rt[mask]).max() <= PVA_DELIVERY_MAX_S+1e-9
            and rt[mask][0] <= window[0]+PVA_DELIVERY_MAX_S and rt[mask][-1] >= window[1]-PVA_DELIVERY_MAX_S,
            "Received trajectory setpoints do not cover complete stage")
    delays = []
    for receive_time, value in zip(rt[mask], received[mask]):
        candidates = np.flatnonzero((sent_t <= receive_time+CLOCK_BOUNDARY_S+1e-9)
                                   &(sent_t >= receive_time-PVA_DELIVERY_MAX_S-1e-9))
        matches = candidates[np.max(abs(sent[candidates]-value), axis=1) <= PVA_FLOAT_TOLERANCE] if len(candidates) else []
        require(len(matches) > 0, "A received PVA has no matching bounded-causal actual send")
        delays.append(float(receive_time-sent_t[matches[-1]]))
    for send_time, value in zip(times, np.column_stack((p, v, a, yaw))):
        candidates = np.flatnonzero((rt >= send_time-CLOCK_BOUNDARY_S-1e-9)&(rt <= send_time+PVA_DELIVERY_MAX_S+1e-9))
        # NAV_TAKEOFF/LAND outside the measurement can legitimately carry
        # ignored NaNs. Such rows never count as a matching complete PVA.
        require(len(candidates) and np.any(np.max(abs(received[candidates]-value), axis=1) <= PVA_FLOAT_TOLERANCE),
                "A measurement PVA send is absent from the received trajectory log")
    internal = precision.dataset(ulog, "vehicle_local_position_setpoint")
    it = precision.stamp(internal)
    imask = (it >= window[0]-1e-9)&(it <= window[1]+1e-9)
    query = it[imask]
    iv = precision.fields(internal, ("vx", "vy", "vz"))[imask]
    # ULog may contain repeated publication stamps; retain the final held value
    # exactly, without averaging, moving time, or creating extra observations.
    unique = np.r_[np.diff(query) > 0, True]
    query, iv = query[unique], iv[unique]
    require(len(query) >= 2 and query[0] <= window[0]+.05 and query[-1] >= window[1]-.05
            and np.diff(query).max() <= .05+1e-9, "Internal PX4 velocity targets lack complete coverage")
    # Retain bounded causal context rather than discarding a real held target
    # at the window boundary. resample validates the actual held values only;
    # a queried pre-window NAV NaN remains an error and is never filled in.
    context = rt >= window[0]-PVA_DELIVERY_MAX_S-1e-9
    held = precision.resample(rt[context], received[context], query, PVA_DELIVERY_MAX_S, hold=True)
    lpos = precision.dataset(ulog, "vehicle_local_position")
    estimate_p = precision.resample(precision.stamp(lpos), precision.fields(lpos, ("x", "y", "z")), query, .024)
    received_u = gains*(held[:, :3]-estimate_p)+held[:, 3:6]
    reconstructed_error = (iv-received_u)@NED_TO_ENU.T
    reconstructed_metrics = precision.metrics(query, reconstructed_error, "m/s")
    logical_at = precision.resample(sent_t, fields(all_sent, "base_velocity_target_", "xyz"), query, .10, hold=True)
    logical_metrics = precision.metrics(query, iv@NED_TO_ENU.T-logical_at, "m/s")
    require(np.all(np.asarray(reconstructed_metrics["axis_rmse"]) <= PROXY_AXIS_RMS_LIMIT),
            "Internal PX4 target differs from received-PVA plus same-time estimated position reconstruction")
    require(np.all(np.asarray(logical_metrics["axis_rmse"]) <= PROXY_AXIS_RMS_LIMIT),
            "Internal PX4 target differs materially from original-time logical velocity, including real delivery delay")
    limits_respected = bool(np.all(np.linalg.norm(received_u[:, :2], axis=1) <= pid["MPC_XY_VEL_MAX"]+1e-6)
        and np.all(received_u[:, 2] >= -pid["MPC_Z_VEL_MAX_UP"]-1e-6)
        and np.all(received_u[:, 2] <= pid["MPC_Z_VEL_MAX_DN"]+1e-6))
    require(limits_respected, "Reconstructed received velocity exceeds recorded PX4 speed limits")
    return {"verified": True, "control_interface": "raw_ev_world_pva_v2", "measurement_sends": len(selected),
        "received_rows": int(mask.sum()), "source_snapshot_identity_max_abs_m_s": float(np.max(abs(reconstructed-logical@NED_TO_ENU.T))),
        "received_pva_matches_actual_sends": True, "all_measurement_sends_observed": True,
        "causal_delivery_allowance_s": PVA_DELIVERY_MAX_S, "clock_boundary_allowance_s": CLOCK_BOUNDARY_S,
        "pva_float_tolerance": PVA_FLOAT_TOLERANCE, "matched_delivery_range_s": [min(delays), max(delays)],
        "internal_minus_received_pva_reconstruction": reconstructed_metrics,
        "internal_minus_original_time_logical_velocity": logical_metrics,
        "axis_rms_acceptance_limit_m_s": PROXY_AXIS_RMS_LIMIT.tolist(),
        "reconstructed_velocity_within_actual_px4_limits": limits_respected,
        "saturation_scope": "Speed-bound check and internal/reconstructed residuals; no unsupported claim that every inner-loop actuator avoided saturation",
        "zero_acceleration_feedforward": bool(np.max(abs(a)) <= 1e-12),
        "time_alignment": "Original send simulation timestamps and raw ULog HRT; no estimated time shift or delay compensation"}

def check_worker(live, start, config, events, resources, protocol, *, decode=None):
    if decode is None:
        from rclpy.serialization import deserialize_message
        from flyarm_control_interfaces.msg import ControlSnapshot, ControlProposal
        decode = lambda raw, proposal: deserialize_message(raw, ControlProposal if proposal else ControlSnapshot)
    final, sender, timing = (read(live/name) for name in ('worker_final.json','diagnostic_sender.json','worker_timing.json'))
    files = sorted((live/'native_records').glob('*.json'))
    require(files and final['implementation'] == 'cpp_full_pipeline_v1' and final['live_started'] is True
        and final['faulted'] is False and final['last_error'] == ''
        and final['requests'] == final['accepted'] == len(files), 'Native worker did not complete cleanly')
    require(sender['complete'] is True and sender['failed'] is False and sender['dropped'] == 0
        and sender['offered'] == sender['written'] == len(files), 'Lost actual Core evidence')
    columns = timing['columns']
    require(timing['kind'] == 'cpp_worker_phase_timing_v1' and timing['complete'] is True and timing['dropped'] == 0
        and len(columns) == len(set(columns)) and len(timing['rows']) == len(files)
        and all(len(r)==len(columns) for r in timing['rows']), 'Incomplete bounded native phase evidence')
    phase = [dict(zip(columns,r)) for r in timing['rows']]
    require(len(events) <= len(files) <= len(events)+1 and all(e['accepted'] is True for e in events)
        and [e['sequence'] for e in events] == list(range(1,len(events)+1))
        and [e['proposal_sequence'] for e in events] == [e['snapshot_sequence'] for e in events]
        == list(range(1,len(events)+1)), 'Native candidate/gate prefix differs')
    affinity = read(live/'worker_affinity_final.json')
    ready = read(live/'worker_affinity_ready.json')
    executor = protocol['worker_thread_roles']['executor']
    auxiliary = protocol['worker_thread_roles']['auxiliary']
    require(len(executor)==1 and sorted(executor+auxiliary)==resources['roles']['worker']
        and affinity['executor_cpu']==executor[0] and affinity['auxiliary_cpus']==auxiliary
        and affinity['stages'][:1]==ready['stages'] and [v['stage'] for v in affinity['stages']]==['ready','live_start','final'],
        'Worker thread placement declaration/stages differ')
    identity_set = None
    for stage in affinity['stages']:
        current={(r['tid'],r['starttime_ticks']) for r in stage['threads']}
        require(len(current)==len(stage['threads']) and (identity_set is None or current==identity_set), 'Worker TIDs changed')
        identity_set=current
        require(all(r['cpus']==(executor if r['tid']==affinity['main_tid'] else auxiliary) for r in stage['threads']),
                'Worker stage CPU masks differ')
    calls=[]; stats={name:[] for name in ('core_ms','callback_ms','thread_cpu_ms','publish_ms','record_after_publish_ms','gate_roundtrip_ms')}
    previous = np.asarray(start['history_command'][(start['history_count']-1)*5:start['history_count']*5])
    for seq,(path,t) in enumerate(zip(files,phase),1):
        value=read(path)
        for name in ('snapshot_cdr','proposal_cdr'):
            require(identity(value[name]['path'])==value[name], 'Native raw CDR bytes changed')
        s=decode(Path(value['snapshot_cdr']['path']).read_bytes(),False)
        p=decode(Path(value['proposal_cdr']['path']).read_bytes(),True)
        require(s.sequence==p.sequence==p.snapshot_sequence==seq==t['sequence']
            and s.session_id==p.session_id==start['session_id'] and s.epoch==p.epoch==start['epoch']
            and bytes(s.configuration_sha256)==bytes(p.configuration_sha256)==bytes(start['configuration_sha256'])
            and bytes(s.configuration_sha256).hex()==final['configuration_sha256']
            and p.status==0 and p.accepted_optimum and p.nonlinear_check_passed and not p.replay_only,
            'Native actual candidate identity/status differs')
        require(t['snapshot_issued_ns']==s.issued_monotonic_ns and t['callback_entry_ns']==p.compute_start_monotonic_ns,
                'Native phase/CDR timestamp differs')
        stamps=[s.issued_monotonic_ns,t['callback_entry_ns'],t['core_begin_ns'],t['core_end_ns'],p.compute_end_monotonic_ns,
                t['publish_begin_ns'],t['publish_end_ns'],t['record_offer_end_ns'],t['callback_exit_ns']]
        require(all(type(v) is int and v>0 for v in stamps) and all(b>=a for a,b in zip(stamps,stamps[1:]))
            and t['record_offered']==1 and t['thread_cpu_end_ns']>=t['thread_cpu_begin_ns']
            and t['tid_begin']==t['tid_end']==affinity['main_tid'] and t['cpu_begin']==t['cpu_end']==executor[0],
            'Native phase/CPU identity invalid')
        require(0<s.dt_s<=.1+1e-10 and 0<=s.control_time_s-s.joint_time_s<=.025+1e-10
            and p.compute_end_monotonic_ns-p.compute_start_monotonic_ns<=50_000_000,
            'Original native interval/joint freshness/computation deadline exceeded')
        if seq<=len(events):
            e=events[seq-1]
            require(e['session_id']==s.session_id and e['epoch']==s.epoch
                and e['received_monotonic_ns']>=t['publish_begin_ns']
                and e['roundtrip_ns']==e['received_monotonic_ns']-s.issued_monotonic_ns
                and e['roundtrip_ns']<=50_000_000, 'Gate altered source age or accepted a stale candidate')
            stats['gate_roundtrip_ms'].append(e['roundtrip_ns']/1e6)
        previous=check_numeric_result(value,s,p,config,previous)
        check_hold_guard(s,config,value['result']['diagnostics'])
        for name,a,b in [('core_ms','core_begin_ns','core_end_ns'),('callback_ms','callback_entry_ns','callback_exit_ns'),
                         ('thread_cpu_ms','thread_cpu_begin_ns','thread_cpu_end_ns'),('publish_ms','publish_begin_ns','publish_end_ns'),
                         ('record_after_publish_ms','publish_end_ns','callback_exit_ns')]: stats[name].append((t[b]-t[a])/1e6)
        calls.append((s,p,value))
    # Admission source stamps and actual-send releases are independent of worker
    # phase observations; never infer them from a later accepted proposal.
    ingress=read(live/'ingress_timing.json'); names=ingress['columns']
    require(ingress['complete'] is True and ingress['overflow']==0 and len(names)==len(set(names))
        and all(len(r)==len(names) for r in ingress['rows']), 'Incomplete ingress timing')
    admissions=[dict(zip(names,r)) for r in ingress['rows'] if r[names.index('kind')]==4 and r[names.index('forward')]]
    by_seq={r['sequence']:r for r in admissions}
    require(len(by_seq)==len(admissions), 'Duplicate admission')
    for s,_,_ in calls:
        require(s.sequence in by_seq, 'Candidate has no actual ingress admission')
        a=by_seq[s.sequence]
        require(a['source_issued_ns']==s.issued_monotonic_ns and a['latest_control_s']==s.control_time_s
            and abs(a['latest_control_s']-a['last_control_s']-s.dt_s)<=1e-10
            and (s.sequence==1 or a['released_sequence']==a['last_admitted_sequence']==s.sequence-1),
            'Admission source/dt/actual-send barrier differs')
    ordered=[by_seq[int(s.sequence)] for s,_,_ in calls]
    require(all(b['entry_ns']-a['entry_ns']>=40_000_000 for a,b in zip(ordered,ordered[1:])),
        'New upper admission ran faster than its explicit 40 ms minimum')
    return calls, dict(verified=True, calls=len(calls), no_lost_diagnostics=True, executor_cpu=executor[0],
        numerical_scope='Actual owning outputs checked against original wire; frozen independent software proofs separately reverified',
        durations={k:summary(v) for k,v in stats.items()})

def check_ticks(ticks, sends, calls, start, *, convert=None, source_issues=None, source_controls=None):
    if convert is None:
        from rosidl_runtime_py.convert import message_to_ordereddict
        convert = message_to_ordereddict
    require([r['sequence'] for r in ticks]==list(range(1,len(ticks)+1))
        and all(r['session_id']==start['session_id'] and r['epoch']==start['epoch'] for r in ticks)
        and not any(r['status']==3 for r in ticks), 'Dispatch identity/gap/fault')
    valid=[r for r in ticks if r['output_valid']]; normal=[r for r in sends if not r['stop_zero']]
    require(len(valid)>1 and [v['sequence'] for v in valid]==[v['dispatch_sequence'] for v in normal],
            'Actual send/valid tick identity differs')
    require(isinstance(source_controls,dict) and source_controls,
        'Physical hold requires the original tick-referenced C++ source control times')
    by_seq={int(s.sequence):(s,p) for s,p,_ in calls}
    physical_ages=[];writer_observation_ages=[]
    for t,s in zip(valid,normal):
        require(t['command_sequence'] in by_seq,'Output lacks original candidate')
        source,p=by_seq[t['command_sequence']]
        require(t['command_issued_monotonic_ns']==source.issued_monotonic_ns
            and t['command_age_ns']==t['actual_monotonic_ns']-source.issued_monotonic_ns
            and 0<=t['command_age_ns']<=160_000_000
            and 0<=t['actual_monotonic_ns']-t['source_received_monotonic_ns']<=25_000_000
            and 0<=s['sent_monotonic_ns']-source.issued_monotonic_ns<=160_000_000
            and t['command_sequence']==s['command_sequence'] and t['repeated']==s['repeated'],
            'Actual output age/source/candidate differs')
        require(0<=t['actual_monotonic_ns']-t['scheduled_monotonic_ns']<=20_000_000
            and s['sent_monotonic_ns']>=t['actual_monotonic_ns'], 'Actual dispatch missed its original absolute slot')
        if source_issues is not None:
            require(t['source_sequence'] in source_issues, 'Tick source absent from raw C++ evidence')
            issued=source_issues[t['source_sequence']]
            require(0<issued<=t['source_received_monotonic_ns']
                and 0<=t['actual_monotonic_ns']-issued<=25_000_000, 'Original source issue age exceeded 25 ms')
        require(t['source_sequence'] in source_controls,
            'Tick physical clock is absent from original source evidence')
        current_control=source_controls[t['source_sequence']]
        require(type(current_control) in (int,float) and np.isfinite(current_control),
            'Malformed original source control time')
        physical_age=current_control-source.control_time_s
        require(0<=physical_age<=.16+1e-10,
            'Actual repeated command exceeded its independently guarded physical hold')
        physical_ages.append(physical_age*1000.)
        # ActuatorSend.sim_s is the writer's latest ActuatorState observation.
        # Its separate callback can lag the controller's causal clock. Keep it
        # as an observation, never substitute it for the tick's physical clock.
        writer_observation_ages.append((s['sim_s']-source.control_time_s)*1000.)
        # Re-encoding the same proposal cannot create a new lease.
        require(t['command']==convert(p), 'Dispatch candidate differs from actual worker CDR')
    gaps=np.diff([s['sent_monotonic_ns'] for s in normal])/1e6
    require(np.all(gaps>0) and np.max(gaps)<=40., 'Actual send gap exceeded original 40 ms bound')
    due=np.diff([t['scheduled_monotonic_ns'] for t in valid])
    require(np.all(due==20_000_000), 'Absolute output slots skipped or phase rewritten')
    return dict(verified=True, actual_send_gap_ms=summary(gaps), command_age_ms=summary([v['command_age_ns']/1e6 for v in valid]),
        physical_hold_age_ms=summary(physical_ages),
        physical_hold_clock='Original raw C++ source control_s at the tick source_sequence; same clock as candidate control_time_s',
        writer_observation_to_candidate_ms=summary(writer_observation_ages),
        writer_observation_precedes_candidate=sum(v<0 for v in writer_observation_ages),
        writer_observation_scope='ActuatorSend.sim_s is a latest observed actuator state time, not physical time of publication',
        original_absolute_period_ns=20_000_000, normal_sends=len(normal))

def check_runtime_recoveries(calls, start, config):
    """Independently check compact native recovery evidence from original CDRs.

    Compact mode intentionally omits the full QP matrix and propagation stream.
    This checks its copied node/envelope/reference evidence, while the exact
    implementation and full-matrix replay remain bound by software admission.
    No prediction or optimizer is rerun and no missing matrix is reconstructed.
    """
    require(calls, 'No actual native calls available for recovery evidence')
    previous = list(start['history_command'][(start['history_count']-1)*5:start['history_count']*5])
    recovered, normal, total_projection = [], 0, 0
    for snapshot, proposal, value in calls:
        d = value['result']['diagnostics']; triggered = replay.bool_field(d, 'recovery_triggered')
        attempts = d['recovery_attempts']
        require(isinstance(attempts,list), 'Malformed actual recovery centre evidence')
        if not triggered:
            require(attempts == [] and d['recovery_reason'] == '' and d['recovery_trust_radius'] == 0.
                and d['recovery_stopping_violation'] == 0., 'Normal call carries untriggered recovery evidence')
            normal += 1
        else:
            fixture = interval_fixture(snapshot, config, previous)
            require(d['recovery_reason'] in ('QP infeasible','constant QP infeasible')
                and replay.number(d['recovery_trust_radius']) == .02
                and min(replay.number(config['trust_radius']),.02) == .02,
                'Actual recovery changed its trigger or original initial radius')
            require(1 <= len(attempts) <= 2 and 2 <= d['subproblems'] <= 3,
                    'Actual recovery centre/control QP budget differs')
            require([a['centre_kind'] for a in attempts] ==
                ['slew_limited_brake','original_constraints_brake_projection'][:len(attempts)],
                'Actual recovery centre order differs')
            checked, projections = [], 0
            for a in attempts:
                kind = a['centre_kind']
                require(kind in ('slew_limited_brake','original_constraints_brake_projection'), 'Unknown recovery centre')
                projected = replay.bool_field(a,'projection_attempted')
                solved = replay.bool_field(a,'projection_solved')
                require(not solved or projected, 'Projection solved without an actual attempt')
                require(not projected or kind == 'original_constraints_brake_projection', 'Brake falsely declares projection')
                projections += projected
                require(0 <= replay.number(a['elapsed_ms']) <= replay.number(d['compute_ms']) <= 50.,
                        'Recovery centre clock exceeds original call/deadline')
                if not replay.bool_field(a,'trusted'):
                    require(isinstance(a['rejection'],str) and a['rejection'], 'Rejected centre lacks actual failure reason')
                    continue
                require(a['rejection'] == '' and replay.bool_field(a,'nonlinear_passed')
                    and replay.bool_field(a,'original_constraints_passed') and a['substeps'] > 0,
                    'Unvalidated recovery centre was trusted')
                require(kind != 'original_constraints_brake_projection' or (projected and solved),
                        'Trusted projected centre lacks an actual solved projection')
                check_hold_guard(snapshot,config,dict(d,hold_guard=a['hold_guard'],plan=a['centre_plan']))
                times = replay.vector(a['node_times'], config['nodes'])
                require(max(abs(x-y) for x,y in zip(times,replay.node_times(fixture))) <= 1e-9,
                        'Trusted centre horizon differs from actual snapshot')
                slew = replay.check_plan(fixture,a['centre_plan'],a['node_references'])
                envelope = replay.check_envelope(fixture,a['envelope'])
                stopping = stopping_violation(fixture,a['node_states'])
                tolerance = replay.number(config['feasibility_tolerance'])
                require(stopping <= tolerance and 0 <= replay.number(a['original_primal_violation']) <= tolerance
                    and abs(stopping-replay.number(a['stopping_violation'])) <= 1e-12,
                    'Trusted centre violates original nonlinear stopping reserve')
                checked.append(dict(kind=kind,plan_violation=slew,envelope_violation=envelope,
                    nonlinear_stopping_violation=stopping,elapsed_ms=a['elapsed_ms']))
            require(checked and projections <= 1 and d['subproblems']+projections <= 4,
                    'No trusted centre or original total QP budget exceeded')
            states = replay.vector(d['predicted_states'], 12*config['nodes'])
            nonlinear_nodes = [states[12*i+j] for i in range(config['nodes']) for j in (6,7,8,9)]
            stopping = stopping_violation(fixture,nonlinear_nodes)
            require(stopping <= config['feasibility_tolerance']
                and abs(stopping-replay.number(d['recovery_stopping_violation'])) <= 1e-12,
                'Final actual nonlinear candidate violates original stopping reserve')
            total_projection += projections
            recovered.append(dict(sequence=int(snapshot.sequence),task_time_s=float(snapshot.task_time_s),
                centre_attempts=len(attempts),trusted_centres=checked,projection_solves=projections,
                control_subproblems=d['subproblems'],final_nonlinear_stopping_violation=stopping,
                core_ms=d['compute_ms']))
        previous = list(value['result']['command'])
    return dict(verified=True,normal_calls=normal,recovery_calls=len(recovered),projection_solves=total_projection,
        calls=recovered,initial_trust_radius=.02,core_budget_ms=50.,
        recovery_core_ms=summary([c['core_ms'] for c in recovered]) if recovered else None,
        compact_capture_scope='Copied actual native centre/final nodes, extrema and integrated reference; full QP rows and model rollout are independently bound by software admission, not recaptured/reconstructed here')

def audit(launch, run, out):
    report=dict(kind='x650_cpp_multirate_tip_hold_certificate_v1',timing_profile=profile.NAME,effective_profile=profile.effective(),passed=False,complete=False,
        certificate=False,physical_acceptance=False,hardware_qualified=False,launch_directory=str(launch),run_directory=str(run),
        auditor_version='multirate_physical_v3',auditor_identity=identity(Path(__file__)))
    artifacts=[]
    try:
        # Even a failed audit retains the hashes of its exact original evidence.
        paths={p for base in (launch,run) for p in base.rglob('*') if p.is_file()}
        paths.update(ROOT/name for name in ('scripts/audit_x650_cpp_hold.py','scripts/report_x650_cpp_live.py',
            'scripts/audit_x650_cpp_source.py','scripts/audit_x650_streaming_consumption.py',
            'scripts/audit_x650_isolated_live.py','scripts/audit_x650_precision_stage.py','scripts/audit_x650_upper_stage.py',
            'scripts/analyze_x650_hover_precision.py','simulations/x650_cpp_live_v1/qualification.py',
            'simulations/x650_ts20_hold_v1/audit.py','simulations/x650_ts20_hold_v1/task.py'))
        paths.update(ROOT/name for name in ('scripts/audit_x650_cpp_multirate.py','scripts/audit_x650_interval_robustness.py','scripts/audit_x650_recovery_robustness.py','scripts/audit_x650_recovery_replay.py','scripts/audit_x650_interval_constraints.py'))
        paths.update((ROOT/'simulations/x650_cpp_multirate_v1').glob('*.py'))
        artifacts=[identity(p) for p in sorted(paths)]
        meta=read(run/'controller_metadata.json')
        require(all(meta.get(k) is True for k in ('complete','landed_and_disarmed','diagnostic_parameters_restored'))
            and meta.get('diagnostic_restore_pending') is False, 'Incomplete flight lifecycle: '+str(meta.get('error')))
        stage=read(run/'stage_result.json')
        require(all(stage.get(k) is True for k in ('complete','passed','success','landed_and_disarmed'))
            and read(run/'result.json').get('success') is True, 'Original provisional stage/run failed')
        cfg,protocol,startup,launch_record,proof=check_archives(launch,run,meta)
        report['software_qualification']=proof
        wrench=cfg['multirate']['case_id']=='wrench_fast'
        duration=cfg['duration_s']
        report['kind']='x650_cpp_multirate_wrench_certificate_v1' if wrench else 'x650_cpp_multirate_tip_hold_certificate_v1'
        report.update(task_config=identity(launch/'task.json'),startup_manifest=identity(launch/'startup_manifest.json'),runtime_binaries_sha256=launch_record['runtime_binaries_sha256'])
        if wrench:report['physical_model']=check_physical_model(cfg,meta,stage,proof)
        live=run/'isolated_live'; lifecycle=read(live/'lifecycle.json'); start=read(live/'live_start.json')
        require(lifecycle['started'] is True and lifecycle['error'] is None
            and all(p['returncode']==0 for p in lifecycle['processes'].values()), 'Native child lifecycle failed')
        require(start['history_count']>0 and abs(meta['task_end_sim_s']-start['origin_s']-duration)<=.05,
                'Incomplete full 30 s task')
        window=[start['origin_s'],start['origin_s']+duration]
        import audit_x650_isolated_live as old
        report['ground_initialization']=old.check_preflight_transport(live,meta,start)
        report['fault_supervision']=old.check_fault_supervision(live,start)
        resources=read(live/'execution_resources.json')
        require(resources==launch_record['execution_resources'] and resources['roles']==protocol['cpu_roles'], 'Actual resources changed')
        observed=read(live/'execution_affinity_at_handoff.json')
        require(set(observed)=={'source',*lifecycle['processes']}, 'Incomplete process placement observation')
        for name,threads in observed.items():
            if name=='worker':continue
            require(threads and all(mask==resources['roles'].get(name,resources['background']) for mask in threads.values()),
                    'Handoff CPU partition changed: '+name)
        actual_startup=read(live/'worker_startup.json')
        report['timing_selection']=check_timing_selection(live,actual_startup)
        require(actual_startup.get('core_abi')==4 and actual_startup['startup_manifest']==startup and actual_startup['python_in_worker'] is False
            and actual_startup['startup_controller_steps']==0 and actual_startup['diagnostic_mode']=='compact'
            and actual_startup['session_id']==start['session_id'] and actual_startup['epoch']==start['epoch']
            and actual_startup['configuration_sha256']==qualification.sha(launch/'task.json'), 'Actual worker startup differs')
        for field,loaded in [('core_library','loaded_core_path'),('optimization_library','loaded_optimization_path'),('predictor_library','loaded_predictor_path')]:
            require(str(Path(startup[field]['path']).resolve())==actual_startup[loaded], 'Actually loaded native library differs')
        import audit_x650_streaming_consumption as streaming
        messages=streaming.original_messages(live)
        sends,receipts,ticks,events=(messages[k] for k in ('sends','receipts','dispatch','events'))
        calls,report['worker']=check_worker(live,start,startup['config'],events,resources,protocol)
        report['optimizer_recovery']=check_runtime_recoveries(calls,start,startup['config'])
        report['optimizer_interval']=check_runtime_intervals(calls,start,startup['config'])
        report['first_command_hold_guard']=dict(verified=True,calls_checked=len(calls),duration_s=.16)
        report['actual_send_barrier']=check_actual_send_barrier(read(live/'ingress_timing.json'),sends,calls)
        source_rows=rows(live/'cpp_source/source_selections.csv')
        source_issues={int(v['sequence']):int(v['issued_ns']) for v in source_rows}
        source_controls={int(v['sequence']):float(v['control_s']) for v in source_rows}
        require(len(source_issues)==len(source_rows), 'Repeated source issue identity')
        report['timing']=check_ticks(ticks,sends,calls,start,source_issues=source_issues,source_controls=source_controls)
        publications=[json.loads(line) for line in (run/'control_execution_trace.jsonl').open() if line.strip()]
        report['consumption']=streaming.audit_consumption(sends,receipts,rows(run/'ts20_state.csv'),start,lifecycle['release'],publications)
        require(report['consumption']['passed'] is True, 'Asynchronous physical consumption accounting failed')
        from flyarm_control_interfaces.msg import ActuatorState
        from rclpy.serialization import deserialize_message
        from rosidl_runtime_py.convert import message_to_ordereddict
        baseline=message_to_ordereddict(deserialize_message((live/'actuator_epoch_state.cdr').read_bytes(),ActuatorState))
        report['consumption']['online_ledger']=streaming.audit_async_ledger(read(live/'actuator_ledger.json'),baseline,
            sends,receipts,rows(run/'ts20_state.csv'),start,lifecycle['release'])
        require(report['consumption']['online_ledger']['passed'] is True, 'Online asynchronous progress evidence failed')
        if wrench:
            from audit_x650_cpp_fast_source import audit_source
        else:
            from audit_x650_cpp_source import audit_source
        reference=read(run/'hover_reference.json')
        report['feedback']=audit_source(live/'cpp_source',run/'ts20_state.csv',run/'ts20_packets.csv',reference,
            read(run/'estimated_feedback.json')['extra_reset_counters_at_bind'],worker_directory=live/'native_records',
            expected_binding=dict(configuration_sha256=actual_startup['configuration_sha256'],
                position_gains=startup['config']['base_position_gain'],bus_period_s=startup['config']['bus_period'],delay_s=startup['config']['motion'][4]),**({'trajectory':startup['trajectory']} if wrench else {}))
        require(all(report['feedback'][key] is True for key in ('passed','record_integrity_passed','clean_source_stop','worker_link_checked'))
            and report['feedback']['original_worker_snapshots_verified']==len(calls), 'Raw C++ feedback/source validation failed')
        docs={k:read(run/n) for k,n in {'session':'diagnostic_parameter_session.json','journal':'diagnostic_restore_journal_completed.json',
            'before':'diagnostic_parameters_before.json','applied':'diagnostic_parameters_applied.json','restored':'diagnostic_parameters_restored.json',
            'px4_before':'px4_parameters_before.json','px4_after':'px4_parameters_after.json'}.items()}
        _,pid,_=old.frozen_research_pid(cfg)
        for name,document in [('metadata',meta),('stage',stage)]:
            old.fixed.verify_condition(document,old.condition_for_task(cfg),name)
        report['parameters']=old.fixed.verify_parameter_session(docs,meta,old.condition_for_task(cfg),pid)
        old.precision.verify_reference(reference,old.precision.DEFAULT_REFERENCE_SOURCE)
        manifest=read(run/'native_trace_manifest.json')
        require(manifest['complete'] is True and manifest['sha256']==qualification.sha(run/'native_trace.csv'), 'Incomplete native physical trace')
        old.precision.capture_window_covers(manifest,window)
        native=old.precision.native_states(run/'native_trace.csv',window)
        from pyulog import ULog
        ulog=ULog(str(run/'px4.ulg'))
        report['recovery']=check_recovery(old.precision.dataset(ulog,'vehicle_status'),
            old.precision.dataset(ulog,'vehicle_land_detected'),rows(run/'flight_samples.csv'),window[1])
        report['configuration']=old.fixed.effective_configuration(ulog,native,reference,window,old.fixed.records(docs['applied']))
        control,derived=old.command_view(run,sends,calls,start)
        (out/'derived_command_audit_view.json').write_text(json.dumps(dict(scope='Actual send CDR matched to original worker/source; originals untouched',rows=derived),allow_nan=False))
        value_control,value_proof=ulog_value_view(control,derived,sends)
        (out/'distinct_pva_value_audit_view.json').write_text(json.dumps(dict(**value_proof,rows=value_control),allow_nan=False))
        report['px4_value_view']=value_proof
        report['handoff']=check_handoff_bridge(value_control,derived,start,ulog)
        report['handoff'].update(full_physical_window_s=window[1]-window[0],
            cpp_writer_observation_span_to_window_end_s=window[1]-report['handoff']['first_cpp_writer_observation_sim_s'],
            cpp_received_value_span_to_window_end_s=window[1]-report['handoff']['first_cpp_value_received_s'])
        report['px4_commands']=upper_commands(ulog,native,reference,value_control,window,pid,response=False,handoff=report['handoff'])
        report['px4_commands']['all_distinct_pva_value_rows_observed']=report['px4_commands'].pop('all_measurement_sends_observed')
        report['px4_commands']['distinct_pva_measurement_value_rows']=report['px4_commands'].pop('measurement_sends')
        report['px4_commands']['observation_scope']=value_proof['scope']
        report['flight']=old.upper.flight_states(run/'flight_samples.csv',window)
        report['ground']=old.fixed.native_ground_check(run,read(run/'native_ground.json'))
        report['actuator']=old.prior_audit.actuator_check(run,task.baseline_config(cfg),window)
        report['actuator'].pop('control_snapshot_to_plugin_consumption_lag',None)
        require(report['actuator']['new_joint_prediction_verified'], 'Independent whole-window joint prediction failed')
        target=(qualification.robust.figure8(start['target_position_enu']) if wrench else old.shared.ConstantTarget(np.asarray(start['target_position_enu'])))
        endpoint=old.shared.native_tip_metrics(run,target.sample,window[0],(0.,duration))
        limits=cfg['ts20_protocol'];tilt=float(native['tilt'].max())
        require(endpoint['position_rmse_m']<=limits['tip_rms_max_m'] and endpoint['position_max_m']<=limits['tip_peak_max_m']
            and tilt<=limits['tilt_max_deg'], 'Original full-window physical hold envelope exceeded')
        if wrench:
            endpoints=endpoint_windows(run,start,cfg,old.shared.native_tip_metrics)
            report.update(**endpoints)
            report['external_wrench']=check_wrench(run,start,cfg,meta,stage)
            report['robustness_performance']=robustness_performance(run,start,cfg)
            report['recovery_performance_passed']=report['robustness_performance']['performance_passed']
        else:
            report['endpoint']=endpoint
        report.update(tilt_peak_deg=tilt,window_sim_s=window)
        require(all(identity(v['path'])==v for v in artifacts), 'Original evidence changed while auditing')
        report.update(passed=True,complete=True,certificate=True,physical_acceptance=True,
            full_window_ulog_verified=True,parameter_restore_verified=True,native_ground_verified=True,
            actuator_log_verified=True,integrity_and_execution_passed=True,scope='One fresh complete multirate simulation window; original physical thresholds; not hardware qualification or a worst-case timing guarantee')
    except Exception as error:
        report.update(error=repr(error),traceback=traceback.format_exc())
    report['artifact_identities']=artifacts
    report['artifacts']={a['path']:a['sha256'] for a in artifacts}
    (out/'report.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n',encoding='utf-8')
    return report

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ('launch-dir','run-dir','output'):
        parser.add_argument('--'+name, type=Path, required=True)
    args = parser.parse_args()
    launch, run, out = args.launch_dir.resolve(), args.run_dir.resolve(), args.output.resolve()
    require(not out.exists() and not out.is_relative_to(run) and not out.is_relative_to(launch),
        'New audit output must be outside original evidence')
    out.mkdir(parents=True)
    result = audit(launch,run,out)
    print(json.dumps({k:result.get(k) for k in ('passed','integrity_and_execution_passed',
        'recovery_performance_passed','error','endpoint','timing')}, indent=2))
    return 0 if result['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
