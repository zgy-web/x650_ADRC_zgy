#!/usr/bin/env python3
"""Independent v3 acceleration-feedback structural audit and report-only proxy.

The complete existing feedback safeguards are copied into a distinct v3 check;
old audit functions and old evidence are not modified or relabeled. Raw DDS
source timestamps remain diagnostic epochs. Matching to ULog uses the original
bounded callback receipt window and the same complete position/velocity/accel
payload; no epoch offset, phase, velocity derivative or truth state is fitted.
"""
from __future__ import annotations
import csv
from pathlib import Path
import sys
import numpy as np
ROOT = Path(__file__).resolve().parents[1]
for directory in (ROOT, ROOT/"scripts", ROOT/"control_packages/x650_wholebody_v1"):
    if str(directory) not in sys.path: sys.path.insert(0, str(directory))
import audit_x650_upper_stage as upper
from audit_x650_upper_stage import (selected_rows, vector, fields, require, fixed_audit,
                                  NED_TO_ENU, FRD_TO_FLU, rotation_matrix, precision)
from simulations.x650_wholebody_mpc_v1.estimated_feedback import ACCELERATION_PROFILE, ACCELERATION_CONTRACT

# Receipt bounds are the existing state contract and existing clock boundary.
# Payload tolerance accommodates round-trip float32 text serialization only.
ULOG_RECEIPT_MAX_AGE_S = .10
CLOCK_BOUNDARY_S = .008
PAYLOAD_TOLERANCE = 2e-5
NATIVE_INTERPOLATION_MAX_GAP_S = .008
ULOG_NATIVE_HOLD_MAX_AGE_S = .024


def feedback_check_mixed(control, feedback_lines, reference, window):
    selected, times = selected_rows(control, window)
    entries = [entry for entry in feedback_lines if entry.get("phase") == "measure"
               and window[0]-1e-9 <= float(entry["sim_s"]) <= window[1]+1e-9]
    require(len(entries) == len(selected), "Every control row needs exactly one feedback snapshot")
    prior, maximum_age, snapshots = {}, {}, []
    prior_joint_stamp, joint_age_range = None, [float("inf"), -float("inf")]
    bound = reference["px4_reference_at_bind"]
    first_extra = None
    for row, entry, now in zip(selected, entries, times):
        require(abs(float(entry["sim_s"])-now) <= 1e-9 and abs(float(entry["t_s"])-float(row["t_s"])) <= 1e-9,
                "Feedback/control snapshot pairing changed")
        diagnostic = entry.get("feedback")
        require(isinstance(diagnostic, dict) and diagnostic.get("truth_feedback") is False
                and diagnostic.get("truth_clock") is False and diagnostic.get("feedback_profile") == ACCELERATION_PROFILE
                and diagnostic.get("feedback_contract") == ACCELERATION_CONTRACT
                and diagnostic.get("snapshot_protocol") == "latest_asynchronous_receipt_age_guarded"
                and abs(float(diagnostic["sim_time_s"])-now) <= 1e-8, "Upper feedback is not the reviewed PX4 receipt snapshot")
        callback_map = diagnostic["callbacks"]
        require(set(callback_map) == {"position", "attitude", "angular_velocity", "estimator_flags", "joints", "clock"},
                "Missing independently aged estimator callbacks")
        for name, callback in callback_map.items():
            seq = callback["sequence"]
            require(isinstance(seq, int) and not isinstance(seq, bool) and seq >= 1, "Invalid feedback sequence")
            received_sim, received_wall, age, wall_age = (float(callback[key]) for key in
                ("received_sim_s", "received_wall_s", "sim_age_s", "wall_age_s"))
            # Pinned PX4 EKF2 publishes status flags once a second or on a
            # change. DDS's 5 Hz configuration is only an upper rate bound.
            limit = 1.25 if name == "estimator_flags" else .25 if name == "joints" else .10
            wall_limit = 10. if name == "estimator_flags" else 4.
            require(np.isfinite([received_sim, received_wall, age, wall_age]).all()
                    and abs(now-received_sim-age) <= 1e-8 and -1e-9 <= age <= limit+1e-9 and -1e-6 <= wall_age <= wall_limit,
                    "Invalid/stale callback receipt age: "+name)
            previous = prior.get(name)
            if previous:
                require(seq >= previous[0] and received_sim >= previous[1]-1e-9 and received_wall >= previous[2]-1e-9,
                        "Feedback callback sequence/time regressed: "+name)
                if seq == previous[0]:
                    require(received_sim == previous[1] and received_wall == previous[2], "Held receipt mutated without callback")
            prior[name] = (seq, received_sim, received_wall)
            maximum_age[name] = max(maximum_age.get(name, 0.), age)
        require(abs(float(diagnostic["state_stamp_s"])-callback_map["angular_velocity"]["received_sim_s"]) <= 1e-9,
                "Controller state derivative stamp is not angular callback receipt time")
        joint_stamp, joint_age = float(diagnostic["joint_stamp_s"]), float(diagnostic["joint_source_age_s"])
        require(np.isfinite([joint_stamp, joint_age]).all() and abs(now-joint_stamp-joint_age) <= 1e-8
                and abs(joint_age) <= .25+1e-9 and (prior_joint_stamp is None or joint_stamp >= prior_joint_stamp-1e-9),
                "Joint source timestamp is stale, inconsistent or regressed")
        prior_joint_stamp = joint_stamp
        joint_age_range = [min(joint_age_range[0], joint_age), max(joint_age_range[1], joint_age)]
        require(int(float(row["feedback_snapshot_sequence"])) == callback_map["position"]["sequence"], "Control used another feedback sequence")
        snapshot, state = diagnostic["px4_snapshot"], diagnostic["control_state"]
        require(np.max(abs(vector(state["q_rad"], 2, "measured joints")-fields([row], "q", "12")[0])) <= 1e-9
                and np.max(abs(vector(state["dq_rad_s"], 2, "measured joint rates")-fields([row], "qd", "12")[0])) <= 1e-9,
                "Controller joint feedback differs from the paired snapshot")
        pn, vn = vector(snapshot["position_ned_m"], 3, "PX4 p"), vector(snapshot["velocity_ned_m_s"], 3, "PX4 v")
        pw = NED_TO_ENU@pn
        pw[2] += bound["ref_alt"]
        vw = NED_TO_ENU@vn
        require(np.max(abs(vector(state["position_enu_m"], 3, "feedback p")-pw)) <= 1e-9
                and np.max(abs(vector(state["velocity_enu_m_s"], 3, "feedback v")-vw)) <= 1e-9,
                "PX4 feedback world conversion does not use frozen raw-EV origin")
        for prefix, expected, axes in (("feedback_position_", pn, "ned"), ("feedback_velocity_", vn, "ned"),
                                      ("feedback_p", pw, "xyz"), ("feedback_v", vw, "xyz")):
            require(np.max(abs(fields([row], prefix, axes)[0]-expected)) <= 1e-9, "Control row did not use logged snapshot: "+prefix)
        quaternion = vector(snapshot["attitude_wxyz"], 4, "PX4 q")
        require(abs(np.linalg.norm(quaternion)-1.) <= 1e-3, "Nonunit PX4 attitude")
        rotation = np.asarray(state["rotation_flu_to_enu"], float)
        require(rotation.shape == (3, 3) and np.isfinite(rotation).all()
                and np.max(abs(rotation-NED_TO_ENU@rotation_matrix(quaternion)@FRD_TO_FLU)) <= 1e-9,
                "PX4 attitude conversion changed")
        require(np.max(abs(vector(state["omega_flu_rad_s"], 3, "feedback rates")
                    -FRD_TO_FLU@vector(snapshot["angular_velocity_frd_rad_s"], 3, "PX4 rates"))) <= 1e-9,
                "PX4 angular-rate conversion changed")
        require(all(snapshot["valid"].get(key) is True for key in
                    ("xy_valid", "z_valid", "v_xy_valid", "v_z_valid", "heading_good_for_control"))
                and snapshot.get("dead_reckoning") is False, "PX4 feedback valid/dead-reckoning guard failed")
        flags = snapshot["estimator_flags"]
        require(all(flags.get(key) is bool(value) for key, value in fixed_audit.EV_FLAGS.items())
                and flags.get("cs_inertial_dead_reckoning") is False and flags.get("cs_wind_dead_reckoning") is False,
                "PX4 feedback fusion/dead-reckoning flags changed")
        origin = snapshot["origin"]
        for key, tolerance in (("ref_lat", 1e-11), ("ref_lon", 1e-11), ("ref_alt", 1e-6)):
            require(abs(float(origin[key])-bound[key]) <= tolerance, "Feedback origin changed: "+key)
        for key in ("xy_global", "z_global", "xy_reset_counter", "z_reset_counter", "heading_reset_counter"):
            require(origin[key] == bound[key], "Feedback origin/reset changed: "+key)
        extra = snapshot["extra_reset_counters"]
        require(set(extra) == {"vxy_reset_counter", "vz_reset_counter", "quat_reset_counter"}, "Extra reset counters missing")
        if first_extra is None:
            first_extra = extra.copy()
        require(extra == first_extra, "Velocity/quaternion reset in feedback window")
        for topic in ("position", "attitude", "angular_velocity", "estimator_flags"):
            raw_times = diagnostic["px4_raw_timestamps"][topic]
            require(set(raw_times) == {"timestamp", "timestamp_sample"}
                    and all(isinstance(value, int) and not isinstance(value, bool) and value >= 0
                            for value in raw_times.values()), "Raw PX4 diagnostic timestamps invalid")
        snapshots.append(snapshot)
    return {"verified": True, "feedback_contract": ACCELERATION_CONTRACT, "sample_count": len(selected),
            "maximum_callback_receipt_age_s": maximum_age, "extra_reset_counters": first_extra,
            "joint_source_age_range_s": joint_age_range,
            "truth_feedback": False, "truth_clock": False,
            "timestamp_semantics": "Asynchronous held PX4 callbacks; individual simulation receipt ages; DDS epochs are diagnostic only"}, snapshots


def load_native_acceleration(path, window):
    """Load independently recorded world acceleration, with bounded edge context.

    WorldLinearAcceleration is read directly from the recorder. It is never
    synthesized from position or velocity, and it is never fed to a controller.
    """
    with Path(path).open(newline='', encoding='utf-8') as stream:
        records = [row for row in csv.DictReader(stream)
                   if window[0]-.008-1e-9 <= float(row['sim_s']) <= window[1]+.008+1e-9]
    require(len(records) >= 2 and all(row.get('valid_accel') == '1' for row in records),
            'Independent native acceleration missing or invalid')
    t = np.asarray([float(row['sim_s']) for row in records])
    acceleration = np.asarray([[float(row['base_a'+axis]) for axis in 'xyz'] for row in records])
    require(np.isfinite(t).all() and np.isfinite(acceleration).all() and np.all(np.diff(t) > 0)
            and np.max(np.diff(t)) <= .008+1e-9
            and t[0] <= window[0]+.008+1e-9 and t[-1] >= window[1]-.008-1e-9,
            'Native acceleration lacks complete bounded physical coverage')
    return {'t': t, 'acceleration': acceleration,
            'source': 'Independent native WorldLinearAcceleration, world ENU, no numerical differentiation'}


def _acceleration_snapshots(control, feedback_lines, window):
    selected, times = selected_rows(control, window)
    entries = [entry for entry in feedback_lines if entry.get('phase') == 'measure'
               and window[0]-1e-9 <= float(entry['sim_s']) <= window[1]+1e-9]
    require(len(entries) == len(selected), 'Acceleration snapshots must pair every control row')
    result, previous = [], None
    for row, entry, now in zip(selected, entries, times):
        require(abs(float(entry['sim_s'])-now) <= 1e-9, 'Acceleration/control snapshot pairing changed')
        diagnostic = entry['feedback']
        require(diagnostic.get('feedback_profile') == ACCELERATION_PROFILE
                and diagnostic.get('feedback_contract') == ACCELERATION_CONTRACT
                and diagnostic.get('truth_feedback') is False and diagnostic.get('truth_clock') is False,
                'Acceleration feedback is not the declared PX4-only v3 contract')
        callback = diagnostic['callbacks']['position']
        raw = diagnostic['px4_raw_timestamps']['position']
        source = diagnostic.get('acceleration_source', {})
        require(source.get('topic') == 'vehicle_local_position'
                and source.get('quantity') == 'earth_fixed_velocity_derivative'
                and source.get('input_frame') == 'NED' and source.get('output_frame') == 'ENU'
                and source.get('numerical_differentiation') is False and source.get('truth_feedback') is False,
                'Acceleration must be native PX4 earth-fixed derivative, without truth/differentiation fallback')
        sequence = callback['sequence']
        require(type(sequence) is int and sequence >= 1 and source.get('callback_sequence') == sequence,
                'Acceleration source callback sequence differs from position')
        receipt, wall = float(callback['received_sim_s']), float(callback['received_wall_s'])
        age, wall_age = float(callback['sim_age_s']), float(callback['wall_age_s'])
        require(np.isfinite([receipt, wall, age, wall_age]).all()
                and -1e-9 <= age <= .10+1e-9 and -1e-6 <= wall_age <= 4.
                and abs(now-receipt-age) <= 1e-8
                and source.get('received_sim_s') == callback['received_sim_s']
                and source.get('received_wall_s') == callback['received_wall_s']
                and diagnostic.get('acceleration_stamp_s') == callback['received_sim_s'],
                'Acceleration source is stale or uses another callback receipt stamp')
        require(all(type(raw[key]) is int and raw[key] >= 0 and source.get(key) == raw[key]
                    for key in ('timestamp', 'timestamp_sample')), 'Acceleration raw timestamps differ from position payload')
        snapshot, state = diagnostic['px4_snapshot'], diagnostic['control_state']
        an = vector(snapshot.get('acceleration_ned_m_s2'), 3, 'PX4 earth-fixed NED acceleration')
        ae = vector(state.get('acceleration_enu_m_s2'), 3, 'Controller ENU acceleration')
        require(np.max(abs(ae-NED_TO_ENU@an)) <= 1e-9,
                'Acceleration frame conversion changed or added gravity/body rotation')
        require(np.max(abs(fields([row], 'feedback_a', 'xyz')[0]-ae)) <= 1e-9
                and abs(float(row['feedback_acceleration_position_stamp_s'])-receipt) <= 1e-9
                and float(row['feedback_acceleration_position_sequence']) == sequence,
                'Control did not use its paired position-callback acceleration')
        payload = np.concatenate((vector(snapshot['position_ned_m'], 3, 'PX4 position'),
                                  vector(snapshot['velocity_ned_m_s'], 3, 'PX4 velocity'), an))
        if previous is not None:
            require(sequence >= previous['sequence'] and receipt >= previous['receipt']-1e-9
                    and wall >= previous['wall']-1e-9, 'Acceleration callback source regressed')
            if sequence == previous['sequence']:
                require(receipt == previous['receipt'] and wall == previous['wall']
                        and raw == previous['raw'] and np.array_equal(payload, previous['payload']),
                        'Held position callback payload/stamps changed without a new callback')
        result.append({'sim_s': float(now), 'receipt': receipt, 'wall': wall, 'sequence': sequence,
                       'raw': raw, 'payload': payload, 'acceleration': ae, 'age': age})
        previous = result[-1]
    return result


def audit_mixed_acceleration_feedback(control, feedback_lines, ulog, native_acceleration, window):
    """Verify estimator acceleration provenance; physical proxy scores are report-only.

    Call feedback_check_mixed as the full pose/rates/reset/fusion guard first.
    Matching uses unmodified ULog publication timestamps and complete p/v/a
    payloads inside the already declared receipt-age/clock-boundary allowances.
    Raw DDS epochs are retained, never fitted or reinterpreted as simulation time.
    """
    samples = _acceleration_snapshots(control, feedback_lines, window)
    local = precision.dataset(ulog, 'vehicle_local_position')
    sample_t = precision.stamp(local)
    publish_t = np.asarray(local['timestamp'], float)*1e-6
    payload = precision.fields(local, ('x', 'y', 'z', 'vx', 'vy', 'vz', 'ax', 'ay', 'az'))
    require(len(publish_t) >= 2 and len(publish_t) == len(sample_t) == len(payload)
            and np.isfinite(publish_t).all() and np.all(np.diff(publish_t) >= 0)
            and publish_t.max() <= 1e7, 'ULog acceleration publication clock must be original HRT')
    inside = (publish_t >= window[0]-ULOG_RECEIPT_MAX_AGE_S)&(publish_t <= window[1]+CLOCK_BOUNDARY_S)
    require(np.isfinite(payload[inside]).all(), 'ULog acceleration/position/velocity payload is nonfinite')
    matches, ages, sample_lags, raw_lags, max_residual = {}, [], [], [], 0.
    for snapshot in samples:
        seq = snapshot['sequence']
        if seq not in matches:
            receipt = snapshot['receipt']
            candidate = np.flatnonzero((publish_t >= receipt-ULOG_RECEIPT_MAX_AGE_S-1e-9)
                                      &(publish_t <= receipt+CLOCK_BOUNDARY_S+1e-9))
            if len(candidate):
                residual = np.max(abs(payload[candidate]-snapshot['payload']), axis=1)
                candidate = candidate[residual <= PAYLOAD_TOLERANCE]
            require(len(candidate) > 0, 'Position callback acceleration has no bounded-causal same-payload ULog source')
            # When stationary identical payloads may recur. Latest matching
            # publication is deterministic; this is a bounded source witness,
            # not a claim that a unique transport delay was identified.
            index = int(candidate[-1])
            matches[seq] = index
            ages.append(float(receipt-publish_t[index]))
            sample_lags.append(float(publish_t[index]-sample_t[index]))
            raw_lags.append((snapshot['raw']['timestamp']-snapshot['raw']['timestamp_sample'])*1e-6)
            max_residual = max(max_residual, float(np.max(abs(payload[index]-snapshot['payload']))))
    native_t = np.asarray(native_acceleration['t'], float)
    native_a = np.asarray(native_acceleration['acceleration'], float)
    require(native_a.shape == (len(native_t), 3) and len(native_t) >= 2
            and np.isfinite(native_t).all() and np.isfinite(native_a).all()
            and np.all(np.diff(native_t) > 0) and np.max(np.diff(native_t)) <= .008+1e-9
            and native_t[0] <= window[0]+.008+1e-9 and native_t[-1] >= window[1]-.008-1e-9,
            'Independent native acceleration coverage is insufficient')
    times = np.asarray([sample['sim_s'] for sample in samples])
    held_a = np.asarray([sample['acceleration'] for sample in samples])
    # Only an <=8 ms file-edge sliver can lack interpolation support. Report
    # any omitted boundary observations; never invent or backfill truth values.
    covered = (times >= native_t[0]-1e-9)&(times <= native_t[-1]+1e-9)
    require(covered.sum() >= 2
            and np.all(native_t[0]-times[~covered] <= .008+1e-9)
            and np.all(times[~covered]-native_t[-1] <= .008+1e-9),
            'Control/native acceleration timing lacks bounded edge coverage')
    query = times[covered]
    truth_at_control = precision.resample(native_t, native_a, query, NATIVE_INTERPOLATION_MAX_GAP_S)
    held_proxy = precision.metrics(query, held_a[covered]-truth_at_control, 'm/s^2')
    measured = (native_t >= window[0]-1e-9)&(native_t <= window[1]+1e-9)
    native_query = native_t[measured]
    estimated_at_native = precision.resample(sample_t, payload[:, 6:9]@NED_TO_ENU.T,
                                            native_query, ULOG_NATIVE_HOLD_MAX_AGE_S, hold=True)
    native_proxy = precision.metrics(native_query, estimated_at_native-native_a[measured], 'm/s^2')
    return {'verified': True, 'feedback_contract': ACCELERATION_CONTRACT,
        'measurement_snapshots': len(samples), 'distinct_position_callbacks': len(matches),
        'source_payload': 'vehicle_local_position x/y/z,vx/vy/vz,ax/ay/az from one publication',
        'source_matching': 'Bounded original callback receipt window; identical static payloads may have multiple witnesses',
        'ulog_receipt_max_age_s': ULOG_RECEIPT_MAX_AGE_S, 'clock_boundary_allowance_s': CLOCK_BOUNDARY_S,
        'payload_absolute_tolerance': PAYLOAD_TOLERANCE, 'maximum_matched_payload_difference': max_residual,
        'matched_receipt_minus_publication_range_s': [min(ages), max(ages)],
        'ulog_publication_minus_sample_range_s': [min(sample_lags), max(sample_lags)],
        'dds_raw_publication_minus_sample_range_s': [min(raw_lags), max(raw_lags)],
        'maximum_position_callback_hold_s': max(sample['age'] for sample in samples),
        'truth_feedback': False, 'numerical_differentiation': False, 'gravity_added': False,
        'fitted_time_offset_s': None, 'transport_delay_identified': False,
        'physical_acceleration_proxy': {'acceptance_policy': 'report_only', 'acceptance_thresholds': None,
            'changes_response_model_acceptance': False, 'native_source': native_acceleration.get('source', 'Independent native world linear acceleration'),
            'held_controller_snapshot_minus_native_at_control_time': held_proxy,
            'ulog_causal_hold_minus_native_at_native_time': native_proxy,
            'control_samples_omitted_at_native_file_edges': int((~covered).sum()),
            'native_interpolation_max_gap_s': NATIVE_INTERPOLATION_MAX_GAP_S,
            'ulog_native_hold_max_age_s': ULOG_NATIVE_HOLD_MAX_AGE_S,
            'alignment': 'Original simulation times only, no fitted offset/phase, no gravity removal or velocity differentiation'}}
