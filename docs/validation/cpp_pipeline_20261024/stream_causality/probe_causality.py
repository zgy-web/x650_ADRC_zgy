"""Offline counterexample only: ACK arrival is not actuator adoption.

Uses the frozen TS20 gains/normalized coefficients and independent reference
integrator. Does not invoke a controller, publish, or modify any model policy.
The late-adoption schedule is a counterfactual truth supplied to this test, not
an online estimate or a proposed insertion into actual command history.
"""
from pathlib import Path
import hashlib
import json
import sys
import numpy as np

ROOT = next(parent for parent in Path(__file__).resolve().parents
            if (parent/'control_packages/x650_wholebody_v1').is_dir())
sys.path.insert(0, str(ROOT/'control_packages/x650_wholebody_v1'))
from x650_wholebody.ts20 import build_mit_commands
from x650_wholebody.ts20_prediction import Ts20PredictionModel


def identity(path):
    return dict(path=str(path), bytes=path.stat().st_size,
                sha256=hashlib.sha256(path.read_bytes()).hexdigest())


def main():
    here = Path(__file__).resolve().parent
    config = json.loads((here/'startup_manifest.json').read_text())['config']
    m = config['motion']
    metadata = dict(motor='TS20-50', schema='ts20_public_motion_v1',
        status='frozen_conditional_interface_model',
        fixed_parameters=dict(J_total_over_ka=m[0], b_over_ka=m[1], Fc_over_ka=m[2],
            offset_over_ka=m[3], combined_interface_delay_s=m[4]),
        friction_parameters=dict(static_threshold=m[5], delta_static=m[6], stribeck_velocity=m[7]))
    predictor = Ts20PredictionModel(metadata, kp=config['kp'], kd=config['kd'],
        bus_period_s=config['bus_period'], max_step_s=config['integration_step'],
        trust_radius_normalized=config['trust_radius'])
    old = np.array([.30, .20]); prior = np.array([.31, .21]); candidate = np.array([.32, .22])
    q0 = np.zeros(2); dq0 = old.copy(); r0 = np.zeros(2)
    h = np.r_[.02, np.full(14, .04)]; nodes = np.cumsum(h)
    query = np.arange(0., .580000001, .001)
    historic = [dict(send_time_s=-.005, q_ref=-.005*old, dq_ref=old,
                     tau_ff=np.zeros(2), kp=predictor.kp.copy(), kd=predictor.kd.copy())]
    plan = np.zeros((15, 5)); plan[:, 3:] = candidate
    prediction_packets = predictor._future_packets(0., nodes, plan, np.array(config['velocity_limit'][3:]),
        r0, [dict(p) for p in historic], 0., sensitivities=False)[0]

    def actual_packets(adoption_times, values):
        encoded = build_mit_commands(adoption_times, values, r0,
            bus_period_s=config['bus_period'], kp=config['kp'][0], kd=config['kd'][0])
        return [dict(p) for p in historic] + [dict(send_time_s=float(t),
            **{key: val[i].copy() for key, val in encoded['command'].items()})
            for i, t in enumerate(encoded['input_time_s']) if t < nodes[-1]+1e-10]

    def evaluate(packets):
        fresh = Ts20PredictionModel(metadata, kp=config['kp'], kd=config['kd'],
            bus_period_s=config['bus_period'], max_step_s=config['integration_step'],
            trust_radius_normalized=config['trust_radius'])
        return fresh.rollout(0., q0.copy(), dq0.copy(), packets, query)

    # Two ACK delivery histories, identical physical adoption. ACKs are NOT an
    # input to packet generation or mechanical dynamics.
    timely = actual_packets([0., .58], np.array([candidate, candidate]))
    ack_late = actual_packets([0., .58], np.array([candidate, candidate]))
    delayed = actual_packets([0., .008, .028, .58], np.array([old, prior, candidate, candidate]))
    predicted = evaluate(prediction_packets)
    ideal = evaluate(timely)
    late_ack = evaluate(ack_late)
    late_adopt = evaluate(delayed)
    indices = np.array([20, 40, 100, 300, 580])
    delta_q = late_adopt['q']-predicted['q']
    delta_dq = late_adopt['dq']-predicted['dq']
    # Exact ZOH missing integral, independently from the plant integrator.
    missing_r = (candidate-old)*.008+(candidate-prior)*.020
    assert np.all(abs(candidate-prior) <= np.array(config['acceleration_limit'][3:])*.02+1e-12)
    assert np.all(abs(prior-old) <= np.array(config['acceleration_limit'][3:])*.02+1e-12)
    assert np.array_equal(ideal['q'], late_ack['q']) and np.array_equal(ideal['dq'], late_ack['dq'])
    assert np.max(abs(delta_q)) > 0 and np.max(abs(delta_dq)) > 0
    scan=[]
    for lag in (.0,.004,.008,.012,.020):
        ts=[0.,lag,lag+.020,.580] if lag else [0.,.020,.580]
        vs=np.array([old,prior,candidate,candidate]) if lag else np.array([prior,candidate,candidate])
        actual=evaluate(actual_packets(ts,vs))
        eq=np.max(abs(actual['q']-predicted['q']),axis=0)
        edq=np.max(abs(actual['dq']-predicted['dq']),axis=0)
        scan.append(dict(each_publish_to_adopt_ms=(lag+.020)*1000,
            omitted_reference_rad=((candidate-old)*lag+(candidate-prior)*.020).tolist(),
            q_peak_rad=eq.tolist(),dq_peak_rad_s=edq.tolist(),
            endpoint_fixed_body_geometric_upper_bound_mm=float(1000*(.55*eq[0]+.25*eq[1]))))
    result = dict(kind='offline_ack_vs_physical_adoption_counterexample_v1', physical_acceptance=False,
        scope='Synthetic causal limit, not a plant experiment, timing benchmark, or online in-flight estimate',
        frozen_config=identity(here/'startup_manifest.json'),
        sources=[identity(Path(__file__).resolve()), identity(ROOT/'control_packages/x650_wholebody_v1/x650_wholebody/ts20.py'),
                 identity(ROOT/'control_packages/x650_wholebody_v1/x650_wholebody/ts20_prediction.py')],
        initial=dict(q=q0.tolist(), dq=dq0.tolist(), r=r0.tolist(), held_command=old.tolist()),
        commands=dict(prior_sent_at_s=-.02, prior=prior.tolist(), new_candidate=candidate.tolist(),
            command_delta_within_original_acceleration_limit=True),
        ack_only=dict(first_ack_delivery_ms=[4., 44.], physical_schedule_unchanged=True,
            q_bit_exact=np.array_equal(ideal['q'], late_ack['q']), dq_bit_exact=np.array_equal(ideal['dq'], late_ack['dq']),
            current_predictor_vs_ideal_q_max=float(np.max(abs(predicted['q']-ideal['q']))),
            current_predictor_vs_ideal_dq_max=float(np.max(abs(predicted['dq']-ideal['dq'])))),
        actual_adoption_delayed=dict(prior_adopt_s=.008, candidate_adopt_s=.028,
            known_at_current_snapshot=False, omitted_unquantized_r_integral_rad=missing_r.tolist(),
            q_difference_peak_rad=np.max(abs(delta_q), axis=0).tolist(),
            dq_difference_peak_rad_s=np.max(abs(delta_dq), axis=0).tolist(),
            samples=[dict(t_s=float(query[k]), predicted_q=predicted['q'][k].tolist(),
                actual_q=late_adopt['q'][k].tolist(), predicted_dq=predicted['dq'][k].tolist(),
                actual_dq=late_adopt['dq'][k].tolist()) for k in indices]),
        bounded_adoption_scan=scan,
        conclusion='ACK-only delay leaves the prediction unchanged. Genuine future adoption delay is missing from the current predictor; later state correction does not make this earlier prediction exact.')
    (here/'report.json').write_text(json.dumps(result, indent=2)+'\n', encoding='utf-8')
    print(json.dumps({k: result[k] for k in ('ack_only', 'actual_adoption_delayed', 'conclusion')}, indent=2))


if __name__ == '__main__':
    main()
