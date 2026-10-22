"""Predeclared upper-controller comparison; no free-form flight tuning."""
from copy import deepcopy
import hashlib
import json


PROTOCOL_NAME = "mixed_response_zero_ff_v1"
PREDICTION_CHAIN_PROTOCOL_NAME = "mixed_response_prediction_chain_v2"
STAGES = ("tip_hold", "small", "normal", "fast")


def build_mixed_mpc_protocol():
    return {
        "kind": "x650_mixed_mpc_protocol", "schema_version": 1, "name": PROTOCOL_NAME,
        "research_profile": "precision_ev_mixed_mpc_v3",
        "feedback_contract": "px4_estimator_acceleration_receipt_enu_v2",
        "control_interface": "raw_ev_world_pva_v2",
        "acceleration_feedforward": "zero", "response_model_kind": "xy_second_order_z_first_order_v1",
        "stages": list(STAGES),
        "common_controller_config": {
            "arm_share": .2, "velocity_limit": [.55, .35, .45, .45, .35],
            "acceleration_limit": [.45, .30, .45, .60, .60],
            "q_lower": [-.75, -.35], "q_upper": [.45, .85],
            "joint_position_gain": 3., "tip_gain": 2., "max_tip_correction_m_s": .20,
            "task_weight": 1., "measured_base_weight": 4., "base_nominal_weight": 4.,
            "arm_regularizer_length_m_rad": .02, "max_dt_s": .05,
        },
        "mpc_controller_config": {
            "nodes": 15, "node_dt_s": .04, "attitude_rate_decay_s": .15,
            "max_iterations": 100, "solver_ftol": 1e-9, "feasibility_tolerance": 2e-7,
            "kkt_tolerance": 2e-3, "solve_budget_ms": 20., "failure_policy": "raise",
            "tip_position_weight": 400., "tip_velocity_weight": 4., "base_position_weight": .25,
            "joint_posture_weight": .02, "input_nominal_weight": .01, "input_delta_weight": .01,
        },
        "acceptance": {
            "full_window_required": True, "tip_rms_max_m": .05, "tip_peak_max_m": .10,
            "tilt_max_deg": 8.,
            "solver": {"accepted_optimum": True, "previous_plan_fallback": False,
                "primal_max": 2e-7, "kkt_max": 2e-3, "solve_budget_ms": 20.,
                "solve_budget_exceeded": False},
        },
        "interpretation": "Nominal base response only; moving-arm joint dynamics and torque-level NMPC remain unvalidated",
    }


def protocol_hash(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(",", ":"), allow_nan=False).encode()).hexdigest()


def build_prediction_chain_protocol():
    """New, explicit development candidate; the original protocol is immutable.

    Same physical/feedback operating condition and all comparison limits. The
    joint delay is measured from state consumption to a native velocity change,
    not a separately identified servo time constant. Fresh full-window flights
    test this controller; old failed flights are development evidence only.
    """
    value = build_mixed_mpc_protocol()
    value.update(schema_version=2, name=PREDICTION_CHAIN_PROTOCOL_NAME)
    value["mpc_controller_config"]["prediction_chain"] = {
        "kind": "delayed_joint_thrust_attitude_v1",
        "joint_delay_s": .024, "joint_delay_guard_s": .05,
        "gravity_m_s2": 9.8, "attitude_residual_decay_s": .15,
    }
    value["interpretation"] = (
        "Accepted nominal base response unchanged; delayed joint command history and "
        "acceleration-driven thrust attitude are a development candidate tested by fresh "
        "complete endpoint holds. Not an independently identified physical servo, "
        "moving-arm dynamics or torque-level NMPC model.")
    return value


def validate_mixed_mpc_protocol(value):
    if value is None:
        return None
    expected = (build_prediction_chain_protocol() if isinstance(value, dict)
                and value.get("name") == PREDICTION_CHAIN_PROTOCOL_NAME else build_mixed_mpc_protocol())
    if not isinstance(value, dict) or protocol_hash(value) != protocol_hash(expected):
        raise ValueError("Mixed MPC protocol differs from the frozen preset")
    return deepcopy(expected)


def baseline_config(protocol):
    checked = validate_mixed_mpc_protocol(protocol)
    if checked is None:
        raise ValueError("An explicit mixed MPC comparison protocol is required")
    return deepcopy(checked["common_controller_config"])


def controller_config(protocol, model):
    checked = validate_mixed_mpc_protocol(protocol)
    result = baseline_config(checked)
    from accepted_response_model import validate_model_structure
    validate_model_structure(model)
    result.update(deepcopy(checked["mpc_controller_config"]))
    result.update(response_model={"kind": checked["response_model_kind"],
                                  "parameters": deepcopy(model["parameters"])}, response_identified=True)
    return result


def protocol_annotation(cfg):
    protocol = validate_mixed_mpc_protocol(cfg.get("mixed_mpc_protocol"))
    return {} if protocol is None else {"mixed_mpc_protocol": protocol,
                                       "mixed_mpc_protocol_sha256": protocol_hash(protocol)}
