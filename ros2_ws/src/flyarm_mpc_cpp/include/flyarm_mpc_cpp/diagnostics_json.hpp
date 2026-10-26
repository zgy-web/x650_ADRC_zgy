#pragma once
#include "mpc_core.hpp"
#include <json/json.h>
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace flyarm_mpc_cpp {
inline Json::Value scalar_json(double value) {
  if(!std::isfinite(value))throw std::runtime_error("Nonfinite native diagnostic scalar");
  return Json::Value(value);
}
template<class Values> Json::Value numeric_json(const Values& values) {
  Json::Value out(Json::arrayValue);
  for(auto value:values){if(!std::isfinite(double(value)))throw std::runtime_error("Nonfinite native diagnostic");out.append(value);}
  return out;
}
inline Json::Value packets_json(const std::vector<x650::Packet>& packets) {
  Json::Value rows(Json::arrayValue);for(const auto& packet:packets){Json::Value row;row["send_time_s"]=scalar_json(packet.send_time);
    row["q_ref"]=numeric_json(packet.q_ref);row["dq_ref"]=numeric_json(packet.dq_ref);row["tau_ff"]=numeric_json(packet.tau_ff);
    row["kp"]=numeric_json(packet.kp);row["kd"]=numeric_json(packet.kd);rows.append(std::move(row));}return rows;
}
inline Json::Value diagnostics_json(const x650::Diagnostics& d) {
  Json::Value out;
#define SCALAR(name) out[#name]=d.name
  SCALAR(subproblems);SCALAR(active_constraints);SCALAR(solver_mode);SCALAR(qp_rows);SCALAR(qp_columns);
  SCALAR(accepted);SCALAR(nonlinear_passed);SCALAR(actual_bounds);SCALAR(previous_plan_fallback);SCALAR(candidate_substeps);
#undef SCALAR
#define FINITE_SCALAR(name) out[#name]=scalar_json(d.name)
  FINITE_SCALAR(primal_violation);FINITE_SCALAR(kkt_stationarity);FINITE_SCALAR(kkt_complementarity);FINITE_SCALAR(objective);
  FINITE_SCALAR(q_linearization_error);FINITE_SCALAR(dq_linearization_error);FINITE_SCALAR(compute_ms);
#undef FINITE_SCALAR
#define ARRAY(name) out[#name]=numeric_json(d.name)
  ARRAY(candidate_envelope);ARRAY(horizon);ARRAY(plan);ARRAY(predicted_states);ARRAY(linearized_states);ARRAY(predicted_tip);ARRAY(predicted_rotations);
  ARRAY(query_times);ARRAY(nominal_states);ARRAY(candidate_states);ARRAY(candidate_extrema);ARRAY(candidate_boundary_times);ARRAY(candidate_modes);ARRAY(candidate_zero_events);
  ARRAY(A);ARRAY(b);ARRAY(H);ARRAY(g);ARRAY(C);ARRAY(lower);ARRAY(upper);ARRAY(solution);
#undef ARRAY
  out["nominal_packets"]=packets_json(d.nominal_packets);out["candidate_packets"]=packets_json(d.candidate_packets);
  out["attempts"]=Json::Value(Json::arrayValue);for(const auto& a:d.attempts){Json::Value v;v["passed"]=a.passed;v["actual_bounds"]=a.actual_bounds;
    v["q_error"]=scalar_json(a.q_error);v["dq_error"]=scalar_json(a.dq_error);v["elapsed_ms"]=scalar_json(a.elapsed_ms);out["attempts"].append(std::move(v));}
  out["recovery_triggered"]=d.recovery_triggered;out["recovery_reason"]=d.recovery_reason;
  out["recovery_trust_radius"]=scalar_json(d.recovery_trust_radius);
  out["recovery_stopping_violation"]=scalar_json(d.recovery_stopping_violation);
  out["recovery_attempts"]=Json::Value(Json::arrayValue);
  for(const auto& a:d.recovery_attempts){Json::Value v;
    v["centre_kind"]=a.centre_kind;v["rejection"]=a.rejection;
    v["nonlinear_passed"]=a.nonlinear_passed;v["original_constraints_passed"]=a.original_constraints_passed;v["trusted"]=a.trusted;
    v["projection_attempted"]=a.projection_attempted;v["projection_solved"]=a.projection_solved;
    v["original_primal_violation"]=scalar_json(a.original_primal_violation);v["stopping_violation"]=scalar_json(a.stopping_violation);v["elapsed_ms"]=scalar_json(a.elapsed_ms);
    v["centre_plan"]=numeric_json(a.centre_plan);v["node_times"]=numeric_json(a.node_times);v["node_states"]=numeric_json(a.node_states);v["node_references"]=numeric_json(a.node_references);
    v["envelope"]=numeric_json(a.envelope);v["substeps"]=a.substeps;out["recovery_attempts"].append(std::move(v));}
  out["capture_semantics"]="owning values from the actual native call, moved after publication; no extra prediction or solver call";
  return out;
}
inline Json::Value result_json(const x650::StepResult& result) {
  Json::Value out;
#define ARRAY(name) out[#name]=numeric_json(result.name)
  ARRAY(command);ARRAY(position_reference);ARRAY(velocity_feedforward);ARRAY(acceleration_feedforward);ARRAY(joint_velocity);
  ARRAY(joint_reference);ARRAY(joint_velocity_reference);ARRAY(joint_acceleration_reference);ARRAY(nominal_acceleration);ARRAY(tip_actual);
#undef ARRAY
  out["diagnostics"]=diagnostics_json(result.diagnostics);return out;
}
// Metadata cross-check only: original Core nonlinear/physical checks remain
// authoritative. Projection QPs are seed generation and never wire proposals.
inline bool recovery_evidence_valid(const x650::Diagnostics& d,const x650::Config& c) {
  if(!d.recovery_triggered)return d.recovery_attempts.empty()&&d.recovery_reason.empty();
  if(d.recovery_reason.empty()||d.recovery_attempts.empty()||d.recovery_attempts.size()>2
      ||d.recovery_trust_radius!=std::min(c.trust_radius,.02)
      ||!std::isfinite(d.recovery_stopping_violation)||d.recovery_stopping_violation>c.feasibility_tolerance)return false;
  int projections=0;bool trusted=false;
  for(const auto& a:d.recovery_attempts){projections+=a.projection_attempted;
    if(a.projection_solved&&!a.projection_attempted)return false;
    if(a.trusted){if(!a.nonlinear_passed||!a.original_constraints_passed
        ||!std::isfinite(a.original_primal_violation)||a.original_primal_violation>c.feasibility_tolerance
        ||!std::isfinite(a.stopping_violation)||a.stopping_violation>c.feasibility_tolerance
        ||a.centre_plan.size()!=std::size_t(5*c.nodes)||a.node_times.size()!=std::size_t(c.nodes)
        ||a.node_states.size()!=std::size_t(4*c.nodes)||a.node_references.size()!=std::size_t(2*c.nodes))return false;
      trusted=true;}}
  return trusted&&projections<=1&&d.subproblems+projections<=4;
}
}
