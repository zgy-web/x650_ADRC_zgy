#pragma once
#include "mpc_core.hpp"
#include <json/json.h>
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
  ARRAY(candidate_envelope);ARRAY(horizon);ARRAY(plan);ARRAY(predicted_states);ARRAY(predicted_tip);ARRAY(predicted_rotations);
  ARRAY(query_times);ARRAY(nominal_states);ARRAY(candidate_states);ARRAY(candidate_extrema);ARRAY(candidate_boundary_times);ARRAY(candidate_modes);ARRAY(candidate_zero_events);
  ARRAY(A);ARRAY(b);ARRAY(H);ARRAY(g);ARRAY(C);ARRAY(lower);ARRAY(upper);ARRAY(solution);
#undef ARRAY
  out["nominal_packets"]=packets_json(d.nominal_packets);out["candidate_packets"]=packets_json(d.candidate_packets);
  out["attempts"]=Json::Value(Json::arrayValue);for(const auto& a:d.attempts){Json::Value v;v["passed"]=a.passed;v["actual_bounds"]=a.actual_bounds;
    v["q_error"]=scalar_json(a.q_error);v["dq_error"]=scalar_json(a.dq_error);v["elapsed_ms"]=scalar_json(a.elapsed_ms);out["attempts"].append(std::move(v));}
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
}
