#pragma once
#include "mpc_core.hpp"
#include "joint_safety.hpp"
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
inline Json::Value interval_point_json(const x650::IntervalPoint& p) {
  Json::Value out;out["time"]=scalar_json(p.time);out["value"]=scalar_json(p.value);
  out["lower"]=scalar_json(p.lower);out["upper"]=scalar_json(p.upper);
  out["joint"]=p.joint;out["component"]=p.component;out["source_kind"]=p.source_kind;return out;
}
inline Json::Value diagnostics_json(const x650::Diagnostics& d) {
  Json::Value out;
#define SCALAR(name) out[#name]=d.name
  SCALAR(subproblems);SCALAR(active_constraints);SCALAR(solver_mode);SCALAR(qp_rows);SCALAR(qp_columns);SCALAR(interval_rows);SCALAR(interval_canonical_substeps);
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
  ARRAY(interval_centre_plan);ARRAY(interval_canonical_boundaries);ARRAY(interval_canonical_states);ARRAY(interval_canonical_envelope);
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
  out["interval_repairs"]=Json::Value(Json::arrayValue);
  for(const auto& repair:d.interval_repairs){Json::Value v;v["control_subproblem"]=repair.control_subproblem;
    v["added_points"]=repair.added_points;v["total_points"]=repair.total_points;v["canonical_substeps"]=repair.canonical_substeps;
    v["elapsed_ms"]=scalar_json(repair.elapsed_ms);v["rejection"]=repair.rejection;v["envelope"]=numeric_json(repair.envelope);
    v["trigger_points"]=Json::Value(Json::arrayValue);for(const auto& p:repair.trigger_points)v["trigger_points"].append(interval_point_json(p));
    v["centre_plan"]=numeric_json(repair.centre_plan);v["risk_radius"]=scalar_json(repair.risk_radius);v["risk_margin"]=numeric_json(repair.risk_margin);
    v["risk_encoding_lower"]=numeric_json(repair.risk_encoding_lower);v["risk_encoding_upper"]=numeric_json(repair.risk_encoding_upper);
    v["points"]=Json::Value(Json::arrayValue);for(const auto& p:repair.points)v["points"].append(interval_point_json(p));
    out["interval_repairs"].append(std::move(v));}
  out["interval_constraints"]=Json::Value(Json::arrayValue);
  for(const auto& constraint:d.interval_constraints){Json::Value v;v["point"]=interval_point_json(constraint.point);
    v["affine_offset"]=scalar_json(constraint.affine_offset);v["gradient"]=numeric_json(constraint.gradient);v["qp_row"]=constraint.qp_row;
    v["encoding_lower_remainder"]=scalar_json(constraint.encoding_lower_remainder);v["encoding_upper_remainder"]=scalar_json(constraint.encoding_upper_remainder);
    v["encoding_kernel_l1"]=scalar_json(constraint.encoding_kernel_l1);v["encoding_anchor_bias"]=scalar_json(constraint.encoding_anchor_bias);
    v["encoding_effort_half_lsb"]=scalar_json(constraint.encoding_effort_half_lsb);
    out["interval_constraints"].append(std::move(v));}
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
// Shape/identity/finite metadata guard only. The owning Core's canonical native
// parity, hard QP rows and final full-interval candidate gate are authoritative.
// This guard performs no rollout, solver call or state/history modification.
inline bool interval_evidence_valid(const x650::Diagnostics& d,const x650::Config& c) {
  auto finite=[](const auto& values){return std::all_of(values.begin(),values.end(),[](double v){return std::isfinite(v);});};
  auto point=[&](const x650::IntervalPoint& p){
    if(!std::isfinite(p.time)||!std::isfinite(p.value)||!std::isfinite(p.lower)||!std::isfinite(p.upper)
        ||p.joint<0||p.joint>1||p.component<0||p.component>1||p.source_kind<0||p.source_kind>3||p.lower>=p.upper)return false;
    const double lower=p.component?-c.velocity_limit[3+p.joint]+x650::joint_safety::dq_padding:c.q_lower[p.joint]+x650::joint_safety::q_padding;
    const double upper=p.component?c.velocity_limit[3+p.joint]-x650::joint_safety::dq_padding:c.q_upper[p.joint]-x650::joint_safety::q_padding;
    return std::abs(p.lower-lower)<=1e-12&&std::abs(p.upper-upper)<=1e-12;};
  if(d.interval_repairs.empty())return d.interval_rows==0&&d.interval_constraints.empty()
      &&d.interval_centre_plan.empty()&&d.interval_canonical_boundaries.empty()&&d.interval_canonical_states.empty()
      &&d.interval_canonical_substeps==0&&std::all_of(d.interval_canonical_envelope.begin(),d.interval_canonical_envelope.end(),[](double v){return v==0.;});
  if(d.subproblems<2||d.subproblems>3||d.interval_repairs.size()>2||d.interval_rows<=0||d.interval_rows>512
      ||d.interval_constraints.size()!=std::size_t(d.interval_rows)||d.qp_columns!=5*c.nodes||d.qp_rows<d.interval_rows
      ||d.interval_canonical_substeps<=0||!finite(d.interval_canonical_envelope)
      ||!std::isfinite(d.compute_ms)||d.compute_ms<0.||d.compute_ms>50.)return false;
  int previous_subproblem=0,total=0;
  for(const auto& repair:d.interval_repairs){
    if(repair.control_subproblem<=previous_subproblem||repair.control_subproblem>=d.subproblems
        ||repair.added_points<0||repair.total_points!=total+repair.added_points||repair.total_points<1||repair.total_points>512
        ||repair.canonical_substeps<=0||!repair.rejection.empty()||repair.points.empty()||repair.points.size()>512
        ||repair.trigger_points.empty()||repair.trigger_points.size()>8||repair.risk_margin.size()!=repair.points.size()
        ||repair.risk_encoding_lower.size()!=repair.points.size()||repair.risk_encoding_upper.size()!=repair.points.size()
        ||!std::isfinite(repair.risk_radius)||repair.risk_radius!=std::min(c.trust_radius,.02)
        ||repair.centre_plan.size()!=std::size_t(5*c.nodes)||!finite(repair.centre_plan)||!finite(repair.risk_margin)
        ||!finite(repair.risk_encoding_lower)||!finite(repair.risk_encoding_upper)
        ||!std::isfinite(repair.elapsed_ms)||repair.elapsed_ms<0.||repair.elapsed_ms>d.compute_ms||!finite(repair.envelope))return false;
    for(const auto& p:repair.trigger_points)if(!point(p)
        ||!(p.value<p.lower-c.feasibility_tolerance||p.value>p.upper+c.feasibility_tolerance))return false;
    for(std::size_t i=0;i<repair.centre_plan.size();++i)
      if(std::abs(repair.centre_plan[i])>c.velocity_limit[i%5]+1e-10)return false;
    for(std::size_t i=0;i<repair.points.size();++i){const auto& p=repair.points[i];double margin=repair.risk_margin[i];
      const double dlo=repair.risk_encoding_lower[i],dhi=repair.risk_encoding_upper[i];
      if(!point(p)||margin<0.||dlo>dhi)return false;
      bool at_risk=false;for(const auto& trigger:repair.trigger_points)
        if(trigger.joint==p.joint&&trigger.component==p.component)
          at_risk=at_risk||(trigger.value<trigger.lower-c.feasibility_tolerance&&p.value-margin+dlo<=p.lower+c.feasibility_tolerance)
              ||(trigger.value>trigger.upper+c.feasibility_tolerance&&p.value+margin+dhi>=p.upper-c.feasibility_tolerance);
      if(!at_risk)return false;}
    total=repair.total_points;previous_subproblem=repair.control_subproblem;}
  if(total!=d.interval_rows)return false;
  for(std::size_t i=0;i<d.interval_constraints.size();++i){const auto& row=d.interval_constraints[i];
    if(!point(row.point)||!std::isfinite(row.affine_offset)||row.gradient.size()!=std::size_t(5*c.nodes)||!finite(row.gradient)
        ||!std::isfinite(row.encoding_lower_remainder)||!std::isfinite(row.encoding_upper_remainder)
        ||!std::isfinite(row.encoding_kernel_l1)||!std::isfinite(row.encoding_anchor_bias)||!std::isfinite(row.encoding_effort_half_lsb)
        ||row.encoding_kernel_l1<0.||row.encoding_effort_half_lsb<0.
        ||row.qp_row!=d.qp_rows-d.interval_rows+int(i))return false;
    const double encoding_radius=row.encoding_kernel_l1*row.encoding_effort_half_lsb;
    const double expected_half_lsb=.5*(c.kp[row.point.joint]*(2.*std::acos(-1.)/std::pow(2.,24))+c.kd[row.point.joint]/512.);
    if(!std::isfinite(encoding_radius)
        ||std::abs(row.encoding_effort_half_lsb-expected_half_lsb)>1e-12
        ||std::abs(row.encoding_anchor_bias)>encoding_radius+1e-12
        ||std::abs(row.encoding_lower_remainder+encoding_radius+row.encoding_anchor_bias)>1e-12
        ||std::abs(row.encoding_upper_remainder-encoding_radius+row.encoding_anchor_bias)>1e-12)return false;
    double largest=0.;for(double value:row.gradient)largest=std::max(largest,std::abs(value));
    if(largest<=1e-14&&(row.point.value<row.point.lower-c.feasibility_tolerance||row.point.value>row.point.upper+c.feasibility_tolerance))return false;
    for(std::size_t j=0;j<i;++j){const auto& old=d.interval_constraints[j].point;
      if(old.joint==row.point.joint&&old.component==row.point.component&&std::abs(old.time-row.point.time)<=1e-10)return false;}}
  const bool full=!d.interval_centre_plan.empty()||!d.interval_canonical_boundaries.empty()||!d.interval_canonical_states.empty();
  if(full&&(d.interval_centre_plan.size()!=std::size_t(5*c.nodes)||d.interval_canonical_boundaries.empty()
      ||d.interval_canonical_states.size()!=4*d.interval_canonical_boundaries.size()||!finite(d.interval_centre_plan)
      ||!finite(d.interval_canonical_boundaries)||!finite(d.interval_canonical_states)))return false;
  if(full)for(std::size_t i=1;i<d.interval_canonical_boundaries.size();++i)
    if(d.interval_canonical_boundaries[i]<=d.interval_canonical_boundaries[i-1])return false;
  return true;
}
}
