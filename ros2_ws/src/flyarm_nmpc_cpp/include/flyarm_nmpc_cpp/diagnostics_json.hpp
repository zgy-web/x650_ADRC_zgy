#pragma once
#include "flyarm_mpc_cpp/diagnostics_json.hpp"
#include "x650_nmpc/online_solver.hpp"

namespace flyarm_nmpc_cpp {
namespace fm=flyarm_mpc_cpp;
inline Json::Value objective_json(const x650_nmpc::Objective& objective) {
  Json::Value out;out["total"]=fm::scalar_json(objective.total);
  out["components"]=fm::numeric_json(objective.components);return out;
}
inline Json::Value diagnostics_json(const x650_nmpc::OnlineResult& value) {
  Json::Value out;out["kind"]="x650_online_nmpc_call_v1";
  out["accepted_by_solver"]=value.accepted;out["status"]=value.status;
  out["call_elapsed_ms"]=fm::scalar_json(value.call_elapsed_ms);
  const bool valid_issue_age=std::isfinite(value.issue_age_ms);
  if(value.accepted&&!valid_issue_age)throw std::runtime_error("Accepted NMPC result lacks a finite issue age");
  out["issue_age_ms"]=valid_issue_age?Json::Value(value.issue_age_ms):Json::Value();
  out["issue_age_finite"]=valid_issue_age;
  out["completed_monotonic_ns"]=Json::UInt64(value.completed_monotonic_ns);
  out["actual_stopping_violation"]=fm::scalar_json(value.actual_stopping_violation);
  const auto& solve=value.solve;out["selected_iteration"]=solve.selected_iteration;
  out["stop_reason"]=solve.stop_reason;out["nonlinear_total_ms"]=fm::scalar_json(solve.elapsed_ms);
  out["within_proposal_budget"]=solve.within_evaluation_budget;
  out["offline_publication_flag"]=solve.approved_for_online_publication;
  out["first_objective"]=objective_json(solve.first_objective);
  out["selected_objective"]=objective_json(solve.selected_objective);
  out["iterations"]=Json::Value(Json::arrayValue);
  for(const auto& iteration:solve.iterations){Json::Value row;
    row["index"]=iteration.index;row["accepted"]=iteration.accepted;
    row["native_passed"]=iteration.native_passed;row["stopping_passed"]=iteration.stopping_passed;
    row["native_compute_ms"]=fm::scalar_json(iteration.native_compute_ms);
    row["elapsed_ms"]=fm::scalar_json(iteration.elapsed_ms);
    row["stopping_violation"]=fm::scalar_json(iteration.stopping_violation);
    row["normalized_step_inf"]=fm::scalar_json(iteration.normalized_step_inf);
    row["objective"]=objective_json(iteration.objective);row["reason"]=iteration.reason;
    out["iterations"].append(row);
  }
  return out;
}
}  // namespace flyarm_nmpc_cpp
