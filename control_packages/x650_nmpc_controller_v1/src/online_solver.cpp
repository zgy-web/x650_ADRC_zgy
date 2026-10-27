#include "x650_nmpc/online_solver.hpp"
#include "joint_safety.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace x650_nmpc {
namespace {
using Clock=std::chrono::steady_clock;
std::uint64_t monotonic_ns(){return std::chrono::duration_cast<std::chrono::nanoseconds>(Clock::now().time_since_epoch()).count();}
void require(bool condition,const char* reason){if(!condition)throw std::invalid_argument(reason);}
template<class T>bool finite(const T& values){return std::all_of(values.begin(),values.end(),[](double x){return std::isfinite(x);});}
bool same_packet(const x650::Packet& a,const x650::Packet& b){
  return a.send_time==b.send_time&&a.q_ref==b.q_ref&&a.dq_ref==b.dq_ref&&
         a.tau_ff==b.tau_ff&&a.kp==b.kp&&a.kd==b.kd;
}
void check_history_prefix(const std::vector<x650::Packet>& history,const std::vector<x650::Packet>& candidate){
  require(candidate.size()>=history.size(),"online actual packet history missing");
  for(std::size_t i=0;i<history.size();++i)require(same_packet(history[i],candidate[i]),"online actual packet history changed");
}
void check_envelope(const x650::Config& c,const std::array<double,8>& envelope){
  using namespace x650::joint_safety;
  require(finite(envelope),"online native envelope nonfinite");
  for(int j=0;j<2;++j){
    require(envelope[j]<=envelope[j+2]&&envelope[j+4]<=envelope[j+6],"online native envelope order");
    require(envelope[j]-q_padding>=c.q_lower[j]-bound_tolerance&&
            envelope[j+2]+q_padding<=c.q_upper[j]+bound_tolerance&&
            envelope[j+4]-dq_padding>=-c.velocity_limit[j+3]-bound_tolerance&&
            envelope[j+6]+dq_padding<=c.velocity_limit[j+3]+bound_tolerance,
            "online native hard envelope failed");
  }
}
void check_proposed_memory(const x650::Memory& original,const x650::StepInput& in,
                           const x650::StepResult& output,const x650::Memory& next){
  require(original.step_count<std::numeric_limits<std::uint64_t>::max(),"online step counter exhausted");
  require(next.initialized==original.initialized&&next.history_primed==original.history_primed&&
          next.anchor.position==original.anchor.position&&next.anchor.rotation==original.anchor.rotation&&
          next.anchor.q==original.anchor.q,"online proposed anchor/lifecycle changed");
  require(next.has_last_t&&next.last_t==in.t&&next.has_stamp&&next.previous_stamp==in.state.stamp&&
          next.has_origin&&next.origin==in.state.control_time-in.t&&next.plan_t==in.t&&
          next.step_count==original.step_count+1,"online proposed clock/step identity");
  require(next.previous_command==output.command&&next.plan==output.diagnostics.plan&&
          next.plan_h==output.diagnostics.horizon,"online selected command/memory mismatch");
  auto history=original.joint_history;history.push_back({in.t,output.joint_velocity});
  history.erase(std::remove_if(history.begin(),history.end(),[&](const x650::Command& x){return x.time<in.t-.75;}),history.end());
  require(history.size()==next.joint_history.size(),"online unsent trial entered joint history");
  for(std::size_t i=0;i<history.size();++i)
    require(history[i].time==next.joint_history[i].time&&history[i].velocity==next.joint_history[i].velocity,
            "online committed joint history changed");
}
void check_selected_candidate(const x650::Config& c,const x650::Memory& original,
                              const x650::StepInput& in,const OfflineResult& solve,
                              double allowed_proposal_budget,double& stopping){
  require(!solve.approved_for_online_publication,"online offline result flag unexpectedly changed");
  require(solve.selected_iteration>=0&&std::size_t(solve.selected_iteration)<solve.iterations.size(),"online no selected candidate");
  const auto& selected=solve.iterations[solve.selected_iteration];
  require(selected.index==solve.selected_iteration&&selected.accepted&&selected.native_passed&&
          selected.stopping_passed&&std::isfinite(selected.elapsed_ms)&&
          selected.elapsed_ms<=allowed_proposal_budget,"online selected iteration invalid or late");
  const auto& output=solve.output;const auto& d=output.diagnostics;
  require(finite(output.command)&&finite(output.position_reference)&&finite(output.velocity_feedforward)&&
          finite(output.acceleration_feedforward)&&finite(output.joint_velocity)&&finite(output.joint_reference)&&
          finite(output.joint_velocity_reference)&&finite(output.joint_acceleration_reference)&&
          finite(output.nominal_acceleration)&&finite(output.tip_actual),"online output nonfinite");
  require(output.acceleration_feedforward==x650::V3{},"online zero acceleration encoding contract");
  require(output.joint_velocity==x650::V2{output.command[3],output.command[4]},"online joint command mismatch");
  require(d.accepted&&d.nonlinear_passed&&d.actual_bounds&&std::isfinite(d.primal_violation)&&
          d.primal_violation<=c.feasibility_tolerance&&std::isfinite(d.kkt_stationarity)&&
          d.kkt_stationarity<=c.kkt_tolerance&&std::isfinite(d.compute_ms)&&d.compute_ms<=50.,
          "online native candidate gate failed");
  check_envelope(c,d.candidate_envelope);check_history_prefix(in.context.packets,d.candidate_packets);
  const auto& guard=d.hold_guard;
  require(guard.required&&guard.checked&&guard.passed&&guard.duration_s==c.command_hold_s&&
          guard.start_task_s==in.t&&guard.end_task_s==in.t+c.command_hold_s&&
          guard.source_control_time_s==in.state.control_time&&guard.source_joint_time_s==in.state.joint_stamp&&
          guard.command==output.joint_velocity&&guard.actual_history_packet_count==int(in.context.packets.size()),
          "online held-command source identity failed");
  check_envelope(c,guard.envelope);check_history_prefix(in.context.packets,guard.packets);
  for(int j=0;j<2;++j)require(guard.reference_initial[j]>=c.q_lower[j]-c.feasibility_tolerance&&
          guard.reference_initial[j]<=c.q_upper[j]+c.feasibility_tolerance&&
          guard.reference_final[j]>=c.q_lower[j]-c.feasibility_tolerance&&
          guard.reference_final[j]<=c.q_upper[j]+c.feasibility_tolerance,"online held integral reference bound");
  stopping=actual_stopping_violation(c,in,output);
  require(std::isfinite(stopping)&&stopping<=c.feasibility_tolerance,"online actual stopping check failed");
  const auto scored=evaluate_objective(c,original,in,output);
  require(scored.total==solve.selected_objective.total&&scored.components==solve.selected_objective.components&&
          scored.endpoint_positions==solve.selected_objective.endpoint_positions&&
          scored.endpoint_velocities==solve.selected_objective.endpoint_velocities,
          "online selected nonlinear objective changed");
  require(scored.total==selected.objective.total&&scored.components==selected.objective.components,
          "online selected iteration/objective mismatch");
  check_proposed_memory(original,in,output,solve.proposed_memory);
}
}
OnlineResult solve_online(const x650::Config& c,x650::Ts20BatchFn predictor,
                          const x650::Memory& original,const x650::StepInput& in,
                          std::uint64_t issue_ns,const OnlineOptions& options){
  const auto started=monotonic_ns();OnlineResult out;
  auto finish=[&](bool accepted,const std::string& status){
    out.completed_monotonic_ns=monotonic_ns();
    out.call_elapsed_ms=double(out.completed_monotonic_ns-started)*1e-6;
    out.issue_age_ms=issue_ns&&out.completed_monotonic_ns>=issue_ns?
      double(out.completed_monotonic_ns-issue_ns)*1e-6:std::numeric_limits<double>::infinity();
    out.accepted=accepted&&out.issue_age_ms<=options.total_budget_ms;
    out.status=accepted&&!out.accepted?"request_deadline_exceeded":status;
    return std::move(out);
  };
  if(!issue_ns||issue_ns>started)return finish(false,"invalid_original_issue_timestamp");
  if(!std::isfinite(options.total_budget_ms)||options.total_budget_ms<0||options.total_budget_ms>50.||
     !std::isfinite(options.nonlinear.evaluation_budget_ms)||options.nonlinear.evaluation_budget_ms<=0||
     options.nonlinear.evaluation_budget_ms>40.||options.nonlinear.max_proposals<1||options.nonlinear.max_proposals>4||
     !std::isfinite(options.nonlinear.absolute_decrease)||options.nonlinear.absolute_decrease<0||
     !std::isfinite(options.nonlinear.relative_decrease)||options.nonlinear.relative_decrease<0)
    return finish(false,"invalid_online_options");
  const double age_at_entry=double(started-issue_ns)*1e-6;
  if(age_at_entry>=options.total_budget_ms)return finish(false,"request_deadline_exceeded_at_entry");
  if(c.nodes!=15||c.node_dt!=.04||c.max_dt!=.1||c.command_hold_s!=.16)
    return finish(false,"unsupported_multirate_configuration");
  if(!original.initialized||!original.history_primed||!original.has_origin)
    return finish(false,"unprimed_committed_history");
  Options bounded=options.nonlinear;
  bounded.evaluation_budget_ms=std::min(bounded.evaluation_budget_ms,options.total_budget_ms-age_at_entry);
  try{
    out.solve=solve_offline(c,predictor,original,in,bounded);
  }catch(const OfflineError& e){
    out.solve=e.diagnostics;
    return finish(false,out.solve.selected_iteration<0?"no_current_state_feasible_candidate":"nonlinear_evaluation_integrity_failure");
  }catch(const std::exception& e){
    out.solve.stop_reason=e.what();return finish(false,"online_input_or_solver_exception");
  }
  // A second proposal can fail after an earlier safe proposal was selected. It
  // never replaces that pair. Retention is allowed only while the ORIGINAL
  // request is still fresh; an overlong native call is never published late.
  if(double(monotonic_ns()-issue_ns)*1e-6>options.total_budget_ms)
    return finish(false,"request_deadline_exceeded_after_solve");
  try{
    check_selected_candidate(c,original,in,out.solve,bounded.evaluation_budget_ms,out.actual_stopping_violation);
  }catch(const std::exception& e){
    out.solve.stop_reason=std::string("online_selected_candidate_integrity_failure: ")+e.what();
    return finish(false,"selected_candidate_integrity_failure");
  }
  return finish(true,"ready_checked_native_candidate");
}
} // namespace x650_nmpc
