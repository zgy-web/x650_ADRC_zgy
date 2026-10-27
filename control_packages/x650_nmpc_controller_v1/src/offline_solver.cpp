#include "x650_nmpc/offline_solver.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <stdexcept>
extern "C" unsigned x650_optimization_abi_version();

namespace x650_nmpc {
namespace {
using Clock=std::chrono::steady_clock;
double ms(Clock::time_point begin) {
  return std::chrono::duration<double,std::milli>(Clock::now()-begin).count();
}
void require(bool condition,const char* reason) {
  if(!condition) throw std::invalid_argument(reason);
}
}
double actual_stopping_violation(const x650::Config& c,const x650::StepInput& in,
                                const x650::StepResult& output) {
  const auto& states=output.diagnostics.predicted_states;
  require(states.size()==std::size_t(c.nodes*12),"Stopping check needs full predicted states");
  const double age=in.state.control_time-in.state.joint_stamp;
  require(std::isfinite(age)&&age>=0&&age<=.25+1e-10,"Stopping check source age");
  double violation=0.;
  for(int i=0;i<c.nodes;++i)for(int j=0;j<2;++j) {
    const double reserve=c.velocity_limit[3+j]/(2*c.acceleration_limit[3+j])
                       +c.max_dt+c.command_hold_s+age;
    const double value=states[12*i+6+j]+reserve*states[12*i+8+j];
    require(std::isfinite(value),"Stopping check nonfinite actual prediction");
    violation=std::max({violation,c.q_lower[j]-value,value-c.q_upper[j]});
  }
  return violation;
}
OfflineResult solve_offline(const x650::Config& c,x650::Ts20BatchFn fn,
                            const x650::Memory& original,const x650::StepInput& in,
                            const Options& options) {
  const auto begin=Clock::now();
  require(x650_mpc_core_abi_version()==4&&x650_optimization_abi_version()==2,
          "Offline prototype requires core ABI4 / optimizer ABI2");
  require(fn&&options.max_proposals>=1&&options.max_proposals<=16,"Proposal count/predictor");
  require(std::isfinite(options.absolute_decrease)&&options.absolute_decrease>=0&&
          std::isfinite(options.relative_decrease)&&options.relative_decrease>=0&&
          std::isfinite(options.evaluation_budget_ms)&&options.evaluation_budget_ms>0,
          "Invalid nonlinear acceptance/budget options");
  OfflineResult result;
  for(int k=0;k<options.max_proposals;++k) {
    if(ms(begin)>=options.evaluation_budget_ms) {result.stop_reason="evaluation_budget_exhausted";break;}
    Iteration record;record.index=k;
    try {
      // Change a numerical linearization centre, never the observed state,
      // previous executed command, stamp, step count, or actual bus history.
      auto seed=original;
      if(result.selected_iteration>=0) {
        seed.plan=result.output.diagnostics.plan;
        seed.plan_h=result.output.diagnostics.horizon;
        seed.plan_t=in.t;
      }
      x650::Core proposal(c,fn,x650::DiagnosticMode::Full);
      proposal.restore(seed);
      auto output=proposal.step(in);
      record.native_compute_ms=output.diagnostics.compute_ms;
      record.native_passed=output.diagnostics.accepted&&output.diagnostics.nonlinear_passed&&
                           output.diagnostics.actual_bounds&&
                           (!output.diagnostics.hold_guard.required||output.diagnostics.hold_guard.passed);
      record.stopping_violation=actual_stopping_violation(c,in,output);
      record.stopping_passed=record.stopping_violation<=c.feasibility_tolerance;
      record.objective=evaluate_objective(c,original,in,output);
      if(result.selected_iteration>=0) {
        for(std::size_t i=0;i<output.diagnostics.plan.size();++i)
          record.normalized_step_inf=std::max(record.normalized_step_inf,
            std::abs(output.diagnostics.plan[i]-result.output.diagnostics.plan[i])/c.velocity_limit[i%5]);
      }
      if(!record.native_passed||!record.stopping_passed) record.reason="native_or_actual_stopping_check_failed";
      else if(ms(begin)>options.evaluation_budget_ms) record.reason="evaluation_budget_exceeded";
      else if(result.selected_iteration>=0&&record.objective.total>=result.selected_objective.total-
              (options.absolute_decrease+options.relative_decrease*std::abs(result.selected_objective.total)))
        record.reason="no_strict_nonlinear_decrease";
      else {
        record.accepted=true;record.reason=k==0?"first_feasible_proposal":"nonlinear_objective_decreased";
        if(result.selected_iteration<0) result.first_objective=record.objective;
        result.selected_objective=record.objective;result.selected_iteration=k;
        result.output=std::move(output);result.proposed_memory=proposal.memory();
      }
      record.elapsed_ms=ms(begin);result.iterations.push_back(record);
      if(!record.accepted) {result.stop_reason=record.reason;break;}
    } catch(const x650::CoreError& e) {
      record.reason=std::string("native_proposal_failed: ")+e.what();
      record.native_compute_ms=e.diagnostics.compute_ms;record.elapsed_ms=ms(begin);
      result.iterations.push_back(record);result.stop_reason=record.reason;break;
    } catch(const std::exception& e) {
      // Evidence/configuration inconsistency is not ordinary solver stagnation.
      // Preserve the failed evidence but do not return a publishable fallback.
      record.accepted=false;record.reason=std::string("candidate_evaluation_integrity_failure: ")+e.what();
      record.elapsed_ms=ms(begin);result.iterations.push_back(record);
      result.stop_reason=record.reason;result.elapsed_ms=ms(begin);
      result.within_evaluation_budget=result.elapsed_ms<=options.evaluation_budget_ms;
      throw OfflineError(record.reason,std::move(result));
    }
  }
  if(result.stop_reason.empty())result.stop_reason="bounded_proposal_limit";
  result.elapsed_ms=ms(begin);
  result.within_evaluation_budget=result.elapsed_ms<=options.evaluation_budget_ms;
  if(result.selected_iteration<0) {
    const auto reason="No current-state feasible proposal: "+result.stop_reason;
    throw OfflineError(reason,std::move(result));
  }
  return result;
}
} // namespace x650_nmpc
