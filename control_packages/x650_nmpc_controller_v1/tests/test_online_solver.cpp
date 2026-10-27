#include "x650_nmpc/online_solver.hpp"
#include "synthetic_fixture.hpp"
#include <chrono>
#include <cmath>
#include <cstdio>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <thread>
namespace {
using Clock=std::chrono::steady_clock;
std::uint64_t now(){return std::chrono::duration_cast<std::chrono::nanoseconds>(Clock::now().time_since_epoch()).count();}
void check(bool value,const char* reason){if(!value)throw std::runtime_error(reason);}
bool fail_later=false,first_hold_seen=false;
int later_failure_sleep_ms=0,later_failures=0;
double hold_end=0.;
int predictor(const double*p,const double*g,double maxstep,double start,
  const double*initial,const double*knots,const double*forcing,int packets,
  const double*boundaries,int count,int track,double*states,int*modes,int*zeros,
  double*envelope,double*events,int capacity,int*event_count,int*substeps,char*error,int error_size){
  if(fail_later&&first_hold_seen&&!track){
    ++later_failures;
    if(later_failure_sleep_ms)std::this_thread::sleep_for(std::chrono::milliseconds(later_failure_sleep_ms));
    std::snprintf(error,error_size,"forced later current-state proposal failure");return 1;
  }
  const int code=ts20_prediction_batch(p,g,maxstep,start,initial,knots,forcing,packets,boundaries,count,track,
                                     states,modes,zeros,envelope,events,capacity,event_count,substeps,error,error_size);
  if(!code&&track&&count&&std::abs(boundaries[count-1]-hold_end)<1e-10)first_hold_seen=true;
  return code;
}
bool same_outputs(const x650::StepResult&a,const x650::StepResult&b){
  return a.command==b.command&&a.position_reference==b.position_reference&&
    a.velocity_feedforward==b.velocity_feedforward&&a.acceleration_feedforward==b.acceleration_feedforward&&
    a.joint_velocity==b.joint_velocity&&a.joint_reference==b.joint_reference&&
    a.joint_velocity_reference==b.joint_velocity_reference&&a.joint_acceleration_reference==b.joint_acceleration_reference&&
    a.nominal_acceleration==b.nominal_acceleration&&a.tip_actual==b.tip_actual&&
    a.diagnostics.plan==b.diagnostics.plan&&a.diagnostics.predicted_states==b.diagnostics.predicted_states&&
    a.diagnostics.candidate_envelope==b.diagnostics.candidate_envelope;
}
}
int main(){try{
  const auto c=fixture::config();x650::Core owner(c,ts20_prediction_batch);
  owner.reset({});owner.prime({{10.,{0,0,0,0,0}}},10.);
  const auto before=owner.memory();auto in=fixture::input(.031,.031);in.target.position[0]+=.003;
  const auto input_copy=in;
  auto oracle=owner;const auto expected=oracle.step(in);
  x650_nmpc::OnlineOptions one;one.nonlinear.max_proposals=1;
  const auto accepted=x650_nmpc::solve_online(c,ts20_prediction_batch,before,in,now(),one);
  if(!accepted.accepted)std::cerr<<accepted.status<<": "<<accepted.solve.stop_reason<<" age="<<accepted.issue_age_ms<<'\n';
  check(accepted.accepted&&accepted.issue_age_ms<=50.&&accepted.completed_monotonic_ns>0,"fresh valid online result rejected");
  check(same_outputs(accepted.solve.output,expected),"online wrapper changed exact native output");
  check(fixture::same_memory(accepted.solve.proposed_memory,oracle.memory()),"online command/proposed-memory pair differs");
  check(!accepted.solve.approved_for_online_publication,"online wrapper blindly changed offline flag");
  check(fixture::same_memory(before,owner.memory()),"online solve committed caller memory");
  check(in.t==input_copy.t&&in.dt==input_copy.dt&&in.state.q==input_copy.state.q&&
        in.context.commands.size()==input_copy.context.commands.size()&&
        in.context.packets.size()==input_copy.context.packets.size(),"online solve modified input");
  const auto zero_issue=x650_nmpc::solve_online(c,ts20_prediction_batch,before,in,0,one);
  check(!zero_issue.accepted&&zero_issue.status=="invalid_original_issue_timestamp"&&zero_issue.solve.iterations.empty(),"zero original issue accepted");
  const auto future=x650_nmpc::solve_online(c,ts20_prediction_batch,before,in,now()+1'000'000'000,one);
  check(!future.accepted&&future.status=="invalid_original_issue_timestamp","future issue accepted");
  const auto stale=x650_nmpc::solve_online(c,ts20_prediction_batch,before,in,now()-51'000'000,one);
  check(!stale.accepted&&stale.solve.iterations.empty()&&stale.issue_age_ms>=51.,"expired request recalculated or accepted");
  auto zero=one;zero.total_budget_ms=0.;
  const auto zero_budget=x650_nmpc::solve_online(c,ts20_prediction_batch,before,in,now(),zero);
  check(!zero_budget.accepted&&zero_budget.solve.iterations.empty(),"zero total budget ran solver");
  auto relaxed=one;relaxed.total_budget_ms=51.;
  check(!x650_nmpc::solve_online(c,ts20_prediction_batch,before,in,now(),relaxed).accepted,"online hard deadline relaxed");
  auto unprimed=before;unprimed.history_primed=false;
  const auto no_history=x650_nmpc::solve_online(c,ts20_prediction_batch,unprimed,in,now(),one);
  check(!no_history.accepted&&no_history.status=="unprimed_committed_history"&&no_history.solve.iterations.empty(),
        "unprimed history admitted online");
  auto bad=in;bad.state.position[0]=std::numeric_limits<double>::quiet_NaN();
  const auto invalid=x650_nmpc::solve_online(c,ts20_prediction_batch,before,bad,now(),one);
  check(!invalid.accepted&&invalid.solve.selected_iteration<0,"nonfinite state accepted");
  auto outside=in;outside.state.q[0]=c.q_upper[0]+.02;
  const auto infeasible=x650_nmpc::solve_online(c,ts20_prediction_batch,before,outside,now(),one);
  check(!infeasible.accepted&&infeasible.solve.selected_iteration<0&&
        infeasible.status=="no_current_state_feasible_candidate","unsafe initial state did not fail closed");
  // A rejected later proposal must retain exactly the first feasible pair,
  // without silently adopting the unsuccessful trial's numerical state.
  auto multiple=one;multiple.nonlinear.max_proposals=4;
  fail_later=true;first_hold_seen=false;hold_end=in.t+c.command_hold_s;later_failure_sleep_ms=1;later_failures=0;
  const auto retained=x650_nmpc::solve_online(c,predictor,before,in,now(),multiple);
  if(!retained.accepted)std::cerr<<retained.status<<": "<<retained.solve.stop_reason<<" age="<<retained.issue_age_ms<<'\n';
  check(retained.accepted&&later_failures==1&&retained.solve.selected_iteration==0&&retained.solve.iterations.size()==2,
        "fresh earlier feasible candidate was not retained after later rejection");
  check(same_outputs(retained.solve.output,expected)&&fixture::same_memory(retained.solve.proposed_memory,oracle.memory()),
        "later failed proposal replaced selected output/history");
  // The exact same retained numerical plan must NOT acquire publication
  // permission when the later call consumes the original request deadline.
  first_hold_seen=false;later_failure_sleep_ms=55;later_failures=0;
  const auto expired=x650_nmpc::solve_online(c,predictor,before,in,now(),multiple);
  fail_later=false;
  check(!expired.accepted&&later_failures==1&&expired.issue_age_ms>50.&&expired.solve.selected_iteration==0&&
        expired.status=="request_deadline_exceeded_after_solve","expired selected candidate was publishable");
  check(same_outputs(expired.solve.output,expected)&&fixture::same_memory(expired.solve.proposed_memory,oracle.memory()),
        "late failure destroyed retained diagnostic pair");
  check(fixture::same_memory(before,owner.memory()),"rejected solve mutated caller state");
  std::cout<<"PASS online wrapper: original issue deadline, exact selected pair, zero budget, invalid/unsafe input, fresh retention, expired refusal and no commit\n";
  return 0;
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
