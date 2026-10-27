#include "x650_nmpc/offline_solver.hpp"
#include "synthetic_fixture.hpp"
#include <cmath>
#include <iostream>
void check(bool x,const char* reason){if(!x)throw std::runtime_error(reason);}
template<class F> void rejects(F&& f){bool rejected=false;try{f();}catch(const std::exception&){rejected=true;}check(rejected,"Expected rejection");}
int main(){try{
  const auto c=fixture::config();x650::Core original(c,ts20_prediction_batch);
  original.reset({});original.prime({{10.,{0,0,0,0,0}}},10.);
  const auto before=original.memory();auto in=fixture::input(.031,.031);
  in.target.position[0]+=.003;
  auto direct=original;const auto expected=direct.step(in);
  x650_nmpc::Options one;one.max_proposals=1;one.evaluation_budget_ms=200.;
  const auto a=x650_nmpc::solve_offline(c,ts20_prediction_batch,before,in,one);
  check(a.output.command==expected.command&&a.output.diagnostics.plan==expected.diagnostics.plan,"One proposal changed native plan");
  check(fixture::same_memory(a.proposed_memory,direct.memory()),"One proposal changed native next memory");
  check(fixture::same_memory(before,original.memory()),"Offline solve mutated original");
  x650_nmpc::Options multiple=one;multiple.max_proposals=4;
  const auto b=x650_nmpc::solve_offline(c,ts20_prediction_batch,before,in,multiple);
  check(b.selected_objective.total<=b.first_objective.total,"Nonlinear objective increased");
  check(b.proposed_memory.step_count==before.step_count+1,"Trial iterations became executed steps");
  check(b.proposed_memory.previous_command==b.output.command,"Best command/memory mismatch");
  check(b.proposed_memory.plan==b.output.diagnostics.plan,"Best plan/memory mismatch");
  check(b.proposed_memory.joint_history.size()==before.joint_history.size()+1,"Unsent iterations added history");
  check(!b.approved_for_online_publication,"Offline prototype authorized publication");
  double previous=-1.;for(const auto& it:b.iterations)if(it.accepted){
    check(it.native_passed&&it.stopping_passed,"Unsafe candidate accepted");
    if(previous>=0)check(it.objective.total<previous,"Accepted non-decreasing true cost");previous=it.objective.total;
  }
  auto budget=multiple;budget.evaluation_budget_ms=1e-12;
  rejects([&]{x650_nmpc::solve_offline(c,ts20_prediction_batch,before,in,budget);});
  check(fixture::same_memory(before,original.memory()),"Budget rejection altered history");
  auto stale=in;stale.state.stamp=9.;stale.state.control_time=9.;
  rejects([&]{x650_nmpc::solve_offline(c,ts20_prediction_batch,before,stale,multiple);});
  auto invalid=multiple;invalid.max_proposals=0;
  rejects([&]{x650_nmpc::solve_offline(c,ts20_prediction_batch,before,in,invalid);});
  auto unsafe=expected;unsafe.diagnostics.predicted_states[6]=c.q_upper[0]-.01;unsafe.diagnostics.predicted_states[8]=.1;
  check(x650_nmpc::actual_stopping_violation(c,in,unsafe)>.01,"Actual stopping conflict missed");
  std::cout<<"PASS offline candidate selection, single commit semantics, original history, actual stopping and budget rejection\n";return 0;
}catch(const std::exception&e){std::cerr<<e.what()<<'\n';return 1;}}
