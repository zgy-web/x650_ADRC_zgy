#include "x650_mpc/common_gate.hpp"
#include "synthetic_fixture.hpp"
#include <chrono>
#include <iostream>
#include <limits>
#include <new>
#include <string>
#include <thread>

namespace {
void check(bool ok,const char* message){if(!ok)throw std::runtime_error(message);}
template<class F> void rejects(F call){bool rejected=false;try{call();}catch(const std::exception&){rejected=true;}check(rejected,"expected transaction rejection");}
int calls=0,slow_at=0,allocation_failure_at=0;
int predictor(const double* model,const double* gains,double h,double start,
 const double* initial,const double* times,const double* forcing,int count,const double* queries,int nq,int track,
 double* states,int* modes,int* zeros,double* envelope,double* extrema,int capacity,int* events,int* steps,char* error,int es){
 ++calls;
 if(calls==slow_at)std::this_thread::sleep_for(std::chrono::milliseconds(55));
 if(calls==allocation_failure_at)throw std::bad_alloc();
 return ts20_prediction_batch(model,gains,h,start,initial,times,forcing,count,queries,nq,track,
  states,modes,zeros,envelope,extrema,capacity,events,steps,error,es);
}
}
int main(int argc,char** argv){try{
 check(argc==2,"named common gate case required");const std::string name=argv[1];
 using Gate=x650_mpc::CommonGateController;
 Gate gate(fixture::config(),predictor);
 gate.reset({},{{10.,{0,0,0,0,0}}},10.);
 const auto original=gate.committed_memory();
 const auto unchanged=[&]{check(fixture::same_memory(original,gate.committed_memory()),"unsent plan committed memory");};
 const auto prepare=[&]{return gate.prepare(fixture::input(),Gate::monotonic_ns());};
 if(name=="deadline_precheck"){
  auto value=gate.prepare(fixture::input(),Gate::monotonic_ns(),0);
  check(!value.accepted&&value.status=="invalid_budget"&&!value.solver_called&&!gate.has_pending(),"zero budget solved");
  value=gate.prepare(fixture::input(),Gate::monotonic_ns()-60'000'000);
  check(!value.accepted&&value.status=="issue_expired_before_solve"&&!value.solver_called,"stale issue solved");
  value=gate.prepare(fixture::input(),0);check(value.status=="invalid_issue_time"&&!value.solver_called,"zero issue accepted");
  value=gate.prepare(fixture::input(),std::numeric_limits<std::uint64_t>::max());
  check(value.status=="invalid_issue_time"&&!value.solver_called,"future/overflow issue accepted");
  value=gate.prepare(fixture::input(),Gate::monotonic_ns(),Gate::maximum_budget_ns+1);
  check(value.status=="invalid_budget"&&calls==0,"extended original issue budget allowed");unchanged();
 }else if(name=="safe_transaction"){
  x650::Core direct(fixture::config(),ts20_prediction_batch,x650::DiagnosticMode::Full);
  direct.restore(original);const auto expected=direct.step(fixture::input());
  const auto value=prepare();check(value.accepted&&value.token&&value.output&&value.validation_called&&value.validation.passed&&gate.has_pending(),"safe native plan rejected");
  check(value.output->command==expected.command&&value.output->diagnostics.plan==expected.diagnostics.plan,"common gate modified optimizer plan");
  check(value.validation.query_states==expected.diagnostics.candidate_states&&value.validation.envelope==expected.diagnostics.candidate_envelope,"common gate propagation differs");
  check(value.timing.completed_ns<=value.timing.deadline_monotonic_ns&&value.timing.validation_started_ns>=value.timing.core_completed_ns,"timing provenance");unchanged();
  gate.commit_published(value.token,value.output->command);
  check(!gate.has_pending()&&fixture::same_memory(direct.memory(),gate.committed_memory()),"actual publication did not commit exact native memory");
  rejects([&]{gate.commit_published(value.token,value.output->command);});
 }else if(name=="wrong_publication"){
  const auto value=prepare();check(value.accepted,"safe prepare");auto wrong=value.output->command;wrong[3]+=.001;
  rejects([&]{gate.commit_published(value.token,wrong);});unchanged();check(gate.has_pending(),"wrong publication erased outstanding candidate");
  gate.discard(value.token);rejects([&]{gate.commit_published(value.token,value.output->command);});unchanged();
  const auto next=prepare();check(next.accepted&&next.token!=value.token,"token reused");
  rejects([&]{gate.commit_published(value.token,value.output->command);});gate.discard(next.token);unchanged();
 }else if(name=="native_rejection"){
  auto input=fixture::input();input.dt=.100001;
  const auto failed=gate.prepare(input,Gate::monotonic_ns());
  check(!failed.accepted&&failed.status=="legacy_solver_rejected"&&!failed.token&&!gate.has_pending(),"native rejection left candidate");unchanged();
  rejects([&]{gate.commit_published(failed.token,{});});
  const auto valid=prepare();check(valid.accepted,"rejection poisoned next prepare");gate.discard(valid.token);unchanged();
 }else if(name=="pending_reset"){
  const auto first=prepare();check(first.accepted,"safe prepare");
  const auto blocked=prepare();check(!blocked.accepted&&!blocked.solver_called&&blocked.status=="unresolved_pending_candidate"&&gate.has_pending(),"unresolved proposal implicitly discarded");unchanged();
  rejects([&]{gate.reset({},{{10.,{}},{9.,{}}},10.);});check(gate.has_pending(),"failed reset erased pending");
  gate.reset({},{{10.,{0,0,0,0,0}}},10.);rejects([&]{gate.commit_published(first.token,first.output->command);});unchanged();
  const auto next=prepare();check(next.accepted&&next.token>first.token,"reset reused token");gate.restore_for_replay(original);
  rejects([&]{gate.commit_published(next.token,next.output->command);});check(!gate.has_pending(),"restore kept candidate");unchanged();
 }else if(name=="deadline_after_validation"||name=="allocation_cleanup"){
  // Count the real core calls first, then inject delay/allocation failure only
  // when the pure validation stage begins. No artificial actuator state/history.
  x650::Core direct(fixture::config(),predictor,x650::DiagnosticMode::Full);direct.restore(original);
  calls=0;(void)direct.step(fixture::input());const int core_calls=calls;check(core_calls>0,"missing real predictor work");calls=0;
  if(name=="deadline_after_validation"){
   slow_at=core_calls+1;const auto value=prepare();slow_at=0;
   check(!value.accepted&&value.status=="issue_expired_after_validation"&&value.validation_called&&value.validation.passed&&!value.token&&!gate.has_pending(),"validation overrun admitted/left pending");
   check(value.timing.completed_ns>value.timing.deadline_monotonic_ns,"overrun not recorded");
  }else{
   allocation_failure_at=core_calls+1;bool caught=false;try{(void)prepare();}catch(const std::bad_alloc&){caught=true;}
   allocation_failure_at=0;check(caught&&!gate.has_pending(),"exception left a prepared candidate");
  }
  unchanged();calls=0;const auto valid=prepare();check(valid.accepted,"exception/deadline poisoned next prepare");gate.discard(valid.token);unchanged();
 }else check(false,"unknown common gate test");
 std::cout<<"PASS common_gate_"<<name<<'\n';return 0;
 }catch(const std::exception& error){std::cerr<<error.what()<<'\n';return 1;}}
