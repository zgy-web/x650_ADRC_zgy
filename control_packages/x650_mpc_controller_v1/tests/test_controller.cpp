#include "synthetic_fixture.hpp"
#include <iostream>
#include <limits>
#include <string>
void require(bool ok,const char* text){if(!ok)throw std::runtime_error(text);}
template<class F> void rejects(F call){bool failed=false;try{call();}catch(const std::exception&){failed=true;}require(failed,"Expected rejection");}
void same_output(const x650::StepResult&a,const x650::StepResult&b){
 require(a.command==b.command&&a.position_reference==b.position_reference&&a.velocity_feedforward==b.velocity_feedforward&&a.acceleration_feedforward==b.acceleration_feedforward&&a.joint_velocity==b.joint_velocity&&a.joint_reference==b.joint_reference&&a.joint_velocity_reference==b.joint_velocity_reference&&a.joint_acceleration_reference==b.joint_acceleration_reference&&a.nominal_acceleration==b.nominal_acceleration&&a.tip_actual==b.tip_actual,"Output differs from direct native Core");
 const auto& x=a.diagnostics;const auto& y=b.diagnostics;
 require(x.A==y.A&&x.b==y.b&&x.H==y.H&&x.g==y.g&&x.C==y.C&&x.lower==y.lower&&x.upper==y.upper&&x.solution==y.solution&&x.plan==y.plan&&x.predicted_states==y.predicted_states&&x.candidate_envelope==y.candidate_envelope&&x.hold_guard.envelope==y.hold_guard.envelope,"Native matrices/plan/constraints/envelope changed");
}
int main(int argc,char**argv){try{
 require(argc==2,"Named case required");const std::string test=argv[1];
 x650_mpc::Controller c(fixture::config(),ts20_prediction_batch,x650::DiagnosticMode::Full);
 c.reset({},{{10.,{0,0,0,0,0}}},10.);const auto initial=c.committed_memory();
 if(test=="prepare_discard"){
  const auto token=c.prepare(fixture::input()).token;
  require(fixture::same_memory(initial,c.committed_memory()),"Preparation committed an unsent command");
  rejects([&]{c.prepare(fixture::input(.08));});c.discard(token);
  require(fixture::same_memory(initial,c.committed_memory())&&!c.has_pending(),"Discard changed history");
 } else if(test=="equivalence"){
  x650::Core direct(fixture::config(),ts20_prediction_batch);direct.reset({});direct.prime({{10.,{0,0,0,0,0}}},10.);
  auto in=fixture::input();
  for(int i=1;i<=3;++i){in.t=i*.04;in.state.stamp=in.state.control_time=in.state.joint_stamp=10.+in.t;
   const auto expected=direct.step(in);const auto& result=c.prepare(in);same_output(expected,result.output);
   const auto command=result.output.command;c.commit_published(result.token,command);
   require(fixture::same_memory(direct.memory(),c.committed_memory()),"Committed Memory differs");
   in.context.commands.push_back({in.t,{command[3],command[4]}});
  }
 } else if(test=="stale_tokens"){
  auto token=c.prepare(fixture::input()).token;c.discard(token);auto next=c.prepare(fixture::input()).token;
  require(next!=token,"Discarded token reused");rejects([&]{c.commit_published(token,{});});c.discard(next);
  rejects([&]{c.discard(next);});
 } else if(test=="bad_history"){
  const auto token=c.prepare(fixture::input()).token;
  rejects([&]{c.reset({},{{10.,{}},{9.9,{}}},10.);});
  require(c.has_pending()&&fixture::same_memory(initial,c.committed_memory()),"Failed reset mutated state");c.discard(token);
 } else if(test=="native_error"){
  auto in=fixture::input();in.dt=.100001;rejects([&]{c.prepare(in);});
  require(!c.has_pending()&&fixture::same_memory(initial,c.committed_memory()),"Native rejection committed");
  const auto& p=c.prepare(fixture::input());const auto cmd=p.output.command;c.commit_published(p.token,cmd);
  const auto before=c.committed_memory();in=fixture::input(.08);in.state.stamp=9.;rejects([&]{c.prepare(in);});
  require(!c.has_pending()&&fixture::same_memory(before,c.committed_memory()),"Regressed measurement committed");
 } else if(test=="wrong_publication"){
  const auto& p=c.prepare(fixture::input());const auto token=p.token;auto cmd=p.output.command;cmd[0]+=.001;
  rejects([&]{c.commit_published(token,cmd);});require(fixture::same_memory(initial,c.committed_memory()),"Mismatched command committed");
  cmd=p.output.command;c.commit_published(token,cmd);rejects([&]{c.commit_published(token,cmd);});
  require(c.committed_memory().step_count==1,"Double commit advanced state");
 } else if(test=="reset"){
  auto token=c.prepare(fixture::input()).token;c.reset({},{{10.,{0,0,0,0,0}}},10.);
  rejects([&]{c.commit_published(token,{});});const auto next=c.prepare(fixture::input()).token;require(next>token,"Reset reused token");c.discard(next);
  c.restore_for_replay(initial);require(fixture::same_memory(initial,c.committed_memory()),"Replay changed saved memory");
 } else require(false,"Unknown test");
 std::cout<<"PASS "<<test<<'\n';return 0;
 }catch(const std::exception&e){std::cerr<<e.what()<<'\n';return 1;}}
