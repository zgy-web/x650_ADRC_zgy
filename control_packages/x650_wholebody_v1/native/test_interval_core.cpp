// Transaction/parity/deadline tests use the real native TS20 integrator.
// A deliberately falsified wrapper is a rejection counterexample only.
#include "mpc_core.hpp"
#include <chrono>
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string>
#include <thread>

extern "C" int ts20_prediction_batch(const double*,const double*,double,double,
 const double*,const double*,const double*,int,const double*,int,int,
 double*,int*,int*,double*,double*,int,int*,int*,char*,int);

namespace {
void check(bool value,const char* message){if(!value)throw std::runtime_error(message);}
bool same_memory(const x650::Memory& a,const x650::Memory& b){
 if(a.initialized!=b.initialized||a.history_primed!=b.history_primed||a.has_last_t!=b.has_last_t||a.has_stamp!=b.has_stamp||a.has_origin!=b.has_origin||
    a.anchor.position!=b.anchor.position||a.anchor.rotation!=b.anchor.rotation||a.anchor.q!=b.anchor.q||a.previous_command!=b.previous_command||
    a.last_t!=b.last_t||a.previous_stamp!=b.previous_stamp||a.origin!=b.origin||a.plan_t!=b.plan_t||a.step_count!=b.step_count||
    a.plan!=b.plan||a.plan_h!=b.plan_h||a.joint_history.size()!=b.joint_history.size())return false;
 for(std::size_t i=0;i<a.joint_history.size();++i)if(a.joint_history[i].time!=b.joint_history[i].time||a.joint_history[i].velocity!=b.joint_history[i].velocity)return false;
 return true;
}
x650::Config config(){x650::Config c;c.nodes=3;c.geometry={.2,.2,-.7853981633974483,.7853981633974483,0,0,0,0,0,0};
 c.physical_lower={-2,-2};c.physical_upper={2,2};c.response={4.,.8,4.,.8,.25};c.base_position_gain={1,1,1};c.motion={.01,.1,.02,0,.015,.03,.005,.01};return c;}
x650::StepInput input(double time=.02){x650::StepInput in;in.t=time;in.dt=.02;in.state.stamp=in.state.control_time=in.state.joint_stamp=time;
 in.target.position={.4*std::cos(.7853981633974483),0,0};in.context.reference_time=0;in.context.commands={{0,{0,0}}};
 for(int i=-20;i<=0;++i){x650::Packet p;p.send_time=i*.005;p.kp={6,6};p.kd={.25,.25};in.context.packets.push_back(p);}return in;}
void start(x650::Core& core){core.reset({});core.prime({{0,{0,0,0,0,0}}},0);}
bool falsify_envelope=false,delay_envelope=false;
int checked_wrapper(const double* model,const double* gains,double step,double begin,
 const double* initial,const double* knots,const double* forcing,int packets,const double* boundaries,int count,int track,
 double* states,int* modes,int* zeros,double* envelope,double* events,int capacity,int* event_count,int* substeps,char* error,int error_size){
 int code=ts20_prediction_batch(model,gains,step,begin,initial,knots,forcing,packets,boundaries,count,track,
     states,modes,zeros,envelope,events,capacity,event_count,substeps,error,error_size);
 if(!code&&track){
  if(falsify_envelope)envelope[6]=.43; // native states/events do not support this claim
  if(delay_envelope){delay_envelope=false;std::this_thread::sleep_for(std::chrono::milliseconds(55));}
 }
 return code;
}
template<class Action> x650::Diagnostics rejects_unchanged(x650::Core& core,Action action,const char* required=nullptr){
 auto before=core.memory();bool rejected=false;x650::Diagnostics diagnostic;
 try{action();}catch(const x650::CoreError& failure){rejected=true;diagnostic=failure.diagnostics;
  if(required)check(std::string(failure.what()).find(required)!=std::string::npos,"unexpected rejection reason");}
 check(rejected,"unsafe/falsified/late candidate unexpectedly accepted");
 check(!diagnostic.accepted&&!diagnostic.previous_plan_fallback,"rejection must not become accepted fallback");
 check(diagnostic.subproblems<=3,"rejection exceeded three-control-QP budget");
 check(same_memory(before,core.memory()),"rejected interval repair advanced command, clock, history or plan memory");return diagnostic;
}
}

int main(){try{
 auto c=config();x650::Core full(c,ts20_prediction_batch),compact(c,ts20_prediction_batch,x650::DiagnosticMode::Compact);
 start(full);start(compact);auto a=full.step(input()),b=compact.step(input());
 check(a.diagnostics.accepted&&b.diagnostics.accepted&&a.diagnostics.subproblems==1&&b.diagnostics.subproblems==1,"normal real-TS20 hold changed its single-QP path");
 check(a.diagnostics.interval_rows==0&&b.diagnostics.interval_rows==0&&a.diagnostics.interval_repairs.empty()&&b.diagnostics.interval_repairs.empty(),"normal path acquired unsolicited interval rows or repair work");
 check(a.command==b.command&&a.diagnostics.plan==b.diagnostics.plan&&a.diagnostics.predicted_states==b.diagnostics.predicted_states&&a.diagnostics.candidate_envelope==b.diagnostics.candidate_envelope,"normal full/compact outputs differ");
 check(same_memory(full.memory(),compact.memory()),"normal full/compact transaction differs");
 for(double value:a.command)check(std::abs(value)<1e-10,"quiet real-TS20 hold acquired an unsolicited command");

 // The initial velocity .43 is below the original command limit .45 but
 // above the project's configured padded state envelope .45-.03=.42. These
 // simulation bounds are not identified physical motor limits. It cannot be fixed
 // retroactively, even if all future prediction nodes brake into the bound.
 auto initial_bad=input(.04);initial_bad.state.dq[0]=.43;
 auto state_rejection=rejects_unchanged(full,[&]{full.step(initial_bad);});
 check(state_rejection.subproblems>=1,"actual initial-envelope counterexample did not reach native prediction");

 // The canonical observer may not trust a plug-in envelope invented by a
 // callback. The wrapper below runs real TS20, then corrupts only its bound.
 x650::Core parity(c,checked_wrapper);start(parity);parity.step(input());
 falsify_envelope=true;
 rejects_unchanged(parity,[&]{parity.step(input(.04));},"interval observer differs from canonical native rollout");
 falsify_envelope=false;
 auto after_parity=parity.step(input(.04));check(after_parity.diagnostics.accepted&&parity.memory().step_count==2,"parity rejection consumed the input clock");

 x650::Core timed(c,checked_wrapper);start(timed);timed.step(input());delay_envelope=true;
 auto deadline=rejects_unchanged(timed,[&]{timed.step(input(.04));},"deadline");
 check(!delay_envelope&&deadline.compute_ms>=50.,"original 50 ms gate did not reject the actually delayed native call");
 auto after_deadline=timed.step(input(.04));check(after_deadline.diagnostics.accepted&&timed.memory().step_count==2,"deadline rejection consumed the input clock");
 check(x650_mpc_core_abi_version()==4,"interval diagnostics need the explicit ABI 3 worker");
 std::cout<<"PASS: real TS20 normal path/full-compact equality, uncontrollable initial envelope rejection, canonical observer parity rejection, original 50 ms gate and complete transactional rollback\n";return 0;
 }catch(const std::exception& error){std::cerr<<error.what()<<'\n';return 1;}}
