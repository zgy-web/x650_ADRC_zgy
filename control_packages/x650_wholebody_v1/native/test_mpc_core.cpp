#include "mpc_core.hpp"
#include <cmath>
#include <chrono>
#include <iostream>
#include <limits>
#include <thread>

extern "C" int ts20_prediction_batch(const double*,const double*,double,double,
 const double*,const double*,const double*,int,const double*,int,int,
 double*,int*,int*,double*,double*,int,int*,int*,char*,int);
namespace {
bool delay_next_prediction=false;
int delayed_prediction(const double* model,const double* initial,double begin,double end,
 const double* sends,const double* positions,const double* forcing,int count,const double* queries,int query_count,int track_extrema,
 double* states,int* modes,int* zero_events,double* extrema,double* boundaries,int capacity,int* boundary_count,int* substeps,char* error,int error_capacity){
 if(delay_next_prediction){delay_next_prediction=false;std::this_thread::sleep_for(std::chrono::milliseconds(55));}
 return ts20_prediction_batch(model,initial,begin,end,sends,positions,forcing,count,queries,query_count,track_extrema,
   states,modes,zero_events,extrema,boundaries,capacity,boundary_count,substeps,error,error_capacity);
}
bool same_memory(const x650::Memory& a,const x650::Memory& b){
 if(a.initialized!=b.initialized||a.history_primed!=b.history_primed||a.has_last_t!=b.has_last_t||a.has_stamp!=b.has_stamp||a.has_origin!=b.has_origin||
    a.anchor.position!=b.anchor.position||a.anchor.rotation!=b.anchor.rotation||a.anchor.q!=b.anchor.q||a.previous_command!=b.previous_command||
    a.last_t!=b.last_t||a.previous_stamp!=b.previous_stamp||a.origin!=b.origin||a.plan_t!=b.plan_t||a.step_count!=b.step_count||
    a.plan!=b.plan||a.plan_h!=b.plan_h||a.joint_history.size()!=b.joint_history.size())return false;
 for(std::size_t i=0;i<a.joint_history.size();++i)if(a.joint_history[i].time!=b.joint_history[i].time||a.joint_history[i].velocity!=b.joint_history[i].velocity)return false;
 return true;
}
void check(bool v,const char* text){if(!v)throw std::runtime_error(text);}
template<class F>void rejects(F f){bool rejected=false;try{f();}catch(const x650::CoreError&){rejected=true;}check(rejected,"expected rejection");}
x650::Config config(){x650::Config c;c.nodes=3;c.geometry={.2,.2,-.7853981633974483,.7853981633974483,0,0,0,0,0,0};
 c.physical_lower={-2,-2};c.physical_upper={2,2};c.response={4.,.8,4.,.8,.25};c.base_position_gain={1,1,1};c.motion={.01,.1,.02,0,.015,.03,.005,.01};return c;}
x650::StepInput input(){x650::StepInput in;in.t=.02;in.dt=.02;in.state.stamp=in.state.control_time=in.state.joint_stamp=.02;
 in.target.position={.4*std::cos(.7853981633974483),0,0};in.context.reference_time=0;in.context.commands={{0,{0,0}}};
 for(int i=-20;i<=0;++i){x650::Packet p;p.send_time=i*.005;p.kp={6,6};p.kd={.25,.25};in.context.packets.push_back(p);}return in;}
}
int main(){try{
 auto c=config();x650::Core full(c,ts20_prediction_batch),compact(c,ts20_prediction_batch,x650::DiagnosticMode::Compact);
 rejects([&]{full.step(input());});full.reset({});compact.reset({});
 const std::vector<x650::Command5> history{{0,{0,0,0,0,0}}};full.prime(history,0);compact.prime(history,0);rejects([&]{full.prime(history,0);});
 auto in=input();auto a=full.step(in),b=compact.step(in);
 check(a.diagnostics.accepted&&a.diagnostics.nonlinear_passed&&b.diagnostics.accepted,"full/compact accepted");
 check(!a.diagnostics.recovery_triggered&&!b.diagnostics.recovery_triggered&&a.diagnostics.recovery_attempts.empty()&&b.diagnostics.recovery_attempts.empty(),"ordinary optimum must not invoke recovery");
 check(a.command==b.command&&a.position_reference==b.position_reference&&a.velocity_feedforward==b.velocity_feedforward,"compact command equality");
 check(!a.diagnostics.H.empty()&&b.diagnostics.H.empty()&&b.diagnostics.candidate_packets.empty(),"compact avoids log matrices/packets");
 check(a.diagnostics.linearized_states.size()==std::size_t(c.nodes*12)&&b.diagnostics.linearized_states.empty(),"Full captures original linearized states without compact overhead");
 check(a.diagnostics.plan==b.diagnostics.plan&&a.diagnostics.predicted_states==b.diagnostics.predicted_states&&a.diagnostics.candidate_envelope==b.diagnostics.candidate_envelope,"compact keeps checks/endpoint data");
 check(a.diagnostics.candidate_boundary_times.size()==a.diagnostics.candidate_zero_events.size()&&a.diagnostics.candidate_modes.size()==2*a.diagnostics.candidate_boundary_times.size()&&!a.diagnostics.candidate_boundary_times.empty(),"actual propagation boundary evidence alignment");
 check(full.memory().step_count==1&&full.memory().last_t==.02,"successful commit");
 auto old=full.memory();rejects([&]{full.step(in);});check(full.memory().plan==old.plan&&full.memory().step_count==old.step_count,"duplicate rejection rollback");
 in.t=.04;in.state.stamp=in.state.control_time=in.state.joint_stamp=.04;in.context.packets.back().send_time=.040000001;
 rejects([&]{full.step(in);});check(full.memory().previous_command==old.previous_command&&full.memory().plan==old.plan,"future packet rollback");
 in=input();in.t=.04;in.state.stamp=in.state.control_time=in.state.joint_stamp=.04;
 in.context.commands={{0,{0,0}},{.04+5e-11,{0,0}},{.05,{0,0}}};
 rejects([&]{compact.step(in);});check(compact.memory().step_count==1,"validate complete history past epsilon-close future record");
 auto warm=old;warm.has_last_t=false;warm.has_stamp=false;warm.step_count=0;full.restore(warm);check(full.memory().plan==warm.plan&&!full.memory().has_last_t,"offline warm restore without last_t");
 x650::Core timed(c,delayed_prediction);timed.reset({});timed.prime(history,0);timed.step(input());auto before_timeout=timed.memory();
 auto timed_input=input();timed_input.t=.04;timed_input.state.stamp=timed_input.state.control_time=timed_input.state.joint_stamp=.04;
 bool deadline_rejected=false;delay_next_prediction=true;
 try{timed.step(timed_input);}catch(const x650::CoreError& error){deadline_rejected=std::string(error.what()).find("deadline")!=std::string::npos&&error.diagnostics.compute_ms>=50.&&!error.diagnostics.accepted;}
 check(deadline_rejected&&!delay_next_prediction,"real 50 ms deadline rejects delayed native prediction");
 check(same_memory(before_timeout,timed.memory()),"timeout preserves complete plan, command, clocks, anchor and actual history transaction");
 auto after_timeout=timed.step(timed_input);check(after_timeout.diagnostics.accepted&&timed.memory().step_count==before_timeout.step_count+1,"rejected input did not advance monotonic clock");
 auto invalid=c;invalid.feasibility_tolerance=1e-6;rejects([&]{x650::Core x(invalid,ts20_prediction_batch);});
 invalid=c;invalid.motion[0]=std::numeric_limits<double>::quiet_NaN();rejects([&]{x650::Core x(invalid,ts20_prediction_batch);});
 invalid=c;invalid.joint_position_gain=std::numeric_limits<double>::infinity();rejects([&]{x650::Core x(invalid,ts20_prediction_batch);});
 rejects([&]{x650::Core x(c,nullptr);});check(x650_mpc_core_abi_version()==2,"core ABI");
 std::cout<<"PASS: full raw-input core, compact equivalence, transactional rejection including real 50 ms deadline, warm restore and strict configuration\n";return 0;
 }catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
