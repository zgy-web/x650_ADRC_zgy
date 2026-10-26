#include "mpc_core.hpp"
#include <cmath>
#include <iostream>
#include <limits>
#include <string>
extern "C" int ts20_prediction_batch(const double*,const double*,double,double,const double*,const double*,const double*,int,const double*,int,int,double*,int*,int*,double*,double*,int,int*,int*,char*,int);
extern "C" unsigned x650_optimization_abi_version();
namespace {
void check(bool ok,const char* text){if(!ok)throw std::runtime_error(text);}
x650::Config config(){x650::Config c;c.geometry={.2,.2,-.7853981633974483,.7853981633974483,0,0,0,0,0,0};
 c.physical_lower={-2,-2};c.physical_upper={2,2};c.response={4.,.8,4.,.8,.25};c.base_position_gain={1,1,1};
 c.motion={.0007447642530597398,.004194907941904997,.03470285366117535,.0004632811241788992,.003346379092576424,.075,.04,.2};return c;}
x650::StepInput input(double dt=.02){x650::StepInput in;in.t=in.dt=dt;in.state.stamp=in.state.control_time=in.state.joint_stamp=dt;
 in.target.position={.4*std::cos(.7853981633974483),0,0};in.context.reference_time=0;in.context.commands={{0,{0,0}}};
 for(int i=-20;i<=0;++i){x650::Packet p;p.send_time=i*.005;p.kp={6,6};p.kd={.25,.25};in.context.packets.push_back(p);}return in;}
void start(x650::Core& core){core.reset({});core.prime({{0,{0,0,0,0,0}}},0);}
bool same_memory(const x650::Memory&a,const x650::Memory&b){
 if(a.initialized!=b.initialized||a.history_primed!=b.history_primed||a.has_last_t!=b.has_last_t||a.has_stamp!=b.has_stamp||a.has_origin!=b.has_origin||
    a.anchor.position!=b.anchor.position||a.anchor.rotation!=b.anchor.rotation||a.anchor.q!=b.anchor.q||a.previous_command!=b.previous_command||
    a.last_t!=b.last_t||a.previous_stamp!=b.previous_stamp||a.origin!=b.origin||a.plan_t!=b.plan_t||a.step_count!=b.step_count||
    a.plan!=b.plan||a.plan_h!=b.plan_h||a.joint_history.size()!=b.joint_history.size())return false;
 for(std::size_t i=0;i<a.joint_history.size();++i)if(a.joint_history[i].time!=b.joint_history[i].time||a.joint_history[i].velocity!=b.joint_history[i].velocity)return false;
 return true;
}
bool fail_hold=false;
int kernel(const double*m,const double*g,double step,double begin,const double*x,const double*k,const double*f,int np,const double*b,int nb,int track,double*s,int*mode,int*zero,double*env,double*events,int cap,int*ne,int*ns,char*err,int es){
 int code=ts20_prediction_batch(m,g,step,begin,x,k,f,np,b,nb,track,s,mode,zero,env,events,cap,ne,ns,err,es);
 if(!code&&fail_hold&&track&&nb&&std::abs(b[nb-1]-.20)<1e-10)env[6]=.43;
 return code;
}
void legacy(){auto a=config(),b=a;b.command_hold_s=.05;x650::Core ca(a,ts20_prediction_batch),cb(b,ts20_prediction_batch);start(ca);start(cb);
 auto in=input();auto x=ca.step(in),y=cb.step(in);const auto&dx=x.diagnostics;const auto&dy=y.diagnostics;
 check(x.command==y.command&&x.position_reference==y.position_reference&&x.velocity_feedforward==y.velocity_feedforward,"Legacy default output differs");
 check(dx.A==dy.A&&dx.b==dy.b&&dx.H==dy.H&&dx.g==dy.g&&dx.C==dy.C&&dx.lower==dy.lower&&dx.upper==dy.upper&&dx.solution==dy.solution&&dx.plan==dy.plan&&dx.predicted_states==dy.predicted_states&&dx.candidate_envelope==dy.candidate_envelope,"Legacy default full numerical values differ");
 check(!dx.hold_guard.required&&!dx.hold_guard.checked&&dx.hold_guard.packets.empty(),"Legacy default added a propagation");
 check(same_memory(ca.memory(),cb.memory()),"Legacy default memory differs");check(x650_mpc_core_abi_version()==4&&x650_optimization_abi_version()==2,"Explicit native protocol versions differ");
 std::cout<<"PASS legacy_default_equivalence: default/explicit .05 exact matrices, solution, outputs, state and no extra hold rollout; ABI3-vs4 recorded-input equivalence is a separate replay check\n";
}
void extended(){auto c=config();c.max_dt=.1;c.command_hold_s=.16;auto in=input(.04);x650::Core core(c,kernel);start(core);auto good=core.step(in);
 check(good.diagnostics.horizon.size()==15&&good.diagnostics.horizon[0]==.04&&good.diagnostics.horizon.back()==.04&&good.diagnostics.qp_columns==75,"True .04 horizon or fixed variable count changed");
 const auto&h=good.diagnostics.hold_guard;check(h.required&&h.checked&&h.passed&&h.duration_s==.16&&h.start_task_s==.04&&h.end_task_s==.20&&h.source_control_time_s==in.state.control_time&&h.source_joint_time_s==in.state.joint_stamp,"Extended hold guard fabricated source time or duration");
 check(h.actual_history_packet_count==int(in.context.packets.size())&&h.packets.size()>in.context.packets.size()&&h.boundary_states.size()==4*h.boundary_times.size()&&h.substeps>0,"Held trajectory omits actual history/native evidence");
 for(std::size_t i=0;i<in.context.packets.size();++i)check(h.packets[i].send_time==in.context.packets[i].send_time&&h.packets[i].q_ref==in.context.packets[i].q_ref&&h.packets[i].dq_ref==in.context.packets[i].dq_ref,"Hold guard rewrote original packets");
 std::vector<double> plan(75);for(int i=0;i<15;++i)plan[5*i+3]=.45;
 auto unsafe=x650::evaluate_first_command_hold(c,ts20_prediction_batch,in,plan,{0,0});
 check(unsafe.checked&&!unsafe.passed&&unsafe.reference_final[0]<c.q_upper[0]&&unsafe.envelope[6]>.42,"Natural extended native velocity envelope counterexample was not rejected");
 auto before=core.memory();auto later=in;later.t=.14;later.dt=.1;later.state.stamp=later.state.control_time=later.state.joint_stamp=.14;
 auto endpoint=core.step(later);check(endpoint.diagnostics.horizon[0]==.1,"Explicit max_dt .1 boundary failed");
 before=core.memory();later.t+=.100001;later.dt=.100001;later.state.stamp=later.state.control_time=later.state.joint_stamp=later.t;
 bool rejected=false;try{core.step(later);}catch(const x650::CoreError&){rejected=true;}check(rejected&&same_memory(before,core.memory()),"Above-max dt did not reject transactionally");
 x650::Core bad(c,kernel);start(bad);before=bad.memory();fail_hold=true;rejected=false;
 try{bad.step(in);}catch(const x650::CoreError&e){rejected=std::string(e.what()).find("first-command held envelope")!=std::string::npos&&e.diagnostics.hold_guard.checked&&!e.diagnostics.hold_guard.passed;}
 fail_hold=false;check(rejected&&same_memory(before,bad.memory()),"Unsafe held-kernel evidence committed a candidate");
 auto invalid=c;invalid.command_hold_s=.05;rejected=false;try{x650::Core denied(invalid,kernel);}catch(const x650::CoreError&){rejected=true;}check(rejected,"Hold smaller than max dt admitted");
 std::cout<<"PASS extended_dt_hold_guard: native .16 ZOH, real unsafe velocity counterexample, unchanged actual history/time, .04/.1 boundaries, 75 variables and fail-closed rollback\n";
}
}
int main(int argc,char**argv){try{check(argc==2,"mode required");if(std::string(argv[1])=="legacy")legacy();else if(std::string(argv[1])=="extended")extended();else check(false,"unknown mode");return 0;}catch(const std::exception&e){std::cerr<<e.what()<<'\n';return 1;}}
