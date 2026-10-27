#include "mpc_core.hpp"
#include "synthetic_fixture.hpp"
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>

namespace {
void check(bool condition,const char* message){if(!condition)throw std::runtime_error(message);}
void reject(const x650::PlanValidation& value,const char* fragment){
 check(!value.passed&&value.reason.find(fragment)!=std::string::npos,"wrong validator rejection");
}
int predictor_calls=0;
int counted_predictor(const double* model,const double* gains,double h,double start,
 const double* initial,const double* times,const double* forcing,int count,const double* queries,int nq,int track,
 double* states,int* modes,int* zeros,double* envelope,double* extrema,int capacity,int* events,int* steps,char* error,int es){
 ++predictor_calls;
 return ts20_prediction_batch(model,gains,h,start,initial,times,forcing,count,queries,nq,track,
  states,modes,zeros,envelope,extrema,capacity,events,steps,error,es);
}
// Deliberately injected oracle outputs exercise the independent acceptance
// rules, not physical simulation validity. Real-model equivalence is below.
enum class Injected { StoppingOnly, BetweenNodeVelocity } injected;
int injected_predictor(const double*,const double*,double,double,const double*,const double*,const double*,int,
 const double*,int nq,int,double* states,int* modes,int* zeros,double* envelope,double*,int,int* events,int* steps,char*,int){
 const double q=injected==Injected::StoppingOnly?.19:0.,dq=injected==Injected::StoppingOnly?.42:0.;
 for(int i=0;i<nq;++i){states[4*i]=q;states[4*i+1]=0.;states[4*i+2]=dq;states[4*i+3]=0.;
  modes[2*i]=modes[2*i+1]=0;zeros[i]=0;}
 const double e[8]{q,0.,q,0.,dq,0.,injected==Injected::BetweenNodeVelocity?.43:dq,0.};
 for(int i=0;i<8;++i)envelope[i]=e[i];*events=0;*steps=1;return 0;
}
bool same_packet(const x650::Packet& a,const x650::Packet& b){
 return a.send_time==b.send_time&&a.q_ref==b.q_ref&&a.dq_ref==b.dq_ref&&a.tau_ff==b.tau_ff&&a.kp==b.kp&&a.kd==b.kd;
}
void joint_reference(x650::StepInput& in,double q){
 in.context.reference={q,0.};for(auto& p:in.context.packets)p.q_ref={q,0.};
}
}

int main(){try{
 const auto c=fixture::config();const auto in=fixture::input();
 x650::Core core(c,ts20_prediction_batch,x650::DiagnosticMode::Full);
 core.reset({});core.prime({{10.,{0.,0.,0.,0.,0.}}},10.);
 const auto memory=core.memory();const auto original_memory_copy=memory;const std::vector<double> zero(5*c.nodes,0.);
 predictor_calls=0;
 const auto valid=x650::evaluate_physical_plan(c,counted_predictor,memory,in,zero);
 check(valid.passed&&valid.reason=="conditional_event_plan_accepted","real zero plan rejected");
 check(predictor_calls==2&&valid.native_checked&&valid.hold_guard.checked&&valid.hold_guard.duration_s==.16,
       "one native rollout and one declared hold expected");
 check(valid.command_bounds_passed&&valid.reference_bounds_passed&&valid.native_bounds_passed&&
       valid.stopping_passed&&valid.pending_guard_passed&&valid.hold_passed,"missing common check");
 check(valid.node_states.size()==std::size_t(4*c.nodes)&&valid.node_references.size()==std::size_t(2*c.nodes)&&
       valid.actual_packet_count==int(in.context.packets.size())&&valid.source_control_time_s==in.state.control_time&&
       valid.committed_step_count==memory.step_count,"source/node evidence missing");
 check(fixture::same_memory(memory,core.memory()),"validator mutated committed memory");
 for(std::size_t i=0;i<in.context.packets.size();++i)check(same_packet(in.context.packets[i],valid.packets[i]),"actual packet prefix rewritten");

 // A genuine old Core candidate must reproduce all shared event-model evidence.
 const auto old=core.step(in);const auto& d=old.diagnostics;
 const auto replay=x650::evaluate_physical_plan(c,ts20_prediction_batch,memory,in,d.plan);
 check(replay.passed&&replay.query_times==d.query_times&&replay.query_states==d.candidate_states&&
       replay.boundary_times==d.candidate_boundary_times&&replay.envelope==d.candidate_envelope&&
       replay.extrema==d.candidate_extrema&&replay.substeps==d.candidate_substeps,
       "validator changed native packet grid or event propagation");
 check(replay.packets.size()==d.candidate_packets.size(),"candidate packet count changed");
 for(std::size_t i=0;i<replay.packets.size();++i)check(same_packet(replay.packets[i],d.candidate_packets[i]),"candidate encoding changed");
 check(replay.hold_guard.envelope==d.hold_guard.envelope&&replay.hold_guard.boundary_states==d.hold_guard.boundary_states,
       "held-command propagation changed");
 const int first_guard_row=d.qp_rows-d.interval_rows-5;
 check(first_guard_row>=0,"legacy first-command rows missing");
 for(int j=0;j<5;++j)check(replay.first_lower[j]==d.lower[first_guard_row+j]&&
   replay.first_upper[j]==d.upper[first_guard_row+j],"pure first-command guard differs from original QP rows");
 for(int i=0;i<c.nodes;++i)for(int j=0;j<4;++j)
  check(replay.node_states[4*i+j]==d.predicted_states[12*i+6+j],"actual nonlinear node changed");

 for(int axis=0;axis<5;++axis){
  auto plan=zero;plan[axis]=c.velocity_limit[axis]+.01;
  reject(x650::evaluate_physical_plan(c,ts20_prediction_batch,memory,in,plan),"command speed/slew");
  plan=zero;plan[axis]=c.acceleration_limit[axis]*in.dt+.001;
  reject(x650::evaluate_physical_plan(c,ts20_prediction_batch,memory,in,plan),"command speed/slew");
 }
 auto plan=zero;plan[0]=std::numeric_limits<double>::quiet_NaN();
 reject(x650::evaluate_physical_plan(c,ts20_prediction_batch,memory,in,plan),"finite plan shape");
 plan=zero;plan.pop_back();reject(x650::evaluate_physical_plan(c,ts20_prediction_batch,memory,in,plan),"finite plan shape");
 auto changed=in;changed.state.joint_stamp=changed.state.control_time-.26;
 reject(x650::evaluate_physical_plan(c,ts20_prediction_batch,memory,changed,zero),"source age/origin");
 changed=in;changed.state.control_time+=1.;changed.state.joint_stamp+=1.;
 reject(x650::evaluate_physical_plan(c,ts20_prediction_batch,memory,changed,zero),"source age/origin");
 changed=in;changed.context.commands.push_back({in.t+.01,{0.,0.}});
 reject(x650::evaluate_physical_plan(c,ts20_prediction_batch,memory,changed,zero),"actual command history time");
 changed=in;changed.context.packets.back().send_time=in.t+.01;
 reject(x650::evaluate_physical_plan(c,ts20_prediction_batch,memory,changed,zero),"actual packet contract");
 changed=in;changed.context.commands.push_back(changed.context.commands.front());
 reject(x650::evaluate_physical_plan(c,ts20_prediction_batch,memory,changed,zero),"actual command history time");
 auto unprimed=memory;unprimed.history_primed=false;
 reject(x650::evaluate_physical_plan(c,ts20_prediction_batch,unprimed,in,zero),"primed committed history");

 changed=in;joint_reference(changed,c.q_upper[0]-.0001);plan=zero;
 for(int i=0;i<c.nodes;++i)plan[5*i+3]=.02;
 reject(x650::evaluate_physical_plan(c,ts20_prediction_batch,memory,changed,plan),"integrated reference");
 changed=in;changed.state.dq[0]=.43;
 auto envelope=x650::evaluate_physical_plan(c,ts20_prediction_batch,memory,changed,zero);
 reject(envelope,"continuous joint envelope");check(envelope.dq_envelope_violation>.009,"actual .42 inner velocity bound not checked");

 // The missing legacy ordinary-path stopping check is tested independently of
 // the basic q/dq envelope: both components pass while q+r*dq fails.
 changed=in;changed.state.q[0]=.19;changed.state.dq[0]=.42;joint_reference(changed,.19);
 injected=Injected::StoppingOnly;
 const auto stopping=x650::evaluate_physical_plan(c,injected_predictor,memory,changed,zero);
 reject(stopping,"actual node stopping reserve");
 check(stopping.native_bounds_passed&&stopping.pending_guard_passed&&stopping.hold_passed&&
       !stopping.stopping_passed&&std::abs(stopping.stopping_violation-.0067)<1e-12,
       "actual stopping did not independently reject an envelope-feasible plan");
 injected=Injected::BetweenNodeVelocity;
 const auto intersample=x650::evaluate_physical_plan(c,injected_predictor,memory,in,zero);
 reject(intersample,"continuous joint envelope");
 check(intersample.node_states[2]==0.&&intersample.envelope[6]==.43&&!intersample.native_bounds_passed,
       "between-node envelope replaced with node-only checks");

 // The real actuator model can have safe nodes/envelope yet violate the
 // original conservative pending-command guard near an upper joint limit.
 changed=in;changed.state.q[0]=.44;changed.state.dq[0]=.10;joint_reference(changed,.44);
 const auto pending=x650::evaluate_physical_plan(c,ts20_prediction_batch,memory,changed,zero);
 reject(pending,"pending position outside soft envelope");
 check(pending.native_bounds_passed&&pending.stopping_passed&&!pending.pending_guard_passed,
       "original first-command pending guard missing");
 check(fixture::same_memory(memory,original_memory_copy)&&core.memory().step_count==memory.step_count+1,
       "validation unexpectedly changed transaction count");
 check(x650_mpc_core_abi_version()==4,"old Core ABI must remain unchanged");
 std::cout<<"PASS: read-only event plan validator; exact legacy candidate/hold propagation; all five command bounds; "
   "immutable history; continuous inner envelope; unconditional stopping and original pending guard\n";
 return 0;
 }catch(const std::exception& error){std::cerr<<error.what()<<'\n';return 1;}}
