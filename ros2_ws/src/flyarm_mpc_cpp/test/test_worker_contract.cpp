#include "flyarm_mpc_cpp/worker_contract.hpp"
#include <functional>
#include <iostream>
#include <limits>

namespace fm=flyarm_mpc_cpp;namespace wire=flyarm_control_gateway;
void require(bool condition,const char* text){if(!condition)throw std::runtime_error(text);}
wire::Config config(){wire::Config c;c.session_id=23;c.epoch=1;c.configuration_sha256.fill(7);c.position_gains={.95,.95,1.};c.replay_only=false;return c;}
fm::Handoff start(){auto c=config();fm::Handoff s;s.session=c.session_id;s.epoch=c.epoch;s.hash=c.configuration_sha256;s.issued_ns=100;s.origin=10.;s.anchor.position={0,0,2};s.target={0,0,2};s.count=2;s.times[0]=9.98;s.times[1]=10.;return s;}
wire::Snapshot snapshot(double t=.02,double dt=.02,std::uint64_t seq=1){const auto c=config();wire::Snapshot s;
  s.session_id=c.session_id;s.epoch=c.epoch;s.configuration_sha256=c.configuration_sha256;s.replay_only=false;s.sequence=seq;
  s.issued_monotonic_ns=1000000000ULL+seq*20000000ULL;s.control_time_s=10.+t;s.task_time_s=t;s.dt_s=dt;
  s.state_time_s=s.joint_time_s=s.control_time_s-.004;s.reference_time_s=t-.004;s.position_enu=s.target_position_enu={0,0,2};s.rotation_row_major={1,0,0,0,1,0,0,0,1};s.position_gains=c.position_gains;
  s.command_count=s.packet_count=1;s.command_time_s[0]=-.02;s.packet_send_time_s[0]=-.005;s.bus_origin_s=-.025;
  s.reference_r={.01,-.01};s.command_velocity[0]=.02;s.command_velocity[1]=-.01;s.packet_q_ref[0]=.012;s.packet_q_ref[1]=-.011;s.packet_dq_ref[0]=.02;s.packet_dq_ref[1]=-.01;
  s.packet_kp[0]=s.packet_kp[1]=6.;s.packet_kd[0]=s.packet_kd[1]=.25;return s;}
wire::Proposal proposal(const wire::Snapshot& s){wire::Proposal p;p.session_id=s.session_id;p.epoch=s.epoch;p.configuration_sha256=s.configuration_sha256;p.replay_only=false;p.sequence=p.snapshot_sequence=s.sequence;
  p.control_time_s=s.control_time_s;p.task_time_s=s.task_time_s;p.compute_start_monotonic_ns=s.issued_monotonic_ns+1000000;p.compute_end_monotonic_ns=s.issued_monotonic_ns+10000000;
  p.position_reference_enu=s.position_enu;p.base_velocity_target=p.velocity_feedforward_enu={.1,0,0};p.accepted_optimum=p.nonlinear_check_passed=true;p.primal_violation=1e-10;p.kkt_residual=1e-9;p.subproblems=1;return p;}
template<class F>void rejects(F action){bool rejected=false;try{action();}catch(const std::exception&){rejected=true;}require(rejected,"Invalid input was accepted");}
int main(){try{
  {fm::Contract c(config());auto history=c.begin(start(),200);require(history.size()==2&&history[1].time==10.,"Actual five-axis prestream lost");
    double t=0.;std::uint64_t seq=0;for(double dt:{.02,.016,.020,.024}){t+=dt;auto s=snapshot(t,dt,++seq);auto input=c.decode(s,s.issued_monotonic_ns+1000000,15,.04);
      require(input.dt==dt&&input.state.position==s.position_enu&&input.context.reference==s.reference_r,"Actual source/dt changed");
      require(input.context.packets.size()==1&&input.context.packets[0].q_ref[0]==.012&&input.context.packets[0].kp[1]==6.,"Actual consumed MIT packet changed");
      require(input.preview.size()==15&&input.preview[14].position==s.target_position_enu,"Explicit hold horizon differs");
      const auto p=proposal(s);c.accept_output(p,p.compute_end_monotonic_ns);}
    require(c.accepted()==4,"Accepted calls differ");rejects([&]{c.begin(start(),200);});require(c.faulted(),"Repeated start did not latch");}
  for(const auto& mutation:std::vector<std::function<void(wire::Snapshot&)>>{
      [](auto& s){s.session_id++;},[](auto& s){s.target_position_enu[0]=.001;},[](auto& s){s.position_gains[0]+=.001;},
      [](auto& s){s.reference_time_s+=.001;},[](auto& s){s.packet_send_time_s[0]=s.task_time_s+1e-8;},
      [](auto& s){s.packet_kp[2]=6.;},[](auto& s){s.state_time_s=s.control_time_s-.251;},
      [](auto& s){s.joint_time_s=s.control_time_s-.026;},[](auto& s){s.sequence=2;},
      [](auto& s){s.position_enu[0]=std::numeric_limits<double>::quiet_NaN();}}){
    fm::Contract c(config());c.begin(start(),200);auto s=snapshot();mutation(s);rejects([&]{c.decode(s,s.issued_monotonic_ns+1000000,15,.04);});require(c.faulted(),"Invalid snapshot did not latch");}
  {fm::Contract c(config());c.begin(start(),200);auto s=snapshot();rejects([&]{c.decode(s,s.issued_monotonic_ns+50000001,15,.04);});}
  {fm::Contract c(config());c.begin(start(),200);auto s=snapshot();c.decode(s,s.issued_monotonic_ns+1000000,15,.04);auto p=proposal(s);p.primal_violation=2.01e-7;rejects([&]{c.accept_output(p,p.compute_end_monotonic_ns);});}
  {fm::Contract c(config());c.begin(start(),200);auto s=snapshot();c.decode(s,s.issued_monotonic_ns+1000000,15,.04);auto p=proposal(s);p.velocity_feedforward_enu[0]+=.001;rejects([&]{c.accept_output(p,p.compute_end_monotonic_ns);});}
  {fm::Contract c(config());auto s=start();s.commands[10]=.1;rejects([&]{c.begin(s,200);});}
  std::cout<<"C++ worker raw state/history/clock/PVA contracts passed; no controller solve or actuation\n";return 0;
 }catch(const std::exception& error){std::cerr<<error.what()<<'\n';return 1;}}
