#include "flyarm_mavlink/contract.hpp"
#include <iostream>
using namespace flyarm_mavlink;
int checks=0;
void check(bool value,const char* what){++checks;if(!value)throw std::runtime_error(what);}
mavlink_message_t roundtrip(const mavlink_message_t& m) {
  mavlink_message_t rx{},rxbuf{};mavlink_status_t status{},out{};bool got=false;
  for(auto b:wire(m))if(mavlink_frame_char_buffer(&rxbuf,&status,b,&rx,&out)==MAVLINK_FRAMING_OK)got=true;
  check(got,"wire roundtrip CRC");return rx;
}
Receiver healthy(int64_t now) {
  Receiver r;r.px4_identity=true;r.heartbeat_ns=now;r.mode_ns=now;r.custom_main=2;
  r.attitude_clock.update(1000,now);r.rc_clock.update(1000,now);
  r.q={1,0,0,0};r.channels=16;r.throttle=r.sf=r.sa=1000;r.sb=1500;return r;
}
int main(){try{
  const auto a=roundtrip(zero_thrust_attitude({1,0,0,0},42));mavlink_set_attitude_target_t at{};
  mavlink_msg_set_attitude_target_decode(&a,&at);
  check(a.msgid==82&&at.type_mask==7&&at.thrust==0&&at.target_system==1&&at.target_component==1,"zero thrust whitelist");
  check(std::isnan(at.body_roll_rate)&&std::isnan(at.body_yaw_rate),"ignored rates stay NaN");
  auto p=roundtrip(mpc_pva({1,2,3},{.1,.2,.3},{.4,.5,.6},0,314));mavlink_set_position_target_local_ned_t sp{};
  mavlink_msg_set_position_target_local_ned_decode(&p,&sp);
  check(sp.x==2&&sp.y==1&&sp.z==-3&&sp.vx==float(.2)&&sp.vy==float(.1)&&sp.vz==float(-.3),"PVA axis conversion");
  check(sp.afx==float(.5)&&sp.afy==float(.4)&&sp.afz==float(-.6),"acceleration conversion");
  check(sp.type_mask==2048&&sp.coordinate_frame==1&&sp.time_boot_ms==314&&std::isnan(sp.yaw_rate),"PVA flags");
  check(sp.yaw==float(pi/2),"yaw conversion");
  bool threw=false;try{mpc_pva({1e300,0,0},{0,0,0},{0,0,0},0,1);}catch(...){threw=true;}check(threw,"overflow");
  threw=false;try{zero_thrust_attitude({0,0,0,0},1);}catch(...){threw=true;}check(threw,"zero quaternion");
  SampleClock c;c.update(1,100);c.update(1,200);check(c.receipt==100,"duplicate clock");
  c.update(0,300);check(c.bad,"timestamp regression");
  constexpr int64_t t=10000000000LL;auto r=healthy(t);
  check(r.problem(t).empty(),"healthy receiver");
  check(r.problem(t+100000001)=="attitude_unavailable","stale attitude");
  r=healthy(t);r.throttle=1500;check(r.problem(t)=="rc_mapping_or_throttle","throttle guard");
  r=healthy(t);r.sb=2000;check(r.problem(t)=="operator_kill_request","kill request");
  r=healthy(t);BenchGate g;g.test=Case::ManualExit;check(g.tick(r,t),"initial stream");
  r.custom_main=6;r.sa=2000;check(g.tick(r,t+1),"enter offboard");
  r.sa=1000;check(!g.tick(r,t+2)&&g.stopped,"manual exit even before mode message");
  r.sa=2000;check(!g.tick(r,t+3),"no automatic reentry");
  g={};g.test=Case::ManualExit;r=healthy(t);check(g.tick(r,t),"asynchronous case starts");
  r.custom_main=6;check(g.tick(r,t+1)&&g.seen_offboard,"mode may arrive before RC switch packet");
  r.sa=2000;check(g.tick(r,t+2),"later RC confirms entry");
  r.sa=1000;check(!g.tick(r,t+3)&&g.reason=="manual_or_px4_exit","subsequent RC change exits");
  g={};g.test=Case::ManualExit;r=healthy(t);check(g.tick(r,t),"mode-only exit case starts");
  r.custom_main=6;check(g.tick(r,t+1),"mode-only entry");
  r.custom_main=2;check(!g.tick(r,t+2),"actual PX4 exit does not require RC transition");
  g={};g.test=Case::StreamStop;r=healthy(t);check(g.tick(r,t),"stop case starts");
  r.custom_main=6;r.sa=2000;check(g.tick(r,t+1),"stop case enters");
  r.heartbeat_ns=r.mode_ns=t+3000000001LL;r.attitude_clock.update(4000,t+3000000001LL);r.rc_clock.update(4000,t+3000000001LL);
  check(!g.tick(r,t+3000000001LL)&&g.reason=="planned_stream_stop","planned stop");
  g={};g.test=Case::ManualExit;r=healthy(t);r.armed=true;check(!g.tick(r,t),"start disarmed required");
  g={};g.test=Case::Observe;r=healthy(t);check(!g.tick(r,t),"observe never sends");
  r=healthy(t);mavlink_message_t hb{};mavlink_heartbeat_t h{};h.type=MAV_TYPE_QUADROTOR;h.autopilot=MAV_AUTOPILOT_PX4;
  mavlink_msg_heartbeat_encode(42,1,&hb,&h);r.receive(roundtrip(hb),t+100);check(r.heartbeat_ns==t&&r.foreign==1,"foreign identity");
  std::cout<<"{\"contract_checks\":"<<checks<<",\"passed\":true,\"hardware_accessed\":false}\n";return 0;
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
