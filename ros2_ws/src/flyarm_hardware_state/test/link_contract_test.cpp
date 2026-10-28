#include "flyarm_hardware_state/link_contract.hpp"
#include <iostream>
#include <stdexcept>
using namespace flyarm_hw;
void check(bool x,const char *s){if(!x)throw std::runtime_error(s);}
void near(double a,double b){check(std::abs(a-b)<1e-9,"numeric mismatch");}
BodyInput healthy(){
  BodyInput s;for(auto *c:{&s.attitude,&s.position,&s.odometry,&s.status})c->update(1'000'000,990'000,10);
  s.q_ned_frd={1,0,0,0};s.p_ned={2,3,-1};s.v_ned={.1,.2,.3};s.omega_frd={1,2,3};
  s.xy_valid=s.z_valid=s.vxy_valid=s.vz_valid=s.heading_valid=true;
  s.arming_state=1;s.system=1;s.component=1;return s;
}
int main(){
  auto s=healthy();auto b=adapt_body(s,10.01);check(b.ready(),"valid state rejected");
  check(b.p_enu==Vec3({3,2,1})&&b.omega_flu==Vec3({1,-2,-3}),"frame conversion");
  auto hold=shadow_hold(b,1'010'000);check(hold.position==s.p_ned,"hold origin must not gain a simulation offset");
  near(hold.yaw,0);check(std::isnan(hold.yawspeed)&&std::isnan(hold.jerk[0]),"unused fields must be NaN");
  auto p=encode_enu_pva(100,{1,2,3},{4,5,6},{7,8,9},0);
  check(p.position==Vec3({2,1,-3})&&p.velocity==Vec3({5,4,-6})&&p.acceleration==Vec3({8,7,-9}),"PVA axes");
  near(p.yaw,std::acos(-1.0)/2);
  s.xy_valid=false;check(adapt_body(s,10.01).reason==StateReason::position_unusable,"finite but invalid position");
  bool threw=false;try{shadow_hold(adapt_body(s,10.01),100);}catch(const std::invalid_argument&){threw=true;}
  check(threw,"invalid state created a hold");
  s=healthy();s.heading_valid=false;check(adapt_body(s,10.01).reason==StateReason::heading_unusable,"heading gate");
  s=healthy();s.armed_seen=true;check(bench_guard(s,10.01)==StateReason::not_disarmed,"armed latch");
  s=healthy();s.reset_seen=true;check(adapt_body(s,10.01).reason==StateReason::source_reset,"reset latch");
  s=healthy();s.odometry.sample-=60'000;check(adapt_body(s,10.01).reason==StateReason::sample_skew,"sample alignment");
  s=healthy();check(!adapt_body(s,10.11).ready(),"stale sensor state");
  s=healthy();s.q_ned_frd={0,0,0,0};check(!adapt_body(s,10.01).ready(),"invalid quaternion");
  s=healthy();s.attitude.update(1'000'001,990'000,10.2);check(!s.attitude.fresh(10.21,.1),"duplicate sample refreshed data");
  s=healthy();s.attitude.update(900'000,899'000,10.01);check(s.attitude.fault,"clock regression");
  InputClock c;c.update(0,0,1);check(!c.fresh(1,.1),"zero timestamp");
  c={};c.update(1,2,1);check(!c.fresh(1,.1),"sample after publication");
  AckTransaction tx;CommandRequest req{1'010'000,401,1,1,245,191};tx.begin(req,10.01);
  CommandAck ack{1'011'000,401,0,245,191,false};
  auto wrong=ack;wrong.target_component=192;check(!tx.observe(wrong,10.02),"foreign ack");
  wrong=ack;wrong.command=400;check(!tx.observe(wrong,10.02),"wrong command");
  wrong=ack;wrong.from_external=true;check(!tx.observe(wrong,10.02),"external ack");
  wrong=ack;wrong.timestamp=1'000'000;check(!tx.observe(wrong,10.02),"old ack");
  check(tx.observe(ack,10.025)&&tx.accepted(),"accepted ack");near(tx.rtt_ms(),15);
  check(!tx.observe(ack,10.026),"duplicate ack");
  AckTransaction timeout;timeout.begin(req,1);check(timeout.expired(4.01)&&!timeout.observe(ack,4.01),"timeout");
  AckTransaction denied;denied.begin(req,1);ack.result=2;check(denied.observe(ack,1.01)&&!denied.accepted(),"denial is not success");
  AckTransaction bad;req.command=400;threw=false;try{bad.begin(req,1);}catch(const std::invalid_argument&){threw=true;}
  check(threw,"arm command escaped whitelist");
  std::cout<<"PASS: body validity, clock/reset/skew guards, ENU/NED PVA, no simulation origin offset, diagnostic command whitelist and ACK correlation\n";
}
