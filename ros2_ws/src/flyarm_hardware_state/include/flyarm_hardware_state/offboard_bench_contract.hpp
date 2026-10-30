#pragma once
// POWER-DISCONNECTED bench only. No arm/mode/actuator command is generated.
// A session has a one-way exit: renewed state or mode messages cannot resume it.
#include "state_math.hpp"
#include <stdexcept>

namespace flyarm_hw {
struct BenchState {
  Stats status, attitude, manual;
  Quat q{1,0,0,0};
  unsigned arm=0, nav=0, hil=0, system=0, component=0, reset=0;
  bool failsafe=false, rc_valid=false;
  double throttle=0, roll=0, pitch=0, yaw=0;
};
class OffboardBenchContract {
 public:
  enum class Phase { waiting, streaming, engaged, retired };
  Phase phase=Phase::waiting;
  Quat target{1,0,0,0};
  const char* reason="waiting";
  double start=0, entered=0;
  bool offboard_seen=false;

  static bool healthy(const BenchState& s,double now) {
    const auto advancing=[](const Stats& a){return a.count>=3&&!a.zero_stamps&&!a.regressions;};
    // Repeated messages do not advance freshness in the receiver.
    return advancing(s.status)&&advancing(s.attitude)&&advancing(s.manual)
      &&s.status.fresh(now,1.0)&&s.attitude.fresh(now,.2)&&s.manual.fresh(now,.3)
      &&s.hil==0&&s.system==1&&s.component==1&&!s.failsafe&&s.rc_valid
      &&finite(s.q)&&std::abs(norm(s.q)-1)<.02
      &&std::isfinite(s.throttle)&&s.throttle<=-.95&&s.throttle>=-1.01
      &&std::isfinite(s.roll)&&std::abs(s.roll)<=.05
      &&std::isfinite(s.pitch)&&std::abs(s.pitch)<=.05
      &&std::isfinite(s.yaw)&&std::abs(s.yaw)<=.05;
  }
  bool begin(const BenchState& s,double now) {
    if(phase!=Phase::waiting||!healthy(s,now)||s.arm!=1||s.nav!=1)return false;
    target=s.q;for(auto& x:target)x/=norm(s.q);
    reset_=s.reset;start=now;phase=Phase::streaming;reason="streaming";return true;
  }
  void retire(const char* why) {phase=Phase::retired;reason=why;}
  bool tick(const BenchState& s,double now,double dropout_after_s=0) {
    if(phase==Phase::waiting||phase==Phase::retired)return false;
    if(now<start||now-start>=60){retire("session_deadline");return false;}
    if(offboard_seen&&s.nav!=14){retire("px4_left_offboard");return false;}
    if(s.reset!=reset_||!healthy(s,now)){retire("state_guard");return false;}
    if(s.arm!=1&&s.arm!=2){retire("unexpected_arming_state");return false;}
    if(!offboard_seen&&s.nav!=1&&s.nav!=14){retire("unexpected_mode");return false;}
    if(s.nav==14&&!offboard_seen){offboard_seen=true;entered=now;phase=Phase::engaged;}
    if(dropout_after_s>0&&offboard_seen&&now-entered>=dropout_after_s){retire("planned_stream_loss");return false;}
    return true;
  }
 private:
  unsigned reset_=0;
};
} // namespace flyarm_hw
