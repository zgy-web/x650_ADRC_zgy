#pragma once
#include "flyarm_hardware_state/state_math.hpp"
#include <stdexcept>

namespace flyarm_hw {
// Bench diagnostics, not the final flight controller's latency budget.
struct InputClock {
  std::uint64_t stamp{}, sample{}, count{};
  double receipt{}, progress{};
  bool fault{};
  void update(std::uint64_t ts, std::uint64_t sample_ts, double now) {
    if (!std::isfinite(now) || !ts || !sample_ts || sample_ts > ts ||
        (count && (ts < stamp || sample_ts < sample || now < receipt))) fault = true;
    if (!count || (ts > stamp && sample_ts > sample)) progress = now;
    stamp = ts; sample = sample_ts; receipt = now; ++count;
  }
  bool fresh(double now, double budget) const {
    return count && !fault && std::isfinite(now) && now >= receipt &&
           now-receipt <= budget && now-progress <= budget;
  }
};

struct BodyInput {
  InputClock attitude, position, odometry, status;
  Quat q_ned_frd{};
  Vec3 p_ned{}, v_ned{}, omega_frd{};
  bool xy_valid{}, z_valid{}, vxy_valid{}, vz_valid{}, heading_valid{};
  bool failsafe{}, reset_seen{}, armed_seen{}, offboard_seen{}, hil_seen{};
  unsigned arming_state{}, nav_state{}, hil_state{}, system{}, component{};
};

enum class StateReason {
  ready, status_unusable, not_disarmed, offboard_active, hil_active, failsafe,
  source_reset, attitude_unusable, position_unusable, angular_velocity_unusable,
  heading_unusable, sample_skew
};
inline const char *reason_name(StateReason r) {
  switch(r) {
    case StateReason::ready:return "ready";
    case StateReason::status_unusable:return "status_unusable";
    case StateReason::not_disarmed:return "not_disarmed";
    case StateReason::offboard_active:return "offboard_active";
    case StateReason::hil_active:return "hil_active";
    case StateReason::failsafe:return "failsafe";
    case StateReason::source_reset:return "source_reset";
    case StateReason::attitude_unusable:return "attitude_unusable";
    case StateReason::position_unusable:return "position_unusable";
    case StateReason::angular_velocity_unusable:return "angular_velocity_unusable";
    case StateReason::heading_unusable:return "heading_unusable";
    case StateReason::sample_skew:return "sample_skew";
  }
  return "unknown";
}
inline StateReason bench_guard(const BodyInput &s, double now) {
  if (!s.status.fresh(now,1.0) || !s.system || !s.component) return StateReason::status_unusable;
  if (s.arming_state!=1 || s.armed_seen) return StateReason::not_disarmed;
  if (s.nav_state==14 || s.offboard_seen) return StateReason::offboard_active;
  if (s.hil_state || s.hil_seen) return StateReason::hil_active;
  if (s.failsafe) return StateReason::failsafe;
  return StateReason::ready;
}
struct BodySnapshot {
  StateReason reason=StateReason::status_unusable;
  Quat q_enu_flu{};
  Vec3 p_enu{}, v_enu{}, omega_flu{};
  bool ready()const{return reason==StateReason::ready;}
};
inline BodySnapshot adapt_body(const BodyInput &s,double now) {
  BodySnapshot b;
  b.q_enu_flu=enu_flu(s.q_ned_frd); b.p_enu=ned_to_enu(s.p_ned);
  b.v_enu=ned_to_enu(s.v_ned); b.omega_flu=frd_to_flu(s.omega_frd);
  b.reason=bench_guard(s,now);
  if(b.reason!=StateReason::ready)return b;
  if(s.reset_seen)b.reason=StateReason::source_reset;
  else if(!s.attitude.fresh(now,.1)||!finite(b.q_enu_flu))b.reason=StateReason::attitude_unusable;
  else if(!s.position.fresh(now,.1)||!finite(b.p_enu)||!finite(b.v_enu)||
          !s.xy_valid||!s.z_valid||!s.vxy_valid||!s.vz_valid)b.reason=StateReason::position_unusable;
  else if(!s.odometry.fresh(now,.1)||!finite(b.omega_flu))b.reason=StateReason::angular_velocity_unusable;
  else if(!s.heading_valid)b.reason=StateReason::heading_unusable;
  else {
    const auto lo=std::min({s.attitude.sample,s.position.sample,s.odometry.sample});
    const auto hi=std::max({s.attitude.sample,s.position.sample,s.odometry.sample});
    // No extrapolation or interpolation is claimed by this bench adapter.
    if(hi-lo>50'000)b.reason=StateReason::sample_skew;
  }
  return b;
}

struct PvaCommand {
  std::uint64_t timestamp{};
  Vec3 position{}, velocity{}, acceleration{}, jerk{};
  double yaw{}, yawspeed{};
};
inline double wrap_pi(double x){return std::atan2(std::sin(x),std::cos(x));}
inline PvaCommand encode_enu_pva(std::uint64_t timestamp,const Vec3 &p,const Vec3 &v,
                                 const Vec3 &a,double yaw_enu) {
  if(!timestamp||!finite(p)||!finite(v)||!finite(a)||!std::isfinite(yaw_enu))
    throw std::invalid_argument("Non-finite PVA command or zero timestamp");
  const double nan=std::numeric_limits<double>::quiet_NaN();
  // Both frames have the SAME PX4 local origin. No simulation ref_alt offset.
  return {timestamp,ned_to_enu(p),ned_to_enu(v),ned_to_enu(a),{nan,nan,nan},
          wrap_pi(std::acos(-1.0)/2-yaw_enu),nan};
}
inline PvaCommand shadow_hold(const BodySnapshot &b,std::uint64_t timestamp) {
  if(!b.ready())throw std::invalid_argument("Body is not ready for a hold candidate");
  const auto &q=b.q_enu_flu;
  const double yaw=std::atan2(2*(q[0]*q[3]+q[1]*q[2]),1-2*(q[2]*q[2]+q[3]*q[3]));
  return encode_enu_pva(timestamp,b.p_enu,{0,0,0},{0,0,0},yaw);
}

// The only permitted live command in this executable: refresh pre-arm checks.
// PX4 Commander acknowledges 401 without arming, changing mode or parameters.
constexpr unsigned PREARM_CHECK_COMMAND=401, PROBE_SOURCE_SYSTEM=245;
struct CommandRequest {
  std::uint64_t timestamp{};
  unsigned command=PREARM_CHECK_COMMAND, target_system{}, target_component{};
  unsigned source_system=PROBE_SOURCE_SYSTEM, source_component{};
};
struct CommandAck {
  std::uint64_t timestamp{};
  unsigned command{},result{},target_system{},target_component{};
  bool from_external{};
};
class AckTransaction {
 public:
  void begin(const CommandRequest &request,double now) {
    if(sent_||request.command!=PREARM_CHECK_COMMAND||!request.timestamp||
       !request.target_system||!request.target_component||request.source_system!=PROBE_SOURCE_SYSTEM||
       request.source_component<191||request.source_component>220||!std::isfinite(now))
      throw std::invalid_argument("Invalid or repeated diagnostic transaction");
    request_=request; start_=now; sent_=true;
  }
  bool observe(const CommandAck &a,double now) {
    if(!sent_||received_||!std::isfinite(now)||now<start_||now-start_>3.0||
       a.command!=PREARM_CHECK_COMMAND||a.target_system!=request_.source_system||
       a.target_component!=request_.source_component||a.from_external||
       a.timestamp<request_.timestamp)return false;
    // A rejection is still a reply, but is NOT a successful request.
    received_=true;result_=a.result;rtt_ms_=1000*(now-start_);return true;
  }
  bool sent()const{return sent_;}
  bool received()const{return received_;}
  bool accepted()const{return received_&&result_==0;}
  bool expired(double now)const{return sent_&&!received_&&now-start_>3.0;}
  unsigned result()const{return result_;}
  double rtt_ms()const{return rtt_ms_;}
 private:
  CommandRequest request_{};double start_{},rtt_ms_{};
  bool sent_{},received_{};unsigned result_=255;
};
} // namespace flyarm_hw
