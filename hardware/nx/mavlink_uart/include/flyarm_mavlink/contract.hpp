#pragma once
// Bench transport contract; no serial device, threads, or actuator access here.
#include <development/mavlink.h>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

namespace flyarm_mavlink {
constexpr uint8_t system_id=1, component_id=1, companion_system=245, companion_component=191;
constexpr double pi=3.14159265358979323846;
template<class A> bool finite(const A& a) { for(auto x:a) if(!std::isfinite(x)) return false; return true; }
inline std::vector<uint8_t> wire(const mavlink_message_t& m) {
  std::vector<uint8_t> b(MAVLINK_MAX_PACKET_LEN);
  b.resize(mavlink_msg_to_send_buffer(b.data(), &m)); return b;
}
inline mavlink_message_t zero_thrust_attitude(const std::array<float,4>& q, uint32_t fc_ms) {
  const double n=double(q[0])*q[0]+double(q[1])*q[1]+double(q[2])*q[2]+double(q[3])*q[3];
  if(!finite(q)||std::abs(n-1.)>.02) throw std::invalid_argument("invalid attitude");
  mavlink_set_attitude_target_t v{}; v.time_boot_ms=fc_ms;
  v.target_system=system_id; v.target_component=component_id;
  v.type_mask=ATTITUDE_TARGET_TYPEMASK_BODY_ROLL_RATE_IGNORE |
              ATTITUDE_TARGET_TYPEMASK_BODY_PITCH_RATE_IGNORE |
              ATTITUDE_TARGET_TYPEMASK_BODY_YAW_RATE_IGNORE;
  for(int i=0;i<4;++i) v.q[i]=q[i];
  v.body_roll_rate=v.body_pitch_rate=v.body_yaw_rate=std::numeric_limits<float>::quiet_NaN();
  v.thrust=0.f; // Scalar zero, never a hover thrust and never THRUST_BODY_SET.
  mavlink_message_t m{}; mavlink_msg_set_attitude_target_encode(companion_system,companion_component,&m,&v);return m;
}
inline mavlink_message_t mpc_pva(const std::array<double,3>& p, const std::array<double,3>& v,
                               const std::array<double,3>& a,double yaw_enu,uint32_t fc_ms) {
  if(!finite(p)||!finite(v)||!finite(a)||!std::isfinite(yaw_enu)) throw std::invalid_argument("nonfinite MPC output");
  mavlink_set_position_target_local_ned_t s{};s.time_boot_ms=fc_ms;
  s.target_system=system_id;s.target_component=component_id;s.coordinate_frame=MAV_FRAME_LOCAL_NED;
  s.type_mask=POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE; // PVA+yaw, FORCE flag clear.
  s.x=float(p[1]);s.y=float(p[0]);s.z=float(-p[2]);
  s.vx=float(v[1]);s.vy=float(v[0]);s.vz=float(-v[2]);
  s.afx=float(a[1]);s.afy=float(a[0]);s.afz=float(-a[2]);
  s.yaw=float(std::atan2(std::sin(pi/2-yaw_enu),std::cos(pi/2-yaw_enu)));
  s.yaw_rate=std::numeric_limits<float>::quiet_NaN();
  if(!finite(std::array<float,10>{s.x,s.y,s.z,s.vx,s.vy,s.vz,s.afx,s.afy,s.afz,s.yaw}))
    throw std::invalid_argument("float32 overflow");
  mavlink_message_t m{};mavlink_msg_set_position_target_local_ned_encode(companion_system,companion_component,&m,&s);return m;
}
struct SampleClock {
  uint32_t stamp=0; int64_t receipt=0; bool seen=false,bad=false;
  bool update(uint32_t s,int64_t t) {
    if(t<=0||s==0||(seen&&(s<stamp||t<receipt))) {bad=true;return false;}
    if(seen&&s==stamp)return false; // A repeated timestamp never refreshes age.
    stamp=s;receipt=t;seen=true;return true;
  }
  bool fresh(int64_t now,int64_t budget) const {return seen&&!bad&&now>=receipt&&now-receipt<=budget;}
};
struct Receiver {
  SampleClock attitude_clock,rc_clock;
  std::array<float,4> q{};
  uint16_t throttle=0,sf=0,sa=0,sb=0;uint8_t channels=0;
  int64_t heartbeat_ns=0,mode_ns=0; bool armed=false,px4_identity=false,fault=false;
  unsigned custom_main=0; uint64_t accepted=0,foreign=0;
  void receive(const mavlink_message_t& m,int64_t now) {
    if(m.sysid!=system_id||m.compid!=component_id){++foreign;return;}
    ++accepted;
    if(m.msgid==MAVLINK_MSG_ID_HEARTBEAT) {
      mavlink_heartbeat_t h{};mavlink_msg_heartbeat_decode(&m,&h);
      if(h.autopilot!=MAV_AUTOPILOT_PX4||h.type!=MAV_TYPE_QUADROTOR){fault=true;return;}
      heartbeat_ns=now;px4_identity=true;armed=(h.base_mode&MAV_MODE_FLAG_SAFETY_ARMED)!=0;
      // CURRENT_MODE is authoritative when available, HEARTBEAT only bootstraps.
      if(!mode_ns){custom_main=(h.custom_mode>>16)&255;}
    } else if(m.msgid==MAVLINK_MSG_ID_ATTITUDE_QUATERNION) {
      mavlink_attitude_quaternion_t a{};mavlink_msg_attitude_quaternion_decode(&m,&a);
      const std::array<float,4> next{a.q1,a.q2,a.q3,a.q4};
      const double n=double(a.q1)*a.q1+double(a.q2)*a.q2+double(a.q3)*a.q3+double(a.q4)*a.q4;
      if(!finite(next)||std::abs(n-1.)>.02||!finite(std::array<float,3>{a.rollspeed,a.pitchspeed,a.yawspeed})) {fault=true;return;}
      if(attitude_clock.update(a.time_boot_ms,now))q=next;
    } else if(m.msgid==MAVLINK_MSG_ID_RC_CHANNELS) {
      mavlink_rc_channels_t r{};mavlink_msg_rc_channels_decode(&m,&r);
      if(rc_clock.update(r.time_boot_ms,now)) {
        channels=r.chancount;throttle=r.chan3_raw;sf=r.chan5_raw;sa=r.chan6_raw;sb=r.chan7_raw;
      }
    } else if(m.msgid==MAVLINK_MSG_ID_CURRENT_MODE) {
      mavlink_current_mode_t c{};mavlink_msg_current_mode_decode(&m,&c);
      custom_main=(c.custom_mode>>16)&255;mode_ns=now;
    }
    fault=fault||attitude_clock.bad||rc_clock.bad;
  }
  std::string problem(int64_t now) const {
    if(fault)return "invalid_or_regressed_state";
    if(!px4_identity||!heartbeat_ns||now<heartbeat_ns||now-heartbeat_ns>1500000000LL)return "heartbeat_unavailable";
    if(!attitude_clock.fresh(now,100000000))return "attitude_unavailable";
    if(!rc_clock.fresh(now,150000000))return "rc_unavailable";
    // vehicle_status may publish at 2 Hz between mode changes. Requested 10 Hz
    // CURRENT_MODE is a scheduling ceiling, not a synthetic measurement clock.
    if(!mode_ns||now<mode_ns||now-mode_ns>1200000000LL)return "mode_unavailable";
    if(channels<7||sf<800||sf>2200||sa<800||sa>2200||sb<800||sb>2200||throttle<800||throttle>1150)
      return "rc_mapping_or_throttle";
    if(sb>1750)return "operator_kill_request";
    return {};
  }
};
enum class Case {Observe,Readback,ManualExit,StreamStop};
struct BenchGate {
  Case test=Case::Observe;bool started=false,stopped=false,seen_offboard=false,seen_sa_offboard=false;
  int64_t began=0,offboard_at=0;std::string reason;
  std::array<float,4> anchor{};
  void stop(const std::string& why) {stopped=true;if(reason.empty())reason=why;}
  bool tick(const Receiver& r,int64_t now) {
    if(stopped||test==Case::Observe)return false;
    const auto why=r.problem(now);
    if(!started) {
      // Preconditions are observed, never set by this program. No automatic arm
      // or mode command exists. Operator selects SA down only after READY.
      if(!why.empty()||r.armed||r.sf>1200||r.sa>1200||r.custom_main!=2)return false;
      anchor=r.q;began=now;started=true;
    }
    if(!why.empty()){stop(why);return false;}
    if(now<began||now-began>=60000000000LL){stop("session_limit");return false;}
    // RC_CHANNELS and CURRENT_MODE arrive independently. An older SA-up sample
    // must not masquerade as a new manual exit when the mode packet arrives first.
    if(r.sa>=1750)seen_sa_offboard=true;
    if(r.custom_main==6&&!seen_offboard){seen_offboard=true;offboard_at=now;}
    if(seen_offboard) {
      if(r.custom_main!=6||(seen_sa_offboard&&r.sa<1750)){stop("manual_or_px4_exit");return false;}
      if(test==Case::StreamStop&&now-offboard_at>=3000000000LL){stop("planned_stream_stop");return false;}
      if(test==Case::Readback&&now-offboard_at>=10000000000LL){stop("readback_window_complete");return false;}
    }
    return true;
  }
};
} // namespace flyarm_mavlink
