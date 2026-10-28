#pragma once
// Bench/replay adapter. It retains the disarmed guard; not a flight dispatcher.
#include "mpc_core.hpp"
#include "flyarm_hardware_state/px4_message_adapter.hpp"
#include <sensor_msgs/msg/joint_state.hpp>

namespace x650_mpc {
class Px4ReplayAdapter {
 public:
  flyarm_hw::Px4BodyReceiver receiver;
  explicit Px4ReplayAdapter(std::array<std::string,2> names):names_(std::move(names)) {
    if(names_[0].empty()||names_[1].empty()||names_[0]==names_[1])
      throw std::invalid_argument("Two distinct explicit joint names required");
  }
  void receive_joints(const sensor_msgs::msg::JointState& m,double receipt) {
    try {
    if(m.name.size()!=2||m.position.size()!=2||m.velocity.size()!=2||
       m.header.stamp.sec<0||m.header.stamp.nanosec>=1'000'000'000)
      throw std::invalid_argument("Invalid joint feedback dimensions or stamp");
    for(std::size_t i=0;i<2;++i) {
      const auto it=std::find(m.name.begin(),m.name.end(),names_[i]);
      if(it==m.name.end()||std::count(m.name.begin(),m.name.end(),names_[i])!=1)
        throw std::invalid_argument("Missing/duplicate joint identity");
      const auto j=std::size_t(it-m.name.begin());q_[i]=m.position[j];dq_[i]=m.velocity[j];
    }
    if(!flyarm_hw::finite(q_)||!flyarm_hw::finite(dq_))throw std::invalid_argument("Invalid joint numbers");
    const auto ns=std::uint64_t(m.header.stamp.sec)*1'000'000'000+m.header.stamp.nanosec;
    joints_.update(ns,ns,receipt);joint_stamp_=double(ns)/1e9;
    } catch(...) {joint_fault_=true;throw;}
  }
  x650::State state(double control_time,double now)const {
    const auto b=flyarm_hw::adapt_body(receiver.body,now);
    if(!b.ready())throw std::invalid_argument(std::string("PX4 state: ")+flyarm_hw::reason_name(b.reason));
    if(joint_fault_||!joints_.fresh(now,.1))throw std::invalid_argument("Joint feedback invalid, stale or regressed");
    if(!flyarm_hw::finite(receiver.acceleration_ned))throw std::invalid_argument("Invalid world acceleration");
    const double stamp=double(receiver.body.position.sample)/1e6;
    if(!std::isfinite(control_time)||control_time<=0||control_time>=9e9)
      throw std::invalid_argument("Invalid control clock");
    // Compare at integer nanosecond resolution. Decimal seconds and integer
    // message timestamps otherwise label the SAME instant a few ULPs future.
    // This is no extra latency allowance: a sample 1 ns future still fails.
    const auto control_ns=std::uint64_t(std::llround(control_time*1e9));
    const auto age_ok=[&](std::uint64_t ns){return ns<=control_ns&&control_ns-ns<=100'000'000;};
    if(!age_ok(joints_.sample))throw std::invalid_argument("Joint measurement future/stale in common clock");
    for(const auto* c:{&receiver.body.position,&receiver.body.attitude,&receiver.body.odometry}) {
      if(c->sample>control_ns/1000||!age_ok(c->sample*1000))
        throw std::invalid_argument("Body measurement future/stale in common clock");
    }
    x650::State s;
    s.position=b.p_enu;s.velocity=b.v_enu;s.omega_body=b.omega_flu;
    // PX4 local-position ax/ay/az are world-frame velocity derivatives, not
    // raw accelerometer specific force. Do not subtract/add gravity here.
    s.acceleration=flyarm_hw::ned_to_enu(receiver.acceleration_ned);
    const auto& q=b.q_enu_flu;const double w=q[0],x=q[1],y=q[2],z=q[3];
    s.rotation={1-2*(y*y+z*z),2*(x*y-w*z),2*(x*z+w*y),
                2*(x*y+w*z),1-2*(x*x+z*z),2*(y*z-w*x),
                2*(x*z-w*y),2*(y*z+w*x),1-2*(x*x+y*y)};
    s.q=q_;s.dq=dq_;s.stamp=stamp;s.joint_stamp=joint_stamp_;s.control_time=double(control_ns)/1e9;
    return s;
  }
 private:
  std::array<std::string,2> names_;
  x650::V2 q_{},dq_{};double joint_stamp_{};
  bool joint_fault_=false;
  flyarm_hw::InputClock joints_;
};
inline px4_msgs::msg::TrajectorySetpoint encode_mpc_output(const x650::StepResult& r,
                                                         std::uint64_t stamp,double yaw_enu) {
  // velocity_feedforward is already the final PX4 velocity field. Never add
  // command[0:3] again. No simulation ref_alt offset on the physical path.
  return flyarm_hw::trajectory_message(flyarm_hw::encode_enu_pva(stamp,
    r.position_reference,r.velocity_feedforward,r.acceleration_feedforward,yaw_enu));
}
} // namespace x650_mpc
