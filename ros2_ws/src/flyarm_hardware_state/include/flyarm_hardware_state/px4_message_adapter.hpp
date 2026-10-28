#pragma once
// Shared by the physical diagnostic subscriber and the offline CDR replay.
// No publishers, executors, device access or persistent configuration here.
#include "flyarm_hardware_state/link_contract.hpp"
#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <map>
#include <vector>

namespace flyarm_hw {
template<std::size_t N> std::array<double,N> doubles(const std::array<float,N>& x) {
  std::array<double,N> y{}; std::copy(x.begin(),x.end(),y.begin()); return y;
}
class Px4BodyReceiver {
 public:
  BodyInput body;
  Vec3 acceleration_ned{};
  void receive(const px4_msgs::msg::VehicleAttitude& m,double now) {
    reset("attitude",{m.quat_reset_counter});
    body.attitude.update(m.timestamp,m.timestamp_sample,now);body.q_ned_frd=doubles(m.q);
  }
  void receive(const px4_msgs::msg::VehicleLocalPosition& m,double now) {
    reset("position",{m.xy_reset_counter,m.z_reset_counter,m.vxy_reset_counter,m.vz_reset_counter,m.heading_reset_counter});
    body.position.update(m.timestamp,m.timestamp_sample,now);
    body.p_ned={m.x,m.y,m.z};body.v_ned={m.vx,m.vy,m.vz};acceleration_ned={m.ax,m.ay,m.az};
    body.xy_valid=m.xy_valid;body.z_valid=m.z_valid;body.vxy_valid=m.v_xy_valid;
    body.vz_valid=m.v_z_valid;body.heading_valid=m.heading_good_for_control;
  }
  void receive(const px4_msgs::msg::VehicleOdometry& m,double now) {
    reset("odometry",{m.reset_counter});
    body.odometry.update(m.timestamp,m.timestamp_sample,now);body.omega_frd=doubles(m.angular_velocity);
  }
  void receive(const px4_msgs::msg::VehicleStatus& m,double now) {
    body.status.update(m.timestamp,m.timestamp,now);
    body.arming_state=m.arming_state;body.nav_state=m.nav_state;body.hil_state=m.hil_state;
    body.armed_seen|=m.arming_state==px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED;
    body.offboard_seen|=m.nav_state==px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD;
    body.hil_seen|=m.hil_state!=px4_msgs::msg::VehicleStatus::HIL_STATE_OFF;
    body.failsafe=m.failsafe;body.system=m.system_id;body.component=m.component_id;
  }
 private:
  std::map<std::string,std::vector<unsigned>> resets_;
  void reset(const std::string& key,std::vector<unsigned> values) {
    const auto found=resets_.find(key);
    if(found!=resets_.end()&&found->second!=values)body.reset_seen=true;
    resets_[key]=std::move(values);
  }
};

inline px4_msgs::msg::TrajectorySetpoint trajectory_message(const PvaCommand& p) {
  px4_msgs::msg::TrajectorySetpoint m{};
  m.timestamp=p.timestamp;
  for(std::size_t i=0;i<3;++i) {
    m.position[i]=static_cast<float>(p.position[i]);
    m.velocity[i]=static_cast<float>(p.velocity[i]);
    m.acceleration[i]=static_cast<float>(p.acceleration[i]);
    m.jerk[i]=static_cast<float>(p.jerk[i]);
    if(!std::isfinite(m.position[i])||!std::isfinite(m.velocity[i])||!std::isfinite(m.acceleration[i]))
      throw std::invalid_argument("PVA cannot be represented as float32");
  }
  m.yaw=static_cast<float>(p.yaw);m.yawspeed=static_cast<float>(p.yawspeed);
  return m;
}
} // namespace flyarm_hw
