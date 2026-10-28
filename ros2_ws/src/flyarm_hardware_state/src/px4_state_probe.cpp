// Observation only: no publishers, FC commands, arm requests, files, or /clock.
#include "flyarm_hardware_state/state_math.hpp"
#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <px4_msgs/msg/estimator_status_flags.hpp>
#include <px4_msgs/msg/sensor_combined.hpp>
#include <px4_msgs/msg/timesync_status.hpp>
#include <chrono>
#include <iomanip>
#include <iostream>
#include <map>
#include <vector>

using namespace flyarm_hw;
namespace pm = px4_msgs::msg;
using Clock = std::chrono::steady_clock;
double steady_seconds() { return std::chrono::duration<double>(Clock::now().time_since_epoch()).count(); }
void number(double v) { if (std::isfinite(v)) std::cout<<v; else std::cout<<"null"; }
template <std::size_t N> void array(const std::array<double,N> &v) {
  std::cout<<'['; for(std::size_t i=0;i<N;++i){ if(i)std::cout<<','; number(v[i]); } std::cout<<']';
}
template <std::size_t N> std::array<double,N> convert(const std::array<float,N> &v) {
  std::array<double,N> out{}; std::copy(v.begin(),v.end(),out.begin()); return out;
}
struct Channel { Stats all, settled; };

class Probe : public rclcpp::Node {
 public:
  Probe() : Node("flyarm_px4_hardware_state_probe") {
    duration_ = declare_parameter<double>("duration_s",60.0);
    settle_ = declare_parameter<double>("settle_s",5.0);
    if (get_parameter("use_sim_time").as_bool()) throw std::runtime_error("Physical probe requires wall time, not /clock");
    if (duration_ < 10 || duration_ > 600 || settle_ < 0 || settle_ >= duration_-2)
      throw std::runtime_error("Invalid duration or settle window");
    subscribe<pm::VehicleAttitude>(topic("vehicle_attitude",pm::VehicleAttitude::MESSAGE_VERSION),
      [this](const auto &m){ att_=m; });
    subscribe<pm::VehicleLocalPosition>(topic("vehicle_local_position",pm::VehicleLocalPosition::MESSAGE_VERSION),
      [this](const auto &m){ pos_=m; });
    subscribe<pm::VehicleOdometry>(topic("vehicle_odometry",pm::VehicleOdometry::MESSAGE_VERSION),
      [this](const auto &m){ odom_=m; });
    subscribe<pm::VehicleStatus>(topic("vehicle_status",pm::VehicleStatus::MESSAGE_VERSION),
      [this](const auto &m){ status_=m; armed_seen_ |= m.arming_state==pm::VehicleStatus::ARMING_STATE_ARMED; });
    subscribe<pm::EstimatorStatusFlags>("/fmu/out/estimator_status_flags",[this](const auto &m){ flags_=m; });
    subscribe<pm::SensorCombined>("/fmu/out/sensor_combined",[this](const auto &m){ imu_=m; });
    subscribe<pm::TimesyncStatus>("/fmu/out/timesync_status",[this](const auto &m){ sync_=m; });
    start_ = steady_seconds();
    timer_=create_wall_timer(std::chrono::seconds(1),[this](){
      const bool final=steady_seconds()-start_>=duration_;
      report(final);
      if(final)rclcpp::shutdown();
    });
  }
  bool received_core() const {
    for(const auto &key : {topic("vehicle_attitude",pm::VehicleAttitude::MESSAGE_VERSION),
        topic("vehicle_local_position",pm::VehicleLocalPosition::MESSAGE_VERSION),
        topic("vehicle_odometry",pm::VehicleOdometry::MESSAGE_VERSION),
        topic("vehicle_status",pm::VehicleStatus::MESSAGE_VERSION)})
      if(channels_.at(key).settled.count<2)return false;
    return true;
  }
 private:
  template <class T, class F> void subscribe(const std::string &name,F cb) {
    channels_[name]={};
    auto subscription=create_subscription<T>(name,rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile(),
      [this,name,cb](typename T::ConstSharedPtr m){
        const double now=steady_seconds();
        auto &channel=channels_.at(name); channel.all.add(now,m->timestamp);
        if(now-start_>=settle_)channel.settled.add(now,m->timestamp);
        cb(*m);
      });
    subscriptions_.push_back(subscription);
  }
  bool fresh(const std::string &name,double now,double age=0.25)const {
    return channels_.at(name).all.fresh(now,age);
  }
  static void stats(const Stats &s,double now) {
    std::cout<<"{\"count\":"<<s.count<<",\"hz\":"<<s.hz()<<",\"max_receipt_gap_ms\":"<<s.max_gap_ms
      <<",\"last_receipt_age_ms\":";
    number(s.count?1000*(now-s.last):std::numeric_limits<double>::quiet_NaN());
    std::cout<<",\"source_timestamp_us\":"<<s.stamp<<",\"source_regressions\":"<<s.regressions
      <<",\"repeated_source_timestamps\":"<<s.repeats<<",\"zero_source_timestamps\":"<<s.zero_stamps<<'}';
  }
  void report(bool final) {
    const double now=steady_seconds();
    const auto q=convert(att_.q), qe=enu_flu(q);
    const auto omega=convert(odom_.angular_velocity);
    const bool af=fresh(topic("vehicle_attitude",pm::VehicleAttitude::MESSAGE_VERSION),now);
    const bool pf=fresh(topic("vehicle_local_position",pm::VehicleLocalPosition::MESSAGE_VERSION),now);
    const bool of=fresh(topic("vehicle_odometry",pm::VehicleOdometry::MESSAGE_VERSION),now);
    const bool sf=fresh(topic("vehicle_status",pm::VehicleStatus::MESSAGE_VERSION),now,1.0);
    const Vec3 p{pos_.x,pos_.y,pos_.z},v{pos_.vx,pos_.vy,pos_.vz};
    const bool position_usable=pf && finite(p) && finite(v) && pos_.xy_valid && pos_.z_valid && pos_.v_xy_valid && pos_.v_z_valid;
    std::cout<<std::boolalpha<<std::setprecision(12)<<"{\"schema\":\"flyarm.ros2.hardware_state.v1\",\"final\":"<<final
      <<",\"elapsed_s\":"<<now-start_<<",\"settle_s\":"<<settle_<<",\"core_topics_received\":"<<received_core()
      <<",\"core_receipts_fresh\":"<<(af&&pf&&of&&sf)<<",\"armed_seen\":"<<armed_seen_
      <<",\"readonly\":true,\"channels\":{";
    bool first=true;
    for(const auto &entry:channels_) {
      if(!first)std::cout<<',';
      first=false;
      std::cout<<'"'<<entry.first<<"\":{\"all\":";stats(entry.second.all,now);
      std::cout<<",\"settled\":";stats(entry.second.settled,now);std::cout<<'}';
    }
    std::cout<<"},\"body\":{\"attitude_usable\":"<<(af&&finite(qe))<<",\"position_velocity_usable\":"<<position_usable
      <<",\"angular_velocity_usable\":"<<(of&&finite(omega))<<",\"q_ned_frd_wxyz\":"; array(q);
    std::cout<<",\"q_enu_flu_wxyz\":";array(qe);
    std::cout<<",\"quaternion_norm\":";number(norm(q));
    std::cout<<",\"position_enu_m\":";array(ned_to_enu(p));
    std::cout<<",\"velocity_enu_m_s\":";array(ned_to_enu(v));
    std::cout<<",\"angular_velocity_flu_rad_s\":";array(frd_to_flu(omega));
    std::cout<<",\"xy_valid\":"<<pos_.xy_valid<<",\"z_valid\":"<<pos_.z_valid
      <<",\"v_xy_valid\":"<<pos_.v_xy_valid<<",\"v_z_valid\":"<<pos_.v_z_valid
      <<",\"heading_good_for_control\":"<<pos_.heading_good_for_control
      <<",\"xy_reset_counter\":"<<unsigned(pos_.xy_reset_counter)<<",\"z_reset_counter\":"<<unsigned(pos_.z_reset_counter)
      <<",\"quat_reset_counter\":"<<unsigned(att_.quat_reset_counter)<<",\"odometry_reset_counter\":"<<unsigned(odom_.reset_counter)
      <<"},\"status\":{\"fresh\":"<<sf<<",\"arming_state\":"<<unsigned(status_.arming_state)
      <<",\"nav_state\":"<<unsigned(status_.nav_state)<<",\"failsafe\":"<<status_.failsafe
      <<",\"hil_state\":"<<unsigned(status_.hil_state)<<"},\"estimator\":{\"fresh\":"<<fresh("/fmu/out/estimator_status_flags",now,1)
      <<",\"tilt_aligned\":"<<flags_.cs_tilt_align<<",\"yaw_aligned\":"<<flags_.cs_yaw_align
      <<",\"ev_pos_fusion\":"<<flags_.cs_ev_pos<<",\"ev_vel_fusion\":"<<flags_.cs_ev_vel
      <<"},\"timesync\":{\"fresh\":"<<fresh("/fmu/out/timesync_status",now,1)
      <<",\"protocol\":"<<unsigned(sync_.source_protocol)<<",\"estimated_offset_us\":"<<sync_.estimated_offset
      <<",\"round_trip_time_us\":"<<sync_.round_trip_time<<"}}\n"<<std::flush;
  }
  double start_=steady_seconds(),duration_=60,settle_=5;
  bool armed_seen_=false;
  pm::VehicleAttitude att_{};
  pm::VehicleLocalPosition pos_{};
  pm::VehicleOdometry odom_{};
  pm::VehicleStatus status_{};
  pm::EstimatorStatusFlags flags_{};
  pm::SensorCombined imu_{};
  pm::TimesyncStatus sync_{};
  std::map<std::string,Channel> channels_;
  std::vector<rclcpp::SubscriptionBase::SharedPtr> subscriptions_;
  rclcpp::TimerBase::SharedPtr timer_;
};
int main(int argc,char **argv) {
  rclcpp::init(argc,argv);
  try {
    auto node=std::make_shared<Probe>();
    rclcpp::spin(node);
    const bool received=node->received_core();
    if(rclcpp::ok())rclcpp::shutdown();
    return received?0:2;  // Receipt only; never claim flight or control readiness.
  } catch(const std::exception &e) {
    std::cerr<<"state_probe_error: "<<e.what()<<'\n';
    if(rclcpp::ok())rclcpp::shutdown();
    return 3;
  }
}
