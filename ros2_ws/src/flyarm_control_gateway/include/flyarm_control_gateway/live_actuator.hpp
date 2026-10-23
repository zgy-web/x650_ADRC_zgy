#pragma once
// Simulation adapter only: ROS -> existing Gazebo TS20 bridge and loopback
// MAVLink -> the already discovered SITL peer. No network hardware defaults.
#include "flyarm_control_gateway/runtime_wire.hpp"
#include <flyarm_control_interfaces/msg/actuator_state.hpp>
#include <flyarm_control_interfaces/msg/actuator_send.hpp>
#include <flyarm_control_interfaces/msg/execution_receipt.hpp>
#include <flyarm_control_interfaces/msg/live_start.hpp>
#include <std_msgs/msg/float64.hpp>
#include <common/mavlink.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <deque>

namespace flyarm_runtime_wire {
class LiveActuator {
 public:
  explicit LiveActuator(rclcpp::Node &node): node_(node) {
    const int port=node.declare_parameter<int>("sitl_udp_port",0);
    system_=node.declare_parameter<int>("sitl_system_id",0);
    component_=node.declare_parameter<int>("sitl_component_id",0);
    if(port<1024 || port>65535 || system_!=1 || component_!=1)
      throw std::invalid_argument("Explicit loopback SITL port and system/component 1/1 required");
    const int inherited=node.declare_parameter<int>("sitl_socket_fd",-1);
    sockaddr_in local{};socklen_t length=sizeof(local);
    if(inherited<0 || getsockname(inherited,reinterpret_cast<sockaddr*>(&local),&length)!=0 ||
       local.sin_family!=AF_INET || local.sin_addr.s_addr!=htonl(INADDR_LOOPBACK) || ntohs(local.sin_port)!=14550)
      throw std::runtime_error("Existing loopback SITL socket must be explicitly inherited");
    socket_=dup(inherited);
    if(socket_<0) throw std::runtime_error("SITL socket duplication failed");
    peer_.sin_family=AF_INET;peer_.sin_port=htons(port);peer_.sin_addr.s_addr=htonl(INADDR_LOOPBACK);
    for(int j=0;j<2;++j) pubs_[j]=node.create_publisher<std_msgs::msg::Float64>(
      "/arm/arm_joint_"+std::to_string(j+1)+"/cmd_vel",rclcpp::QoS(10).reliable());
    receipts_=node.create_publisher<msg::ExecutionReceipt>("/flyarm/isolated/v1/consumed",rclcpp::QoS(128).best_effort());
    sends_=node.create_publisher<msg::ActuatorSend>("/flyarm/isolated/v1/actuator_send",rclcpp::QoS(512).best_effort());
  }
  ~LiveActuator() { if(socket_>=0) ::close(socket_); }
  void begin(const msg::LiveStart &s) {
    if(started_) {failed_=true;return;}
    if(!std::isfinite(s.ref_alt)||!std::isfinite(s.yaw_ned)) throw std::invalid_argument("Invalid live datum");
    start_=s;started_=true;
  }
  void state(const msg::ActuatorState &s) {
    if(!started_ || s.session_id!=start_.session_id || s.epoch!=start_.epoch) return;
    auto now=monotonic_ns();
    if(!s.observed_monotonic_ns || s.observed_monotonic_ns>now || now-s.observed_monotonic_ns>25'000'000 ||
       !std::isfinite(s.sim_s) || (have_state_ && s.sim_s<state_.sim_s)) {failed_=true;return;}
    state_=s;have_state_=true;
    if(!sent_any_) { expected_=s.consumed_counter;have_counter_=true; }
    if(pending_.empty()) return;
    const auto &p=pending_.front();
    bool complete=true;
    for(int j=0;j<2;++j) {
      if(s.consumed_counter[j]>p.expected_counter[j]) {failed_=true;return;}
      if(s.consumed_counter[j]<p.expected_counter[j]) complete=false;
      else if(!std::isfinite(s.consumed_velocity[j]) || std::abs(s.consumed_velocity[j]-p.joint_velocity[j])>1e-12 ||
              s.consumed_sim_s[j]<p.sim_s-.025) {failed_=true;return;}
    }
    if(complete) {
      msg::ExecutionReceipt r;r.session_id=s.session_id;r.epoch=s.epoch;
      r.command_sequence=p.command_sequence;r.dispatch_sequence=p.dispatch_sequence;
      r.observed_monotonic_ns=now;r.joint_consumed=true;r.pva_send_returned=!p.stop_zero;
      r.observation_sim_s=s.sim_s;
      r.consumed_counter=s.consumed_counter;r.consumed_sim_s=s.consumed_sim_s;
      receipts_->publish(r);pending_.pop_front();
      if(pending_.empty() && stop_sent_) released_=true;
    }
  }
  bool failed() const {return failed_;}
  bool released() const {return released_;}
  bool endpoints_ready() const {
    return pubs_[0]->get_subscription_count()>=1 && pubs_[1]->get_subscription_count()>=1 &&
      receipts_->get_subscription_count()>=2 && sends_->get_subscription_count()>=1;
  }
  void execute(const msg::DispatchTick &tick) {
    if(!started_ || released_) return;
    if(tick.output_valid && !failed_ && !stop_sent_) {
      if(!have_state_ || monotonic_ns()-state_.observed_monotonic_ns>25'000'000 || !pending_.empty()) {
        failed_=true;stop(tick);return;
      }
      const auto &c=tick.command;
      msg::ActuatorSend out=identity(tick);
      out.position_ned={c.position_reference_enu[1],c.position_reference_enu[0],start_.ref_alt-c.position_reference_enu[2]};
      out.velocity_ned={c.velocity_feedforward_enu[1],c.velocity_feedforward_enu[0],-c.velocity_feedforward_enu[2]};
      out.acceleration_ned={0.,0.,0.};out.coordinate_frame=MAV_FRAME_LOCAL_NED;
      out.type_mask=POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE;out.yaw_ned=start_.yaw_ned;
      out.joint_velocity=c.joint_velocity;
      // Use the same LOCAL_NED PVA mask/zero acceleration and yaw as the
      // original encode_world_command path. Float32 conversion is on wire.
      mavlink_message_t m;
      mavlink_msg_set_position_target_local_ned_pack(255,191,&m,
        static_cast<std::uint32_t>(state_.sim_s*1000.),system_,component_,MAV_FRAME_LOCAL_NED,out.type_mask,
        out.position_ned[0],out.position_ned[1],out.position_ned[2],
        out.velocity_ned[0],out.velocity_ned[1],out.velocity_ned[2],0.,0.,0.,out.yaw_ned,0.);
      std::uint8_t bytes[MAVLINK_MAX_PACKET_LEN];const auto n=mavlink_msg_to_send_buffer(bytes,&m);
      if(sendto(socket_,bytes,n,0,reinterpret_cast<const sockaddr*>(&peer_),sizeof(peer_))!=n) {
        failed_=true;stop(tick);return;
      }
      send_joints(out);
    } else if(tick.status==3 || tick.status==4 || failed_) stop(tick);
  }
 private:
  msg::ActuatorSend identity(const msg::DispatchTick &t) {
    msg::ActuatorSend s;s.session_id=start_.session_id;s.epoch=start_.epoch;
    s.command_sequence=t.command_sequence;s.dispatch_sequence=t.sequence;s.repeated=t.repeated;
    s.sent_monotonic_ns=monotonic_ns();s.sim_s=state_.sim_s;return s;
  }
  void send_joints(msg::ActuatorSend &s) {
    sent_any_=true;
    for(int j=0;j<2;++j) {std_msgs::msg::Float64 v;v.data=s.joint_velocity[j];pubs_[j]->publish(v);s.expected_counter[j]=++expected_[j];}
    s.sent_monotonic_ns=monotonic_ns();pending_.push_back(s);sends_->publish(s);
  }
  void stop(const msg::DispatchTick &t) {
    if(stop_sent_) return;
    auto s=identity(t);s.stop_zero=true;s.joint_velocity={0.,0.};
    send_joints(s);stop_sent_=true;
  }
  rclcpp::Node &node_;int socket_{-1},system_{},component_{};sockaddr_in peer_{};
  bool started_{},have_state_{},have_counter_{},sent_any_{},failed_{},stop_sent_{},released_{};
  msg::LiveStart start_;msg::ActuatorState state_;std::array<std::uint64_t,2> expected_{};
  std::deque<msg::ActuatorSend> pending_;
  std::array<rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr,2> pubs_;
  rclcpp::Publisher<msg::ExecutionReceipt>::SharedPtr receipts_;
  rclcpp::Publisher<msg::ActuatorSend>::SharedPtr sends_;
};
}
