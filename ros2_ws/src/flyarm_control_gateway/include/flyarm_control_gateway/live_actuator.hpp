#pragma once
// Simulation adapter only: ROS -> existing Gazebo TS20 bridge and loopback
// MAVLink -> the already discovered SITL peer. No network hardware defaults.
#include "flyarm_control_gateway/runtime_wire.hpp"
#include "flyarm_control_gateway/consumption_wait.hpp"
#include "flyarm_control_gateway/consumption_ledger.hpp"
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
#include <vector>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>

namespace flyarm_runtime_wire {
class LiveActuator {
 public:
  explicit LiveActuator(rclcpp::Node &node): node_(node) {
    policy_=node.declare_parameter<std::string>("consumption_policy","strict_per_tick");
    if(policy_!="strict_per_tick" && policy_!="bounded_wait_v1" && policy_!="latest_value_v1")
      throw std::invalid_argument("Unknown explicit consumption policy");
    const auto budget=node.declare_parameter<int>("consumption_progress_budget_ms",40);
    if(budget!=40)throw std::invalid_argument("This version requires a fixed 40 ms per-joint unconfirmed-progress budget");
    evidence_path_=node.get_parameter("ready_file").as_string();
    ledger_evidence_.reserve(10000);
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
  ~LiveActuator() { persist_ledger();if(socket_>=0) ::close(socket_); }
  void begin(const msg::LiveStart &s) {
    if(started_) {fail(fcg::PeriodicGate::ACTUATOR_PROTOCOL);return;}
    if(!std::isfinite(s.ref_alt)||!std::isfinite(s.yaw_ned)) throw std::invalid_argument("Invalid live datum");
    start_=s;started_=true;
  }
  void state(const msg::ActuatorState &s) {
    if(!started_ || s.session_id!=start_.session_id || s.epoch!=start_.epoch) return;
    auto now=monotonic_ns();
    if(streaming() && have_state_ && s.observed_monotonic_ns<state_.observed_monotonic_ns && s.sim_s<=state_.sim_s) {
      ++ignored_old_states_;return; // no state/history/freshness rollback or renewal
    }
    last_state_received_=now;
    if(!s.observed_monotonic_ns || s.observed_monotonic_ns>now || now-s.observed_monotonic_ns>25'000'000 ||
       !std::isfinite(s.sim_s) || (have_state_ && s.sim_s<state_.sim_s)) {fail(fcg::PeriodicGate::STALE_ACTUATOR_STATE);return;}
    if(streaming()) {
      for(int j=0;j<2;++j) if(!std::isfinite(s.consumed_velocity[j]) || !std::isfinite(s.consumed_sim_s[j]) ||
          s.consumed_sim_s[j]>s.sim_s+1e-9) {fail(fcg::PeriodicGate::ACTUATOR_PROTOCOL);return;}
      // Validate this incoming state above, not the age of the state it replaces.
      // An already-expired send budget still cannot be erased by a late receipt.
      if(!failed_ && !stop_sent_ && ledger_.progress_expired(now))
        fail(fcg::PeriodicGate::CONSUMPTION_PROGRESS_EXPIRED);
      const auto observation=ledger_observation(s,now);
      if(!sent_any_)ledger_.seed(observation);
      else if(!ledger_.observe(observation,[this](const fcg::ConsumptionLedger::Entry &e){retire(e);})) {
        fail(fcg::PeriodicGate::ACTUATOR_PROTOCOL);return;
      }
      state_=s;have_state_=true;
      if(!sent_any_) {expected_=s.consumed_counter;have_counter_=true;}
      return;
    }
    state_=s;have_state_=true;
    if(!sent_any_) { expected_=s.consumed_counter;have_counter_=true; }
    if(pending_.empty()) return;
    const auto &p=pending_.front();
    bool complete=true;
    for(int j=0;j<2;++j) {
      if(s.consumed_counter[j]>p.expected_counter[j]) {fail(fcg::PeriodicGate::ACTUATOR_PROTOCOL);return;}
      if(s.consumed_counter[j]<p.expected_counter[j]) complete=false;
      else if(!std::isfinite(s.consumed_velocity[j]) || std::abs(s.consumed_velocity[j]-p.joint_velocity[j])>1e-12 ||
              !std::isfinite(s.consumed_sim_s[j]) || s.consumed_sim_s[j]<p.sim_s-.025) {fail(fcg::PeriodicGate::ACTUATOR_PROTOCOL);return;}
    }
    if(complete) {
      msg::ExecutionReceipt r;r.session_id=s.session_id;r.epoch=s.epoch;
      r.command_sequence=p.command_sequence;r.dispatch_sequence=p.dispatch_sequence;
      r.observed_monotonic_ns=now;r.joint_consumed=true;r.pva_send_returned=!p.stop_zero;
      r.observation_sim_s=s.sim_s;
      r.consumed_counter=s.consumed_counter;r.consumed_sim_s=s.consumed_sim_s;
      receipts_->publish(r);last_ack_completed_=monotonic_ns();pending_.pop_front();
      if(pending_.empty() && stop_sent_) released_=true;
    }
  }
  bool failed() const {return failed_;}
  bool wait_policy() const {return policy_=="bounded_wait_v1";}
  bool streaming() const {return policy_=="latest_value_v1";}
  void check_progress(std::uint64_t now) {
    if(streaming() && started_ && sent_any_ && !failed_ && !stop_sent_) {
      const auto ready=readiness(now);
      if(ready.fault!=fcg::PeriodicGate::NONE)fail(ready.fault);
    }
  }
  fcg::PeriodicGate::Fault failure_reason() const {return failure_reason_;}
  fcg::ConsumptionReadiness readiness(std::uint64_t now) const {
    if(failed_) return {failure_reason_,false,0};
    if(!started_ || stop_sent_ || released_) return {fcg::PeriodicGate::ACTUATOR_PROTOCOL,false,0};
    const auto base=fcg::consumption_readiness(now,have_state_,state_.observed_monotonic_ns,
                                             last_normal_send_,!streaming() && !pending_.empty());
    if(base.fault!=fcg::PeriodicGate::NONE)return base;
    if(streaming() && ledger_.size()>=fcg::ConsumptionLedger::CAPACITY-1)
      return {fcg::PeriodicGate::CONSUMPTION_LEDGER_OVERFLOW,false,0}; // reserve one slot for STOP
    if(streaming() && ledger_.progress_expired(now))
      return {fcg::PeriodicGate::CONSUMPTION_PROGRESS_EXPIRED,false,0};
    return base;
  }
  bool released() const {return released_;}
  std::uint64_t last_normal_send() const {return last_normal_send_;}
  std::uint64_t state_observed() const {return state_.observed_monotonic_ns;}
  std::uint64_t state_received() const {return last_state_received_;}
  std::uint64_t ack_completed() const {return last_ack_completed_;}
  bool endpoints_ready() const {
    return pubs_[0]->get_subscription_count()>=1 && pubs_[1]->get_subscription_count()>=1 &&
      receipts_->get_subscription_count()>=2 && sends_->get_subscription_count()>=1;
  }
  void execute(msg::DispatchTick &tick) {
    if(!started_ || released_) return;
    if(tick.output_valid && !failed_ && !stop_sent_) {
      const auto ready=readiness(monotonic_ns());
      if(ready.fault!=fcg::PeriodicGate::NONE || ready.pending) {
        fail(ready.fault!=fcg::PeriodicGate::NONE ? ready.fault : fcg::PeriodicGate::CONSUMPTION_TIMEOUT);
        invalidate(tick);stop(tick);return;
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
        fail(fcg::PeriodicGate::ACTUATOR_SEND_FAILED);invalidate(tick);stop(tick);return;
      }
      send_joints(out);
    } else if(tick.status==3 || tick.status==4 || failed_) stop(tick);
  }
 private:
  static fcg::ConsumptionLedger::Observation ledger_observation(const msg::ActuatorState &s,std::uint64_t received) {
    return {s.consumed_counter,s.consumed_velocity,s.consumed_sim_s,s.sim_s,s.observed_monotonic_ns,received};
  }
  void retire(const fcg::ConsumptionLedger::Entry &e) {
    if(ledger_evidence_.size()==ledger_evidence_.capacity()) {
      ledger_evidence_overflow_=true;fail(fcg::PeriodicGate::CONSUMPTION_LEDGER_OVERFLOW);
    } else ledger_evidence_.push_back(e);
    if(!e.paired)return; // never fabricate a group receipt for skipped or mixed observations
    msg::ExecutionReceipt r;r.session_id=start_.session_id;r.epoch=start_.epoch;
    r.command_sequence=e.command;r.dispatch_sequence=e.dispatch;r.observed_monotonic_ns=monotonic_ns();
    r.observation_sim_s=std::max(e.observation_sim[0],e.observation_sim[1]);
    r.joint_consumed=true;r.pva_send_returned=!e.stop;r.consumed_counter=e.counter;r.consumed_sim_s=e.consumed_sim;
    receipts_->publish(r);last_ack_completed_=monotonic_ns();if(e.stop)released_=true;
  }
  void persist_ledger() noexcept {
    if(!streaming() || evidence_path_.empty())return;
    try {
      const auto dir=std::filesystem::path(evidence_path_).parent_path();
      if(have_epoch_baseline_) {
        rclcpp::Serialization<msg::ActuatorState> serializer;rclcpp::SerializedMessage data;
        serializer.serialize_message(&epoch_baseline_,&data);const auto &raw=data.get_rcl_serialized_message();
        std::ofstream f(dir/"actuator_epoch_state.cdr",std::ios::binary);
        f.write(reinterpret_cast<const char*>(raw.buffer),raw.buffer_length);f.close();
        if(!f)throw std::runtime_error("Could not persist initial actuator baseline CDR");
      }
      std::ofstream f(dir/"actuator_ledger.json");f<<std::setprecision(17);
      f<<"{\"schema\":\"latest_value_ledger_v1\",\"complete\":true,\"consumption_policy\":\"latest_value_v1\","
       <<"\"consumption_progress_budget_ms\":40,\"capacity\":16,\"overflow\":"<<(ledger_evidence_overflow_?"true":"false")
       <<",\"ignored_old_states\":"<<ignored_old_states_<<",\"have_epoch_baseline\":"<<(have_epoch_baseline_?"true":"false")
       <<",\"baseline_received_ns\":"<<baseline_received_ns_<<",\"initial_counter\":["<<epoch_baseline_.consumed_counter[0]<<','<<epoch_baseline_.consumed_counter[1]
       <<"],\"rows\":[";
      bool first=true;
      auto write=[&](const fcg::ConsumptionLedger::Entry &e) {
        if(!first)f<<',';first=false;
        f<<"{\"command_sequence\":"<<e.command<<",\"dispatch_sequence\":"<<e.dispatch<<",\"sent_ns\":"<<e.sent
         <<",\"sim_s\":"<<e.sim<<",\"stop_zero\":"<<(e.stop?"true":"false")<<",\"paired\":"<<(e.paired?"true":"false");
        auto array=[&](const char *name,const auto &v){f<<",\""<<name<<"\":["<<+v[0]<<','<<+v[1]<<']';};
        array("expected_counter",e.counter);array("velocity",e.velocity);array("joint_status",e.status);
        array("consumed_sim_s",e.consumed_sim);array("observation_sim_s",e.observation_sim);
        array("observed_ns",e.observed);array("received_ns",e.received);f<<'}';
      };
      for(const auto &e:ledger_evidence_)write(e);
      for(std::size_t i=0;i<ledger_.size();++i)write(ledger_.at(i));
      f<<"]}\n";f.close();if(!f)throw std::runtime_error("Could not persist streaming ledger");
    } catch(const std::exception &e) {std::cerr<<"Streaming ledger evidence: "<<e.what()<<'\n';}
  }
  void fail(fcg::PeriodicGate::Fault reason) {
    if(!failed_) failure_reason_=reason;
    failed_=true;
  }
  void invalidate(msg::DispatchTick &tick) const {
    tick.output_valid=false;tick.status=fcg::PeriodicGate::FAULT;tick.reason=failure_reason_;
  }
  msg::ActuatorSend identity(const msg::DispatchTick &t) {
    msg::ActuatorSend s;s.session_id=start_.session_id;s.epoch=start_.epoch;
    s.command_sequence=t.command_sequence;s.dispatch_sequence=t.sequence;s.repeated=t.repeated;
    s.sent_monotonic_ns=monotonic_ns();s.sim_s=state_.sim_s;return s;
  }
  void send_joints(msg::ActuatorSend &s) {
    if(streaming() && !sent_any_) {epoch_baseline_=state_;baseline_received_ns_=last_state_received_;have_epoch_baseline_=true;}
    sent_any_=true;
    for(int j=0;j<2;++j) {std_msgs::msg::Float64 v;v.data=s.joint_velocity[j];pubs_[j]->publish(v);s.expected_counter[j]=++expected_[j];}
    s.sent_monotonic_ns=monotonic_ns();
    if(!s.stop_zero) last_normal_send_=s.sent_monotonic_ns;
    if(streaming()) {
      fcg::ConsumptionLedger::Entry e;e.command=s.command_sequence;e.dispatch=s.dispatch_sequence;e.sent=s.sent_monotonic_ns;
      e.sim=s.sim_s;e.counter=s.expected_counter;e.velocity=s.joint_velocity;e.stop=s.stop_zero;
      if(!ledger_.append(e))fail(fcg::PeriodicGate::CONSUMPTION_LEDGER_OVERFLOW);
    } else pending_.push_back(s);
    sends_->publish(s);
  }
  void stop(const msg::DispatchTick &t) {
    if(stop_sent_) return;
    auto s=identity(t);s.stop_zero=true;s.joint_velocity={0.,0.};
    send_joints(s);stop_sent_=true;
  }
  rclcpp::Node &node_;int socket_{-1},system_{},component_{};sockaddr_in peer_{};
  bool started_{},have_state_{},have_counter_{},sent_any_{},failed_{},stop_sent_{},released_{};
  std::uint64_t last_normal_send_{},last_state_received_{},last_ack_completed_{};
  fcg::PeriodicGate::Fault failure_reason_{fcg::PeriodicGate::NONE};
  msg::LiveStart start_;msg::ActuatorState state_;std::array<std::uint64_t,2> expected_{};
  std::deque<msg::ActuatorSend> pending_;
  std::string policy_,evidence_path_;fcg::ConsumptionLedger ledger_;
  std::vector<fcg::ConsumptionLedger::Entry> ledger_evidence_;
  msg::ActuatorState epoch_baseline_;std::uint64_t baseline_received_ns_{},ignored_old_states_{};
  bool have_epoch_baseline_{},ledger_evidence_overflow_{};
  std::array<rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr,2> pubs_;
  rclcpp::Publisher<msg::ExecutionReceipt>::SharedPtr receipts_;
  rclcpp::Publisher<msg::ActuatorSend>::SharedPtr sends_;
};
}
