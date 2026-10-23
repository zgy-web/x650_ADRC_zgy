#include "flyarm_control_gateway/shadow_gate.hpp"
#include <flyarm_control_interfaces/msg/control_snapshot.hpp>
#include <flyarm_control_interfaces/msg/control_proposal.hpp>
#include <flyarm_control_interfaces/msg/gate_event.hpp>
#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <iostream>
#include <memory>
#include <string>
#include <vector>
#include <time.h>

namespace fcg = flyarm_control_gateway;
namespace msg = flyarm_control_interfaces::msg;

namespace {

std::uint64_t monotonic_ns() {
  timespec value{};
  if (clock_gettime(CLOCK_MONOTONIC, &value) != 0)
    throw std::runtime_error("CLOCK_MONOTONIC unavailable");
  return static_cast<std::uint64_t>(value.tv_sec)*1'000'000'000ULL + static_cast<std::uint64_t>(value.tv_nsec);
}

fcg::Hash parse_hash(const std::string & text) {
  if (text.size() != 64) throw std::invalid_argument("configuration_sha256 must be 64 lowercase hex characters");
  fcg::Hash bytes{};
  auto digit = [](char value) -> unsigned {
    if (value >= '0' && value <= '9') return static_cast<unsigned>(value-'0');
    if (value >= 'a' && value <= 'f') return static_cast<unsigned>(value-'a'+10);
    throw std::invalid_argument("configuration_sha256 must be lowercase hex");
  };
  for (std::size_t i=0; i<bytes.size(); ++i) bytes[i] = static_cast<std::uint8_t>(16*digit(text[2*i])+digit(text[2*i+1]));
  return bytes;
}

template<typename Ros, typename Plain>
void copy_identity(const Ros & from, Plain & to) {
  to.schema_version=from.schema_version; to.session_id=from.session_id; to.epoch=from.epoch;
  to.sequence=from.sequence; to.replay_only=from.replay_only; to.configuration_sha256=from.configuration_sha256;
}

fcg::Snapshot from_ros(const msg::ControlSnapshot & from) {
  fcg::Snapshot to;
  copy_identity(from, to);
#define COPY(field) to.field=from.field
  COPY(issued_monotonic_ns); COPY(control_time_s); COPY(task_time_s); COPY(dt_s);
  COPY(state_time_s); COPY(joint_time_s); COPY(position_enu); COPY(velocity_enu);
  COPY(omega_body_flu); COPY(acceleration_enu); COPY(rotation_row_major); COPY(q); COPY(dq);
  COPY(target_position_enu); COPY(target_velocity_enu); COPY(target_acceleration_enu); COPY(position_gains);
  COPY(reference_time_s); COPY(bus_origin_s); COPY(reference_r); COPY(command_count); COPY(packet_count);
  COPY(command_time_s); COPY(packet_send_time_s); COPY(command_velocity); COPY(packet_q_ref);
  COPY(packet_dq_ref); COPY(packet_tau_ff); COPY(packet_kp); COPY(packet_kd);
#undef COPY
  return to;
}

fcg::Proposal from_ros(const msg::ControlProposal & from) {
  fcg::Proposal to;
  copy_identity(from, to);
#define COPY(field) to.field=from.field
  COPY(snapshot_sequence); COPY(compute_start_monotonic_ns); COPY(compute_end_monotonic_ns);
  COPY(status); COPY(control_time_s); COPY(task_time_s); COPY(position_reference_enu);
  COPY(velocity_feedforward_enu); COPY(acceleration_feedforward_enu); COPY(base_velocity_target);
  COPY(joint_velocity); COPY(accepted_optimum); COPY(nonlinear_check_passed);
  COPY(primal_violation); COPY(kkt_residual); COPY(subproblems);
#undef COPY
  return to;
}

msg::GateEvent to_ros(const fcg::Event & from) {
  msg::GateEvent to;
  to.schema_version=from.schema_version; to.session_id=from.session_id; to.epoch=from.epoch;
  to.sequence=from.sequence; to.snapshot_sequence=from.snapshot_sequence; to.proposal_sequence=from.proposal_sequence;
  to.replay_only=from.replay_only; to.accepted=from.accepted; to.reason=static_cast<std::uint16_t>(from.reason);
  to.received_monotonic_ns=from.received_monotonic_ns; to.roundtrip_ns=from.roundtrip_ns;
  return to;
}

class ShadowGateway final : public rclcpp::Node {
 public:
  ShadowGateway() : Node("shadow_gateway") {
    const auto session = declare_parameter<std::int64_t>("expected_session_id", 0);
    const auto epoch = declare_parameter<std::int64_t>("expected_epoch", 0);
    const auto prefix = declare_parameter<std::string>("prefix", "/flyarm/offline/v1");
    const auto digest = declare_parameter<std::string>("configuration_sha256", "");
    const auto gains = declare_parameter<std::vector<double>>("position_gains", std::vector<double>{});
    if (session <= 0 || epoch <= 0 || gains.size()!=3 || prefix!="/flyarm/offline/v1")
      throw std::invalid_argument("Positive configured session/epoch, three PID gains and fixed offline prefix required");
    fcg::Config config;
    config.session_id=static_cast<std::uint64_t>(session); config.epoch=static_cast<std::uint64_t>(epoch);
    config.configuration_sha256=parse_hash(digest);
    std::copy(gains.begin(), gains.end(), config.position_gains.begin());
    gate_=std::make_unique<fcg::ShadowGate>(config);
    const auto one = rclcpp::QoS(rclcpp::KeepLast(1)).reliable();
    admitted_=create_publisher<msg::ControlSnapshot>(prefix+"/admitted/snapshot", one);
    shadow_=create_publisher<msg::ControlProposal>(prefix+"/output/shadow_command", one);
    events_=create_publisher<msg::GateEvent>(prefix+"/output/gate_event", rclcpp::QoS(rclcpp::KeepLast(64)).reliable());
    snapshots_=create_subscription<msg::ControlSnapshot>(prefix+"/input/snapshot", one,
      [this](const msg::ControlSnapshot::SharedPtr snapshot) {
        const auto admission=gate_->admit(from_ros(*snapshot), monotonic_ns());
        if (admission.event) events_->publish(to_ros(*admission.event));
        if (admission.forward) admitted_->publish(*snapshot);
      });
    proposals_=create_subscription<msg::ControlProposal>(prefix+"/output/proposal", one,
      [this](const msg::ControlProposal::SharedPtr proposal) {
        const auto event=gate_->propose(from_ros(*proposal), monotonic_ns());
        if (!event) return;
        if (event->accepted) shadow_->publish(*proposal);
        events_->publish(to_ros(*event));
      });
    timer_=create_wall_timer(std::chrono::milliseconds(5), [this] {
      if (const auto event=gate_->poll(monotonic_ns())) events_->publish(to_ros(*event));
    });
    // One startup line only. No formatting, recorder waits or hot-path logging.
    RCLCPP_INFO(get_logger(), "READY offline shadow gateway; no actuator publishers");
  }
 private:
  std::unique_ptr<fcg::ShadowGate> gate_;
  rclcpp::Publisher<msg::ControlSnapshot>::SharedPtr admitted_;
  rclcpp::Publisher<msg::ControlProposal>::SharedPtr shadow_;
  rclcpp::Publisher<msg::GateEvent>::SharedPtr events_;
  rclcpp::Subscription<msg::ControlSnapshot>::SharedPtr snapshots_;
  rclcpp::Subscription<msg::ControlProposal>::SharedPtr proposals_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<ShadowGateway>());
    rclcpp::shutdown();
    return 0;
  } catch (const std::exception & error) {
    std::cerr << "shadow_gateway startup/runtime failure: " << error.what() << '\n';
    rclcpp::shutdown();
    return 1;
  }
}
