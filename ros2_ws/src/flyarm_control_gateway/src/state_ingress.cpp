#include "flyarm_control_gateway/runtime_wire.hpp"
#include <flyarm_control_interfaces/msg/accepted_command.hpp>
#include <flyarm_control_interfaces/msg/ingress_status.hpp>
#include <flyarm_control_interfaces/msg/runtime_stop.hpp>
#include <flyarm_control_interfaces/msg/dispatch_tick.hpp>

namespace wire=flyarm_runtime_wire;
class StateIngress final : public rclcpp::Node {
 public:
  StateIngress(): Node("isolated_state_ingress") {
    fcg::Config config;
    config.session_id=declare_parameter<std::int64_t>("expected_session_id",0);
    config.epoch=declare_parameter<std::int64_t>("expected_epoch",0);
    config.configuration_sha256=wire::parse_hash(declare_parameter<std::string>("configuration_sha256",""));
    auto gains=declare_parameter<std::vector<double>>("position_gains",std::vector<double>{});
    if (gains.size()!=3) throw std::invalid_argument("Three frozen PID gains required");
    std::copy(gains.begin(),gains.end(),config.position_gains.begin());
    last_control_=declare_parameter<double>("task_origin_s",10.);
    if (!std::isfinite(last_control_) || last_control_<0) throw std::invalid_argument("Invalid origin");
    config_=config; gate_=std::make_unique<fcg::ShadowGate>(config);
    const std::string p="/flyarm/isolated/v1";
    const auto latest=rclcpp::QoS(1).best_effort();
    admitted_=create_publisher<msg::ControlSnapshot>(p+"/admitted/snapshot",latest);
    accepted_=create_publisher<msg::AcceptedCommand>(p+"/accepted",latest);
    events_=create_publisher<msg::GateEvent>(p+"/event",rclcpp::QoS(256).best_effort());
    status_=create_publisher<msg::IngressStatus>(p+"/source_status",latest);
    states_=create_subscription<msg::ControlSnapshot>(p+"/source",latest,[this](msg::ControlSnapshot::SharedPtr s) {
      if(stopped_) return; // keep receiving state after a control fault for recovery/diagnosis
      auto now=wire::monotonic_ns();
      // Source identity, chronology and wall ages are never rewritten to make
      // an old sample look new. Superseded unadmitted STATE samples are allowed.
      if(s->session_id!=config_.session_id || s->epoch!=config_.epoch ||
         s->configuration_sha256!=config_.configuration_sha256 || !s->replay_only ||
         s->schema_version!=1 || s->sequence<=source_sequence_ ||
         !std::isfinite(s->control_time_s) || s->control_time_s<=source_time_ ||
         !s->issued_monotonic_ns || s->issued_monotonic_ns>now ||
         now-s->issued_monotonic_ns>25'000'000) {
        invalid_source_=true; publish_source(now); return;
      }
      source_sequence_=s->sequence; source_time_=s->control_time_s;
      latest_=*s; have_latest_=true; publish_source(now);
      if(!next_admission_) next_admission_=now+20'000'000;
      admit_latest(now);
    });
    proposals_=create_subscription<msg::ControlProposal>(p+"/output/proposal",latest,[this](msg::ControlProposal::SharedPtr p) {
      if(stopped_ || output_fault_) return;
      auto event=gate_->propose(wire::from_ros(*p),wire::monotonic_ns());
      if(!event) return;
      if(event->accepted) {
        msg::AcceptedCommand output; output.issued_monotonic_ns=pending_issued_; output.proposal=*p;
        awaiting_output_sequence_=p->sequence;
        accepted_->publish(output);
      }
      events_->publish(wire::to_ros(*event));
      admit_latest(wire::monotonic_ns());
    });
    dispatch_=create_subscription<msg::DispatchTick>(p+"/dispatch",rclcpp::QoS(1).best_effort(),[this](msg::DispatchTick::SharedPtr d) {
      if(d->session_id!=config_.session_id || d->epoch!=config_.epoch) return;
      if(d->status==3) {output_fault_=true;return;}
      if(d->output_valid && awaiting_output_sequence_ && d->command_sequence==awaiting_output_sequence_) {
        // An accepted candidate may not yet have reached a periodic output
        // slot. Do not let the worker advance its command history using an
        // overwritten/never-output candidate. This ACK means shadow output;
        // a live adapter must replace it with its execution receipt.
        awaiting_output_sequence_=0; admit_latest(wire::monotonic_ns());
      }
    });
    stop_=create_subscription<msg::RuntimeStop>(p+"/stop",rclcpp::QoS(1).reliable(),[this](msg::RuntimeStop::SharedPtr s) {
      if(s->session_id==config_.session_id && s->epoch==config_.epoch) stopped_=true;
    });
    timer_=create_wall_timer(std::chrono::milliseconds(1),[this] {
      if(stopped_ || output_fault_) return;
      auto now=wire::monotonic_ns();
      if(awaiting_output_sequence_ && now-pending_issued_>50'000'000) {
        output_fault_=true;publish_source(now);return;
      }
      if (invalid_source_) { publish_source(now); return; }
      if(auto e=gate_->poll(now)) events_->publish(wire::to_ros(*e));
      admit_latest(now);
    });
    RCLCPP_INFO(get_logger(),"READY isolated C++ state receiver; simulation-contract shadow only");
  }
 private:
  void admit_latest(std::uint64_t now) {
    if(stopped_ || output_fault_ || invalid_source_ || awaiting_output_sequence_ || !next_admission_ || now<next_admission_ ||
       gate_->latched() || gate_->pending() || !have_latest_ || latest_.control_time_s<=last_control_) return;
    auto s=latest_;
    // Do not lose a complete period just because the previous result crossed
    // a timer edge. Dispatch as soon as idle AND the 20 ms minimum spacing has
    // elapsed; no catch-up burst, no artificial dt, no old-state queue.
    s.sequence=gate_->next_sequence(); s.dt_s=s.control_time_s-last_control_;
    auto a=gate_->admit(wire::from_ros(s),now);
    if(a.event) events_->publish(wire::to_ros(*a.event));
    if(a.forward) {
      last_control_=s.control_time_s; pending_issued_=s.issued_monotonic_ns;
      next_admission_=now+20'000'000; admitted_->publish(s);
    }
  }
  void publish_source(std::uint64_t now) {
    msg::IngressStatus s; s.session_id=config_.session_id; s.epoch=config_.epoch;
    s.source_sequence=source_sequence_; s.received_monotonic_ns=now;
    s.source_issued_monotonic_ns=latest_.issued_monotonic_ns; s.control_time_s=source_time_;
    s.faulted=invalid_source_ || output_fault_ || gate_->latched(); status_->publish(s);
  }
  fcg::Config config_; std::unique_ptr<fcg::ShadowGate> gate_;
  msg::ControlSnapshot latest_; bool have_latest_{}, stopped_{}, invalid_source_{},output_fault_{};
  double last_control_{},source_time_{-1.}; std::uint64_t source_sequence_{},pending_issued_{},next_admission_{};
  std::uint64_t awaiting_output_sequence_{};
  rclcpp::Publisher<msg::ControlSnapshot>::SharedPtr admitted_;
  rclcpp::Publisher<msg::AcceptedCommand>::SharedPtr accepted_;
  rclcpp::Publisher<msg::GateEvent>::SharedPtr events_;
  rclcpp::Publisher<msg::IngressStatus>::SharedPtr status_;
  rclcpp::Subscription<msg::ControlSnapshot>::SharedPtr states_;
  rclcpp::Subscription<msg::ControlProposal>::SharedPtr proposals_;
  rclcpp::Subscription<msg::RuntimeStop>::SharedPtr stop_;
  rclcpp::Subscription<msg::DispatchTick>::SharedPtr dispatch_;
  rclcpp::TimerBase::SharedPtr timer_;
};
int main(int argc,char **argv) {
  rclcpp::init(argc,argv);
  try {rclcpp::spin(std::make_shared<StateIngress>());rclcpp::shutdown();return 0;}
  catch(const std::exception &e) {std::cerr<<e.what()<<'\n';rclcpp::shutdown();return 1;}
}
