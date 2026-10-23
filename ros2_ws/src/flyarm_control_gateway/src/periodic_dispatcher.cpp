#include "flyarm_control_gateway/runtime_wire.hpp"
#include "flyarm_control_gateway/periodic_gate.hpp"
#include <flyarm_control_interfaces/msg/accepted_command.hpp>
#include <flyarm_control_interfaces/msg/ingress_status.hpp>
#include <flyarm_control_interfaces/msg/dispatch_tick.hpp>
#include <flyarm_control_interfaces/msg/runtime_stop.hpp>
#include <rclcpp/wait_set.hpp>
#include <fstream>
#include <thread>
#ifdef FLYARM_LIVE_SIM
#include "flyarm_control_gateway/live_actuator.hpp"
#endif
namespace wire=flyarm_runtime_wire;

class PeriodicDispatcher final: public rclcpp::Node {
 public:
  PeriodicDispatcher(): Node("isolated_periodic_dispatcher") {
    session_=declare_parameter<std::int64_t>("expected_session_id",0);
    epoch_=declare_parameter<std::int64_t>("expected_epoch",0);
    hash_=wire::parse_hash(declare_parameter<std::string>("configuration_sha256",""));
    live_=declare_parameter<bool>("simulation_live",false);
    ready_file_=declare_parameter<std::string>("ready_file","");
#ifdef FLYARM_LIVE_SIM
    if(live_) {
      actuator_=std::make_unique<wire::LiveActuator>(*this);
      start_=create_subscription<msg::LiveStart>("/flyarm/isolated/v1/live_start",rclcpp::QoS(1).reliable(),[](msg::LiveStart::SharedPtr){});
      actual_=create_subscription<msg::ActuatorState>("/flyarm/isolated/v1/actuator_state",rclcpp::QoS(256).best_effort(),[](msg::ActuatorState::SharedPtr){});
    }
#else
    if(live_) throw std::invalid_argument("Use explicitly compiled live_dispatcher for simulation output");
#endif
    if(!session_ || !epoch_) throw std::invalid_argument("Explicit epoch required");
    const std::string p="/flyarm/isolated/v1"; const auto qos=rclcpp::QoS(1).best_effort();
    ticks_=create_publisher<msg::DispatchTick>(p+"/dispatch",rclcpp::QoS(256).best_effort());
    // Never added to an Executor: a bounded wait-set loop takes queued input
    // BEFORE the output slot. A timer must not overtake a queued candidate.
    source_=create_subscription<msg::IngressStatus>(p+"/source_status",qos,[](msg::IngressStatus::SharedPtr){});
    accepted_=create_subscription<msg::AcceptedCommand>(p+"/accepted",qos,[](msg::AcceptedCommand::SharedPtr){});
    events_=create_subscription<msg::GateEvent>(p+"/event",rclcpp::QoS(256).best_effort(),[](msg::GateEvent::SharedPtr){});
    stop_=create_subscription<msg::RuntimeStop>(p+"/stop",rclcpp::QoS(1).reliable(),[](msg::RuntimeStop::SharedPtr){});
    RCLCPP_INFO(get_logger(),"READY independent absolute 20 ms wait-set publisher; simulation_live=%s", live_ ? "true" : "false");
  }
  void run() {
#ifdef FLYARM_LIVE_SIM
    if(live_) {
      // DDS graph discovery and readiness file I/O are startup operations,
      // not part of the absolute output schedule. No command is taken here.
      const auto begin=wire::monotonic_ns();
      while(rclcpp::ok() && !(actuator_->endpoints_ready() &&
          source_->get_publisher_count()>=1 && accepted_->get_publisher_count()>=1 &&
          events_->get_publisher_count()>=1 && actual_->get_publisher_count()>=1))
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
      if(!rclcpp::ok()) return;
      if(!ready_file_.empty()) {
        std::ofstream f(ready_file_);
        f<<"actual DDS and bridge endpoints matched before timer start\n"
         <<"begin_ns="<<begin<<" ready_ns="<<wire::monotonic_ns()<<'\n';
        f.close();
        if(!f) throw std::runtime_error("Could not persist live readiness");
      }
    }
#endif
    rclcpp::WaitSet wait;
    wait.add_subscription(stop_); wait.add_subscription(source_);
    wait.add_subscription(accepted_); wait.add_subscription(events_);
#ifdef FLYARM_LIVE_SIM
    if(live_) {wait.add_subscription(start_);wait.add_subscription(actual_);}
#endif
    due_=wire::monotonic_ns()+fcg::PeriodicGate::PERIOD_NS;
    while(rclcpp::ok()) {
      drain();
      auto now=wire::monotonic_ns();
      if(now>=due_) {
        // Second bounded take closes the ready-input race after the wakeup.
        // It does not wait for a future candidate or extend a command lease.
        drain(); now=wire::monotonic_ns(); publish_tick(now);
        due_+=fcg::PeriodicGate::PERIOD_NS;
        while(due_<=now) due_+=fcg::PeriodicGate::PERIOD_NS;
      }
      now=wire::monotonic_ns();
      const auto remaining=now<due_ ? due_-now : 0;
      const auto result=wait.wait(std::chrono::nanoseconds(remaining));
      (void)result;
    }
  }
 private:
  void drain() {
    rclcpp::MessageInfo info;
#ifdef FLYARM_LIVE_SIM
    if(live_) {
      msg::LiveStart s;
      if(start_->take(s,info)) {
        if(s.session_id!=session_ || s.epoch!=epoch_ || s.configuration_sha256!=hash_) gate_.reject();
        else actuator_->begin(s);
      }
      msg::ActuatorState a;
      for(int i=0;i<64 && actual_->take(a,info);++i) actuator_->state(a);
      if(actuator_->failed()) gate_.reject();
    }
#endif
    msg::RuntimeStop stop;
    if(stop_->take(stop,info) && stop.session_id==session_ && stop.epoch==epoch_) gate_.stop();
    msg::IngressStatus s;
    if(source_->take(s,info)) {
      if(s.session_id!=session_ || s.epoch!=epoch_ || s.faulted) gate_.reject();
      else gate_.source(s.source_sequence,s.received_monotonic_ns,s.source_issued_monotonic_ns,wire::monotonic_ns());
    }
    msg::AcceptedCommand a;
    if(accepted_->take(a,info)) {
      auto &c=a.proposal;
      if(c.session_id!=session_ || c.epoch!=epoch_ || c.configuration_sha256!=hash_ ||
         c.replay_only==live_ || c.status!=0 || !c.accepted_optimum || !c.nonlinear_check_passed) gate_.reject();
      else if(gate_.accept(c.sequence,a.issued_monotonic_ns,wire::monotonic_ns())) {
        command_=c;
        // Idle READY ticks have no actuator owner or control phase. Anchor
        // the active absolute schedule once, at the first accepted command,
        // instead of inheriting an arbitrary process-start timer phase.
        // Subsequent commands cannot retime ticks or extend their leases.
        if(live_ && !active_phase_) {due_=wire::monotonic_ns();active_phase_=true;}
      }
    }
    msg::GateEvent e;
    for(int i=0;i<16 && events_->take(e,info);++i)
      if(e.session_id!=session_ || e.epoch!=epoch_ || !e.accepted) gate_.reject();
  }
  void publish_tick(std::uint64_t now) {
    auto t=gate_.tick(due_,now);
    msg::DispatchTick out; out.session_id=session_;out.epoch=epoch_;out.sequence=t.sequence;
    out.scheduled_monotonic_ns=t.scheduled;out.actual_monotonic_ns=t.actual;
    out.source_sequence=t.source_sequence;out.source_received_monotonic_ns=t.source_received;
    out.command_sequence=t.command_sequence;out.command_issued_monotonic_ns=t.issued;
    out.command_age_ns=t.age;out.status=t.status;out.reason=t.reason;out.output_valid=t.valid;out.repeated=t.repeated;
    if(t.valid) out.command=command_;
#ifdef FLYARM_LIVE_SIM
    if(live_) actuator_->execute(out);
#endif
    ticks_->publish(out);
  }
  std::uint64_t session_{},epoch_{},due_{};fcg::Hash hash_{};fcg::PeriodicGate gate_;
  msg::ControlProposal command_;
  bool live_{},active_phase_{};
  std::string ready_file_;
#ifdef FLYARM_LIVE_SIM
  std::unique_ptr<wire::LiveActuator> actuator_;
  rclcpp::Subscription<msg::LiveStart>::SharedPtr start_;
  rclcpp::Subscription<msg::ActuatorState>::SharedPtr actual_;
#endif
  rclcpp::Publisher<msg::DispatchTick>::SharedPtr ticks_;
  rclcpp::Subscription<msg::IngressStatus>::SharedPtr source_;
  rclcpp::Subscription<msg::AcceptedCommand>::SharedPtr accepted_;
  rclcpp::Subscription<msg::GateEvent>::SharedPtr events_;
  rclcpp::Subscription<msg::RuntimeStop>::SharedPtr stop_;
};
int main(int argc,char **argv) {
  rclcpp::init(argc,argv);
  try {std::make_shared<PeriodicDispatcher>()->run();rclcpp::shutdown();return 0;}
  catch(const std::exception &e) {
    if(!rclcpp::ok()) return 0;
    std::cerr<<e.what()<<'\n';rclcpp::shutdown();return 1;
  }
}
