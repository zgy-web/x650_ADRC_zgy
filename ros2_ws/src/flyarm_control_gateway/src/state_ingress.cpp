#include "flyarm_control_gateway/runtime_wire.hpp"
#include "flyarm_control_gateway/admission_cadence.hpp"
#include <flyarm_control_interfaces/msg/accepted_command.hpp>
#include <flyarm_control_interfaces/msg/ingress_status.hpp>
#include <flyarm_control_interfaces/msg/runtime_stop.hpp>
#include <flyarm_control_interfaces/msg/dispatch_tick.hpp>
#include <flyarm_control_interfaces/msg/live_start.hpp>
#include <flyarm_control_interfaces/msg/execution_receipt.hpp>
#include <flyarm_control_interfaces/msg/actuator_send.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <vector>

namespace wire=flyarm_runtime_wire;
class StateIngress final : public rclcpp::Node {
 public:
  StateIngress(): Node("isolated_state_ingress") {
    fcg::Config config;
    live_=declare_parameter<bool>("simulation_live",false);
    ready_file_=declare_parameter<std::string>("ready_file","");
    if(live_ && !ready_file_.empty()) timing_.reserve(TIMING_CAPACITY);
    config.replay_only=!live_;
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
      if(stopped_ || (live_ && !started_)) return;
      auto now=wire::monotonic_ns();
      // Source identity, chronology and wall ages are never rewritten to make
      // an old sample look new. Superseded unadmitted STATE samples are allowed.
      if(s->session_id!=config_.session_id || s->epoch!=config_.epoch ||
         s->configuration_sha256!=config_.configuration_sha256 || s->replay_only==live_ ||
         s->schema_version!=1 || s->sequence<=source_sequence_ ||
         !std::isfinite(s->control_time_s) || s->control_time_s<=source_time_ ||
         !s->issued_monotonic_ns || s->issued_monotonic_ns>now ||
         now-s->issued_monotonic_ns>25'000'000) {
        invalid_source_=true; publish_source(now); return;
      }
      source_sequence_=s->sequence; source_time_=s->control_time_s;
      latest_=*s; have_latest_=true; latest_received_=now;
      store_timing(timing(1,s->sequence,now)); publish_source(now);
      if(!next_admission_) next_admission_=now+20'000'000;
      admit_latest(now);
    });
    proposals_=create_subscription<msg::ControlProposal>(p+"/output/proposal",latest,[this](msg::ControlProposal::SharedPtr p) {
      auto trace=timing(2,p->sequence,wire::monotonic_ns());
      if(stopped_ || output_fault_) {trace.blocked_flags=stopped_?1:2;store_timing(trace);return;}
      auto event=gate_->propose(wire::from_ros(*p),wire::monotonic_ns());
      trace.gate_end_ns=wire::monotonic_ns();
      if(!event) {store_timing(trace);return;}
      if(event->accepted) {
        msg::AcceptedCommand output; output.issued_monotonic_ns=pending_issued_; output.proposal=*p;
        awaiting_output_sequence_=p->sequence;
        accepted_->publish(output);
        trace.publish_end_ns=wire::monotonic_ns();trace.forward=true;
      }
      store_timing(trace);
      events_->publish(wire::to_ros(*event));
      admit_latest(wire::monotonic_ns());
    });
    dispatch_=create_subscription<msg::DispatchTick>(p+"/dispatch",rclcpp::QoS(1).best_effort(),[this](msg::DispatchTick::SharedPtr d) {
      if(d->session_id!=config_.session_id || d->epoch!=config_.epoch) return;
      if(d->status==3) {output_fault_=true;return;}
      if(!live_ && d->output_valid && awaiting_output_sequence_ && d->command_sequence==awaiting_output_sequence_) {
        // An accepted candidate may not yet have reached a periodic output
        // slot. Do not let the worker advance its command history using an
        // overwritten/never-output candidate. This ACK means shadow output;
        // a live adapter must replace it with its execution receipt.
        awaiting_output_sequence_=0; admit_latest(wire::monotonic_ns());
      }
    });
    start_=create_subscription<msg::LiveStart>(p+"/live_start",rclcpp::QoS(1).reliable(),[this](msg::LiveStart::SharedPtr s) {
      if(!live_ || stopped_ || started_ || s->session_id!=config_.session_id || s->epoch!=config_.epoch ||
         s->configuration_sha256!=config_.configuration_sha256 || !std::isfinite(s->origin_s)) return;
      last_control_=s->origin_s; started_=true;
    });
    consumed_=create_subscription<msg::ExecutionReceipt>(p+"/consumed",rclcpp::QoS(64).best_effort(),[this](msg::ExecutionReceipt::SharedPtr r) {
      if(!live_ || r->session_id!=config_.session_id || r->epoch!=config_.epoch) return;
      if(r->failed) {output_fault_=true;publish_source(wire::monotonic_ns());return;}
    });
    sent_=create_subscription<msg::ActuatorSend>(p+"/actuator_send",rclcpp::QoS(64).best_effort(),[this](msg::ActuatorSend::SharedPtr s) {
      if(!live_ || s->session_id!=config_.session_id || s->epoch!=config_.epoch || stopped_ || output_fault_) return;
      const auto entry=wire::monotonic_ns();store_timing(timing(3,s->command_sequence,entry));
      // A plan may advance only after the independent writer actually sent
      // it. Joint consumption is checked asynchronously by that writer BEFORE
      // its next output. Do not serialize state sampling/optimization behind
      // a physical receipt: source context still uses only real consumed
      // commands and MIT packets, never this notification as a measurement.
      if(!s->stop_zero && s->command_sequence==awaiting_output_sequence_) {
        const auto now=wire::monotonic_ns();
        if(!s->sent_monotonic_ns || s->sent_monotonic_ns>now || now-s->sent_monotonic_ns>25'000'000 ||
           !std::isfinite(s->sim_s)) {output_fault_=true;publish_source(now);return;}
        if(!cadence_.acknowledge_output(s->command_sequence)) {
          output_fault_=true;publish_source(now);return;
        }
        auto trace=timing(5,s->command_sequence,entry);trace.forward=true;store_timing(trace);
        awaiting_output_sequence_=0;admit_latest(wire::monotonic_ns());
      }
    });
    stop_=create_subscription<msg::RuntimeStop>(p+"/stop",rclcpp::QoS(1).reliable(),[this](msg::RuntimeStop::SharedPtr s) {
      if(s->session_id==config_.session_id && s->epoch==config_.epoch) stopped_=true;
    });
    timer_=create_wall_timer(std::chrono::milliseconds(1),[this] {
      if(!ready_written_ && !ready_file_.empty() && admitted_->get_subscription_count()>=1 &&
         accepted_->get_subscription_count()>=1 && status_->get_subscription_count()>=1 &&
         events_->get_subscription_count()>=2 && consumed_->get_publisher_count()>=1 && sent_->get_publisher_count()>=1) {
        std::ofstream f(ready_file_);f<<"actual DDS endpoints matched\n";f.close();ready_written_=true;
      }
      if(stopped_ || output_fault_) return;
      auto now=wire::monotonic_ns();
      if(awaiting_output_sequence_ && now-pending_issued_>50'000'000) {
        output_fault_=true;publish_source(now);return;
      }
      if (invalid_source_) { publish_source(now); return; }
      if(auto e=gate_->poll(now)) events_->publish(wire::to_ros(*e));
      // Live data arrive asynchronously. The timer must not select a held
      // sample immediately before the next sensor callback; keep the actual
      // source timestamp and admit at new-state/consumption boundaries.
      if(!live_) admit_latest(now);
    });
    RCLCPP_INFO(get_logger(),"READY isolated C++ state receiver; simulation_live=%d",live_);
  }
  ~StateIngress() override {
    if(!live_ || ready_file_.empty()) return;
    // Serialize only at shutdown. A rejected admission never reaches worker
    // CDR recording, so retain its exact fixed-size gate input separately.
    persist_rejected_snapshot();
    try {
      const auto path=std::filesystem::path(ready_file_).parent_path()/"ingress_timing.json";
      std::ofstream f(path);f<<std::setprecision(17);
      f<<"{\"kind\":\"isolated_ingress_timing_v1\",\"complete\":"<<(timing_overflow_?"false":"true")
       <<",\"capacity\":"<<TIMING_CAPACITY<<",\"overflow\":"<<timing_overflow_
       <<",\"columns\":[\"kind\",\"sequence\",\"entry_ns\",\"gate_end_ns\",\"publish_end_ns\","
         "\"source_sequence\",\"source_issued_ns\",\"source_received_ns\",\"latest_control_s\",\"last_control_s\","
         "\"awaiting_output_sequence\",\"next_admission_ns\",\"last_admitted_sequence\",\"released_sequence\",\"blocked_flags\",\"forward\"],\"rows\":[";
      bool first=true;
      for(const auto &t:timing_) {
        if(!first)f<<',';
        first=false;
        f<<'['<<unsigned(t.kind)<<','<<t.sequence<<','<<t.entry_ns<<','<<t.gate_end_ns<<','<<t.publish_end_ns
         <<','<<t.source_sequence<<','<<t.source_issued_ns<<','<<t.source_received_ns
         <<','<<t.latest_control_s<<','<<t.last_control_s<<','<<t.awaiting_output_sequence<<','<<t.next_admission_ns
         <<','<<t.last_admitted_sequence<<','<<t.released_sequence<<','<<t.blocked_flags<<','<<(t.forward?"true":"false")<<']';
      }
      f<<"]}\n";f.close();
      if(!f)std::cerr<<"Could not persist ingress timing evidence\n";
    } catch(const std::exception &e) {std::cerr<<"Ingress timing evidence: "<<e.what()<<'\n';}
  }
 private:
  struct RejectedSnapshot {
    bool present{};
    msg::ControlSnapshot snapshot;
    fcg::Event event;
    std::uint64_t source_sequence{},source_received_ns{},checked_ns{},gate_return_ns{};
  };
  void retain_rejected_snapshot(const msg::ControlSnapshot &snapshot,const fcg::Event &event,
                                std::uint64_t checked,std::uint64_t returned) {
    if(rejected_.present || !live_ || ready_file_.empty()) return;
    // Both messages use only fixed scalar/array fields. This one failure-only
    // copy neither serializes nor performs file I/O in a callback.
    rejected_.snapshot=snapshot;rejected_.event=event;
    rejected_.source_sequence=source_sequence_;rejected_.source_received_ns=latest_received_;
    rejected_.checked_ns=checked;rejected_.gate_return_ns=returned;rejected_.present=true;
  }
  void persist_rejected_snapshot() const noexcept {
    if(!rejected_.present) return;
    try {
      const auto directory=std::filesystem::path(ready_file_).parent_path();
      rclcpp::Serialization<msg::ControlSnapshot> serialization;
      rclcpp::SerializedMessage serialized;
      serialization.serialize_message(&rejected_.snapshot,&serialized);
      const auto &cdr=serialized.get_rcl_serialized_message();
      std::ofstream payload(directory/"rejected_snapshot.cdr",std::ios::binary);
      payload.write(reinterpret_cast<const char *>(cdr.buffer),static_cast<std::streamsize>(cdr.buffer_length));
      payload.close();
      if(!payload) throw std::runtime_error("Could not persist rejected snapshot CDR");
      const auto &s=rejected_.snapshot;const auto &e=rejected_.event;
      std::ofstream metadata(directory/"rejected_snapshot.json");
      metadata<<"{\"kind\":\"isolated_rejected_snapshot_v1\",\"complete\":true,"
        "\"ros_type\":\"flyarm_control_interfaces/msg/ControlSnapshot\","
        "\"snapshot_semantics\":\"exact_gate_input_after_admission_sequence_and_actual_dt_assignment\","
        "\"payload\":\"rejected_snapshot.cdr\",\"bytes\":"<<cdr.buffer_length
        <<",\"session_id\":"<<s.session_id<<",\"epoch\":"<<s.epoch
        <<",\"sequence\":"<<s.sequence<<",\"source_sequence\":"<<rejected_.source_sequence
        <<",\"source_issued_monotonic_ns\":"<<s.issued_monotonic_ns
        <<",\"source_received_monotonic_ns\":"<<rejected_.source_received_ns
        <<",\"checked_monotonic_ns\":"<<rejected_.checked_ns
        <<",\"gate_return_monotonic_ns\":"<<rejected_.gate_return_ns
        <<",\"event_received_monotonic_ns\":"<<e.received_monotonic_ns
        <<",\"event_sequence\":"<<e.sequence<<",\"reason\":"<<static_cast<unsigned>(e.reason)
        <<",\"forward\":false,\"configuration_sha256\":\"";
      for(const auto byte:s.configuration_sha256)
        metadata<<std::hex<<std::setw(2)<<std::setfill('0')<<static_cast<unsigned>(byte);
      metadata<<"\"}\n";metadata.close();
      if(!metadata) throw std::runtime_error("Could not persist rejected snapshot metadata");
    } catch(const std::exception &e) {std::cerr<<"Rejected snapshot evidence: "<<e.what()<<'\n';}
  }
  struct Timing {
    std::uint8_t kind{};std::uint64_t sequence{},entry_ns{},gate_end_ns{},publish_end_ns{};
    std::uint64_t source_sequence{},source_issued_ns{},source_received_ns{};
    double latest_control_s{},last_control_s{};
    std::uint64_t awaiting_output_sequence{},next_admission_ns{},last_admitted_sequence{},released_sequence{};
    std::uint16_t blocked_flags{};bool forward{};
  };
  Timing timing(std::uint8_t kind,std::uint64_t sequence,std::uint64_t entry) const {
    Timing t;t.kind=kind;t.sequence=sequence;t.entry_ns=entry;
    t.source_sequence=source_sequence_;t.source_issued_ns=latest_.issued_monotonic_ns;
    t.source_received_ns=latest_received_;t.latest_control_s=latest_.control_time_s;t.last_control_s=last_control_;
    t.awaiting_output_sequence=awaiting_output_sequence_;t.next_admission_ns=next_admission_;
    t.last_admitted_sequence=cadence_.last_admitted();t.released_sequence=cadence_.released();return t;
  }
  void store_timing(const Timing &t) {
    if(!live_ || ready_file_.empty())return;
    if(timing_.size()<TIMING_CAPACITY)timing_.push_back(t);else ++timing_overflow_;
  }
  void admit_latest(std::uint64_t now) {
    auto trace=timing(4,gate_->next_sequence(),now);
    trace.blocked_flags=(stopped_?1:0)|(output_fault_?2:0)|(invalid_source_?4:0)|
      (awaiting_output_sequence_?8:0)|(!next_admission_?16:0)|
      (!cadence_.due(live_,now,next_admission_,latest_.control_time_s,last_control_)?32:0)|
      (gate_->latched()?64:0)|(gate_->pending()?128:0)|(!have_latest_?256:0)|
      (latest_.control_time_s<=last_control_?512:0);
    if(trace.blocked_flags) {store_timing(trace);return;}
    // Use the latest already received causal observation, including one
    // sampled before the send notification. Requiring another sensor frame
    // serializes reception behind actuation. Original issue/source stamps,
    // chronology, joint age and the 50 ms complete lease remain unchanged.
    auto s=latest_;
    // First live solve waits for 20 ms physical time. Later solves follow a
    // one-use actual-output acknowledgement, never an unsent candidate.
    // State time, actual dt and all independent age checks stay unchanged.
    s.sequence=gate_->next_sequence(); s.dt_s=s.control_time_s-last_control_;
    auto a=gate_->admit(wire::from_ros(s),now);
    trace.gate_end_ns=wire::monotonic_ns();
    if(a.event && !a.forward) retain_rejected_snapshot(s,*a.event,now,trace.gate_end_ns);
    if(a.event) events_->publish(wire::to_ros(*a.event));
    if(a.forward) {
      cadence_.admitted(s.sequence);
      last_control_=s.control_time_s; pending_issued_=s.issued_monotonic_ns;
      next_admission_=now+20'000'000; admitted_->publish(s);
      trace.publish_end_ns=wire::monotonic_ns();trace.forward=true;
    }
    store_timing(trace);
  }
  void publish_source(std::uint64_t now) {
    msg::IngressStatus s; s.session_id=config_.session_id; s.epoch=config_.epoch;
    s.source_sequence=source_sequence_; s.received_monotonic_ns=now;
    s.source_issued_monotonic_ns=latest_.issued_monotonic_ns; s.control_time_s=source_time_;
    s.faulted=invalid_source_ || output_fault_ || gate_->latched(); status_->publish(s);
  }
  fcg::Config config_; std::unique_ptr<fcg::ShadowGate> gate_;
  fcg::AdmissionCadence cadence_;
  static constexpr std::size_t TIMING_CAPACITY=32768;
  std::vector<Timing> timing_;std::uint64_t timing_overflow_{},latest_received_{};
  RejectedSnapshot rejected_;
  bool live_{},started_{};
  std::string ready_file_;bool ready_written_{};
  rclcpp::Subscription<msg::LiveStart>::SharedPtr start_;
  rclcpp::Subscription<msg::ExecutionReceipt>::SharedPtr consumed_;
  rclcpp::Subscription<msg::ActuatorSend>::SharedPtr sent_;
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
