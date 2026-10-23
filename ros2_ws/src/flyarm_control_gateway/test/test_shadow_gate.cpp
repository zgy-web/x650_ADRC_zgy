// Pure synthetic fake-clock tests. No ROS graph, simulation or hardware.
#include "flyarm_control_gateway/shadow_gate.hpp"
#include <functional>
#include <iostream>
#include <string>
#include <vector>

namespace fcg = flyarm_control_gateway;
namespace {
constexpr std::uint64_t ISSUE=1'000'000'000ULL;
constexpr std::uint64_t RECEIPT=ISSUE+1'000'000ULL;
constexpr std::uint64_t RETURN=ISSUE+11'000'000ULL;

void require(bool condition, const char * message) {
  if (!condition) throw std::runtime_error(message);
}

fcg::Config config() {
  fcg::Config value;
  value.session_id=23; value.epoch=1; value.configuration_sha256.fill(7);
  value.position_gains={.949999988079071, .949999988079071, 1.};
  return value;
}

fcg::Snapshot snapshot() {
  fcg::Snapshot value;
  const auto cfg=config();
  value.session_id=cfg.session_id; value.epoch=cfg.epoch; value.configuration_sha256=cfg.configuration_sha256;
  value.sequence=1; value.issued_monotonic_ns=ISSUE;
  value.control_time_s=10.02; value.task_time_s=1.02; value.dt_s=.02;
  value.state_time_s=value.control_time_s-.004; value.joint_time_s=value.control_time_s-.004;
  value.position_enu={1.,2.,3.}; value.rotation_row_major={1.,0.,0.,0.,1.,0.,0.,0.,1.};
  value.position_gains=cfg.position_gains;
  value.reference_time_s=value.joint_time_s-(value.control_time_s-value.task_time_s);
  value.bus_origin_s=-1.; value.command_count=1; value.packet_count=1;
  value.command_time_s[0]=.9; value.packet_send_time_s[0]=.9;
  value.packet_kp[0]=6.; value.packet_kp[1]=6.; value.packet_kd[0]=.25; value.packet_kd[1]=.25;
  return value;
}

fcg::Proposal proposal(const fcg::Snapshot & input=snapshot()) {
  fcg::Proposal value;
  value.session_id=input.session_id; value.epoch=input.epoch; value.configuration_sha256=input.configuration_sha256;
  value.sequence=input.sequence; value.snapshot_sequence=input.sequence;
  value.compute_start_monotonic_ns=input.issued_monotonic_ns+2'000'000;
  value.compute_end_monotonic_ns=input.issued_monotonic_ns+10'000'000;
  value.control_time_s=input.control_time_s; value.task_time_s=input.task_time_s;
  value.position_reference_enu=input.position_enu;
  value.base_velocity_target={.1,-.1,.05}; value.velocity_feedforward_enu=value.base_velocity_target;
  value.joint_velocity={.1,-.1}; value.accepted_optimum=true; value.nonlinear_check_passed=true;
  value.primal_violation=1e-8; value.kkt_residual=1e-4; value.subproblems=1;
  return value;
}

void rejected(const std::optional<fcg::Event> & event, fcg::Reason reason) {
  require(event.has_value(), "missing rejection event");
  require(!event->accepted, "rejection became an accepted shadow command");
  require(event->reason==reason, "wrong rejection reason");
}

void reject_input(const std::function<void(fcg::Snapshot &)> & change, fcg::Reason reason) {
  fcg::ShadowGate gate(config());
  auto input=snapshot(); change(input);
  auto admission=gate.admit(input, RECEIPT);
  require(!admission.forward, "invalid input was forwarded");
  rejected(admission.event, reason);
  require(gate.latched() && !gate.pending() && gate.next_sequence()==1, "input rejection state advanced");
}

void reject_proposal(const std::function<void(fcg::Proposal &)> & change, fcg::Reason reason,
                     std::uint64_t now=RETURN) {
  fcg::ShadowGate gate(config());
  require(gate.admit(snapshot(), RECEIPT).forward, "valid setup input failed");
  auto output=proposal(); change(output);
  rejected(gate.propose(output, now), reason);
  require(gate.latched() && !gate.pending() && gate.next_sequence()==1, "proposal rejection state advanced");
}

void run(const char * name, const std::function<void()> & test, unsigned & passed) {
  test(); ++passed; std::cout << "PASS synthetic " << name << '\n';
}
}  // namespace

int main() {
  unsigned passed=0;
  try {
    run("live_mode_never_implicitly_accepts_shadow_or_live_identity", [] {
      auto cfg=config();cfg.replay_only=false;
      fcg::ShadowGate live(cfg);
      rejected(live.admit(snapshot(),RECEIPT).event,fcg::Reason::REPLAY_REQUIRED);
      fcg::ShadowGate valid(cfg);auto s=snapshot();s.replay_only=false;
      require(valid.admit(s,RECEIPT).forward,"explicit live snapshot rejected");
      auto p=proposal(s);p.replay_only=false;
      auto e=valid.propose(p,RETURN);
      require(e && e->accepted && !e->replay_only,"live event incorrectly labelled shadow");
    },passed);
    run("explicit_configuration_required", [] {
      for (unsigned which=0; which<5; ++which) {
        auto cfg=config();
        if (which==0) cfg.session_id=0;
        if (which==1) cfg.epoch=0;
        if (which==2) cfg.configuration_sha256.fill(0);
        if (which==3) cfg.position_gains[1]=0.;
        if (which==4) cfg.position_gains[1]=std::numeric_limits<double>::infinity();
        bool caught=false;
        try { fcg::ShadowGate invalid(cfg); } catch (const std::invalid_argument &) { caught=true; }
        require(caught, "invalid startup configuration accepted");
      }
    }, passed);
    run("valid_admit_accept_identity_and_roundtrip", [] {
      fcg::ShadowGate gate(config());
      auto admission=gate.admit(snapshot(), RECEIPT);
      require(admission.forward && !admission.event && gate.pending(), "valid snapshot not causally forwarded");
      const auto accepted=gate.propose(proposal(), RETURN);
      require(accepted && accepted->accepted && accepted->reason==fcg::Reason::ACCEPTED, "valid command rejected");
      require(accepted->session_id==23 && accepted->epoch==1 && accepted->sequence==1 &&
              accepted->snapshot_sequence==1 && accepted->proposal_sequence==1, "event identity mismatch");
      require(accepted->roundtrip_ns==11'000'000 && accepted->received_monotonic_ns==RETURN, "fake-clock measurement mismatch");
      require(gate.next_sequence()==2 && !gate.pending() && !gate.latched(), "acceptance state incorrect");
    }, passed);
    run("snapshot_and_proposal_identity_replay_guards", [] {
      for (unsigned which=0; which<5; ++which) {
        const auto mutate=[which](auto & value) {
          if (which==0) value.session_id++;
          if (which==1) value.epoch++;
          if (which==2) value.configuration_sha256[7]++;
          if (which==3) value.schema_version=2;
          if (which==4) value.replay_only=false;
        };
        reject_input(mutate, which==4 ? fcg::Reason::REPLAY_REQUIRED : fcg::Reason::IDENTITY);
        reject_proposal(mutate, which==4 ? fcg::Reason::REPLAY_REQUIRED : fcg::Reason::IDENTITY);
      }
    }, passed);
    run("single_inflight_and_strict_sequences", [] {
      reject_input([](auto & value) { value.sequence=2; }, fcg::Reason::SEQUENCE);
      reject_proposal([](auto & value) { value.sequence=2; }, fcg::Reason::SEQUENCE);
      reject_proposal([](auto & value) { value.snapshot_sequence=2; }, fcg::Reason::SEQUENCE);
      fcg::ShadowGate gate(config());
      require(gate.admit(snapshot(), RECEIPT).forward, "setup rejected");
      rejected(gate.admit(snapshot(), RECEIPT+1).event, fcg::Reason::INFLIGHT);
    }, passed);
    run("proposal_without_admission_cannot_publish", [] {
      fcg::ShadowGate gate(config());
      rejected(gate.propose(proposal(), RETURN), fcg::Reason::SEQUENCE);
    }, passed);
    run("every_numeric_input_category_rejects_nonfinite", [] {
      const auto nan=std::numeric_limits<double>::quiet_NaN();
      const std::vector<std::function<void(fcg::Snapshot &)>> changes{
        [nan](auto & v){v.control_time_s=nan;}, [nan](auto & v){v.task_time_s=nan;},
        [nan](auto & v){v.dt_s=nan;}, [nan](auto & v){v.state_time_s=nan;},
        [nan](auto & v){v.joint_time_s=nan;}, [nan](auto & v){v.reference_time_s=nan;},
        [nan](auto & v){v.bus_origin_s=nan;}, [nan](auto & v){v.position_enu[0]=nan;},
        [nan](auto & v){v.velocity_enu[0]=nan;}, [nan](auto & v){v.omega_body_flu[0]=nan;},
        [nan](auto & v){v.acceleration_enu[0]=nan;}, [nan](auto & v){v.rotation_row_major[0]=nan;},
        [nan](auto & v){v.q[0]=nan;}, [nan](auto & v){v.dq[0]=nan;},
        [nan](auto & v){v.target_position_enu[0]=nan;}, [nan](auto & v){v.target_velocity_enu[0]=nan;},
        [nan](auto & v){v.target_acceleration_enu[0]=nan;}, [nan](auto & v){v.position_gains[0]=nan;},
        [nan](auto & v){v.reference_r[0]=nan;}, [nan](auto & v){v.command_time_s[0]=nan;},
        [nan](auto & v){v.packet_send_time_s[0]=nan;}, [nan](auto & v){v.command_velocity[0]=nan;},
        [nan](auto & v){v.packet_q_ref[0]=nan;}, [nan](auto & v){v.packet_dq_ref[0]=nan;},
        [nan](auto & v){v.packet_tau_ff[0]=nan;}, [nan](auto & v){v.packet_kp[0]=nan;},
        [nan](auto & v){v.packet_kd[127]=nan;}
      };
      for (const auto & change : changes) reject_input(change, fcg::Reason::INPUT);
      reject_input([](auto & v){v.velocity_enu[2]=std::numeric_limits<double>::infinity();}, fcg::Reason::INPUT);
    }, passed);
    run("proper_rotation_and_pid_readback", [] {
      reject_input([](auto & v){v.rotation_row_major[0]=-1.;}, fcg::Reason::FRAME);
      reject_input([](auto & v){v.rotation_row_major[1]=.1;}, fcg::Reason::FRAME);
      reject_input([](auto & v){v.position_gains[0]=1.;}, fcg::Reason::INPUT);
      reject_input([](auto & v){v.target_acceleration_enu[0]=.001;}, fcg::Reason::INPUT);
      auto input=snapshot(); input.rotation_row_major={0.,-1.,0.,1.,0.,0.,0.,0.,1.};
      fcg::ShadowGate gate(config()); require(gate.admit(input, RECEIPT).forward, "proper rotated frame rejected");
    }, passed);
    run("base_250ms_joint_25ms_boundaries_and_future", [] {
      auto input=snapshot(); input.state_time_s=input.control_time_s-.25;
      input.joint_time_s=input.control_time_s-.025;
      input.reference_time_s=input.joint_time_s-(input.control_time_s-input.task_time_s);
      fcg::ShadowGate gate(config()); require(gate.admit(input, RECEIPT).forward, "exact source age boundary rejected");
      reject_input([](auto & v){v.state_time_s=std::nextafter(v.control_time_s-.25, -INFINITY);}, fcg::Reason::SOURCE_AGE);
      reject_input([](auto & v){v.joint_time_s=std::nextafter(v.control_time_s-.025, -INFINITY);}, fcg::Reason::SOURCE_AGE);
      reject_input([](auto & v){v.state_time_s=std::nextafter(v.control_time_s, INFINITY);}, fcg::Reason::SOURCE_AGE);
      reject_input([](auto & v){v.joint_time_s=std::nextafter(v.control_time_s, INFINITY);}, fcg::Reason::SOURCE_AGE);
    }, passed);
    run("fixed_bounded_context_counts_padding_causality", [] {
      const std::vector<std::function<void(fcg::Snapshot &)>> changes{
        [](auto & v){v.command_count=0;}, [](auto & v){v.command_count=65;},
        [](auto & v){v.packet_count=0;}, [](auto & v){v.packet_count=65;},
        [](auto & v){v.command_time_s[63]=1.;}, [](auto & v){v.command_velocity[127]=.1;},
        [](auto & v){v.packet_q_ref[127]=.1;}, [](auto & v){v.packet_dq_ref[127]=.1;},
        [](auto & v){v.packet_tau_ff[127]=.1;}, [](auto & v){v.packet_kp[127]=.1;},
        [](auto & v){v.packet_kd[127]=.1;}, [](auto & v){v.packet_send_time_s[63]=1.;},
        [](auto & v){v.command_time_s[0]=v.task_time_s+.001;},
        [](auto & v){v.packet_send_time_s[0]=v.task_time_s+.001;},
        [](auto & v){v.packet_send_time_s[0]=v.bus_origin_s-.001;},
        [](auto & v){v.reference_time_s+=.001;}, [](auto & v){v.packet_kp[0]=-1.;},
        [](auto & v){v.command_velocity[1]=.351;}
      };
      for (const auto & change : changes) reject_input(change, fcg::Reason::CONTEXT);
    }, passed);
    run("snapshot_context_copy_is_immutable", [] {
      fcg::ShadowGate gate(config()); auto input=snapshot(); auto output=proposal(input);
      require(gate.admit(input, RECEIPT).forward, "setup rejected");
      input.position_enu.fill(1e9); input.position_gains.fill(99.); input.reference_r.fill(1e9);
      input.configuration_sha256.fill(0);
      require(gate.propose(output, RETURN)->accepted, "caller mutation changed frozen input");
    }, passed);
    run("wall_deadline_includes_transport_not_claimed_compute", [] {
      reject_input([](auto & v){v.issued_monotonic_ns=RECEIPT-50'000'001;}, fcg::Reason::DEADLINE);
      reject_input([](auto & v){v.issued_monotonic_ns=RECEIPT+1;}, fcg::Reason::CLOCK_REGRESSION);
      reject_proposal([](auto & v){v.compute_end_monotonic_ns=v.compute_start_monotonic_ns;},
                      fcg::Reason::DEADLINE, ISSUE+50'000'001);
      fcg::ShadowGate gate(config()); require(gate.admit(snapshot(), RECEIPT).forward, "setup rejected");
      require(gate.propose(proposal(), ISSUE+50'000'000)->accepted, "exact wall deadline rejected");
    }, passed);
    run("timeout_one_event_and_epoch_latch", [] {
      fcg::ShadowGate gate(config()); require(gate.admit(snapshot(), RECEIPT).forward, "setup rejected");
      require(!gate.poll(ISSUE+50'000'000), "timeout before allowed boundary");
      const auto timeout=gate.poll(ISSUE+50'000'001); rejected(timeout, fcg::Reason::TIMEOUT);
      require(timeout->proposal_sequence==0 && timeout->snapshot_sequence==1, "timeout identity wrong");
      require(!gate.poll(ISSUE+60'000'000) && !gate.propose(proposal(), ISSUE+61'000'000), "latched epoch emitted repeated rejection");
      require(!gate.admit(snapshot(), ISSUE+62'000'000).forward && gate.next_sequence()==1, "latch reset itself");
    }, passed);
    run("compute_start_end_provenance", [] {
      reject_proposal([](auto & v){v.compute_start_monotonic_ns=RECEIPT-1;}, fcg::Reason::DEADLINE);
      reject_proposal([](auto & v){v.compute_end_monotonic_ns=v.compute_start_monotonic_ns-1;}, fcg::Reason::DEADLINE);
      reject_proposal([](auto & v){v.compute_end_monotonic_ns=RETURN+1;}, fcg::Reason::DEADLINE);
    }, passed);
    run("proposal_finite_time_and_optimizer_contract", [] {
      const std::vector<std::function<void(fcg::Proposal &)>> numeric{
        [](auto & v){v.control_time_s+=.001;}, [](auto & v){v.task_time_s+=.001;},
        [](auto & v){v.position_reference_enu[0]=NAN;}, [](auto & v){v.velocity_feedforward_enu[0]=NAN;},
        [](auto & v){v.acceleration_feedforward_enu[0]=NAN;}, [](auto & v){v.base_velocity_target[0]=NAN;},
        [](auto & v){v.joint_velocity[0]=NAN;}, [](auto & v){v.primal_violation=NAN;},
        [](auto & v){v.kkt_residual=NAN;}
      };
      for (const auto & change:numeric) reject_proposal(change, fcg::Reason::PROPOSAL);
      const std::vector<std::function<void(fcg::Proposal &)>> solver{
        [](auto & v){v.status=1;}, [](auto & v){v.status=2;}, [](auto & v){v.status=3;},
        [](auto & v){v.accepted_optimum=false;}, [](auto & v){v.nonlinear_check_passed=false;},
        [](auto & v){v.primal_violation=std::nextafter(2e-7, INFINITY);},
        [](auto & v){v.kkt_residual=std::nextafter(.002, INFINITY);},
        [](auto & v){v.primal_violation=-1e-9;}, [](auto & v){v.kkt_residual=-1e-9;},
        [](auto & v){v.subproblems=0;}, [](auto & v){v.subproblems=4;}
      };
      for (const auto & change:solver) reject_proposal(change, fcg::Reason::OPTIMIZATION);
    }, passed);
    run("limits_reconstruction_and_zero_acceleration", [] {
      for (std::size_t joint=0; joint<2; ++joint) {
        reject_input([joint](auto & v){v.q[joint]=std::nextafter(fcg::ShadowGate::Q_MIN[joint], -INFINITY);}, fcg::Reason::LIMIT);
        reject_input([joint](auto & v){v.q[joint]=std::nextafter(fcg::ShadowGate::Q_MAX[joint], INFINITY);}, fcg::Reason::LIMIT);
      }
      for (std::size_t axis=0; axis<3; ++axis) {
        reject_proposal([axis](auto & v){v.base_velocity_target[axis]=std::nextafter(fcg::ShadowGate::BASE_LIMIT[axis], INFINITY);}, fcg::Reason::LIMIT);
        reject_proposal([axis](auto & v){v.acceleration_feedforward_enu[axis]=1e-15;}, fcg::Reason::LIMIT);
      }
      for (std::size_t joint=0; joint<2; ++joint)
        reject_proposal([joint](auto & v){v.joint_velocity[joint]=-std::nextafter(fcg::ShadowGate::JOINT_LIMIT[joint], INFINITY);}, fcg::Reason::LIMIT);
      reject_proposal([](auto & v){v.position_reference_enu[0]+=.001;}, fcg::Reason::RECONSTRUCTION);
      fcg::ShadowGate gate(config()); gate.admit(snapshot(), RECEIPT); auto output=proposal();
      output.base_velocity_target=fcg::ShadowGate::BASE_LIMIT; output.velocity_feedforward_enu=output.base_velocity_target;
      output.joint_velocity=fcg::ShadowGate::JOINT_LIMIT; output.primal_violation=2e-7; output.kkt_residual=.002; output.subproblems=3;
      require(gate.propose(output, RETURN)->accepted, "exact numeric limits rejected");
    }, passed);
    run("clock_regression_and_dt_never_silently_reset", [] {
      fcg::ShadowGate gate(config()); gate.admit(snapshot(), RECEIPT);
      rejected(gate.poll(RECEIPT-1), fcg::Reason::CLOCK_REGRESSION);
      reject_input([](auto & v){v.dt_s=0.;}, fcg::Reason::DEADLINE);
      reject_input([](auto & v){v.dt_s=std::nextafter(.05, INFINITY);}, fcg::Reason::DEADLINE);
      for (unsigned which=0; which<3; ++which) {
        fcg::ShadowGate next(config()); next.admit(snapshot(), RECEIPT); next.propose(proposal(), RETURN);
        auto input=snapshot(); input.sequence=2; input.issued_monotonic_ns=ISSUE+20'000'000;
        if (which==0) input.control_time_s-=.02;
        if (which==1) {input.control_time_s+=.06; input.dt_s=.06;}
        if (which==2) input.control_time_s+=.04;
        const auto result=next.admit(input, ISSUE+21'000'000);
        rejected(result.event, which==0 ? fcg::Reason::CLOCK_REGRESSION : fcg::Reason::DEADLINE);
        require(next.next_sequence()==2, "rejected second command advanced accepted sequence");
      }
    }, passed);
    run("bounded_long_sequence_and_no_recorder_ack_dependency", [] {
      fcg::ShadowGate gate(config());
      const auto fixed_bytes=sizeof(gate);
      // 10,000 synthetic rounds: no retained rows, vector, queue or receipts.
      for (std::uint64_t index=0; index<10'000; ++index) {
        auto input=snapshot(); input.sequence=index+1; input.control_time_s+=static_cast<double>(index)*.02;
        input.task_time_s+=static_cast<double>(index)*.02; input.state_time_s=input.control_time_s-.004;
        input.joint_time_s=input.control_time_s-.004;
        input.reference_time_s=input.joint_time_s-(input.control_time_s-input.task_time_s);
        input.issued_monotonic_ns=ISSUE+index*20'000'000;
        require(gate.admit(input, input.issued_monotonic_ns+1'000'000).forward, "ordered round not admitted");
        const auto event=gate.propose(proposal(input), input.issued_monotonic_ns+11'000'000);
        require(event && event->accepted && event->sequence==index+1, "ordered round not accepted");
        require(sizeof(gate)==fixed_bytes, "fixed gateway state size changed");
      }
      require(gate.next_sequence()==10'001 && !gate.pending(), "bounded sequence final state wrong");
    }, passed);
    run("new_process_explicit_epoch_required_for_recovery", [] {
      fcg::ShadowGate old(config()); old.admit(snapshot(), RECEIPT); old.poll(ISSUE+60'000'000);
      require(!old.admit(snapshot(), ISSUE+70'000'000).forward, "failed epoch revived");
      auto cfg=config(); ++cfg.epoch; fcg::ShadowGate fresh(cfg); auto input=snapshot(); input.epoch=cfg.epoch;
      require(fresh.admit(input, RECEIPT).forward, "explicit new epoch could not start");
      require(fresh.propose(proposal(input), RETURN)->accepted, "explicit new epoch valid proposal rejected");
    }, passed);
    std::cout << "PASS " << passed << " synthetic groups; no flight/performance certificate\n";
    return 0;
  } catch (const std::exception & error) {
    std::cerr << "FAIL after " << passed << " synthetic groups: " << error.what() << '\n';
    return 1;
  }
}
