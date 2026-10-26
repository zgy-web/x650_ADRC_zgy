#pragma once
#include "shadow_gate.hpp"
#include <algorithm>

namespace flyarm_control_gateway {
// No ROS, locks, dynamic allocation, disk I/O or Python on the decision path.
// This is an output lease, NOT a fallback controller. A repeat retains its
// original snapshot issue time; receiving/repeating it never renews the lease.
class PeriodicGate {
 public:
  static constexpr std::uint64_t PERIOD_NS=20'000'000, LEASE_NS=50'000'000;
  static constexpr std::uint64_t RX_AGE_NS=25'000'000;
  explicit PeriodicGate(TimingProfile profile={}):profile_(profile){}
  enum Status : std::uint8_t { WAITING=0, NEW=1, REPEATED=2, FAULT=3, STOPPED=4 };
  enum Fault : std::uint16_t { NONE=0, STALE_COMMAND=1, STALE_SOURCE=2,
    BACKWARD_CLOCK=3, GATE_REJECTED=4, BAD_COMMAND=5, MISSED_DEADLINE=6,
    CONSUMPTION_TIMEOUT=7, STALE_ACTUATOR_STATE=8, ACTUATOR_PROTOCOL=9,
    ACTUATOR_SEND_FAILED=10, ACTUAL_OUTPUT_GAP=11, CONSUMPTION_PROGRESS_EXPIRED=12,
    CONSUMPTION_LEDGER_OVERFLOW=13 };
  struct Tick { Status status{WAITING}; Fault reason{NONE}; bool valid{}, repeated{};
    std::uint64_t sequence{}, scheduled{}, actual{}, command_sequence{}, issued{}, age{}, source_sequence{}, source_received{}; };

  void source(std::uint64_t seq, std::uint64_t received, std::uint64_t issued, std::uint64_t now,
              double control_time_s=std::numeric_limits<double>::quiet_NaN()) {
    if (stopped_ || fault_) return;
    if (!clock(now) || received>now || issued>received || !issued || seq<=source_seq_ || received<source_received_) {
      fail(BACKWARD_CLOCK); return;
    }
    if(profile_.multirate() && (!std::isfinite(control_time_s) || control_time_s<0. ||
         (source_seq_ && control_time_s<source_control_s_))) {fail(BACKWARD_CLOCK);return;}
    source_seq_=seq; source_received_=received; source_issued_=issued;
    if(profile_.multirate()) source_control_s_=control_time_s;
  }
  bool accept(std::uint64_t seq, std::uint64_t issued, std::uint64_t now,
              double command_control_time_s=std::numeric_limits<double>::quiet_NaN()) {
    if (stopped_ || fault_) return false;
    if (!clock(now) || !issued || issued>now || issued<command_issued_ || seq<=command_seq_) {
      fail(BAD_COMMAND); return false;
    }
    if (now-issued>profile_.command_lease_ns()) { fail(STALE_COMMAND); return false; }
    if(profile_.multirate() && (!std::isfinite(command_control_time_s) || command_control_time_s<0. ||
         (command_seq_ && command_control_time_s<command_control_s_))) {fail(BAD_COMMAND);return false;}
    if(profile_.multirate() && source_seq_ && source_control_s_>command_control_time_s+profile_.command_hold_s()) {
      fail(STALE_COMMAND);return false;
    }
    if(profile_.multirate()) command_control_s_=command_control_time_s;
    command_seq_=seq; command_issued_=issued; return true;
  }
  void reject(Fault reason=GATE_REJECTED) { fail(reason); }
  void stop() { stopped_=true; }
  bool faulted() const { return fault_!=NONE; }
  // Inspection latches genuine clock/age failures, but never emits a tick,
  // commits a command, renews its issue time, or changes the absolute phase.
  Tick preview(std::uint64_t scheduled, std::uint64_t now) {
    Tick t; t.scheduled=scheduled; t.actual=now;
    t.command_sequence=command_seq_; t.issued=command_issued_;
    t.source_sequence=source_seq_; t.source_received=source_received_;
    t.age=command_issued_ && now>=command_issued_ ? now-command_issued_ : 0;
    if (stopped_) { t.status=STOPPED; return t; }
    if (!clock(now) || scheduled>now) fail(BACKWARD_CLOCK);
    // READY has no command to execute. A late idle heartbeat cannot miss a
    // control output deadline or poison the later live handoff. The first
    // accepted command starts ACTIVE permanently (command_seq_ never resets);
    // all active deadlines, leases and latched faults remain unchanged.
    if (command_seq_ && now>=scheduled && now-scheduled>profile_.output_period_ns()) fail(MISSED_DEADLINE);
    if (command_seq_ && !fault_) {
      if (!source_issued_ || now<source_issued_ || now-source_issued_>profile_.source_wall_age_max_ns()) fail(STALE_SOURCE);
      else if (t.age>profile_.command_lease_ns()) fail(STALE_COMMAND);
      // Both clocks bind. A simulator faster than wall time cannot silently
      // exceed the physical hold covered by pending/stopping constraints.
      // Comparing absolute control times retains the exact inclusive boundary.
      else if(profile_.multirate() && source_control_s_>command_control_s_+profile_.command_hold_s()) fail(STALE_COMMAND);
    }
    if (fault_) { t.status=FAULT; t.reason=fault_; return t; }
    if (!command_seq_) return t;
    t.valid=true; t.repeated=(last_sent_==command_seq_);
    t.status=t.repeated?REPEATED:NEW; return t;
  }
  std::uint64_t wait_deadline(std::uint64_t scheduled) const {
    return std::min({scheduled+profile_.output_period_ns(), command_issued_+profile_.command_lease_ns(),
                     source_issued_+profile_.source_wall_age_max_ns()});
  }
  Tick tick(std::uint64_t scheduled, std::uint64_t now) {
    auto t=preview(scheduled,now); t.sequence=++ticks_;
    if(t.valid) last_sent_=command_seq_;
    return t;
  }
 private:
  TimingProfile profile_;
  bool clock(std::uint64_t now) {
    if (now<last_now_) return false;
    last_now_=now; return true;
  }
  void fail(Fault f) { if (!fault_ && !stopped_) fault_=f; }
  std::uint64_t last_now_{}, ticks_{}, source_seq_{}, source_received_{}, source_issued_{};
  std::uint64_t command_seq_{}, command_issued_{}, last_sent_{};
  double source_control_s_{},command_control_s_{};
  bool stopped_{}; Fault fault_{NONE};
};
}
