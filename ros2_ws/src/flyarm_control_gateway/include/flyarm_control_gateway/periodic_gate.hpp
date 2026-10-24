#pragma once
#include "shadow_gate.hpp"

namespace flyarm_control_gateway {
// No ROS, locks, dynamic allocation, disk I/O or Python on the decision path.
// This is an output lease, NOT a fallback controller. A repeat retains its
// original snapshot issue time; receiving/repeating it never renews the lease.
class PeriodicGate {
 public:
  static constexpr std::uint64_t PERIOD_NS=20'000'000, LEASE_NS=50'000'000;
  static constexpr std::uint64_t RX_AGE_NS=25'000'000;
  enum Status : std::uint8_t { WAITING=0, NEW=1, REPEATED=2, FAULT=3, STOPPED=4 };
  enum Fault : std::uint16_t { NONE=0, STALE_COMMAND=1, STALE_SOURCE=2,
    BACKWARD_CLOCK=3, GATE_REJECTED=4, BAD_COMMAND=5, MISSED_DEADLINE=6 };
  struct Tick { Status status{WAITING}; Fault reason{NONE}; bool valid{}, repeated{};
    std::uint64_t sequence{}, scheduled{}, actual{}, command_sequence{}, issued{}, age{}, source_sequence{}, source_received{}; };

  void source(std::uint64_t seq, std::uint64_t received, std::uint64_t issued, std::uint64_t now) {
    if (stopped_ || fault_) return;
    if (!clock(now) || received>now || issued>received || !issued || seq<=source_seq_ || received<source_received_) {
      fail(BACKWARD_CLOCK); return;
    }
    source_seq_=seq; source_received_=received; source_issued_=issued;
  }
  bool accept(std::uint64_t seq, std::uint64_t issued, std::uint64_t now) {
    if (stopped_ || fault_) return false;
    if (!clock(now) || !issued || issued>now || issued<command_issued_ || seq<=command_seq_) {
      fail(BAD_COMMAND); return false;
    }
    if (now-issued>LEASE_NS) { fail(STALE_COMMAND); return false; }
    command_seq_=seq; command_issued_=issued; return true;
  }
  void reject() { fail(GATE_REJECTED); }
  void stop() { stopped_=true; }
  bool faulted() const { return fault_!=NONE; }
  Tick tick(std::uint64_t scheduled, std::uint64_t now) {
    Tick t; t.sequence=++ticks_; t.scheduled=scheduled; t.actual=now;
    t.command_sequence=command_seq_; t.issued=command_issued_;
    t.source_sequence=source_seq_; t.source_received=source_received_;
    t.age=command_issued_ && now>=command_issued_ ? now-command_issued_ : 0;
    if (stopped_) { t.status=STOPPED; return t; }
    if (!clock(now) || scheduled>now) fail(BACKWARD_CLOCK);
    // READY has no command to execute. A late idle heartbeat cannot miss a
    // control output deadline or poison the later live handoff. The first
    // accepted command starts ACTIVE permanently (command_seq_ never resets);
    // all active deadlines, leases and latched faults remain unchanged.
    if (command_seq_ && now>=scheduled && now-scheduled>PERIOD_NS) fail(MISSED_DEADLINE);
    if (command_seq_ && !fault_) {
      if (!source_issued_ || now<source_issued_ || now-source_issued_>RX_AGE_NS) fail(STALE_SOURCE);
      else if (t.age>LEASE_NS) fail(STALE_COMMAND);
    }
    if (fault_) { t.status=FAULT; t.reason=fault_; return t; }
    if (!command_seq_) return t;
    t.valid=true; t.repeated=(last_sent_==command_seq_);
    t.status=t.repeated?REPEATED:NEW; last_sent_=command_seq_; return t;
  }
 private:
  bool clock(std::uint64_t now) {
    if (now<last_now_) return false;
    last_now_=now; return true;
  }
  void fail(Fault f) { if (!fault_ && !stopped_) fault_=f; }
  std::uint64_t last_now_{}, ticks_{}, source_seq_{}, source_received_{}, source_issued_{};
  std::uint64_t command_seq_{}, command_issued_{}, last_sent_{};
  bool stopped_{}; Fault fault_{NONE};
};
}
