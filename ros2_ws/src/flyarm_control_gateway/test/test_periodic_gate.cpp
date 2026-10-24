#include "flyarm_control_gateway/periodic_gate.hpp"
#include <iostream>
#include <stdexcept>
using G=flyarm_control_gateway::PeriodicGate;
void check(bool ok) { if (!ok) throw std::runtime_error("periodic gate check failed"); }

void idle_to_active_contract() {
  constexpr std::uint64_t B=1'000'000'000, PAUSE=120'000'000;
  // Slow DDS/ground preparation can delay idle heartbeat ticks. No command
  // exists, so these cannot be counted as missed actuator output slots.
  G idle;
  auto t=idle.tick(B,B);
  check(t.status==G::WAITING && !t.valid && t.sequence==1);
  t=idle.tick(B+G::PERIOD_NS,B+PAUSE);
  check(t.status==G::WAITING && !t.valid && t.reason==G::NONE &&
        t.sequence==2 && t.command_sequence==0 && !idle.faulted());
  const auto start=B+PAUSE+1;
  idle.source(1,start,start,start);
  check(idle.accept(1,start,start));
  // The live dispatcher anchors the first active slot at first acceptance.
  t=idle.tick(start,start);
  check(t.status==G::NEW && t.valid && t.command_sequence==1 && t.age==0);

  // A subsequent fresh command must not erase a missed active output slot,
  // even if source age and candidate lease would individually permit it.
  const auto late=start+PAUSE;
  idle.source(2,late,late,late);
  check(idle.accept(2,late,late));
  t=idle.tick(start+G::PERIOD_NS,late);
  check(t.status==G::FAULT && !t.valid && t.reason==G::MISSED_DEADLINE);
  check(!idle.accept(3,late+1,late+1));
  t=idle.tick(late+1,late+1);
  check(t.status==G::FAULT && !t.valid && t.reason==G::MISSED_DEADLINE);
  idle.stop();
  check(idle.tick(late+2,late+2).status==G::STOPPED);
  check(!idle.accept(3,late+3,late+3));

  // READY does not relax candidate age or clock validity, and it does not
  // clear explicit faults. STOP is still terminal before any acceptance.
  G stale;
  check(stale.tick(B,B).status==G::WAITING);
  check(stale.tick(B+G::PERIOD_NS,B+PAUSE).status==G::WAITING);
  check(!stale.accept(1,B,B+PAUSE));
  check(stale.tick(B+PAUSE,B+PAUSE).reason==G::STALE_COMMAND);
  G backwards;
  check(backwards.tick(B,B).status==G::WAITING);
  check(backwards.tick(B-1,B-1).reason==G::BACKWARD_CLOCK);
  check(!backwards.accept(1,B+1,B+1));
  G future;
  check(future.tick(B+1,B).reason==G::BACKWARD_CLOCK);
  G rejected; rejected.reject();
  check(rejected.tick(B,B+PAUSE).status==G::FAULT);
  check(!rejected.accept(1,B+PAUSE,B+PAUSE));
  G stopped; stopped.stop();
  check(stopped.tick(B,B+PAUSE).status==G::STOPPED);
  check(!stopped.accept(1,B+PAUSE,B+PAUSE));

  // Exact active thresholds retain their original inclusive boundary. A
  // single nanosecond beyond 20 ms lateness, 25 ms source age or 50 ms lease
  // must still fault; idle handling cannot silently widen any of them.
  G boundary;
  boundary.source(1,B,B,B); check(boundary.accept(1,B,B));
  check(boundary.tick(B,B+G::PERIOD_NS).valid);
  check(boundary.tick(B,B+G::PERIOD_NS+1).reason==G::MISSED_DEADLINE);
  G source;
  source.source(1,B,B,B); check(source.accept(1,B,B));
  check(source.tick(B+G::RX_AGE_NS,B+G::RX_AGE_NS).valid);
  check(source.tick(B+G::RX_AGE_NS+1,B+G::RX_AGE_NS+1).reason==G::STALE_SOURCE);
  G lease;
  lease.source(1,B,B,B); check(lease.accept(1,B,B));
  lease.source(2,B+G::LEASE_NS,B+G::LEASE_NS,B+G::LEASE_NS);
  check(lease.tick(B+G::LEASE_NS,B+G::LEASE_NS).valid);
  check(lease.tick(B+G::LEASE_NS+1,B+G::LEASE_NS+1).reason==G::STALE_COMMAND);
}

int main() {
  idle_to_active_contract();
  constexpr std::uint64_t B=1'000'000'000;
  G g; g.source(1,B,B,B); check(g.accept(1,B,B));
  auto t=g.tick(B,B); check(t.valid && !t.repeated);
  g.source(2,B+20'000'000,B+20'000'000,B+20'000'000);
  t=g.tick(B+20'000'000,B+20'000'000); check(t.valid && t.repeated && t.age==20'000'000);
  g.source(3,B+40'000'000,B+40'000'000,B+40'000'000);
  t=g.tick(B+40'000'000,B+40'000'000); check(t.valid && t.repeated && t.age==40'000'000);
  g.source(4,B+60'000'000,B+60'000'000,B+60'000'000);
  t=g.tick(B+60'000'000,B+60'000'000); check(!t.valid && t.reason==G::STALE_COMMAND);
  check(!g.accept(2,B+60'000'000,B+60'000'000));
  G s; s.source(1,B,B,B); check(s.accept(1,B,B));
  check(s.tick(B+40'000'000,B+40'000'000).reason==G::STALE_SOURCE);
  G d; d.source(1,B,B,B); check(d.accept(1,B,B)); check(!d.accept(1,B,B));
  G n; n.source(1,B,B,B); check(n.accept(1,B,B)); n.stop(); check(n.tick(B,B).status==G::STOPPED);
  check(!n.accept(2,B,B));
  G back; back.source(1,B,B,B); back.source(2,B-1,B-1,B-1); check(back.faulted());
  G late; late.source(1,B,B,B); check(!late.accept(1,B,B+G::LEASE_NS+1));
  G missing; missing.source(1,B,B,B); check(missing.accept(1,B,B));
  check(missing.tick(B,B+G::PERIOD_NS+1).reason==G::MISSED_DEADLINE);
  G rejected; rejected.reject(); check(rejected.tick(B,B).status==G::FAULT);
  std::cout<<"PASS: idle-to-active lifecycle, unchanged active boundaries and fault latch, lease, repeats, source loss, identity order, clock regression, late candidate, missed tick, stop latch\n";
}
