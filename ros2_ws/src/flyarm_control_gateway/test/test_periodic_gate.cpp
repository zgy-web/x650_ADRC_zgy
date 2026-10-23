#include "flyarm_control_gateway/periodic_gate.hpp"
#include <iostream>
#include <stdexcept>
using G=flyarm_control_gateway::PeriodicGate;
void check(bool ok) { if (!ok) throw std::runtime_error("periodic gate check failed"); }
int main() {
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
  std::cout<<"PASS: lease, repeats, source loss, identity order, clock regression, late candidate, missed tick, stop latch\n";
}
