#include "flyarm_control_gateway/admission_cadence.hpp"
#include "flyarm_control_gateway/periodic_gate.hpp"
#include <iostream>
#include <limits>
#include <stdexcept>
using flyarm_control_gateway::AdmissionCadence;
using G=flyarm_control_gateway::PeriodicGate;
void check(bool ok) {if(!ok)throw std::runtime_error("admission cadence check failed");}
void one_output_one_solve() {
  constexpr std::uint64_t B=1'000'000'000;
  AdmissionCadence c;
  check(!c.acknowledge_output(0));check(!c.acknowledge_output(1));
  check(!c.due(true,B,B,42.616,42.600));
  check(c.due(true,B,B+1'000'000,42.620,42.600));
  c.admitted(1);
  // Accepted-but-unsent candidates cannot advance controller state.
  check(!c.due(true,B+100'000'000,B,42.660,42.620));
  check(!c.acknowledge_output(2));
  check(c.acknowledge_output(1));check(!c.acknowledge_output(1));
  check(!c.due(true,B,B,42.620,42.620));
  check(!c.due(true,B,B,42.616,42.620));
  check(!c.due(true,B,B,std::numeric_limits<double>::quiet_NaN(),42.620));
  check(!c.due(true,B,B,42.636,std::numeric_limits<double>::infinity()));
  check(c.due(true,B,B+100'000'000,42.636,42.620));
  c.admitted(2);
  check(!c.due(true,B,B,42.660,42.636));check(!c.acknowledge_output(1));
  check(c.acknowledge_output(2));check(c.due(true,B,B,42.660,42.636));
  c.admitted(3);check(c.acknowledge_output(3));
  check(c.due(true,B,B,42.664,42.660)); // actual recorder's4ms quantum
  check(!c.due(false,B-1,B,42.664,42.660));
  check(c.due(false,B,B,42.664,42.660));
}

void recorded_extra_frame_opportunity() {
  // hold20 send41's output tick confirms source207 was already received:
  // previous42.468, source42.480(12ms). Its20ms successor was not issued until
  // 7.643259ms after send41. Do not invent an unrecorded ACK callback time.
  constexpr std::uint64_t sent=59598695598939956ULL;
  constexpr std::uint64_t received=59598695596396841ULL;
  check(received<sent);
  AdmissionCadence c;c.admitted(41);
  check(!c.due(true,sent,sent,42.480,42.468));
  check(c.acknowledge_output(41));
  check(c.due(true,sent,sent+20'000'000,42.480,42.468));
  check(42.480-42.468<.020-1e-10); // former independent guard rejects
  c.admitted(42);check(!c.acknowledge_output(41));
  check(!c.due(true,sent+8'000'000,sent,42.488,42.480));
}

void recorded_late_worker_still_faults() {
  // hold20 final211->212 already had eligible dt40ms. A cadence change must
  // not rescue a result completing after the next original leased slot.
  constexpr std::uint64_t issued=59598699174524446ULL;
  constexpr std::uint64_t sent=59598699219018377ULL;
  constexpr std::uint64_t source=59598699215124564ULL;
  constexpr std::uint64_t compute_begin=59598699220009258ULL;
  constexpr std::uint64_t compute_end=59598699241285661ULL;
  const auto next_slot=sent+G::PERIOD_NS;
  check(compute_begin>sent && compute_end>next_slot);
  AdmissionCadence c;c.admitted(211);check(c.acknowledge_output(211));
  check(c.due(true,sent,sent,46.076,46.036));
  G gate;gate.source(1099,sent,source,sent);check(gate.accept(211,issued,sent));
  check(gate.tick(sent,sent).valid);
  gate.source(1104,next_slot,next_slot-1'000'000,next_slot);
  const auto fault=gate.tick(next_slot,next_slot);
  check(fault.status==G::FAULT && fault.reason==G::STALE_COMMAND && !fault.valid);
  check(!gate.accept(212,source,compute_end));
}

int main() {
  one_output_one_solve();recorded_extra_frame_opportunity();recorded_late_worker_still_faults();
  std::cout<<"PASS: first20ms, actual-output one-use grant, held/backward source, true16/24/4ms, unchanged shadow cadence, recorded opportunity, preserved hold20 terminal fault\n";
}
