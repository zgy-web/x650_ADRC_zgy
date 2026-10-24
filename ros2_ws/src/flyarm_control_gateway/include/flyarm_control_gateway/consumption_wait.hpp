#pragma once
#include "periodic_gate.hpp"

namespace flyarm_control_gateway {
// No allocation or clock reads: the actual adapter supplies immutable facts
// for this inspection. Only a pending consumption receipt is waitable.
struct ConsumptionReadiness {
  static constexpr std::uint64_t STATE_AGE_NS=25'000'000, OUTPUT_GAP_NS=40'000'000;
  PeriodicGate::Fault fault{PeriodicGate::NONE};
  bool pending{};
  std::uint64_t deadline{};
};
inline ConsumptionReadiness consumption_readiness(
    std::uint64_t now, bool have_state, std::uint64_t observed,
    std::uint64_t last_normal_send, bool pending) {
  using G=PeriodicGate; using R=ConsumptionReadiness;
  if(!have_state || !observed || observed>now || now-observed>R::STATE_AGE_NS)
    return {G::STALE_ACTUATOR_STATE,false,0};
  if(last_normal_send && (last_normal_send>now || now-last_normal_send>R::OUTPUT_GAP_NS))
    return {G::ACTUAL_OUTPUT_GAP,false,0};
  auto deadline=observed+R::STATE_AGE_NS;
  if(last_normal_send) deadline=std::min(deadline,last_normal_send+R::OUTPUT_GAP_NS);
  return {G::NONE,pending,deadline};
}
}
