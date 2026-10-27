#pragma once
#include "x650_nmpc/offline_solver.hpp"
#include <cstdint>

namespace x650_nmpc {
struct OnlineOptions {
  // Soft allowance for new numerical proposals, not an actuator validity lease.
  Options nonlinear{4,1e-10,1e-7,40.};
  // Hard request-issue to checked-result deadline. This version permits at most
  // 50 ms. Zero is a deliberate reject-all budget useful for supervision/tests.
  double total_budget_ms=50.;
};
struct OnlineResult {
  bool accepted=false;
  std::string status;
  OfflineResult solve;
  double call_elapsed_ms=0.,issue_age_ms=0.,actual_stopping_violation=0.;
  std::uint64_t completed_monotonic_ns=0;
};

// Numerical admission wrapper for the existing mpc25_dispatch50 execution
// contract. issue_monotonic_ns must be the ORIGINAL request's CLOCK_MONOTONIC /
// std::chrono::steady_clock time, never a fresh timestamp invented by a worker.
// Returns structured rejection and never commits the caller's Memory. On
// accepted=true, solve.output and solve.proposed_memory form one selected pair.
// Caller still checks session/state/source/issue freshness immediately before
// publication and applies the unchanged dispatcher/actuator lease checks.
// The nominal 40 ms request cadence and 20 ms dispatch slots belong to the ROS
// execution layer. Native proposals are not preemptible; a late completion is
// rejected, so this API does not claim a hard scheduling or WCET guarantee.
// solve.approved_for_online_publication intentionally remains false: this new
// wrapper provides its own explicit admission, not an offline flag toggle.
OnlineResult solve_online(const x650::Config&,x650::Ts20BatchFn,
                          const x650::Memory& original_memory,
                          const x650::StepInput&,std::uint64_t issue_monotonic_ns,
                          const OnlineOptions& = {});
} // namespace x650_nmpc
