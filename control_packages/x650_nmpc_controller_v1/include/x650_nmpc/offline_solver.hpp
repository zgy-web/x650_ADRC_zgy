#pragma once
#include "x650_nmpc/objective.hpp"
#include <string>
#include <vector>

namespace x650_nmpc {
// Research-only outer nonlinear iteration. The original ABI4 controller supplies
// feasible local QP proposals; no actuator or committed controller is owned.
struct Options {
  int max_proposals=4;
  double absolute_decrease=1e-10, relative_decrease=1e-7;
  double evaluation_budget_ms=50.;
};
struct Iteration {
  int index=0;
  bool native_passed=false, stopping_passed=false, accepted=false;
  double native_compute_ms=0., elapsed_ms=0., stopping_violation=0.;
  double normalized_step_inf=0.;
  Objective objective;
  std::string reason;
};
struct OfflineResult {
  x650::StepResult output;
  x650::Memory proposed_memory;
  Objective first_objective, selected_objective;
  std::vector<Iteration> iterations;
  int selected_iteration=-1;
  double elapsed_ms=0.;
  bool within_evaluation_budget=false;
  // Always false: neither feasible research output nor a short wall duration
  // substitutes for a fresh-state dispatch/lease and full integration audit.
  bool approved_for_online_publication=false;
  std::string stop_reason;
};
struct OfflineError : std::runtime_error {
  OfflineResult diagnostics;
  OfflineError(const std::string& message,OfflineResult result)
      :std::runtime_error(message),diagnostics(std::move(result)) {}
};

double actual_stopping_violation(const x650::Config&, const x650::StepInput&,
                                const x650::StepResult&);
// Does not change original_memory or input, even on failure or time expiry.
// The first feasible native proposal establishes J0. Later proposals are
// accepted only on strict decrease of the SAME nonlinear objective. A failed
// or non-improving proposal stops the bounded loop. No NLP convergence claim.
OfflineResult solve_offline(const x650::Config&, x650::Ts20BatchFn,
                            const x650::Memory& original_memory,
                            const x650::StepInput&, const Options& = {});
} // namespace x650_nmpc
