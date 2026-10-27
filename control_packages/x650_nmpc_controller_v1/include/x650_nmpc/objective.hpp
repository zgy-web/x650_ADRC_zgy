#pragma once
#include "mpc_core.hpp"
#include <array>
#include <vector>

namespace x650_nmpc {
// Original physical nonlinear least-squares cost, not the final local QP cost.
// Component order: endpoint position, endpoint velocity, nominal base position,
// nominal joint position, velocity-normalized input, acceleration-normalized slew.
struct Objective {
  std::array<double,6> components{};
  double total{};
  std::vector<double> endpoint_positions;  // row-major [node][ENU xyz]
  std::vector<double> endpoint_velocities; // row-major [node][ENU xyz]
};

// Read-only scoring of an already checked native candidate. Full diagnostics
// are required. The original committed memory supplies the first slew reference
// and nominal anchor; a proposed/committed candidate memory must not replace it.
// This performs no optimization, propagation, state commit or actuator I/O.
// "Exact" means the original nonlinear endpoint/output cost on the candidate's
// native TS20 node states, not an exact physical robot or smooth NLP derivative.
Objective evaluate_objective(const x650::Config&,
                             const x650::Memory& original_committed_memory,
                             const x650::StepInput&,
                             const x650::StepResult& already_native_validated_candidate);
} // namespace x650_nmpc
