#pragma once

// The same state-envelope margins govern QP prediction nodes and the final
// continuous native candidate check. They are not command/reference limits,
// trust-region radii, stopping-distance reserves, or solver tolerances.
// Keep the Python reference CANDIDATE_POLICY numerically equivalent; the
// independent native/reference boundary tests check this contract.
namespace x650::joint_safety {
inline constexpr double q_padding = .0003;
inline constexpr double dq_padding = .03;
inline constexpr double bound_tolerance = 2e-7;
inline constexpr double q_error_max = .006;
inline constexpr double dq_error_max = .10;
}
