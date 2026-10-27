#pragma once
#include "case_builder.hpp"
#include <array>
#include <memory>
#include <vector>

namespace x650_standard {
struct NativeSolveResult {
  int raw_status=-1;
  bool solver_success=false;
  // Numerical evidence only. There is intentionally no publication approval.
  std::vector<double> plan, states, rollout_states;
  std::array<double,4> residuals{}; // stationarity, equality, inequality, complementarity
  double setup_ms=0., solve_ms=0., postprocess_ms=0.;
  double local_dynamics_residual=0., rollout_disagreement=0.;
};

// One generated, fixed-configuration SQP or SQP-RTI instance. Not thread-safe:
// one worker owns it. This class has no ROS I/O and never updates executed history.
// The generated solver and the builder configuration must share the qualification
// manifest; the executable adapter binds that identity before constructing it.
class AcadosBackend final {
 public:
  explicit AcadosBackend(const x650::Config& matching_generated_config);
  ~AcadosBackend();
  AcadosBackend(const AcadosBackend&)=delete;
  AcadosBackend& operator=(const AcadosBackend&)=delete;
  NativeSolveResult solve(const NumericalCase&);
  std::vector<double> rollout(const NumericalCase&,const std::vector<double>& plan);
 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
} // namespace x650_standard
