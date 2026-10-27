#pragma once
// Transport-independent facade over the existing ABI 4 controller.
#include "mpc_core.hpp"
#include <cstdint>
#include <optional>

namespace x650_mpc {
using Token = std::uint64_t;
struct Candidate {
  Token token{};
  x650::StepResult output;
};

// One instance per vehicle/session. Calls must be serialized by the owner.
// This class never publishes, waits for transport, or invents actuator history.
class Controller final {
 public:
  Controller(x650::Config config, x650::Ts20BatchFn predictor,
             x650::DiagnosticMode diagnostics = x650::DiagnosticMode::Compact);
  Controller(const Controller&) = delete;
  Controller& operator=(const Controller&) = delete;
  Controller(Controller&&) = delete;
  Controller& operator=(Controller&&) = delete;

  // Actual pre-task publications use absolute simulation/device time.
  // A successful reset invalidates outstanding candidates, retaining no plan.
  void reset(const x650::Anchor& anchor,
             const std::vector<x650::Command5>& actual_history,
             double absolute_origin);

  // The returned reference remains valid until commit/discard/reset/restore.
  // A second prepare requires explicitly resolving the first candidate.
  const Candidate& prepare(const x650::StepInput& input);

  // Call only after the dispatcher has published BOTH the matching base and
  // joint outputs. This is not a consumer acknowledgement and does not wait.
  // exact_command is [base vx,vy,vz,joint dq1,dq2], before transport encoding.
  void commit_published(Token token, const x650::V5& exact_command);
  void discard(Token token);

  bool has_pending() const noexcept { return pending_.has_value(); }
  const x650::Memory& committed_memory() const noexcept { return core_.memory(); }
  const x650::Config& config() const noexcept { return core_.config(); }

  // Explicit offline replay only: caller supplies an authentic saved Memory.
  // Never synthesize an actuator history or use this for live reconnection.
  void restore_for_replay(const x650::Memory& memory);

 private:
  struct Pending { Candidate candidate; x650::Memory next; };
  x650::Core core_;
  std::optional<Pending> pending_;
  Token last_token_{};
  void require_pending(Token token) const;
};
}  // namespace x650_mpc
