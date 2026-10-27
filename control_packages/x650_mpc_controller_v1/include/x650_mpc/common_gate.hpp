#pragma once
// New admission policy over the unchanged legacy Controller. No transport I/O.
#include "x650_mpc/controller.hpp"
#include <optional>
#include <string>

namespace x650_mpc {
struct CommonGateTiming {
  std::uint64_t issue_monotonic_ns{}, budget_ns{}, deadline_monotonic_ns{};
  std::uint64_t call_started_ns{}, core_completed_ns{}, validation_started_ns{};
  std::uint64_t validation_completed_ns{}, completed_ns{};
  double core_ms{}, validation_ms{}, call_ms{}, issue_age_ms{};
};

struct CommonGateResult {
  bool accepted{}, solver_called{}, validation_called{};
  std::string status, detail;
  Token token{};  // Nonzero only in an accepted result; later stale use is rejected.
  // Rejected candidates may retain diagnostics for audit, never for publication.
  std::optional<x650::StepResult> output;
  x650::PlanValidation validation;
  CommonGateTiming timing;
};

class CommonGateController final {
 public:
  static constexpr std::uint64_t maximum_budget_ns = 50'000'000;
  CommonGateController(x650::Config config, x650::Ts20BatchFn predictor,
                       x650::DiagnosticMode diagnostics = x650::DiagnosticMode::Full);
  CommonGateController(const CommonGateController&) = delete;
  CommonGateController& operator=(const CommonGateController&) = delete;

  // Same host CLOCK_MONOTONIC on Linux. Issue timestamps MUST use this clock.
  static std::uint64_t monotonic_ns();
  void reset(const x650::Anchor&, const std::vector<x650::Command5>& actual_history,
             double absolute_origin);
  void restore_for_replay(const x650::Memory& authentic_saved_memory);

  // Deadline includes old Core, common event validation, and result copy.
  // Ordinary native/validation errors produce a rejected result. Allocation or
  // unexpected exceptions propagate AFTER discarding this call's new proposal.
  // A pre-existing unresolved candidate is retained and this call is rejected.
  CommonGateResult prepare(const x650::StepInput&, std::uint64_t issue_monotonic_ns,
                           std::uint64_t budget_ns = maximum_budget_ns);

  // Caller must check publication deadline immediately before its own I/O.
  // Commit records an already completed matching real publication; notification
  // latency must not erase such actual history. No consumer-ACK wait is implied.
  void commit_published(Token, const x650::V5& exact_command);
  void discard(Token);
  bool has_pending() const noexcept { return controller_.has_pending(); }
  const x650::Memory& committed_memory() const noexcept { return controller_.committed_memory(); }
  const x650::Config& config() const noexcept { return controller_.config(); }

 private:
  Controller controller_;
  x650::Ts20BatchFn predictor_;
  std::optional<Token> admitted_;
};
}  // namespace x650_mpc
