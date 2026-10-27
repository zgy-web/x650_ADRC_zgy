#include "x650_mpc/common_gate.hpp"
#include <chrono>
#include <limits>
#include <new>
#include <stdexcept>
#include <utility>

namespace x650_mpc {
std::uint64_t CommonGateController::monotonic_ns() {
  return static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count());
}
CommonGateController::CommonGateController(x650::Config config, x650::Ts20BatchFn predictor,
                                           x650::DiagnosticMode diagnostics)
    : controller_(std::move(config), predictor, diagnostics), predictor_(predictor) {}

void CommonGateController::reset(const x650::Anchor& anchor,
    const std::vector<x650::Command5>& history, double origin) {
  controller_.reset(anchor, history, origin);
  admitted_.reset();
}
void CommonGateController::restore_for_replay(const x650::Memory& memory) {
  controller_.restore_for_replay(memory);
  admitted_.reset();
}

CommonGateResult CommonGateController::prepare(const x650::StepInput& input,
    std::uint64_t issue, std::uint64_t budget) {
  CommonGateResult result;
  auto& timing = result.timing;
  timing.issue_monotonic_ns = issue;
  timing.budget_ns = budget;
  timing.call_started_ns = monotonic_ns();
  Token created = 0;
  const auto finish = [&] {
    timing.completed_ns = monotonic_ns();
    timing.call_ms = double(timing.completed_ns-timing.call_started_ns)/1e6;
    timing.issue_age_ms = timing.completed_ns >= issue ? double(timing.completed_ns-issue)/1e6 : -1.;
  };
  const auto discard_created = [&] {
    if (created) { controller_.discard(created); created = 0; admitted_.reset(); }
  };
  const auto reject = [&](const char* status) {
    discard_created();
    result.accepted = false;
    result.token = 0;
    result.status = status;
    finish();
  };
  const auto expired = [&] {
    const auto now = monotonic_ns();
    return now < issue || now > timing.deadline_monotonic_ns;
  };
  try {
    if (controller_.has_pending()) { reject("unresolved_pending_candidate"); return result; }
    if (!budget || budget > maximum_budget_ns) { reject("invalid_budget"); return result; }
    if (!issue || issue > timing.call_started_ns ||
        issue > std::numeric_limits<std::uint64_t>::max()-budget) {
      reject("invalid_issue_time"); return result;
    }
    timing.deadline_monotonic_ns = issue+budget;
    if (expired()) { reject("issue_expired_before_solve"); return result; }

    result.solver_called = true;
    const auto& prepared = controller_.prepare(input);
    created = prepared.token;  // Capture before any owning diagnostic copy.
    timing.core_completed_ns = monotonic_ns();
    timing.core_ms = double(timing.core_completed_ns-timing.call_started_ns)/1e6;
    if (expired()) { reject("issue_expired_after_solve"); return result; }

    timing.validation_started_ns = monotonic_ns();
    result.validation_called = true;
    result.validation = x650::evaluate_physical_plan(config(), predictor_, committed_memory(),
                                                    input, prepared.output.diagnostics.plan);
    timing.validation_completed_ns = monotonic_ns();
    timing.validation_ms = double(timing.validation_completed_ns-timing.validation_started_ns)/1e6;
    result.output = prepared.output;
    if (expired()) { reject("issue_expired_after_validation"); return result; }
    if (!result.validation.passed) {
      result.detail = result.validation.reason;
      reject("common_plan_rejected"); return result;
    }
    result.status = "ready_common_checked_candidate";
    result.token = created;
    result.accepted = true;
    admitted_ = created;
    finish();
    // Cover status/result construction as well as native work. There is still
    // no authority to publish after this deadline; transport rechecks on use.
    if (timing.completed_ns > timing.deadline_monotonic_ns) {
      reject("issue_expired_after_validation"); return result;
    }
    return result;
  } catch (const std::bad_alloc&) {
    discard_created();
    throw;
  } catch (const std::exception& error) {
    // Discard FIRST: even allocating the diagnostic error string can fail.
    discard_created();
    result.accepted = false; result.token = 0;
    result.detail = error.what();
    result.status = result.validation_called ? "validation_exception" : "legacy_solver_rejected";
    finish();
    return result;
  } catch (...) {
    discard_created();
    throw;
  }
}

void CommonGateController::commit_published(Token token, const x650::V5& command) {
  if (!admitted_ || *admitted_ != token)
    throw std::logic_error("Unknown, rejected, stale or already committed common-gate token");
  controller_.commit_published(token, command);
  admitted_.reset();
}
void CommonGateController::discard(Token token) {
  if (!admitted_ || *admitted_ != token)
    throw std::logic_error("Unknown, rejected, stale or already committed common-gate token");
  controller_.discard(token);
  admitted_.reset();
}
}  // namespace x650_mpc
