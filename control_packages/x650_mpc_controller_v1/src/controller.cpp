#include "x650_mpc/controller.hpp"
#include <limits>
#include <stdexcept>
#include <utility>
extern "C" unsigned x650_optimization_abi_version();

namespace x650_mpc {
namespace {
x650::Config checked_config(x650::Config config, x650::Ts20BatchFn predictor) {
  // Check the stable C version functions before constructing a C++ Core whose
  // Config/Memory layout would be incompatible with a different native ABI.
  if (x650_mpc_core_abi_version() != 4 || x650_optimization_abi_version() != 2)
    throw std::invalid_argument("Required MPC core ABI 4 / optimization ABI 2");
  if (!predictor) throw std::invalid_argument("TS20 predictor function is required");
  return config;
}
}
Controller::Controller(x650::Config config, x650::Ts20BatchFn predictor,
                       x650::DiagnosticMode diagnostics)
    : core_(checked_config(std::move(config), predictor), predictor, diagnostics) {}

void Controller::reset(const x650::Anchor& anchor,
                       const std::vector<x650::Command5>& actual_history,
                       double absolute_origin) {
  auto trial = core_;
  trial.reset(anchor);
  trial.prime(actual_history, absolute_origin);
  core_ = std::move(trial);
  pending_.reset();
}

const Candidate& Controller::prepare(const x650::StepInput& input) {
  if (pending_) throw std::logic_error("Resolve pending candidate before prepare");
  if (last_token_ == std::numeric_limits<Token>::max())
    throw std::overflow_error("Candidate token space exhausted; create a new session");
  // Native Core commits only to this private trial. The live Memory remains
  // unchanged even if the solver, candidate checks, or allocation throws.
  auto trial = core_;
  auto output = trial.step(input);
  Pending next{{last_token_ + 1, std::move(output)}, trial.memory()};
  pending_ = std::move(next);
  ++last_token_;
  return pending_->candidate;
}

void Controller::require_pending(Token token) const {
  if (!pending_ || token == 0 || pending_->candidate.token != token)
    throw std::logic_error("Unknown, stale, discarded, or already committed token");
}

void Controller::commit_published(Token token, const x650::V5& exact_command) {
  require_pending(token);
  if (exact_command != pending_->candidate.output.command)
    throw std::invalid_argument("Published command differs from prepared candidate");
  // Validate and copy before changing the authoritative state. No send-time
  // substitution is made for the native model's original proposal time.
  auto trial = core_;
  trial.restore(pending_->next);
  core_ = std::move(trial);
  pending_.reset();
}

void Controller::discard(Token token) {
  require_pending(token);
  pending_.reset();
}

void Controller::restore_for_replay(const x650::Memory& memory) {
  auto trial = core_;
  trial.restore(memory);
  core_ = std::move(trial);
  pending_.reset();
}
}  // namespace x650_mpc
