#pragma once
#include "timing_profile.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <optional>
#include <stdexcept>

namespace flyarm_control_gateway {

using Hash = std::array<std::uint8_t, 32>;

struct Identity {
  std::uint16_t schema_version{1};
  std::uint64_t session_id{}, epoch{}, sequence{};
  bool replay_only{true};
  Hash configuration_sha256{};
};

struct Snapshot : Identity {
  std::uint64_t issued_monotonic_ns{};
  double control_time_s{}, task_time_s{}, dt_s{}, state_time_s{}, joint_time_s{};
  std::array<double, 3> position_enu{}, velocity_enu{}, omega_body_flu{}, acceleration_enu{};
  std::array<double, 9> rotation_row_major{};
  std::array<double, 2> q{}, dq{};
  std::array<double, 3> target_position_enu{}, target_velocity_enu{}, target_acceleration_enu{}, position_gains{};
  double reference_time_s{}, bus_origin_s{};
  std::array<double, 2> reference_r{};
  std::uint32_t command_count{}, packet_count{};
  std::array<double, 64> command_time_s{}, packet_send_time_s{};
  std::array<double, 128> command_velocity{}, packet_q_ref{}, packet_dq_ref{}, packet_tau_ff{}, packet_kp{}, packet_kd{};
};

struct Proposal : Identity {
  std::uint64_t snapshot_sequence{}, compute_start_monotonic_ns{}, compute_end_monotonic_ns{};
  std::uint8_t status{};
  double control_time_s{}, task_time_s{};
  std::array<double, 3> position_reference_enu{}, velocity_feedforward_enu{}, acceleration_feedforward_enu{}, base_velocity_target{};
  std::array<double, 2> joint_velocity{};
  bool accepted_optimum{}, nonlinear_check_passed{};
  double primal_violation{}, kkt_residual{};
  std::uint32_t subproblems{};
};

enum class Reason : std::uint16_t {
  ACCEPTED = 0, IDENTITY = 1, REPLAY_REQUIRED = 2, SEQUENCE = 3,
  INPUT = 4, FRAME = 5, SOURCE_AGE = 6, CLOCK_REGRESSION = 7,
  DEADLINE = 8, TIMEOUT = 9, PROPOSAL = 10, OPTIMIZATION = 11,
  LIMIT = 12, RECONSTRUCTION = 13, INFLIGHT = 14, CONTEXT = 15
};

struct Event {
  std::uint16_t schema_version{1};
  std::uint64_t session_id{}, epoch{}, sequence{}, snapshot_sequence{}, proposal_sequence{};
  bool replay_only{true}, accepted{};
  Reason reason{Reason::INPUT};
  std::uint64_t received_monotonic_ns{}, roundtrip_ns{};
};

struct Config {
  TimingProfile timing_profile{};
  std::uint64_t session_id{}, epoch{};
  Hash configuration_sha256{};
  // Must be supplied from the frozen PID readback, not inferred from a request.
  std::array<double, 3> position_gains{};
  bool replay_only{true};
};

struct Admission {
  bool forward{};
  std::optional<Event> event;
};

class ShadowGate {
 public:
  static constexpr std::uint64_t DEADLINE_NS = 50'000'000;
  static constexpr double BASE_SOURCE_AGE_S = .25;
  static constexpr double JOINT_SOURCE_AGE_S = .025;
  static constexpr double MAX_DT_S = .05;
  static constexpr std::array<double, 3> BASE_LIMIT{.55, .35, .45};
  static constexpr std::array<double, 2> JOINT_LIMIT{.45, .35};
  static constexpr std::array<double, 2> Q_MIN{-.75, -.35};
  static constexpr std::array<double, 2> Q_MAX{.45, .85};

  explicit ShadowGate(Config config) : config_(config) {
    if (config.session_id == 0 || config.epoch == 0 ||
        std::all_of(config.configuration_sha256.begin(), config.configuration_sha256.end(),
                    [](std::uint8_t value) { return value == 0; }) ||
        !finite(config.position_gains) ||
        std::any_of(config.position_gains.begin(), config.position_gains.end(),
                    [](double value) { return value <= 0.; })) {
      throw std::invalid_argument("Explicit positive session/epoch, configuration SHA and PID gains required");
    }
  }

  Admission admit(const Snapshot & snapshot, std::uint64_t now) {
    if (latched_) return {};
    const auto reject = [&](Reason reason) { return Admission{false, fail(reason, snapshot.sequence, 0, now)}; };
    if (!observe_now(now)) return reject(Reason::CLOCK_REGRESSION);
    if (pending_) return reject(Reason::INFLIGHT);
    if (snapshot.replay_only != config_.replay_only) return reject(Reason::REPLAY_REQUIRED);
    if (!identity(snapshot)) return reject(Reason::IDENTITY);
    if (snapshot.sequence != next_sequence_ || snapshot.sequence == std::numeric_limits<std::uint64_t>::max())
      return reject(Reason::SEQUENCE);
    if (!finite_snapshot(snapshot)) return reject(Reason::INPUT);
    if (snapshot.issued_monotonic_ns == 0 || snapshot.issued_monotonic_ns > now)
      return reject(Reason::CLOCK_REGRESSION);
    if (now-snapshot.issued_monotonic_ns > config_.timing_profile.solve_roundtrip_max_ns()) return reject(Reason::DEADLINE);
    if (last_control_time_ && snapshot.control_time_s <= *last_control_time_)
      return reject(Reason::CLOCK_REGRESSION);
    if (snapshot.dt_s <= 0. || snapshot.dt_s > config_.timing_profile.control_dt_max_s()+config_.timing_profile.control_dt_roundoff_s() ||
        (last_control_time_ &&
         (snapshot.control_time_s > *last_control_time_+config_.timing_profile.control_dt_max_s()+config_.timing_profile.control_dt_roundoff_s() ||
          !near(snapshot.dt_s, snapshot.control_time_s-*last_control_time_))))
      return reject(Reason::DEADLINE);
    if (!source_time(snapshot.state_time_s, snapshot.control_time_s, BASE_SOURCE_AGE_S) ||
        !source_time(snapshot.joint_time_s, snapshot.control_time_s, JOINT_SOURCE_AGE_S))
      return reject(Reason::SOURCE_AGE);
    if (!proper_rotation(snapshot.rotation_row_major)) return reject(Reason::FRAME);
    for (std::size_t axis = 0; axis < 3; ++axis) {
      if (!near(snapshot.position_gains[axis], config_.position_gains[axis]) ||
          (config_.replay_only && snapshot.target_acceleration_enu[axis] != 0.)) return reject(Reason::INPUT);
    }
    for (std::size_t joint = 0; joint < 2; ++joint)
      if (snapshot.q[joint] < Q_MIN[joint] || snapshot.q[joint] > Q_MAX[joint]) return reject(Reason::LIMIT);
    if (!context(snapshot)) return reject(Reason::CONTEXT);
    // A single fixed copy freezes all reference/history inputs for this round.
    cached_ = snapshot;
    receipt_ns_ = now;
    pending_ = true;
    return {true, std::nullopt};
  }

  std::optional<Event> propose(const Proposal & proposal, std::uint64_t now) {
    if (latched_) return std::nullopt;
    const auto reject = [&](Reason reason) {
      return fail(reason, pending_ ? cached_.sequence : proposal.snapshot_sequence, proposal.sequence, now);
    };
    if (!observe_now(now)) return reject(Reason::CLOCK_REGRESSION);
    if (!pending_) return reject(Reason::SEQUENCE);
    if (proposal.replay_only != config_.replay_only) return reject(Reason::REPLAY_REQUIRED);
    if (!identity(proposal)) return reject(Reason::IDENTITY);
    if (proposal.sequence != next_sequence_ || proposal.snapshot_sequence != cached_.sequence)
      return reject(Reason::SEQUENCE);
    // Measured locally: a fabricated worker compute time cannot extend a round.
    if (now < receipt_ns_ || now < cached_.issued_monotonic_ns)
      return reject(Reason::CLOCK_REGRESSION);
    if (now-cached_.issued_monotonic_ns > config_.timing_profile.solve_roundtrip_max_ns() || now-receipt_ns_ > config_.timing_profile.solve_roundtrip_max_ns())
      return reject(Reason::DEADLINE);
    if (!finite_proposal(proposal) || !near(proposal.control_time_s, cached_.control_time_s) ||
        !near(proposal.task_time_s, cached_.task_time_s)) return reject(Reason::PROPOSAL);
    if (proposal.compute_start_monotonic_ns < receipt_ns_ ||
        proposal.compute_end_monotonic_ns < proposal.compute_start_monotonic_ns ||
        proposal.compute_end_monotonic_ns > now ||
        proposal.compute_end_monotonic_ns-proposal.compute_start_monotonic_ns > config_.timing_profile.solve_roundtrip_max_ns())
      return reject(Reason::DEADLINE);
    if (proposal.status != 0 || !proposal.accepted_optimum || !proposal.nonlinear_check_passed ||
        proposal.primal_violation < 0. || proposal.primal_violation > 2e-7 ||
        proposal.kkt_residual < 0. || proposal.kkt_residual > .002 ||
        proposal.subproblems < 1 || proposal.subproblems > 3) return reject(Reason::OPTIMIZATION);
    for (std::size_t axis = 0; axis < 3; ++axis) {
      if (std::abs(proposal.base_velocity_target[axis]) > BASE_LIMIT[axis] ||
          proposal.acceleration_feedforward_enu[axis] != 0.) return reject(Reason::LIMIT);
      const double reconstructed = cached_.position_gains[axis] *
        (proposal.position_reference_enu[axis]-cached_.position_enu[axis]) + proposal.velocity_feedforward_enu[axis];
      if (!std::isfinite(reconstructed) || !near(reconstructed, proposal.base_velocity_target[axis]))
        return reject(Reason::RECONSTRUCTION);
    }
    for (std::size_t joint = 0; joint < 2; ++joint)
      if (std::abs(proposal.joint_velocity[joint]) > JOINT_LIMIT[joint]) return reject(Reason::LIMIT);
    auto event = make_event(true, Reason::ACCEPTED, cached_.sequence, proposal.sequence, now);
    last_control_time_ = cached_.control_time_s;
    pending_ = false;
    ++next_sequence_;
    return event;
  }

  std::optional<Event> poll(std::uint64_t now) {
    if (latched_) return std::nullopt;
    if (!observe_now(now)) return fail(Reason::CLOCK_REGRESSION, pending_ ? cached_.sequence : 0, 0, now);
    if (pending_ && (now-cached_.issued_monotonic_ns > config_.timing_profile.solve_roundtrip_max_ns() || now-receipt_ns_ > config_.timing_profile.solve_roundtrip_max_ns()))
      return fail(Reason::TIMEOUT, cached_.sequence, 0, now);
    return std::nullopt;
  }

  bool latched() const { return latched_; }
  bool pending() const { return pending_; }
  std::uint64_t next_sequence() const { return next_sequence_; }

 private:
  template<std::size_t Size> static bool finite(const std::array<double, Size> & values) {
    return std::all_of(values.begin(), values.end(), [](double value) { return std::isfinite(value); });
  }
  static bool near(double a, double b) { return std::abs(a-b) <= 1e-8; }
  static bool source_time(double source, double control, double maximum_age) {
    return source <= control && source >= control-maximum_age;
  }
  bool identity(const Identity & value) const {
    return value.schema_version == 1 && value.session_id == config_.session_id &&
      value.epoch == config_.epoch && value.configuration_sha256 == config_.configuration_sha256;
  }
  static bool finite_snapshot(const Snapshot & value) {
    return std::isfinite(value.control_time_s) && std::isfinite(value.task_time_s) &&
      std::isfinite(value.dt_s) && std::isfinite(value.state_time_s) && std::isfinite(value.joint_time_s) &&
      std::isfinite(value.reference_time_s) && std::isfinite(value.bus_origin_s) &&
      finite(value.position_enu) && finite(value.velocity_enu) && finite(value.omega_body_flu) &&
      finite(value.acceleration_enu) && finite(value.rotation_row_major) && finite(value.q) && finite(value.dq) &&
      finite(value.target_position_enu) && finite(value.target_velocity_enu) &&
      finite(value.target_acceleration_enu) && finite(value.position_gains) && finite(value.reference_r) &&
      finite(value.command_time_s) && finite(value.packet_send_time_s) && finite(value.command_velocity) &&
      finite(value.packet_q_ref) && finite(value.packet_dq_ref) && finite(value.packet_tau_ff) &&
      finite(value.packet_kp) && finite(value.packet_kd);
  }
  static bool finite_proposal(const Proposal & value) {
    return std::isfinite(value.control_time_s) && std::isfinite(value.task_time_s) &&
      std::isfinite(value.primal_violation) && std::isfinite(value.kkt_residual) &&
      finite(value.position_reference_enu) && finite(value.velocity_feedforward_enu) &&
      finite(value.acceleration_feedforward_enu) && finite(value.base_velocity_target) && finite(value.joint_velocity);
  }
  static bool proper_rotation(const std::array<double, 9> & r) {
    for (std::size_t i = 0; i < 3; ++i) for (std::size_t j = 0; j < 3; ++j) {
      double dot = 0.;
      for (std::size_t k = 0; k < 3; ++k) dot += r[k*3+i]*r[k*3+j];
      if (std::abs(dot-(i == j ? 1. : 0.)) > 1e-6) return false;
    }
    const double determinant = r[0]*(r[4]*r[8]-r[5]*r[7]) - r[1]*(r[3]*r[8]-r[5]*r[6]) + r[2]*(r[3]*r[7]-r[4]*r[6]);
    return std::abs(determinant-1.) <= 1e-6;
  }
  template<std::size_t Size> static bool padding_zero(const std::array<double, Size> & values, std::size_t used) {
    return std::all_of(values.begin()+used, values.end(), [](double value) { return value == 0.; });
  }
  static bool context(const Snapshot & value) {
    // Match the existing Ts20Runtime/ReferenceEngine task-relative history
    // comparison. Independently formed simulation timestamps can straddle
    // the same bus instant by one binary64 ULP after origin subtraction.
    // This does not alter source ages, dt, wall leases or original stamps.
    constexpr double HISTORY_TIME_EPS_S = 1e-10;
    if (value.command_count < 1 || value.command_count > 64 || value.packet_count < 1 || value.packet_count > 64 ||
        !near(value.reference_time_s, value.joint_time_s-(value.control_time_s-value.task_time_s))) return false;
    if (!padding_zero(value.command_time_s, value.command_count) ||
        !padding_zero(value.command_velocity, 2*value.command_count) ||
        !padding_zero(value.packet_send_time_s, value.packet_count) ||
        !padding_zero(value.packet_q_ref, 2*value.packet_count) ||
        !padding_zero(value.packet_dq_ref, 2*value.packet_count) ||
        !padding_zero(value.packet_tau_ff, 2*value.packet_count) ||
        !padding_zero(value.packet_kp, 2*value.packet_count) ||
        !padding_zero(value.packet_kd, 2*value.packet_count)) return false;
    for (std::size_t i = 0; i < value.command_count; ++i) {
      if (value.command_time_s[i] > value.task_time_s+HISTORY_TIME_EPS_S ||
          (i && value.command_time_s[i] < value.command_time_s[i-1])) return false;
      for (std::size_t joint = 0; joint < 2; ++joint)
        if (std::abs(value.command_velocity[2*i+joint]) > JOINT_LIMIT[joint]) return false;
    }
    for (std::size_t i = 0; i < value.packet_count; ++i) {
      if (value.packet_send_time_s[i] > value.task_time_s+HISTORY_TIME_EPS_S || value.packet_send_time_s[i] < value.bus_origin_s ||
          (i && value.packet_send_time_s[i] <= value.packet_send_time_s[i-1])) return false;
      for (std::size_t joint = 0; joint < 2; ++joint)
        if (value.packet_kp[2*i+joint] < 0. || value.packet_kd[2*i+joint] < 0.) return false;
    }
    return true;
  }
  bool observe_now(std::uint64_t now) {
    if (last_now_ && now < *last_now_) return false;
    last_now_ = now;
    return true;
  }
  Event make_event(bool accepted, Reason reason, std::uint64_t snapshot, std::uint64_t proposal, std::uint64_t now) {
    Event event;
    event.session_id = config_.session_id;
    event.replay_only = config_.replay_only;
    event.epoch = config_.epoch;
    event.sequence = ++event_sequence_;
    event.snapshot_sequence = snapshot;
    event.proposal_sequence = proposal;
    event.accepted = accepted;
    event.reason = reason;
    event.received_monotonic_ns = now;
    event.roundtrip_ns = pending_ && now >= cached_.issued_monotonic_ns ? now-cached_.issued_monotonic_ns : 0;
    return event;
  }
  Event fail(Reason reason, std::uint64_t snapshot, std::uint64_t proposal, std::uint64_t now) {
    auto event = make_event(false, reason, snapshot, proposal, now);
    latched_ = true;
    pending_ = false;
    return event;
  }

  const Config config_;
  Snapshot cached_{};
  std::uint64_t receipt_ns_{}, next_sequence_{1}, event_sequence_{};
  std::optional<std::uint64_t> last_now_;
  std::optional<double> last_control_time_;
  bool pending_{}, latched_{};
};

}  // namespace flyarm_control_gateway
