#pragma once
// Owning, ROS/Python-independent controller API. No caller-supplied matrices.
#include <array>
#include <cstddef>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace x650 {
enum class DiagnosticMode { Full, Compact };
using V2 = std::array<double,2>;
using V3 = std::array<double,3>;
using V5 = std::array<double,5>;
using R9 = std::array<double,9>; // row-major FLU to ENU
using Ts20BatchFn = int(*)(const double*,const double*,double,double,
    const double*,const double*,const double*,int,const double*,int,int,
    double*,int*,int*,double*,double*,int,int*,int*,char*,int);

struct Config {
  int nodes=15, max_iterations=100;
  double node_dt=.04, max_dt=.05, command_hold_s=.05, feasibility_tolerance=2e-7, kkt_tolerance=2e-3;
  double arm_share=.2, joint_position_gain=3.;
  std::array<double,6> weights{400.,4.,.25,.02,.01,.01};
  V5 velocity_limit{.55,.35,.45,.45,.35}, acceleration_limit{.45,.30,.45,.60,.60};
  V2 q_lower{-.75,-.35}, q_upper{.45,.85}, physical_lower{}, physical_upper{};
  // lengths2, angle_offsets2, mount_body3, elbow_offset_body3.
  std::array<double,10> geometry{};
  // x wn,zeta; y wn,zeta; z tau.
  std::array<double,5> response{};
  V3 base_position_gain{};
  // J,b,Fc,offset,combined_interface_delay,Fs,delta,vs.
  std::array<double,8> motion{};
  V2 kp{6.,6.}, kd{.25,.25};
  double bus_period=.005, integration_step=.001, trust_radius=.15;
};
struct Anchor { V3 position{}; R9 rotation{1,0,0,0,1,0,0,0,1}; V2 q{}; };
struct State {
  V3 position{},velocity{},omega_body{},acceleration{};
  R9 rotation{1,0,0,0,1,0,0,0,1}; V2 q{},dq{};
  double stamp=0.,control_time=0.,joint_stamp=0.;
};
struct Target { V3 position{},velocity{},acceleration{}; };
struct Command { double time=0.; V2 velocity{}; };
struct Command5 { double time=0.; V5 velocity{}; }; // absolute time for prime()
struct Packet { double send_time=0.; V2 q_ref{},dq_ref{},tau_ff{},kp{},kd{}; };
struct Context {
  double reference_time=0.,bus_origin=0.; V2 reference{};
  std::vector<Command> commands; std::vector<Packet> packets;
};
struct StepInput {
  double t=0.,dt=0.; State state; Target target; Context context;
  // Exactly Config.nodes C++ trajectory samples at t+cumsum([dt,node_dt,...]).
  // Empty selects the existing constant-acceleration current-target fallback.
  std::vector<Target> preview;
};
struct Memory {
  bool initialized=false, history_primed=false, has_last_t=false,has_stamp=false,has_origin=false;
  Anchor anchor; V5 previous_command{};
  double last_t=0.,previous_stamp=0.,origin=0.,plan_t=0.;
  std::uint64_t step_count=0;
  std::vector<double> plan,plan_h;
  std::vector<Command> joint_history;
};
struct Attempt {
  bool passed=false,actual_bounds=false; double q_error=0.,dq_error=0.,elapsed_ms=0.;
};
// ABI 4: a supplementary first-command zero-order-hold prediction. It starts
// from the original joint state/time and immutable actual packet history.
// It is not a fabricated measurement or a proof for unknown future send phase.
struct HoldGuard {
  bool required=false,checked=false,passed=false;
  double start_task_s=0.,end_task_s=0.,duration_s=0.,source_control_time_s=0.,source_joint_time_s=0.;
  V2 command{},reference_initial{},reference_final{};
  std::array<double,8> envelope{};
  int substeps=0,actual_history_packet_count=0;
  std::vector<Packet> packets;
  std::vector<double> query_times,boundary_times,boundary_states;
};
// ABI 2: a recovery centre is only a numerical seed. It is never published.
// Original-row feasibility and native continuous-envelope validation must both
// pass before it may replace an infeasible linearization centre.
struct RecoveryAttempt {
  std::string centre_kind,rejection;
  bool nonlinear_passed=false,original_constraints_passed=false,trusted=false,projection_attempted=false,projection_solved=false;
  double original_primal_violation=0.,stopping_violation=0.,elapsed_ms=0.;
  std::vector<double> centre_plan;
  // Exact native node states [q1,q2,dq1,dq2] and reference integration used by
  // the checks, copied from propagation without a second diagnostic rollout.
  std::vector<double> node_times,node_states,node_references;
  std::array<double,8> envelope{};
  int substeps=0;
  HoldGuard hold_guard;
};
// ABI 3: finite hard constraints generated from the authoritative continuous
// candidate envelope. component 0=q, 1=dq; joint is zero based. source_kind:
// 0 initial/canonical boundary, 1 accepted substep end, 2 velocity-zero event,
// 3 acceleration-zero (velocity extremum) event. No slack is introduced.
struct IntervalPoint {
  double time=0.,value=0.,lower=0.,upper=0.;
  int joint=0,component=0,source_kind=0;
};
struct IntervalConstraint {
  IntervalPoint point;
  double affine_offset=0.;
  std::vector<double> gradient;
  int qp_row=0;
  // Command-encoding remainder in the same sliding linear force response.
  // The actual state bounds above are unchanged; these bounds account for
  // a future quantizer cell differing from the centre's known encoding error.
  double encoding_lower_remainder=0.,encoding_upper_remainder=0.;
  double encoding_kernel_l1=0.,encoding_anchor_bias=0.,encoding_effort_half_lsb=0.;
};
struct IntervalRepair {
  int control_subproblem=0,added_points=0,total_points=0,canonical_substeps=0;
  double elapsed_ms=0.;
  std::string rejection;
  std::array<double,8> envelope{};
  std::vector<IntervalPoint> trigger_points,points;
  std::vector<double> centre_plan,risk_margin,risk_encoding_lower,risk_encoding_upper;
  double risk_radius=0.;
};
struct Diagnostics {
  int subproblems=0,active_constraints=0,solver_mode=-1,qp_rows=0,qp_columns=0;
  double primal_violation=0.,kkt_stationarity=0.,kkt_complementarity=0.,objective=0.;
  double q_linearization_error=0.,dq_linearization_error=0.,compute_ms=0.;
  bool accepted=false,nonlinear_passed=false,actual_bounds=false,previous_plan_fallback=false;
  std::array<double,8> candidate_envelope{};
  int candidate_substeps=0;
  HoldGuard hold_guard;
  std::vector<Attempt> attempts;
  bool recovery_triggered=false;
  std::string recovery_reason;
  std::vector<RecoveryAttempt> recovery_attempts;
  double recovery_trust_radius=0.,recovery_stopping_violation=0.;
  int interval_rows=0;
  std::vector<IntervalRepair> interval_repairs;
  std::vector<IntervalConstraint> interval_constraints;
  // Full captures of the exact canonical centre supporting the last hard rows.
  // Extra observation times never enter native propagation boundaries.
  std::vector<double> interval_centre_plan,interval_canonical_boundaries,interval_canonical_states;
  std::array<double,8> interval_canonical_envelope{};
  int interval_canonical_substeps=0;
  std::vector<double> horizon,plan,predicted_states,predicted_tip,predicted_rotations;
  // Full diagnostics retain the surrogate before native q/dq replaces it.
  std::vector<double> linearized_states;
  // Complete actual propagation boundary evidence, never recomputed for logging.
  std::vector<Packet> nominal_packets,candidate_packets;
  std::vector<double> query_times,nominal_states,candidate_states,candidate_extrema,candidate_boundary_times;
  // Modes are flattened [boundary][joint]; zero-event counts and boundary
  // times have exactly one element per actual native propagation boundary.
  std::vector<int> candidate_modes,candidate_zero_events;
  // Exact assembled numerical values for independent offline equivalence.
  std::vector<double> A,b,H,g,C,lower,upper,solution;
};
struct StepResult {
  V5 command{};
  V3 position_reference{},velocity_feedforward{},acceleration_feedforward{};
  V2 joint_velocity{},joint_reference{},joint_velocity_reference{},joint_acceleration_reference{};
  V3 nominal_acceleration{},tip_actual{};
  Diagnostics diagnostics;
};
class CoreError : public std::runtime_error {
 public: Diagnostics diagnostics;
  CoreError(const std::string& what,Diagnostics value={}) :std::runtime_error(what),diagnostics(std::move(value)) {}
};
// Read-only numerical check for testing/replay of the explicit extended lease.
// No controller Memory, observations, actual history or actuator is modified.
HoldGuard evaluate_first_command_hold(const Config&,Ts20BatchFn,const StepInput&,
    const std::vector<double>& physical_plan,const V2& reference_at_task_time);
class Core {
 public:
  Core(Config config,Ts20BatchFn predictor,DiagnosticMode diagnostics=DiagnosticMode::Full);
  void reset(const Anchor& anchor);
  void prime(const std::vector<Command5>& actual_history,double absolute_origin);
  StepResult step(const StepInput& input);
  const Memory& memory() const noexcept {return memory_;}
  const Config& config() const noexcept {return config_;}
  // Explicit offline-replay state restoration; never implicitly called by step.
  void restore(const Memory& memory);
 private:
  Config config_; Ts20BatchFn predictor_; Memory memory_; DiagnosticMode diagnostic_mode_;
};
} // namespace x650
extern "C" unsigned x650_mpc_core_abi_version();
