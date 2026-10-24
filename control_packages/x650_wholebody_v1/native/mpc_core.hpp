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
  double node_dt=.04, max_dt=.05, feasibility_tolerance=2e-7, kkt_tolerance=2e-3;
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
struct Diagnostics {
  int subproblems=0,active_constraints=0,solver_mode=-1,qp_rows=0,qp_columns=0;
  double primal_violation=0.,kkt_stationarity=0.,kkt_complementarity=0.,objective=0.;
  double q_linearization_error=0.,dq_linearization_error=0.,compute_ms=0.;
  bool accepted=false,nonlinear_passed=false,actual_bounds=false,previous_plan_fallback=false;
  std::array<double,8> candidate_envelope{};
  int candidate_substeps=0;
  std::vector<Attempt> attempts;
  std::vector<double> horizon,plan,predicted_states,predicted_tip,predicted_rotations;
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
