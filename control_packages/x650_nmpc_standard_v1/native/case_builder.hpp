#pragma once
// Pure numerical mapping of the original committed state/history. No ROS,
// optimizer, file I/O, state commit or actuator publication is performed here.
#include "mpc_core.hpp"
#include <array>
#include <vector>

namespace x650_standard {
inline constexpr int state_dim=23,input_dim=5,parameter_dim=161,horizon_nodes=15;
inline constexpr int event_slots=22;
namespace parameter {
inline constexpr int h=0,attitude_context=1,target_position=21,target_velocity=24,
 nominal_position=27,nominal_q=30,nominal_control=32,durations=37,apply_new=59,
 apply_pending=81,send_relative=103,sample_mask=125,pending_at_end=147,
 pending_send_relative=148,stopping_reserve=149,control_lower=151,control_upper=156;
}
struct NumericalCase {
  std::array<double,state_dim> x0{};
  std::vector<double> parameters,initial_plan,horizon;
};
// Matches model.py numerical model v2: epsilon=.05, future RK4 10 substeps
// per bus interval, history alignment 20. Every input is retained read-only.
// Unsupported/inconsistent history/configuration throws std::invalid_argument.
NumericalCase build_case(const x650::Config&,const x650::Memory&,const x650::StepInput&);
}
