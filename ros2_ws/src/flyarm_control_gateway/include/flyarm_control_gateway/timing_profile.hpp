#pragma once
#include <cstdint>
#include <ostream>
#include <stdexcept>
#include <string>

namespace flyarm_control_gateway {
// Named experimental budgets, not a hard-real-time guarantee. Construction by
// name prevents independently widened fields at the two admission boundaries.
class TimingProfile {
 public:
  constexpr TimingProfile() = default;
  static TimingProfile from_name(const std::string& name) {
    if(name=="legacy_v1") return {};
    if(name=="mpc25_dispatch50_v1") return TimingProfile(true);
    throw std::invalid_argument("Unknown explicit control timing profile");
  }
  constexpr const char* name() const {return multirate_?"mpc25_dispatch50_v1":"legacy_v1";}
  constexpr bool multirate() const {return multirate_;}
  constexpr std::uint64_t nominal_solve_period_ns() const {return multirate_?40'000'000:20'000'000;}
  constexpr std::uint64_t output_period_ns() const {return 20'000'000;}
  constexpr std::uint64_t command_lease_ns() const {return multirate_?160'000'000:50'000'000;}
  constexpr double command_hold_s() const {return multirate_?.16:.05;}
  constexpr double control_dt_max_s() const {return multirate_?.1:.05;}
  // A subtraction of two actual binary64 clocks can round above 100 ms.
  // This is less than one nanosecond; actual dt is never rounded or clamped.
  constexpr double control_dt_roundoff_s() const {return multirate_?1e-10:0.;}
  constexpr std::uint64_t solve_roundtrip_max_ns() const {return 50'000'000;}
  constexpr std::uint64_t source_wall_age_max_ns() const {return 25'000'000;}
  constexpr std::uint64_t consumption_progress_max_ns() const {return 40'000'000;}
 private:
  explicit constexpr TimingProfile(bool multirate):multirate_(multirate){}
  bool multirate_{};
};

inline void write_timing_profile(std::ostream& out,const TimingProfile& p) {
  out<<"{\"name\":\""<<p.name()<<"\",\"nominal_solve_period_ns\":"<<p.nominal_solve_period_ns()
     <<",\"output_period_ns\":"<<p.output_period_ns()<<",\"command_lease_ns\":"<<p.command_lease_ns()
     <<",\"command_hold_s\":"<<p.command_hold_s()
     <<",\"control_dt_max_s\":"<<p.control_dt_max_s()<<",\"solve_roundtrip_max_ns\":"<<p.solve_roundtrip_max_ns()
     <<",\"source_wall_age_max_ns\":"<<p.source_wall_age_max_ns()
     <<",\"consumption_progress_max_ns\":"<<p.consumption_progress_max_ns()
     <<",\"hard_realtime_qualified\":false}";
}
}
