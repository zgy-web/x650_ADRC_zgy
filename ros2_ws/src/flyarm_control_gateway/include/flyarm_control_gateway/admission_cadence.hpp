#pragma once
#include "timing_profile.hpp"
#include <cmath>
#include <cstdint>
namespace flyarm_control_gateway {
// First admission waits for 20 ms of actual model time. Thereafter only an
// actual output acknowledgement releases one new solve, using a newer actual
// state and its true dt. A second physical 20 ms minimum would independently
// quantize the wall-clock output schedule and can cost another sensor frame.
// The explicitly named multirate profile additionally honors next_wall and
// starts after 40 ms. Its caller reanchors at actual admission, never catches
// up missed solves. This class never invents state/history or source stamps.
class AdmissionCadence {
 public:
  explicit AdmissionCadence(TimingProfile profile={}):profile_(profile){}
  bool due(bool live,std::uint64_t now,std::uint64_t next_wall,
           double control,double previous_control) const {
    if(!live) return now>=next_wall;
    if(!std::isfinite(control) || !std::isfinite(previous_control) || control<=previous_control) return false;
    if(profile_.multirate() && now<next_wall) return false;
    if(!last_admitted_) return control-previous_control>=double(profile_.nominal_solve_period_ns())/1e9-1e-10;
    return released_==last_admitted_;
  }
  void admitted(std::uint64_t sequence) { last_admitted_=sequence; released_=0; }
  bool acknowledge_output(std::uint64_t sequence) {
    if(!sequence || sequence!=last_admitted_ || released_==sequence) return false;
    released_=sequence; return true;
  }
  std::uint64_t last_admitted() const { return last_admitted_; }
  std::uint64_t released() const { return released_; }
 private:
  TimingProfile profile_;
  std::uint64_t last_admitted_{},released_{};
};
}
