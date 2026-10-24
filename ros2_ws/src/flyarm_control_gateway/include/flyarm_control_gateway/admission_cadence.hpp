#pragma once
#include <cmath>
#include <cstdint>
namespace flyarm_control_gateway {
// First admission waits for 20 ms of actual model time. Thereafter only an
// actual output acknowledgement releases one new solve, using a newer actual
// state and its true dt. A second physical 20 ms minimum would independently
// quantize the wall-clock output schedule and can cost another sensor frame.
// This class neither invents a state/history nor changes any age/deadline gate.
class AdmissionCadence {
 public:
  bool due(bool live,std::uint64_t now,std::uint64_t next_wall,
           double control,double previous_control) const {
    if(!live) return now>=next_wall;
    if(!std::isfinite(control) || !std::isfinite(previous_control) || control<=previous_control) return false;
    if(!last_admitted_) return control-previous_control>=.020-1e-10;
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
  std::uint64_t last_admitted_{},released_{};
};
}
