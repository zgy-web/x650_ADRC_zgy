#pragma once
#include <cstdint>
#include <stdexcept>

namespace flyarm_control_gateway {
// Construction/discovery and historical CSV catch-up produce no live state.
// A real LiveStart starts the original 25 ms guard. Its first EOF barrier
// discards every already-buffered row; only a subsequent new complete row may
// be published. No startup/idle backlog receives a fresh live timestamp.
class ConsumptionLifecycle {
 public:
  static constexpr std::uint64_t MAX_GAP_NS=25'000'000;
  enum Phase { STARTING, READY, RESYNC, ACTIVE };
  Phase phase() const {return phase_;}
  void ready() {
    if(phase_!=STARTING)throw std::runtime_error("Consumption readiness repeated");
    phase_=READY;
  }
  void start(std::uint64_t issued,std::uint64_t now) {
    if(phase_!=READY || !issued || issued>now || now-issued>MAX_GAP_NS)
      throw std::runtime_error("Consumption LiveStart identity/time phase invalid");
    phase_=RESYNC;last_poll_=issued;
  }
  void begin_poll(std::uint64_t now) {
    if(phase_>=RESYNC) {
      if(now<last_poll_ || now-last_poll_>MAX_GAP_NS)
        throw std::runtime_error("Consumption reader polling gap exceeds 25 ms");
      last_poll_=now;
    }
  }
  bool finish_poll(std::uint64_t now,bool caught_up,bool complete_row) {
    if(phase_<RESYNC)return false;
    if(now<last_poll_ || now-last_poll_>MAX_GAP_NS)
      throw std::runtime_error("Consumption active parsing exceeds 25 ms");
    if(phase_==RESYNC) {
      if(caught_up)phase_=ACTIVE;
      return false;
    }
    return caught_up && complete_row;
  }
 private:
  Phase phase_{STARTING};
  std::uint64_t last_poll_{};
};
}
