#pragma once
#include "periodic_gate.hpp"
#include <array>
#include <cmath>
#include <limits>

namespace flyarm_control_gateway {
// Simulation transport bookkeeping, not an actuator position/settling test.
// Anonymous Float64 commands require one exclusive ordered exactly-once writer
// per joint. Counter/payload conflicts fail; skipped observations never invent
// an acknowledgement for a command that was not individually observed.
class ConsumptionLedger {
 public:
  static constexpr std::size_t CAPACITY=16;
  static constexpr std::uint64_t PROGRESS_BUDGET_NS=40'000'000;
  enum JointStatus : std::uint8_t { UNCONFIRMED=0, OBSERVED_ADOPTED=1, SUPERSEDED_OR_UNOBSERVED=2 };
  struct Entry {
    std::uint64_t command{},dispatch{},sent{};
    double sim{};std::array<std::uint64_t,2> counter{};
    std::array<double,2> velocity{};
    std::array<JointStatus,2> status{{UNCONFIRMED,UNCONFIRMED}};
    std::array<double,2> consumed_sim{},observation_sim{};
    std::array<std::uint64_t,2> observed{},received{};
    bool stop{},paired{};
  };
  struct Observation {
    std::array<std::uint64_t,2> counter{};
    std::array<double,2> velocity{},consumed_sim{};
    double sim{};std::uint64_t observed{},received{};
  };
  void seed(const Observation &o) { last_=o;expected_=o.counter;seeded_=true; }
  bool empty() const {return size_==0;}
  std::size_t size() const {return size_;}
  const Entry &at(std::size_t i) const {return entries_[(head_+i)%CAPACITY];}
  bool room() const {return size_<CAPACITY;}
  bool append(const Entry &e) {
    if(!seeded_ || !room())return false;
    for(int j=0;j<2;++j)
      if(expected_[j]==std::numeric_limits<std::uint64_t>::max() || e.counter[j]!=expected_[j]+1 || !std::isfinite(e.velocity[j]))return false;
    entries_[(head_+size_)%CAPACITY]=e;++size_;expected_=e.counter;return true;
  }
  bool progress_expired(std::uint64_t now) const {
    // Budget is attached to each original send. Later sends/observations of
    // unchanged counters never renew an unresolved joint's deadline.
    for(std::size_t i=0;i<size_;++i) {
      const auto &e=at(i);
      if(e.status[0]==UNCONFIRMED || e.status[1]==UNCONFIRMED)
        if(now<e.sent || now-e.sent>PROGRESS_BUDGET_NS)return true;
    }
    return false;
  }
  template<class Retired> bool observe(const Observation &o,Retired retired) {
    if(!seeded_ || !std::isfinite(o.sim) || o.sim<last_.sim || o.observed<last_.observed)return false;
    for(int j=0;j<2;++j) {
      if(!std::isfinite(o.velocity[j]) || !std::isfinite(o.consumed_sim[j]) ||
         o.consumed_sim[j]>o.sim+1e-9 || o.consumed_sim[j]<last_.consumed_sim[j] ||
         o.counter[j]<last_.counter[j] || o.counter[j]>expected_[j])return false;
      if(o.counter[j]==last_.counter[j] &&
         (o.velocity[j]!=last_.velocity[j] || o.consumed_sim[j]!=last_.consumed_sim[j]))return false;
      if(o.counter[j]>last_.counter[j]) {
        bool found=false;
        for(std::size_t i=0;i<size_;++i) {
          const auto &e=at(i);
          if(e.counter[j]==o.counter[j]) {
            found=true;
            if(std::abs(e.velocity[j]-o.velocity[j])>1e-12 || o.consumed_sim[j]<e.sim-.025)return false;
          }
        }
        if(!found)return false;
      }
    }
    // Validate the whole observation before changing either joint's history.
    for(std::size_t i=0;i<size_;++i) {
      auto &e=entries_[(head_+i)%CAPACITY];
      for(int j=0;j<2;++j) if(e.status[j]==UNCONFIRMED && o.counter[j]>=e.counter[j]) {
        e.status[j]=o.counter[j]==e.counter[j]?OBSERVED_ADOPTED:SUPERSEDED_OR_UNOBSERVED;
        e.consumed_sim[j]=o.consumed_sim[j];e.observation_sim[j]=o.sim;
        e.observed[j]=o.observed;e.received[j]=o.received;
      }
      if(o.counter==e.counter && e.status[0]==OBSERVED_ADOPTED && e.status[1]==OBSERVED_ADOPTED)e.paired=true;
    }
    last_=o;
    while(size_) {
      const auto &e=entries_[head_];
      if(e.status[0]==UNCONFIRMED || e.status[1]==UNCONFIRMED)break;
      retired(e);head_=(head_+1)%CAPACITY;--size_;
    }
    return true;
  }
 private:
  std::array<Entry,CAPACITY> entries_{};std::size_t head_{},size_{};
  Observation last_{};std::array<std::uint64_t,2> expected_{};bool seeded_{};
};
}
