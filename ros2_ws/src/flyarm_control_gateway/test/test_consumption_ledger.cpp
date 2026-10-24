#include "flyarm_control_gateway/consumption_ledger.hpp"
#include <iostream>
#include <stdexcept>
#include <vector>
using L=flyarm_control_gateway::ConsumptionLedger;
static void check(bool value) {if(!value)throw std::runtime_error("consumption ledger contract");}
static L::Observation baseline() {return {{100,200},{0,0},{1.,1.},1.,1'000'000'000,1'000'000'001};}
static L::Entry entry(std::uint64_t n) {
  L::Entry e;e.command=n;e.dispatch=n+10;e.sent=1'000'000'000+(n-1)*20'000'000;e.sim=1.+(n-1)*.02;
  e.counter={100+n,200+n};e.velocity={.01*n,-.01*n};return e;
}
static L::Observation observation(const L::Entry &e,std::uint64_t elapsed=23'000'000) {
  return {e.counter,e.velocity,{e.sim,e.sim},e.sim+.03,e.sent+elapsed,e.sent+elapsed+1};
}
int main() {
  std::vector<L::Entry> retired;auto keep=[&](const auto&e){retired.push_back(e);};
  L normal;normal.seed(baseline());auto a=entry(1),b=entry(2);
  check(normal.append(a));check(normal.append(b)); // no receipt barrier
  check(normal.observe(observation(a),keep));check(retired.size()==1&&retired[0].paired);
  check(normal.observe(observation(b),keep));check(retired.size()==2&&normal.empty());
  L exact;exact.seed(baseline());check(exact.append(a));check(!exact.progress_expired(a.sent+L::PROGRESS_BUDGET_NS));
  check(exact.progress_expired(a.sent+L::PROGRESS_BUDGET_NS+1));
  check(exact.append(b));check(exact.progress_expired(a.sent+L::PROGRESS_BUDGET_NS+1)); // new send never renews
  L jump;jump.seed(baseline());check(jump.append(a)&&jump.append(b));retired.clear();
  check(jump.observe(observation(b),keep));check(retired.size()==2);
  check(!retired[0].paired&&retired[0].status[0]==L::SUPERSEDED_OR_UNOBSERVED&&retired[1].paired);
  L mixed;mixed.seed(baseline());check(mixed.append(a)&&mixed.append(b));retired.clear();
  auto m=observation(b);m.counter[1]=a.counter[1];m.velocity[1]=a.velocity[1];m.consumed_sim[1]=a.sim;
  check(mixed.observe(m,keep));check(retired.size()==1&&!retired[0].paired);
  auto both=observation(b,30'000'000);check(mixed.observe(both,keep));check(retired.size()==2&&retired[1].paired);
  check(retired[1].observed[0]!=retired[1].observed[1]);
  auto duplicate=both;check(mixed.observe(duplicate,keep));check(retired.size()==2);
  duplicate.velocity[0]+=.1;check(!mixed.observe(duplicate,keep));
  L invalid;invalid.seed(baseline());check(invalid.append(a));retired.clear();
  auto bad=observation(a);bad.counter[0]++;check(!invalid.observe(bad,keep));check(retired.empty());
  bad=observation(a);bad.velocity[0]+=.1;check(!invalid.observe(bad,keep));
  check(invalid.at(0).status[0]==L::UNCONFIRMED&&invalid.at(0).status[1]==L::UNCONFIRMED);
  bad=observation(a);bad.consumed_sim[0]=bad.sim+1.;check(!invalid.observe(bad,keep));
  check(invalid.observe(observation(a),keep));auto regressed=baseline();regressed.observed+=50'000'000;regressed.sim+=.05;
  check(!invalid.observe(regressed,keep));
  L stop;stop.seed(baseline());check(stop.append(a));b.stop=true;b.velocity={0,0};check(stop.append(b));retired.clear();
  check(stop.observe(observation(b),keep));check(retired.size()==2&&!retired[0].paired&&retired[1].stop&&retired[1].paired);
  L capacity;capacity.seed(baseline());for(std::size_t i=1;i<=L::CAPACITY;++i)check(capacity.append(entry(i)));
  check(!capacity.append(entry(L::CAPACITY+1)));check(capacity.size()==L::CAPACITY);
  std::cout<<"PASS: two sends without ACK, 40 ms original-send budget, counter jump, per-joint mixed batch, duplicate, payload/future/regression rejection, zero skips unobserved, bounded capacity\n";
}
