#include "synthetic_fixture.hpp"
#include <iostream>
int main() {
  try {
    x650_mpc::Controller controller(fixture::config(), ts20_prediction_batch);
    controller.reset({}, {{10.,{0,0,0,0,0}}}, 10.);
    const auto& candidate=controller.prepare(fixture::input());
    std::cout<<"Synthetic API call; no simulation and no actuator publication.\ncommand:";
    for(double value:candidate.output.command)std::cout<<' '<<value;
    std::cout<<"\ncommitted steps before discard: "<<controller.committed_memory().step_count<<'\n';
    // A runnable API example must not assert that an actuator received this.
    controller.discard(candidate.token);
    std::cout<<"committed steps after discard: "<<controller.committed_memory().step_count<<'\n';
    return 0;
  } catch(const std::exception& error) { std::cerr<<error.what()<<'\n';return 1; }
}
