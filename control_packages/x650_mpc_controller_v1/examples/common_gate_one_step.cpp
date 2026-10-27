#include "x650_mpc/common_gate.hpp"
#include "synthetic_fixture.hpp"
#include <iostream>
int main(){try{
 x650_mpc::CommonGateController controller(fixture::config(),ts20_prediction_batch);
 controller.reset({},{{10.,{0,0,0,0,0}}},10.);
 const auto result=controller.prepare(fixture::input(),controller.monotonic_ns());
 std::cout<<"implementation=legacy_mpc_common_gate_v1 accepted="<<result.accepted
   <<" status="<<result.status<<" total_issue_ms="<<result.timing.issue_age_ms
   <<" stopping_violation_rad="<<result.validation.stopping_violation<<'\n';
 // This standalone example does not publish either actuator command.
 if(result.accepted)controller.discard(result.token);
 if(controller.has_pending()||controller.committed_memory().step_count!=0)return 2;
 return result.accepted?0:1;
 }catch(const std::exception& error){std::cerr<<error.what()<<'\n';return 2;}}
