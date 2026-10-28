#include "flyarm_hardware_state/state_math.hpp"
#include <iostream>
#include <stdexcept>
using namespace flyarm_hw;
void check(bool x) { if (!x) throw std::runtime_error("hardware state unit check failed"); }
void near(double x,double y) { check(std::abs(x-y)<1e-10); }
Vec3 rotate(const Quat &q, const Vec3 &v) {
  auto p=multiply(multiply(q,{0,v[0],v[1],v[2]}),{q[0],-q[1],-q[2],-q[3]});
  return {p[1],p[2],p[3]};
}
int main() {
  check(ned_to_enu({1,2,3})==Vec3({2,1,-3}));
  check(frd_to_flu({1,2,3})==Vec3({1,-2,-3}));
  auto identity=enu_flu({1,0,0,0});
  auto north=rotate(identity,{1,0,0}); near(north[0],0); near(north[1],1); near(north[2],0);
  const double s=std::sqrt(0.5);
  auto east=enu_flu({s,0,0,s}); near(east[0],1); near(east[1],0); near(east[2],0); near(east[3],0);
  Quat a{0.8,0.1,0.2,0.3}; auto n=norm(a); for(auto &x:a)x/=n;
  // Independent vector-frame composition exercises roll, pitch and yaw together.
  for(const Vec3 v : {Vec3{1,0,0},Vec3{0,1,0},Vec3{0,0,1}}) {
    auto expected=ned_to_enu(rotate(a,frd_to_flu(v)));
    auto actual=rotate(enu_flu(a),v);
    for(int i=0;i<3;++i)near(expected[i],actual[i]);
  }
  check(!finite(enu_flu({0,0,0,0})));
  check(!finite(enu_flu({2,0,0,0})));
  check(topic("vehicle_local_position",1)=="/fmu/out/vehicle_local_position_v1");
  check(topic("vehicle_attitude",0)=="/fmu/out/vehicle_attitude");
  Stats st; check(!st.fresh(0,1));
  st.add(1,100); st.add(1.02,120); near(st.hz(),50); near(st.max_gap_ms,20);
  check(st.fresh(1.03,0.1)); check(!st.fresh(1.2,0.1));
  st.add(1.04,120); st.add(1.06,50); st.add(1.08,0);
  check(st.repeats==1 && st.regressions==2 && st.zero_stamps==1);
  check(!st.fresh(0.5,1));
  std::cout << "PASS: coordinate bases, quaternion composition, invalid states, topic versions, freshness and source time discontinuity\n";
}
