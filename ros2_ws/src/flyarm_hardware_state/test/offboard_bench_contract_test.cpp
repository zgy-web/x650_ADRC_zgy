#include "flyarm_hardware_state/offboard_bench_contract.hpp"
#include <iostream>
#include <stdexcept>
using namespace flyarm_hw;
void check(bool value){if(!value)throw std::runtime_error("offboard bench contract test failed");}
BenchState state(double now=10){
  BenchState s;s.arm=1;s.nav=1;s.system=s.component=1;s.rc_valid=true;s.throttle=-1;
  for(int i=0;i<3;++i){s.status.add(now-0.02+i*.01,1000000+i);s.attitude.add(now-0.02+i*.01,1000000+i);s.manual.add(now-0.02+i*.01,1000000+i);}
  return s;
}
int main(){
  auto s=state();OffboardBenchContract c;check(c.begin(s,10));check(c.tick(s,10));
  s.arm=2;s.nav=14;check(c.tick(s,10.01));check(c.offboard_seen);
  s.nav=1;check(!c.tick(s,10.02));s.nav=14;check(!c.tick(s,10.03));check(!c.begin(state(),10.03));
  unsigned cases=1;
  for(int fault=0;fault<12;++fault){
    OffboardBenchContract g;auto t=state();check(g.begin(t,10));
    switch(fault){
      case 0:t.status.last=8;break;case 1:t.attitude.last=9;break;case 2:t.manual.last=9;break;
      case 3:t.status.regressions=1;break;case 4:t.attitude.zero_stamps=1;break;case 5:t.reset=1;break;
      case 6:t.hil=1;break;case 7:t.failsafe=true;break;case 8:t.rc_valid=false;break;
      case 9:t.throttle=0;break;case 10:t.q[0]=0;break;case 11:t.roll=.2;break;
    }
    check(!g.tick(t,10.01));check(!g.tick(state(),10.02));++cases;
  }
  for(int mode: {0,2,14}){OffboardBenchContract g;auto t=state();t.nav=mode;check(!g.begin(t,10));++cases;}
  {OffboardBenchContract g;auto t=state();t.arm=2;check(!g.begin(t,10));++cases;}
  {OffboardBenchContract g;auto t=state();check(g.begin(t,10));t.nav=14;check(g.tick(t,10.01,3));
   t=state(13.02);t.nav=14;check(!g.tick(t,13.02,3));check(g.reason==std::string("planned_stream_loss"));++cases;}
  {OffboardBenchContract g;check(g.begin(state(),10));check(!g.tick(state(70),70));++cases;}
  std::cout<<"offboard_bench_contract: "<<cases<<" cases passed\n";
}
