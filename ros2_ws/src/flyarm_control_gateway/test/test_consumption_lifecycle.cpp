#include "flyarm_control_gateway/consumption_lifecycle.hpp"
#include <stdexcept>
#include <iostream>

using Life=flyarm_control_gateway::ConsumptionLifecycle;
static void require(bool pass,const char* message){if(!pass)throw std::runtime_error(message);}
template<class F>static void rejected(F f){bool failed=false;try{f();}catch(const std::runtime_error&){failed=true;}require(failed,"expected fail-closed rejection");}

int main() {
  constexpr std::uint64_t ms=1'000'000;
  Life life;
  life.begin_poll(1*ms);
  require(!life.finish_poll(100*ms,true,true),"startup parse cannot publish live state");
  life.ready();
  life.begin_poll(500*ms);
  require(!life.finish_poll(600*ms,true,true),"idle backlog cannot publish live state");
  life.start(700*ms,701*ms);
  life.begin_poll(702*ms);
  require(!life.finish_poll(703*ms,false,true),"full buffer remains historical");
  life.begin_poll(704*ms);
  require(!life.finish_poll(705*ms,true,true),"EOF handoff barrier must discard existing complete rows");
  require(life.phase()==Life::ACTIVE,"caught-up barrier advances active state");
  life.begin_poll(706*ms);
  require(!life.finish_poll(707*ms,true,false),"no newly completed row means no evidence");
  life.begin_poll(708*ms);
  require(life.finish_poll(709*ms,true,true),"new complete row can publish after barrier");
  life.begin_poll(733*ms);
  require(life.finish_poll(734*ms,true,true),"exactly 25 ms gap remains valid");
  rejected([&]{life.begin_poll(758*ms+1);});

  Life stale;stale.ready();rejected([&]{stale.start(1*ms,26*ms+1);});
  Life delayed;delayed.ready();delayed.start(100*ms,124*ms);
  rejected([&]{delayed.begin_poll(125*ms+1);}); // callback arrival cannot renew the original lease
  Life future;future.ready();rejected([&]{future.start(10*ms,9*ms);});
  Life premature;rejected([&]{premature.start(1*ms,1*ms);});
  Life twice;twice.ready();twice.start(1*ms,1*ms);rejected([&]{twice.start(2*ms,2*ms);});
  Life parse;parse.ready();parse.start(1*ms,1*ms);parse.begin_poll(2*ms);
  rejected([&]{parse.finish_poll(27*ms+1,true,true);});
  Life backward;backward.ready();backward.start(10*ms,10*ms);rejected([&]{backward.begin_poll(9*ms);});
  std::cout<<"consumption startup/new-row/active timing contract passed\n";
}
