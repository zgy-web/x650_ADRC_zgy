#include "flyarm_control_gateway/consumption_csv.hpp"
#include <iostream>
using P=flyarm_control_gateway::ConsumptionCsv;
void check(bool ok) {if(!ok)throw std::runtime_error("consumption CSV check failed");}
const std::string H="sim_s,upper_sequence1,upper_sequence2,u1,u2,receipt1_s,receipt2_s\n";
template<class F>void rejects(F f) {bool bad=false;try{f();}catch(const std::exception &){bad=true;}check(bad);}
int main() {
  P p;check(!p.append(H+"1.004,2,3,0.1,-0.1,1.0"));
  auto r=p.append("04,1.004\n");check(r && r->counter[0]==2 && r->counter[1]==3 && r->sim==1.004);
  check(!p.append(""));r=p.append("1.008,2,3,0.1,-0.1,1.004,1.004\n");check(r.has_value());
  rejects([&]{p.append("1.012,2,3,0.2,-0.1,1.004,1.004\n");});
  for(auto line:{"1.0,1,1,0,0,1,1\n","1.1,1,1,0,0,1,1\n","1.008,0,1,0,0,1,1\n",
      "1.008,1.5,1,0,0,1,1\n","1.008,1,1,nan,0,1,1\n","1.008,1,1,0,0,2,1\n","1.008,1,1,0,0,1\n"}) {
    P q;q.append(H+"1.004,1,1,0,0,1,1\n");rejects([&]{q.append(line);});
  }
  rejects([&]{P q;q.append("sim_s,sim_s\n");});
  std::cout<<"PASS: partial rows, finite values, counter/value identity, source chronology and physics gaps\n";
}
