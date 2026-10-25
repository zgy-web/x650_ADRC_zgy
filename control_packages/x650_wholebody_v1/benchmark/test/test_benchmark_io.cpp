#include "benchmark_io.hpp"
#include <iostream>

namespace b=x650_benchmark;
template<class F> void rejects(F f) {
  bool rejected=false;try {f();}catch(const std::exception&){rejected=true;}
  if(!rejected)throw std::runtime_error("Expected strict input rejection");
}
int main() {try {
  if(b::sha256_bytes("abc")!="ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad")throw std::runtime_error("SHA known answer failed");
  rejects([]{b::parse_json("{\"a\":1,\"a\":2}");});
  rejects([]{b::parse_json("{\"a\":1} garbage");});
  rejects([]{b::parse_json("{\"a\":NaN}");});
  rejects([]{b::number(Json::Value(true));});
  rejects([]{b::integer(Json::Value(1.2));});
  rejects([]{b::numbers<2>(b::parse_json("[1]"));});
  rejects([]{b::oracle_of(b::parse_json("{\"command\":[0,0,0,0,0]}"));});
  rejects([]{b::oracle_of(b::parse_json("{\"command\":[0,0,0,0,0],\"position_reference\":[0,0,0],\"velocity_feedforward\":[0,0,0],\"extra\":0}"));});
  const auto oracle=b::oracle_of(b::parse_json("{\"command\":[0,0,0,0,0],\"position_reference\":[0,0,0],\"velocity_feedforward\":[0,0,0]}"));
  if(oracle.fields[0].size()!=5||!oracle.fields[3].empty())throw std::runtime_error("Oracle availability was invented");
  x650::Memory a,bm; a.plan={1,2};bm=a;
  if(!b::same_memory(a,bm))throw std::runtime_error("Memory equality failed");bm.previous_stamp=1;
  if(b::same_memory(a,bm))throw std::runtime_error("Memory clocks ignored");
  bm=a;bm.joint_history.push_back({0,{0,0}});
  if(b::same_memory(a,bm))throw std::runtime_error("Memory history ignored");
  std::cout<<"PASS standalone strict JSON, SHA, oracle availability and complete Memory equality\n";return 0;
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
