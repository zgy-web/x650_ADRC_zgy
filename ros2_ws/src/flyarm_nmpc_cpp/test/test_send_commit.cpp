#include "flyarm_nmpc_cpp/send_commit.hpp"
#include <iostream>
namespace fn=flyarm_nmpc_cpp;
struct Snapshot {std::uint64_t sequence{},issued_ns{};};
void require(bool ok,const char* reason){if(!ok)throw std::runtime_error(reason);}
template<class F> void rejects(F f){bool bad=false;try{f();}catch(const std::exception&){bad=true;}require(bad,"Expected transaction rejection");}
int main(){try{
  fn::SendCommit<Snapshot> gate(3,7);gate.begin(2.,.25);
  fn::ExpectedSend candidate{1,100,{.1,.2,.3,.04,-.02},{1.,2.,3.},{.1,.2,.3}};
  gate.stage(candidate);require(gate.committed()==0,"Preparing or publishing changed committed memory");
  gate.defer({2,120});rejects([&]{gate.defer({3,130});});
  fn::SendReceipt send{3,7,1,10,150,8.,false,{91,87},{.04,-.02},1,2048,{2.,1.,-1.},{.2,.1,-.3},{0.,0.,0.},.25};
  auto wrong=send;wrong.joint_velocity[1]=.02;rejects([&]{gate.matches(wrong,160);});
  wrong=send;wrong.sent_ns=99;rejects([&]{gate.matches(wrong,160);});
  rejects([&]{gate.matches(send,25000151);});
  wrong=send;wrong.session=4;require(!gate.matches(wrong,160),"Foreign session accepted");
  wrong=send;wrong.stop_zero=true;require(!gate.matches(wrong,160),"Stop output committed candidate");
  wrong=send;wrong.sequence=3;rejects([&]{gate.matches(wrong,160);});
  require(gate.matches(send,160),"Actual publication receipt rejected");
  // No consumption receipt is supplied. Only successful backend memory commit
  // may call complete; a thrown backend commit would leave the ledger pending.
  require(gate.committed()==0&&gate.pending(),"Matching check prematurely committed");
  const auto next=gate.complete();require(next&&next->sequence==2&&next->issued_ns==120,"Original buffered issue changed");
  require(gate.committed()==1&&!gate.pending()&&!gate.matches(send,161),"Repeated actual send double committed");
  candidate.sequence=2;candidate.published_begin_ns=170;gate.stage(candidate);gate.discard();
  require(gate.committed()==1&&!gate.pending(),"Unsent final proposal advanced memory");
  std::cout<<"PASS actual publication commit, one reordered snapshot, stale/payload rejection, duplicate suppression, unsent discard; no consumer wait\n";return 0;
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
