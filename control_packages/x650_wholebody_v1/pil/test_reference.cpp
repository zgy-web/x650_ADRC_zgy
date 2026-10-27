#include "flyarm_mpc_cpp/worker_contract.hpp"
#include <stdexcept>
#include <limits>
#include <cmath>
void check(bool ok){if(!ok)throw std::runtime_error("Cross-host reference contract regression");}
int main(){
  using flyarm_mpc_cpp::same_reference;
  const std::array<double,3> a{.001,0.,2.};auto b=a;
  check(same_reference(a,b));
  b[0]=std::nextafter(a[0],1.);check(same_reference(a,b));
  b=a;b[0]+=8.673617379884035e-19;check(same_reference(a,b));
  b=a;b[1]=64*std::numeric_limits<double>::epsilon();check(same_reference(a,b));
  b[1]=std::nextafter(b[1],1.);check(!same_reference(a,b));
  for(auto x:{1e-12,1e-10,1e-7}){b=a;b[0]+=x;check(!same_reference(a,b));}
  b=a;b[0]=std::numeric_limits<double>::quiet_NaN();check(!same_reference(a,b));
  b=a;b[0]=std::numeric_limits<double>::infinity();check(!same_reference(a,b));
}
