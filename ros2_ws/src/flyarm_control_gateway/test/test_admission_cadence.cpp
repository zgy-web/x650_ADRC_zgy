#include "flyarm_control_gateway/admission_cadence.hpp"
#include <iostream>
#include <limits>
#include <stdexcept>
using flyarm_control_gateway::admission_due;
void check(bool ok) {if(!ok)throw std::runtime_error("admission cadence check failed");}
int main() {
  // Recorded failure pattern: a 20 ms physics advance arrives 0.3 ms before
  // the independent wall minimum. It must not wait an extra physics step.
  check(admission_due(true,1'019'700'000,1'020'000'000,42.624,42.604));
  // Wall time passing cannot create physical model time, nor make a repeated
  // or backwards source sample eligible. Source age guards remain separate.
  check(!admission_due(true,1'050'000'000,1'020'000'000,42.620,42.604));
  check(!admission_due(true,1'050'000'000,1'020'000'000,42.604,42.604));
  check(!admission_due(true,1'050'000'000,1'020'000'000,42.600,42.604));
  check(!admission_due(true,1'050'000'000,1'020'000'000,std::numeric_limits<double>::quiet_NaN(),42.604));
  check(!admission_due(false,1'019'700'000,1'020'000'000,42.624,42.604));
  check(admission_due(false,1'020'000'000,1'020'000'000,42.624,42.604));
  std::cout<<"PASS: physical cadence, held/backwards source, and unchanged shadow wall cadence\n";
}
