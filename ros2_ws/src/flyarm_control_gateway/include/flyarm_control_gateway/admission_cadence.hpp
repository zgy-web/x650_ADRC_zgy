#pragma once
#include <cmath>
#include <cstdint>
namespace flyarm_control_gateway {
// The physical model's control period is measured on its actual /clock.
// Monotonic wall time separately governs all transport age/deadline checks.
// Comparing both as minima quantizes a 20 ms source update into 24/28 ms.
inline bool admission_due(bool live,std::uint64_t now,std::uint64_t next_wall,
                          double control,double previous_control) {
  return live ? std::isfinite(control) && std::isfinite(previous_control) &&
                  control-previous_control>=.020-1e-10
              : now>=next_wall;
}
}
