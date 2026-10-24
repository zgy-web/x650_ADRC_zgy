#pragma once
// One frozen reference law shared with the actual C++ state source.
// No trajectory fitting, phase reset, dynamics modification or approximate preview.
#include "mpc_core.hpp"
#include "flyarm_state_cpp/source_core.hpp"
#include <cmath>
#include <stdexcept>
#include <vector>

namespace flyarm_mpc_cpp {
class TrajectoryPlan {
 public:
  TrajectoryPlan() = default;  // Original strict hold remains the default.
  static TrajectoryPlan normal() {
    TrajectoryPlan output;
    output.law_.figure8=true;output.law_.ax=.25;output.law_.az=.10;
    output.law_.period=12.;output.law_.cycles=2;output.law_.start=2.;output.law_.ramp=2.;
    output.law_.validate();return output;
  }
  static TrajectoryPlan fast() {
    TrajectoryPlan output;
    output.law_.figure8=true;output.law_.ax=.25;output.law_.az=.10;
    output.law_.period=8.;output.law_.cycles=2;output.law_.start=2.;output.law_.ramp=2.;
    output.law_.validate();return output;
  }
  bool is_normal() const noexcept {return law_.figure8 && law_.period==12.;}
  bool is_fast() const noexcept {return law_.figure8 && law_.period==8.;}
  x650::Target sample(double time,const x650::V3& center) const {
    const auto value=law_.sample(time,center);
    return {value.p,value.v,value.a};
  }
  std::vector<x650::Target> preview(double time,double dt,int nodes,double node_dt,
                                     const x650::V3& center) const {
    if(!std::isfinite(time)||!std::isfinite(dt)||dt<=0.||dt>.05||
       nodes<1||nodes>60||!std::isfinite(node_dt)||node_dt<=0.)
      throw std::invalid_argument("Invalid explicit reference horizon");
    std::vector<x650::Target> output;output.reserve(static_cast<std::size_t>(nodes));
    // Match Core's cumulative([actual dt,node_dt,...]), not a fixed 20-ms grid.
    double offset=0.;
    for(int i=0;i<nodes;++i){offset+=(i==0?dt:node_dt);output.push_back(sample(time+offset,center));}
    return output;
  }
 private:
  flyarm_state_cpp::Trajectory law_{};
};
}
