#pragma once
#include "mpc_core.hpp"
namespace x650_mpc {
// Exact Config extracted from the already-tested multirate startup manifest.
// Source SHA256: ee6d3c300717b13b21b6a7a0eecf257e628664ff822f8683baf11b731fa4009b
// No geometry, dynamics, numeric tolerances, or weights are retuned here.
inline x650::Config selected_x650_simulation_profile() {
  x650::Config c;
  c.nodes=15;c.max_iterations=100;c.node_dt=.04;c.max_dt=.1;c.command_hold_s=.16;
  c.feasibility_tolerance=2e-7;c.kkt_tolerance=.002;c.arm_share=.2;c.joint_position_gain=3.;
  c.weights={400.,4.,.25,.02,.01,.01};
  c.velocity_limit={.55,.35,.45,.45,.35};c.acceleration_limit={.45,.30,.45,.60,.60};
  c.q_lower={-.75,-.35};c.q_upper={.45,.85};
  c.physical_lower={-1.0471975511965976,-.4363323129985824};
  c.physical_upper={.5235987755982988,1.0471975511965976};
  c.geometry={.3,.25,2.6179938779914944,0.,0.,0.,-.18,0.,.04,0.};
  c.response={2.8471958701784543,.695123475648465,2.7956770526957433,.69189021293641,.19588596825267549};
  c.base_position_gain={.949999988079071,.949999988079071,1.};
  c.motion={.0007447642530597398,.004194907941904997,.03470285366117535,.0004632811241788992,.003346379092576424,.075,.04,.2};
  c.kp={6.,6.};c.kd={.25,.25};c.bus_period=.005;c.integration_step=.001;c.trust_radius=.15;
  return c;
}
}
