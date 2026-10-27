#pragma once
// Synthetic call fixture only; these are not the X650 flight parameters.
#include "x650_mpc/controller.hpp"
#include <cmath>
extern "C" int ts20_prediction_batch(const double*,const double*,double,double,const double*,const double*,const double*,int,const double*,int,int,double*,int*,int*,double*,double*,int,int*,int*,char*,int);
namespace fixture {
inline x650::Config config() {
  x650::Config c;
  c.geometry={.2,.2,-.7853981633974483,.7853981633974483,0,0,0,0,0,0};
  c.physical_lower={-2,-2};c.physical_upper={2,2};
  c.response={4.,.8,4.,.8,.25};c.base_position_gain={1,1,1};
  c.motion={.0007447642530597398,.004194907941904997,.03470285366117535,.0004632811241788992,.003346379092576424,.075,.04,.2};
  c.max_dt=.1;c.command_hold_s=.16;c.trust_radius=.02;
  return c;
}
inline x650::StepInput input(double t=.04, double dt=.04) {
  x650::StepInput in;in.t=t;in.dt=dt;
  in.state.stamp=in.state.control_time=in.state.joint_stamp=10.+t;
  in.target.position={.4*std::cos(.7853981633974483),0,0};
  in.context.reference_time=0;in.context.commands={{0,{0,0}}};
  for(int i=-20;i<=0;++i){x650::Packet p;p.send_time=i*.005;p.kp={6,6};p.kd={.25,.25};in.context.packets.push_back(p);}
  return in;
}
inline bool same_memory(const x650::Memory&a,const x650::Memory&b) {
  if(a.initialized!=b.initialized||a.history_primed!=b.history_primed||a.has_last_t!=b.has_last_t||a.has_stamp!=b.has_stamp||a.has_origin!=b.has_origin||a.anchor.position!=b.anchor.position||a.anchor.rotation!=b.anchor.rotation||a.anchor.q!=b.anchor.q||a.previous_command!=b.previous_command||a.last_t!=b.last_t||a.previous_stamp!=b.previous_stamp||a.origin!=b.origin||a.plan_t!=b.plan_t||a.step_count!=b.step_count||a.plan!=b.plan||a.plan_h!=b.plan_h||a.joint_history.size()!=b.joint_history.size())return false;
  for(std::size_t i=0;i<a.joint_history.size();++i)if(a.joint_history[i].time!=b.joint_history[i].time||a.joint_history[i].velocity!=b.joint_history[i].velocity)return false;
  return true;
}
}
