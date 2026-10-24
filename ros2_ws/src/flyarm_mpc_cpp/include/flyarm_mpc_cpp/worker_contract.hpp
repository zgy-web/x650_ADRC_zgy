#pragma once
// Typed, owning boundary. No Python, ROS executor, actuator writer or file I/O.
#include "mpc_core.hpp"
#include "flyarm_control_gateway/shadow_gate.hpp"
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <vector>

namespace flyarm_mpc_cpp {
namespace wire = flyarm_control_gateway;
struct Handoff {
  std::uint64_t session{}, epoch{}, issued_ns{};
  wire::Hash hash{};
  double origin{};
  x650::Anchor anchor;
  x650::V3 target{};
  std::uint32_t count{};
  std::array<double,64> times{};
  std::array<double,320> commands{};
};

class Contract {
 public:
  explicit Contract(wire::Config config) : config_(config), gate_(config) {
    if (config.replay_only) throw std::invalid_argument("C++ live worker requires explicit simulation identity");
  }
  std::vector<x650::Command5> begin(const Handoff& start,std::uint64_t now) {
    if (started_ || requests_ || faulted_) fail("LiveStart must occur once before every solve");
    if (start.session!=config_.session_id || start.epoch!=config_.epoch || start.hash!=config_.configuration_sha256
        || start.issued_ns==0 || start.issued_ns>now || !std::isfinite(start.origin) || start.origin<0
        || start.count<1 || start.count>64 || !finite(start.anchor.position) || !proper_rotation(start.anchor.rotation)
        || !finite(start.anchor.q) || !finite(start.target) || !finite(start.times) || !finite(start.commands))
      fail("Invalid actual LiveStart identity, time or finite anchor/history");
    if (std::abs(start.times[start.count-1]-start.origin)>1e-9) fail("Origin is not final actual prestream send");
    std::vector<x650::Command5> history; history.reserve(start.count);
    for (std::size_t i=0;i<start.count;++i) {
      if (i && start.times[i]<=start.times[i-1]) fail("Actual five-axis history is not strictly ordered");
      x650::Command5 row;row.time=start.times[i];
      std::copy_n(start.commands.begin()+5*i,5,row.velocity.begin());history.push_back(row);
    }
    for (std::size_t i=start.count;i<64;++i) {
      if (start.times[i]!=0.) fail("Unused actual handoff times must be zero");
      for (std::size_t j=0;j<5;++j) if(start.commands[5*i+j]!=0.) fail("Unused handoff commands must be zero");
    }
    origin_=last_control_=start.origin;target_=start.target;started_=true;
    return history;
  }
  x650::StepInput decode(const wire::Snapshot& s,std::uint64_t begin,int nodes,double node_dt) {
    ++requests_;
    if (!started_ || faulted_) fail("Live worker is unstarted or latched faulted");
    const auto admission=gate_.admit(s,begin);
    if (!admission.forward) fail("Independent worker input contract rejected snapshot");
    for(std::size_t i=1;i<s.command_count;++i)
      if(s.command_time_s[i]<=s.command_time_s[i-1])fail("Actual command history must be strictly ordered");
    if (std::abs(s.control_time_s-origin_-s.task_time_s)>1e-9
        || std::abs(s.control_time_s-last_control_-s.dt_s)>1e-9
        || std::abs(s.reference_time_s-(s.task_time_s-(s.control_time_s-s.joint_time_s)))>1e-9
        || !proper_rotation(s.rotation_row_major) || s.position_gains!=config_.position_gains || s.target_position_enu!=target_
        || s.target_velocity_enu!=x650::V3{} || s.target_acceleration_enu!=x650::V3{}
        || nodes<1 || nodes>60 || !std::isfinite(node_dt) || node_dt<=0.)
      fail("Frozen hold reference, actual dt or reference anchor differs");
    x650::StepInput value;value.t=s.task_time_s;value.dt=s.dt_s;
    value.state.position=s.position_enu;value.state.velocity=s.velocity_enu;
    value.state.omega_body=s.omega_body_flu;value.state.acceleration=s.acceleration_enu;
    value.state.rotation=s.rotation_row_major;value.state.q=s.q;value.state.dq=s.dq;
    value.state.stamp=s.state_time_s;value.state.control_time=s.control_time_s;value.state.joint_stamp=s.joint_time_s;
    value.target={s.target_position_enu,s.target_velocity_enu,s.target_acceleration_enu};
    // Strict hold preview is explicit; never the core's approximate fallback.
    value.preview.assign(static_cast<std::size_t>(nodes),value.target);
    value.context.reference_time=s.reference_time_s;value.context.reference=s.reference_r;value.context.bus_origin=s.bus_origin_s;
    value.context.commands.reserve(s.command_count);value.context.packets.reserve(s.packet_count);
    for (std::size_t i=0;i<s.command_count;++i)
      value.context.commands.push_back({s.command_time_s[i],{s.command_velocity[2*i],s.command_velocity[2*i+1]}});
    for (std::size_t i=0;i<s.packet_count;++i) {
      x650::Packet packet;packet.send_time=s.packet_send_time_s[i];
      for (std::size_t j=0;j<2;++j) {
        packet.q_ref[j]=s.packet_q_ref[2*i+j];packet.dq_ref[j]=s.packet_dq_ref[2*i+j];
        packet.tau_ff[j]=s.packet_tau_ff[2*i+j];packet.kp[j]=s.packet_kp[2*i+j];packet.kd[j]=s.packet_kd[2*i+j];
      }
      value.context.packets.push_back(packet);
    }
    return value;
  }
  void accept_output(const wire::Proposal& proposal,std::uint64_t now) {
    if (faulted_) fail("Worker output attempted after latched failure");
    const auto event=gate_.propose(proposal,now);
    if (!event || !event->accepted) fail("Independent worker output quality/clock/PVA contract rejected candidate");
    last_control_=proposal.control_time_s;++accepted_;
  }
  void latch() noexcept {faulted_=true;}
  bool faulted() const noexcept {return faulted_;}
  bool started() const noexcept {return started_;}
  std::uint64_t requests() const noexcept {return requests_;}
  std::uint64_t accepted() const noexcept {return accepted_;}
 private:
  template<std::size_t N> static bool finite(const std::array<double,N>& value) {
    return std::all_of(value.begin(),value.end(),[](double v){return std::isfinite(v);});
  }
  static bool proper_rotation(const x650::R9& r) {
    if(!finite(r))return false;
    // Same ControlState/adapter check: np.allclose(R.T@R,I,atol=1e-8)
    // with its default rtol=1e-5, and the independent determinant bound.
    for(int i=0;i<3;++i)for(int j=0;j<3;++j){double dot=0.;for(int k=0;k<3;++k)dot+=r[3*k+i]*r[3*k+j];
      if(std::abs(dot-(i==j?1.:0.))>1e-8+(i==j?1e-5:0.))return false;}
    const double determinant=r[0]*(r[4]*r[8]-r[5]*r[7])-r[1]*(r[3]*r[8]-r[5]*r[6])+r[2]*(r[3]*r[7]-r[4]*r[6]);
    return std::abs(determinant-1.)<=1e-8;
  }
  [[noreturn]] void fail(const char* text) {faulted_=true;throw std::invalid_argument(text);}
  wire::Config config_;wire::ShadowGate gate_;
  bool started_{},faulted_{};std::uint64_t requests_{},accepted_{};
  double origin_{},last_control_{};x650::V3 target_{};
};
}
