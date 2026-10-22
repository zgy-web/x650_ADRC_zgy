// Conditional TS20-50 free-motion I/O surrogate. This is NOT a torque model.
// Command-normalized inertia/friction are never written into Gazebo inertias.
// Physics is driven with interval-mean velocity so its joint displacement is
// the free-motion model displacement; available shaft torque/load dependence
// is not represented. Physical and internal endpoint velocities differ.
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <deque>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

namespace ts20interface {
struct Parameters {
  double inertia{}, viscous{}, coulomb{}, offset{}, delay{}, fs{}, delta{}, vs{};
};
struct State { double q{}, v{}; };
struct Mit { double q{}, v{}, tau{}, kp{}, kd{}; };
struct Packet { double send{}, apply{}; std::uint64_t sequence{}; std::array<Mit,2> mit{}; };
constexpr double PositionStep = 6.283185307179586476925286766559 / 16777216.;
constexpr double VelocityStep = 1. / 512.;
double quantize(double x, double step) { return std::round(x / step) * step; }
double drive(const State &x, const Mit &m, const Parameters &p) {
  return m.kp * (m.q-x.q) + m.kd*m.v + m.tau - p.offset;
}
State derivative(const State &x, const Mit &m, const Parameters &p, double sign) {
  const double ratio = std::abs(x.v)/p.vs;
  const double friction = p.coulomb + (ratio < 28 ? p.delta*std::exp(-ratio*ratio) : 0.);
  return {x.v, (drive(x,m,p)-(p.viscous+m.kd)*x.v-friction*sign)/p.inertia};
}
State rk4(const State &x, const Mit &m, const Parameters &p, double sign, double h) {
  const auto a = derivative(x,m,p,sign);
  const auto b = derivative({x.q+h*a.q/2,x.v+h*a.v/2},m,p,sign);
  const auto c = derivative({x.q+h*b.q/2,x.v+h*b.v/2},m,p,sign);
  const auto d = derivative({x.q+h*c.q,x.v+h*c.v},m,p,sign);
  return {x.q+h*(a.q+2*b.q+2*c.q+d.q)/6,
          x.v+h*(a.v+2*b.v+2*c.v+d.v)/6};
}
// Zero-velocity events use a signed sliding branch and bisection, not an
// arbitrary low-speed deadband. Static equality sticks, as in upstream.
void advance(State &x, const Mit &m, const Parameters &p, double duration,
             std::uint64_t &zeros, std::uint64_t &releases) {
  double remaining=duration;
  unsigned count=0;
  while (remaining > 1e-14) {
    if (++count > 20000) throw std::runtime_error("TS20 event integration budget exhausted");
    if (x.v == 0.) {
      if (std::abs(drive(x,m,p)) <= p.fs) return;
      ++releases;
    }
    const double sign = x.v == 0. ? std::copysign(1.,drive(x,m,p)) : std::copysign(1.,x.v);
    const double h = std::min(remaining,.00025);
    const auto next = rk4(x,m,p,sign,h);
    if (x.v != 0. && sign*next.v <= 0.) {
      double lo=0., hi=h;
      for (int i=0;i<45;++i) {
        const double mid=(lo+hi)/2;
        if (sign*rk4(x,m,p,sign,mid).v > 0.) lo=mid; else hi=mid;
      }
      auto root=rk4(x,m,p,sign,hi); root.v=0.;
      x=root; remaining-=hi; ++zeros;
    } else { x=next; remaining-=h; }
    if (!std::isfinite(x.q) || !std::isfinite(x.v) || std::abs(x.q)>1e3 || std::abs(x.v)>1e3)
      throw std::runtime_error("TS20 interface state diverged");
  }
}
}

#ifdef TS20_CORE_SELF_TEST
#include <iostream>
int main() {
  using namespace ts20interface;
  const Parameters p{.0007447642530597398,.004194907941904997,.03470285366117535,
      .0004632811241788992,.003346379092576424,.075,.04,.2};
  State rest{}; std::uint64_t zeros=0,releases=0;
  advance(rest,{0.,.2,0.,0.,3276./32768.},p,1.,zeros,releases);
  if (rest.q!=0. || rest.v!=0.) throw std::runtime_error("static equality / pure velocity test failed");
  State x{};
  for(int i=0;i<1000;++i) advance(x,{.1,0.,0.,6.,3276./32768.},p,.001,zeros,releases);
  if (!(x.q>.08 && x.q<.12 && std::abs(x.v)<.02)) throw std::runtime_error("position tracking test failed");
  const double q=x.q;
  for(int i=0;i<1000;++i) advance(x,{-.1,0.,0.,6.,3276./32768.},p,.001,zeros,releases);
  if (!(x.q<-.08 && x.q>-.12 && q>0 && zeros>0)) throw std::runtime_error("reverse/stick event test failed");
  if (quantize(-.5/512.,VelocityStep) != -1./512.) throw std::runtime_error("wire rounding test failed");
  std::cout << "TS20 core self-test PASS; q=" << x.q << " v=" << x.v
            << " zero_events=" << zeros << " release_events=" << releases << '\n';
}
#else
#include <gz/sim/System.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Joint.hh>
#include <gz/sim/components/JointVelocityCmd.hh>
#include <gz/plugin/Register.hh>
#include <gz/transport/Node.hh>
#include <gz/msgs/double.pb.h>
#include <chrono>
#include <memory>

namespace ts20interface {
namespace gs=gz::sim;
class TS20Interface : public gs::System, public gs::ISystemConfigure,
                      public gs::ISystemPreUpdate, public gs::ISystemPostUpdate {
  Parameters p;
  std::array<gs::Entity,2> joints{};
  std::array<State,2> internal{};
  std::array<double,2> reference{}, upper{}, receipt{}, pending{};
  std::array<std::uint64_t,2> pendingSeq{}, consumedSeq{};
  std::array<std::uint64_t,2> zeros{}, releases{};
  std::mutex mutex;
  gz::transport::Node node;
  std::ofstream states, packets;
  std::deque<Packet> queue;
  Packet active{}, sent{};
  double busPeriod=.005, kp=6., kd=3276./32768., origin=0., nextBus=0.;
  double intervalBegin=0., lastEnd=-1., referenceTime=0.;
  std::uint64_t busIndex=0, packetSequence=0;
  bool ready=false;
  static double scalar(const std::shared_ptr<const sdf::Element> &s, const char *key) {
    if (!s->HasElement(key)) throw std::runtime_error(std::string("TS20 missing SDF field: ")+key);
    const double value=s->Get<double>(key);
    if (!std::isfinite(value)) throw std::runtime_error("TS20 nonfinite configuration");
    return value;
  }
  static double first(const std::optional<std::vector<double>> &v) {
    return v && !v->empty() ? v->at(0) : std::numeric_limits<double>::quiet_NaN();
  }
  void receive(std::size_t joint, const gz::msgs::Double &msg) {
    if (!std::isfinite(msg.data())) return; // Never turn NaN into a motion command.
    std::lock_guard<std::mutex> lock(mutex);
    pending[joint]=msg.data(); ++pendingSeq[joint];
  }
  void send(double t) {
    sent={t,t+p.delay,++packetSequence,{}};
    for (std::size_t j=0;j<2;++j)
      sent.mit[j]={quantize(reference[j],PositionStep),quantize(upper[j],VelocityStep),0.,kp,kd};
    queue.push_back(sent);
    packets<<sent.send<<','<<sent.apply<<','<<sent.sequence;
    for(const auto &m:sent.mit) packets<<','<<m.q<<','<<m.v<<','<<m.tau<<','<<m.kp<<','<<m.kd;
    packets<<'\n'; packets.flush();
  }
  void velocity(gs::EntityComponentManager &ecm, std::size_t j, double v) {
    auto component=ecm.Component<gs::components::JointVelocityCmd>(joints[j]);
    if(component) component->Data()=std::vector<double>{v};
    else ecm.CreateComponent(joints[j],gs::components::JointVelocityCmd({v}));
  }
public:
  void Configure(const gs::Entity &entity,const std::shared_ptr<const sdf::Element> &s,
                 gs::EntityComponentManager &ecm,gs::EventManager &) override {
    if(!s) throw std::runtime_error("TS20 requires explicit configuration");
    p={scalar(s,"normalized_inertia"),scalar(s,"normalized_viscous"),scalar(s,"normalized_coulomb"),
       scalar(s,"normalized_offset"),scalar(s,"combined_interface_delay_s"),scalar(s,"static_threshold"),
       scalar(s,"delta_static"),scalar(s,"stribeck_velocity")};
    busPeriod=scalar(s,"bus_period_s"); kp=scalar(s,"kp"); kd=scalar(s,"kd");
    if (!(p.inertia>0 && p.viscous>=0 && p.coulomb>=0 && p.delay>=0 && p.delay<=.1 &&
          p.fs>=p.coulomb+p.delta && p.delta>=0 && p.vs>0 && busPeriod>=.001 && busPeriod<=.01 && kp>=0 && kd>=0))
      throw std::runtime_error("TS20 normalized parameter sign/range error");
    const gs::Model model(entity);
    const auto modelName=model.Name(ecm);
    const std::array<std::string,2> names={"arm_joint_1","arm_joint_2"};
    for(std::size_t j=0;j<2;++j) {
      joints[j]=model.JointByName(ecm,names[j]);
      if(joints[j]==gs::kNullEntity) throw std::runtime_error("TS20 arm joint missing");
      gs::Joint(joints[j]).EnablePositionCheck(ecm);
      gs::Joint(joints[j]).EnableVelocityCheck(ecm);
      velocity(ecm,j,0.);
      const auto topic="/model/"+modelName+"/joint/"+names[j]+"/cmd_vel";
      if(!node.Subscribe<gz::msgs::Double>(topic,[this,j](const gz::msgs::Double &msg){receive(j,msg);}))
        throw std::runtime_error("TS20 command subscription failed");
    }
    std::filesystem::path statePath=s->Get<std::string>("state_csv");
    std::filesystem::path packetPath=s->Get<std::string>("packet_csv");
    if(const char *directory=std::getenv("AMSIM_TS20_LOG_DIR")) {
      const std::filesystem::path folder(directory);
      if(!folder.is_absolute() || !std::filesystem::is_directory(folder))
        throw std::runtime_error("AMSIM_TS20_LOG_DIR must name an existing absolute run directory");
      statePath=folder/"ts20_state.csv"; packetPath=folder/"ts20_packets.csv";
    }
    if(std::filesystem::exists(statePath)||std::filesystem::exists(packetPath))
      throw std::runtime_error("TS20 refuses to overwrite prior state/packet evidence");
    states.open(statePath,std::ios::out|std::ios::trunc);
    packets.open(packetPath,std::ios::out|std::ios::trunc);
    if(!states || !packets) throw std::runtime_error("TS20 cannot create independent state/packet CSV");
    states<<std::setprecision(17); packets<<std::setprecision(17);
    packets<<"send_s,apply_s,sequence,qref1,dqref1,tau1,kp1,kd1,qref2,dqref2,tau2,kp2,kd2\n";
    states<<"sim_s,step_dt_s,interval_begin_s";
    for(int j=1;j<=2;++j)
      states<<",q"<<j<<"_internal,dq"<<j<<"_internal,q"<<j<<"_physical,dq"<<j<<"_physical,r"<<j
            <<",u"<<j<<",receipt"<<j<<"_s,upper_sequence"<<j<<",zero_events"<<j<<",release_events"<<j;
    states<<",last_send_s,last_send_apply_s,last_send_sequence,last_qref1,last_dqref1,last_qref2,last_dqref2"
          <<",active_send_s,active_apply_s,active_sequence,active_qref1,active_dqref1,active_qref2,active_dqref2\n";
    states.flush(); packets.flush();
  }
  void PreUpdate(const gs::UpdateInfo &info,gs::EntityComponentManager &ecm) override {
    if(info.paused) return;
    const double end=std::chrono::duration<double>(info.simTime).count();
    const double dt=std::chrono::duration<double>(info.dt).count();
    if (!(dt>0) || dt>.02) throw std::runtime_error("TS20 requires 0 < physics dt <= 20 ms");
    intervalBegin=end-dt;
    if(ready && std::abs(intervalBegin-lastEnd)>1e-8)
      throw std::runtime_error("TS20 simulation clock reset/gap; launch a fresh instance");
    if(!ready) {
      for(std::size_t j=0;j<2;++j) {
        const double q=first(gs::Joint(joints[j]).Position(ecm));
        const double v=first(gs::Joint(joints[j]).Velocity(ecm));
        if(!std::isfinite(q)||!std::isfinite(v)) return;
        internal[j]={q,v}; reference[j]=q; receipt[j]=intervalBegin;
        active.mit[j]={quantize(q,PositionStep),0.,0.,kp,kd};
      }
      origin=intervalBegin; nextBus=origin; referenceTime=origin;
      active.send=origin; active.apply=origin; ready=true;
    }
    {
      std::lock_guard<std::mutex> lock(mutex);
      for(std::size_t j=0;j<2;++j) if(pendingSeq[j]!=consumedSeq[j]) {
        upper[j]=pending[j]; consumedSeq[j]=pendingSeq[j]; receipt[j]=intervalBegin;
      }
    }
    const auto before=internal;
    double cursor=intervalBegin;
    unsigned events=0;
    while(cursor<end-1e-13) {
      if(++events>1000) throw std::runtime_error("TS20 packet event budget exhausted");
      if(nextBus<=cursor+1e-13) {
        for(std::size_t j=0;j<2;++j) reference[j]+=upper[j]*(cursor-referenceTime);
        referenceTime=cursor;
        send(nextBus);
        nextBus=origin+(++busIndex)*busPeriod;
      }
      while(!queue.empty() && queue.front().apply<=cursor+1e-13) {
        active=queue.front(); queue.pop_front();
      }
      double right=std::min(end,nextBus);
      if(!queue.empty()) right=std::min(right,queue.front().apply);
      if(right<=cursor) throw std::runtime_error("TS20 packet clock failed to advance");
      for(std::size_t j=0;j<2;++j) advance(internal[j],active.mit[j],p,right-cursor,zeros[j],releases[j]);
      cursor=right;
    }
    // Keep the continuous reference current at every physics boundary. This
    // matters when a new upper command arrives before the next 5 ms bus send.
    for(std::size_t j=0;j<2;++j) {
      reference[j]+=upper[j]*(end-referenceTime);
      velocity(ecm,j,(internal[j].q-before[j].q)/dt);
    }
    referenceTime=end; lastEnd=end;
  }
  void PostUpdate(const gs::UpdateInfo &info,const gs::EntityComponentManager &ecm) override {
    if(info.paused || !ready) return;
    const double sim=std::chrono::duration<double>(info.simTime).count();
    states<<sim<<','<<std::chrono::duration<double>(info.dt).count()<<','<<intervalBegin;
    for(std::size_t j=0;j<2;++j)
      states<<','<<internal[j].q<<','<<internal[j].v
            <<','<<first(gs::Joint(joints[j]).Position(ecm))<<','<<first(gs::Joint(joints[j]).Velocity(ecm))
            <<','<<reference[j]<<','<<upper[j]<<','<<receipt[j]<<','<<consumedSeq[j]<<','<<zeros[j]<<','<<releases[j];
    states<<','<<sent.send<<','<<sent.apply<<','<<sent.sequence;
    for(const auto &m:sent.mit) states<<','<<m.q<<','<<m.v;
    states<<','<<active.send<<','<<active.apply<<','<<active.sequence;
    for(const auto &m:active.mit) states<<','<<m.q<<','<<m.v;
    states<<'\n'; states.flush();
  }
};
}
GZ_ADD_PLUGIN(ts20interface::TS20Interface,gz::sim::System,gz::sim::ISystemConfigure,
              gz::sim::ISystemPreUpdate,gz::sim::ISystemPostUpdate)
GZ_ADD_PLUGIN_ALIAS(ts20interface::TS20Interface,"ts20interface::TS20Interface")
#endif

