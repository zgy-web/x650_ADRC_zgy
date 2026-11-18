// Physical closure plus bounded force servo and independent world-pose audit.
#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <gz/common/Console.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/DetachableJoint.hh>
#include <gz/sim/components/JointPosition.hh>
#include <gz/sim/components/JointVelocity.hh>
#include <gz/sim/components/JointForceCmd.hh>
#include <gz/msgs/double.pb.h>
#include <gz/msgs/vector3d.pb.h>
#include <gz/transport/Node.hh>

namespace cad_arm {
class Closure : public gz::sim::System, public gz::sim::ISystemConfigure,
  public gz::sim::ISystemPreUpdate, public gz::sim::ISystemPostUpdate {
  gz::sim::Entity big{}, small{}, proxy{};
  std::array<gz::sim::Entity,2> motors{};
  gz::math::Vector3d anchorBig, anchorSmall;
  gz::transport::Node node;
  std::array<std::atomic<double>,2> requested{{0.,0.}};
  std::array<std::atomic<double>,2> velocity{{0.,0.}};
  std::array<std::atomic<unsigned long>,2> sequence{{0,0}};
  std::array<unsigned long,2> seen{};
  std::array<double,2> lastCommand{};
  gz::transport::Node::Publisher metrics;
  std::array<double,2> target{}, torque{};
  std::ofstream csv;
  bool ready=false, exercise=false, velocityMode=false;
  double nextLog=0.;
 public:
  void Configure(const gz::sim::Entity &e, const std::shared_ptr<const sdf::Element> &s,
      gz::sim::EntityComponentManager &ecm, gz::sim::EventManager &) override {
    gz::sim::Model m(e);
    big=m.LinkByName(ecm,"upper_arm_Link"); small=m.LinkByName(ecm,"fore_arm_Link");
    proxy=m.LinkByName(ecm,"B_closure_proxy");
    motors={m.JointByName(ecm,"upper_arm_joint"),m.JointByName(ecm,"fore_crank_joint")};
    if (!big || !small || !proxy || !motors[0] || !motors[1]) {gzerr<<"Missing closure entity\n";return;}
    anchorBig=s->Get<gz::math::Vector3d>("anchor_big");
    anchorSmall=s->Get<gz::math::Vector3d>("anchor_small");
    exercise=s->Get<bool>("exercise",false).first;
    velocityMode=s->Get<bool>("velocity_mode",false).first;
    metrics=node.Advertise<gz::msgs::Vector3d>("/x650_cad_arm/closure_metrics");
    if (s->HasElement("csv")) {
      csv.open(s->Get<std::string>("csv"));
      if (!csv) {gzerr<<"Cannot open closure audit CSV\n";return;}
      csv<<"t,closure_m,axis_rad,q0,q1,dq0,dq1,target0,target1,tau0,tau1\n"<<std::setprecision(12);
    }
    for (auto j:motors) {
      ecm.CreateComponent(j,gz::sim::components::JointPosition());
      ecm.CreateComponent(j,gz::sim::components::JointVelocity());
      ecm.CreateComponent(j,gz::sim::components::JointForceCmd({0.}));
    }
    if (s->Get<bool>("enabled",true).first) {
      auto constraint=ecm.CreateEntity();
      gz::sim::components::DetachableJointInfo info;
      info.parentLink=big;info.childLink=proxy;info.jointType="fixed";
      ecm.CreateComponent(constraint,gz::sim::components::DetachableJoint(info));
    }
    node.Subscribe("/x650_cad_arm/upper/position", &Closure::Upper, this);
    node.Subscribe("/x650_cad_arm/crank/position", &Closure::Crank, this);
    const auto prefix="/model/"+m.Name(ecm)+"/joint/";
    node.Subscribe(prefix+"upper_arm_joint/cmd_vel", &Closure::UpperVelocity,this);
    node.Subscribe(prefix+"fore_crank_joint/cmd_vel", &Closure::CrankVelocity,this);
    ready=true;
  }
  void Upper(const gz::msgs::Double &m) { if(std::isfinite(m.data())) requested[0]=std::clamp(m.data(),0.,1.95); }
  void Crank(const gz::msgs::Double &m) { if(std::isfinite(m.data())) requested[1]=std::clamp(m.data(),0.,.78); }
  void UpperVelocity(const gz::msgs::Double &m) { if(std::isfinite(m.data())) {velocity[0]=std::clamp(m.data(),-.3,.3);++sequence[0];} }
  void CrankVelocity(const gz::msgs::Double &m) { if(std::isfinite(m.data())) {velocity[1]=std::clamp(m.data(),-.3,.3);++sequence[1];} }
  void PreUpdate(const gz::sim::UpdateInfo &info, gz::sim::EntityComponentManager &ecm) override {
    if(!ready || info.paused) return;
    double t=std::chrono::duration<double>(info.simTime).count();
    double dt=std::chrono::duration<double>(info.dt).count();
    if(dt<=0) return;
    std::array<double,2> goal{requested[0].load(),requested[1].load()};
    if(exercise) {
      if(t<2) goal={0.,0.}; else if(t<4) goal={.15,0.};
      else if(t<6) goal={.15,.15}; else if(t<8) goal={.3,.25}; else goal={.1,.1};
    }
    for(size_t i=0;i<2;i++) {
      if(velocityMode && !exercise) {
        auto seq=sequence[i].load();
        if(seq!=seen[i]) {seen[i]=seq;lastCommand[i]=t;}
        const double speed=t-lastCommand[i]<.25?velocity[i].load():0.;
        // Zero/stale velocity holds the integrated position through the force servo.
        goal[i]=std::clamp(target[i]+speed*dt,0.,i==0?1.95:.78);
      }
      target[i]+=std::clamp(goal[i]-target[i],-.3*dt,.3*dt);
      const auto *q=ecm.Component<gz::sim::components::JointPosition>(motors[i]);
      const auto *v=ecm.Component<gz::sim::components::JointVelocity>(motors[i]);
      if(!q || !v || q->Data().empty() || v->Data().empty()) continue;
      torque[i]=std::clamp(40.*(target[i]-q->Data()[0])-.6*v->Data()[0],-10.,10.);
      ecm.Component<gz::sim::components::JointForceCmd>(motors[i])->Data()={torque[i]};
    }
  }
  void PostUpdate(const gz::sim::UpdateInfo &info, const gz::sim::EntityComponentManager &ecm) override {
    if(!ready || info.paused) return;
    double t=std::chrono::duration<double>(info.simTime).count();
    if(t<nextLog) return;
    nextLog=t+.01;
    const auto a=gz::sim::worldPose(big,ecm),b=gz::sim::worldPose(small,ecm);
    const auto pa=a.Pos()+a.Rot().RotateVector(anchorBig),pb=b.Pos()+b.Rot().RotateVector(anchorSmall);
    auto za=a.Rot().RotateVector(gz::math::Vector3d::UnitZ),zb=b.Rot().RotateVector(gz::math::Vector3d::UnitZ);
    const double gap=(pa-pb).Length(),angle=std::acos(std::clamp(std::abs(za.Dot(zb)),0.,1.));
    gz::msgs::Vector3d msg;msg.set_x(gap);msg.set_y(angle);msg.set_z(t);metrics.Publish(msg);
    if(!csv) return;
    csv<<t<<','<<gap<<','<<angle;
    for(auto j:motors) { auto q=ecm.Component<gz::sim::components::JointPosition>(j);csv<<','<<(q && !q->Data().empty()?q->Data()[0]:NAN); }
    for(auto j:motors) { auto v=ecm.Component<gz::sim::components::JointVelocity>(j);csv<<','<<(v && !v->Data().empty()?v->Data()[0]:NAN); }
    csv<<','<<target[0]<<','<<target[1]<<','<<torque[0]<<','<<torque[1]<<'\n';
    csv.flush();
  }
};
}
GZ_ADD_PLUGIN(cad_arm::Closure,gz::sim::System,cad_arm::Closure::ISystemConfigure,
 cad_arm::Closure::ISystemPreUpdate,cad_arm::Closure::ISystemPostUpdate)
GZ_ADD_PLUGIN_ALIAS(cad_arm::Closure,"cad_arm::Closure")
