// Bullet Featherstone's frame acceleration is zero in the installed backend.
// Supply physical translational acceleration from successive world velocities.
// This runs after Physics::Update and before Imu::PostUpdate. Gazebo's IMU
// continues to apply gravity subtraction, sensor noise, orientation and rate.
#include <gz/plugin/RegisterMore.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/Imu.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/sim/components/LinearAcceleration.hh>
#include <gz/sim/components/LinearVelocity.hh>
#include <gz/sim/components/Pose.hh>

namespace cad_arm {
class ImuAcceleration : public gz::sim::System,
 public gz::sim::ISystemConfigure, public gz::sim::ISystemConfigurePriority,
 public gz::sim::ISystemUpdate {
  gz::sim::Entity base{};
  gz::math::Vector3d previous;
  bool initialized=false;
 public:
  gz::sim::System::PriorityType ConfigurePriority() override {return 1;}
  void Configure(const gz::sim::Entity &entity,const std::shared_ptr<const sdf::Element> &,
    gz::sim::EntityComponentManager &ecm,gz::sim::EventManager &) override {
    base=gz::sim::Model(entity).LinkByName(ecm,"base_link");
    gz::sim::Link(base).EnableVelocityChecks(ecm);
  }
  void Update(const gz::sim::UpdateInfo &info,gz::sim::EntityComponentManager &ecm) override {
    if(info.paused) return;
    auto v=gz::sim::Link(base).WorldLinearVelocity(ecm);
    const double dt=std::chrono::duration<double>(info.dt).count();
    if(!v || dt<=0) {initialized=false;return;}
    auto acceleration=initialized?(*v-previous)/dt:gz::math::Vector3d::Zero;
    previous=*v;initialized=true;
    ecm.Each<gz::sim::components::Imu,gz::sim::components::ParentEntity>(
      [&](const gz::sim::Entity &e,const gz::sim::components::Imu *,const gz::sim::components::ParentEntity *p) {
        if(p->Data()!=base)return true;
        // Supported contract: IMU at base_link origin (as in the PX4 sensor template).
        auto local=ecm.Component<gz::sim::components::Pose>(e);
        if(local && local->Data().Pos().Length()>1e-9)return true;
        auto a=ecm.Component<gz::sim::components::LinearAcceleration>(e);
        if(a)a->Data()=gz::sim::worldPose(e,ecm).Rot().RotateVectorReverse(acceleration);
        return true;
      });
  }
};
}
GZ_ADD_PLUGIN(cad_arm::ImuAcceleration,gz::sim::System,
 cad_arm::ImuAcceleration::ISystemConfigure,cad_arm::ImuAcceleration::ISystemConfigurePriority,
 cad_arm::ImuAcceleration::ISystemUpdate)
GZ_ADD_PLUGIN_ALIAS(cad_arm::ImuAcceleration,"cad_arm::ImuAcceleration")
