// T1 same-physics-step observer. No position, effort or wrench commands.
// Gazebo 8 Joint::TransmittedWrench yields parent-on-child at joint origin,
// expressed in JOINT axes; mount_* is its negative (arm-on-base).
// The wrench is NOT expressed at the base origin or in world coordinates.
// Missing data are NaN, never silently substituted by zero.
#include <gz/sim/System.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Joint.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>
#include <gz/plugin/Register.hh>
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <limits>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

namespace wbbench {
namespace gs = gz::sim;
constexpr double NaN = std::numeric_limits<double>::quiet_NaN();
class WholeBodyRecorder : public gs::System, public gs::ISystemConfigure,
                   public gs::ISystemPreUpdate, public gs::ISystemPostUpdate {
  std::string modelName = "x500_underfold_0";
  gs::Entity model = gs::kNullEntity, base = gs::kNullEntity, distal = gs::kNullEntity;
  std::array<gs::Entity, 2> joints{};
  const std::array<std::string, 2> jointNames = {"arm_joint_1", "arm_joint_2"};
  std::ofstream out;
  double duration = 300., firstSim = NaN, lastSim = -1.;
  bool ready = false, done = false;
  std::size_t samples = 0;

  static std::string value(const std::string &path, const std::string &key) {
    std::ifstream in(path);
    if (!in) throw std::runtime_error("WholeBodyRecorder cannot read config: " + path);
    std::string line;
    while (std::getline(in, line)) {
      if (!line.empty() && line.back() == '\r') line.pop_back();
      auto split = line.find('=');
      if (split != std::string::npos && line.substr(0, split) == key)
        return line.substr(split + 1);
    }
    return "";
  }
  static bool finite(const std::optional<gz::math::Vector3d> &v) {
    return v && std::isfinite(v->X()) && std::isfinite(v->Y()) &&
                std::isfinite(v->Z());
  }
  void vector(const std::optional<gz::math::Vector3d> &v) {
    out << ',' << (v ? v->X() : NaN)
        << ',' << (v ? v->Y() : NaN)
        << ',' << (v ? v->Z() : NaN);
  }
  static double first(const std::optional<std::vector<double>> &v) {
    return (v && !v->empty()) ? v->at(0) : NaN;
  }
public:
  void Configure(const gs::Entity &,
                 const std::shared_ptr<const sdf::Element> &sdf,
                 gs::EntityComponentManager &, gs::EventManager &) override {
    if (!sdf || !sdf->HasElement("config_path"))
      throw std::runtime_error("WholeBodyRecorder requires config_path");
    const auto config = sdf->Get<std::string>("config_path");
    const auto output = value(config, "output");
    const auto name = value(config, "model");
    const auto length = value(config, "duration");
    if (!name.empty()) modelName = name;
    if (!length.empty()) duration = std::stod(length);
    if (!std::isfinite(duration) || duration <= 0. || duration > 300.)
      throw std::runtime_error("WholeBodyRecorder duration must be in (0, 300] seconds");
    if (output.empty()) throw std::runtime_error("WholeBodyRecorder requires output");
    out.open(output, std::ios::out | std::ios::trunc);
    if (!out) throw std::runtime_error("WholeBodyRecorder cannot open output: " + output);
    out << std::setprecision(17)
        << "sim_s,step_dt_s,valid_base,valid_accel,"
        << "base_px,base_py,base_pz,base_qw,base_qx,base_qy,base_qz,"
        << "base_vx,base_vy,base_vz,base_wx,base_wy,base_wz,"
        << "base_ax,base_ay,base_az,base_alphax,base_alphay,base_alphaz,"
        << "arm_joint_1_q,arm_joint_1_qd,arm_joint_2_q,arm_joint_2_qd,"
        << "valid_joint_state,valid_mount_wrench,"
        << "mount_fx,mount_fy,mount_fz,mount_tx,mount_ty,mount_tz,tip_x,tip_y,tip_z,valid_tip\n";
    out.flush();
  }

  void PreUpdate(const gs::UpdateInfo &info,
                 gs::EntityComponentManager &ecm) override {
    if (info.paused || done || ready) return;
    model = ecm.EntityByComponents(gs::components::Model(),
                                  gs::components::Name(modelName));
    if (model == gs::kNullEntity) return;
    gs::Model m(model);
    base = m.LinkByName(ecm, "base_link"); distal = m.LinkByName(ecm, "arm_link_2");
    for (std::size_t i = 0; i < joints.size(); ++i)
      joints[i] = m.JointByName(ecm, jointNames[i]);
    if (base == gs::kNullEntity || distal == gs::kNullEntity || joints[0] == gs::kNullEntity ||
        joints[1] == gs::kNullEntity)
      throw std::runtime_error("WholeBodyRecorder model missing base_link or arm joints");
    // Enabling readback components asks Physics to report state; no commands.
    gs::Link(base).EnableVelocityChecks(ecm);
    gs::Link(distal).EnableVelocityChecks(ecm);
    gs::Link(base).EnableAccelerationChecks(ecm);
    for (const auto j : joints) {
      gs::Joint(j).EnablePositionCheck(ecm);
      gs::Joint(j).EnableVelocityCheck(ecm);
    }
    gs::Joint(joints[0]).EnableTransmittedWrenchCheck(ecm);
    ready = true;
    firstSim = std::chrono::duration<double>(info.simTime).count();
  }

  void PostUpdate(const gs::UpdateInfo &info,
                  const gs::EntityComponentManager &ecm) override {
    if (!ready || done || info.paused) return;
    const double sim = std::chrono::duration<double>(info.simTime).count();
    const double dt = std::chrono::duration<double>(info.dt).count();
    if (sim - firstSim > duration) {
      out.flush();
      done = true;
      return;
    }
    if (dt <= 0. || sim <= lastSim) return;
    lastSim = sim;
    const gs::Link link(base);
    const auto p = link.WorldPose(ecm);
    const auto v = link.WorldLinearVelocity(ecm);
    const auto w = link.WorldAngularVelocity(ecm);
    const auto a = link.WorldLinearAcceleration(ecm);
    const auto alpha = link.WorldAngularAcceleration(ecm);
    const bool validPose = p && std::isfinite(p->Pos().X()) &&
      std::isfinite(p->Pos().Y()) && std::isfinite(p->Pos().Z()) &&
      std::isfinite(p->Rot().W()) && std::isfinite(p->Rot().X()) &&
      std::isfinite(p->Rot().Y()) && std::isfinite(p->Rot().Z());
    const bool validBase = validPose && finite(v) && finite(w);
    out << sim << ',' << dt << ',' << int(validBase)
        << ',' << int(finite(a) && finite(alpha));
    if (p) {
      const auto &r = p->Pos();
      const auto &q = p->Rot();
      out << ',' << r.X() << ',' << r.Y() << ',' << r.Z()
          << ',' << q.W() << ',' << q.X() << ',' << q.Y() << ',' << q.Z();
    } else {
      for (int i = 0; i < 7; ++i) out << ',' << NaN;
    }
    vector(v); vector(w); vector(a); vector(alpha);
    bool validJoints = true;
    for (const auto j : joints) {
      const double q = first(gs::Joint(j).Position(ecm));
      const double qd = first(gs::Joint(j).Velocity(ecm));
      validJoints = validJoints && std::isfinite(q) && std::isfinite(qd);
      out << ',' << q << ',' << qd;
    }
    std::array<double, 6> mount;
    mount.fill(NaN);
    const auto transmitted = gs::Joint(joints[0]).TransmittedWrench(ecm);
    if (transmitted && !transmitted->empty()) {
      const auto &r = transmitted->at(0);
      mount = {-r.force().x(), -r.force().y(), -r.force().z(),
               -r.torque().x(), -r.torque().y(), -r.torque().z()};
    }
    const bool validMount = std::all_of(mount.begin(), mount.end(),
                             [](double x) { return std::isfinite(x); });
    out << ',' << int(validJoints) << ',' << int(validMount);
    for (const auto x : mount) out << ',' << x;
    const auto distalPose = gs::Link(distal).WorldPose(ecm);
    if (distalPose) {
      const auto tip = distalPose->Pos() + distalPose->Rot().RotateVector(gz::math::Vector3d(0.25, 0., 0.));
      out << ',' << tip.X() << ',' << tip.Y() << ',' << tip.Z() << ",1";
    } else out << ",nan,nan,nan,0";
    out << '\n';
    if (++samples % 100 == 0) out.flush();
  }
  ~WholeBodyRecorder() override { if (out) out.flush(); }
};
}
GZ_ADD_PLUGIN(wbbench::WholeBodyRecorder, gz::sim::System,
              gz::sim::ISystemConfigure, gz::sim::ISystemPreUpdate,
              gz::sim::ISystemPostUpdate)
GZ_ADD_PLUGIN_ALIAS(wbbench::WholeBodyRecorder, "wbbench::WholeBodyRecorder")
