#include "x650_nmpc/objective.hpp"
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

extern "C" int x650_endpoint(int,const double*,const double*,const double*,
                              const double*,const double*,double*,char*,int);
namespace x650_nmpc {
namespace {
using Matrix=Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic,Eigen::RowMajor>;
using R3=Eigen::Matrix<double,3,3,Eigen::RowMajor>;
using V2=Eigen::Vector2d;
using V3=Eigen::Vector3d;
using V5=Eigen::Matrix<double,5,1>;
void require(bool ok,const char* message) {
  if(!ok) throw std::invalid_argument(message);
}
template<class T> bool finite(const T& values) {
  return std::all_of(values.begin(),values.end(),[](double v){return std::isfinite(v);});
}
template<std::size_t N> auto vec(const std::array<double,N>& a) {
  return Eigen::Map<const Eigen::Matrix<double,N,1>>(a.data());
}
R3 rotation(const x650::R9& a) {
  R3 r=Eigen::Map<const R3>(a.data());
  require(r.allFinite()&&(r.transpose()*r-R3::Identity()).cwiseAbs().maxCoeff()<=1e-6&&
          std::abs(r.determinant()-1.)<=1e-6,"objective rotation contract");
  return r;
}
R3 skew(const V3& v) {
  R3 a;a<<0,-v.z(),v.y(),v.z(),0,-v.x(),-v.y(),v.x(),0;return a;
}
R3 rotvec(const V3& v) {
  double angle=v.norm();
  if(angle<1e-8){R3 k=skew(v);return R3::Identity()+k+.5*k*k;}
  return Eigen::AngleAxisd(angle,v/angle).toRotationMatrix();
}
R3 thrust_rotation(const V3& a,double yaw) {
  constexpr double gravity=9.8;
  V3 f=a+V3(0,0,gravity);double norm=f.norm();
  require(norm>=gravity*.25&&f.z()>gravity*.25,"objective upright thrust domain");
  V3 z=f/norm,xc(std::cos(yaw),std::sin(yaw),0),cross=z.cross(xc);
  require(cross.norm()>=.25,"objective heading singularity");
  V3 y=cross/cross.norm();R3 r;r.col(0)=y.cross(z);r.col(1)=y;r.col(2)=z;return r;
}
V3 body_tip(const V2& q,const x650::Config& c) {
  const auto& g=c.geometry;double a=g[2]+q[0],b=g[3]+q.sum();
  return V3(g[4],g[5],g[6])+g[0]*V3(std::cos(a),0,-std::sin(a))+
         V3(g[7],g[8],g[9])+g[1]*V3(std::cos(b),0,-std::sin(b));
}
struct Nominal { V3 position; V2 joints; V5 command; };
Nominal nominal(const x650::Config& c,const x650::Anchor& anchor,
                const x650::Target& target) {
  const auto& g=c.geometry;const auto r=rotation(anchor.rotation);
  const V2 qa=vec(anchor.q);const V3 tip0=body_tip(qa,c);
  const V3 tipw0=vec(anchor.position)+r*tip0;
  V3 delta=r.transpose()*(vec(target.position)-tipw0);delta[1]=0.;
  V3 goal=tip0+c.arm_share*delta-V3(g[4],g[5],g[6])-V3(g[7],g[8],g[9]);
  require(std::abs(goal[1])<=1e-8,"objective nominal outside arm plane");
  double cosine=(goal.x()*goal.x()+goal.z()*goal.z()-g[0]*g[0]-g[1]*g[1])/(2*g[0]*g[1]);
  require(std::abs(cosine)<=1+1e-10,"objective nominal unreachable");
  V2 qn;double best=std::numeric_limits<double>::infinity();const double pi=std::acos(-1.);
  for(int sign:{-1,1}) {
    double beta=sign*std::acos(std::clamp(cosine,-1.,1.));
    double theta=std::atan2(-goal.z(),goal.x())-
      std::atan2(g[1]*std::sin(beta),g[0]+g[1]*std::cos(beta));
    V2 q(theta-g[2],beta+g[2]-g[3]);
    for(int j=0;j<2;++j){double v=std::fmod(q[j]-qa[j]+pi,2*pi);if(v<0)v+=2*pi;q[j]=qa[j]+v-pi;}
    double distance=(q-qa).norm();if(distance<best){best=distance;qn=q;}
  }
  require((qn.array()>=vec(c.physical_lower).array()).all()&&
          (qn.array()<=vec(c.physical_upper).array()).all(),"objective nominal physical limit");
  Eigen::Matrix<double,3,2> j;
  V3 t1(-std::sin(g[2]+qn[0]),0,-std::cos(g[2]+qn[0]));
  V3 t2(-std::sin(g[3]+qn.sum()),0,-std::cos(g[3]+qn.sum()));
  j.col(0)=g[0]*t1+g[1]*t2;j.col(1)=g[1]*t2;
  Eigen::Matrix2d jxz;jxz.row(0)=j.row(0);jxz.row(1)=j.row(2);
  require(std::abs(jxz.determinant())>1e-14,"objective nominal IK singular");
  const V3 vb=c.arm_share*r.transpose()*vec(target.velocity);
  const V2 dqn=jxz.partialPivLu().solve(V2(vb.x(),vb.z()));
  V5 un;un<<vec(target.velocity)-r*j*dqn,dqn;
  return {vec(target.position)-r*body_tip(qn,c),qn,un};
}
}

Objective evaluate_objective(const x650::Config& c,const x650::Memory& memory,
                             const x650::StepInput& in,const x650::StepResult& candidate) {
  const auto& d=candidate.diagnostics;const std::size_t n=std::size_t(c.nodes);
  require(c.nodes>=2&&c.nodes<=60,"objective node count");
  require(memory.initialized&&finite(memory.anchor.position)&&finite(memory.anchor.q)&&
          finite(memory.previous_command),"objective original committed memory");
  require(!memory.has_last_t||in.t>memory.last_t,"objective requires pre-step committed memory");
  require(std::isfinite(in.t)&&in.t>=0&&std::isfinite(in.dt)&&in.dt>0&&
          in.dt<=c.max_dt+1e-10&&std::isfinite(c.node_dt)&&c.node_dt>0,"objective time contract");
  require(finite(c.geometry)&&c.geometry[0]>0&&c.geometry[1]>0&&finite(c.response)&&
          (vec(c.response).array()>0).all()&&finite(c.weights)&&
          (vec(c.weights).array()>=0).all()&&finite(c.velocity_limit)&&
          (vec(c.velocity_limit).array()>0).all()&&finite(c.acceleration_limit)&&
          (vec(c.acceleration_limit).array()>0).all()&&std::isfinite(c.arm_share),"objective configuration finite/scales");
  require(finite(c.physical_lower)&&finite(c.physical_upper)&&
          (vec(c.physical_lower).array()<vec(c.physical_upper).array()).all(),"objective physical joint bounds");
  require(finite(in.state.acceleration)&&finite(in.state.omega_body),"objective attitude state finite");
  require(d.accepted&&d.nonlinear_passed&&d.actual_bounds,"objective requires native validated candidate");
  require(d.qp_columns==int(n*5)&&d.qp_rows>0&&d.horizon.size()==n&&d.plan.size()==n*5&&
          d.predicted_states.size()==n*12&&d.predicted_tip.size()==n*3&&d.predicted_rotations.size()==n*9&&
          d.linearized_states.size()==n*12&&!d.A.empty()&&!d.b.empty()&&
          d.A.size()==d.b.size()*n*5&&!d.candidate_packets.empty()&&
          !d.query_times.empty()&&d.candidate_states.size()==d.query_times.size()*4,
          "objective requires complete full diagnostics");
  require(finite(d.horizon)&&finite(d.plan)&&finite(d.predicted_states)&&finite(d.predicted_tip)&&
          finite(d.predicted_rotations)&&finite(d.candidate_states)&&finite(d.query_times)&&
          finite(candidate.command),"objective candidate finite values");
  require(d.hold_guard.required==(c.command_hold_s!=.05)&&
          (!d.hold_guard.required||(d.hold_guard.checked&&d.hold_guard.passed)),
          "objective held-command check did not pass");
  for(int j=0;j<5;++j)require(candidate.command[j]==d.plan[j],"objective first command differs from plan");
  for(std::size_t k=1;k<d.query_times.size();++k)
    require(d.query_times[k]>d.query_times[k-1],"objective native query time order");
  rotation(memory.anchor.rotation);const R3 r0=rotation(in.state.rotation);
  double yaw=std::atan2(r0(1,0),r0(0,0)),roll=std::atan2(r0(2,1),r0(2,2));
  double pitch=std::asin(std::clamp(-r0(2,0),-1.,1.));
  require(std::abs(std::cos(pitch))>1e-10,"objective yaw-rate singularity");
  double yr=(std::sin(roll)*in.state.omega_body[1]+std::cos(roll)*in.state.omega_body[2])/std::cos(pitch);
  Eigen::AngleAxisd aa(thrust_rotation(vec(in.state.acceleration),yaw).transpose()*r0);
  const V3 residual=aa.axis()*aa.angle();
  constexpr double tau=.15;
  Matrix contexts(n,20);double elapsed=0.;
  std::vector<x650::Target> targets=in.preview;
  require(targets.empty()||targets.size()==n,"objective preview node count");
  const bool fallback=targets.empty();if(fallback)targets.resize(n);
  for(std::size_t i=0;i<n;++i) {
    const double expected=i?c.node_dt:in.dt;
    require(std::abs(d.horizon[i]-expected)<=1e-12,"objective horizon differs from input");
    elapsed+=expected;double decay=std::exp(-elapsed/tau);
    R3 e=rotvec(decay*residual),ed=e*skew(-decay*residual/tau);
    contexts(i,0)=yaw+tau*(1-decay)*yr;contexts(i,1)=decay*yr;
    Eigen::Map<Matrix>(contexts.row(i).data()+2,3,3)=e;
    Eigen::Map<Matrix>(contexts.row(i).data()+11,3,3)=ed;
    if(fallback){targets[i]=in.target;for(int j=0;j<3;++j){
      targets[i].position[j]+=elapsed*in.target.velocity[j]+.5*elapsed*elapsed*in.target.acceleration[j];
      targets[i].velocity[j]+=elapsed*in.target.acceleration[j];}}
    require(finite(targets[i].position)&&finite(targets[i].velocity)&&finite(targets[i].acceleration),
            "objective preview finite");
    const auto at=std::lower_bound(d.query_times.begin(),d.query_times.end(),in.t+elapsed-1e-10);
    require(at!=d.query_times.end()&&std::abs(*at-in.t-elapsed)<=1e-9,"objective native node absent");
    const auto k=std::size_t(at-d.query_times.begin());
    for(int j=0;j<4;++j)require(std::abs(d.predicted_states[12*i+6+j]-d.candidate_states[4*k+j])<=1e-11,
                              "objective q/dq are not actual candidate node states");
  }
  std::vector<double> endpoint(n*15);char error[512]{};
  if(x650_endpoint(int(n),d.predicted_states.data(),d.plan.data(),contexts.data(),c.geometry.data(),
                   c.response.data(),endpoint.data(),error,512))throw std::runtime_error(error);
  require(finite(endpoint),"objective nonlinear endpoint nonfinite");
  Objective out;out.endpoint_positions.resize(n*3);out.endpoint_velocities.resize(n*3);
  const auto x=Eigen::Map<const Matrix>(d.predicted_states.data(),n,12);
  const auto u=Eigen::Map<const Matrix>(d.plan.data(),n,5);
  V5 previous=vec(memory.previous_command);
  for(std::size_t i=0;i<n;++i) {
    const V3 p=Eigen::Map<const V3>(endpoint.data()+15*i);
    const V3 v=Eigen::Map<const V3>(endpoint.data()+15*i+3);
    for(int j=0;j<3;++j){
      require(std::abs(p[j]-d.predicted_tip[3*i+j])<=1e-10,"objective endpoint differs from native diagnostics");
      out.endpoint_positions[3*i+j]=p[j];out.endpoint_velocities[3*i+j]=v[j];
    }
    for(int j=0;j<9;++j)require(std::abs(endpoint[15*i+6+j]-d.predicted_rotations[9*i+j])<=1e-10,
                              "objective rotation differs from native diagnostics");
    const auto nom=nominal(c,memory.anchor,targets[i]);const V5 command=u.row(i).transpose();
    const double h=d.horizon[i];
    std::array<double,6> norms{
      (p-vec(targets[i].position)).squaredNorm(),(v-vec(targets[i].velocity)).squaredNorm(),
      (x.row(i).head<3>().transpose()-nom.position).squaredNorm(),
      (x.row(i).segment<2>(6).transpose()-nom.joints).squaredNorm(),
      (command-nom.command).cwiseQuotient(vec(c.velocity_limit)).squaredNorm(),
      (command-previous).cwiseQuotient(h*vec(c.acceleration_limit)).squaredNorm()};
    for(int j=0;j<6;++j)out.components[j]+=h*c.weights[j]*norms[j];
    previous=command;
  }
  require(finite(out.components),"objective components nonfinite");
  for(double value:out.components)out.total+=value;
  require(std::isfinite(out.total),"objective total nonfinite");return out;
}
} // namespace x650_nmpc
