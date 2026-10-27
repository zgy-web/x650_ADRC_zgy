#include "x650_nmpc/objective.hpp"
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
extern "C" int ts20_prediction_batch(const double*,const double*,double,double,
  const double*,const double*,const double*,int,const double*,int,int,double*,int*,int*,
  double*,double*,int,int*,int*,char*,int);
namespace {
void check(bool v,const char* message){if(!v)throw std::runtime_error(message);}
void near(double a,double b,double tol,const char* message){check(std::abs(a-b)<=tol,message);}
template<class F>void rejects(F&& fn){bool rejected=false;try{fn();}catch(const std::exception&){rejected=true;}check(rejected,"expected objective rejection");}
x650::Config config(){x650::Config c;c.nodes=6;c.max_dt=.1;c.command_hold_s=.16;
  c.geometry={.2,.2,-.7853981633974483,.7853981633974483,0,0,0,0,0,0};
  c.physical_lower={-2,-2};c.physical_upper={2,2};c.response={4.,.8,4.,.8,.25};c.base_position_gain={1,1,1};
  c.motion={.0007447642530597398,.004194907941904997,.03470285366117535,.0004632811241788992,.003346379092576424,.075,.04,.2};return c;}
x650::StepInput input(){x650::StepInput in;in.t=in.dt=.04;in.state.stamp=in.state.control_time=in.state.joint_stamp=.04;
  in.target.position={.4*std::cos(.7853981633974483)+.003,0,0};in.target.velocity={.001,0,.0005};
  in.target.acceleration={.0002,0,0};in.context.reference_time=0;in.context.commands={{0,{0,0}}};
  for(int i=-20;i<=0;++i){x650::Packet p;p.send_time=i*.005;p.kp={6,6};p.kd={.25,.25};in.context.packets.push_back(p);}return in;}
void prime(x650::Core& core){core.reset({});core.prime({{0,{0,0,0,0,0}}},0);}
using V3=Eigen::Vector3d;
using M3=Eigen::Matrix3d;
M3 direct_thrust(const V3& acceleration,double heading){
  V3 z=acceleration+V3(0,0,9.8);z.normalize();
  V3 heading_axis(std::cos(heading),std::sin(heading),0),y=z.cross(heading_axis);y.normalize();
  M3 value;value.col(0)=y.cross(z);value.col(1)=y;value.col(2)=z;return value;
}
struct DirectContext {
  double yaw{},yaw_rate{};Eigen::Quaterniond residual;
  explicit DirectContext(const x650::State& state){
    M3 measured;for(int j=0;j<3;++j)for(int k=0;k<3;++k)measured(j,k)=state.rotation[3*j+k];
    yaw=std::atan2(measured(1,0),measured(0,0));
    const double roll=std::atan2(measured(2,1),measured(2,2));
    const double pitch=std::asin(-measured(2,0));
    yaw_rate=(std::sin(roll)*state.omega_body[1]+std::cos(roll)*state.omega_body[2])/std::cos(pitch);
    residual=Eigen::Quaterniond(direct_thrust(V3(state.acceleration.data()),yaw).transpose()*measured);
  }
  M3 at(double relative_time,const V3& acceleration)const{
    const double decay=std::exp(-relative_time/.15),heading=yaw+.15*(1-decay)*yaw_rate;
    // Quaternion interpolation is an independent formulation of the decaying
    // residual; this test never calls x650_endpoint or objective helpers.
    const M3 residual_rotation=Eigen::Quaterniond::Identity().slerp(decay,residual).toRotationMatrix();
    return direct_thrust(acceleration,heading)*residual_rotation;
  }
};
V3 direct_arm(const x650::Config& c,double q1,double q2){
  const auto& g=c.geometry;const double a=g[2]+q1,b=g[3]+q1+q2;
  return V3(g[4]+g[7]+g[0]*std::cos(a)+g[1]*std::cos(b),g[5]+g[8],
            g[6]+g[9]-g[0]*std::sin(a)-g[1]*std::sin(b));
}
double independent_all_components(const x650::Config& c,const x650::Memory& original,
                                 const x650::StepInput& in,const x650::StepResult& result,
                                 const x650_nmpc::Objective& actual){
  check(original.anchor.position==x650::V3{}&&original.anchor.q==x650::V2{}&&
        original.anchor.rotation==x650::R9{1,0,0,0,1,0,0,0,1},"independent fixture anchor changed");
  const auto& d=result.diagnostics;DirectContext context(in.state);
  std::array<double,6> expected{};double time=0.,largest_rotation_contribution=0.;
  auto previous=original.previous_command;
  const auto initial_tip=direct_arm(c,0,0);
  for(int i=0;i<c.nodes;++i){
    const double h=i?c.node_dt:in.dt;time+=h;
    const auto* x=d.predicted_states.data()+12*i;const auto* u=d.plan.data()+5*i;
    V3 position(x),velocity(x+3),acceleration(x[10],x[11],(u[2]-x[5])/c.response[4]);
    V3 jerk(c.response[0]*c.response[0]*(u[0]-x[3])-2*c.response[1]*c.response[0]*x[10],
            c.response[2]*c.response[2]*(u[1]-x[4])-2*c.response[3]*c.response[2]*x[11],
            -acceleration.z()/c.response[4]);
    const auto arm=direct_arm(c,x[6],x[7]);const auto rotation=context.at(time,acceleration);
    const V3 endpoint=position+rotation*arm;
    // Finite difference of actual nonlinear world position evolves p, q, a,
    // heading and the attitude residual simultaneously, including Rdot*r.
    constexpr double epsilon=1e-6;
    const V3 plus=position+epsilon*velocity+context.at(time+epsilon,acceleration+epsilon*jerk)*
      direct_arm(c,x[6]+epsilon*x[8],x[7]+epsilon*x[9]);
    const V3 minus=position-epsilon*velocity+context.at(time-epsilon,acceleration-epsilon*jerk)*
      direct_arm(c,x[6]-epsilon*x[8],x[7]-epsilon*x[9]);
    const V3 endpoint_velocity=(plus-minus)/(2*epsilon);
    const V3 arm_velocity=(direct_arm(c,x[6]+epsilon*x[8],x[7]+epsilon*x[9])-
                           direct_arm(c,x[6]-epsilon*x[8],x[7]-epsilon*x[9]))/(2*epsilon);
    largest_rotation_contribution=std::max(largest_rotation_contribution,
      (endpoint_velocity-(velocity+rotation*arm_velocity)).norm());
    V3 target_p,target_v;
    for(int j=0;j<3;++j){
      target_p[j]=in.target.position[j]+time*in.target.velocity[j]+.5*time*time*in.target.acceleration[j];
      target_v[j]=in.target.velocity[j]+time*in.target.acceleration[j];
      near(endpoint[j],actual.endpoint_positions[3*i+j],2e-12,"independent quaternion world position");
      near(endpoint_velocity[j],actual.endpoint_velocities[3*i+j],2e-8,"independent world-position derivative including Rdot*r");
    }
    // Analytic positive-elbow IK branch for this symmetric-arm fixture. It is
    // selected a priori from the known anchor, without copying the scorer's
    // two-branch search or reading its nominal quantities.
    const V3 goal=initial_tip+c.arm_share*(target_p-initial_tip);
    const double l1=c.geometry[0],l2=c.geometry[1];
    const double beta=std::acos((goal.x()*goal.x()+goal.z()*goal.z()-l1*l1-l2*l2)/(2*l1*l2));
    const double theta=std::atan2(-goal.z(),goal.x())-std::atan2(l2*std::sin(beta),l1+l2*std::cos(beta));
    const double qn1=theta-c.geometry[2],qn2=beta+c.geometry[2]-c.geometry[3];
    check(std::abs(qn1)<.1&&std::abs(qn2)<.1,"independent IK fixture left selected anchor branch");
    const double a=c.geometry[2]+qn1,b=c.geometry[3]+qn1+qn2;
    const double j00=-l1*std::sin(a)-l2*std::sin(b),j01=-l2*std::sin(b);
    const double j10=-l1*std::cos(a)-l2*std::cos(b),j11=-l2*std::cos(b);
    const double determinant=j00*j11-j01*j10;
    const double dqn1=c.arm_share*(target_v.x()*j11-j01*target_v.z())/determinant;
    const double dqn2=c.arm_share*(j00*target_v.z()-target_v.x()*j10)/determinant;
    const V3 nominal_position=target_p-direct_arm(c,qn1,qn2);
    const std::array<double,5> nominal_input{target_v.x()-j00*dqn1-j01*dqn2,target_v.y(),
      target_v.z()-j10*dqn1-j11*dqn2,dqn1,dqn2};
    expected[0]+=h*c.weights[0]*(endpoint-target_p).squaredNorm();
    expected[1]+=h*c.weights[1]*(endpoint_velocity-target_v).squaredNorm();
    expected[2]+=h*c.weights[2]*(position-nominal_position).squaredNorm();
    expected[3]+=h*c.weights[3]*((x[6]-qn1)*(x[6]-qn1)+(x[7]-qn2)*(x[7]-qn2));
    for(int j=0;j<5;++j){
      const double effort=(u[j]-nominal_input[j])/c.velocity_limit[j];
      const double slew=(u[j]-previous[j])/(c.acceleration_limit[j]*h);
      expected[4]+=h*c.weights[4]*effort*effort;expected[5]+=h*c.weights[5]*slew*slew;
      previous[j]=u[j];
    }
  }
  for(int j=0;j<6;++j)near(expected[j],actual.components[j],j==1?2e-9:2e-12,"independent six-component objective");
  return largest_rotation_contribution;
}
}
int main(){try{
  auto c=config();auto in=input();x650::Core core(c,ts20_prediction_batch);prime(core);
  const auto original=core.memory();auto candidate=core.step(in);const auto committed=core.memory();
  const auto cost=x650_nmpc::evaluate_objective(c,original,in,candidate);
  independent_all_components(c,original,in,candidate,cost);
  check(cost.total>0&&cost.endpoint_positions.size()==std::size_t(c.nodes*3)&&
        cost.endpoint_velocities.size()==std::size_t(c.nodes*3),"objective outputs");
  double sum=0;for(double component:cost.components){check(component>=0,"nonnegative component");sum+=component;}
  near(sum,cost.total,1e-15,"component sum");
  // Independent direct world FK and physical slew arithmetic. These use neither
  // the nonlinear evaluator nor its endpoint/normalization helpers.
  double endpoint_cost=0,slew_cost=0,time=0;auto previous=original.previous_command;
  const auto& d=candidate.diagnostics;
  for(int i=0;i<c.nodes;++i){double h=d.horizon[i];time+=h;
    double q1=d.predicted_states[12*i+6],q2=d.predicted_states[12*i+7];
    double a=c.geometry[2]+q1,b=c.geometry[3]+q1+q2;
    double arm[3]={c.geometry[4]+c.geometry[7]+c.geometry[0]*std::cos(a)+c.geometry[1]*std::cos(b),
      c.geometry[5]+c.geometry[8],c.geometry[6]+c.geometry[9]-c.geometry[0]*std::sin(a)-c.geometry[1]*std::sin(b)};
    for(int j=0;j<3;++j){double position=d.predicted_states[12*i+j];
      for(int k=0;k<3;++k)position+=d.predicted_rotations[9*i+3*j+k]*arm[k];
      near(position,cost.endpoint_positions[3*i+j],1e-12,"direct FK differs");
      double target=in.target.position[j]+time*in.target.velocity[j]+.5*time*time*in.target.acceleration[j];
      endpoint_cost+=h*c.weights[0]*(position-target)*(position-target);
    }
    for(int j=0;j<5;++j){double value=d.plan[5*i+j];double scaled=(value-previous[j])/(c.acceleration_limit[j]*h);
      slew_cost+=h*c.weights[5]*scaled*scaled;previous[j]=value;}
  }
  near(endpoint_cost,cost.components[0],1e-13,"nonlinear position cost");
  near(slew_cost,cost.components[5],1e-13,"physical slew normalization");
  auto separate=c;
  for(int active=0;active<6;++active){separate.weights.fill(0);separate.weights[active]=c.weights[active];
    auto one=x650_nmpc::evaluate_objective(separate,original,in,candidate);
    near(one.total,cost.components[active],1e-13,"cost component separability");}
  // Explicitly supplied preview must match the constant-acceleration fallback.
  auto explicit_input=in;time=0;
  for(double h:d.horizon){time+=h;auto t=in.target;for(int j=0;j<3;++j){t.position[j]+=time*t.velocity[j]+.5*time*time*t.acceleration[j];t.velocity[j]+=time*t.acceleration[j];}explicit_input.preview.push_back(t);}
  auto explicit_cost=x650_nmpc::evaluate_objective(c,original,explicit_input,candidate);
  near(explicit_cost.total,cost.total,1e-15,"preview fallback mismatch");
  check(core.memory().step_count==committed.step_count&&core.memory().plan==committed.plan&&
        core.memory().previous_command==committed.previous_command,"scoring changed Core memory");
  rejects([&]{x650_nmpc::evaluate_objective(c,committed,in,candidate);});
  auto bad=candidate;bad.diagnostics.actual_bounds=false;
  rejects([&]{x650_nmpc::evaluate_objective(c,original,in,bad);});
  bad=candidate;bad.diagnostics.hold_guard.passed=false;
  rejects([&]{x650_nmpc::evaluate_objective(c,original,in,bad);});
  bad=candidate;bad.diagnostics.A.clear();
  rejects([&]{x650_nmpc::evaluate_objective(c,original,in,bad);});
  bad=candidate;bad.diagnostics.predicted_tip[0]+=.001;
  rejects([&]{x650_nmpc::evaluate_objective(c,original,in,bad);});
  bad=candidate;bad.diagnostics.predicted_states[6]+=.001;
  rejects([&]{x650_nmpc::evaluate_objective(c,original,in,bad);});
  bad=candidate;bad.diagnostics.plan[0]=std::numeric_limits<double>::quiet_NaN();
  rejects([&]{x650_nmpc::evaluate_objective(c,original,in,bad);});
  // Nonzero attitude/acceleration/rotation-rate exercises residual context and
  // predicted velocity, while remaining a fresh synthetic raw-input Core solve.
  auto tilted=input();tilted.t=tilted.dt=.031;tilted.state.stamp=tilted.state.control_time=tilted.state.joint_stamp=.031;
  Eigen::Matrix3d r=(Eigen::AngleAxisd(.01,Eigen::Vector3d::UnitZ())*
    Eigen::AngleAxisd(-.005,Eigen::Vector3d::UnitY())*Eigen::AngleAxisd(.004,Eigen::Vector3d::UnitX())).toRotationMatrix();
  for(int j=0;j<3;++j)for(int k=0;k<3;++k)tilted.state.rotation[3*j+k]=r(j,k);
  tilted.state.acceleration={.01,-.005,.002};tilted.state.omega_body={.001,-.002,.003};
  x650::Core rotated(c,ts20_prediction_batch);prime(rotated);auto before=rotated.memory();auto result=rotated.step(tilted);
  auto rotated_cost=x650_nmpc::evaluate_objective(c,before,tilted,result);
  check(std::isfinite(rotated_cost.total)&&rotated_cost.total>0,"tilted exact objective");
  check(result.diagnostics.horizon[0]==.031&&result.diagnostics.horizon[1]==.04,"nonuniform first interval fixture");
  const double rotation_velocity=independent_all_components(c,before,tilted,result,rotated_cost);
  check(rotation_velocity>1e-4,"velocity fixture did not exercise Rdot*r");
  std::cout<<"PASS nonlinear objective: native endpoint agreement, independent six costs and world-position derivative including Rdot*r, nonuniform dt=.031, original memory, preview and fail-closed evidence\n";return 0;
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
