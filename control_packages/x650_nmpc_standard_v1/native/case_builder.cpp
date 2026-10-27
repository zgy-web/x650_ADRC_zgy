#include "case_builder.hpp"
#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <stdexcept>

namespace x650_standard { namespace {
using V2=Eigen::Vector2d;using V3=Eigen::Vector3d;using V4=Eigen::Vector4d;
using V5=Eigen::Matrix<double,5,1>;using R3=Eigen::Matrix<double,3,3,Eigen::RowMajor>;
constexpr double gravity=9.8,attitude_decay=.15,friction_width=.05;
void require(bool ok,const char* message){if(!ok)throw std::invalid_argument(message);}
template<class T>bool finite(const T& values){return std::all_of(values.begin(),values.end(),[](double x){return std::isfinite(x);});}
template<std::size_t N>auto vec(const std::array<double,N>& a){return Eigen::Map<const Eigen::Matrix<double,N,1>>(a.data());}
template<class T>void put(double* destination,const T& value){std::copy(value.data(),value.data()+value.size(),destination);}
void validate(const x650::Config& c){
 require(c.nodes==15&&c.node_dt==.04&&c.max_dt==.1&&c.command_hold_s==.16&&c.bus_period==.005,
         "standard OCP requires bound 15-node/40ms/max100ms/hold160ms/bus5ms configuration");
 require(finite(c.geometry)&&finite(c.response)&&finite(c.motion)&&finite(c.weights)&&finite(c.velocity_limit)&&
         finite(c.acceleration_limit)&&finite(c.q_lower)&&finite(c.q_upper)&&finite(c.physical_lower)&&
         finite(c.physical_upper)&&finite(c.kp)&&finite(c.kd)&&finite(c.base_position_gain)&&
         std::isfinite(c.integration_step)&&std::isfinite(c.arm_share)&&std::isfinite(c.joint_position_gain)&&
         std::isfinite(c.feasibility_tolerance)&&std::isfinite(c.kkt_tolerance),"standard OCP nonfinite configuration");
 require((vec(c.velocity_limit).array()>0).all()&&(vec(c.acceleration_limit).array()>0).all()&&
         (vec(c.response).array()>0).all()&&(vec(c.kp).array()>0).all()&&(vec(c.kd).array()>=0).all()&&
         (vec(c.weights).array()>=0).all()&&c.geometry[0]>0&&c.geometry[1]>0&&c.motion[0]>0&&
         c.motion[1]>=0&&c.motion[2]>=0&&c.motion[5]>=0&&c.motion[6]>=0&&c.motion[7]>0&&
         c.motion[4]>0&&c.motion[4]<c.bus_period,"standard OCP physical scales or delay");
 require((vec(c.q_lower).array()<vec(c.q_upper).array()).all()&&
         (vec(c.physical_lower).array()<=vec(c.q_lower).array()).all()&&
         (vec(c.physical_upper).array()>=vec(c.q_upper).array()).all()&&
         std::abs(c.velocity_limit[3]-.03-.42)<1e-14&&std::abs(c.velocity_limit[4]-.03-.32)<1e-14,
         "standard OCP original physical and prediction inner bounds");
 double slope=(c.motion[2]+c.motion[6])/friction_width+std::sqrt(2/std::exp(1.))*c.motion[6]/c.motion[7];
 double step=c.bus_period/10.;double screen=step*(c.motion[1]+std::max(c.kd[0],c.kd[1])+slope+
                                             std::max(c.kp[0],c.kp[1])*step)/c.motion[0];
 require(screen<=2.,"standard smooth ODE integration stiffness screen");
}
R3 rotation(const x650::R9& value){
 R3 R=Eigen::Map<const R3>(value.data());
 require(R.allFinite()&&(R.transpose()*R-R3::Identity()).cwiseAbs().maxCoeff()<=1e-6&&
         std::abs(R.determinant()-1)<=1e-6,"standard FLU to ENU rotation");return R;
}
R3 skew(const V3& v){R3 s;s<<0,-v.z(),v.y(),v.z(),0,-v.x(),-v.y(),v.x(),0;return s;}
R3 rotvec(const V3& v){double a=v.norm();R3 K=skew(v);
 if(a<1e-8)return R3::Identity()+K+.5*K*K;
 return R3::Identity()+std::sin(a)/a*K+(1-std::cos(a))/(a*a)*K*K;}
V3 rotation_log(const R3& R){double angle=std::acos(std::clamp((R.trace()-1)/2,-1.,1.));
 V3 v(R(2,1)-R(1,2),R(0,2)-R(2,0),R(1,0)-R(0,1));if(angle<1e-8)return .5*v;
 require(std::acos(-1.)-angle>=1e-5,"standard near-pi attitude residual unsupported");
 return angle/(2*std::sin(angle))*v;}
R3 thrust_rotation(const V3& a,double yaw){V3 f=a+V3(0,0,gravity);double norm=f.norm();
 require(norm>=gravity*.25&&f.z()>gravity*.25,"standard upright attitude domain");
 V3 z=f/norm,y=z.cross(V3(std::cos(yaw),std::sin(yaw),0));double yn=y.norm();
 require(yn>=.25,"standard heading singularity");y/=yn;R3 R;R.col(0)=y.cross(z);R.col(1)=y;R.col(2)=z;return R;}
V3 body_tip(const V2& q,const x650::Config& c){const auto& g=c.geometry;double a=g[2]+q[0],b=g[3]+q.sum();
 return V3(g[4]+g[7],g[5]+g[8],g[6]+g[9])+g[0]*V3(std::cos(a),0,-std::sin(a))+
        g[1]*V3(std::cos(b),0,-std::sin(b));}
struct Nominal{V3 position;V2 q;V5 control;};
Nominal nominal(const x650::Config& c,const x650::Anchor& anchor,const x650::Target& target){
 require(finite(anchor.q)&&finite(anchor.position)&&finite(target.position)&&finite(target.velocity),"standard nominal input finite");
 R3 R=rotation(anchor.rotation);V2 qa=vec(anchor.q);V3 pa=vec(anchor.position),tp=vec(target.position),tv=vec(target.velocity);
 const auto& g=c.geometry;V3 tip0=body_tip(qa,c),delta=R.transpose()*(tp-pa-R*tip0);delta[1]=0.;
 V3 goal=tip0+c.arm_share*delta-V3(g[4]+g[7],g[5]+g[8],g[6]+g[9]);
 require(std::abs(goal[1])<=1e-8,"standard nominal arm plane");
 double cosine=(goal.x()*goal.x()+goal.z()*goal.z()-g[0]*g[0]-g[1]*g[1])/(2*g[0]*g[1]);
 require(std::abs(cosine)<=1+1e-10,"standard nominal unreachable");
 V2 qn;double best=std::numeric_limits<double>::infinity(),pi=std::acos(-1.);
 for(int sign:{-1,1}){double beta=sign*std::acos(std::clamp(cosine,-1.,1.));
  double theta=std::atan2(-goal.z(),goal.x())-std::atan2(g[1]*std::sin(beta),g[0]+g[1]*std::cos(beta));
  V2 q(theta-g[2],beta+g[2]-g[3]);
  for(int j=0;j<2;++j){double v=std::fmod(q[j]-qa[j]+pi,2*pi);if(v<0)v+=2*pi;q[j]=qa[j]+v-pi;}
  double distance=(q-qa).norm();if(distance<best){best=distance;qn=q;}
 }
 require((qn.array()>=vec(c.physical_lower).array()).all()&&(qn.array()<=vec(c.physical_upper).array()).all(),"standard nominal physical bounds");
 double a=g[2]+qn[0],b=g[3]+qn.sum();V3 j2=g[1]*V3(-std::sin(b),0,-std::cos(b));
 Eigen::Matrix<double,3,2> J;J.col(0)=g[0]*V3(-std::sin(a),0,-std::cos(a))+j2;J.col(1)=j2;
 Eigen::Matrix2d Jxz;Jxz.row(0)=J.row(0);Jxz.row(1)=J.row(2);
 require(std::abs(Jxz.determinant())>=1e-14,"standard nominal IK singular");
 V3 vb=c.arm_share*R.transpose()*tv;V2 dqn=Jxz.partialPivLu().solve(V2(vb.x(),vb.z()));V5 un;un<<tv-R*J*dqn,dqn;
 return {tp-R*body_tip(qn,c),qn,un};
}
V2 reference(const x650::StepInput& in,const x650::Config& c){
 const auto& ctx=in.context;double at=ctx.reference_time,last=-std::numeric_limits<double>::infinity();
 require(std::isfinite(at)&&finite(ctx.reference)&&!ctx.commands.empty()&&ctx.commands.size()<=64&&
         at<=in.t+1e-10&&ctx.commands.front().time<=at+1e-10,"standard actual reference history coverage");
 V2 r=vec(ctx.reference),active=vec(ctx.commands.front().velocity);
 for(const auto& cmd:ctx.commands){require(std::isfinite(cmd.time)&&cmd.time>last&&cmd.time<=in.t+1e-10&&finite(cmd.velocity)&&
  (vec(cmd.velocity).cwiseAbs().array()<=(vec(c.velocity_limit).tail<2>().array()+1e-10)).all(),"standard actual command history");last=cmd.time;}
 for(const auto& cmd:ctx.commands){if(cmd.time<=at+1e-10){active=vec(cmd.velocity);continue;}
  if(cmd.time>in.t)break;
  r+=(cmd.time-at)*active;at=cmd.time;active=vec(cmd.velocity);}
 return r+(in.t-at)*active;
}
struct Applied{double sent,time;V2 force;};
std::vector<Applied> packets(const x650::StepInput& in,const x650::Config& c){
 const auto& source=in.context.packets;require(!source.empty()&&source.size()<=64,"standard actual packet history required");
 std::vector<Applied> result;double last=-std::numeric_limits<double>::infinity();
 for(const auto& packet:source){require(std::isfinite(packet.send_time)&&packet.send_time>last&&packet.send_time<=in.t+1e-10&&
   finite(packet.q_ref)&&finite(packet.dq_ref)&&finite(packet.kp)&&finite(packet.kd)&&finite(packet.tau_ff)&&
   packet.kp==c.kp&&packet.kd==c.kd,"standard actual packet order/gains/finite");
  if(!result.empty()){double ticks=(packet.send_time-result.front().sent)/c.bus_period;
   require(std::abs(ticks-std::round(ticks))<=1e-7,"standard actual packet bus grid changed");}
  V2 force=vec(packet.kp).cwiseProduct(vec(packet.q_ref))+vec(packet.kd).cwiseProduct(vec(packet.dq_ref))+vec(packet.tau_ff);
  result.push_back({packet.send_time,packet.send_time+c.motion[4],force});last=packet.send_time;
 }return result;
}
V4 rhs(const V4& joint,const V2& force,const x650::Config& c){V4 out;out.head<2>()=joint.tail<2>();
 for(int j=0;j<2;++j){double v=joint[2+j],friction=(c.motion[2]+c.motion[6]*std::exp(-std::pow(v/c.motion[7],2)))*std::tanh(v/friction_width);
  out[2+j]=(force[j]-c.kp[j]*joint[j]-(c.motion[1]+c.kd[j])*v-c.motion[3]-friction)/c.motion[0];}
 return out;}
V4 rk4(V4 joint,const V2& force,double duration,const x650::Config& c){
 constexpr int steps=20;double h=duration/steps;
 for(int k=0;k<steps;++k){V4 a=rhs(joint,force,c),b=rhs(joint+h*a/2,force,c),d=rhs(joint+h*b/2,force,c),e=rhs(joint+h*d,force,c);joint+=h*(a+2*b+2*d+e)/6;}
 return joint;}
struct Aligned{V4 joint;V2 active;std::optional<Applied> pending;double age;};
Aligned aligned_joint(const x650::StepInput& in,const x650::Config& c,const std::vector<Applied>& source){
 double age=in.state.control_time-in.state.joint_stamp,start=in.t-age;
 require(std::isfinite(age)&&age>=0&&age<=.025+1e-10&&finite(in.state.q)&&finite(in.state.dq),"standard original joint state/age");
 std::optional<Applied> eligible;for(const auto& p:source)if(p.time<=start+1e-10)eligible=p;
 require(bool(eligible),"standard actual delayed packet does not cover joint measurement");
 V4 joint;joint<<vec(in.state.q),vec(in.state.dq);V2 active=eligible->force;double cursor=start;
 auto advance=[&](double duration){int parts=std::max(1,int(std::ceil(duration/c.bus_period)));
  for(int k=0;k<parts;++k)joint=rk4(joint,active,duration/parts,c);};
 for(const auto& packet:source)if(packet.time>start+1e-10&&packet.time<=in.t+1e-10){advance(packet.time-cursor);cursor=packet.time;active=packet.force;}
 advance(std::max(0.,in.t-cursor));std::optional<Applied> pending;
 for(const auto& p:source)if(p.time>in.t+1e-10){require(!pending,"standard multiple pending actual packets");pending=p;}
 return {joint,active,pending,age};
}
std::pair<V5,V5> first_bounds(const x650::Config& c,const x650::Memory& m,const x650::StepInput& in,const Aligned& aligned){
 V5 previous=vec(m.previous_command),limit=vec(c.velocity_limit),accel=vec(c.acceleration_limit);
 V2 slew=accel.tail<2>()*in.dt,vmin=vec(in.state.dq).cwiseMin(aligned.joint.tail<2>()).cwiseMin(previous.tail<2>()-slew),
     vmax=vec(in.state.dq).cwiseMax(aligned.joint.tail<2>()).cwiseMax(previous.tail<2>()+slew);
 for(const auto& cmd:in.context.commands)if(cmd.time>=in.t-c.command_hold_s-aligned.age-c.max_dt){vmin=vmin.cwiseMin(vec(cmd.velocity));vmax=vmax.cwiseMax(vec(cmd.velocity));}
 V2 qgl=aligned.joint.head<2>()+(c.command_hold_s+aligned.age)*vmin.cwiseMin(V2::Zero()),
    qgh=aligned.joint.head<2>()+(c.command_hold_s+aligned.age)*vmax.cwiseMax(V2::Zero());
 require((qgl.array()>=vec(c.q_lower).array()-1e-8).all()&&(qgh.array()<=vec(c.q_upper).array()+1e-8).all(),"standard pending history stopping envelope infeasible");
 V5 lo=(-limit).cwiseMax(previous-accel*in.dt),hi=limit.cwiseMin(previous+accel*in.dt);
 for(int j=0;j<2;++j){double a=c.acceleration_limit[3+j],reserve=a*c.max_dt;
  lo[3+j]=std::max(lo[3+j],-(std::sqrt(reserve*reserve+2*a*std::max(qgl[j]-c.q_lower[j],0.))-reserve));
  hi[3+j]=std::min(hi[3+j],std::sqrt(reserve*reserve+2*a*std::max(c.q_upper[j]-qgh[j],0.))-reserve);}
 require((lo.array()<=hi.array()+1e-10).all(),"standard first command history bounds infeasible");return {lo,hi};
}
std::optional<double> schedule(double* p,double start,double h,double first_future,const x650::Config& c,std::optional<double> pending){
 double end=start+h;int index=std::max(0,int(std::ceil((start-first_future)/c.bus_period-1e-9)));
 std::vector<double> sends;while(first_future+index*c.bus_period<end-1e-10){double send=first_future+index*c.bus_period;
  if(send>=start-1e-10)sends.push_back(send);
  ++index;require(index<1000,"standard schedule index overflow");}
 struct Event{double at,isnew,ispending,relative;};std::vector<Event> events;
 if(pending){require(*pending>=start-1e-9&&*pending<=end+1e-9,"standard pending packet outside stage");events.push_back({std::max(start,*pending),0,1,0});}
 for(double send:sends)if(send+c.motion[4]<=end+1e-10)events.push_back({std::min(end,send+c.motion[4]),1,0,std::max(0.,send-start)});
 std::stable_sort(events.begin(),events.end(),[](const auto& a,const auto& b){return a.at<b.at;});
 double cursor=start;int count=0;
 auto add=[&](double duration,double isnew,double ispending,double relative){
  require(count<event_slots&&duration<=c.bus_period+1e-9,"standard schedule capacity/integration interval");
  p[parameter::durations+count]=duration;p[parameter::apply_new+count]=isnew;p[parameter::apply_pending+count]=ispending;
  p[parameter::send_relative+count]=relative;p[parameter::sample_mask+count]=1.;++count;};
 for(const auto& e:events){require(e.at>=cursor-1e-10,"standard schedule event order");add(std::max(0.,e.at-cursor),e.isnew,e.ispending,e.relative);cursor=e.at;}
 if(end-cursor>1e-10||count==0)add(std::max(0.,end-cursor),0,0,0);
 std::optional<double> next;
 for(double send:sends)if(send+c.motion[4]>end+1e-10){require(!next,"standard multiple future pending packets");next=send;
  p[parameter::pending_at_end]=1.;p[parameter::pending_send_relative]=send-start;}
 return next?std::optional<double>(*next+c.motion[4]):std::nullopt;
}
} // namespace

NumericalCase build_case(const x650::Config& c,const x650::Memory& m,const x650::StepInput& in){
 validate(c);require(m.initialized&&m.history_primed&&m.has_origin,"standard primed original committed memory required");
 require(std::isfinite(in.t)&&in.t>=0&&std::isfinite(in.dt)&&in.dt>0&&in.dt<=c.max_dt+1e-10&&
  std::isfinite(in.state.control_time)&&std::isfinite(m.origin)&&std::abs(in.state.control_time-in.t-m.origin)<=1e-8&&
  (!m.has_last_t||in.t>m.last_t),"standard original control time/dt/epoch inconsistent");
 require(finite(m.previous_command)&&finite(in.state.position)&&finite(in.state.velocity)&&finite(in.state.acceleration)&&
         finite(in.state.omega_body),"standard original feedback finite");
 auto source=packets(in,c);auto aligned=aligned_joint(in,c,source);V2 r=reference(in,c);
 NumericalCase out;put(out.x0.data(),vec(in.state.position));put(out.x0.data()+3,vec(in.state.velocity));
 put(out.x0.data()+6,aligned.joint);std::copy_n(in.state.acceleration.begin(),2,out.x0.data()+10);
 put(out.x0.data()+12,r);put(out.x0.data()+14,aligned.active);if(aligned.pending)put(out.x0.data()+16,aligned.pending->force);
 put(out.x0.data()+18,vec(m.previous_command));out.horizon.assign(horizon_nodes,c.node_dt);out.horizon[0]=in.dt;
 std::vector<x650::Target> targets=in.preview;require(targets.empty()||targets.size()==horizon_nodes,"standard preview dimension");
 if(targets.empty()){require(finite(in.target.position)&&finite(in.target.velocity)&&finite(in.target.acceleration),"standard fallback target finite");
  double elapsed=0.;for(double h:out.horizon){elapsed+=h;auto target=in.target;
   for(int j=0;j<3;++j){target.position[j]+=elapsed*in.target.velocity[j]+.5*elapsed*elapsed*in.target.acceleration[j];target.velocity[j]+=elapsed*in.target.acceleration[j];}targets.push_back(target);}}
 R3 R=rotation(in.state.rotation);double yaw=std::atan2(R(1,0),R(0,0)),roll=std::atan2(R(2,1),R(2,2)),pitch=std::asin(std::clamp(-R(2,0),-1.,1.));
 require(std::abs(std::cos(pitch))>=1e-6,"standard yaw rate singularity");
 double yr=(std::sin(roll)*in.state.omega_body[1]+std::cos(roll)*in.state.omega_body[2])/std::cos(pitch);
 V3 residual=rotation_log(thrust_rotation(vec(in.state.acceleration),yaw).transpose()*R);
 double origin=in.context.bus_origin;require(std::isfinite(origin),"standard original bus origin nonfinite");
 double first_future=origin+std::ceil((in.t-origin)/c.bus_period-1e-9)*c.bus_period;
 if(first_future<=source.back().sent+1e-10)first_future+=c.bus_period;
 require(first_future>=in.t-1e-10,"standard future packet before current time");
 std::optional<double> pending=aligned.pending?std::optional<double>(aligned.pending->time):std::nullopt;
 auto bounds=first_bounds(c,m,in,aligned);out.parameters.assign((horizon_nodes+1)*parameter_dim,0.);double elapsed=0.;
 for(int k=0;k<horizon_nodes;++k){double* p=out.parameters.data()+k*parameter_dim;double h=out.horizon[k];
  pending=schedule(p,in.t+elapsed,h,first_future,c,pending);elapsed+=h;double decay=std::exp(-elapsed/attitude_decay);
  R3 E=rotvec(decay*residual),Ed=E*skew(-decay*residual/attitude_decay);
  p[parameter::h]=h;p[parameter::attitude_context]=yaw+attitude_decay*(1-decay)*yr;p[parameter::attitude_context+1]=decay*yr;
  put(p+parameter::attitude_context+2,E);put(p+parameter::attitude_context+11,Ed);
  require(finite(targets[k].position)&&finite(targets[k].velocity),"standard preview target finite");
  put(p+parameter::target_position,vec(targets[k].position));put(p+parameter::target_velocity,vec(targets[k].velocity));
  auto nom=nominal(c,m.anchor,targets[k]);put(p+parameter::nominal_position,nom.position);put(p+parameter::nominal_q,nom.q);put(p+parameter::nominal_control,nom.control);
  for(int j=0;j<2;++j)p[parameter::stopping_reserve+j]=c.velocity_limit[3+j]/(2*c.acceleration_limit[3+j])+c.max_dt+c.command_hold_s+aligned.age;
  for(int j=0;j<5;++j){p[parameter::control_lower+j]=k? -c.velocity_limit[j]:bounds.first[j];p[parameter::control_upper+j]=k?c.velocity_limit[j]:bounds.second[j];}
 }
 std::copy_n(out.parameters.data()+(horizon_nodes-1)*parameter_dim,parameter_dim,out.parameters.data()+horizon_nodes*parameter_dim);
 out.initial_plan.resize(horizon_nodes*input_dim);
 for(int k=0;k<horizon_nodes;++k)std::copy(m.previous_command.begin(),m.previous_command.end(),out.initial_plan.data()+k*input_dim);
 if(!m.plan.empty()){
  require(m.plan.size()==horizon_nodes*input_dim&&m.plan_h.size()==horizon_nodes&&finite(m.plan)&&finite(m.plan_h)&&
   std::all_of(m.plan_h.begin(),m.plan_h.end(),[](double h){return h>0;})&&std::isfinite(m.plan_t),"standard original committed plan");
  std::vector<double> ends(horizon_nodes);double sum=0.;for(int k=0;k<horizon_nodes;++k){sum+=m.plan_h[k];ends[k]=sum;}
  double offset=0.;for(int k=0;k<horizon_nodes;++k){double at=in.t-m.plan_t+offset;auto index=std::min(int(std::upper_bound(ends.begin(),ends.end(),at)-ends.begin()),horizon_nodes-1);
   std::copy_n(m.plan.data()+index*input_dim,input_dim,out.initial_plan.data()+k*input_dim);offset+=out.horizon[k];}
 }
 require(finite(out.x0)&&finite(out.parameters)&&finite(out.initial_plan),"standard restored smooth OCP nonfinite");return out;
}
} // namespace x650_standard
