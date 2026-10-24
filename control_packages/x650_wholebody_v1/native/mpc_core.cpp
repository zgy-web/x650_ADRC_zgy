#include "mpc_core.hpp"
#include <Eigen/Dense>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <numeric>

extern "C" {
int x650_assemble(int,const double*,int,double*,int,int*,char*,int);
int x650_solve(int,int,const double*,const double*,const double*,const double*,const double*,int,double*,double*,int*,char*,int);
int x650_endpoint(int,const double*,const double*,const double*,const double*,const double*,double*,char*,int);
int x650_candidate(int,int,const double*,const double*,const double*,const double*,const double*,const double*,const double*,const double*,double*,char*,int);
unsigned x650_mpc_core_abi_version(){return 1;}
}
namespace x650 {
namespace {
using Matrix=Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic,Eigen::RowMajor>;
using E2=Eigen::Vector2d; using E3=Eigen::Vector3d; using E5=Eigen::Matrix<double,5,1>;
using ER=Eigen::Matrix<double,3,3,Eigen::RowMajor>;
using Clock=std::chrono::steady_clock;
void require(bool ok,const char* message){if(!ok)throw CoreError(message);}
template<class T> bool finite(const T& values){return std::all_of(values.begin(),values.end(),[](double v){return std::isfinite(v);});}
template<std::size_t N> auto vec(const std::array<double,N>& a){return Eigen::Map<const Eigen::Matrix<double,N,1>>(a.data());}
template<std::size_t N,class Derived> std::array<double,N> arr(const Eigen::MatrixBase<Derived>& a){
 std::array<double,N> out{};require(a.size()==N,"array conversion size");for(std::size_t i=0;i<N;++i)out[i]=a.derived().coeff(i);return out;}
ER rotation(const R9& a){ER r=Eigen::Map<const ER>(a.data());require(r.allFinite()&&
 (r.transpose()*r-ER::Identity()).cwiseAbs().maxCoeff()<=1e-6&&std::abs(r.determinant()-1.)<=1e-6,"rotation contract");return r;}
double elapsed(Clock::time_point begin){return std::chrono::duration<double,std::milli>(Clock::now()-begin).count();}
template<class T> void append(std::vector<double>& out,const T& a){out.insert(out.end(),a.begin(),a.end());}
void append(std::vector<double>& out,double x){out.push_back(x);}
std::vector<double> cumulative(const std::vector<double>& v){std::vector<double> o;double s=0;for(double x:v){s+=x;o.push_back(s);}return o;}
void unique_sort(std::vector<double>& values){std::sort(values.begin(),values.end());values.erase(std::unique(values.begin(),values.end()),values.end());}
E3 tip(const V2& q,const Config& c){const auto& g=c.geometry;double a=g[2]+q[0],b=g[3]+q[0]+q[1];
 return E3(g[4],g[5],g[6])+g[0]*E3(std::cos(a),0,-std::sin(a))+E3(g[7],g[8],g[9])+g[1]*E3(std::cos(b),0,-std::sin(b));}
struct Nominal {E3 p,v,a;E2 q,dq,ddq;};
Nominal nominal(const Config& c,const Anchor& anchor,const Target& target){
 require(finite(target.position)&&finite(target.velocity)&&finite(target.acceleration),"target finite");
 ER R=rotation(anchor.rotation);const auto& g=c.geometry;E3 tip0=tip(anchor.q,c);
 E3 delta=R.transpose()*(vec(target.position)-vec(anchor.position)-R*tip0);delta.y()=0;
 E3 goal=tip0+c.arm_share*delta-E3(g[4],g[5],g[6])-E3(g[7],g[8],g[9]);
 require(std::abs(goal.y())<=1e-8,"nominal arm plane");
 double cosine=(goal.x()*goal.x()+goal.z()*goal.z()-g[0]*g[0]-g[1]*g[1])/(2*g[0]*g[1]);
 require(std::abs(cosine)<=1+1e-10,"nominal reachability");E2 q;double best=std::numeric_limits<double>::infinity();
 const double pi=std::acos(-1.);
 for(int sign:{-1,1}){double beta=sign*std::acos(std::clamp(cosine,-1.,1.));double theta=std::atan2(-goal.z(),goal.x())-std::atan2(g[1]*std::sin(beta),g[0]+g[1]*std::cos(beta));
  E2 candidate(theta-g[2],beta+g[2]-g[3]);for(int j=0;j<2;++j){double v=std::fmod(candidate[j]-anchor.q[j]+pi,2*pi);if(v<0)v+=2*pi;candidate[j]=anchor.q[j]+v-pi;}
  double d=(candidate-vec(anchor.q)).norm();if(d<best){best=d;q=candidate;}}
 require((q.array()>=vec(c.physical_lower).array()).all()&&(q.array()<=vec(c.physical_upper).array()).all(),"nominal physical bounds");
 double a=g[2]+q[0],b=g[3]+q.sum();Eigen::Matrix<double,3,2> J;
 J.col(1)=g[1]*E3(-std::sin(b),0,-std::cos(b));J.col(0)=g[0]*E3(-std::sin(a),0,-std::cos(a))+J.col(1);
 Eigen::Matrix2d Jxz;Jxz.row(0)=J.row(0);Jxz.row(1)=J.row(2);require(std::abs(Jxz.determinant())>1e-14,"nominal Jacobian singular");
 E3 vb=c.arm_share*(R.transpose()*vec(target.velocity));E2 dq=Jxz.partialPivLu().solve(E2(vb.x(),vb.z()));
 E3 centripetal=-g[0]*E3(std::cos(a),0,-std::sin(a))*dq[0]*dq[0]-g[1]*E3(std::cos(b),0,-std::sin(b))*dq.sum()*dq.sum();
 E3 ab=c.arm_share*(R.transpose()*vec(target.acceleration))-centripetal;E2 ddq=Jxz.partialPivLu().solve(E2(ab.x(),ab.z()));
 return {vec(target.position)-R*tip(arr<2>(q),c),vec(target.velocity)-R*J*dq,vec(target.acceleration)-R*(J*ddq+centripetal),q,dq,ddq};
}
E2 reference(const StepInput& in,const Config& c){
 const auto& ctx=in.context;require(std::isfinite(ctx.reference_time)&&finite(ctx.reference)&&std::isfinite(ctx.bus_origin),"reference finite");
 require(ctx.reference_time<=in.t+1e-10&&!ctx.commands.empty()&&ctx.commands.size()<=64&&ctx.commands.front().time<=ctx.reference_time+1e-10,"reference coverage");
 double last=-std::numeric_limits<double>::infinity();
 for(const auto& item:ctx.commands){require(std::isfinite(item.time)&&item.time>last&&item.time<=in.t+1e-10&&finite(item.velocity),"actual command history time");
  require((vec(item.velocity).cwiseAbs().array()<=vec(c.velocity_limit).tail<2>().array()+1e-10).all(),"actual command speed");last=item.time;}
 // Validate the entire history before integration can stop at an epsilon-close
 // future record. No later unsent/nonfinite/out-of-order record may be hidden.
 double at=ctx.reference_time;E2 r=vec(ctx.reference),active=vec(ctx.commands.front().velocity);
 for(const auto& item:ctx.commands){
  if(item.time<=at+1e-10){active=vec(item.velocity);continue;}if(item.time>in.t)break;
  r+=(item.time-at)*active;at=item.time;active=vec(item.velocity);}
 return r+(in.t-at)*active;
}
double quantize(double value,double step){double scaled=value/step;require(std::isfinite(scaled)&&std::abs(scaled)<=std::pow(2.,52)-1,"quantization exact-integer range");return std::copysign(std::floor(std::abs(scaled)+.5),scaled)*step;}
struct PacketPlan {std::vector<Packet> packets;std::vector<Matrix>P,V;};
PacketPlan packets(const Config& c,const StepInput& in,const std::vector<double>& h,const std::vector<double>& nodes,const Matrix& plan,E2 r,bool sensitivity){
 require(plan.rows()==c.nodes&&plan.cols()==5&&plan.allFinite(),"packet plan shape/finite");
 for(int j=0;j<2;++j)require((plan.col(3+j).cwiseAbs().array()<=c.velocity_limit[3+j]+1e-10).all(),"packet plan joint velocity bound");
 PacketPlan out;out.packets=in.context.packets;int n=5*c.nodes;require(!out.packets.empty()&&out.packets.size()<=64,"actual packets missing or capacity");
 double last=-std::numeric_limits<double>::infinity();for(const auto& p:out.packets){require(std::isfinite(p.send_time)&&p.send_time>last&&p.send_time<=in.t+1e-10&&finite(p.q_ref)&&finite(p.dq_ref)&&finite(p.tau_ff)&&p.kp==c.kp&&p.kd==c.kd,"actual packet contract");last=p.send_time;}
 if(sensitivity){out.P.assign(out.packets.size(),Matrix::Zero(2,n));out.V=out.P;}
 std::vector<double> starts{0.};starts.insert(starts.end(),nodes.begin(),nodes.end()-1);
 std::vector<E2> integral{E2::Zero()};std::vector<Matrix> prefix{Matrix::Zero(2,n)};
 for(int i=0;i<c.nodes;++i){integral.push_back(integral.back()+h[i]*plan.row(i).tail<2>().transpose());Matrix next=prefix.back();for(int j=0;j<2;++j)next(j,5*i+3+j)=h[i]*c.velocity_limit[3+j];prefix.push_back(std::move(next));}
 double first=in.context.bus_origin+std::ceil((in.t-in.context.bus_origin)/c.bus_period-1e-9)*c.bus_period;
 if(first<=last+1e-10)first+=c.bus_period;
 double stop=in.t+nodes.back()+1e-10;
 int count=std::max(0,int(std::ceil((stop-first)/c.bus_period)));require(count<=2100,"future packet capacity");
 // numpy.arange uses the representable first+step increment after length calculation.
 double increment=(first+c.bus_period)-first;
 for(int b=0;b<count;++b){double send=first+b*increment,relative=send-in.t;
  int active=std::clamp(int(std::upper_bound(starts.begin(),starts.end(),relative+1e-10)-starts.begin())-1,0,c.nodes-1);
  double partial=relative-starts[active];E2 q=r+integral[active]+partial*plan.row(active).tail<2>().transpose();Packet p;p.send_time=send;p.kp=c.kp;p.kd=c.kd;
  for(int j=0;j<2;++j){p.q_ref[j]=quantize(q[j],2*std::acos(-1.)/std::pow(2.,24));p.dq_ref[j]=quantize(plan(active,3+j),1./512);}
  out.packets.push_back(p);if(sensitivity){Matrix P=prefix[active],V=Matrix::Zero(2,n);for(int j=0;j<2;++j){P(j,5*active+3+j)+=partial*c.velocity_limit[3+j];V(j,5*active+3+j)=c.velocity_limit[3+j];}out.P.push_back(std::move(P));out.V.push_back(std::move(V));}}
 double start=in.t-(in.state.control_time-in.state.joint_stamp);
 require(out.packets.front().send_time+c.motion[4]<=start+1e-10,"packet delay coverage");
 for(std::size_t i=1;i<out.packets.size();++i){double grid=(out.packets[i].send_time-out.packets.front().send_time)/c.bus_period;
  require(out.packets[i].send_time>out.packets[i-1].send_time&&std::abs(grid-std::nearbyint(grid))<=1e-7,"fixed packet bus grid");}
 return out;
}
struct Rollout {std::vector<double> boundaries;Matrix states,query_states,extrema;std::vector<int>modes,zeros;std::array<double,8>envelope{};int substeps=0;};
Rollout rollout(const Config& c,Ts20BatchFn fn,const StepInput& in,const std::vector<Packet>& pp,const std::vector<double>& queries,bool extrema,const Matrix* forcing_override=nullptr){
 Rollout out;double start=in.t-(in.state.control_time-in.state.joint_stamp);out.boundaries={start};append(out.boundaries,queries);
 std::vector<double>knots;Matrix forcing(pp.size(),2);for(std::size_t i=0;i<pp.size();++i){knots.push_back(pp[i].send_time+c.motion[4]);if(knots.back()>start&&knots.back()<=queries.back())out.boundaries.push_back(knots.back());
  for(int j=0;j<2;++j)forcing(i,j)=c.kp[j]*pp[i].q_ref[j]+c.kd[j]*pp[i].dq_ref[j]+pp[i].tau_ff[j];}
 unique_sort(out.boundaries);if(forcing_override)forcing=*forcing_override;
 out.states.resize(out.boundaries.size(),4);out.modes.resize(out.boundaries.size()*2);out.zeros.resize(out.boundaries.size());out.extrema.resize(4096,5);
 std::array<double,4> gains{c.kp[0],c.kp[1],c.kd[0],c.kd[1]},initial{in.state.q[0],in.state.q[1],in.state.dq[0],in.state.dq[1]};int event_count=0;char error[512]{};
 int code=fn(c.motion.data(),gains.data(),c.integration_step,start,initial.data(),knots.data(),forcing.data(),int(pp.size()),out.boundaries.data(),int(out.boundaries.size()),extrema?1:0,out.states.data(),out.modes.data(),out.zeros.data(),out.envelope.data(),out.extrema.data(),4096,&event_count,&out.substeps,error,512);
 require(!code,error);require(event_count>=0&&event_count<=4096,"native extrema capacity");out.extrema.conservativeResize(event_count,5);out.query_states.resize(queries.size(),4);
 for(std::size_t i=0;i<queries.size();++i){auto at=std::lower_bound(out.boundaries.begin(),out.boundaries.end(),queries[i]);require(at!=out.boundaries.end()&&std::abs(*at-queries[i])<=1e-9,"propagation query coverage");out.query_states.row(i)=out.states.row(at-out.boundaries.begin());}
 require(out.states.allFinite()&&finite(out.envelope),"native propagation finite");return out;
}
std::pair<Eigen::Matrix2d,E2> transition(const Config& c,int j,double h){
 double k=c.kp[j]/c.motion[0],d=(c.motion[1]+c.kd[j])/c.motion[0],half=-d/2,disc=half*half-k,co=1,s=h;
 if(std::abs(disc)*h*h>=1e-14){double w=std::sqrt(std::abs(disc));co=disc>0?std::cosh(w*h):std::cos(w*h);s=(disc>0?std::sinh(w*h):std::sin(w*h))/w;}
 Eigen::Matrix2d A;A<<co-half*s,s,-k*s,co+half*s;A*=std::exp(half*h);E2 B;
 if(c.kp[j]>0)B<< (1-A(0,0))/c.kp[j],-A(1,0)/c.kp[j];else if(d>0){double gain=-std::expm1(-d*h)/d;B<<(h-gain)/(c.motion[1]+c.kd[j]),gain/c.motion[0];}else B<<h*h/(2*c.motion[0]),h/c.motion[0];return {A,B};
}
struct Prediction {Matrix joints,references;E2 q,dq,r;std::vector<double>queries;PacketPlan nominal;Rollout run;};
Prediction predict(const Config& c,Ts20BatchFn fn,const StepInput& in,const std::vector<double>& h,const std::vector<double>& nodes,const Matrix& warm,bool exact){
 Prediction p;p.r=reference(in,c);p.nominal=packets(c,in,h,nodes,warm,p.r,true);p.queries={in.t};for(double x:nodes)p.queries.push_back(in.t+x);
 for(const auto& v:p.nominal.packets){double applied=v.send_time+c.motion[4];if(applied>=in.t&&applied<=in.t+nodes.back())p.queries.push_back(applied);}unique_sort(p.queries);
 p.run=rollout(c,fn,in,p.nominal.packets,p.queries,false);int n=c.nodes*5,nq=int(p.queries.size());
 std::vector<Matrix> deriv(nq,Matrix::Zero(4,n));std::vector<Matrix>force;
 for(std::size_t i=0;i<p.nominal.packets.size();++i){Matrix f(2,n);for(int j=0;j<2;++j)f.row(j)=c.kp[j]*p.nominal.P[i].row(j)+c.kd[j]*p.nominal.V[i].row(j);force.push_back(std::move(f));}
 if(exact){Matrix base(force.size(),2);for(std::size_t i=0;i<force.size();++i)for(int j=0;j<2;++j){const auto& v=p.nominal.packets[i];base(i,j)=c.kp[j]*v.q_ref[j]+c.kd[j]*v.dq_ref[j]+v.tau_ff[j];}
  for(int column=0;column<n;++column){bool active=false;for(const auto& F:force)active=active||(F.col(column).array()!=0).any();if(!active)continue;
   Matrix plus=base,minus=base;for(std::size_t i=0;i<force.size();++i)for(int j=0;j<2;++j){plus(i,j)+=.0001*force[i](j,column);minus(i,j)-=.0001*force[i](j,column);}
   auto positive=rollout(c,fn,in,p.nominal.packets,p.queries,false,&plus);auto negative=rollout(c,fn,in,p.nominal.packets,p.queries,false,&minus);
   for(int i=0;i<nq;++i)deriv[i].col(column)=((positive.query_states.row(i)-negative.query_states.row(i))/.0002).transpose();}
 }else{Matrix S=Matrix::Zero(4,n);double cursor=in.t-(in.state.control_time-in.state.joint_stamp);int query=0;
  std::vector<double> knots;for(const auto& pp:p.nominal.packets)knots.push_back(pp.send_time+c.motion[4]);
  for(double end:p.run.boundaries){double duration=end-cursor;int packet=std::max(0,int(std::upper_bound(knots.begin(),knots.end(),cursor+1e-11)-knots.begin())-1);
   if(duration>0){for(int j=0;j<2;++j){auto ab=transition(c,j,duration);Matrix old(2,n);old.row(0)=S.row(j);old.row(1)=S.row(j+2);Matrix next=ab.first*old+ab.second*force[packet].row(j);S.row(j)=next.row(0);S.row(j+2)=next.row(1);}}
   if(query<nq&&std::abs(p.queries[query]-end)<1e-9){deriv[query++]=S;}
   cursor=end;}require(query==nq,"sensitivity query coverage");}
 Eigen::VectorXd z(n);for(int i=0;i<c.nodes;++i)for(int j=0;j<5;++j)z[i*5+j]=warm(i,j)/(j<3?1.:c.velocity_limit[j]);
 std::vector<int> indices;for(int k=-1;k<c.nodes;++k){double relative=k<0?0:nodes[k];int at=0;while(at<nq&&std::abs((p.queries[at]-in.t)-relative)>1e-9)++at;require(at<nq,"relative node lookup");indices.push_back(at);}
 Matrix affine(nq,4);for(int i=0;i<nq;++i)affine.row(i)=p.run.query_states.row(i)-(deriv[i]*z).transpose();
 Eigen::Vector4d now=affine.row(indices[0]).transpose()+deriv[indices[0]]*z;p.q=now.head<2>();p.dq=now.tail<2>();
 p.joints.resize(c.nodes,4+4*n);p.references.resize(c.nodes,2+2*n);Matrix R=Matrix::Zero(2,n);E2 r=p.r;
 for(int i=0;i<c.nodes;++i){int k=indices[i+1];p.joints.row(i).head(4)=affine.row(k);std::copy(deriv[k].data(),deriv[k].data()+4*n,p.joints.row(i).data()+4);
  for(int j=0;j<2;++j){R(j,5*i+3+j)=h[i]*c.velocity_limit[3+j];}
  r+=h[i]*warm.row(i).tail<2>().transpose();p.references.row(i).head(2)=(r-R*z).transpose();std::copy(R.data(),R.data()+2*n,p.references.row(i).data()+2);}
 return p;
}
std::vector<double> matrix_data(const Matrix& x){return {x.data(),x.data()+x.size()};}
}

Core::Core(Config config,Ts20BatchFn predictor,DiagnosticMode diagnostics):config_(std::move(config)),predictor_(predictor),diagnostic_mode_(diagnostics){
 const auto& c=config_;require(predictor_&&c.nodes>=2&&c.nodes<=60&&c.max_iterations>=1&&c.max_iterations<=10000,"core configuration dimensions");
 require(finite(c.geometry)&&finite(c.response)&&finite(c.motion)&&finite(c.weights)&&finite(c.velocity_limit)&&finite(c.acceleration_limit)&&finite(c.q_lower)&&finite(c.q_upper)&&finite(c.physical_lower)&&finite(c.physical_upper)&&finite(c.base_position_gain)&&finite(c.kp)&&finite(c.kd),"core configuration finite");
 require(c.geometry[0]>0&&c.geometry[1]>0&&c.node_dt>0&&c.max_dt>0&&c.node_dt*(c.nodes-1)+c.max_dt<=2&&std::isfinite(c.node_dt)&&std::isfinite(c.max_dt)&&c.arm_share>=0&&c.arm_share<=1&&std::isfinite(c.joint_position_gain)&&c.joint_position_gain>0,"core horizon/geometry/gains");
 require(c.feasibility_tolerance==2e-7&&c.kkt_tolerance==2e-3,"core frozen quality policy");
 for(int i=0;i<5;++i)require(c.velocity_limit[i]>0&&c.acceleration_limit[i]>0&&c.response[i]>0,"core positive dynamics/limits");
 for(int i=0;i<3;++i)require(c.base_position_gain[i]>0,"core PX4 gain");
 for(int j=0;j<2;++j)require(c.q_lower[j]>=c.physical_lower[j]&&c.q_upper[j]<=c.physical_upper[j]&&c.q_lower[j]<c.q_upper[j]&&c.kp[j]>=0&&c.kd[j]>=0,"core joint bounds/gains");
 for(double w:c.weights){require(w>=0,"core nonnegative weights");}require(c.weights[0]>0&&c.weights[5]>0,"core positive weights");
 require(c.motion[0]>0&&c.motion[1]>=0&&c.motion[2]>=0&&c.motion[4]>=0&&c.motion[4]<=.25&&c.motion[6]>=0&&c.motion[7]>0&&c.motion[5]>=c.motion[2]+c.motion[6]&&c.bus_period>=.001&&c.bus_period<=.01&&c.integration_step>=1e-5&&c.integration_step<=.001&&c.trust_radius>0&&c.trust_radius<=.25,"core TS20 parameters");
}
void Core::reset(const Anchor& anchor){require(finite(anchor.position)&&finite(anchor.q),"anchor finite");rotation(anchor.rotation);
 for(int j=0;j<2;++j){require(anchor.q[j]>=config_.q_lower[j]&&anchor.q[j]<=config_.q_upper[j],"anchor soft bounds");}memory_=Memory{};memory_.anchor=anchor;memory_.initialized=true;}
void Core::restore(const Memory& memory){require(memory.initialized&&finite(memory.previous_command)&&finite(memory.plan)&&finite(memory.plan_h),"restored memory finite/initialized");
 require((memory.plan.empty()&&memory.plan_h.empty())||(memory.plan.size()==std::size_t(config_.nodes*5)&&memory.plan_h.size()==std::size_t(config_.nodes)),"restored plan dimensions");
 for(double h:memory.plan_h){require(h>0,"restored horizon positive");}rotation(memory.anchor.rotation);require(finite(memory.anchor.position)&&finite(memory.anchor.q),"restored anchor finite");
 require((!memory.has_last_t||std::isfinite(memory.last_t))&&(!memory.has_stamp||std::isfinite(memory.previous_stamp))&&(!memory.has_origin||std::isfinite(memory.origin))&&std::isfinite(memory.plan_t),"restored memory clocks");memory_=memory;}
void Core::prime(const std::vector<Command5>& history,double origin){require(memory_.initialized&&!memory_.history_primed&&!memory_.step_count&&std::isfinite(origin)&&origin>=0&&!history.empty()&&history.size()<=64,"prime lifecycle");
 double last=-std::numeric_limits<double>::infinity();std::vector<Command> selected;
 for(const auto& h:history){require(std::isfinite(h.time)&&h.time>=0&&h.time>last&&h.time<=origin+1e-9&&finite(h.velocity),"prime time/finite");for(int j=0;j<5;++j)require(std::abs(h.velocity[j])<=config_.velocity_limit[j]+1e-9,"prime speed");last=h.time;
  if(h.time-origin>=-.75)selected.push_back({h.time-origin,{h.velocity[3],h.velocity[4]}});}
 require(last-origin>=-.25,"prime recent command");memory_.joint_history=std::move(selected);memory_.previous_command=history.back().velocity;memory_.last_t=last-origin;memory_.has_last_t=true;memory_.origin=origin;memory_.has_origin=true;memory_.history_primed=true;}

StepResult Core::step(const StepInput& in){const auto begin=Clock::now();Diagnostics dg;
 try {const auto& c=config_;const auto& m=memory_;const auto& s=in.state;int N=c.nodes,n=N*5;
  require(m.initialized&&std::isfinite(in.t)&&in.t>=0&&std::isfinite(in.dt)&&in.dt>0&&in.dt<=c.max_dt+1e-10&&(!m.has_last_t||in.t>m.last_t),"increasing control time/dt");
  require(finite(s.position)&&finite(s.velocity)&&finite(s.omega_body)&&finite(s.acceleration)&&finite(s.q)&&finite(s.dq),"state finite");rotation(s.rotation);
  require(std::isfinite(s.stamp)&&s.stamp>=0&&(!m.has_stamp||s.stamp>=m.previous_stamp-1e-9)&&std::isfinite(s.control_time)&&std::isfinite(s.joint_stamp),"state timestamps");
  double age=s.control_time-s.joint_stamp;require(age>=0&&age<=.25+1e-10&&(!m.has_origin||std::abs(s.control_time-in.t-m.origin)<=1e-8),"source age/origin");
  Nominal ref=nominal(c,m.anchor,in.target);for(int j=0;j<2;++j)require(ref.q[j]>=c.q_lower[j]&&ref.q[j]<=c.q_upper[j]&&std::abs(ref.dq[j])<=c.velocity_limit[3+j]+1e-9&&std::abs(ref.ddq[j])<=c.acceleration_limit[3+j]+1e-9,"nominal joint limits");
  for(int j=0;j<3;++j)require(std::abs(ref.v[j])<=c.velocity_limit[j]+1e-9&&std::abs(ref.a[j])<=c.acceleration_limit[j]+1e-9,"nominal base limits");
  std::vector<double>h(N,c.node_dt);h[0]=in.dt;auto nodes=cumulative(h);Matrix warm(N,5);
  if(m.plan.empty())for(int i=0;i<N;++i)warm.row(i)=vec(m.previous_command).transpose();else {auto ends=cumulative(m.plan_h);double offset=0;for(int i=0;i<N;++i){double sample=in.t-m.plan_t+offset;int k=std::min(int(std::upper_bound(ends.begin(),ends.end(),sample)-ends.begin()),N-1);for(int j=0;j<5;++j)warm(i,j)=m.plan[k*5+j];offset+=h[i];}}
  require(in.preview.empty()||in.preview.size()==std::size_t(N),"preview node count");std::vector<Target> targets=in.preview;
  if(targets.empty())for(double offset:nodes){Target t=in.target;for(int j=0;j<3;++j){t.position[j]+=offset*in.target.velocity[j]+.5*offset*offset*in.target.acceleration[j];t.velocity[j]+=offset*in.target.acceleration[j];}targets.push_back(t);}
  for(const auto& t:targets)require(finite(t.position)&&finite(t.velocity)&&finite(t.acceleration),"preview finite");
  for(int attempt=0;attempt<3;++attempt){dg.subproblems=attempt+1;Prediction pred=predict(c,predictor_,in,h,nodes,warm,attempt>0);
   std::vector<double> data;data.reserve(100000);append(data,std::array<double,5>{in.dt,c.node_dt,c.max_dt,age,std::min(c.trust_radius,.02)});
   append(data,c.weights);append(data,c.velocity_limit);append(data,c.acceleration_limit);append(data,m.previous_command);append(data,c.q_lower);append(data,c.q_upper);
   append(data,s.position);append(data,s.velocity);append(data,arr<2>(pred.q));append(data,arr<2>(pred.dq));append(data,V2{s.acceleration[0],s.acceleration[1]});
   append(data,s.rotation);append(data,s.omega_body);append(data,s.acceleration);append(data,c.response);append(data,c.geometry);append(data,m.anchor.position);append(data,m.anchor.rotation);append(data,m.anchor.q);append(data,c.arm_share);
   append(data,c.physical_lower);append(data,c.physical_upper);append(data,s.dq);append(data,arr<2>(pred.q));append(data,arr<2>(pred.dq));
   std::vector<Command>recent;for(const auto& v:in.context.commands)if(v.time>=in.t-.05-age-c.max_dt)recent.push_back(v);append(data,double(recent.size()));for(const auto& v:recent)append(data,v.velocity);
   append(data,matrix_data(warm));for(const auto& t:targets){append(data,t.position);append(data,t.velocity);}append(data,matrix_data(pred.joints));append(data,matrix_data(pred.references));
   std::vector<double> output(21*N*(n+1)+n*n+n+(15*N+5)*(n+2)+N*(53+12*n)+17);int shape[3]{};char error[512]{};
   if(x650_assemble(N,data.data(),int(data.size()),output.data(),int(output.size()),shape,error,512))throw CoreError(error,dg);
   int offset=0;auto take=[&](int size){require(offset+size<=shape[2],"assembly output shape");std::vector<double>v(output.begin()+offset,output.begin()+offset+size);offset+=size;return v;};int ar=shape[0],cr=shape[1];
   dg.A=take(ar*n);dg.b=take(ar);dg.H=take(n*n);dg.g=take(n);dg.C=take(cr*n);dg.lower=take(cr);dg.upper=take(cr);
   auto offsets=take(N*12),jac=take(N*12*n);take(N*12);take(N*9);auto contexts=take(N*20);take(17);require(offset==shape[2],"assembly complete output");
   dg.solution.resize(n);double quality[4]{};int mode=-1;
   if(x650_solve(n,cr,dg.H.data(),dg.g.data(),dg.C.data(),dg.lower.data(),dg.upper.data(),c.max_iterations,dg.solution.data(),quality,&mode,error,512))throw CoreError(error,dg);
   dg.primal_violation=quality[0];dg.kkt_stationarity=quality[1];dg.kkt_complementarity=quality[2];dg.active_constraints=int(quality[3]);dg.solver_mode=mode;dg.qp_rows=cr;dg.qp_columns=n;
   require(finite(dg.solution)&&quality[0]<=c.feasibility_tolerance&&quality[1]<=c.kkt_tolerance,"QP original-row/KKT rejection");
   Matrix plan(N,5),predicted(N,12);auto z=Eigen::Map<const Eigen::VectorXd>(dg.solution.data(),n);
   for(int i=0;i<N;++i){for(int j=0;j<5;++j)plan(i,j)=dg.solution[i*5+j]*c.velocity_limit[j];predicted.row(i)=(Eigen::Map<const Eigen::VectorXd>(offsets.data()+i*12,12)+Eigen::Map<const Matrix>(jac.data()+i*12*n,12,n)*z).transpose();}
   auto actual_packets=packets(c,in,h,nodes,plan,pred.r,false);auto actual=rollout(c,predictor_,in,actual_packets.packets,pred.queries,true);
   Matrix q(N,2),dq(N,2),references(N,2);E2 r=pred.r;for(int i=0;i<N;++i){int k=0;double distance=std::numeric_limits<double>::infinity();for(std::size_t j=0;j<pred.queries.size();++j){double d=std::abs(pred.queries[j]-(in.t+nodes[i]));if(d<distance){distance=d;k=int(j);}}
    q.row(i)=actual.query_states.row(k).head(2);dq.row(i)=actual.query_states.row(k).tail(2);r+=h[i]*plan.row(i).tail<2>().transpose();references.row(i)=r.transpose();}
   double candidate[4]{};if(x650_candidate(N,N,predicted.data(),q.data(),dq.data(),references.data(),actual.envelope.data(),c.q_lower.data(),c.q_upper.data(),c.velocity_limit.data()+3,candidate,error,512))throw CoreError(error,dg);
   dg.q_linearization_error=candidate[0];dg.dq_linearization_error=candidate[1];dg.actual_bounds=candidate[2]!=0;dg.nonlinear_passed=candidate[3]!=0;
   if(diagnostic_mode_==DiagnosticMode::Full){dg.nominal_packets=pred.nominal.packets;dg.candidate_packets=actual_packets.packets;dg.query_times=pred.queries;dg.nominal_states=matrix_data(pred.run.query_states);dg.candidate_states=matrix_data(actual.query_states);dg.candidate_extrema=matrix_data(actual.extrema);dg.candidate_boundary_times=actual.boundaries;dg.candidate_modes=actual.modes;dg.candidate_zero_events=actual.zeros;}
   dg.candidate_envelope=actual.envelope;dg.candidate_substeps=actual.substeps;dg.horizon=h;dg.plan=matrix_data(plan);
   dg.attempts.push_back({dg.nonlinear_passed,dg.actual_bounds,candidate[0],candidate[1],elapsed(begin)});
   if(!dg.nonlinear_passed){if(dg.actual_bounds&&attempt<2&&elapsed(begin)<50.){warm=plan;continue;}throw CoreError("TS20 nonlinear candidate rejected before command publication",dg);}
   predicted.middleCols(6,2)=q;predicted.middleCols(8,2)=dq;dg.predicted_states=matrix_data(predicted);std::vector<double>endpoint(N*15);
   if(x650_endpoint(N,predicted.data(),plan.data(),contexts.data(),c.geometry.data(),c.response.data(),endpoint.data(),error,512))throw CoreError(error,dg);
   dg.predicted_tip.resize(N*3);dg.predicted_rotations.resize(N*9);for(int i=0;i<N;++i){std::copy_n(endpoint.data()+15*i,3,dg.predicted_tip.data()+3*i);std::copy_n(endpoint.data()+15*i+6,9,dg.predicted_rotations.data()+9*i);}
   dg.objective=(Eigen::Map<const Matrix>(dg.A.data(),ar,n)*z-Eigen::Map<const Eigen::VectorXd>(dg.b.data(),ar)).squaredNorm();
   StepResult result;result.command=arr<5>(plan.row(0).transpose());result.position_reference=arr<3>(ref.p);result.velocity_feedforward=arr<3>(plan.row(0).head(3).transpose()-vec(c.base_position_gain).cwiseProduct(ref.p-vec(s.position)));
   result.joint_velocity={result.command[3],result.command[4]};result.joint_reference=arr<2>(ref.q);result.joint_velocity_reference=arr<2>(plan.row(0).tail(2).transpose()-c.joint_position_gain*(ref.q-vec(s.q)));result.joint_acceleration_reference={(result.command[3]-m.previous_command[3])/in.dt,(result.command[4]-m.previous_command[4])/in.dt};result.nominal_acceleration=arr<3>(ref.a);result.tip_actual=arr<3>(vec(s.position)+rotation(s.rotation)*tip(s.q,c));
   Memory next=m;next.plan=dg.plan;next.plan_h=h;next.plan_t=in.t;next.previous_command=result.command;next.last_t=in.t;next.has_last_t=true;next.previous_stamp=s.stamp;next.has_stamp=true;next.origin=s.control_time-in.t;next.has_origin=true;++next.step_count;
   next.joint_history.push_back({in.t,result.joint_velocity});next.joint_history.erase(std::remove_if(next.joint_history.begin(),next.joint_history.end(),[&](const Command& v){return v.time<in.t-.75;}),next.joint_history.end());
   if(diagnostic_mode_==DiagnosticMode::Compact){for(auto* v:{&dg.A,&dg.b,&dg.H,&dg.g,&dg.C,&dg.lower,&dg.upper,&dg.solution})std::vector<double>().swap(*v);}
   dg.compute_ms=elapsed(begin);require(dg.compute_ms<=50.,"TS20 total prepublication computation deadline exceeded");dg.accepted=true;result.diagnostics=std::move(dg);memory_=std::move(next);return result;
  }
  throw CoreError("Unreachable TS20 relinearization exit",dg);
 }catch(CoreError& error){if(error.diagnostics.subproblems==0)error.diagnostics=std::move(dg);error.diagnostics.accepted=false;error.diagnostics.compute_ms=elapsed(begin);throw;}
 catch(const std::exception& error){dg.accepted=false;dg.compute_ms=elapsed(begin);throw CoreError(error.what(),std::move(dg));}
}
} // namespace x650
