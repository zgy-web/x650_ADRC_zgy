#pragma once
// Passive dense observation of the already selected TS20 integration grid.
// The authoritative predictor is always called on its original boundaries.
// This observer repeats its accepted frozen-effort pieces without adding any
// integration boundary, and must match native states/envelope before use.
#include "mpc_core.hpp"
#include "joint_safety.hpp"
#include <algorithm>
#include <cmath>
#include <limits>

namespace x650::interval {
inline bool same_point(const IntervalPoint& a,const IntervalPoint& b){
 return a.joint==b.joint&&a.component==b.component&&std::abs(a.time-b.time)<=1e-10;
}
inline int merge_points(std::vector<IntervalPoint>& existing,const std::vector<IntervalPoint>& detected){
 const auto before=existing.size();for(const auto& point:detected){
  if(std::none_of(existing.begin(),existing.end(),[&](const auto& p){return same_point(p,point);})){
   if(existing.size()>=512)throw CoreError("interval safety point budget exhausted");existing.push_back(point);}}
 return int(existing.size()-before);
}
inline IntervalConstraint hard_constraint(const IntervalPoint& point,const std::vector<double>& gradient,
 const std::vector<double>& normalized_centre,int qp_row,double tolerance=2e-7){
 if(!std::isfinite(point.time)||!std::isfinite(point.value)||!std::isfinite(point.lower)||!std::isfinite(point.upper)||
    point.lower>point.upper||point.joint<0||point.joint>1||point.component<0||point.component>1||qp_row<0)
  throw CoreError("interval safety point finite/bounds");
 if(gradient.size()!=normalized_centre.size()||gradient.empty())throw CoreError("interval safety gradient shape");
 IntervalConstraint out;out.point=point;out.gradient=gradient;out.qp_row=qp_row;out.affine_offset=point.value;
 double peak=0.;for(std::size_t i=0;i<gradient.size();++i){if(!std::isfinite(gradient[i])||!std::isfinite(normalized_centre[i]))throw CoreError("interval safety gradient finite");out.affine_offset-=gradient[i]*normalized_centre[i];peak=std::max(peak,std::abs(gradient[i]));}
 if(peak<=1e-14&&(point.value<point.lower-tolerance||point.value>point.upper+tolerance))throw CoreError("interval safety violation in uncontrollable history");
 return out;
}
struct EncodingRemainder {double lower=0.,upper=0.,kernel_l1=0.,anchor_bias=0.,effort_half_lsb=0.;};
inline void add_encoding(EncodingRemainder& out,double kernel,double centre_error){
 if(!std::isfinite(kernel)||!std::isfinite(centre_error)||!std::isfinite(out.effort_half_lsb)||out.effort_half_lsb<0||
    std::abs(centre_error)>out.effort_half_lsb+1e-10)throw CoreError("interval encoding anchor quantization contract");
 out.kernel_l1+=std::abs(kernel);out.anchor_bias+=kernel*centre_error;
}
inline void finish_encoding(EncodingRemainder& out){
 out.lower=-out.kernel_l1*out.effort_half_lsb-out.anchor_bias;out.upper=out.kernel_l1*out.effort_half_lsb-out.anchor_bias;
 if(!std::isfinite(out.lower)||!std::isfinite(out.upper)||out.lower>1e-10||out.upper < -1e-10)throw CoreError("interval encoding remainder contract");
}
struct Motion {double q=0.,v=0.;};
struct Transition {double a00,a01,a10,a11,b0,b1;};
struct Dynamics {
 double J,b,Fc,offset,Fs,delta,vs,kp,kd;
 Transition transition(double h)const{
  const double k=kp/J,d=(b+kd)/J,half=-d/2,disc=half*half-k;double c,s;
  if(std::abs(disc)*h*h<1e-14){c=1;s=h;}else if(disc>0){const auto w=std::sqrt(disc);c=std::cosh(w*h);s=std::sinh(w*h)/w;}
  else{const auto w=std::sqrt(-disc);c=std::cos(w*h);s=std::sin(w*h)/w;}
  const double e=std::exp(half*h);Transition A{e*(c-half*s),e*s,e*(-k*s),e*(c+half*s),0,0};
  if(kp>0){A.b0=(1-A.a00)/kp;A.b1=-A.a10/kp;}else if(d>0){const auto gain=-std::expm1(-d*h)/d;A.b0=(h-gain)/(b+kd);A.b1=gain/J;}else{A.b0=h*h/(2*J);A.b1=h/J;}return A;
 }
 static Motion step(Motion x,Transition a,double effort){return {a.a00*x.q+a.a01*x.v+a.b0*effort,a.a10*x.q+a.a11*x.v+a.b1*effort};}
 double friction(double v)const{const auto ratio=std::min(28.,std::abs(v)/vs);return Fc+delta*std::exp(-ratio*ratio);}
};
struct Piece {double begin,end,effort;int joint;Motion initial,final;bool stuck=false;};
struct Observation {
 std::vector<double> states;
 std::array<double,8> envelope{};
 std::array<IntervalPoint,8> extrema{};
 std::vector<IntervalPoint> risk_candidates;
 std::vector<Piece> pieces;
 int substeps=0;
};
inline Dynamics dynamics(const Config& c,int j){return {c.motion[0],c.motion[1],c.motion[2],c.motion[3],c.motion[5],c.motion[6],c.motion[7],c.kp[j],c.kd[j]};}
inline IntervalPoint point(const Config& c,double time,int joint,int component,double value,int kind){
 IntervalPoint p;p.time=time;p.joint=joint;p.component=component;p.value=value;p.source_kind=kind;
 p.lower=component?-c.velocity_limit[3+joint]+joint_safety::dq_padding:c.q_lower[joint]+joint_safety::q_padding;
 p.upper=component? c.velocity_limit[3+joint]-joint_safety::dq_padding:c.q_upper[joint]-joint_safety::q_padding;return p;
}
inline Observation observe(const Config& c,const State& state,double start,const std::vector<Packet>& packets,
 const std::vector<double>& boundaries){
 if(packets.empty()||boundaries.empty())throw CoreError("interval observer missing canonical inputs");
 Observation out;std::array<Motion,2>x{{{state.q[0],state.dq[0]},{state.q[1],state.dq[1]}}};
 auto record=[&](double time,int j,int component,double value,int kind){
  int lo=component?4+j:j,hi=component?6+j:2+j;
  if(kind==2||kind==3)out.risk_candidates.push_back(point(c,time,j,component,value,kind));
  if(value<out.envelope[lo]){out.envelope[lo]=value;out.extrema[lo]=point(c,time,j,component,value,kind);}
  if(value>out.envelope[hi]){out.envelope[hi]=value;out.extrema[hi]=point(c,time,j,component,value,kind);}};
 for(int j=0;j<2;++j){out.envelope[j]=out.envelope[2+j]=x[j].q;out.envelope[4+j]=out.envelope[6+j]=x[j].v;
  out.extrema[j]=out.extrema[2+j]=point(c,start,j,0,x[j].q,0);out.extrema[4+j]=out.extrema[6+j]=point(c,start,j,1,x[j].v,0);}
 std::vector<double>knots;for(const auto& p:packets)knots.push_back(p.send_time+c.motion[4]);
 int active=0;while(active+1<int(packets.size())&&knots[active+1]<=start+1e-11)++active;double cursor=start;
 for(double end:boundaries){const double duration=end-cursor;if(duration < -1e-12)throw CoreError("interval observer regressed time");
  const int steps=std::max(1,int(std::ceil(duration/c.integration_step-1e-12)));const double h=duration/steps;if(duration>0)out.substeps+=steps;
  for(int j=0;j<2;++j){const auto d=dynamics(c,j);const auto full=d.transition(h),half=d.transition(h/2);
   const auto& p=packets[active];const double force=c.kp[j]*p.q_ref[j]+c.kd[j]*p.dq_ref[j]+p.tau_ff[j],damping=d.b+d.kd;
   for(int sub=0;sub<steps&&h>0;++sub){double remaining=h;int restarts=0;
    while(remaining>1e-14){if(++restarts>12)throw CoreError("interval observer zero-event budget exhausted");
     const double at=cursor+sub*h+h-remaining,drive=force-d.kp*x[j].q-d.offset;
     if(x[j].v==0&&std::abs(drive)<=d.Fs){out.pieces.push_back({at,at+remaining,0.,j,x[j],x[j],true});break;}
     const double sign=std::copysign(1.,x[j].v!=0?x[j].v:drive);double effort=force-d.offset-sign*d.friction(x[j].v);
     const auto midpoint=Dynamics::step(x[j],remaining==h?half:d.transition(remaining/2),effort);
     effort=force-d.offset-sign*d.friction(midpoint.v);Motion next=Dynamics::step(x[j],remaining==h?full:d.transition(remaining),effort);
     double accepted=remaining;bool zero=false;
     if(sign*next.v<0){double lo=0,hi=remaining;for(int i=0;i<36;++i){const auto mid=(lo+hi)/2;if(sign*Dynamics::step(x[j],d.transition(mid),effort).v>0)lo=mid;else hi=mid;}
      next=Dynamics::step(x[j],d.transition(hi),effort);next.v=0;accepted=hi;zero=true;}
     out.pieces.push_back({at,at+accepted,effort,j,x[j],next,false});
     record(at+accepted,j,0,next.q,zero?2:1);record(at+accepted,j,1,next.v,zero?2:1);
     const double acc0=(effort-d.kp*x[j].q-damping*x[j].v)/d.J,acc1=(effort-d.kp*next.q-damping*next.v)/d.J;
     if(acc0*acc1<0){double lo=0,hi=accepted,mid=0;Motion value{};for(int i=0;i<30;++i){mid=(lo+hi)/2;value=Dynamics::step(x[j],d.transition(mid),effort);const auto a=(effort-d.kp*value.q-damping*value.v)/d.J;if(a*acc0>0)lo=mid;else hi=mid;}
      record(at+mid,j,1,value.v,3);}
     x[j]=next;remaining-=accepted;
    }
   }
  }
  for(int j=0;j<2;++j)if(!std::isfinite(x[j].q)||!std::isfinite(x[j].v))throw CoreError("interval observer nonfinite");
  out.states.insert(out.states.end(),{x[0].q,x[1].q,x[0].v,x[1].v});
  for(int j=0;j<2;++j){out.risk_candidates.push_back(point(c,end,j,0,x[j].q,0));out.risk_candidates.push_back(point(c,end,j,1,x[j].v,0));}cursor=end;
  while(active+1<int(packets.size())&&knots[active+1]<=end+1e-11)++active;
 }
 return out;
}
inline void verify_canonical(const Observation& observed,const std::vector<double>& states,
 const std::array<double,8>& envelope,int substeps){
 bool equal=observed.states.size()==states.size()&&observed.substeps==substeps;
 for(std::size_t i=0;equal&&i<states.size();++i)equal=std::abs(observed.states[i]-states[i])<=1e-12;
 for(int i=0;equal&&i<8;++i)equal=std::abs(observed.envelope[i]-envelope[i])<=1e-12;
 if(!equal)throw CoreError("interval observer differs from canonical native rollout");
}
inline std::vector<IntervalPoint> violations(const Observation& o,double tolerance=2e-7){
 std::vector<IntervalPoint> result;for(const auto& p:o.extrema)if(p.value<p.lower-tolerance||p.value>p.upper+tolerance){
  if(std::none_of(result.begin(),result.end(),[&](const auto& old){return same_point(old,p);}))result.push_back(p);}
 return result;
}
inline IntervalPoint evaluate(const Config& c,const State& state,double start,const Observation& o,IntervalPoint p){
 if(std::abs(p.time-start)<=1e-12)return point(c,p.time,p.joint,p.component,p.component?state.dq[p.joint]:state.q[p.joint],p.source_kind);
 for(const auto& piece:o.pieces)if(piece.joint==p.joint&&p.time>=piece.begin-1e-12&&p.time<=piece.end+1e-12){
  Motion value=piece.initial;if(std::abs(p.time-piece.end)<=1e-12)value=piece.final;
  else if(!piece.stuck)value=Dynamics::step(piece.initial,dynamics(c,p.joint).transition(std::max(0.,p.time-piece.begin)),piece.effort);
  return point(c,p.time,p.joint,p.component,p.component?value.v:value.q,p.source_kind);}
 throw CoreError("interval dense observation time not covered by canonical pieces");
}
} // namespace x650::interval
