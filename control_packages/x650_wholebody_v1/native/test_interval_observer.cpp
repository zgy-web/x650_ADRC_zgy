// Canonical observer parity uses the real TS20 kernel. Pure affine samples
// below test point/constraint bookkeeping only; they are not flight evidence.
#include "canonical_interval.hpp"
#include <algorithm>
#include <array>
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

extern "C" int ts20_prediction_batch(const double*,const double*,double,double,
 const double*,const double*,const double*,int,const double*,int,int,
 double*,int*,int*,double*,double*,int,int*,int*,char*,int);
extern "C" int x650_solve(int,int,const double*,const double*,const double*,const double*,const double*,int,double*,double*,int*,char*,int);
namespace {
void check(bool value,const char* message){if(!value)throw std::runtime_error(message);}
void close(double a,double b,const char* message,double tolerance=1e-11){check(std::abs(a-b)<=tolerance,message);}
template<class Action>void rejected(Action action,const char* text){bool failed=false;try{action();}catch(const x650::CoreError& error){failed=std::string(error.what()).find(text)!=std::string::npos;}check(failed,"expected fail-closed interval-helper rejection");}
x650::Config config(){x650::Config c;c.nodes=3;c.geometry={.2,.2,-.7853981633974483,.7853981633974483,0,0,0,0,0,0};
 c.physical_lower={-2,-2};c.physical_upper={2,2};c.response={4.,.8,4.,.8,.25};c.base_position_gain={1,1,1};c.motion={.01,.1,.02,0,.015,.03,.005,.01};return c;}

void canonical_case(int waveform){
 const auto c=config();x650::State state;
 if(waveform==2){state.q={.02,-.015};state.dq={.12,-.08};}
 std::vector<x650::Packet> packets;
 for(int i=-6;i<=38;++i){x650::Packet p;p.send_time=.005*i;p.kp=c.kp;p.kd=c.kd;
  if(waveform==1){p.q_ref={.015*std::sin(35*p.send_time),-.012*std::cos(27*p.send_time)};p.dq_ref={.08*std::cos(35*p.send_time),.06*std::sin(27*p.send_time)};}
  if(waveform==2){double sign=i<15?1.:-1.;p.q_ref={sign*.035,-sign*.025};p.dq_ref={sign*.15,-sign*.12};}
  packets.push_back(p);}
 std::vector<double> boundaries{0.,.02,.06,.10,.14,.18};
 std::vector<double> knots,forcing;
 for(const auto& p:packets){double applied=p.send_time+c.motion[4];knots.push_back(applied);
  if(applied>0&&applied<=.18)boundaries.push_back(applied);
  for(int j=0;j<2;++j)forcing.push_back(c.kp[j]*p.q_ref[j]+c.kd[j]*p.dq_ref[j]+p.tau_ff[j]);}
 std::sort(boundaries.begin(),boundaries.end());boundaries.erase(std::unique(boundaries.begin(),boundaries.end(),[](double a,double b){return std::abs(a-b)<1e-12;}),boundaries.end());
 const auto original_boundaries=boundaries;const auto original_packets=packets;
 std::array<double,4> gains{c.kp[0],c.kp[1],c.kd[0],c.kd[1]},initial{state.q[0],state.q[1],state.dq[0],state.dq[1]};
 std::vector<double> states(4*boundaries.size()),events(4096*5);
 std::vector<int> modes(2*boundaries.size()),zeros(boundaries.size());std::array<double,8> envelope{};
 int event_count=0,substeps=0;char error[512]{};
 int code=ts20_prediction_batch(c.motion.data(),gains.data(),c.integration_step,0.,initial.data(),knots.data(),forcing.data(),int(packets.size()),boundaries.data(),int(boundaries.size()),1,
   states.data(),modes.data(),zeros.data(),envelope.data(),events.data(),4096,&event_count,&substeps,error,512);
 check(!code,error);
 auto observed=x650::interval::observe(c,state,0.,packets,boundaries);
 x650::interval::verify_canonical(observed,states,envelope,substeps);
 check(boundaries==original_boundaries&&packets.size()==original_packets.size(),"observer changed canonical boundary/packet count");
 for(std::size_t i=0;i<packets.size();++i)check(packets[i].send_time==original_packets[i].send_time&&packets[i].q_ref==original_packets[i].q_ref&&packets[i].dq_ref==original_packets[i].dq_ref,"observer changed native packets");
 const auto saved_states=observed.states;const auto saved_envelope=observed.envelope;
 for(std::size_t i=0;i<boundaries.size();++i)for(int j=0;j<2;++j)for(int component=0;component<2;++component){
  auto p=x650::interval::point(c,boundaries[i],j,component,0,0);
  auto value=x650::interval::evaluate(c,state,0.,observed,p);
  close(value.value,states[4*i+j+2*component],"dense observer differs at original native boundary",1e-12);}
 // Ask for true interior accepted-piece values without adding a native grid
 // point. This observation must not rewrite canonical states or envelopes.
 for(std::size_t i=0;i<observed.pieces.size();i+=17){const auto& piece=observed.pieces[i];
  auto p=x650::interval::point(c,.5*(piece.begin+piece.end),piece.joint,1,0,1);
  auto value=x650::interval::evaluate(c,state,0.,observed,p);check(std::isfinite(value.value),"nonfinite dense piece value");}
 check(observed.states==saved_states&&observed.envelope==saved_envelope,"dense observations mutated original canonical rollout");
 auto corrupt=envelope;corrupt[6]+=.0001;
 rejected([&]{x650::interval::verify_canonical(observed,states,corrupt,substeps);},"differs from canonical native rollout");
 rejected([&]{x650::interval::verify_canonical(observed,states,envelope,substeps+1);},"differs from canonical native rollout");
}

std::vector<double> solve(const std::vector<x650::IntervalConstraint>& rows,std::array<double,2> target={0,0}){
 std::vector<double>C,lo,hi;for(const auto& row:rows){check(row.gradient.size()==2,"pure QP test gradient size");C.insert(C.end(),row.gradient.begin(),row.gradient.end());lo.push_back(row.point.lower-row.affine_offset-row.encoding_lower_remainder);hi.push_back(row.point.upper-row.affine_offset-row.encoding_upper_remainder);}
 const double H[4]{2,0,0,2};const double g[2]{-2*target[0],-2*target[1]};double solution[2]{},quality[4]{};int mode=-1;char error[512]{};
 int code=x650_solve(2,int(rows.size()),H,g,C.data(),lo.data(),hi.data(),100,solution,quality,&mode,error,512);check(!code,error);
 check(quality[0]<=2e-7&&quality[1]<=.002,"pure interval-row QP quality failed");return {solution[0],solution[1]};
}
x650::interval::Observation sample(const x650::Config& c,const std::vector<double>& z){
 x650::interval::Observation o;
 // Both sparse horizon nodes can be zero; affine internal A initially
 // violates .42, while internal B becomes limiting after correcting A.
 o.states={0,0,0,0,0,0,0,0};
 o.extrema[6]=x650::interval::point(c,.0525,0,1,.425+z[0],3);
 o.extrema[7]=x650::interval::point(c,.0725,1,1,.317-2*z[0]+z[1],3);
 o.envelope[6]=o.extrema[6].value;o.envelope[7]=o.extrema[7].value;
 return o;
}
void pure_rows_and_moving_extrema(){
 auto c=config();std::vector<double> centre{0,0};auto initial=sample(c,centre);
 check(initial.states[2]==0&&initial.states[3]==0,"pure sparse nodes should be individually safe");
 auto violating=x650::interval::violations(initial);check(violating.size()==1&&violating[0].joint==0,"internal peak omitted despite safe nodes");
 std::vector<x650::IntervalPoint> retained;check(x650::interval::merge_points(retained,violating)==1,"first peak was not retained");
 auto first=x650::interval::hard_constraint(retained[0],{1,0},centre,230);
 close(first.point.upper,.42,"original joint-1 hard state limit changed");check(first.qp_row==230,"hard row identity lost");
 close(first.affine_offset,.425,"affine offset does not reproduce centre value");
 auto z=solve({first});check(z[0]<-.0049,"new hard row did not change actual QP optimum");
 auto second=sample(c,z);violating=x650::interval::violations(second);
 check(violating.size()==1&&violating[0].joint==1,"moved second peak must be rechecked after first constrained solve");
 check(x650::interval::merge_points(retained,violating)==1&&retained.size()==2,"moving peak replaced/lost first hard point");
 // Rebuild all rows about the current centre. No previous affine offset or
 // violation value can silently be reused at the next linearization.
 auto A=x650::interval::hard_constraint(second.extrema[6],{1,0},z,230);
 auto B=x650::interval::hard_constraint(second.extrema[7],{-2,1},z,231);
 close(A.affine_offset,.425,"first retained row relinearization centre drift");close(B.affine_offset,.317,"new row affine centre mismatch");
 auto final=solve({A,B});auto complete=sample(c,final);
 check(x650::interval::violations(complete).empty(),"two retained hard rows failed the next full affine envelope check");
 check(retained.size()==2,"rechecked peak set must retain original and moved peak");
 auto near=retained[0];near.time+=5e-11;near.value+=.1;
 check(x650::interval::merge_points(retained,{near})==0&&retained.size()==2,"same-time point duplicated instead of relinearized");
 auto position=retained[0];position.component=0;position.value=.5;position.lower=-.7497;position.upper=.4497;
 check(x650::interval::merge_points(retained,{position})==1,"q and dq at same time/joint were improperly merged");
 std::vector<x650::IntervalPoint> capacity;
 for(int i=0;i<512;++i){auto p=position;p.time=.001*i;x650::interval::merge_points(capacity,{p});}
 auto extra=position;extra.time=1.;rejected([&]{x650::interval::merge_points(capacity,{extra});},"point budget exhausted");
}
void uncontrollable_and_tolerance(){
 auto c=config();auto p=x650::interval::point(c,0.,0,1,.43,0);
 rejected([&]{x650::interval::hard_constraint(p,{0,0},{0,0},230);},"uncontrollable history");
 p.time=.013;p.source_kind=1;rejected([&]{x650::interval::hard_constraint(p,{0,0},{0,0},230);},"uncontrollable history");
 p.value=p.upper+1e-7;auto within=x650::interval::hard_constraint(p,{0,0},{0,0},230);close(within.affine_offset,p.value,"constant tolerance changed centre value");
 p.value=p.upper+3e-7;rejected([&]{x650::interval::hard_constraint(p,{0,0},{0,0},230);},"uncontrollable history");
 rejected([&]{x650::interval::hard_constraint(p,{0},{0,0},230);},"gradient shape");
}
double quantize(double value,double step){return std::copysign(std::floor(std::abs(value/step)+.5),value)*step;}
void encoding_remainder_boundaries(){
 const auto c=config();const double qstep=2*std::acos(-1.)/std::pow(2.,24),vstep=1./512.;
 const double epsilon=.5*(c.kp[0]*qstep+c.kd[0]*vstep);const std::array<double,3> kernel{1.,-.5,.3};
 auto error=[&](double fraction,int cell){double q=(cell+fraction)*qstep,v=(cell+fraction)*vstep;
  return c.kp[0]*(quantize(q,qstep)-q)+c.kd[0]*(quantize(v,vstep)-v);};
 std::array<double,3> anchor{error(.2,2),error(-.3,-4),error(.4,5)};
 x650::interval::EncodingRemainder guard;guard.effort_half_lsb=epsilon;
 for(int i=0;i<3;++i)x650::interval::add_encoding(guard,kernel[i],anchor[i]);x650::interval::finish_encoding(guard);
 close(guard.kernel_l1,1.8,"encoding kernel absolute sum changed");
 double lowest=1.,highest=-1.;
 // Actual half-away encodings on both sides of cells, with positive and
 // negative response kernels. The centre's signed bias cannot be discarded.
 for(int mask=0;mask<8;++mask){double delta=0.;for(int i=0;i<3;++i){double sign=(mask&(1<<i))?1.:-1.;
   delta+=kernel[i]*(error(sign*.499999,i%2?7:-6)-anchor[i]);}
  check(delta>=guard.lower-1e-12&&delta<=guard.upper+1e-12,"true quantizer cell change escaped derived remainder");lowest=std::min(lowest,delta);highest=std::max(highest,delta);}
 check(highest>0&&lowest<0,"encoding counterexamples must exercise both bound directions");
 auto p=x650::interval::point(c,.0525,0,1,0.,3);auto row=x650::interval::hard_constraint(p,{1,0},{0,0},230);
 row.encoding_lower_remainder=guard.lower;row.encoding_upper_remainder=guard.upper;
 row.encoding_kernel_l1=guard.kernel_l1;row.encoding_anchor_bias=guard.anchor_bias;row.encoding_effort_half_lsb=epsilon;
 check(p.upper+highest>p.upper&&p.lower+lowest<p.lower,"unguarded edge should fail after real cell rounding");
 auto upper=solve({row},{p.upper,0.}),lower=solve({row},{p.lower,0.});
 check(upper[0]+highest<=p.upper+1e-10&&lower[0]+lowest>=p.lower-1e-10,"guarded actual QP edge failed the unchanged hard bound");
 rejected([&]{auto bad=guard;x650::interval::add_encoding(bad,1.,epsilon*1.1);},"quantization contract");
 x650::interval::EncodingRemainder no_future;no_future.effort_half_lsb=epsilon;x650::interval::finish_encoding(no_future);
 check(no_future.lower==0&&no_future.upper==0&&no_future.kernel_l1==0&&no_future.anchor_bias==0,"actual history received fictitious future encoding uncertainty");
}
}
int main(){try{
 for(int waveform=0;waveform<3;++waveform)canonical_case(waveform);
 pure_rows_and_moving_extrema();uncontrollable_and_tolerance();encoding_remainder_boundaries();
 std::cout<<"PASS: real TS20 canonical-grid hold/sine/reversal parity, passive dense observations, internal-hard-row QP, retained moving peaks, bounded points and uncontrollable/tolerance rejection. Pure affine rows are numerical tests, not a physical repair certificate.\n";return 0;
 }catch(const std::exception& error){std::cerr<<error.what()<<'\n';return 1;}}
