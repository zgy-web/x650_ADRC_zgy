#pragma once
// Direct-state port of estimated_feedback.py and ts20_runtime.py. No ROS,
// optimizer, estimated/truth alignment, or planned actuator values enter here.
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <deque>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace flyarm_state_cpp {
using V2=std::array<double,2>; using V3=std::array<double,3>;
using M3=std::array<double,9>; using Q4=std::array<double,4>;
inline void require(bool value,const char* message) { if(!value) throw std::runtime_error(message); }
inline bool finite(double v) { return std::isfinite(v); }
template<class T> bool all_finite(const T& a) { for(auto v:a) if(!finite(v)) return false; return true; }
inline std::uint64_t integer(double v) {
  require(finite(v)&&v>=0&&v<9007199254740992.&&std::floor(v)==v,"Invalid exact nonnegative CSV integer");
  return static_cast<std::uint64_t>(v);
}
inline V3 ned_enu(const V3& x) { return {x[1],x[0],-x[2]}; }
inline M3 rotation(const Q4& q) {
  require(all_finite(q),"Nonfinite PX4 quaternion");
  double n=std::sqrt(q[0]*q[0]+q[1]*q[1]+q[2]*q[2]+q[3]*q[3]);
  require(std::abs(n-1)<=1e-3,"PX4 quaternion is not unit length");
  const double w=q[0]/n,x=q[1]/n,y=q[2]/n,z=q[3]/n;
  const M3 r{1-2*(y*y+z*z),2*(x*y-w*z),2*(x*z+w*y),
             2*(x*y+w*z),1-2*(x*x+z*z),2*(y*z-w*x),
             2*(x*z-w*y),2*(y*z+w*x),1-2*(x*x+y*y)};
  // NED_TO_ENU * R(FRD->NED) * FRD_TO_FLU.
  return {r[3],-r[4],-r[5],r[0],-r[1],-r[2],-r[6],r[7],r[8]};
}
struct Receipt { std::uint64_t sequence{},wall_ns{}; double sim_s{std::numeric_limits<double>::quiet_NaN()}; };
struct Origin {
  double lat{},lon{},alt{}; std::uint64_t ref_timestamp{};
  bool xy_global{},z_global{};
  std::uint8_t xy_reset{},z_reset{},heading_reset{},vxy_reset{},vz_reset{},quat_reset{};
};
struct Position {
  V3 p{},v{},a{}; Origin origin;
  bool xy_valid{},z_valid{},vxy_valid{},vz_valid{},heading_good{},dead_reckoning{};
};
struct Flags {
  bool ev_pos{},ev_vel{},ev_hgt{},gnss_pos{},gnss_vel{},gps_hgt{},baro_hgt{},inertial_dead{},wind_dead{};
};
struct EstimatedState { V3 p{},v{},a{},omega{}; M3 R{}; double state_time{}; };
class EstimatorGuard {
 public:
  explicit EstimatorGuard(Origin bound):bound_(bound) {
    require(finite(bound.lat)&&finite(bound.lon)&&finite(bound.alt)&&std::abs(bound.lat)<=90&&
      std::abs(bound.lon)<=180&&bound.z_global&&bound.ref_timestamp>0,"Invalid bound raw-EV origin");
  }
  EstimatedState read(double now,std::uint64_t wall,const Position& p,const Q4& q,const V3& omega,
                      const Flags& flags,const std::array<Receipt,5>& receipts) {
    require(finite(now)&&now>=0&&(!have_||now>=last_sim_-1e-9),"Estimator clock regressed");
    // Receipt order: local position, attitude, angular rate, flags, /clock.
    for(std::size_t i=0;i<receipts.size();++i) {
      const auto& r=receipts[i]; const double sim_age=now-r.sim_s;
      const double wall_age=static_cast<double>(wall)*1e-9-static_cast<double>(r.wall_ns)*1e-9;
      require(r.sequence>0&&r.wall_ns>0&&finite(r.sim_s),"Missing raw PX4 receipt");
      require(sim_age>=-1e-9&&sim_age<=(i==3?1.25:.10)+1e-9,"PX4 receipt simulation age invalid");
      require(wall_age>=-1e-6&&wall_age<=(i==3?10.:4.),"PX4 receipt wall age invalid");
      if(have_) {
        const auto& old=previous_[i];
        require(r.sequence>=old.sequence&&r.sim_s>=old.sim_s-1e-9&&r.wall_ns>=old.wall_ns,
          "Raw PX4 sequence/receipt regressed");
        require(r.sequence!=old.sequence||(r.sim_s==old.sim_s&&r.wall_ns==old.wall_ns),
          "Held PX4 receipt changed without callback");
      }
    }
    require(std::abs(receipts[4].sim_s-now)<=1e-8,"Selection clock differs from /clock");
    const auto& o=p.origin;
    require(finite(o.lat)&&finite(o.lon)&&finite(o.alt)&&std::abs(o.lat)<=90&&std::abs(o.lon)<=180,
      "Invalid live PX4 origin");
    require(std::abs(o.lat-bound_.lat)<=1e-11&&std::abs(o.lon-bound_.lon)<=1e-11&&std::abs(o.alt-bound_.alt)<=1e-6,
      "PX4 origin changed");
    require(o.xy_global==bound_.xy_global&&o.z_global==bound_.z_global&&o.xy_reset==bound_.xy_reset&&
      o.z_reset==bound_.z_reset&&o.heading_reset==bound_.heading_reset&&o.vxy_reset==bound_.vxy_reset&&
      o.vz_reset==bound_.vz_reset&&o.quat_reset==bound_.quat_reset,"PX4 reset/global frame changed");
    require(p.xy_valid&&p.z_valid&&p.vxy_valid&&p.vz_valid&&p.heading_good&&!p.dead_reckoning,
      "PX4 estimate invalid or dead reckoning");
    require(flags.ev_pos&&flags.ev_vel&&flags.ev_hgt&&!flags.gnss_pos&&!flags.gnss_vel&&!flags.gps_hgt&&
      !flags.baro_hgt&&!flags.inertial_dead&&!flags.wind_dead,"PX4 EV flags invalid");
    require(all_finite(p.p)&&all_finite(p.v)&&all_finite(p.a)&&all_finite(omega),"Nonfinite raw PX4 state");
    EstimatedState state;state.p=ned_enu(p.p);state.p[2]+=bound_.alt;state.v=ned_enu(p.v);state.a=ned_enu(p.a);
    state.R=rotation(q);state.omega={omega[0],-omega[1],-omega[2]};state.state_time=receipts[2].sim_s;
    previous_=receipts;last_sim_=now;have_=true;return state;
  }
 private: Origin bound_;std::array<Receipt,5> previous_{};double last_sim_{};bool have_{};
};

struct Packet {
  std::uint64_t sequence{}; double send{},apply{};V2 q{},dq{},tau{},kp{},kd{};
};
struct JointRow {
  double sim{},begin{};std::array<std::uint64_t,2> upper{};V2 u{},receipt{},q_physical{},q_internal{},dq{},r{};
  std::uint64_t last_sequence{},active_sequence{};double last_send{},last_apply{},active_send{},active_apply{};
  V2 last_q{},last_dq{},active_q{},active_dq{};
};
struct Command { double when{},available{};V2 u{}; };
struct JointContext {
  JointRow row;double control_time{},reference_time{},bus_origin{};std::uint64_t generation{};
  std::vector<Packet> packets;std::vector<Command> commands;
};
class JointHistory {
 public:
  JointHistory(double bus_period,double delay):period_(bus_period),delay_(delay) {
    require(finite(period_)&&period_>0&&finite(delay_)&&delay_>=0,"Invalid frozen TS20 bus/delay");
  }
  void append(const JointRow& r) {
    require(finite(r.sim)&&finite(r.begin)&&all_finite(r.u)&&all_finite(r.receipt)&&all_finite(r.q_physical)&&
      all_finite(r.q_internal)&&all_finite(r.dq)&&all_finite(r.r)&&all_finite(r.last_q)&&all_finite(r.last_dq)&&
      all_finite(r.active_q)&&all_finite(r.active_dq)&&finite(r.last_send)&&finite(r.last_apply)&&
      finite(r.active_send)&&finite(r.active_apply),"Nonfinite TS20 state");
    require(!have_row_||r.sim>last_row_time_,"TS20 state source clock did not increase");
    require(!have_row_||r.sim-last_row_time_<=.02000001,"TS20 physics interval gap");
    if(!have_row_) commands_.push_back({r.begin,r.sim,r.u});
    else {
      for(std::size_t j=0;j<2;++j) require(r.upper[j]>=last_seq_[j],"TS20 consumed counter regressed");
      std::vector<double> changes;
      for(std::size_t j=0;j<2;++j) if(r.upper[j]!=last_seq_[j]) changes.push_back(r.receipt[j]);
      std::sort(changes.begin(),changes.end());changes.erase(std::unique(changes.begin(),changes.end()),changes.end());
      for(double when:changes) {
        V2 value=last_u_;
        for(std::size_t j=0;j<2;++j) if(r.upper[j]!=last_seq_[j]&&r.receipt[j]<=when) value[j]=r.u[j];
        require(when>=commands_.back().when-1e-10,"TS20 consumed receipt clock regressed");
        Command c{when,r.sim,value};
        if(std::abs(when-commands_.back().when)<1e-10) commands_.back()=c;else commands_.push_back(c);
      }
    }
    rows_.push_back(r);last_row_time_=r.sim;last_seq_=r.upper;last_u_=r.u;have_row_=true;check_bound();
  }
  void append(const Packet& p) {
    require(finite(p.send)&&finite(p.apply)&&all_finite(p.q)&&all_finite(p.dq)&&all_finite(p.tau)&&
      all_finite(p.kp)&&all_finite(p.kd),"Nonfinite actual MIT packet");
    require(!have_packet_||p.send>last_packet_time_,"TS20 packet clock did not increase");
    require(!have_packet_||(p.sequence==last_packet_sequence_+1&&std::abs(p.send-last_packet_time_-period_)<=1e-9),
      "TS20 actual MIT packet missing interval");
    require(std::abs(p.apply-p.send-delay_)<=1e-9,"TS20 actual/predictor delay differs");
    if(!have_packet_) bus_origin_=p.send;
    const auto grid=(p.send-bus_origin_)/period_;
    require(std::abs(grid-std::round(grid))<=1e-6,"TS20 actual send off frozen grid");
    packets_.push_back(p);last_packet_time_=p.send;last_packet_sequence_=p.sequence;have_packet_=true;check_bound();
  }
  bool has_causal_row(double now) const {return !rows_.empty()&&rows_.front().sim<=now;}
  // Used during READY to bound historical cache without manufacturing a control
  // sample. Trimming is always relative to a causal row, never a future read.
  void trim_ready(double now) {
    const JointRow* selected=nullptr;for(const auto& r:rows_) if(r.sim<=now)selected=&r;
    if(selected)trim(selected->sim);
  }
  JointContext select(double now,double task_time) {
    require(finite(now)&&finite(task_time)&&(!have_selection_||now>=last_selection_-1e-9),"TS20 selection clock regressed");
    const JointRow* selected=nullptr;for(const auto& r:rows_) if(r.sim<=now) selected=&r;
    require(selected!=nullptr,"No causal TS20 encoder sample");const JointRow r=*selected;
    require(0<=now-r.sim+1e-10&&now-r.sim+1e-10<=.025+1e-10,"TS20 encoder exceeds 25 ms age");
    for(std::size_t j=0;j<2;++j)require(std::abs(r.q_physical[j]-r.q_internal[j])<=.0001,"TS20 physical/internal q disagreement");
    check_packet(r.last_sequence,r.last_send,r.last_apply,r.last_q,r.last_dq);
    if(r.active_sequence!=0)check_packet(r.active_sequence,r.active_send,r.active_apply,r.active_q,r.active_dq);
    require(0<=r.sim-r.last_send+1e-10&&r.sim-r.last_send+1e-10<=period_+2e-10,"Actual MIT stream stale");
    trim(r.sim);JointContext out;out.row=r;out.control_time=now;const double origin=now-task_time;
    out.reference_time=r.sim-origin;out.bus_origin=bus_origin_-origin;
    for(const auto& p:packets_) if(p.send<=now+1e-10)out.packets.push_back(p);
    std::size_t active=0;for(std::size_t i=0;i<out.packets.size();++i)if(out.packets[i].apply<=r.sim+1e-10)active=i;
    out.packets.erase(out.packets.begin(),out.packets.begin()+active);
    for(auto& p:out.packets) {p.send-=origin;p.apply-=origin;}
    for(const auto& c:commands_) if(c.when<=now+1e-10&&c.available<=now+1e-10)out.commands.push_back(c);
    active=0;for(std::size_t i=0;i<out.commands.size();++i)if(out.commands[i].when<=r.sim+1e-10)active=i;
    out.commands.erase(out.commands.begin(),out.commands.begin()+active);
    for(auto& c:out.commands){c.when-=origin;c.available-=origin;}
    require(have_packet_&&!out.commands.empty()&&!out.packets.empty(),"Missing actual command/MIT history");
    require(out.commands.size()<=64&&out.packets.size()<=64,"TS20 context exceeds 64-entry wire capacity");
    out.generation=++generation_;last_selection_=now;have_selection_=true;return out;
  }
  std::size_t row_count()const{return rows_.size();}
 private:
  template<class T,class F> static void trim_one(std::deque<T>& data,double cutoff,F time) {
    while(data.size()>1&&time(data[1])<cutoff)data.pop_front();
  }
  void trim(double at){const auto cutoff=at-.75;
    trim_one(rows_,cutoff,[](const JointRow&r){return r.sim;});
    trim_one(packets_,cutoff,[](const Packet&r){return r.send;});
    trim_one(commands_,cutoff,[](const Command&r){return r.when;});}
  void check_bound()const{require(rows_.size()<=65536&&packets_.size()<=65536&&commands_.size()<=65536,
    "TS20 bounded history overflow (no silent truncation)");}
  void check_packet(std::uint64_t seq,double send,double apply,const V2&q,const V2&dq)const{
    const Packet* found=nullptr;for(const auto& p:packets_)if(p.sequence==seq)found=&p;
    require(found!=nullptr,"TS20 state declares missing actual packet");
    require(std::abs(send-found->send)<=1e-10&&std::abs(apply-found->apply)<=1e-10,"TS20 packet/state times differ");
    for(std::size_t j=0;j<2;++j)require(std::abs(q[j]-found->q[j])<=1e-10&&std::abs(dq[j]-found->dq[j])<=1e-10,
      "TS20 packet/state values differ");
  }
  double period_,delay_,bus_origin_{},last_row_time_{},last_packet_time_{},last_selection_{};
  bool have_row_{},have_packet_{},have_selection_{};std::uint64_t last_packet_sequence_{},generation_{};
  std::array<std::uint64_t,2>last_seq_{};V2 last_u_{};
  std::deque<JointRow>rows_;std::deque<Packet>packets_;std::deque<Command>commands_;
};
struct Target {V3 p{},v{},a{};};
struct Trajectory {
  bool figure8{};double ax{},az{},period{12},start{2},ramp{2};std::uint32_t cycles{1};
  void validate()const{require(finite(ax)&&finite(az)&&finite(period)&&period>0&&finite(start)&&start>=0&&
    finite(ramp)&&ramp>0&&cycles>=1&&ramp<=cycles*period,"Invalid frozen C2 figure-eight");}
  Target sample(double t,V3 center)const{
    require(finite(t)&&all_finite(center),"Invalid target sample");Target o;o.p=center;if(!figure8)return o;
    validate();constexpr double pi=3.14159265358979323846;const double w=2*pi/period,total=2*pi*cycles,duration=cycles*period+ramp;
    const double s=t-start;double phi=0,speed=0,acc=0;
    if(s>=duration)phi=total;
    else if(s>0&&(s<ramp||s>duration-ramp)){
      const bool ending=s>duration-ramp;const double u=(ending?duration-s:s)/ramp;
      const double a=2.5*std::pow(u,4)-3*std::pow(u,5)+std::pow(u,6);
      const double b=10*std::pow(u,3)-15*std::pow(u,4)+6*std::pow(u,5);
      const double c=30*u*u*(1-u)*(1-u);
      phi=ending?total-w*ramp*a:w*ramp*a;speed=w*b;acc=(ending?-1:1)*w*c/ramp;
    }else if(s>0){phi=w*(s-.5*ramp);speed=w;}
    const V3 d{ax*std::cos(phi),0,2*az*std::cos(2*phi)},dd{-ax*std::sin(phi),0,-4*az*std::sin(2*phi)};
    o.p[0]+=ax*std::sin(phi);o.p[2]+=az*std::sin(2*phi);
    for(std::size_t j=0;j<3;++j){o.v[j]=d[j]*speed;o.a[j]=dd[j]*speed*speed+d[j]*acc;}return o;
  }
};
} // namespace flyarm_state_cpp
