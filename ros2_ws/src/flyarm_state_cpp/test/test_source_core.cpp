#include "flyarm_state_cpp/source_core.hpp"
#include "flyarm_state_cpp/csv_source.hpp"
#include <iostream>
#include <cstdio>
#include <functional>
using namespace flyarm_state_cpp;
static unsigned checks=0;
static void check(bool v){++checks;if(!v)throw std::runtime_error("Test check failed at "+std::to_string(checks));}
static void near(double a,double b,double tol=1e-12){check(std::abs(a-b)<=tol);}
template<class F>static void reject(F f){bool rejected=false;try{f();}catch(const std::exception&){rejected=true;}check(rejected);}
static Origin origin(){Origin o;o.lat=47;o.lon=8;o.alt=500;o.ref_timestamp=100;o.z_global=true;return o;}
static Position position(){Position p;p.origin=origin();p.p={1,2,3};p.v={4,5,6};p.a={7,8,9};
  p.xy_valid=p.z_valid=p.vxy_valid=p.vz_valid=p.heading_good=true;return p;}
static Flags flags(){Flags f;f.ev_pos=f.ev_vel=f.ev_hgt=true;return f;}
static std::array<Receipt,5> receipts(){std::array<Receipt,5>a{};for(auto& r:a)r={1,1000000000ULL,1};return a;}
static Packet packet(unsigned seq){Packet p;p.sequence=seq;p.send=.005*(seq-1);p.apply=p.send+.005;p.q={.1,.2};p.dq={.3,.4};p.kp={5,6};p.kd={.2,.3};return p;}
static JointRow row(double at,unsigned last,unsigned active){JointRow r;r.sim=at;r.begin=at-.005;r.upper={1,1};r.u={.2,-.1};r.receipt={.005,.005};
  r.q_physical=r.q_internal={.1,.2};r.dq={.3,.4};r.r={.11,.21};r.last_sequence=last;r.active_sequence=active;
  const auto p=packet(last);r.last_send=p.send;r.last_apply=p.apply;r.last_q=p.q;r.last_dq=p.dq;
  const auto a=packet(active?active:1);r.active_send=a.send;r.active_apply=a.apply;r.active_q=a.q;r.active_dq=a.dq;return r;}
static JointHistory history(){JointHistory h(.005,.005);for(unsigned i=1;i<=6;++i)h.append(packet(i));h.append(row(.01,3,2));h.append(row(.02,5,4));return h;}
int main(){try{
  {EstimatorGuard g(origin());auto r=receipts();const auto s=g.read(1,1000000000ULL,position(),{1,0,0,0},{1,2,3},flags(),r);
    check(s.p==V3({2,1,497}));check(s.v==V3({5,4,-6}));check(s.a==V3({8,7,-9}));check(s.omega==V3({1,-2,-3}));
    check(s.R==M3({0,-1,0,1,0,0,0,0,1}));near(s.state_time,1);
    r[0]={2,1010000000ULL,1.01};r[4]={2,1010000000ULL,1.01};const auto held=g.read(1.01,1010000000ULL,position(),{1,0,0,0},{1,2,3},flags(),r);
    near(held.state_time,1); // Angular derivative stamp may not follow position or clock.
    auto bad=r;bad[2].sim_s=1.005;reject([&]{g.read(1.01,1010000000ULL,position(),{1,0,0,0},{1,2,3},flags(),bad);});
    auto p=position();p.origin.vz_reset=1;reject([&]{g.read(1.01,1010000000ULL,p,{1,0,0,0},{1,2,3},flags(),r);});
    p=position();p.origin.alt+=1e-3;reject([&]{g.read(1.01,1010000000ULL,p,{1,0,0,0},{1,2,3},flags(),r);});
    auto f=flags();f.gnss_vel=true;reject([&]{g.read(1.01,1010000000ULL,position(),{1,0,0,0},{1,2,3},f,r);});
    reject([&]{g.read(1.01,1010000000ULL,position(),{1.002,0,0,0},{1,2,3},flags(),r);});
    bad=r;bad[4]={3,1110000000ULL,1.11};reject([&]{g.read(1.11,1110000000ULL,position(),{1,0,0,0},{1,2,3},flags(),bad);});
    // A rejected snapshot must not poison successful guard history.
    g.read(1.01,1010000000ULL,position(),{1,0,0,0},{1,2,3},flags(),r);check(true);
  }
  {auto h=history();auto future=row(.025,6,5);future.upper={2,2};future.u={.8,.9};future.receipt={.015,.015};h.append(future);
    const auto c=h.select(.02,.01);near(c.row.sim,.02);near(c.reference_time,.01);near(c.bus_origin,-.01);
    check(c.row.q_physical==V2({.1,.2}));check(c.row.dq==V2({.3,.4}));check(c.commands.size()==1);check(c.commands[0].u==V2({.2,-.1}));
    check(c.packets.size()==2);near(c.packets[0].send,.005);near(c.packets[1].send,.01);
    const auto next=h.select(.025,.015);check(next.commands[0].u==V2({.8,.9}));near(next.commands[0].when,.005);
    check(c.commands[0].u==V2({.2,-.1})); // Owned old generation remains immutable.
    reject([&]{h.select(.019,.009);});
  }
  {auto h=history();reject([&]{h.select(.0450001,.01);});}
  {auto h=history();auto r=row(.025,6,5);r.q_physical[0]+=.0001001;h.append(r);reject([&]{h.select(.025,.02);});}
  {auto h=history();auto r=row(.025,6,5);r.last_q[1]+=.001;h.append(r);reject([&]{h.select(.025,.02);});}
  {auto h=history();auto r=row(.025,7,5);h.append(r);reject([&]{h.select(.025,.02);});}
  {auto h=history();reject([&]{h.append(row(.041,6,5));});}
  {auto h=history();auto r=row(.025,6,5);r.upper[0]=0;reject([&]{h.append(r);});}
  {auto h=history();reject([&]{h.append(packet(8));});}
  {auto h=history();auto p=packet(7);p.apply+=.000001;reject([&]{h.append(p);});}
  {auto h=history();auto r=row(.025,6,5);r.upper={2,2};r.receipt={.010,.015};r.u={.8,.9};h.append(r);
    const auto c=h.select(.025,.025);check(c.commands.size()==1);check(c.commands[0].u==V2({.8,.9}));}
  {Trajectory t;t.figure8=true;t.ax=.25;t.az=.1;t.cycles=2;t.validate();
    for(double at:{0.,1.,2.,28.,40.}){const auto s=t.sample(at,{1,2,3});check(s.v==V3({0,0,0}));check(s.a==V3({0,0,0}));near(s.p[0],1);near(s.p[2],3);}
    for(double at:{2.1,3.,4.,10.,26.5,27.5}){const double h=1e-5;const auto a=t.sample(at-h,{}),b=t.sample(at,{}),c=t.sample(at+h,{});
      for(unsigned j=0;j<3;++j){near((c.p[j]-a.p[j])/(2*h),b.v[j],1e-9);near((c.v[j]-a.v[j])/(2*h),b.a[j],1e-9);}}
    t.ramp=25;reject([&]{t.validate();});}
  {const char* path="source_core_csv_test.tmp";{std::ofstream f(path);f<<"a,b\n1,2";}
    CsvTail c(path);unsigned seen=0;check(c.poll([&](const CsvTail&){++seen;}));check(seen==0);
    {std::ofstream f(path,std::ios::app);f<<"\n3,4\n";}
    check(c.poll([&](const CsvTail& v){++seen;near(v.get("a"),seen==1?1:3);near(v.get("b"),seen==1?2:4);}));check(seen==2);
    {std::ofstream f(path,std::ios::app);f<<"5,nan\n";}reject([&]{c.poll([](const CsvTail&){});});std::remove(path);}
  reject([]{integer(-1);});reject([]{integer(.5);});
  std::cout<<"PASS "<<checks<<" independent source contract checks\n";return 0;
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
