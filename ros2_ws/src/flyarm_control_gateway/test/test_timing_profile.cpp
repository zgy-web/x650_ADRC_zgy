#include "flyarm_control_gateway/admission_cadence.hpp"
#include "flyarm_control_gateway/periodic_gate.hpp"
#include <iostream>
#include <sstream>
#include <stdexcept>
namespace f=flyarm_control_gateway;
namespace {
constexpr std::uint64_t B=1'000'000'000,MS=1'000'000;
void check(bool ok,const char* why){if(!ok)throw std::runtime_error(why);}
void observe(f::PeriodicGate& g,std::uint64_t seq,std::uint64_t received,std::uint64_t issued,std::uint64_t now){
 g.source(seq,received,issued,now,10.+double(issued-B)/1e9);}
bool adopt(f::PeriodicGate& g,std::uint64_t seq,std::uint64_t issued,std::uint64_t now){return g.accept(seq,issued,now,10.+double(issued-B)/1e9);}
f::Config config(bool multirate){f::Config c;c.session_id=23;c.epoch=1;c.configuration_sha256.fill(7);
 c.position_gains={1.,1.,1.};if(multirate)c.timing_profile=f::TimingProfile::from_name("mpc25_dispatch50_v1");return c;}
f::Snapshot snapshot(double dt){f::Snapshot s;const auto c=config(false);s.session_id=c.session_id;s.epoch=c.epoch;s.configuration_sha256=c.configuration_sha256;
 s.sequence=1;s.issued_monotonic_ns=B;s.control_time_s=10.+dt;s.task_time_s=1.+dt;s.dt_s=dt;
 s.state_time_s=s.joint_time_s=s.control_time_s;s.rotation_row_major={1.,0.,0.,0.,1.,0.,0.,0.,1.};s.position_gains=c.position_gains;
 s.reference_time_s=s.task_time_s;s.command_count=s.packet_count=1;s.command_time_s[0]=s.packet_send_time_s[0]=.9;
 s.packet_kp[0]=s.packet_kp[1]=6.;s.packet_kd[0]=s.packet_kd[1]=.25;return s;}
f::Proposal proposal(const f::Snapshot& s){f::Proposal p;p.session_id=s.session_id;p.epoch=s.epoch;p.configuration_sha256=s.configuration_sha256;
 p.sequence=p.snapshot_sequence=s.sequence;p.control_time_s=s.control_time_s;p.task_time_s=s.task_time_s;
 p.compute_start_monotonic_ns=B+MS;p.compute_end_monotonic_ns=B+50*MS;p.accepted_optimum=p.nonlinear_check_passed=true;p.subproblems=1;return p;}
void profiles(){
 const f::TimingProfile old;const auto fresh=config(true).timing_profile;
 check(std::string(old.name())=="legacy_v1"&&old.command_lease_ns()==50*MS&&old.control_dt_max_s()==.05&&old.nominal_solve_period_ns()==20*MS,"legacy values changed");
 check(fresh.command_lease_ns()==160*MS&&fresh.control_dt_max_s()==.1&&fresh.nominal_solve_period_ns()==40*MS,"explicit multirate values differ");
 check(fresh.output_period_ns()==old.output_period_ns()&&fresh.solve_roundtrip_max_ns()==50*MS&&fresh.source_wall_age_max_ns()==25*MS&&fresh.consumption_progress_max_ns()==40*MS,"unrelated time gates changed");
 bool rejected=false;try{(void)f::TimingProfile::from_name("");}catch(const std::invalid_argument&){rejected=true;}check(rejected,"unknown profile accepted");
 std::ostringstream out;f::write_timing_profile(out,fresh);check(out.str().find("\"command_lease_ns\":160000000")!=std::string::npos&&out.str().find("\"hard_realtime_qualified\":false")!=std::string::npos,"effective profile report incomplete");
}
void cadence(){
 f::AdmissionCadence c(config(true).timing_profile);
 check(!c.due(true,B+39*MS,B+40*MS,10.04,10.),"wall cadence ignored");
 check(!c.due(true,B+40*MS,B+40*MS,10.036,10.),"first actual 40ms ignored");
 check(c.due(true,B+40*MS,B+40*MS,10.04,10.),"first 40ms rejected");c.admitted(1);
 check(!c.due(true,B+80*MS,B+80*MS,10.08,10.04),"unsent plan released next solve");
 check(c.acknowledge_output(1),"actual first send not released");
 check(!c.due(true,B+79*MS,B+80*MS,10.08,10.04),"next wall deadline ignored");
 check(!c.due(true,B+80*MS,B+80*MS,10.04,10.04),"held sample reused");
 check(c.due(true,B+101*MS,B+80*MS,10.10,10.04),"late actual 60ms sample rejected");
 c.admitted(2);check(c.acknowledge_output(2),"late output release failed");
 // The caller schedules from actual 101ms admission, not the missed 80ms slot.
 check(!c.due(true,B+120*MS,B+141*MS,10.12,10.10),"catch-up burst admitted");
 check(c.due(true,B+141*MS,B+141*MS,10.14,10.10),"reanchored deadline rejected");
 check(!c.acknowledge_output(2),"one output used twice");
}
void admission(){
 {f::ShadowGate old(config(false));const auto a=old.admit(snapshot(.06),B);check(a.event&&a.event->reason==f::Reason::DEADLINE,"legacy dt widened");}
 {f::ShadowGate next(config(true));check(next.admit(snapshot(.1),B+MS).forward,"exact100ms actual dt rejected");auto p=proposal(snapshot(.1));const auto accepted=next.propose(p,B+50*MS);check(accepted&&accepted->accepted,"exact50ms computation rejected");}
 {const double actual_dt=42.4-42.3;check(actual_dt>.1,"test did not expose binary64 subtraction roundoff");f::ShadowGate next(config(true));auto s=snapshot(actual_dt);check(next.admit(s,B).forward&&s.dt_s==actual_dt,"100ms clock difference was rejected or rewritten");}
 {f::ShadowGate next(config(true));const auto a=next.admit(snapshot(.100000001),B);check(a.event&&a.event->reason==f::Reason::DEADLINE,"over100ms dt admitted");}
 {f::ShadowGate next(config(true));check(next.admit(snapshot(.06),B+MS).forward,"60ms actual dt rejected");auto p=proposal(snapshot(.06));p.compute_end_monotonic_ns=B+50*MS+1;const auto e=next.propose(p,B+50*MS+1);check(e&&!e->accepted&&e->reason==f::Reason::DEADLINE,"lease widened solve deadline");}
 {f::ShadowGate next(config(true));auto s=snapshot(.06);s.joint_time_s=s.control_time_s-.025-1e-8;s.reference_time_s=s.joint_time_s-(s.control_time_s-s.task_time_s);const auto a=next.admit(s,B);check(a.event&&a.event->reason==f::Reason::SOURCE_AGE,"joint freshness widened");}
}
void output(){
 const auto profile=config(true).timing_profile;f::PeriodicGate g(profile);observe(g,1,B,B,B);check(adopt(g,1,B,B),"setup command rejected");
 for(std::uint64_t k=0;k<=8;++k){const auto at=B+k*20*MS;if(k)observe(g,k+1,at,at,at);const auto t=g.tick(at,at);
  check(t.valid&&t.issued==B&&t.age==k*20*MS&&(k==0||t.repeated),"repeat renewed lease or refused valid hold");}
 const auto expiry=B+160*MS+1;observe(g,10,expiry,expiry,expiry);check(g.tick(expiry,expiry).reason==f::PeriodicGate::STALE_COMMAND,"160ms+1ns did not stop");
 f::PeriodicGate source(profile);observe(source,1,B,B,B);check(adopt(source,1,B,B),"source setup failed");check(source.tick(B+25*MS+1,B+25*MS+1).reason==f::PeriodicGate::STALE_SOURCE,"source25ms widened");
 f::PeriodicGate late(profile);observe(late,1,B,B,B);check(adopt(late,1,B,B),"late setup failed");observe(late,2,B+20*MS+1,B+20*MS+1,B+20*MS+1);
 check(late.tick(B,B+20*MS+1).reason==f::PeriodicGate::MISSED_DEADLINE,"output20ms late bound widened");
 // Actual first-flight shape: 454 first sent at42.67ms, repeated at62.71ms,
 // 455 arrives at~77ms. The new lease permits that genuine repeat, without
 // modifying either issue time or the 20ms output grid.
 f::PeriodicGate recorded(profile);observe(recorded,1,B+32*MS,B+31*MS,B+32*MS);check(adopt(recorded,454,B,B+32*MS),"recorded454 reject");
 observe(recorded,2,B+42*MS,B+41*MS,B+42*MS);check(recorded.tick(B+42*MS,B+42*MS).valid,"454 first send reject");
 observe(recorded,3,B+62*MS,B+61*MS,B+62*MS);const auto repeat=recorded.tick(B+62*MS,B+62*MS);check(repeat.valid&&repeat.repeated&&repeat.age==62*MS,"recorded repeat changed issue");
 check(adopt(recorded,455,B+41*MS,B+77*MS),"recorded455 reject");observe(recorded,4,B+82*MS,B+81*MS,B+82*MS);
 check(recorded.tick(B+82*MS,B+82*MS).command_sequence==455,"next real candidate not adopted");
 // Faster physical simulation must not consume the wall lease as extra
 // physical motion. Both limits apply independently and retain source stamps.
 f::PeriodicGate physical(profile);physical.source(1,B,B,B,10.);check(physical.accept(1,B,B,10.),"physical setup failed");
 physical.source(2,B+20*MS,B+20*MS,B+20*MS,10.+.16);check(physical.tick(B+20*MS,B+20*MS).valid,"exact physical hold boundary rejected");
 physical.source(3,B+21*MS,B+21*MS,B+21*MS,10.+.16+1e-9);
 check(physical.tick(B+21*MS,B+21*MS).reason==f::PeriodicGate::STALE_COMMAND,"fast physical clock escaped hold guard");
 f::PeriodicGate slow(profile);slow.source(1,B,B,B,10.);check(slow.accept(1,B,B,10.),"slow setup failed");
 slow.source(2,expiry,expiry,expiry,10.04);check(slow.tick(expiry,expiry).reason==f::PeriodicGate::STALE_COMMAND,"slow physical clock escaped wall lease");
 f::PeriodicGate missing(profile);missing.source(1,B,B,B);check(missing.faulted(),"new profile accepted missing physical clock");
 f::PeriodicGate missing_command(profile);missing_command.source(1,B,B,B,10.);check(!missing_command.accept(1,B,B),"new profile accepted missing command control time");
 f::PeriodicGate stale_candidate(profile);stale_candidate.source(1,B,B,B,10.17);check(!stale_candidate.accept(1,B,B,10.),"physically stale candidate admitted despite fresh wall issue");
 f::PeriodicGate backwards(profile);backwards.source(1,B,B,B,10.);backwards.source(2,B+MS,B+MS,B+MS,9.99);check(backwards.faulted(),"physical clock regressed");
}
}
int main(){profiles();cadence();std::cout<<"PASS multirate_cadence\n";admission();output();std::cout<<"PASS lease_repetition\nPASS timeout_and_stale_source\n";}
