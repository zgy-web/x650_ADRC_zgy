// Standalone software-only contract test. No ROS, optimizer call or actuator output.
#define main run_original_hold_contract_tests
#include "test_worker_contract.cpp"
#undef main

namespace {
const x650::V3 center{1.2,-.3,2.1};
fm::Handoff actual_start(){auto h=start();h.target=center;h.anchor.position=center;return h;}
wire::Snapshot dynamic_snapshot(double t,double dt,std::uint64_t seq){
  auto s=snapshot(t,dt,seq);const auto target=fm::TrajectoryPlan::fast().sample(t,center);
  s.position_enu=center;s.target_position_enu=target.position;
  s.target_velocity_enu=target.velocity;s.target_acceleration_enu=target.acceleration;return s;
}
void accept(fm::Contract& contract,const wire::Snapshot& s){
  const auto p=proposal(s);contract.accept_output(p,p.compute_end_monotonic_ns);
}
void advance(fm::Contract& contract,double until,std::uint64_t& seq){
  for(double t=.02;t<until+1e-9;t+=.02){auto s=dynamic_snapshot(t,.02,++seq);
    contract.decode(s,s.issued_monotonic_ns+1000000,15,.04);accept(contract,s);}
}
}
int main(){try{
  require(run_original_hold_contract_tests()==0,"Original hold contract regressed");
  const auto trajectory=fm::TrajectoryPlan::fast();unsigned checked=0;
  for(double node_dt:{.04,.03}){
    fm::Contract contract(config(),trajectory);contract.begin(actual_start(),200);
    std::uint64_t seq=0;double t=0.;const double steps[]={.012,.016,.02,.024};
    while(t<24.-1e-10){const double dt=std::min(steps[seq%4],24.-t);t+=dt;
      auto s=dynamic_snapshot(t,dt,++seq);const auto input=contract.decode(s,s.issued_monotonic_ns+1000000,15,node_dt);
      require(input.target.position==s.target_position_enu&&input.target.velocity==s.target_velocity_enu&&
              input.target.acceleration==s.target_acceleration_enu,"Dynamic current target changed");
      require(input.preview.size()==15&&input.dt==dt,"Actual dt / explicit horizon lost");
      double offset=0.;for(unsigned i=0;i<15;++i){offset+=(i==0?dt:node_dt);auto expected=trajectory.sample(t+offset,center);
        require(input.preview[i].position==expected.position&&input.preview[i].velocity==expected.velocity&&
                input.preview[i].acceleration==expected.acceleration,"Cumulative actual-dt horizon differs");++checked;}
      require(input.context.reference==s.reference_r&&input.context.packets[0].q_ref[0]==.012,
              "Actual TS20 context changed with trajectory");accept(contract,s);
    }
  }
  {fm::Contract contract(config(),trajectory);contract.begin(actual_start(),200);
    auto s=dynamic_snapshot(.05,.05,1);const auto input=contract.decode(s,s.issued_monotonic_ns+1000000,15,.04);
    require(input.dt==.05,"Original 50-ms maximum boundary changed");accept(contract,s);}
  for(const auto& mutation:std::vector<std::function<void(wire::Snapshot&)>>{
      [](auto& s){s.target_position_enu[0]+=.000001;},
      [](auto& s){s.target_velocity_enu[2]+=.000001;},
      [](auto& s){s.target_acceleration_enu[0]+=.000001;},
      [](auto& s){const auto bad=fm::TrajectoryPlan::fast().sample(s.task_time_s+.001,center);
        s.target_position_enu=bad.position;s.target_velocity_enu=bad.velocity;s.target_acceleration_enu=bad.acceleration;},
      [](auto& s){s.dt_s=.05;},
      [](auto& s){s.packet_send_time_s[0]=s.task_time_s+1e-8;}}){
    fm::Contract contract(config(),trajectory);contract.begin(actual_start(),200);std::uint64_t seq=0;
    advance(contract,2.98,seq);auto s=dynamic_snapshot(3.,.02,++seq);mutation(s);
    rejects([&]{contract.decode(s,s.issued_monotonic_ns+1000000,15,.04);});
    require(contract.faulted(),"Dynamic target/context rejection did not latch");
  }
  for(double bad:{0.,-.001,.050000001,std::numeric_limits<double>::quiet_NaN()})
    rejects([&]{trajectory.preview(4.,bad,15,.04,center);});
  rejects([&]{trajectory.preview(4.,.02,0,.04,center);});
  rejects([&]{trajectory.preview(4.,.02,61,.04,center);});
  rejects([&]{trajectory.preview(4.,.02,15,0.,center);});
  require(trajectory.preview(19.99,.02,15,.04,center)[0].velocity==x650::V3{},"End-crossing horizon not held");
  require(fm::TrajectoryPlan{}.preview(4.,.02,15,.04,center)[5].position==center,"Default hold changed");
  std::cout<<"PASS fast dynamic contract; "<<checked<<" explicit horizon nodes; original hold regressions included\n";return 0;
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
