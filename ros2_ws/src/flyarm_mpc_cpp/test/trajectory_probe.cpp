#include "flyarm_mpc_cpp/trajectory_plan.hpp"
#include <iomanip>
#include <iostream>
#include <stdexcept>
// Input: case_id kind(0=hold,1=normal) t dt nodes node_dt center[3].
// Output: case_id node(-1=current) sample_time position[3] velocity[3] acceleration[3].
int main(){try{
  unsigned id;int kind,nodes;double t,dt,node_dt;x650::V3 center;
  std::cout<<std::setprecision(17);
  while(std::cin>>id>>kind>>t>>dt>>nodes>>node_dt>>center[0]>>center[1]>>center[2]){
    if(kind!=0&&kind!=1)throw std::invalid_argument("Invalid probe trajectory kind");
    const auto plan=kind?flyarm_mpc_cpp::TrajectoryPlan::normal():flyarm_mpc_cpp::TrajectoryPlan{};
    const auto emit=[&](int node,double at,const x650::Target& value){
      std::cout<<id<<' '<<node<<' '<<at;
      for(const auto& v:{value.position,value.velocity,value.acceleration})for(double n:v)std::cout<<' '<<n;
      std::cout<<'\n';
    };
    emit(-1,t,plan.sample(t,center));const auto preview=plan.preview(t,dt,nodes,node_dt,center);
    double offset=0.;for(int n=0;n<nodes;++n){offset+=(n==0?dt:node_dt);emit(n,t+offset,preview[n]);}
  }
  if(!std::cin.eof())throw std::invalid_argument("Malformed corpus");return 0;
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
