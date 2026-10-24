#include "flyarm_mpc_cpp/config_io.hpp"
#include <functional>
#include <iostream>
#include <limits>
namespace fm=flyarm_mpc_cpp;
void require(bool ok,const char* why){if(!ok)throw std::runtime_error(why);}
template<class F>void rejects(F action){bool rejected=false;try{action();}catch(const std::exception&){rejected=true;}require(rejected,"Bad trajectory accepted");}
Json::Value fast(){Json::Value x;x["kind"]="fast_figure8";x["amplitude_x"]=.25;x["amplitude_z"]=.10;
  x["period_s"]=8.;x["cycles"]=2;x["start_s"]=2.;x["ramp_s"]=2.;x["duration_s"]=24.;return x;}
int main(){try{
  auto x=fast();require(fm::trajectory_from_json(x).is_fast(),"Fast not selected");
  Json::Value hold;hold["kind"]="hold";require(!fm::trajectory_from_json(hold).is_fast(),"Hold changed");
  for(const auto& mutation:std::vector<std::function<void(Json::Value&)>>{
    [](auto& j){j["period_s"]=12.;},[](auto& j){j["duration_s"]=32.;},[](auto& j){j["cycles"]=true;},
    [](auto& j){j["cycles"]=2.5;},[](auto& j){j["amplitude_x"]=.20;},[](auto& j){j["amplitude_z"]=-.1;},
    [](auto& j){j["start_s"]=0.;},[](auto& j){j["ramp_s"]=0.;},[](auto& j){j["extra"]=1;},
    [](auto& j){j.removeMember("period_s");},[](auto& j){j["kind"]="figure8";},[](auto& j){j["kind"]="fast";},
    [](auto& j){j["period_s"]=std::numeric_limits<double>::quiet_NaN();},[](auto& j){j["period_s"]="8";}}){
    auto bad=x;mutation(bad);rejects([&]{fm::trajectory_from_json(bad);});}
  hold["period_s"]=8.;rejects([&]{fm::trajectory_from_json(hold);});
  std::cout<<"PASS frozen fast schema and 15 explicit invalid-config cases\n";return 0;
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
