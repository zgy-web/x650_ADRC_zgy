#include "flyarm_mpc_cpp/config_io.hpp"
#include "flyarm_mpc_cpp/diagnostics_json.hpp"
#include "flyarm_mpc_cpp/bounded_record_queue.hpp"
#include "flyarm_mpc_cpp/thread_placement.hpp"
#include <functional>
#include <iostream>
#include <limits>
#include <unistd.h>
#include <thread>

namespace fm=flyarm_mpc_cpp;
void require(bool condition,const char* text){if(!condition)throw std::runtime_error(text);}
template<class F>void rejects(F action){bool rejected=false;try{action();}catch(const std::exception&){rejected=true;}require(rejected,"Invalid configuration/evidence was accepted");}
Json::Value config_json(){x650::Config c;Json::Value j;
#define FIELD(name) j[#name]=c.name
 FIELD(nodes);FIELD(max_iterations);FIELD(node_dt);FIELD(max_dt);FIELD(feasibility_tolerance);FIELD(kkt_tolerance);FIELD(arm_share);FIELD(joint_position_gain);FIELD(bus_period);FIELD(integration_step);FIELD(trust_radius);
#undef FIELD
#define FIELD(name) j[#name]=fm::numeric_json(c.name)
 FIELD(weights);FIELD(velocity_limit);FIELD(acceleration_limit);FIELD(q_lower);FIELD(q_upper);FIELD(physical_lower);FIELD(physical_upper);FIELD(geometry);FIELD(response);FIELD(base_position_gain);FIELD(motion);FIELD(kp);FIELD(kd);
#undef FIELD
 return j;
}
int main(){fm::fs::path folder=fm::fs::temp_directory_path()/std::string("flyarm_cpp_config_test_").append(std::to_string(getpid()));
 try{
  rejects([]{fm::ThreadPlacement p(std::int64_t(1)<<32,{1,2});});
  rejects([]{fm::ThreadPlacement p(1,{1,2});});
  rejects([]{fm::ThreadPlacement p(1,{2,2});});
  rejects([]{fm::ThreadPlacement p(1,{-1,2});});
  {fm::BoundedRecordQueue<std::unique_ptr<int>,2> queue;auto a=std::make_unique<int>(1),b=std::make_unique<int>(2),c=std::make_unique<int>(3);std::unique_ptr<int> received;
   require(queue.push(std::move(a))&&queue.push(std::move(b))&&!a&&!b,"Queue did not transfer ownership");
   require(!queue.push(std::move(c))&&c&&*c==3,"Full queue moved or lost the rejected record");
   require(queue.pop(received)&&*received==1&&queue.push(std::move(c))&&queue.pop(received)&&*received==2&&queue.pop(received)&&*received==3&&!queue.pop(received),"Queue order/full ownership differs");}
  {fm::BoundedRecordQueue<std::unique_ptr<unsigned>,4> queue;bool ordered=true;std::thread consumer([&]{for(unsigned i=0;i<10000;++i){std::unique_ptr<unsigned> received;while(!queue.pop(received))std::this_thread::yield();if(!received||*received!=i)ordered=false;}});
   for(unsigned i=0;i<10000;++i){auto record=std::make_unique<unsigned>(i);while(!queue.push(std::move(record)))std::this_thread::yield();}
   consumer.join();require(ordered,"Concurrent single-producer/consumer records changed");}
  const auto original=config_json();auto c=fm::config_from_json(original);require(c.node_dt==.04&&c.nodes==15&&c.velocity_limit[4]==.35,"Explicit config values changed");
  for(const auto& mutation:std::vector<std::function<void(Json::Value&)>>{
   [](auto& j){j.removeMember("motion");},[](auto& j){j["implicit_default"]=1;},[](auto& j){j["nodes"]=true;},
   [](auto& j){j["kp"].append(6.);},[](auto& j){j["max_dt"]=.051;},[](auto& j){j["feasibility_tolerance"]=2.1e-7;},
   [](auto& j){j["kkt_tolerance"]=.003;},[](auto& j){j["motion"][0]=std::numeric_limits<double>::quiet_NaN();}}){auto j=original;mutation(j);rejects([&]{fm::config_from_json(j);});}
  require(fm::fs::create_directory(folder),"Fresh test directory required");
  const auto source=folder/"source.txt";{std::ofstream f(source);f<<"bound artifact";}
  Json::Value bound;bound["path"]=source.string();bound["sha256"]=fm::sha256(source);require(fm::bound_path(bound)==fm::fs::canonical(source),"Artifact binding changed");
  auto wrong=bound;wrong["sha256"]=std::string(64,'0');rejects([&]{fm::bound_path(wrong);});
  const auto duplicate=folder/"duplicate.json";{std::ofstream f(duplicate);f<<"{\"x\":1,\"x\":2}";}rejects([&]{fm::read_json(duplicate);});
  Json::Value startup;startup["schema_version"]=1;startup["kind"]="cpp_full_pipeline_v1";startup["simulation_only"]=true;
  startup["task_config"]=startup["core_library"]=startup["optimization_library"]=startup["predictor_library"]=bound;startup["predictor_library"]["symbol"]="ts20_prediction_batch";
  startup["source_artifacts"].append(bound);startup["config"]=original;startup["trajectory"]["kind"]="hold";
  const auto good=folder/"startup.json";fm::write_json(good,startup);auto loaded=fm::load_startup(good,fm::sha256(good));require(loaded.config.nodes==15,"Startup config changed");
  unsigned index=0;for(const auto& mutation:std::vector<std::function<void(Json::Value&)>>{
   [](auto& j){j["simulation_only"]=1;},[](auto& j){j.removeMember("optimization_library");},[](auto& j){j["trajectory"]["kind"]="figure8";},
   [](auto& j){j["predictor_library"]["symbol"]="replacement";},[](auto& j){j["source_artifacts"].append(j["source_artifacts"][0]);}}){
    auto j=startup;mutation(j);auto p=folder/("reject_"+std::to_string(++index)+".json");fm::write_json(p,j);rejects([&]{fm::load_startup(p,fm::sha256(p));});}
  x650::StepResult result;result.diagnostics.candidate_packets.push_back({});result.diagnostics.candidate_packets[0].q_ref={.123,-.234};
  result.diagnostics.candidate_states={1,2,3};result.diagnostics.C={4,5};result.diagnostics.attempts.push_back({true,true,.01,.02,.3});
  const auto evidence=fm::result_json(result);result.diagnostics.candidate_packets[0].q_ref[0]=9.;
  require(evidence["diagnostics"]["candidate_packets"][0]["q_ref"][0].asDouble()==.123&&evidence["diagnostics"]["C"][1].asDouble()==5.,"Actual native evidence missing or aliased");
  result.diagnostics.compute_ms=std::numeric_limits<double>::infinity();rejects([&]{fm::result_json(result);});
  fm::fs::remove_all(folder);std::cout<<"Strict startup/source binding and actual native diagnostics contracts passed\n";return 0;
 }catch(const std::exception& e){std::cerr<<e.what()<<'\n';if(fm::fs::exists(folder))fm::fs::remove_all(folder);return 1;}}
