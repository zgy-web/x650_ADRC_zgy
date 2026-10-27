#include "x650_mpc/controller.hpp"
#include "x650_mpc/selected_profile.hpp"
#include "benchmark_io.hpp"
#include <dlfcn.h>
#include <fstream>
#include <iostream>
#include <tuple>
extern "C" int ts20_prediction_batch(const double*,const double*,double,double,const double*,const double*,const double*,int,const double*,int,int,double*,int*,int*,double*,double*,int,int*,int*,char*,int);
extern "C" unsigned x650_optimization_abi_version();
namespace b=x650_benchmark;
bool same_config(const x650::Config& a,const x650::Config& b){
 const auto fields=[](const x650::Config& c){return std::tie(c.nodes,c.max_iterations,c.node_dt,c.max_dt,c.command_hold_s,c.feasibility_tolerance,c.kkt_tolerance,c.arm_share,c.joint_position_gain,c.weights,c.velocity_limit,c.acceleration_limit,c.q_lower,c.q_upper,c.physical_lower,c.physical_upper,c.geometry,c.response,c.base_position_gain,c.motion,c.kp,c.kd,c.bus_period,c.integration_step,c.trust_radius);};
 return fields(a)==fields(b);
}
std::array<std::vector<double>,10> fields(const x650::StepResult& r){
 std::array<std::vector<double>,10> out;std::size_t at=0;
 const auto add=[&](const auto& x){out[at++]={x.begin(),x.end()};};
 add(r.command);add(r.position_reference);add(r.velocity_feedforward);add(r.acceleration_feedforward);
 add(r.joint_velocity);add(r.joint_reference);add(r.joint_velocity_reference);add(r.joint_acceleration_reference);add(r.nominal_acceleration);add(r.tip_actual);return out;
}
Json::Value loaded(void* symbol){Dl_info info{};b::require(dladdr(symbol,&info)&&info.dli_fname,"Cannot resolve loaded library");
 const auto path=b::fs::canonical(info.dli_fname);Json::Value out;out["path"]=path.string();out["sha256"]=b::sha256_file(path);return out;}
int main(int argc,char** argv){Json::Value report;report["kind"]="x650_mpc_callable_replay_v1";report["physical_validation"]=false;report["complete"]=false;
 try{
  b::require(argc==3,"usage: x650_mpc_replay input.json input_sha256");b::require(b::is_sha256(argv[2])&&b::sha256_file(argv[1])==argv[2],"Input SHA differs");
  std::ifstream stream(argv[1],std::ios::binary);std::string text((std::istreambuf_iterator<char>(stream)),{});
  const auto data=b::dataset_of(b::parse_json(text));
  report["selected_profile_matches_recording"]=same_config(data.config,x650_mpc::selected_x650_simulation_profile());
  b::require(report["selected_profile_matches_recording"].asBool(),"Selected X650 profile differs from original recorded Config");
  x650_mpc::Controller controller(data.config,ts20_prediction_batch,x650::DiagnosticMode::Full);
  x650::Core direct(data.config,ts20_prediction_batch,x650::DiagnosticMode::Full);
  report["input_sha256"]=argv[2];report["core_abi"]=x650_mpc_core_abi_version();
  report["loaded_core"]=loaded(reinterpret_cast<void*>(&x650_mpc_core_abi_version));
  report["loaded_optimization"]=loaded(reinterpret_cast<void*>(&x650_optimization_abi_version));
  report["loaded_predictor"]=loaded(reinterpret_cast<void*>(&ts20_prediction_batch));
  report["rows"]=Json::Value(Json::arrayValue);bool passed=true;
  for(const auto& row:data.cases){
   controller.restore_for_replay(row.memory);direct.restore(row.memory);
   const auto expected=direct.step(row.input);const auto& candidate=controller.prepare(row.input);
   const auto actual_fields=fields(candidate.output),expected_fields=fields(expected);
   const auto& a=candidate.output.diagnostics;const auto& d=expected.diagnostics;
   const bool exact=actual_fields==expected_fields&&a.A==d.A&&a.b==d.b&&a.H==d.H&&a.g==d.g&&a.C==d.C&&a.lower==d.lower&&a.upper==d.upper&&a.solution==d.solution&&a.plan==d.plan&&a.predicted_states==d.predicted_states&&a.candidate_envelope==d.candidate_envelope&&a.hold_guard.envelope==d.hold_guard.envelope;
   const bool before=b::same_memory(controller.committed_memory(),row.memory);
   bool oracle=true;double error=0.;
   for(std::size_t i=0;i<row.oracle.fields.size();++i)for(std::size_t j=0;j<row.oracle.fields[i].size();++j){
    const double e=std::abs(actual_fields[i][j]-row.oracle.fields[i][j]);error=std::max(error,e);oracle=oracle&&e<=(i==1?1e-9:1e-7);
   }
   // Replay simulates the application publication decision. It is not a
   // transport action or evidence that a real actuator consumed this output.
   const auto command=candidate.output.command;controller.commit_published(candidate.token,command);
   const bool after=b::same_memory(controller.committed_memory(),direct.memory());
   Json::Value item;item["id"]=row.id;item["facade_vs_direct_exact"]=exact;item["prepare_did_not_commit"]=before;
   item["commit_memory_exact"]=after;item["historical_oracle_match"]=oracle;item["historical_oracle_max_error"]=error;
   item["passed"]=exact&&before&&after&&oracle;passed=passed&&item["passed"].asBool();report["rows"].append(item);
  }
  report["row_count"]=Json::UInt64(data.cases.size());report["complete"]=true;report["passed"]=passed;
  report["scope"]="Offline fixed recorded inputs and saved Memory: original library versus facade. Simulated publication commit only; no ROS, vehicle, flight, new controller tuning or closed-loop equivalence claim.";
 }catch(const std::exception& e){report["error"]=e.what();report["passed"]=false;}
 Json::StreamWriterBuilder writer;writer["indentation"]="  ";std::cout<<Json::writeString(writer,report)<<'\n';return report["passed"].asBool()?0:1;
}
