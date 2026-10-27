#include "acados_backend.hpp"
#include "x650_model_identity.hpp"
#include "benchmark_io.hpp"
#include <chrono>
#include <iostream>
namespace b=x650_benchmark;
int main(int argc,char** argv){try{
  b::require(argc==5,"usage: x650_standard_solver_replay original_cases.json SHA256 output.json repetitions");
  b::require(!b::fs::exists(argv[3]),"Refusing to overwrite numerical evidence");
  const int repetitions=std::stoi(argv[4]);b::require(repetitions>=1&&repetitions<=50,"Repetitions must be 1..50");
  std::ifstream stream(argv[1],std::ios::binary);b::require(bool(stream),"Cannot read bound source cases");
  const std::string bytes((std::istreambuf_iterator<char>(stream)),{});
  b::require(b::is_sha256(argv[2])&&b::sha256_bytes(bytes)==argv[2],"Original fixture SHA differs");
  const auto doc=b::parse_json(bytes);const auto config=b::config_of(doc["config"]);
  b::require(doc["cases"].isArray()&&doc["cases"].size()>0&&doc["cases"].size()<=512,"Bounded source cases required");
  x650_standard::AcadosBackend solver(config);
  Json::Value report;report["kind"]="x650_standard_cpp_builder_and_solver_v1";report["source_sha256"]=argv[2];
  report["compiled_solver_mode"]=x650_standard_generated::solver_mode;
  report["compiled_graph_sha256"]=x650_standard_generated::graph_sha256;
  report["compiled_config_sha256"]=x650_standard_generated::physical_config_sha256;
  report["scope"]="C++ actual-state/history mapping and generated C nonlinear solver; independent fixed cases, no physics or publication";
  report["records"]=Json::Value(Json::arrayValue);
  unsigned failed_cases=0, completed_samples=0;
  for(const auto& wrapper:doc["cases"]){
    const auto& original=wrapper.isMember("case")?wrapper["case"]:wrapper;
    Json::Value row;row["id"]=original["id"];row["samples"]=Json::Value(Json::arrayValue);
    try {
      const auto memory=b::memory_of(original["memory"]);const auto input=b::input_of(original["input"]);
      const auto before=std::chrono::steady_clock::now();
      const auto numeric=x650_standard::build_case(config,memory,input);
      row["case_builder_ms"]=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-before).count();
      row["x0"]=b::array_json(numeric.x0);row["parameters"]=b::array_json(numeric.parameters);
      row["initial_plan"]=b::array_json(numeric.initial_plan);row["horizon"]=b::array_json(numeric.horizon);
      for(int rep=0;rep<repetitions;++rep){
        const auto output=solver.solve(numeric);Json::Value sample;
        sample["raw_status"]=output.raw_status;sample["solver_success"]=output.solver_success;
        sample["setup_ms"]=output.setup_ms;sample["solve_ms"]=output.solve_ms;sample["postprocess_ms"]=output.postprocess_ms;
        sample["plan"]=b::array_json(output.plan);sample["states"]=b::array_json(output.states);
        sample["rollout_states"]=b::array_json(output.rollout_states);sample["residuals"]=b::array_json(output.residuals);
        sample["local_dynamics_residual"]=output.local_dynamics_residual;
        sample["rollout_disagreement"]=output.rollout_disagreement;
        sample["accepted_for_publication"]=false;row["samples"].append(std::move(sample));
        ++completed_samples;
      }
      row["completed"]=true;
    }catch(const std::exception& e){row["completed"]=false;row["error"]=e.what();++failed_cases;}
    report["records"].append(std::move(row));
    std::cout<<"fixed_case="<<original["id"].asString()<<" completed="<<report["records"][report["records"].size()-1]["completed"].asBool()<<std::endl;
  }
  report["config"]=doc["config"];report["repetitions"]=repetitions;
  report["complete"]=failed_cases==0;
  report["failed_cases"]=failed_cases;report["completed_samples"]=completed_samples;
  report["accepted_for_publication"]=false;
  std::ofstream output(argv[3],std::ios::binary);b::require(bool(output),"Cannot create output");
  Json::StreamWriterBuilder writer;writer["indentation"]="  ";writer["precision"]=17;
  output<<Json::writeString(writer,report)<<'\n';output.close();b::require(bool(output),"Evidence write failed");return failed_cases==0?0:2;
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 2;}}
