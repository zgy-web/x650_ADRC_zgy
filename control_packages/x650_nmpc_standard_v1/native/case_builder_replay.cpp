#include "case_builder.hpp"
#include "benchmark_io.hpp"
#include <iostream>
namespace b=x650_benchmark;
int main(int argc,char** argv){try{
 b::require(argc==3,"usage: x650_standard_case_replay original_fixture.json new_output.json");
 b::require(!b::fs::exists(argv[2]),"Refusing to overwrite numerical-case evidence");
 std::ifstream input(argv[1],std::ios::binary);b::require(bool(input),"Cannot open original fixture");
 const std::string bytes((std::istreambuf_iterator<char>(input)),{});auto data=b::parse_json(bytes);
 const auto config=b::config_of(data["config"]);b::require(data["cases"].isArray()&&!data["cases"].empty()&&data["cases"].size()<=512,"Bounded fixture cases required");
 Json::Value report;report["kind"]="x650_standard_cpp_case_mapping_v1";report["input_sha256"]=b::sha256_bytes(bytes);
 report["history_rk4_substeps_per_bus"]=20;report["future_rk4_substeps_per_bus"]=10;
 report["velocity_smoothing_rad_s"]=.05;report["states"]=23;report["controls"]=5;report["stage_parameters"]=161;
 report["records"]=Json::Value(Json::arrayValue);int passed=0;
 for(const auto& wrapper:data["cases"]){const auto& row=wrapper.isMember("case")?wrapper["case"]:wrapper;
  Json::Value result;result["id"]=b::string(row["id"]);
  try{const auto memory=b::memory_of(row["memory"]);const auto input=b::input_of(row["input"]);
   auto numerical=x650_standard::build_case(config,memory,input);result["x0"]=b::array_json(numerical.x0);
   result["parameters"]=b::array_json(numerical.parameters);result["initial_plan"]=b::array_json(numerical.initial_plan);
   result["horizon"]=b::array_json(numerical.horizon);result["built"]=true;++passed;
  }catch(const std::exception& e){result["built"]=false;result["reason"]=e.what();}
  report["records"].append(std::move(result));
 }
 report["count"]=data["cases"].size();report["built_count"]=passed;report["all_built"]=passed==int(data["cases"].size());
 std::ofstream output(argv[2],std::ios::binary);b::require(bool(output),"Cannot create output");
 Json::StreamWriterBuilder writer;writer["indentation"]="  ";writer["precision"]=17;
 output<<Json::writeString(writer,report)<<'\n';output.close();b::require(bool(output),"Output write failed");
 std::cout<<"built="<<passed<<'/'<<data["cases"].size()<<'\n';return 0;
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 2;}}
