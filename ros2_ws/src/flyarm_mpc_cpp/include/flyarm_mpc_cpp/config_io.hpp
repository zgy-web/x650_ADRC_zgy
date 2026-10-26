#pragma once
#include "mpc_core.hpp"
#include "flyarm_mpc_cpp/trajectory_plan.hpp"
#include <json/json.h>
#include <openssl/evp.h>
#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <memory>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace flyarm_mpc_cpp {
namespace fs=std::filesystem;
inline std::string sha256(const fs::path& path) {
  std::ifstream input(path,std::ios::binary);if(!input)throw std::runtime_error("Cannot open bound artifact: "+path.string());
  std::unique_ptr<EVP_MD_CTX,decltype(&EVP_MD_CTX_free)> context(EVP_MD_CTX_new(),EVP_MD_CTX_free);
  if(!context || EVP_DigestInit_ex(context.get(),EVP_sha256(),nullptr)!=1)throw std::runtime_error("SHA initialization failed");
  std::array<char,65536> buffer{};
  while(input){input.read(buffer.data(),buffer.size());if(EVP_DigestUpdate(context.get(),buffer.data(),input.gcount())!=1)throw std::runtime_error("SHA update failed");}
  if(!input.eof())throw std::runtime_error("Artifact read failed");
  std::array<unsigned char,EVP_MAX_MD_SIZE> digest{};unsigned size{};
  if(EVP_DigestFinal_ex(context.get(),digest.data(),&size)!=1 || size!=32)throw std::runtime_error("SHA finalization failed");
  std::ostringstream result;result<<std::hex<<std::setfill('0');for(unsigned i=0;i<size;++i)result<<std::setw(2)<<unsigned(digest[i]);return result.str();
}
inline std::string string_field(const Json::Value& value,const char* key) {
  if(!value.isMember(key)||!value[key].isString()||value[key].asString().empty())throw std::invalid_argument(std::string("Explicit string required: ")+key);
  return value[key].asString();
}
inline double number(const Json::Value& value) {
  if(!value.isNumeric()||value.isBool()||!std::isfinite(value.asDouble()))throw std::invalid_argument("Finite numeric configuration required");
  return value.asDouble();
}
inline int integer(const Json::Value& value) {
  if(!value.isInt()||value.isBool())throw std::invalid_argument("Explicit integer configuration required");
  return value.asInt();
}
template<std::size_t N> std::array<double,N> numbers(const Json::Value& value) {
  if(!value.isArray()||value.size()!=N)throw std::invalid_argument("Fixed-size configuration array differs");
  std::array<double,N> output{};for(std::size_t i=0;i<N;++i)output[i]=number(value[Json::ArrayIndex(i)]);return output;
}
inline void exact_keys(const Json::Value& value,std::initializer_list<const char*> names) {
  if(!value.isObject())throw std::invalid_argument("Configuration must be an object");
  std::set<std::string> expected;for(auto name:names)expected.insert(name);
  auto members=value.getMemberNames();if(std::set<std::string>(members.begin(),members.end())!=expected)throw std::invalid_argument("Configuration fields differ from explicit C++ schema");
}
inline fs::path bound_path(const Json::Value& value) {
  const auto path=fs::path(string_field(value,"path"));const auto hash=string_field(value,"sha256");
  if(!path.is_absolute()||hash.size()!=64||hash.find_first_not_of("0123456789abcdef")!=std::string::npos||sha256(path)!=hash)
    throw std::invalid_argument("Artifact path/SHA binding differs: "+path.string());
  return fs::canonical(path);
}
inline Json::Value read_json(const fs::path& path) {
  if(fs::file_size(path)>8*1024*1024)throw std::invalid_argument("Startup JSON exceeds bounded size");
  std::ifstream input(path);Json::CharReaderBuilder reader;reader["rejectDupKeys"]=true;reader["failIfExtra"]=true;
  Json::Value value;std::string errors;if(!Json::parseFromStream(reader,input,&value,&errors))throw std::invalid_argument("Invalid startup JSON: "+errors);return value;
}
inline void write_json(const fs::path& path,const Json::Value& value) {
  if(fs::exists(path))throw std::runtime_error("Evidence must not be overwritten: "+path.string());
  std::ofstream output(path);if(!output)throw std::runtime_error("Cannot create evidence");Json::StreamWriterBuilder writer;writer["indentation"]="";
  output<<Json::writeString(writer,value)<<'\n';if(!output)throw std::runtime_error("Evidence write failed");
}
inline TrajectoryPlan trajectory_from_json(const Json::Value& value) {
  const auto kind=string_field(value,"kind");
  if(kind=="hold"){exact_keys(value,{"kind"});return {};}
  if(kind!="normal_figure8" && kind!="fast_figure8")
    throw std::invalid_argument("Only hold or explicitly frozen normal/fast figure-eight admitted");
  exact_keys(value,{"kind","amplitude_x","amplitude_z","period_s","cycles","start_s","ramp_s","duration_s"});
  const bool fast=kind=="fast_figure8";
  if(number(value["amplitude_x"])!=.25 || number(value["amplitude_z"])!=.10 ||
     number(value["period_s"])!=(fast?8.:12.) || integer(value["cycles"])!=2 ||
     number(value["start_s"])!=2. || number(value["ramp_s"])!=2. ||
     number(value["duration_s"])!=(fast?24.:32.))
    throw std::invalid_argument("Frozen figure-eight stage parameters differ");
  return fast?TrajectoryPlan::fast():TrajectoryPlan::normal();
}
struct Startup {
  Json::Value manifest;
  x650::Config config;
  TrajectoryPlan trajectory;
  fs::path core_library,optimization_library,predictor_library;
  std::string task_hash;
};
inline x650::Config config_from_json(const Json::Value& c) {
  x650::Config v;
  auto legacy=c;legacy.removeMember("command_hold_s");
  exact_keys(legacy,{"nodes","max_iterations","node_dt","max_dt","feasibility_tolerance","kkt_tolerance","arm_share","joint_position_gain","weights","velocity_limit","acceleration_limit","q_lower","q_upper","physical_lower","physical_upper","geometry","response","base_position_gain","motion","kp","kd","bus_period","integration_step","trust_radius"});
  if(c.isMember("command_hold_s"))v.command_hold_s=number(c["command_hold_s"]);
  v.nodes=integer(c["nodes"]);v.max_iterations=integer(c["max_iterations"]);
#define SCALAR(name) v.name=number(c[#name])
  SCALAR(node_dt);SCALAR(max_dt);SCALAR(feasibility_tolerance);SCALAR(kkt_tolerance);SCALAR(arm_share);SCALAR(joint_position_gain);SCALAR(bus_period);SCALAR(integration_step);SCALAR(trust_radius);
#undef SCALAR
#define ARRAY(name,n) v.name=numbers<n>(c[#name])
  ARRAY(weights,6);ARRAY(velocity_limit,5);ARRAY(acceleration_limit,5);ARRAY(q_lower,2);ARRAY(q_upper,2);ARRAY(physical_lower,2);ARRAY(physical_upper,2);ARRAY(geometry,10);ARRAY(response,5);ARRAY(base_position_gain,3);ARRAY(motion,8);ARRAY(kp,2);ARRAY(kd,2);
#undef ARRAY
  const bool old=v.max_dt==.05&&v.command_hold_s==.05;
  const bool multirate=v.max_dt==.1&&v.command_hold_s==.16&&v.nodes==15&&v.node_dt==.04&&v.bus_period==.005;
  if((!old&&!multirate)||v.feasibility_tolerance!=2e-7||v.kkt_tolerance!=.002)throw std::invalid_argument("Explicit timing profile or original quality gates differ");
  return v;
}
inline Startup load_startup(const fs::path& path,const std::string& expected) {
  Json::Value identity;identity["path"]=path.string();identity["sha256"]=expected;bound_path(identity);
  Startup output;output.manifest=read_json(path);const auto& root=output.manifest;
  auto original=root;original.removeMember("timing_profile");
  exact_keys(original,{"schema_version","kind","simulation_only","task_config","core_library","optimization_library","predictor_library","source_artifacts","config","trajectory"});
  if(integer(root["schema_version"])!=1||root["kind"]!="cpp_full_pipeline_v1"||root["simulation_only"]!=true)
    throw std::invalid_argument("Explicit simulation-only full C++ startup required");
  exact_keys(root["task_config"],{"path","sha256"});bound_path(root["task_config"]);output.task_hash=string_field(root["task_config"],"sha256");
  exact_keys(root["core_library"],{"path","sha256"});output.core_library=bound_path(root["core_library"]);
  exact_keys(root["optimization_library"],{"path","sha256"});output.optimization_library=bound_path(root["optimization_library"]);
  exact_keys(root["predictor_library"],{"path","sha256","symbol"});output.predictor_library=bound_path(root["predictor_library"]);
  if(root["predictor_library"]["symbol"]!="ts20_prediction_batch")throw std::invalid_argument("Frozen predictor symbol differs");
  if(!root["source_artifacts"].isArray()||root["source_artifacts"].empty())throw std::invalid_argument("Source artifact bindings are required");
  std::set<std::string> sources;for(const auto& item:root["source_artifacts"]){exact_keys(item,{"path","sha256"});if(!sources.insert(bound_path(item).string()).second)throw std::invalid_argument("Duplicate source binding");}
  output.trajectory=trajectory_from_json(root["trajectory"]);
  output.config=config_from_json(root["config"]);
  if(output.config.command_hold_s>.05){if(string_field(root,"timing_profile")!="mpc25_dispatch50_v1")throw std::invalid_argument("Bound multirate startup profile required");}
  else if(root.isMember("timing_profile")&&string_field(root,"timing_profile")!="legacy_v1")throw std::invalid_argument("Legacy startup profile differs");
  return output;
}
}
