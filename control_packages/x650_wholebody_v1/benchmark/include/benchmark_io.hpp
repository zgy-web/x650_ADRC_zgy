#pragma once
// The standalone input adapter is intentionally separate from ROS config_io.
// All JSON parsing, source hashing and typed construction happens before timing.
#include "mpc_core.hpp"
#include <json/json.h>
#include <openssl/evp.h>
#include <algorithm>
#include <array>
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

namespace x650_benchmark {
namespace fs=std::filesystem;
constexpr std::size_t max_input_bytes=32*1024*1024;
constexpr std::size_t max_cases=512, max_samples=100000;
inline void require(bool condition,const std::string& reason) {
  if(!condition)throw std::invalid_argument(reason);
}
inline void exact_keys(const Json::Value& v,std::initializer_list<const char*> names) {
  require(v.isObject(),"Object required");std::set<std::string> expected;
  for(const auto* name:names)expected.insert(name);
  const auto actual=v.getMemberNames();
  require(std::set<std::string>(actual.begin(),actual.end())==expected,"Explicit schema fields differ");
}
inline double number(const Json::Value& v) {
  require(v.isNumeric()&&!v.isBool()&&std::isfinite(v.asDouble()),"Finite number required");return v.asDouble();
}
inline int integer(const Json::Value& v) {
  require(v.isInt()&&!v.isBool(),"Integer required");return v.asInt();
}
inline std::string string(const Json::Value& v) {
  require(v.isString()&&!v.asString().empty(),"Nonempty string required");return v.asString();
}
inline bool boolean(const Json::Value& v) { require(v.isBool(),"Boolean required");return v.asBool(); }
template<std::size_t N> std::array<double,N> numbers(const Json::Value& v) {
  require(v.isArray()&&v.size()==N,"Fixed array dimensions differ");std::array<double,N> out{};
  for(std::size_t i=0;i<N;++i)out[i]=number(v[Json::ArrayIndex(i)]);return out;
}
inline std::vector<double> vector(const Json::Value& v,std::size_t bound) {
  require(v.isArray()&&v.size()<=bound,"Bounded array required");std::vector<double> out;out.reserve(v.size());
  for(const auto& item:v)out.push_back(number(item));return out;
}
template<class T> Json::Value array_json(const T& values) {
  Json::Value out(Json::arrayValue);for(const auto& v:values)out.append(v);return out;
}
inline std::string finish_digest(EVP_MD_CTX* context) {
  std::array<unsigned char,EVP_MAX_MD_SIZE> digest{};unsigned size{};
  if(EVP_DigestFinal_ex(context,digest.data(),&size)!=1||size!=32)throw std::runtime_error("SHA finalization failed");
  std::ostringstream out;out<<std::hex<<std::setfill('0');
  for(unsigned i=0;i<size;++i)out<<std::setw(2)<<unsigned(digest[i]);return out.str();
}
inline auto digest_context() {
  std::unique_ptr<EVP_MD_CTX,decltype(&EVP_MD_CTX_free)> out(EVP_MD_CTX_new(),EVP_MD_CTX_free);
  if(!out||EVP_DigestInit_ex(out.get(),EVP_sha256(),nullptr)!=1)throw std::runtime_error("SHA initialization failed");return out;
}
inline std::string sha256_bytes(const std::string& bytes) {
  auto context=digest_context();
  if(EVP_DigestUpdate(context.get(),bytes.data(),bytes.size())!=1)throw std::runtime_error("SHA update failed");return finish_digest(context.get());
}
inline std::string sha256_file(const fs::path& path) {
  std::ifstream input(path,std::ios::binary);require(bool(input),"Cannot read code identity: "+path.string());
  auto context=digest_context();std::array<char,65536> buffer{};
  while(input){input.read(buffer.data(),buffer.size());
    if(EVP_DigestUpdate(context.get(),buffer.data(),std::size_t(input.gcount()))!=1)throw std::runtime_error("SHA update failed");}
  require(input.eof(),"Code identity read failed");return finish_digest(context.get());
}
inline bool is_sha256(const std::string& v) {
  return v.size()==64&&v.find_first_not_of("0123456789abcdef")==std::string::npos;
}
inline Json::Value parse_json(const std::string& bytes) {
  require(bytes.size()<=max_input_bytes,"Input exceeds 32 MiB bound");
  Json::CharReaderBuilder b;b["rejectDupKeys"]=true;b["failIfExtra"]=true;
  b["allowSpecialFloats"]=false;b["allowComments"]=false;
  std::unique_ptr<Json::CharReader> reader(b.newCharReader());Json::Value v;std::string errors;
  require(reader->parse(bytes.data(),bytes.data()+bytes.size(),&v,&errors),"Invalid benchmark JSON: "+errors);return v;
}
inline x650::Config config_of(const Json::Value& c) {
  exact_keys(c,{"nodes","max_iterations","node_dt","max_dt","feasibility_tolerance","kkt_tolerance","arm_share","joint_position_gain","weights","velocity_limit","acceleration_limit","q_lower","q_upper","physical_lower","physical_upper","geometry","response","base_position_gain","motion","kp","kd","bus_period","integration_step","trust_radius"});
  x650::Config v;v.nodes=integer(c["nodes"]);v.max_iterations=integer(c["max_iterations"]);
#define SCALAR(name) v.name=number(c[#name])
  SCALAR(node_dt);SCALAR(max_dt);SCALAR(feasibility_tolerance);SCALAR(kkt_tolerance);SCALAR(arm_share);SCALAR(joint_position_gain);SCALAR(bus_period);SCALAR(integration_step);SCALAR(trust_radius);
#undef SCALAR
#define ARRAY(name,n) v.name=numbers<n>(c[#name])
  ARRAY(weights,6);ARRAY(velocity_limit,5);ARRAY(acceleration_limit,5);ARRAY(q_lower,2);ARRAY(q_upper,2);ARRAY(physical_lower,2);ARRAY(physical_upper,2);ARRAY(geometry,10);ARRAY(response,5);ARRAY(base_position_gain,3);ARRAY(motion,8);ARRAY(kp,2);ARRAY(kd,2);
#undef ARRAY
  require(v.max_dt==.05&&v.feasibility_tolerance==2e-7&&v.kkt_tolerance==.002,"Production quality/time gates may not change");return v;
}
inline x650::Anchor anchor_of(const Json::Value& v) {
  exact_keys(v,{"position","rotation","q"});return {numbers<3>(v["position"]),numbers<9>(v["rotation"]),numbers<2>(v["q"])};
}
inline x650::Target target_of(const Json::Value& v) {
  exact_keys(v,{"position","velocity","acceleration"});return {numbers<3>(v["position"]),numbers<3>(v["velocity"]),numbers<3>(v["acceleration"])};
}
inline std::vector<x650::Command> commands_of(const Json::Value& v) {
  require(v.isArray()&&v.size()<=64,"Command history array exceeds bound");std::vector<x650::Command> out;out.reserve(v.size());
  for(const auto& row:v){exact_keys(row,{"time","velocity"});out.push_back({number(row["time"]),numbers<2>(row["velocity"])});}return out;
}
inline x650::Memory memory_of(const Json::Value& v) {
  exact_keys(v,{"initialized","history_primed","has_last_t","has_stamp","has_origin","anchor","previous_command","last_t","previous_stamp","origin","plan_t","step_count","plan","plan_h","joint_history"});x650::Memory m;
#define BOOL(name) m.name=boolean(v[#name])
  BOOL(initialized);BOOL(history_primed);BOOL(has_last_t);BOOL(has_stamp);BOOL(has_origin);
#undef BOOL
  m.anchor=anchor_of(v["anchor"]);m.previous_command=numbers<5>(v["previous_command"]);
#define DOUBLE(name) m.name=number(v[#name])
  DOUBLE(last_t);DOUBLE(previous_stamp);DOUBLE(origin);DOUBLE(plan_t);
#undef DOUBLE
  require(v["step_count"].isUInt64(),"Unsigned step_count required");m.step_count=v["step_count"].asUInt64();
  m.plan=vector(v["plan"],300);m.plan_h=vector(v["plan_h"],60);m.joint_history=commands_of(v["joint_history"]);return m;
}
inline x650::StepInput input_of(const Json::Value& v) {
  exact_keys(v,{"t","dt","state","target","context","preview"});x650::StepInput in;
  in.t=number(v["t"]);in.dt=number(v["dt"]);const auto& s=v["state"];
  exact_keys(s,{"position","velocity","omega_body","acceleration","rotation","q","dq","stamp","control_time","joint_stamp"});
#define ARRAY(name,n) in.state.name=numbers<n>(s[#name])
  ARRAY(position,3);ARRAY(velocity,3);ARRAY(omega_body,3);ARRAY(acceleration,3);ARRAY(rotation,9);ARRAY(q,2);ARRAY(dq,2);
#undef ARRAY
  in.state.stamp=number(s["stamp"]);in.state.control_time=number(s["control_time"]);in.state.joint_stamp=number(s["joint_stamp"]);
  in.target=target_of(v["target"]);require(v["preview"].isArray()&&v["preview"].size()<=60,"Explicit bounded preview required");
  for(const auto& t:v["preview"])in.preview.push_back(target_of(t));const auto& c=v["context"];
  exact_keys(c,{"reference_time","bus_origin","reference","commands","packets"});
  in.context.reference_time=number(c["reference_time"]);in.context.bus_origin=number(c["bus_origin"]);
  in.context.reference=numbers<2>(c["reference"]);in.context.commands=commands_of(c["commands"]);
  require(c["packets"].isArray()&&c["packets"].size()<=64,"Explicit bounded packets required");
  for(const auto& p:c["packets"]){exact_keys(p,{"send_time","q_ref","dq_ref","tau_ff","kp","kd"});x650::Packet pp;pp.send_time=number(p["send_time"]);
#define PACKET(name) pp.name=numbers<2>(p[#name])
    PACKET(q_ref);PACKET(dq_ref);PACKET(tau_ff);PACKET(kp);PACKET(kd);
#undef PACKET
    in.context.packets.push_back(pp);
  }return in;
}
struct Oracle {
  std::array<std::vector<double>,10> fields;
};
inline const std::array<const char*,10>& output_names() {
  static const std::array<const char*,10> names{"command","position_reference","velocity_feedforward","acceleration_feedforward","joint_velocity","joint_reference","joint_velocity_reference","joint_acceleration_reference","nominal_acceleration","tip_actual"};return names;
}
inline Oracle oracle_of(const Json::Value& v) {
  require(v.isObject(),"Oracle object required");Oracle out;const auto& names=output_names();
  for(const auto& name:v.getMemberNames())require(std::find(names.begin(),names.end(),name)!=names.end(),"Unknown oracle field");
  for(std::size_t i=0;i<names.size();++i){require(i>=3||v.isMember(names[i]),"Historical command/position/velocity oracle required");
    if(v.isMember(names[i])){const std::size_t count=i==0?5:((i>=4&&i<=7)?2:3);out.fields[i]=vector(v[names[i]],count);require(out.fields[i].size()==count,"Oracle field dimensions differ");}}
  return out;
}
struct Case { std::string id; x650::Memory memory; x650::StepInput input; Oracle oracle; };
struct Run { int warmup_rounds=0,measured_rounds=1,cpu=-1;double oracle_absolute_tolerance=1e-7; };
struct Dataset { x650::Config config;std::vector<Case> cases;Run run;Json::Value provenance; };
inline Dataset dataset_of(const Json::Value& v) {
  exact_keys(v,{"schema_version","kind","config","cases","run","provenance"});
  require(integer(v["schema_version"])==1&&v["kind"]=="x650_mpc_memory_benchmark_input_v1","Benchmark schema differs");
  Dataset d;d.config=config_of(v["config"]);require(v["provenance"].isObject(),"Explicit provenance object required");d.provenance=v["provenance"];
  const auto& r=v["run"];exact_keys(r,{"warmup_rounds","measured_rounds","cpu","oracle_absolute_tolerance"});
  d.run={integer(r["warmup_rounds"]),integer(r["measured_rounds"]),integer(r["cpu"]),number(r["oracle_absolute_tolerance"])};
  require(d.run.warmup_rounds>=0&&d.run.warmup_rounds<=1000&&d.run.measured_rounds>=1&&d.run.measured_rounds<=10000&&d.run.cpu>=-1,"Run counts/cpu outside bounds");
  require(d.run.oracle_absolute_tolerance==1e-7,"Original absolute oracle tolerance may not change");
  require(v["cases"].isArray()&&!v["cases"].empty()&&v["cases"].size()<=max_cases,"Bounded nonempty cases required");
  require(std::size_t(d.run.measured_rounds)*v["cases"].size()<=max_samples&&std::size_t(d.run.warmup_rounds)*v["cases"].size()<=max_samples,"Sample count exceeds 100000 bound");
  std::set<std::string> ids;for(const auto& c:v["cases"]){exact_keys(c,{"id","memory","input","oracle"});Case item;
    item.id=string(c["id"]);require(item.id.size()<=128&&ids.insert(item.id).second,"Case id length/uniqueness differs");
    item.memory=memory_of(c["memory"]);item.input=input_of(c["input"]);item.oracle=oracle_of(c["oracle"]);d.cases.push_back(std::move(item));}
  return d;
}
inline bool same_memory(const x650::Memory& a,const x650::Memory& b) {
  if(a.initialized!=b.initialized||a.history_primed!=b.history_primed||a.has_last_t!=b.has_last_t||a.has_stamp!=b.has_stamp||a.has_origin!=b.has_origin||a.anchor.position!=b.anchor.position||a.anchor.rotation!=b.anchor.rotation||a.anchor.q!=b.anchor.q||a.previous_command!=b.previous_command||a.last_t!=b.last_t||a.previous_stamp!=b.previous_stamp||a.origin!=b.origin||a.plan_t!=b.plan_t||a.step_count!=b.step_count||a.plan!=b.plan||a.plan_h!=b.plan_h||a.joint_history.size()!=b.joint_history.size())return false;
  for(std::size_t i=0;i<a.joint_history.size();++i)if(a.joint_history[i].time!=b.joint_history[i].time||a.joint_history[i].velocity!=b.joint_history[i].velocity)return false;
  return true;
}
} // namespace x650_benchmark
