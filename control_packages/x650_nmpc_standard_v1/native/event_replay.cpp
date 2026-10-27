// Offline, solver-independent check against the original encoded event model.
#include "benchmark_io.hpp"
#include <iostream>
extern "C" int ts20_prediction_batch(const double*,const double*,double,double,const double*,const double*,const double*,int,const double*,int,int,double*,int*,int*,double*,double*,int,int*,int*,char*,int);
namespace b=x650_benchmark;
Json::Value report(const x650::PlanValidation& v){
  Json::Value j;
#define FIELD(n) j[#n]=v.n
  FIELD(passed);FIELD(input_valid);FIELD(command_bounds_passed);FIELD(reference_bounds_passed);
  FIELD(native_checked);FIELD(native_bounds_passed);FIELD(stopping_passed);FIELD(pending_guard_passed);FIELD(hold_passed);FIELD(reason);
  FIELD(task_time_s);FIELD(dt_s);FIELD(source_age_s);FIELD(source_control_time_s);FIELD(source_joint_time_s);
  FIELD(input_violation);FIELD(slew_violation);FIELD(reference_violation);FIELD(q_envelope_violation);FIELD(dq_envelope_violation);FIELD(stopping_violation);FIELD(pending_violation);
  FIELD(actual_packet_count);FIELD(actual_command_count);FIELD(substeps);
#undef FIELD
  j["committed_step_count"]=Json::UInt64(v.committed_step_count);
#define ARRAY(n) j[#n]=b::array_json(v.n)
  ARRAY(plan);ARRAY(horizon);ARRAY(node_times);ARRAY(node_states);ARRAY(node_references);ARRAY(envelope);
  ARRAY(first_lower);ARRAY(first_upper);ARRAY(current_q);ARRAY(current_dq);ARRAY(reference_at_task_time);
  ARRAY(query_times);ARRAY(query_states);ARRAY(boundary_times);ARRAY(boundary_states);ARRAY(extrema);
#undef ARRAY
  j["hold"]["passed"]=v.hold_guard.passed;j["hold"]["duration_s"]=v.hold_guard.duration_s;
  j["hold"]["envelope"]=b::array_json(v.hold_guard.envelope);
  j["scope"]="Conditional original event model; no optimizer, actuator, controller-memory mutation or issue-deadline approval";
  return j;
}
int main(int argc,char** argv){try{
  b::require(argc==3,"usage: x650_standard_event_replay input.json output.json");
  b::require(!b::fs::exists(argv[2]),"Refusing to replace existing event evidence");
  std::ifstream f(argv[1],std::ios::binary);b::require(bool(f),"Cannot read input");
  const std::string bytes((std::istreambuf_iterator<char>(f)),{});const auto data=b::parse_json(bytes);
  const auto c=b::config_of(data["config"]);Json::Value out;
  out["kind"]="x650_standard_common_event_replay_v1";out["input_sha256"]=b::sha256_bytes(bytes);
  out["records"]=Json::Value(Json::arrayValue);int accepted=0;
  b::require(data["cases"].isArray()&&!data["cases"].empty()&&data["cases"].size()<=512,"Bounded cases required");
  for(const auto& row:data["cases"]){
    const auto memory=b::memory_of(row["memory"]);const auto input=b::input_of(row["input"]);
    auto plan=b::vector(row["plan"],300);auto checked=x650::evaluate_physical_plan(c,ts20_prediction_batch,memory,input,plan);
    auto value=report(checked);value["id"]=b::string(row["id"]);out["records"].append(std::move(value));accepted+=checked.passed;
  }
  out["accepted_count"]=accepted;out["count"]=data["cases"].size();out["all_accepted"]=accepted==int(data["cases"].size());
  std::ofstream output(argv[2],std::ios::binary);b::require(bool(output),"Cannot open output");
  Json::StreamWriterBuilder writer;writer["indentation"]="  ";writer["precision"]=17;
  output<<Json::writeString(writer,out)<<'\n';output.close();b::require(bool(output),"Output write failed");
  std::cout<<"event_checks="<<out["count"]<<" accepted="<<accepted<<'\n';return 0;
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 2;}}
