#include "benchmark_io.hpp"
#include <dlfcn.h>
#include <sched.h>
#include <sys/resource.h>
#include <time.h>
#include <unistd.h>
#include <cerrno>
#include <cstring>
#include <iostream>
#include <limits>
#include <numeric>

namespace b=x650_benchmark;
namespace {
using Ns=std::int64_t;
Ns now(clockid_t clock) {
  timespec t{};if(clock_gettime(clock,&t))throw std::runtime_error("clock_gettime failed");
  return Ns(t.tv_sec)*1000000000+t.tv_nsec;
}
struct Times { Ns wall=0,thread=0; };
struct Output {
  std::array<std::array<double,5>,10> fields{};
  bool finite=true;
};
template<class T> void copy_field(Output& output,int index,const T& value) {
  std::copy(value.begin(),value.end(),output.fields[index].begin());
  for(double v:value)output.finite=output.finite&&std::isfinite(v);
}
Output output_of(const x650::StepResult& v) {
  Output o;
#define FIELD(index,name) copy_field(o,index,v.name)
  FIELD(0,command);FIELD(1,position_reference);FIELD(2,velocity_feedforward);FIELD(3,acceleration_feedforward);
  FIELD(4,joint_velocity);FIELD(5,joint_reference);FIELD(6,joint_velocity_reference);FIELD(7,joint_acceleration_reference);
  FIELD(8,nominal_acceleration);FIELD(9,tip_actual);
#undef FIELD
  return o;
}
struct Diagnostic {
  int subproblems=0,active_constraints=0,solver_mode=-1,qp_rows=0,qp_columns=0,candidate_substeps=0;
  double primal_violation=0.,kkt_stationarity=0.,kkt_complementarity=0.,objective=0.,compute_ms=0.;
  double q_linearization_error=0.,dq_linearization_error=0.;
  bool accepted=false,nonlinear_passed=false,actual_bounds=false,previous_plan_fallback=false;
  std::array<double,8> candidate_envelope{};
  std::array<x650::Attempt,3> attempts{};std::size_t attempt_count=0;
};
Diagnostic diagnostic_of(const x650::Diagnostics& d) {
  Diagnostic o;
#define FIELD(name) o.name=d.name
  FIELD(subproblems);FIELD(active_constraints);FIELD(solver_mode);FIELD(qp_rows);FIELD(qp_columns);FIELD(candidate_substeps);
  FIELD(primal_violation);FIELD(kkt_stationarity);FIELD(kkt_complementarity);FIELD(objective);FIELD(compute_ms);
  FIELD(q_linearization_error);FIELD(dq_linearization_error);FIELD(accepted);FIELD(nonlinear_passed);FIELD(actual_bounds);FIELD(previous_plan_fallback);FIELD(candidate_envelope);
#undef FIELD
  o.attempt_count=std::min(d.attempts.size(),o.attempts.size());
  std::copy_n(d.attempts.begin(),o.attempt_count,o.attempts.begin());return o;
}
struct Sample {
  Times elapsed;Output output;Diagnostic diagnostic;
  bool returned=false,core_error=false,unexpected_exception=false,memory_unchanged=false,memory_commit_valid=false,oracle_checked=false,oracle_match=false;
  std::uint64_t step_count_after=0;
  double oracle_max_error=0.;std::array<char,512> error{};
};
void save_error(Sample& sample,const char* error) {
  const auto size=std::min(std::strlen(error),sample.error.size()-1);std::copy_n(error,size,sample.error.data());sample.error[size]='\0';
}
// No JSON, file operations, hashing, ROS, Python, network or diagnostic output
// occurs in this function. Restore and validation are explicitly outside the
// Core timing bracket. Core's own allocations and original 50 ms gates remain.
Sample call(x650::Core& core,const b::Case& c,double tolerance) {
  core.restore(c.memory); // Identical anchor/history/previous plan for every paired call.
  Sample sample;x650::StepResult result;
  const auto thread_begin=now(CLOCK_THREAD_CPUTIME_ID);
  const auto wall_begin=now(CLOCK_MONOTONIC_RAW);
  try {
    result=core.step(c.input);
    sample.elapsed.wall=now(CLOCK_MONOTONIC_RAW)-wall_begin;
    sample.elapsed.thread=now(CLOCK_THREAD_CPUTIME_ID)-thread_begin;
    sample.returned=true; // Everything below is outside both timing brackets.
    sample.output=output_of(result);sample.diagnostic=diagnostic_of(result.diagnostics);
  }catch(const x650::CoreError& e) {
    sample.elapsed.wall=now(CLOCK_MONOTONIC_RAW)-wall_begin;
    sample.elapsed.thread=now(CLOCK_THREAD_CPUTIME_ID)-thread_begin;
    sample.core_error=true;save_error(sample,e.what());sample.diagnostic=diagnostic_of(e.diagnostics);
  }catch(const std::exception& e) {
    sample.elapsed.wall=now(CLOCK_MONOTONIC_RAW)-wall_begin;
    sample.elapsed.thread=now(CLOCK_THREAD_CPUTIME_ID)-thread_begin;
    sample.unexpected_exception=true;save_error(sample,e.what());
  }
  sample.step_count_after=core.memory().step_count;
  sample.memory_unchanged=b::same_memory(core.memory(),c.memory);
  sample.memory_commit_valid=sample.returned&&sample.step_count_after==c.memory.step_count+1&&
    core.memory().last_t==c.input.t&&core.memory().previous_command==result.command;
  if(sample.returned){sample.oracle_checked=true;sample.oracle_match=sample.output.finite;
    for(std::size_t i=0;i<c.oracle.fields.size();++i)for(std::size_t j=0;j<c.oracle.fields[i].size();++j){
      const auto error=std::abs(sample.output.fields[i][j]-c.oracle.fields[i][j]);
      sample.oracle_max_error=std::max(sample.oracle_max_error,error);
      sample.oracle_match=sample.oracle_match&&error<=(i==1?1e-9:tolerance);}}
  return sample;
}
bool valid(const Sample& s) {
  return s.returned&&s.oracle_checked&&s.oracle_match&&s.memory_commit_valid&&
    s.diagnostic.accepted&&s.diagnostic.nonlinear_passed&&s.diagnostic.actual_bounds&&
    s.elapsed.wall<=50000000&&s.diagnostic.compute_ms<=50.;
}
Json::Value json_number(double value) { return std::isfinite(value)?Json::Value(value):Json::Value(); }
Json::Value diagnostic_json(const Diagnostic& d) {
  Json::Value o;
#define FIELD(name) o[#name]=d.name
  FIELD(subproblems);FIELD(active_constraints);FIELD(solver_mode);FIELD(qp_rows);FIELD(qp_columns);FIELD(candidate_substeps);
  FIELD(primal_violation);FIELD(kkt_stationarity);FIELD(kkt_complementarity);FIELD(objective);FIELD(compute_ms);
  FIELD(q_linearization_error);FIELD(dq_linearization_error);FIELD(accepted);FIELD(nonlinear_passed);FIELD(actual_bounds);FIELD(previous_plan_fallback);
#undef FIELD
  o["candidate_envelope"]=b::array_json(d.candidate_envelope);o["attempts"]=Json::Value(Json::arrayValue);
  for(std::size_t i=0;i<d.attempt_count;++i){const auto& a=d.attempts[i];Json::Value v;
    v["passed"]=a.passed;v["actual_bounds"]=a.actual_bounds;v["q_error"]=json_number(a.q_error);
    v["dq_error"]=json_number(a.dq_error);v["elapsed_ms"]=json_number(a.elapsed_ms);o["attempts"].append(v);}
  return o;
}
Json::Value sample_json(const Sample& s,std::size_t case_index,std::size_t round) {
  Json::Value o;o["case_index"]=Json::UInt64(case_index);o["round"]=Json::UInt64(round);
  o["core_step_wall_ns"]=Json::Int64(s.elapsed.wall);o["core_step_thread_cpu_ns"]=Json::Int64(s.elapsed.thread);
  o["returned"]=s.returned;o["core_error"]=s.core_error;o["unexpected_exception"]=s.unexpected_exception;
  o["error"]=s.error[0]?Json::Value(s.error.data()):Json::Value();o["valid"]=valid(s);
  o["exceeded_20ms_target"]=s.elapsed.wall>20000000;o["exceeded_50ms_wall"]=s.elapsed.wall>50000000;
  o["production_deadline_error"]=std::strstr(s.error.data(),"deadline exceeded")!=nullptr;
  o["memory_unchanged"]=s.memory_unchanged;o["memory_commit_valid"]=s.memory_commit_valid;
  o["step_count_after"]=Json::UInt64(s.step_count_after);o["oracle_checked"]=s.oracle_checked;
  o["oracle_match"]=s.oracle_checked?Json::Value(s.oracle_match):Json::Value();
  o["oracle_max_absolute_error"]=s.oracle_checked?json_number(s.oracle_max_error):Json::Value();
  o["output"]=Json::Value();if(s.returned){Json::Value out;
    for(std::size_t i=0;i<b::output_names().size();++i){const int count=i==0?5:((i>=4&&i<=7)?2:3);
      Json::Value a(Json::arrayValue);for(int j=0;j<count;++j)a.append(json_number(s.output.fields[i][j]));out[b::output_names()[i]]=a;}o["output"]=out;}
  o["diagnostics"]=diagnostic_json(s.diagnostic);return o;
}
Json::Value statistics(std::vector<Ns> values) {
  Json::Value out;out["sample_count"]=Json::UInt64(values.size());if(values.empty())return out;
  std::sort(values.begin(),values.end());const auto percentile=[&](double p){return values[std::size_t(std::ceil(p*values.size()))-1];};
  long double sum=0;for(auto value:values)sum+=value;
  out["min_ns"]=Json::Int64(values.front());out["median_ns"]=Json::Int64(percentile(.5));out["p95_ns"]=Json::Int64(percentile(.95));
  out["p99_ns"]=Json::Int64(percentile(.99));out["max_ns"]=Json::Int64(values.back());out["mean_ns"]=double(sum/values.size());return out;
}
Json::Value summarize(const std::vector<Sample>& samples) {
  std::vector<Ns> wall,thread;wall.reserve(samples.size());thread.reserve(samples.size());
  std::size_t accepted=0,checked=0,matches=0,valid_count=0,errors=0,unexpected=0,over20=0,over50=0,deadlines=0,failed_unchanged=0;
  for(const auto& s:samples){wall.push_back(s.elapsed.wall);thread.push_back(s.elapsed.thread);
    accepted+=s.returned;checked+=s.oracle_checked;matches+=s.oracle_checked&&s.oracle_match;valid_count+=valid(s);errors+=s.core_error;
    unexpected+=s.unexpected_exception;over20+=s.elapsed.wall>20000000;over50+=s.elapsed.wall>50000000;
    deadlines+=std::strstr(s.error.data(),"deadline exceeded")!=nullptr;failed_unchanged+=!s.returned&&s.memory_unchanged;}
  Json::Value out;out["sample_count"]=Json::UInt64(samples.size());out["returned_count"]=Json::UInt64(accepted);
  out["valid_count"]=Json::UInt64(valid_count);out["oracle_checked_count"]=Json::UInt64(checked);out["oracle_match_count"]=Json::UInt64(matches);
  out["core_error_count"]=Json::UInt64(errors);out["unexpected_exception_count"]=Json::UInt64(unexpected);
  out["exceeded_20ms_target_count"]=Json::UInt64(over20);out["exceeded_50ms_wall_count"]=Json::UInt64(over50);
  out["production_deadline_error_count"]=Json::UInt64(deadlines);out["failed_calls_with_unchanged_memory"]=Json::UInt64(failed_unchanged);
  out["all_calls_valid"]=valid_count==samples.size();out["wall"]=statistics(std::move(wall));out["thread_cpu"]=statistics(std::move(thread));
  out["statistics_include_all_failed_and_slow_calls"]=true;out["percentile_method"]="nearest_rank";return out;
}
Json::Value loaded_identity(void* symbol) {
  Dl_info info{};b::require(symbol&&dladdr(symbol,&info)&&info.dli_fname,"Loaded code identity unavailable");
  const auto path=b::fs::canonical(info.dli_fname);Json::Value out;out["path"]=path.string();out["sha256"]=b::sha256_file(path);return out;
}
Json::Value resource_json(const rusage& r) {
  Json::Value o;o["minor_page_faults"]=Json::Int64(r.ru_minflt);o["major_page_faults"]=Json::Int64(r.ru_majflt);
  o["voluntary_context_switches"]=Json::Int64(r.ru_nvcsw);o["involuntary_context_switches"]=Json::Int64(r.ru_nivcsw);
  o["max_rss_kib"]=Json::Int64(r.ru_maxrss);return o;
}
Json::Value run(int argc,char** argv) {
  b::require(argc==7,"Usage: x650_mpc_memory_benchmark --predictor /abs/predictor.so --predictor-sha256 HEX --input-sha256 HEX < stdin");
  std::string predictor,predictor_hash,input_hash;
  for(int i=1;i<argc;i+=2){const std::string key=argv[i];
    if(key=="--predictor"&&predictor.empty())predictor=argv[i+1];
    else if(key=="--predictor-sha256"&&predictor_hash.empty())predictor_hash=argv[i+1];
    else if(key=="--input-sha256"&&input_hash.empty())input_hash=argv[i+1];
    else throw std::invalid_argument("Unknown or duplicate CLI argument");}
  b::require(b::fs::path(predictor).is_absolute()&&b::is_sha256(predictor_hash)&&b::is_sha256(input_hash),"Absolute predictor and lowercase SHA256 bindings required");
  std::string bytes;bytes.reserve(1024*1024);std::array<char,65536> buffer{};
  while(std::cin){std::cin.read(buffer.data(),buffer.size());const auto count=std::size_t(std::cin.gcount());
    b::require(bytes.size()+count<=b::max_input_bytes,"Input exceeds 32 MiB bound");bytes.append(buffer.data(),count);}
  b::require(std::cin.eof(),"Stdin read failed");b::require(b::sha256_bytes(bytes)==input_hash,"Stdin SHA256 differs");
  const auto document=b::parse_json(bytes);const auto data=b::dataset_of(document);
  b::require(b::sha256_file(predictor)==predictor_hash,"Predictor SHA256 differs");
  void* handle=dlopen(predictor.c_str(),RTLD_NOW|RTLD_LOCAL);if(!handle)throw std::runtime_error(dlerror());
  auto fn=reinterpret_cast<x650::Ts20BatchFn>(dlsym(handle,"ts20_prediction_batch"));b::require(fn,"Missing TS20 prediction ABI");
  Json::Value identities;identities["executable"]["path"]=b::fs::canonical("/proc/self/exe").string();
  identities["executable"]["sha256"]=b::sha256_file("/proc/self/exe");
  identities["core"]=loaded_identity(reinterpret_cast<void*>(&x650_mpc_core_abi_version));
  identities["optimization"]=loaded_identity(dlsym(RTLD_DEFAULT,"x650_assemble"));identities["predictor"]=loaded_identity(reinterpret_cast<void*>(fn));
  b::require(identities["predictor"]["sha256"].asString()==predictor_hash,"Loaded predictor differs");
  x650::Core core(data.config,fn,x650::DiagnosticMode::Compact);
  // Fail invalid restore contracts before the first timed invocation.
  for(const auto& c:data.cases)core.restore(c.memory);
  if(data.run.cpu>=0){b::require(data.run.cpu<CPU_SETSIZE,"CPU exceeds CPU_SETSIZE");cpu_set_t affinity;CPU_ZERO(&affinity);CPU_SET(data.run.cpu,&affinity);
    if(sched_setaffinity(0,sizeof(affinity),&affinity))throw std::runtime_error(std::string("CPU affinity failed: ")+std::strerror(errno));}
  cpu_set_t affinity;CPU_ZERO(&affinity);if(sched_getaffinity(0,sizeof(affinity),&affinity))throw std::runtime_error("Cannot read CPU affinity");
  std::vector<int> allowed_cpus;for(int cpu=0;cpu<CPU_SETSIZE;++cpu)if(CPU_ISSET(cpu,&affinity))allowed_cpus.push_back(cpu);
  const auto count=data.cases.size();
  // Only fixed-size summaries are retained; Core's large intermediate vectors
  // are destroyed between calls, outside each Core timing bracket.
  std::vector<Sample> warmup(count*std::size_t(data.run.warmup_rounds));
  std::vector<Sample> samples(count*std::size_t(data.run.measured_rounds));
  std::vector<Ns> overhead_wall(1000),overhead_thread(1000);
  for(std::size_t i=0;i<overhead_wall.size();++i){const auto tc=now(CLOCK_THREAD_CPUTIME_ID);const auto wc=now(CLOCK_MONOTONIC_RAW);
    overhead_wall[i]=now(CLOCK_MONOTONIC_RAW)-wc;overhead_thread[i]=now(CLOCK_THREAD_CPUTIME_ID)-tc;}
  const auto cold=call(core,data.cases.front(),data.run.oracle_absolute_tolerance);
  for(std::size_t round=0;round<std::size_t(data.run.warmup_rounds);++round)
    for(std::size_t i=0;i<count;++i)warmup[round*count+i]=call(core,data.cases[i],data.run.oracle_absolute_tolerance);
  rusage before{},after{};getrusage(RUSAGE_SELF,&before);
  const auto all_begin=now(CLOCK_MONOTONIC_RAW);
  for(std::size_t round=0;round<std::size_t(data.run.measured_rounds);++round)
    for(std::size_t i=0;i<count;++i)samples[round*count+i]=call(core,data.cases[i],data.run.oracle_absolute_tolerance);
  const auto all_elapsed=now(CLOCK_MONOTONIC_RAW)-all_begin;getrusage(RUSAGE_SELF,&after);
  // From here on, measurement is finished. One stdout JSON is assembled/written.
  Json::Value out;out["schema_version"]=1;out["kind"]="x650_mpc_memory_benchmark_result_v1";out["complete"]=true;
  out["backend"]="cpu";out["mode"]="fixed_snapshot_paired";out["input_sha256"]=input_hash;out["provenance"]=data.provenance;
  out["case_count"]=Json::UInt64(count);out["run"]=document["run"];out["loaded_code"]=identities;out["allowed_cpus"]=b::array_json(allowed_cpus);
  out["scope"]["timed_function"]="x650::Core::step(StepInput), Compact production diagnostics";
  out["scope"]["timed_wall_clock"]="CLOCK_MONOTONIC_RAW";out["scope"]["timed_thread_clock"]="CLOCK_THREAD_CPUTIME_ID";
  out["scope"]["clock_order"]="thread begin, wall begin, Core::step, wall end, thread end";
  out["scope"]["memory_restore_inside_core_timer"]=false;out["scope"]["input_parse_hash_load_inside_core_timer"]=false;
  out["scope"]["oracle_validation_inside_core_timer"]=false;out["scope"]["production_deadline_ms"]=50;
  out["scope"]["control_update_target_ms"]=20;out["scope"]["runtime_python_calls"]=0;out["scope"]["runtime_ros_calls"]=0;
  out["scope"]["application_file_writes"]=0;out["scope"]["measurement_loop_output_bytes"]=0;out["scope"]["network_calls_in_measurement"]=0;
  out["scope"]["cuda_implemented"]=false;out["scope"]["hardware_closed_loop_claimed"]=false;
  out["scope"]["local_control_stack_latency_measured"]=false;
  out["scope"]["os_swap_or_system_logging_controlled_by_harness"]=false;
  out["scope"]["cold_first_call_definition"]="First Core::step after preload, dlopen, construction and restore validation; not cold boot or process launch";
  out["scope"]["all_measured_calls_retained"]=true;out["scope"]["wall_minus_thread_is_not_pure_scheduler_delay"]=true;
  for(std::size_t i=0;i<b::output_names().size();++i)out["scope"]["oracle_field_absolute_tolerances"][b::output_names()[i]]=i==1?1e-9:data.run.oracle_absolute_tolerance;
  out["clock_calibration"]["wall"]=statistics(std::move(overhead_wall));out["clock_calibration"]["thread_cpu"]=statistics(std::move(overhead_thread));
  out["clock_calibration"]["subtracted_from_samples"]=false;
  out["cases"]=Json::Value(Json::arrayValue);
  for(const auto& c:data.cases){Json::Value v;v["id"]=c.id;v["input_t_s"]=c.input.t;v["dt_s"]=c.input.dt;v["step_count_before"]=Json::UInt64(c.memory.step_count);
    v["oracle_fields"]=Json::Value(Json::arrayValue);for(std::size_t i=0;i<c.oracle.fields.size();++i)if(!c.oracle.fields[i].empty())v["oracle_fields"].append(b::output_names()[i]);out["cases"].append(v);}
  out["cold_first_call"]=sample_json(cold,0,0);out["warmup_samples"]=Json::Value(Json::arrayValue);out["samples"]=Json::Value(Json::arrayValue);
  for(std::size_t i=0;i<warmup.size();++i)out["warmup_samples"].append(sample_json(warmup[i],i%count,i/count));
  for(std::size_t i=0;i<samples.size();++i)out["samples"].append(sample_json(samples[i],i%count,i/count));
  out["summary"]["warmup"]=summarize(warmup);out["summary"]["measured"]=summarize(samples);
  out["summary"]["measured_loop_wall_ns_including_restore_and_summary_copy"]=Json::Int64(all_elapsed);
  out["summary"]["resource_before_measured_loop"]=resource_json(before);out["summary"]["resource_after_measured_loop"]=resource_json(after);
  out["all_calls_valid"]=valid(cold)&&out["summary"]["warmup"]["all_calls_valid"].asBool()&&out["summary"]["measured"]["all_calls_valid"].asBool();
  dlclose(handle);return out;
}
} // namespace
int main(int argc,char** argv) {
  Json::Value result;int exit_code=0;
  try {result=run(argc,argv);}catch(const std::exception& error){result["schema_version"]=1;
    result["kind"]="x650_mpc_memory_benchmark_result_v1";result["complete"]=false;result["error"]=error.what();exit_code=2;}
  Json::StreamWriterBuilder writer;writer["indentation"]="";std::cout<<Json::writeString(writer,result)<<'\n';
  if(!std::cout)return 3;return exit_code;
}
