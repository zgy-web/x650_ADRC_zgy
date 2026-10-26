// Independent, raw-input replay of one owning Core call. Build this source
// separately against each version's matching mpc_core.hpp and shared library.
// It never publishes commands, reconstructs history, or changes the deadline.
#include "benchmark_io.hpp"
#include <dlfcn.h>
#include <chrono>
#include <iostream>
#include <type_traits>

namespace b=x650_benchmark;
namespace {
template<class D,class=void>struct has_recovery:std::false_type{};
template<class D>struct has_recovery<D,std::void_t<decltype(std::declval<D>().recovery_triggered),
    decltype(std::declval<D>().recovery_reason),decltype(std::declval<D>().recovery_attempts)>>:std::true_type{};
template<class R,class=void>struct has_recovery_states:std::false_type{};
template<class R>struct has_recovery_states<R,std::void_t<decltype(std::declval<R>().query_times),
    decltype(std::declval<R>().candidate_states)>>:std::true_type{};
template<class R,class=void>struct has_recovery_nodes:std::false_type{};
template<class R>struct has_recovery_nodes<R,std::void_t<decltype(std::declval<R>().node_times),
    decltype(std::declval<R>().node_states),decltype(std::declval<R>().node_references)>>:std::true_type{};
template<class D,class=void>struct has_linearized_states:std::false_type{};
template<class D>struct has_linearized_states<D,std::void_t<decltype(std::declval<D>().linearized_states)>>:std::true_type{};
template<class D,class=void>struct has_intervals:std::false_type{};
template<class D>struct has_intervals<D,std::void_t<decltype(std::declval<D>().interval_repairs),
    decltype(std::declval<D>().interval_constraints)>>:std::true_type{};
Json::Value memory_json(const x650::Memory& m) {
  Json::Value out;
#define F(name) out[#name]=m.name
  F(initialized);F(history_primed);F(has_last_t);F(has_stamp);F(has_origin);
  F(last_t);F(previous_stamp);F(origin);F(plan_t);
#undef F
  out["step_count"]=Json::UInt64(m.step_count);
  out["anchor"]["position"]=b::array_json(m.anchor.position);
  out["anchor"]["rotation"]=b::array_json(m.anchor.rotation);
  out["anchor"]["q"]=b::array_json(m.anchor.q);
  out["previous_command"]=b::array_json(m.previous_command);
  out["plan"]=b::array_json(m.plan);out["plan_h"]=b::array_json(m.plan_h);
  out["joint_history"]=Json::Value(Json::arrayValue);
  for(const auto& item:m.joint_history){Json::Value row;row["time"]=item.time;
    row["velocity"]=b::array_json(item.velocity);out["joint_history"].append(row);}
  return out;
}
Json::Value packets_json(const std::vector<x650::Packet>& packets) {
  Json::Value out(Json::arrayValue);for(const auto& p:packets){Json::Value row;row["send_time"]=p.send_time;
#define F(name) row[#name]=b::array_json(p.name)
    F(q_ref);F(dq_ref);F(tau_ff);F(kp);F(kd);
#undef F
    out.append(row);}return out;
}
template<class D>void recovery_json(Json::Value& out,const D& d) {
  if constexpr(has_linearized_states<D>::value)out["linearized_states"]=b::array_json(d.linearized_states);
  out["recovery_diagnostics_available"]=has_recovery<D>::value;
  if constexpr(has_recovery<D>::value){out["recovery_triggered"]=d.recovery_triggered;
    out["recovery_reason"]=d.recovery_reason;out["recovery_trust_radius"]=d.recovery_trust_radius;
    out["recovery_stopping_violation"]=d.recovery_stopping_violation;out["recovery_attempts"]=Json::Value(Json::arrayValue);
    for(const auto& a:d.recovery_attempts){Json::Value row;
      row["centre_kind"]=a.centre_kind;row["rejection"]=a.rejection;
      row["nonlinear_passed"]=a.nonlinear_passed;row["original_constraints_passed"]=a.original_constraints_passed;
      row["trusted"]=a.trusted;row["original_primal_violation"]=a.original_primal_violation;
      row["projection_attempted"]=a.projection_attempted;row["projection_solved"]=a.projection_solved;row["stopping_violation"]=a.stopping_violation;
      row["elapsed_ms"]=a.elapsed_ms;row["centre_plan"]=b::array_json(a.centre_plan);
      row["envelope"]=b::array_json(a.envelope);row["substeps"]=a.substeps;
      if constexpr(has_recovery_states<std::decay_t<decltype(a)>>::value){
        row["query_times"]=b::array_json(a.query_times);row["candidate_states"]=b::array_json(a.candidate_states);}
      if constexpr(has_recovery_nodes<std::decay_t<decltype(a)>>::value){
        row["node_times"]=b::array_json(a.node_times);row["node_states"]=b::array_json(a.node_states);row["node_references"]=b::array_json(a.node_references);}
      out["recovery_attempts"].append(row);}}
}
template<class P>Json::Value interval_point_json(const P& p){Json::Value out;
 out["time"]=p.time;out["value"]=p.value;out["lower"]=p.lower;out["upper"]=p.upper;
 out["joint"]=p.joint;out["component"]=p.component;out["source_kind"]=p.source_kind;return out;
}
template<class D>void intervals_json(Json::Value& out,const D& d){
 if constexpr(has_intervals<D>::value){out["interval_rows"]=d.interval_rows;
  out["interval_repairs"]=Json::Value(Json::arrayValue);out["interval_constraints"]=Json::Value(Json::arrayValue);
  for(const auto& r:d.interval_repairs){Json::Value row;row["control_subproblem"]=r.control_subproblem;
   row["added_points"]=r.added_points;row["total_points"]=r.total_points;row["canonical_substeps"]=r.canonical_substeps;
   row["elapsed_ms"]=r.elapsed_ms;row["rejection"]=r.rejection;row["envelope"]=b::array_json(r.envelope);row["points"]=Json::Value(Json::arrayValue);
   for(const auto& p:r.points)row["points"].append(interval_point_json(p));
   row["trigger_points"]=Json::Value(Json::arrayValue);for(const auto& p:r.trigger_points)row["trigger_points"].append(interval_point_json(p));
   row["centre_plan"]=b::array_json(r.centre_plan);row["risk_radius"]=r.risk_radius;row["risk_margin"]=b::array_json(r.risk_margin);
   row["risk_encoding_lower"]=b::array_json(r.risk_encoding_lower);row["risk_encoding_upper"]=b::array_json(r.risk_encoding_upper);
   out["interval_repairs"].append(row);}
  for(const auto& c:d.interval_constraints){Json::Value row;row["point"]=interval_point_json(c.point);
   row["affine_offset"]=c.affine_offset;row["gradient"]=b::array_json(c.gradient);row["qp_row"]=c.qp_row;
   row["encoding_lower_remainder"]=c.encoding_lower_remainder;row["encoding_upper_remainder"]=c.encoding_upper_remainder;
   row["encoding_kernel_l1"]=c.encoding_kernel_l1;row["encoding_anchor_bias"]=c.encoding_anchor_bias;row["encoding_effort_half_lsb"]=c.encoding_effort_half_lsb;
   out["interval_constraints"].append(row);}
  out["interval_centre_plan"]=b::array_json(d.interval_centre_plan);
  out["interval_canonical_boundaries"]=b::array_json(d.interval_canonical_boundaries);
  out["interval_canonical_states"]=b::array_json(d.interval_canonical_states);
  out["interval_canonical_envelope"]=b::array_json(d.interval_canonical_envelope);
  out["interval_canonical_substeps"]=d.interval_canonical_substeps;
 }
}
Json::Value diagnostic_json(const x650::Diagnostics& d) {
  Json::Value out;
#define F(name) out[#name]=d.name
  F(subproblems);F(active_constraints);F(solver_mode);F(qp_rows);F(qp_columns);F(primal_violation);
  F(kkt_stationarity);F(kkt_complementarity);F(objective);F(q_linearization_error);F(dq_linearization_error);
  F(compute_ms);F(accepted);F(nonlinear_passed);F(actual_bounds);F(previous_plan_fallback);F(candidate_substeps);
#undef F
#define V(name) out[#name]=b::array_json(d.name)
  V(candidate_envelope);V(horizon);V(plan);V(predicted_states);V(predicted_tip);V(predicted_rotations);
  V(query_times);V(nominal_states);V(candidate_states);V(candidate_extrema);V(candidate_modes);V(candidate_zero_events);V(candidate_boundary_times);
  V(A);V(b);V(H);V(g);V(C);V(lower);V(upper);V(solution);
#undef V
  out["candidate_packets"]=packets_json(d.candidate_packets);out["nominal_packets"]=packets_json(d.nominal_packets);
  out["attempts"]=Json::Value(Json::arrayValue);for(const auto& a:d.attempts){Json::Value row;
    row["passed"]=a.passed;row["actual_bounds"]=a.actual_bounds;row["q_error"]=a.q_error;
    row["dq_error"]=a.dq_error;row["elapsed_ms"]=a.elapsed_ms;out["attempts"].append(row);}
  recovery_json(out,d);intervals_json(out,d);return out;
}
Json::Value result_json(const x650::StepResult& r) {
  Json::Value out;
#define F(name) out[#name]=b::array_json(r.name)
  F(command);F(position_reference);F(velocity_feedforward);F(acceleration_feedforward);
  F(joint_velocity);F(joint_reference);F(joint_velocity_reference);F(joint_acceleration_reference);
  F(nominal_acceleration);F(tip_actual);
#undef F
  return out;
}
Json::Value identity(void* symbol) {
  Dl_info info{};b::require(symbol&&dladdr(symbol,&info)&&info.dli_fname,"Loaded code identity unavailable");
  const auto path=b::fs::canonical(info.dli_fname);Json::Value out;
  out["path"]=path.string();out["sha256"]=b::sha256_file(path);return out;
}
std::string read_bound(const char* path,const std::string& digest) {
  b::require(b::is_sha256(digest)&&b::fs::file_size(path)<=b::max_input_bytes,"Input binding/size differs");
  std::ifstream stream(path,std::ios::binary);b::require(bool(stream),"Cannot open input");
  std::string bytes((std::istreambuf_iterator<char>(stream)),{});
  b::require(b::sha256_bytes(bytes)==digest,"Input SHA256 differs");return bytes;
}
void write_new(const char* path,const Json::Value& value) {
  b::require(!b::fs::exists(path),"Evidence output must not be overwritten");
  std::ofstream output(path,std::ios::binary);b::require(bool(output),"Cannot create output");
  Json::StreamWriterBuilder writer;writer["indentation"]="";writer["useSpecialFloats"]=false;
  output<<Json::writeString(writer,value)<<'\n';b::require(bool(output),"Evidence write failed");
}
} // namespace
int main(int argc,char** argv) {
  if(argc!=6){std::cerr<<"usage: x650_cpp_recovery_replay predictor.so predictor_sha input.json input_sha output.json\n";return 2;}
  try{
    b::require(b::is_sha256(argv[2])&&b::sha256_file(argv[1])==argv[2],"Predictor SHA256 differs");
    const auto bytes=read_bound(argv[3],argv[4]);const auto v=b::parse_json(bytes);
    const auto config=b::config_of(v["config"]);const auto before=b::memory_of(v["memory"]);
    const auto input=b::input_of(v["input"]);
    void* handle=dlopen(argv[1],RTLD_NOW|RTLD_LOCAL);if(!handle)throw std::runtime_error(dlerror());
    auto fn=reinterpret_cast<x650::Ts20BatchFn>(dlsym(handle,"ts20_prediction_batch"));
    b::require(fn,"Missing TS20 ABI");x650::Core core(config,fn,x650::DiagnosticMode::Full);core.restore(before);
    Json::Value result;result["kind"]="x650_cpp_recovery_raw_replay_v1";result["schema_version"]=1;
    result["input_sha256"]=argv[4];result["core_abi"]=x650_mpc_core_abi_version();
    result["loaded_code"]["core"]=identity(reinterpret_cast<void*>(&x650_mpc_core_abi_version));
    result["loaded_code"]["optimization"]=identity(dlsym(RTLD_DEFAULT,"x650_assemble"));
    result["loaded_code"]["predictor"]=identity(reinterpret_cast<void*>(fn));
    result["scope"]["offline_fixed_snapshot"]=true;result["scope"]["actuator_io"]=false;
    result["scope"]["closed_loop_or_physical_validation"]=false;result["scope"]["injected_delay"]=false;
    result["scope"]["original_core_deadline_ms"]=50.;result["scope"]["initial_trust_radius"]=std::min(config.trust_radius,.02);
    result["returned"]=false;result["core_error"]=false;result["unexpected_exception"]=false;
    result["output"]=Json::Value();result["error"]=Json::Value();
    result["memory_before"]=memory_json(before);const auto begin=std::chrono::steady_clock::now();
    x650::StepResult output;
    try{output=core.step(input);
      result["core_step_wall_ms"]=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-begin).count();
      result["returned"]=true;result["output"]=result_json(output);result["diagnostics"]=diagnostic_json(output.diagnostics);
    }catch(const x650::CoreError& e){
      result["core_step_wall_ms"]=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-begin).count();
      result["core_error"]=true;result["error"]=e.what();result["diagnostics"]=diagnostic_json(e.diagnostics);
    }catch(const std::exception& e){
      result["core_step_wall_ms"]=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-begin).count();
      result["unexpected_exception"]=true;result["error"]=e.what();}
    result["memory_after"]=memory_json(core.memory());result["memory_unchanged"]=b::same_memory(before,core.memory());
    result["memory_commit_valid"]=result["returned"].asBool()&&core.memory().step_count==before.step_count+1&&
      core.memory().last_t==input.t&&core.memory().previous_command==output.command;
    result["complete"]=true;write_new(argv[5],result);return result["returned"].asBool()?0:1;
  }catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 2;}
}
