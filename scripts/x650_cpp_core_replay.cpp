// Offline driver of the same owning Core used by the C++ ROS worker.
// Input is raw state/history/configuration, never Python matrices or candidates.
#include "flyarm_mpc_cpp/config_io.hpp"
#include <dlfcn.h>
#include <chrono>
#include <iostream>
using namespace flyarm_mpc_cpp;
extern "C" unsigned x650_mpc_core_abi_version();
Json::Value loaded_identity(void* symbol) {
  Dl_info info{};if(!symbol||!dladdr(symbol,&info)||!info.dli_fname)throw std::runtime_error("Loaded-library identity unavailable");
  const auto path=fs::canonical(info.dli_fname);Json::Value result;result["path"]=path.string();result["sha256"]=sha256(path);return result;
}
template<class T> Json::Value json_array(const T& values) {
  Json::Value a(Json::arrayValue); for(auto value:values)a.append(value); return a;
}
std::vector<double> vector_of(const Json::Value& values) {
  if(!values.isArray())throw std::invalid_argument("Array required");
  std::vector<double> result;for(const auto& v:values)result.push_back(number(v));return result;
}
x650::Anchor anchor_of(const Json::Value& v) {
  x650::Anchor a;a.position=numbers<3>(v["position"]);a.rotation=numbers<9>(v["rotation"]);a.q=numbers<2>(v["q"]);return a;
}
x650::Target target_of(const Json::Value& v) {
  return {numbers<3>(v["position"]),numbers<3>(v["velocity"]),numbers<3>(v["acceleration"])};
}
std::vector<x650::Command> commands_of(const Json::Value& v) {
  if(!v.isArray())throw std::invalid_argument("Commands array required");
  std::vector<x650::Command> out;for(const auto& row:v)out.push_back({number(row["time"]),numbers<2>(row["velocity"])});return out;
}
x650::Memory memory_of(const Json::Value& v) {
  x650::Memory m;
#define B(name) if(!v[#name].isBool())throw std::invalid_argument("Boolean memory required");m.name=v[#name].asBool()
  B(initialized);B(history_primed);B(has_last_t);B(has_stamp);B(has_origin);
#undef B
  m.anchor=anchor_of(v["anchor"]);m.previous_command=numbers<5>(v["previous_command"]);
#define D(name) m.name=number(v[#name])
  D(last_t);D(previous_stamp);D(origin);D(plan_t);
#undef D
  if(!v["step_count"].isUInt64())throw std::invalid_argument("step_count required");
  m.step_count=v["step_count"].asUInt64();m.plan=vector_of(v["plan"]);m.plan_h=vector_of(v["plan_h"]);
  m.joint_history=commands_of(v["joint_history"]);return m;
}
x650::StepInput input_of(const Json::Value& v) {
  x650::StepInput input;input.t=number(v["t"]);input.dt=number(v["dt"]);const auto& s=v["state"];
#define A(name,n) input.state.name=numbers<n>(s[#name])
  A(position,3);A(velocity,3);A(omega_body,3);A(acceleration,3);A(rotation,9);A(q,2);A(dq,2);
#undef A
  input.state.stamp=number(s["stamp"]);input.state.control_time=number(s["control_time"]);input.state.joint_stamp=number(s["joint_stamp"]);
  input.target=target_of(v["target"]);for(const auto& t:v["preview"])input.preview.push_back(target_of(t));
  const auto& c=v["context"];input.context.reference_time=number(c["reference_time"]);input.context.bus_origin=number(c["bus_origin"]);
  input.context.reference=numbers<2>(c["reference"]);input.context.commands=commands_of(c["commands"]);
  for(const auto& p:c["packets"]){x650::Packet packet;packet.send_time=number(p["send_time"]);
#define P(name) packet.name=numbers<2>(p[#name])
    P(q_ref);P(dq_ref);P(tau_ff);P(kp);P(kd);
#undef P
    input.context.packets.push_back(packet);
  }return input;
}
Json::Value packets_json(const std::vector<x650::Packet>& packets) {
  Json::Value rows(Json::arrayValue);for(const auto& p:packets){Json::Value row;row["send_time_s"]=p.send_time;
#define P(name) row[#name]=json_array(p.name)
    P(q_ref);P(dq_ref);P(tau_ff);P(kp);P(kd);
#undef P
    rows.append(row);
  }return rows;
}
Json::Value diagnostic_json(const x650::Diagnostics& d) {
  Json::Value out;
#define S(name) out[#name]=d.name
  S(subproblems);S(active_constraints);S(solver_mode);S(qp_rows);S(qp_columns);S(primal_violation);
  S(kkt_stationarity);S(kkt_complementarity);S(objective);S(q_linearization_error);S(dq_linearization_error);
  S(compute_ms);S(accepted);S(nonlinear_passed);S(actual_bounds);S(previous_plan_fallback);S(candidate_substeps);
#undef S
#define V(name) out[#name]=json_array(d.name)
  V(candidate_envelope);V(horizon);V(plan);V(predicted_states);V(predicted_tip);V(predicted_rotations);
  V(query_times);V(nominal_states);V(candidate_states);V(candidate_extrema);V(candidate_modes);V(candidate_zero_events);V(candidate_boundary_times);
  V(A);V(b);V(H);V(g);V(C);V(lower);V(upper);V(solution);
#undef V
  out["candidate_packets"]=packets_json(d.candidate_packets);out["nominal_packets"]=packets_json(d.nominal_packets);
  out["attempts"]=Json::Value(Json::arrayValue);for(const auto& a:d.attempts){Json::Value row;
    row["passed"]=a.passed;row["actual_bounds"]=a.actual_bounds;row["q_error"]=a.q_error;row["dq_error"]=a.dq_error;row["elapsed_ms"]=a.elapsed_ms;out["attempts"].append(row);}
  return out;
}
int main(int argc,char** argv) {
  if(argc!=5){std::cerr<<"usage: cpp_core_replay predictor.so predictor_sha input.json output.json\n";return 2;}
  try {
    if(sha256(argv[1])!=argv[2])throw std::invalid_argument("Predictor SHA differs");
    void* handle=dlopen(argv[1],RTLD_NOW|RTLD_LOCAL);if(!handle)throw std::runtime_error(dlerror());
    auto fn=reinterpret_cast<x650::Ts20BatchFn>(dlsym(handle,"ts20_prediction_batch"));if(!fn)throw std::runtime_error("Missing TS20 ABI");
    const auto value=read_json(argv[3]);x650::Core core(config_from_json(value["config"]),fn);
    core.restore(memory_of(value["memory"]));const auto input=input_of(value["input"]);
    Json::Value result;result["kind"]="owning_cpp_core_raw_input_replay_v1";result["input_sha256"]=sha256(argv[3]);
    result["loaded_core"]=loaded_identity(reinterpret_cast<void*>(&x650_mpc_core_abi_version));
    result["loaded_optimization"]=loaded_identity(dlsym(RTLD_DEFAULT,"x650_assemble"));
    result["runtime_python_calls"]=0;result["plant_or_flight_claimed"]=false;
    const auto begin=std::chrono::steady_clock::now();
    try {const auto output=core.step(input);
      result["core_step_wall_ms"]=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-begin).count();
      result["pass"]=true;result["command"]=json_array(output.command);
      result["position_reference"]=json_array(output.position_reference);result["velocity_feedforward"]=json_array(output.velocity_feedforward);
      result["diagnostics"]=diagnostic_json(output.diagnostics);
    }catch(const x650::CoreError& e){result["core_step_wall_ms"]=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-begin).count();
      result["pass"]=false;result["error"]=e.what();result["diagnostics"]=diagnostic_json(e.diagnostics);}
    result["step_count_after"]=Json::UInt64(core.memory().step_count);
    write_json(argv[4],result);return result["pass"].asBool()?0:1;
  }catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 2;}
}
