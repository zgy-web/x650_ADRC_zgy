// Rebuild owning Core memory exclusively from original LiveStart plus raw
// recorded snapshots. Original proposals are comparison oracles, never inputs.
// Uses production Contract::decode and TrajectoryPlan; no Python reconstruction.
#define main x650_single_replay_main
#include "x650_cpp_recovery_replay.cpp"
#undef main
#include "flyarm_mpc_cpp/worker_contract.hpp"
#include "flyarm_mpc_cpp/config_io.hpp"

namespace fm=flyarm_mpc_cpp;
namespace wire=flyarm_control_gateway;
namespace {
std::uint64_t unsigned_of(const Json::Value& v){b::require(v.isUInt64(),"Unsigned wire integer required");return v.asUInt64();}
template<std::size_t N>std::array<std::uint8_t,N> bytes_of(const Json::Value& v){
  b::require(v.isArray()&&v.size()==N,"Fixed wire byte array required");std::array<std::uint8_t,N> out{};
  for(std::size_t i=0;i<N;++i){const auto& x=v[Json::ArrayIndex(i)];b::require(x.isUInt()&&x.asUInt()<=255,"Wire byte differs");out[i]=x.asUInt();}return out;
}
fm::Handoff handoff_of(const Json::Value& v){fm::Handoff out;
  out.session=unsigned_of(v["session_id"]);out.epoch=unsigned_of(v["epoch"]);
  out.issued_ns=unsigned_of(v["issued_monotonic_ns"]);out.hash=bytes_of<32>(v["configuration_sha256"]);
  out.origin=b::number(v["origin_s"]);out.anchor={b::numbers<3>(v["anchor_position_enu"]),b::numbers<9>(v["anchor_rotation_row_major"]),b::numbers<2>(v["anchor_q"])};
  out.target=b::numbers<3>(v["target_position_enu"]);out.count=unsigned_of(v["history_count"]);
  out.times=b::numbers<64>(v["history_time_s"]);out.commands=b::numbers<320>(v["history_command"]);return out;
}
wire::Snapshot snapshot_of(const Json::Value& v){wire::Snapshot out;
#define U(name) out.name=unsigned_of(v[#name])
  U(schema_version);U(session_id);U(epoch);U(sequence);U(issued_monotonic_ns);U(command_count);U(packet_count);
#undef U
  out.replay_only=b::boolean(v["replay_only"]);out.configuration_sha256=bytes_of<32>(v["configuration_sha256"]);
#define D(name) out.name=b::number(v[#name])
  D(control_time_s);D(task_time_s);D(dt_s);D(state_time_s);D(joint_time_s);D(reference_time_s);D(bus_origin_s);
#undef D
#define A(name,n) out.name=b::numbers<n>(v[#name])
  A(position_enu,3);A(velocity_enu,3);A(omega_body_flu,3);A(acceleration_enu,3);A(rotation_row_major,9);A(q,2);A(dq,2);
  A(target_position_enu,3);A(target_velocity_enu,3);A(target_acceleration_enu,3);A(position_gains,3);A(reference_r,2);
  A(command_time_s,64);A(packet_send_time_s,64);A(command_velocity,128);A(packet_q_ref,128);A(packet_dq_ref,128);A(packet_tau_ff,128);A(packet_kp,128);A(packet_kd,128);
#undef A
  return out;
}
wire::Proposal proposal_of(const Json::Value& v){wire::Proposal out;
#define U(name) out.name=unsigned_of(v[#name])
  U(schema_version);U(session_id);U(epoch);U(sequence);U(snapshot_sequence);U(compute_start_monotonic_ns);U(compute_end_monotonic_ns);U(status);U(subproblems);
#undef U
  out.replay_only=b::boolean(v["replay_only"]);out.configuration_sha256=bytes_of<32>(v["configuration_sha256"]);
#define D(name) out.name=b::number(v[#name])
  D(control_time_s);D(task_time_s);D(primal_violation);D(kkt_residual);
#undef D
  out.accepted_optimum=b::boolean(v["accepted_optimum"]);out.nonlinear_check_passed=b::boolean(v["nonlinear_check_passed"]);
#define A(name,n) out.name=b::numbers<n>(v[#name])
  A(position_reference_enu,3);A(velocity_feedforward_enu,3);A(acceleration_feedforward_enu,3);A(base_velocity_target,3);A(joint_velocity,2);
#undef A
  return out;
}
Json::Value input_json(const x650::StepInput& v){Json::Value out;
  out["t"]=v.t;out["dt"]=v.dt;
#define A(name) out["state"][#name]=b::array_json(v.state.name)
  A(position);A(velocity);A(omega_body);A(acceleration);A(rotation);A(q);A(dq);
#undef A
  out["state"]["stamp"]=v.state.stamp;out["state"]["control_time"]=v.state.control_time;out["state"]["joint_stamp"]=v.state.joint_stamp;
  const auto target=[](const x650::Target& t){Json::Value row;row["position"]=b::array_json(t.position);row["velocity"]=b::array_json(t.velocity);row["acceleration"]=b::array_json(t.acceleration);return row;};
  out["target"]=target(v.target);out["preview"]=Json::Value(Json::arrayValue);for(const auto& t:v.preview)out["preview"].append(target(t));
  out["context"]["reference_time"]=v.context.reference_time;out["context"]["bus_origin"]=v.context.bus_origin;
  out["context"]["reference"]=b::array_json(v.context.reference);out["context"]["packets"]=packets_json(v.context.packets);
  out["context"]["commands"]=Json::Value(Json::arrayValue);for(const auto& c:v.context.commands){Json::Value row;row["time"]=c.time;row["velocity"]=b::array_json(c.velocity);out["context"]["commands"].append(row);}return out;
}
Json::Value identity_of(const b::fs::path& path){Json::Value out;out["path"]=b::fs::canonical(path).string();out["bytes"]=Json::UInt64(b::fs::file_size(path));out["sha256"]=b::sha256_file(path);return out;}
Json::Value bound_json(const Json::Value& receipt){const auto path=b::string(receipt["path"]);
  b::require(receipt["bytes"].isUInt64()&&b::fs::file_size(path)==receipt["bytes"].asUInt64(),"Original row byte size differs");
  return b::parse_json(read_bound(path.c_str(),b::string(receipt["sha256"])));
}
double max_difference(const Json::Value& a,const Json::Value& z){b::require(a.isArray()&&z.isArray()&&a.size()==z.size(),"Original result shape differs");double out=0.;
  for(Json::ArrayIndex i=0;i<a.size();++i)out=std::max(out,std::abs(b::number(a[i])-b::number(z[i])));return out;
}
double compare_proposal(const x650::StepResult& r,const wire::Proposal& p){double error=0.;
  auto compare=[&](const auto& actual,const auto& expected){for(std::size_t i=0;i<actual.size();++i)error=std::max(error,std::abs(actual[i]-expected[i]));};
  compare(r.position_reference,p.position_reference_enu);compare(r.velocity_feedforward,p.velocity_feedforward_enu);compare(r.acceleration_feedforward,p.acceleration_feedforward_enu);compare(r.joint_velocity,p.joint_velocity);
  for(int j=0;j<3;++j)error=std::max(error,std::abs(r.command[j]-p.base_velocity_target[j]));return error;
}
} // namespace
int main(int argc,char** argv){
  if(argc!=6){std::cerr<<"usage: x650_cpp_recovery_sequence predictor.so predictor_sha recording.json recording_sha new_output_dir\n";return 2;}
  try{b::require(b::is_sha256(argv[2])&&b::sha256_file(argv[1])==argv[2],"Predictor SHA256 differs");
    const auto document=b::parse_json(read_bound(argv[3],argv[4]));b::require(document["kind"]=="x650_original_cpp_recording_export_v1","Recording kind differs");
    const auto config=b::config_of(document["config"]);const auto trajectory=fm::trajectory_from_json(document["trajectory"]);const auto start=handoff_of(document["live_start"]);
    b::require(document["rows"].isArray()&&!document["rows"].empty()&&document["rows"].size()<=20000,"Bounded nonempty recording required");
    const b::fs::path output_dir=argv[5];b::require(output_dir.is_absolute()&&!b::fs::exists(output_dir),"New absolute output directory required");b::fs::create_directory(output_dir);
    void* handle=dlopen(argv[1],RTLD_NOW|RTLD_LOCAL);if(!handle)throw std::runtime_error(dlerror());
    auto fn=reinterpret_cast<x650::Ts20BatchFn>(dlsym(handle,"ts20_prediction_batch"));b::require(fn,"Missing TS20 ABI");
    x650::Core core(config,fn,x650::DiagnosticMode::Full);wire::Config wc;wc.session_id=start.session;wc.epoch=start.epoch;wc.configuration_sha256=start.hash;wc.position_gains=config.base_position_gain;wc.replay_only=false;
    fm::Contract contract(wc,trajectory);const auto history=contract.begin(start,start.issued_ns);core.reset(start.anchor);core.prime(history,start.origin);
    Json::Value report;report["kind"]="x650_cpp_original_sequence_reconstruction_v1";report["schema_version"]=1;
    report["recording_sha256"]=argv[4];report["loaded_code"]["core"]=identity(reinterpret_cast<void*>(&x650_mpc_core_abi_version));
    report["loaded_code"]["optimization"]=identity(dlsym(RTLD_DEFAULT,"x650_assemble"));report["loaded_code"]["predictor"]=identity(reinterpret_cast<void*>(fn));
    report["core_abi"]=x650_mpc_core_abi_version();report["memory_reconstructed_in_cpp"]=true;report["history_from_proposals"]=false;
    report["counterfactual_closed_loop_claimed"]=false;report["physical_validation_claimed"]=false;report["rows"]=Json::Value(Json::arrayValue);
    bool all_match=true,stopped=false;std::uint64_t last_sequence=0;
    for(const auto& receipt:document["rows"]){const auto row=bound_json(receipt);const auto sequence=unsigned_of(row["sequence"]);
      b::require(sequence==last_sequence+1&&sequence==unsigned_of(receipt["sequence"]),"Recording sequence missing or out of order");last_sequence=sequence;
      const auto s=snapshot_of(row["snapshot"]);const auto p=proposal_of(row["proposal"]);b::require(s.sequence==sequence&&p.sequence==sequence&&p.snapshot_sequence==sequence,"Snapshot/proposal sequence binding differs");
      const auto input=contract.decode(s,p.compute_start_monotonic_ns,config.nodes,config.node_dt);const auto before=core.memory();
      Json::Value fixture;fixture["schema_version"]=1;fixture["config"]=document["config"];fixture["memory"]=memory_json(before);fixture["input"]=input_json(input);
      fixture["label"]["sequence"]=Json::UInt64(sequence);fixture["provenance"]["recording_sha256"]=argv[4];fixture["provenance"]["row_identity"]=receipt;
      fixture["provenance"]["source_snapshot_identity"]=row["snapshot_identity"];fixture["provenance"]["source_proposal_identity"]=row["proposal_identity"];
      fixture["provenance"]["memory_from_original_core_prefix"]=true;fixture["provenance"]["recorded_physical_state"]=true;
      std::ostringstream label;label<<std::setw(6)<<std::setfill('0')<<sequence;const auto fixture_path=output_dir/(label.str()+".input.json");
      Json::Value item;item["sequence"]=Json::UInt64(sequence);item["original_status"]=p.status;item["input_task_time_s"]=input.t;
      try{const auto result=core.step(input);const double error=compare_proposal(result,p);
        b::require(p.status==0&&p.accepted_optimum&&p.nonlinear_check_passed,"Original record was rejected but replay returned");
        item["maximum_proposal_absolute_error"]=error;item["original_proposal_match"]=error<=1e-8;all_match=all_match&&error<=1e-8;
        if(row.isMember("record_identity")){const auto original=bound_json(row["record_identity"]);const auto actual=result_json(result);const auto& saved=original["result"];
          double full_error=0.;for(const auto& name:actual.getMemberNames())full_error=std::max(full_error,max_difference(actual[name],saved[name]));
          item["maximum_full_result_absolute_error"]=full_error;item["original_full_result_match"]=full_error<=1e-8;all_match=all_match&&full_error<=1e-8;}
        b::require(all_match,"Original successful command/result equivalence failed");
        item["returned"]=true;item["core_compute_ms"]=result.diagnostics.compute_ms;
        contract.accept_output(p,p.compute_end_monotonic_ns);
        fixture["oracle"]=result_json(result);fixture["oracle"]["scope"]="original Core regenerated from actual recorded prefix, checked against original proposal/result";
      }catch(const x650::CoreError& e){item["returned"]=false;item["error"]=e.what();item["memory_unchanged"]=b::same_memory(before,core.memory());item["diagnostics"]=diagnostic_json(e.diagnostics);
        const auto original=bound_json(row["record_identity"]);item["original_error"]=original["error"];
        const bool reproduced=p.status!=0&&original["error"].isString()&&original["error"].asString()==e.what()&&b::same_memory(before,core.memory());
        item["original_failure_reproduced"]=reproduced;all_match=all_match&&reproduced;stopped=true;
        fixture["expected_original_error"]=e.what();fixture["original_status"]=p.status;
      }
      write_new(fixture_path.c_str(),fixture);item["fixture"]=identity_of(fixture_path);report["rows"].append(item);
      std::cout<<"sequence "<<sequence<<" "<<(item["returned"].asBool()?"matched":"stopped")<<'\n';
      if(stopped)break;
    }
    report["row_count"]=report["rows"].size();report["complete"]=report["rows"].size()==document["rows"].size();report["all_original_outcomes_reproduced"]=all_match;
    report["stopped_at_original_rejection"]=stopped;report["step_count_after"]=Json::UInt64(core.memory().step_count);
    const auto result_path=output_dir/"report.json";write_new(result_path.c_str(),report);
    return report["complete"].asBool()&&all_match?0:1;
  }catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 2;}
}
