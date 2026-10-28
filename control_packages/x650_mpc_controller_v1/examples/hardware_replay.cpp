#include "x650_mpc/controller.hpp"
#include "x650_mpc/selected_profile.hpp"
#include "x650_mpc/px4_replay_adapter.hpp"
#include "benchmark_io.hpp"
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <dlfcn.h>
#include <iostream>
#include <tuple>

extern "C" int ts20_prediction_batch(const double*,const double*,double,double,const double*,const double*,const double*,int,const double*,int,int,double*,int*,int*,double*,double*,int,int*,int*,char*,int);
extern "C" unsigned x650_optimization_abi_version();
namespace b=x650_benchmark;
namespace pm=px4_msgs::msg;

template<class T>T cdr(const T& in) {
  rclcpp::Serialization<T> codec;rclcpp::SerializedMessage bytes;T out;
  codec.serialize_message(&in,&bytes);codec.deserialize_message(&bytes,&out);return out;
}
template<std::size_t N>std::array<float,N> floats(const Json::Value& j) {
  const auto in=b::numbers<N>(j);std::array<float,N> out{};
  std::copy(in.begin(),in.end(),out.begin());return out;
}
template<class A,class B>double error(const A& a,const B& z) {
  b::require(a.size()==z.size(),"Comparison shape mismatch");double e=0;
  for(std::size_t i=0;i<a.size();++i){b::require(std::isfinite(a[i])&&std::isfinite(z[i]),"Nonfinite comparison");e=std::max(e,std::abs(a[i]-z[i]));}return e;
}
std::array<std::vector<double>,10> fields(const x650::StepResult& r) {
  std::array<std::vector<double>,10> out;std::size_t at=0;
  const auto add=[&](const auto& x){out[at++]={x.begin(),x.end()};};
  add(r.command);add(r.position_reference);add(r.velocity_feedforward);add(r.acceleration_feedforward);
  add(r.joint_velocity);add(r.joint_reference);add(r.joint_velocity_reference);add(r.joint_acceleration_reference);add(r.nominal_acceleration);add(r.tip_actual);return out;
}
bool same_config(const x650::Config& a,const x650::Config& c) {
  const auto f=[](const x650::Config& v){return std::tie(v.nodes,v.max_iterations,v.node_dt,v.max_dt,v.command_hold_s,v.feasibility_tolerance,v.kkt_tolerance,v.arm_share,v.joint_position_gain,v.weights,v.velocity_limit,v.acceleration_limit,v.q_lower,v.q_upper,v.physical_lower,v.physical_upper,v.geometry,v.response,v.base_position_gain,v.motion,v.kp,v.kd,v.bus_period,v.integration_step,v.trust_radius);};return f(a)==f(c);
}
Json::Value loaded(void* symbol) {
  Dl_info info{};b::require(dladdr(symbol,&info)&&info.dli_fname,"Cannot identify loaded library");
  const auto path=b::fs::canonical(info.dli_fname);Json::Value out;out["path"]=path.string();out["sha256"]=b::sha256_file(path);return out;
}
x650::State decoded_state(const Json::Value& f,double control_time) {
  x650_mpc::Px4ReplayAdapter adapter({"joint_1","joint_2"});
  const auto us=f["sample_us"].asUInt64();constexpr double receipt=100.;
  pm::VehicleAttitude a;a.timestamp=a.timestamp_sample=us;a.q=floats<4>(f["q_ned_frd"]);
  pm::VehicleLocalPosition p;p.timestamp=p.timestamp_sample=us;
  const auto pos=floats<3>(f["position_ned"]),v=floats<3>(f["velocity_ned"]),acc=floats<3>(f["acceleration_ned"]);
  p.x=pos[0];p.y=pos[1];p.z=pos[2];p.vx=v[0];p.vy=v[1];p.vz=v[2];p.ax=acc[0];p.ay=acc[1];p.az=acc[2];
  p.ref_alt=f["ignored_ref_alt_m"].asFloat();
  p.xy_valid=p.z_valid=p.v_xy_valid=p.v_z_valid=p.heading_good_for_control=true;
  pm::VehicleOdometry o;o.timestamp=o.timestamp_sample=us;o.angular_velocity=floats<3>(f["omega_frd"]);
  pm::VehicleStatus s;s.timestamp=us;s.system_id=s.component_id=1;s.arming_state=pm::VehicleStatus::ARMING_STATE_DISARMED;
  sensor_msgs::msg::JointState j;const auto ns=f["joint_ns"].asUInt64();
  j.header.stamp.sec=ns/1'000'000'000;j.header.stamp.nanosec=ns%1'000'000'000;
  // Deliberately reverse wire order to catch index-based joint assignment.
  j.name={"joint_2","joint_1"};j.position={b::number(f["q"][1]),b::number(f["q"][0])};
  j.velocity={b::number(f["dq"][1]),b::number(f["dq"][0])};
  adapter.receiver.receive(cdr(a),receipt);adapter.receiver.receive(cdr(p),receipt);
  adapter.receiver.receive(cdr(o),receipt);adapter.receiver.receive(cdr(s),receipt);adapter.receive_joints(cdr(j),receipt);
  return adapter.state(control_time,receipt+.001);
}
double state_error(const x650::State& s,const Json::Value& e) {
  double out=0;
#define CHECK_ARRAY(name,n) out=std::max(out,error(s.name,b::numbers<n>(e[#name])))
  CHECK_ARRAY(position,3);CHECK_ARRAY(velocity,3);CHECK_ARRAY(acceleration,3);CHECK_ARRAY(omega_body,3);
  CHECK_ARRAY(rotation,9);CHECK_ARRAY(q,2);CHECK_ARRAY(dq,2);
#undef CHECK_ARRAY
  for(const auto& p:{std::pair{s.stamp,b::number(e["stamp"])},std::pair{s.control_time,b::number(e["control_time"])},std::pair{s.joint_stamp,b::number(e["joint_stamp"])}})
    out=std::max(out,std::abs(p.first-p.second));return out;
}
bool output_fields(const pm::TrajectorySetpoint& m,const x650::StepResult& r,std::uint64_t stamp,double yaw) {
  // Independent direct field mapping, not calling the production encoder.
  bool ok=m.timestamp==stamp&&std::isnan(m.yawspeed);
  const std::array<std::size_t,3> index{1,0,2};const std::array<double,3> sign{1,1,-1};
  for(std::size_t i=0;i<3;++i)ok=ok&&m.position[i]==float(sign[i]*r.position_reference[index[i]])&&
    m.velocity[i]==float(sign[i]*r.velocity_feedforward[index[i]])&&
    m.acceleration[i]==float(sign[i]*r.acceleration_feedforward[index[i]])&&std::isnan(m.jerk[i]);
  const double target=std::acos(-1.)/2-yaw;
  return ok&&m.yaw==float(std::atan2(std::sin(target),std::cos(target)));
}
Json::Value wire_json(const pm::TrajectorySetpoint& m) {
  Json::Value j;j["timestamp"]=Json::UInt64(m.timestamp);j["position"]=b::array_json(m.position);
  j["velocity"]=b::array_json(m.velocity);j["acceleration"]=b::array_json(m.acceleration);j["yaw"]=m.yaw;
  j["jerk"]=Json::Value(Json::arrayValue);for(int i=0;i<3;++i)j["jerk"].append(Json::Value());
  j["yawspeed"]=Json::Value();j["null_represents_nan"]=true;return j;
}
int main(int argc,char** argv) {
  Json::Value report;report["kind"]="x650_hardware_interface_replay_result_v1";report["complete"]=false;
  report["physical_validation"]=false;report["raw_cdr_recording"]=false;report["publishers_created"]=0;
  report["mpc_mode"]="fixed_snapshot_restore_then_original_ABI4_solve";
  report["rows"]=Json::Value(Json::arrayValue);
  try {
    b::require(argc==3,"usage: x650_hardware_replay input.json SHA256");
    b::require(b::is_sha256(argv[2])&&b::sha256_file(argv[1])==argv[2],"Replay input SHA mismatch");
    std::ifstream f(argv[1]);const auto root=b::parse_json(std::string((std::istreambuf_iterator<char>(f)),{}));
    b::require(root["kind"]=="x650_hardware_interface_replay_v1"&&!root["physical_validation"].asBool()&&!root["raw_cdr_recording"].asBool(),"Wrong replay scope");
    b::require(root["joint_names"][0]=="joint_1"&&root["joint_names"][1]=="joint_2","Joint fixture mapping differs");
    const auto data=b::dataset_of(root["recorded"]);b::require(root["frames"].size()==data.cases.size(),"Frame count mismatch");
    b::require(same_config(data.config,x650_mpc::selected_x650_simulation_profile()),"Selected profile differs");
    report["input_sha256"]=argv[2];report["source_sha256"]=root["source_sha256"];
    report["loaded_core"]=loaded(reinterpret_cast<void*>(&x650_mpc_core_abi_version));
    report["loaded_optimizer"]=loaded(reinterpret_cast<void*>(&x650_optimization_abi_version));
    report["loaded_predictor"]=loaded(reinterpret_cast<void*>(&ts20_prediction_batch));
    x650_mpc::Controller wrapper(data.config,ts20_prediction_batch,x650::DiagnosticMode::Full);
    x650::Core direct(data.config,ts20_prediction_batch,x650::DiagnosticMode::Full);
    bool all=true;std::size_t n=0;
    for(const auto& row:data.cases) {
      const auto& frame=root["frames"][Json::ArrayIndex(n++)];Json::Value item;item["id"]=row.id;
      try {
        b::require(frame["id"]==row.id,"Frame identity mismatch");
        direct.restore(row.memory);const auto baseline=direct.step(row.input);const auto baseline_fields=fields(baseline);
        bool historical=true;double historical_error=0;
        for(std::size_t k=0;k<10;++k)if(!row.oracle.fields[k].empty()) {
          const double e=error(baseline_fields[k],row.oracle.fields[k]);historical_error=std::max(historical_error,e);
          historical=historical&&e<=(k==1?1e-9:1e-7);
        }
        auto input=row.input;input.state=decoded_state(frame,input.state.control_time);
        const double state_e=state_error(input.state,frame["expected_state"]);
        direct.restore(row.memory);const auto expected=direct.step(input);
        wrapper.restore_for_replay(row.memory);const auto& candidate=wrapper.prepare(input);
        const bool before=b::same_memory(wrapper.committed_memory(),row.memory);
        const auto actual=fields(candidate.output),reference=fields(expected);
        const auto& a=candidate.output.diagnostics;const auto& z=expected.diagnostics;
        const bool exact=actual==reference&&a.A==z.A&&a.b==z.b&&a.H==z.H&&a.g==z.g&&a.C==z.C&&a.lower==z.lower&&a.upper==z.upper&&a.solution==z.solution&&a.plan==z.plan&&a.predicted_states==z.predicted_states&&a.candidate_envelope==z.candidate_envelope&&a.hold_guard.envelope==z.hold_guard.envelope;
        const auto stamp=std::uint64_t(std::llround(input.state.control_time*1e6));
        const auto message=x650_mpc::encode_mpc_output(candidate.output,stamp,b::number(frame["yaw_enu_rad"]));
        const auto decoded=cdr(message);
        const bool encoded=output_fields(message,candidate.output,stamp,frame["yaw_enu_rad"].asDouble())&&
                           output_fields(decoded,candidate.output,stamp,frame["yaw_enu_rad"].asDouble());
        const bool joints=candidate.output.joint_velocity==x650::V2{candidate.output.command[3],candidate.output.command[4]};
        // Simulated local publication transaction only; no DDS publisher exists.
        const auto token=candidate.token;const auto command=candidate.output.command;
        wrapper.commit_published(token,command);const bool committed=b::same_memory(wrapper.committed_memory(),direct.memory());
        item["adapter_state_max_error"]=state_e;item["historical_baseline_match"]=historical;
        item["historical_baseline_max_error"]=historical_error;item["facade_vs_direct_exact"]=exact;
        item["prepare_did_not_commit"]=before;item["commit_memory_exact"]=committed;
        item["cdr_setpoint_fields_exact"]=encoded;item["joint_output_identity"]=joints;
        item["quantized_input_command_difference"]=error(actual[0],baseline_fields[0]);
        item["quantized_input_command_absolute_differences"]=Json::Value(Json::arrayValue);
        for(std::size_t axis=0;axis<5;++axis)
          item["quantized_input_command_absolute_differences"].append(std::abs(actual[0][axis]-baseline_fields[0][axis]));
        item["quantized_input_position_reference_difference"]=error(actual[1],baseline_fields[1]);
        item["expected_setpoint"]=wire_json(decoded);
        item["passed"]=state_e<=2e-12&&historical&&exact&&before&&committed&&encoded&&joints;
      } catch(const std::exception& e) {item["error"]=e.what();item["passed"]=false;}
      all=all&&item["passed"].asBool();report["rows"].append(item);
    }
    report["row_count"]=Json::UInt64(n);report["passed"]=all;report["complete"]=true;
    report["scope"]="Reconstructed typed PX4 and JointState messages, in-memory CDR, shared bench adapter, original MPC, setpoint encoding and simulated commit. No ROS graph, USB, physical consumer, live clock synchronization, continuous controller history or real joint driver qualification.";
  } catch(const std::exception& e) {report["error"]=e.what();report["passed"]=false;}
  Json::StreamWriterBuilder writer;writer["indentation"]="";
  std::cout<<Json::writeString(writer,report)<<'\n';return report["passed"].asBool()?0:1;
}
