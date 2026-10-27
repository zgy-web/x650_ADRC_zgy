// Persistent processor-in-loop endpoint. No ROS, actuator publisher or log files.
// The server owns end-to-end freshness. Its timestamps remain opaque on NX.
#include "benchmark_io.hpp"
#include "flyarm_mpc_cpp/worker_contract.hpp"
#include "flyarm_mpc_cpp/config_io.hpp"
#include "flyarm_mpc_cpp/diagnostics_json.hpp"
#include <dlfcn.h>
#include <poll.h>
#include <sys/resource.h>
#include <unistd.h>
#include <cerrno>
#include <iostream>
#include <time.h>

extern "C" unsigned x650_mpc_core_abi_version();
extern "C" unsigned x650_optimization_abi_version();
namespace b=x650_benchmark;
namespace fm=flyarm_mpc_cpp;
namespace w=flyarm_control_gateway;
namespace {
constexpr std::size_t MAX_FRAME=131072;
std::uint64_t ns(clockid_t clock=CLOCK_MONOTONIC) {
  timespec t{};b::require(clock_gettime(clock,&t)==0,"Clock unavailable");
  return std::uint64_t(t.tv_sec)*1000000000ULL+std::uint64_t(t.tv_nsec);
}
std::uint64_t uint(const Json::Value& v) {
  b::require(v.isUInt64()&&!v.isBool(),"Unsigned integer required");return v.asUInt64();
}
w::Hash hash(const Json::Value& v) {
  b::require(v.isArray()&&v.size()==32,"SHA byte array required");w::Hash out{};
  for(unsigned i=0;i<32;++i){auto a=uint(v[i]);b::require(a<=255,"Invalid SHA byte");out[i]=a;}return out;
}
w::Snapshot snapshot(const Json::Value& v) {
  b::exact_keys(v,{"schema_version","session_id","epoch","sequence","replay_only","configuration_sha256",
    "issued_monotonic_ns","control_time_s","task_time_s","dt_s","state_time_s","joint_time_s",
    "position_enu","velocity_enu","omega_body_flu","acceleration_enu","rotation_row_major","q","dq",
    "target_position_enu","target_velocity_enu","target_acceleration_enu","position_gains","reference_time_s",
    "bus_origin_s","reference_r","command_count","packet_count","command_time_s","packet_send_time_s",
    "command_velocity","packet_q_ref","packet_dq_ref","packet_tau_ff","packet_kp","packet_kd"});
  w::Snapshot s;
  b::require(uint(v["schema_version"])==1,"Unsupported snapshot schema");s.schema_version=1;
#define U(k) s.k=uint(v[#k])
  U(session_id);U(epoch);U(sequence);U(issued_monotonic_ns);
#undef U
  b::require(uint(v["command_count"])<=64&&uint(v["packet_count"])<=64,"History bound");
  s.command_count=uint(v["command_count"]);s.packet_count=uint(v["packet_count"]);
  s.configuration_sha256=hash(v["configuration_sha256"]);s.replay_only=b::boolean(v["replay_only"]);
#define N(k) s.k=b::number(v[#k])
  N(control_time_s);N(task_time_s);N(dt_s);N(state_time_s);N(joint_time_s);N(reference_time_s);N(bus_origin_s);
#undef N
#define A(k,n) s.k=b::numbers<n>(v[#k])
  A(position_enu,3);A(velocity_enu,3);A(omega_body_flu,3);A(acceleration_enu,3);A(rotation_row_major,9);
  A(q,2);A(dq,2);A(target_position_enu,3);A(target_velocity_enu,3);A(target_acceleration_enu,3);
  A(position_gains,3);A(reference_r,2);A(command_time_s,64);A(packet_send_time_s,64);
  A(command_velocity,128);A(packet_q_ref,128);A(packet_dq_ref,128);A(packet_tau_ff,128);A(packet_kp,128);A(packet_kd,128);
#undef A
  return s;
}
Json::Value proposal_json(const w::Proposal& p) {
  Json::Value v;
#define U(k) v[#k]=Json::UInt64(p.k)
  U(schema_version);U(session_id);U(epoch);U(sequence);U(snapshot_sequence);
  U(compute_start_monotonic_ns);U(compute_end_monotonic_ns);U(status);U(subproblems);
#undef U
#define N(k) v[#k]=p.k
  N(replay_only);N(control_time_s);N(task_time_s);N(accepted_optimum);N(nonlinear_check_passed);N(primal_violation);N(kkt_residual);
#undef N
#define A(k) v[#k]=b::array_json(p.k)
  A(configuration_sha256);A(position_reference_enu);A(velocity_feedforward_enu);A(acceleration_feedforward_enu);
  A(base_velocity_target);A(joint_velocity);
#undef A
  return v;
}
// A length-prefixed UTF-8 frame bounds allocation and handles partial TCP/SSH reads.
bool receive_bytes(void* ptr,std::size_t n,int idle_ms,bool clean_eof=false) {
  auto* p=static_cast<char*>(ptr);std::size_t done=0;auto deadline=ns()+std::uint64_t(idle_ms)*1000000ULL;
  while(done<n){auto now=ns();b::require(now<deadline,"Transport idle/partial-frame timeout");
    pollfd fd{STDIN_FILENO,POLLIN,0};int r=poll(&fd,1,int((deadline-now)/1000000ULL)+1);
    if(r<0&&errno==EINTR)continue;
    b::require(r>0,"Transport idle/partial-frame timeout");
    auto got=read(STDIN_FILENO,p+done,n-done);
    if(got<0&&errno==EINTR)continue;
    if(got==0&&done==0&&clean_eof)return false;
    b::require(got>0,"Truncated transport frame");done+=std::size_t(got);
  }return true;
}
bool frame(std::string& bytes) {
  unsigned char h[4];if(!receive_bytes(h,4,5000,true))return false;
  const std::size_t n=(std::size_t(h[0])<<24)|(std::size_t(h[1])<<16)|(std::size_t(h[2])<<8)|h[3];
  b::require(n>0&&n<=MAX_FRAME,"Frame length outside bound");bytes.resize(n);
  return receive_bytes(bytes.data(),n,2000);
}
void send(const Json::Value& v) {
  Json::StreamWriterBuilder writer;writer["indentation"]="";const auto bytes=Json::writeString(writer,v);
  b::require(bytes.size()<=MAX_FRAME,"Response frame exceeds bound");const auto n=bytes.size();
  const unsigned char h[4]={static_cast<unsigned char>(n>>24),static_cast<unsigned char>(n>>16),static_cast<unsigned char>(n>>8),static_cast<unsigned char>(n)};
  // Backpressure cannot create more commands. The server watchdog owns expiry.
  std::cout.write(reinterpret_cast<const char*>(h),4);std::cout.write(bytes.data(),bytes.size());std::cout.flush();
  b::require(bool(std::cout),"Response transport failed");
}
class Session {
 public:
  explicit Session(x650::Ts20BatchFn batch):batch_(batch){}
  Json::Value handle(const Json::Value& request,std::uint64_t received) {
    b::require(!faulted_&&!stopped_,"Session is latched; restart with a new epoch");
    const auto op=b::string(request["op"]);
    if(op=="start")return start(request,received);
    b::require(bool(core_),"Actual handoff required before requests");
    if(op=="stop"){
      b::exact_keys(request,{"op","session_id","epoch"});identity(request);stopped_=true;
      Json::Value out;out["status"]="stopped";out["steps"]=Json::UInt64(core_->memory().step_count);return out;
    }
    b::require(op=="step","Unknown operation");
    b::exact_keys(request,{"op","snapshot"});auto s=snapshot(request["snapshot"]);
    b::require(s.issued_monotonic_ns>0,"Original source issue required");
    const auto original_issue=s.issued_monotonic_ns;
    // Explicit clock-domain adapter, never an assertion that transport age is zero.
    // The server must retain original_issue and enforce its own full roundtrip.
    s.issued_monotonic_ns=received;
    const auto expected=trajectory_.sample(s.task_time_s,target_);
    reference_difference_=Json::Value();
    for(int i=0;i<3;++i){
      reference_difference_["position"].append(s.target_position_enu[i]-expected.position[i]);
      reference_difference_["velocity"].append(s.target_velocity_enu[i]-expected.velocity[i]);
      reference_difference_["acceleration"].append(s.target_acceleration_enu[i]-expected.acceleration[i]);
    }
    const auto input=contract_->decode(s,received,config_.nodes,config_.node_dt,config_.max_dt);
    const auto before=core_->memory();
    try {
      const auto cpu=ns(CLOCK_THREAD_CPUTIME_ID), begin=ns();auto result=core_->step(input);
      const auto end=ns(), cpu_end=ns(CLOCK_THREAD_CPUTIME_ID);const auto& d=result.diagnostics;
      b::require(d.accepted&&d.nonlinear_passed&&d.actual_bounds&&!d.previous_plan_fallback
        &&!d.attempts.empty()&&d.attempts.size()<=3&&d.subproblems==int(d.attempts.size())
        &&fm::recovery_evidence_valid(d,config_)&&fm::interval_evidence_valid(d,config_)
        &&fm::hold_guard_evidence_valid(d.hold_guard,config_,input),"Native candidate evidence rejected");
      w::Proposal p;p.session_id=session_;p.epoch=epoch_;p.configuration_sha256=hash_;p.replay_only=false;
      p.sequence=p.snapshot_sequence=s.sequence;p.compute_start_monotonic_ns=received;p.compute_end_monotonic_ns=ns();
      p.control_time_s=s.control_time_s;p.task_time_s=s.task_time_s;p.position_reference_enu=result.position_reference;
      p.velocity_feedforward_enu=result.velocity_feedforward;p.acceleration_feedforward_enu=result.acceleration_feedforward;
      std::copy_n(result.command.begin(),3,p.base_velocity_target.begin());p.joint_velocity=result.joint_velocity;
      p.accepted_optimum=p.nonlinear_check_passed=true;p.primal_violation=d.primal_violation;p.kkt_residual=d.kkt_stationarity;p.subproblems=d.subproblems;
      contract_->accept_output(p,ns());
      Json::Value out;out["status"]="candidate";out["proposal_nx_clock"]=proposal_json(p);
      out["source_issue_ns"]=Json::UInt64(original_issue);out["nx_received_ns"]=Json::UInt64(received);
      out["core_wall_ns"]=Json::UInt64(end-begin);out["core_thread_ns"]=Json::UInt64(cpu_end-cpu);
      out["memory_step_count"]=Json::UInt64(core_->memory().step_count);
      out["hold_guard_passed"]=d.hold_guard.passed;out["hold_guard_s"]=d.hold_guard.duration_s;
      out["interval_repairs"]=int(d.interval_repairs.size());out["recovery_triggered"]=d.recovery_triggered;
      Json::Value outputs;
#define A(k) outputs[#k]=b::array_json(result.k)
      A(command);A(position_reference);A(velocity_feedforward);A(acceleration_feedforward);A(joint_velocity);
      A(joint_reference);A(joint_velocity_reference);A(joint_acceleration_reference);A(nominal_acceleration);A(tip_actual);
#undef A
      out["outputs"]=std::move(outputs);return out;
    }catch(...){core_->restore(before);throw;}
  }
  void latch(){faulted_=true;if(contract_)contract_->latch();}
  bool stopped()const{return stopped_;}
  std::uint64_t steps()const{return core_?core_->memory().step_count:0;}
  const Json::Value& reference_difference()const{return reference_difference_;}
 private:
  void identity(const Json::Value& r){b::require(uint(r["session_id"])==session_&&uint(r["epoch"])==epoch_,"Session/epoch mismatch");}
  Json::Value start(const Json::Value& r,std::uint64_t received) {
    b::require(!core_,"Handoff cannot be repeated");
    b::exact_keys(r,{"op","scope","config","trajectory","handoff"});
    scope_=b::string(r["scope"]);b::require(scope_=="recorded_stream"||scope_=="simulation_pil","Simulation or recorded stream only");
    config_=b::config_of(r["config"]);b::require(config_.max_dt==.1&&config_.command_hold_s==.16,"ABI 4 multirate profile required");
    const auto& v=r["handoff"];
    b::exact_keys(v,{"session_id","epoch","configuration_sha256","issued_monotonic_ns","origin_s","anchor_position_enu",
      "anchor_rotation_row_major","anchor_q","target_position_enu","history_count","history_time_s","history_command","ref_alt","yaw_ned"});
    fm::Handoff h;h.session=session_=uint(v["session_id"]);h.epoch=epoch_=uint(v["epoch"]);h.hash=hash_=hash(v["configuration_sha256"]);
    b::require(uint(v["issued_monotonic_ns"])>0,"Original handoff issue required");h.issued_ns=received;
    h.origin=b::number(v["origin_s"]);h.anchor={b::numbers<3>(v["anchor_position_enu"]),b::numbers<9>(v["anchor_rotation_row_major"]),b::numbers<2>(v["anchor_q"])};
    h.target=b::numbers<3>(v["target_position_enu"]);b::require(uint(v["history_count"])<=64,"Handoff history bound");h.count=uint(v["history_count"]);
    h.times=b::numbers<64>(v["history_time_s"]);h.commands=b::numbers<320>(v["history_command"]);
    b::number(v["ref_alt"]);b::number(v["yaw_ned"]);
    w::Config c;c.session_id=session_;c.epoch=epoch_;c.configuration_sha256=hash_;c.position_gains=config_.base_position_gain;c.replay_only=false;
    c.timing_profile=w::TimingProfile::from_name("mpc25_dispatch50_v1");
    trajectory_=fm::trajectory_from_json(r["trajectory"]);target_=h.target;
    contract_=std::make_unique<fm::Contract>(c,trajectory_);
    auto history=contract_->begin(h,ns());core_=std::make_unique<x650::Core>(config_,batch_,x650::DiagnosticMode::Compact);
    core_->reset(h.anchor);core_->prime(history,h.origin);
    Json::Value out;out["status"]="ready";out["scope"]=scope_;out["session_id"]=Json::UInt64(session_);out["epoch"]=Json::UInt64(epoch_);
    out["core_abi"]=4;out["optimization_abi"]=2;out["state_restored_each_step"]=false;out["actuator_publishers"]=0;
    out["hardware_flight_qualified"]=false;return out;
  }
  x650::Ts20BatchFn batch_;x650::Config config_;w::Hash hash_{};std::uint64_t session_{},epoch_{};
  bool faulted_{},stopped_{};std::string scope_;std::unique_ptr<x650::Core> core_;std::unique_ptr<fm::Contract> contract_;
  fm::TrajectoryPlan trajectory_;x650::V3 target_{};Json::Value reference_difference_;
};
}
int main(int argc,char** argv) {
  std::ios::sync_with_stdio(false);rlimit limit{0,0};setrlimit(RLIMIT_CORE,&limit);
  try {
    b::require(argc==7,"Usage: worker predictor predictor_sha core_sha optimization_sha executable_sha source_sha");
    b::require(x650_mpc_core_abi_version()==4&&x650_optimization_abi_version()==2,"Loaded ABI mismatch");
    Dl_info core{},opt{};b::require(dladdr(reinterpret_cast<void*>(&x650_mpc_core_abi_version),&core)&&dladdr(reinterpret_cast<void*>(&x650_optimization_abi_version),&opt),"Library identity unavailable");
    b::require(b::sha256_file(argv[1])==argv[2]&&b::sha256_file(core.dli_fname)==argv[3]
      &&b::sha256_file(opt.dli_fname)==argv[4]&&b::sha256_file("/proc/self/exe")==argv[5]&&b::is_sha256(argv[6]),"Code SHA mismatch");
    void* raw=dlopen(argv[1],RTLD_NOW|RTLD_LOCAL);b::require(raw,"Predictor load failed");
    std::shared_ptr<void> library(raw,[](void* p){dlclose(p);});
    auto batch=reinterpret_cast<x650::Ts20BatchFn>(dlsym(raw,"ts20_prediction_batch"));b::require(batch,"Predictor ABI missing");
    Session session(batch);std::string bytes;bool failed=false;
    try {
      while(frame(bytes)) {
        auto received=ns();Json::Value out;
        try {out=session.handle(b::parse_json(bytes),received);}
        catch(const std::exception& e){session.latch();failed=true;out["status"]="rejected";out["error"]=e.what();out["memory_step_count"]=Json::UInt64(session.steps());out["reference_difference"]=session.reference_difference();}
        out["request_sha256"]=b::sha256_bytes(bytes);out["nx_service_ns"]=Json::UInt64(ns()-received);
        out["source_sha256"]=argv[6];send(out);if(session.stopped())return failed?1:0;
      }
      // EOF without STOP is abnormal and never grants a successful session.
      session.latch();std::cerr<<"EOF before explicit stop; no actuator publisher exists\n";return 2;
    }catch(const std::exception& e){session.latch();std::cerr<<e.what()<<'\n';return 2;}
  }catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 2;}
}
