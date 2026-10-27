#include "flyarm_mpc_cpp/worker_contract.hpp"
#include "flyarm_mpc_cpp/config_io.hpp"
#include "flyarm_mpc_cpp/diagnostics_json.hpp"
#include "flyarm_mpc_cpp/thread_placement.hpp"
#include "flyarm_mpc_cpp/bounded_record_queue.hpp"
#include "flyarm_nmpc_cpp/options_io.hpp"
#include "flyarm_nmpc_cpp/send_commit.hpp"
#include "flyarm_nmpc_cpp/fast_json.hpp"
#include "x650_mpc/controller.hpp"
#include <flyarm_control_interfaces/msg/actuator_send.hpp>
#include "flyarm_nmpc_cpp/diagnostics_json.hpp"
#include "flyarm_control_gateway/runtime_wire.hpp"
#include <flyarm_control_interfaces/msg/live_start.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <atomic>
#include <csignal>
#include <dlfcn.h>
#include <iostream>
#include <optional>
#include <semaphore.h>
#include <cerrno>
#include <thread>

extern "C" unsigned x650_mpc_core_abi_version();
namespace fm=flyarm_mpc_cpp;
namespace fn=flyarm_nmpc_cpp;
namespace messages=flyarm_control_interfaces::msg;
namespace wire=flyarm_control_gateway;
namespace {
volatile std::sig_atomic_t stop_requested=0;
void stop_handler(int){stop_requested=1;}
using Bytes=std::vector<std::uint8_t>;
template<class Message> Bytes cdr(const Message& message){rclcpp::Serialization<Message> serializer;rclcpp::SerializedMessage buffer;
  serializer.serialize_message(&message,&buffer);const auto& value=buffer.get_rcl_serialized_message();return Bytes(value.buffer,value.buffer+value.buffer_length);}
void bytes_file(const fm::fs::path& path,const Bytes& bytes){if(fm::fs::exists(path))throw std::runtime_error("Raw CDR evidence already exists");std::ofstream out(path,std::ios::binary);out.write(reinterpret_cast<const char*>(bytes.data()),bytes.size());if(!out)throw std::runtime_error("CDR write failed");}
Json::Value artifact(const fm::fs::path& path){Json::Value out;out["path"]=path.string();out["bytes"]=Json::UInt64(fm::fs::file_size(path));out["sha256"]=fm::sha256(path);return out;}
struct Record{std::uint64_t sequence{},memory_before_step_count{};std::optional<std::uint64_t> selected_next_step_count;Bytes snapshot,proposal;std::optional<x650_nmpc::OnlineResult> nmpc;std::optional<x650::StepResult> mpc;std::string backend;bool valid_proposal_published=false;std::optional<x650::Diagnostics> rejected_diagnostics;std::string error;};
class Recorder {
 public:
  explicit Recorder(fm::fs::path output):output_(std::move(output)){
    phases_.reserve(20000);
    fm::fs::create_directory(output_/"native_records");if(sem_init(&available_,0,0))throw std::runtime_error("Recorder semaphore initialization failed");
    try{thread_=std::thread([this]{run();});}catch(...){sem_destroy(&available_);throw;}
    while(!started_.load(std::memory_order_acquire))std::this_thread::yield();
  }
  ~Recorder(){close();sem_destroy(&available_);}
  bool offer(Record&& value){
    if(failed_||closing_||!queue_.push(std::move(value))){++dropped_;return false;}
    ++offered_;if(sem_post(&available_)){failed_=true;return false;}return true;}
  void close(){if(thread_.joinable()){closing_=true;sem_post(&available_);thread_.join();}}
  bool failed()const{return failed_.load()||dropped_.load()!=0;}
  Json::Value report()const{Json::Value out;out["kind"]="cpp_native_evidence_recorder_v1";out["offered"]=Json::UInt64(offered_);out["written"]=Json::UInt64(written_);out["dropped"]=Json::UInt64(dropped_.load());out["failed"]=failed_.load();out["error"]=error_;out["complete"]=!failed()&&offered_==written_;out["queue_capacity"]=4;out["scope"]="raw received snapshot/proposal CDR and moved actual native call outputs; JSON and file writes on bounded auxiliary recorder only";
    out["json_encoder"]="lossless_std_to_chars_max_digits10_v1";out["recorder_threads"]=1;
    out["phase_columns"]=Json::Value(Json::arrayValue);for(const char* key:{"sequence","begin_ns","pop_end_ns","cdr_end_ns","dom_end_ns","encode_end_ns","write_end_ns","release_end_ns"})out["phase_columns"].append(key);
    out["phase_rows"]=Json::Value(Json::arrayValue);for(const auto& row:phases_){Json::Value values(Json::arrayValue);for(const auto v:row)values.append(Json::UInt64(v));out["phase_rows"].append(std::move(values));}return out;}
 private:
  void run(){started_.store(true,std::memory_order_release);try{for(;;){
      while(sem_wait(&available_)!=0){if(errno!=EINTR)throw std::runtime_error("Recorder semaphore wait failed");}
      std::array<std::uint64_t,8> phase{};phase[1]=fm::monotonic_ns();
      Record value;if(!queue_.pop(value)){if(closing_)break;throw std::runtime_error("Recorder semaphore/owned-slot mismatch");}
      phase[0]=value.sequence;phase[2]=fm::monotonic_ns();if(phases_.size()==phases_.capacity())throw std::runtime_error("Recorder phase evidence capacity exceeded");
      std::ostringstream label;label<<std::setfill('0')<<std::setw(6)<<value.sequence;const auto stem=output_/"native_records"/label.str();
      auto input=stem;input+=".snapshot.cdr";auto proposal=stem;proposal+=".proposal.cdr";auto diagnostic=stem;diagnostic+=".json";
      bytes_file(input,value.snapshot);bytes_file(proposal,value.proposal);Json::Value record;record["kind"]="cpp_full_pipeline_call_v1";record["sequence"]=Json::UInt64(value.sequence);record["snapshot_cdr"]=artifact(input);record["proposal_cdr"]=artifact(proposal);record["error"]=value.error;
      phase[3]=fm::monotonic_ns();
      record["backend"]=value.backend;record["implementation"]=value.backend=="nmpc"?"cpp_upper_nmpc_v1":"cpp_callable_mpc_v1";record["diagnostic_mode"]="full";record["full_matrix_packet_capture"]=true;
      record["valid_proposal_published"]=value.valid_proposal_published;record["memory_before_step_count"]=Json::UInt64(value.memory_before_step_count);if(value.selected_next_step_count)record["selected_next_step_count"]=Json::UInt64(*value.selected_next_step_count);
      if(value.nmpc){record["nmpc"]=fn::diagnostics_json(*value.nmpc);
        if(value.nmpc->solve.selected_iteration>=0)record["result"]=fm::result_json(value.nmpc->solve.output);}
      if(value.mpc)record["result"]=fm::result_json(*value.mpc);
      if(value.rejected_diagnostics)record["rejected_diagnostics"]=fm::diagnostics_json(*value.rejected_diagnostics);
      phase[4]=fm::monotonic_ns();auto encoded=fn::encode_json(record);phase[5]=fm::monotonic_ns();
      if(fm::fs::exists(diagnostic))throw std::runtime_error("Diagnostic evidence already exists");
      {std::ofstream output(diagnostic,std::ios::binary);if(!output)throw std::runtime_error("Cannot create diagnostic evidence");output.write(encoded.data(),encoded.size());output.close();if(!output)throw std::runtime_error("Diagnostic evidence write/close failed");}
      phase[6]=fm::monotonic_ns();value=Record{};record=Json::Value();std::string{}.swap(encoded);phase[7]=fm::monotonic_ns();phases_.push_back(phase);++written_;
    }}catch(const std::exception& error){error_=error.what();failed_=true;}}
  fm::fs::path output_;std::thread thread_;sem_t available_{};fm::BoundedRecordQueue<Record,4> queue_;
  std::vector<std::array<std::uint64_t,8>> phases_;
  std::atomic<bool> started_{},failed_{},closing_{};std::atomic<std::uint64_t> dropped_{};std::uint64_t offered_{},written_{};std::string error_;
};

constexpr std::size_t TIMING_WIDTH=16,TIMING_CAPACITY=20000;
using TimingRow=std::array<std::uint64_t,TIMING_WIDTH>;
class Worker : public rclcpp::Node {
 public:
  Worker():Node("cpp_upper_nmpc"){
    declare_parameter("nmpc_options_manifest",std::string{});declare_parameter("nmpc_options_sha256",std::string{});
    declare_parameter("startup_manifest",std::string{});declare_parameter("startup_sha256",std::string{});declare_parameter("output_dir",std::string{});
    declare_parameter("expected_session_id",std::int64_t{});declare_parameter("expected_epoch",std::int64_t{});declare_parameter("simulation_live",false);
    declare_parameter("executor_cpu",std::int64_t{-1});declare_parameter("auxiliary_cpus",std::vector<std::int64_t>{});
    declare_parameter("timing_profile",std::string("legacy_v1"));
    if(!get_parameter("simulation_live").as_bool())throw std::invalid_argument("Explicit simulation_live=true required; no hardware mode");
    const auto session=get_parameter("expected_session_id").as_int(),epoch=get_parameter("expected_epoch").as_int();
    if(session<=0||epoch<=0)throw std::invalid_argument("Positive session/epoch required");
    session_=session;epoch_=epoch;
    out_=fm::fs::path(get_parameter("output_dir").as_string());if(!out_.is_absolute()||!fm::fs::is_directory(out_))throw std::invalid_argument("Existing absolute owned output directory required");
    startup_=fm::load_startup(get_parameter("startup_manifest").as_string(),get_parameter("startup_sha256").as_string());hash_=flyarm_runtime_wire::parse_hash(startup_.task_hash);
    nmpc_options_=fn::load_options(get_parameter("nmpc_options_manifest").as_string(),get_parameter("nmpc_options_sha256").as_string(),startup_.task_hash);
    timing_profile_=wire::TimingProfile::from_name(get_parameter("timing_profile").as_string());
    if(timing_profile_.control_dt_max_s()!=startup_.config.max_dt||timing_profile_.command_hold_s()!=startup_.config.command_hold_s)
      throw std::runtime_error("Worker timing profile differs from bound native max_dt/command hold");
    Dl_info library{};if(x650_mpc_core_abi_version()!=4||!dladdr(reinterpret_cast<void*>(&x650_mpc_core_abi_version),&library)
        ||fm::fs::canonical(library.dli_fname)!=startup_.core_library)throw std::runtime_error("Loaded complete C++ core differs from bound library");
    Dl_info optimization{};void* assemble=dlsym(RTLD_DEFAULT,"x650_assemble");
    if(!assemble||!dladdr(assemble,&optimization)||fm::fs::canonical(optimization.dli_fname)!=startup_.optimization_library)
      throw std::runtime_error("Actually loaded optimization dependency differs from bound library");
    void* raw=dlopen(startup_.predictor_library.c_str(),RTLD_NOW|RTLD_LOCAL);if(!raw)throw std::runtime_error(std::string("Frozen predictor load failed: ")+dlerror());
    predictor_=std::shared_ptr<void>(raw,[](void* handle){dlclose(handle);});auto batch=reinterpret_cast<x650::Ts20BatchFn>(dlsym(raw,"ts20_prediction_batch"));if(!batch)throw std::runtime_error("Frozen predictor batch ABI missing");
    batch_=batch;
    if(nmpc_options_.backend=="nmpc")core_=std::make_unique<x650::Core>(startup_.config,batch,x650::DiagnosticMode::Compact);
    else callable_=std::make_unique<x650_mpc::Controller>(startup_.config,batch,x650::DiagnosticMode::Full);
    send_commit_=std::make_unique<fn::SendCommit<messages::ControlSnapshot>>(session_,epoch_);commit_timing_.reserve(TIMING_CAPACITY);
    wire::Config config;config.session_id=session_;config.epoch=epoch_;config.configuration_sha256=hash_;config.position_gains=startup_.config.base_position_gain;config.replay_only=false;config.timing_profile=timing_profile_;contract_=std::make_unique<fm::Contract>(config,startup_.trajectory);
    recorder_=std::make_unique<Recorder>(out_);timing_.resize(TIMING_CAPACITY);
    placement_=std::make_unique<fm::ThreadPlacement>(get_parameter("executor_cpu").as_int(),get_parameter("auxiliary_cpus").as_integer_array());
    const auto qos=rclcpp::QoS(rclcpp::KeepLast(1)).best_effort();publisher_=create_publisher<messages::ControlProposal>("/flyarm/isolated/v1/output/proposal",qos);
    snapshots_=create_subscription<messages::ControlSnapshot>("/flyarm/isolated/v1/admitted/snapshot",qos,[this](messages::ControlSnapshot::ConstSharedPtr snapshot){receive(*snapshot);});
    sent_=create_subscription<messages::ActuatorSend>("/flyarm/isolated/v1/actuator_send",rclcpp::QoS(64).best_effort(),[this](messages::ActuatorSend::ConstSharedPtr send){sent(*send);});
    starts_=create_subscription<messages::LiveStart>("/flyarm/isolated/v1/live_start",1,[this](messages::LiveStart::ConstSharedPtr start){begin(*start);});
  }
  void ready(){placement_->bind();Json::Value value;value["kind"]="cpp_full_pipeline_worker_startup_v1";value["implementation"]=implementation();value["backend"]=nmpc_options_.backend;value["nmpc_options_manifest"]=nmpc_options_.manifest;value["python_in_worker"]=false;value["hardware_qualified"]=false;value["startup_manifest"]=startup_.manifest;
    value["configuration_sha256"]=startup_.task_hash;value["session_id"]=Json::UInt64(session_);value["epoch"]=Json::UInt64(epoch_);value["core_abi"]=4;value["startup_controller_steps"]=Json::UInt64(memory().step_count);
    value["timing_profile"]=timing_profile_.name();std::ostringstream profile;wire::write_timing_profile(profile,timing_profile_);
    std::istringstream profile_input(profile.str());Json::CharReaderBuilder reader;std::string profile_error;
    if(!Json::parseFromStream(reader,profile_input,&value["effective_profile"],&profile_error))throw std::runtime_error("Cannot encode effective worker timing profile");
    value["diagnostic_mode"]="full";value["full_matrix_packet_capture"]=true;
    value["memory_protocol"]="pending after proposal publication; commit once only on matching actual ActuatorSend; one reordered admitted snapshot may wait for send notification, never for consumer acknowledgement";
    value["loaded_core_path"]=startup_.core_library.string();value["loaded_optimization_path"]=startup_.optimization_library.string();value["loaded_predictor_path"]=startup_.predictor_library.string();
    if(memory().step_count!=0)throw std::runtime_error("A controller step ran before actual LiveStart");
    fm::write_json(out_/"worker_startup.json",value);fm::write_json(out_/"worker_affinity_ready.json",placement_->report());
    if(fm::fs::exists(out_/"worker_ready"))throw std::runtime_error("Worker READY already exists");
    std::ofstream marker(out_/"worker_ready");marker<<"separate upper NMPC worker constructed; awaiting actual LiveStart\n";if(!marker)throw std::runtime_error("Cannot write READY");}
  bool healthy()const{return !fatal_&&!contract_->faulted()&&!recorder_->failed();}
  void finish(){try{placement_->check("final");}catch(const std::exception& error){fatal_=true;last_error_=error.what();}
    discarded_unsent_sequence_=send_commit_->pending_sequence();
    if(callable_&&pending_token_)callable_->discard(*pending_token_);
    pending_token_.reset();pending_memory_.reset();send_commit_->discard();
    recorder_->close();
    Json::Value commits;commits["kind"]="upper_worker_actual_send_commits_v1";commits["clock"]="same-host CLOCK_MONOTONIC";commits["requires_consumer_ack"]=false;commits["physical_acceptance"]=false;
    commits["columns"]=Json::Value(Json::arrayValue);for(const char* name:{"sequence","dispatch_sequence","sent_monotonic_ns","received_monotonic_ns","committed_memory_steps","buffered_next_sequence"})commits["columns"].append(name);
    commits["rows"]=Json::Value(Json::arrayValue);for(const auto& values:commit_timing_){Json::Value row(Json::arrayValue);for(const auto v:values)row.append(Json::UInt64(v));commits["rows"].append(std::move(row));}
    commits["complete"]=!commit_overflow_;commits["discarded_unsent_sequence"]=Json::UInt64(discarded_unsent_sequence_);fm::write_json(out_/"worker_commit_timing.json",commits);
    Json::Value timing;timing["kind"]="cpp_worker_phase_timing_v1";timing["clock"]="same-host CLOCK_MONOTONIC; CPU columns CLOCK_THREAD_CPUTIME_ID";
    timing["columns"]=Json::Value(Json::arrayValue);for(const char* name:{"sequence","snapshot_issued_ns","callback_entry_ns","core_begin_ns","core_end_ns","publish_begin_ns","publish_end_ns","record_offer_end_ns","callback_exit_ns","thread_cpu_begin_ns","thread_cpu_end_ns","tid_begin","cpu_begin","tid_end","cpu_end","record_offered"})timing["columns"].append(name);
    timing["rows"]=Json::Value(Json::arrayValue);for(std::size_t i=0;i<timing_count_;++i){Json::Value row(Json::arrayValue);for(auto value:timing_[i])row.append(Json::UInt64(value));timing["rows"].append(std::move(row));}
    timing["complete"]=timing_dropped_==0;timing["dropped"]=Json::UInt64(timing_dropped_);timing["capacity"]=Json::UInt64(TIMING_CAPACITY);
    fm::write_json(out_/"worker_timing.json",timing);fm::write_json(out_/"diagnostic_sender.json",recorder_->report());fm::write_json(out_/"worker_affinity_final.json",placement_->report());
    if(!live_start_cdr_.empty())bytes_file(out_/"worker_live_start.cdr",live_start_cdr_);
    Json::Value final;final["kind"]="cpp_full_pipeline_worker_final_v1";final["implementation"]=implementation();final["backend"]=nmpc_options_.backend;final["actual_send_commits"]=Json::UInt64(send_commit_->committed());final["committed_send_sequence"]=Json::UInt64(send_commit_->committed());final["pending_send_sequence"]=Json::UInt64(send_commit_->pending_sequence());final["discarded_unsent_sequence"]=Json::UInt64(discarded_unsent_sequence_);final["controller_steps"]=Json::UInt64(memory().step_count);final["requests"]=Json::UInt64(contract_->requests());final["accepted"]=Json::UInt64(contract_->accepted());final["live_started"]=contract_->started();final["faulted"]=contract_->faulted()||!healthy();final["last_error"]=last_error_;final["configuration_sha256"]=startup_.task_hash;final["no_python_gc_or_pickle"]=true;final["physical_acceptance"]=false;fm::write_json(out_/"worker_final.json",final);}
 private:
  const x650::Memory& memory()const{return core_?core_->memory():callable_->committed_memory();}
  const char* implementation()const{return core_?"cpp_upper_nmpc_v1":"cpp_callable_mpc_v1";}
  void receive(const messages::ControlSnapshot& snapshot){try{
    if(snapshot.session_id!=session_||snapshot.epoch!=epoch_||snapshot.configuration_sha256!=hash_)
      throw std::runtime_error("Admitted snapshot identity differs from worker");
    if(send_commit_->pending()){send_commit_->defer(snapshot);return;}
    process(snapshot);
  }catch(const std::exception& error){fatal_=true;contract_->latch();last_error_=error.what();}}
  void sent(const messages::ActuatorSend& message){try{
    const auto received=fm::monotonic_ns();
    const fn::SendReceipt s{message.session_id,message.epoch,message.command_sequence,message.dispatch_sequence,message.sent_monotonic_ns,message.sim_s,message.stop_zero,message.expected_counter,message.joint_velocity,message.coordinate_frame,message.type_mask,message.position_ned,message.velocity_ned,message.acceleration_ned,message.yaw_ned};
    if(!send_commit_->matches(s,received))return;
    if(!healthy())throw std::runtime_error("Cannot commit after worker fault");
    if(commit_timing_.size()==commit_timing_.capacity()){commit_overflow_=true;throw std::runtime_error("Actual send evidence overflow");}
    const auto buffered=send_commit_->buffered_sequence();
    if(core_){if(!pending_memory_)throw std::logic_error("Missing NMPC pending Memory");core_->restore(*pending_memory_);pending_memory_.reset();}
    else{if(!pending_token_)throw std::logic_error("Missing callable MPC transaction");callable_->commit_published(*pending_token_,pending_command_);pending_token_.reset();}
    auto next=send_commit_->complete();
    commit_timing_.push_back({s.sequence,s.dispatch,s.sent_ns,received,memory().step_count,buffered});
    if(next)process(*next); // Original issue timestamp and actual history retained.
  }catch(const std::exception& error){fatal_=true;contract_->latch();last_error_=error.what();}}
  void begin(const messages::LiveStart& value){try{placement_->check("live_start");fm::Handoff start;start.session=value.session_id;start.epoch=value.epoch;start.hash=value.configuration_sha256;start.issued_ns=value.issued_monotonic_ns;start.origin=value.origin_s;
    start.anchor={value.anchor_position_enu,value.anchor_rotation_row_major,value.anchor_q};start.target=value.target_position_enu;start.count=value.history_count;start.times=value.history_time_s;start.commands=value.history_command;
    const auto history=contract_->begin(start,fm::monotonic_ns());
    if(core_){core_->reset(start.anchor);core_->prime(history,start.origin);}else callable_->reset(start.anchor,history,start.origin);
    send_commit_->begin(value.ref_alt,value.yaw_ned);
    live_start_cdr_=cdr(value);
  }catch(const std::exception& error){contract_->latch();fatal_=true;last_error_=error.what();}}
  void process(const messages::ControlSnapshot& snapshot){TimingRow row{};row[0]=snapshot.sequence;row[1]=snapshot.issued_monotonic_ns;row[2]=fm::monotonic_ns();row[9]=fm::thread_cpu_ns();Record record;record.sequence=snapshot.sequence;record.backend=nmpc_options_.backend;record.memory_before_step_count=memory().step_count;
    messages::ControlProposal proposal;proposal.schema_version=1;proposal.session_id=session_;proposal.epoch=epoch_;proposal.sequence=proposal.snapshot_sequence=snapshot.sequence;proposal.configuration_sha256=hash_;proposal.replay_only=false;proposal.status=1;proposal.compute_start_monotonic_ns=row[2];
    const auto invalidate=[&]{if(proposal.status==0)proposal.status=2;proposal.accepted_optimum=proposal.nonlinear_check_passed=false;proposal.position_reference_enu={};proposal.velocity_feedforward_enu={};proposal.acceleration_feedforward_enu={};proposal.base_velocity_target={};proposal.joint_velocity={};record.error=last_error_;};
    const auto expired=[&](std::uint64_t now){return snapshot.issued_monotonic_ns==0||now<snapshot.issued_monotonic_ns||now-snapshot.issued_monotonic_ns>50000000ULL;};
    try{const auto cpu=placement_->observe();row[11]=cpu[0];row[12]=cpu[1];if(!healthy())throw std::runtime_error("Worker/recorder has latched fault");
      const auto input=contract_->decode(flyarm_runtime_wire::from_ros(snapshot),row[2],startup_.config.nodes,startup_.config.node_dt,startup_.config.max_dt);proposal.control_time_s=snapshot.control_time_s;proposal.task_time_s=snapshot.task_time_s;proposal.status=2;
      row[3]=fm::monotonic_ns();
      if(core_){
        record.nmpc=x650_nmpc::solve_online(startup_.config,batch_,memory(),input,snapshot.issued_monotonic_ns,nmpc_options_.options);
        if(!record.nmpc->accepted)throw std::runtime_error("Online NMPC rejected candidate: "+record.nmpc->status);
        pending_memory_=record.nmpc->solve.proposed_memory;
      }else{
        const auto& candidate=callable_->prepare(input);pending_token_=candidate.token;record.mpc=candidate.output;
      }
      row[4]=fm::monotonic_ns();
      const auto& result=record.nmpc?record.nmpc->solve.output:*record.mpc;const auto& d=result.diagnostics;
      pending_command_=result.command;record.selected_next_step_count=memory().step_count+1;
      if(!d.accepted||!d.nonlinear_passed||!d.actual_bounds||d.previous_plan_fallback||d.attempts.empty()||d.attempts.size()>3||d.subproblems!=int(d.attempts.size())||!fm::recovery_evidence_valid(d,startup_.config)||!fm::interval_evidence_valid(d,startup_.config)||!fm::hold_guard_evidence_valid(d.hold_guard,startup_.config,input))throw std::runtime_error("Selected NMPC native optimum/nonlinear/physical-bound, recovery, interval or held-command evidence missing");
      if(pending_memory_&&(pending_memory_->step_count!=memory().step_count+1||pending_memory_->previous_command!=result.command))
        throw std::runtime_error("NMPC internal iterations changed publication-step memory contract");
      proposal.position_reference_enu=result.position_reference;proposal.velocity_feedforward_enu=result.velocity_feedforward;proposal.acceleration_feedforward_enu=result.acceleration_feedforward;
      std::copy_n(result.command.begin(),3,proposal.base_velocity_target.begin());proposal.joint_velocity=result.joint_velocity;
      // Wire subproblems describes the selected native QP certificate. Total
      // outer proposals and true objective values are recorded separately.
      proposal.accepted_optimum=true;proposal.nonlinear_check_passed=true;proposal.primal_violation=d.primal_violation;proposal.kkt_residual=d.kkt_stationarity;proposal.subproblems=d.attempts.size();proposal.status=0;
      proposal.compute_end_monotonic_ns=fm::monotonic_ns();
      if(expired(proposal.compute_end_monotonic_ns)){proposal.status=3;throw std::runtime_error("NMPC original issue-to-candidate deadline exceeded unchanged 50 ms");}
      contract_->accept_output(flyarm_runtime_wire::from_ros(proposal),proposal.compute_end_monotonic_ns);
    }catch(x650::CoreError& error){record.rejected_diagnostics=std::move(error.diagnostics);last_error_=error.what();contract_->latch();proposal.status=2;}
    catch(const std::exception& error){last_error_=error.what();contract_->latch();}
    proposal.compute_end_monotonic_ns=fm::monotonic_ns();
    if(expired(proposal.compute_end_monotonic_ns)){proposal.status=3;contract_->latch();last_error_="NMPC original issue-to-publication deadline exceeded unchanged 50 ms";}
    if(contract_->faulted())invalidate();
    try{row[5]=fm::monotonic_ns();
      // Check again immediately before publisher entry; never refresh the
      // original issue timestamp to hide dispatch, decoding or scoring time.
      if(proposal.status==0&&expired(row[5])){proposal.status=3;contract_->latch();last_error_="NMPC final publication deadline exceeded unchanged 50 ms";invalidate();}
      if(proposal.status==0)send_commit_->stage({proposal.sequence,row[5],pending_command_,proposal.position_reference_enu,proposal.velocity_feedforward_enu});
      publisher_->publish(proposal);row[6]=fm::monotonic_ns();
      if(proposal.status==0)record.valid_proposal_published=true;
      record.snapshot=cdr(snapshot);record.proposal=cdr(proposal);row[15]=recorder_->offer(std::move(record));row[7]=fm::monotonic_ns();
      if(!row[15])throw std::runtime_error("Bounded native recording queue rejected actual call evidence");
      const auto cpu=placement_->observe();row[13]=cpu[0];row[14]=cpu[1];
    }catch(const std::exception& error){fatal_=true;contract_->latch();last_error_=error.what();}
    row[10]=fm::thread_cpu_ns();row[8]=fm::monotonic_ns();if(timing_count_<timing_.size())timing_[timing_count_++]=row;else{++timing_dropped_;fatal_=true;contract_->latch();}
  }
  fm::Startup startup_;fn::BoundOptions nmpc_options_;x650::Ts20BatchFn batch_{};fm::fs::path out_;std::uint64_t session_{},epoch_{};wire::Hash hash_{};wire::TimingProfile timing_profile_;
  std::shared_ptr<void> predictor_;std::unique_ptr<x650::Core> core_;std::unique_ptr<x650_mpc::Controller> callable_;std::unique_ptr<fn::SendCommit<messages::ControlSnapshot>> send_commit_;
  std::optional<x650::Memory> pending_memory_;std::optional<x650_mpc::Token> pending_token_;x650::V5 pending_command_{};
  std::vector<std::array<std::uint64_t,6>> commit_timing_;std::uint64_t discarded_unsent_sequence_{};bool commit_overflow_{};std::unique_ptr<fm::Contract> contract_;std::unique_ptr<Recorder> recorder_;std::unique_ptr<fm::ThreadPlacement> placement_;
  std::vector<TimingRow> timing_;std::size_t timing_count_{},timing_dropped_{};bool fatal_{};std::string last_error_;Bytes live_start_cdr_;
  rclcpp::Publisher<messages::ControlProposal>::SharedPtr publisher_;rclcpp::Subscription<messages::ControlSnapshot>::SharedPtr snapshots_;rclcpp::Subscription<messages::LiveStart>::SharedPtr starts_;rclcpp::Subscription<messages::ActuatorSend>::SharedPtr sent_;
};
}
int main(int argc,char** argv){rclcpp::init(argc,argv,rclcpp::InitOptions{},rclcpp::SignalHandlerOptions::None);std::signal(SIGINT,stop_handler);std::signal(SIGTERM,stop_handler);int result=0;
  std::shared_ptr<Worker> worker;try{worker=std::make_shared<Worker>();rclcpp::executors::SingleThreadedExecutor executor;executor.add_node(worker);worker->ready();
    while(!stop_requested&&rclcpp::ok()&&worker->healthy())executor.spin_once(std::chrono::milliseconds(20));
    if(!worker->healthy())result=1;
    worker->finish();if(!worker->healthy())result=1;executor.remove_node(worker);
  }catch(const std::exception& error){std::cerr<<error.what()<<'\n';result=1;}worker.reset();rclcpp::shutdown();return result;}
