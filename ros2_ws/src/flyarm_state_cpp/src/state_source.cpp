#include "flyarm_state_cpp/source_core.hpp"
#include "flyarm_state_cpp/csv_source.hpp"
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <px4_msgs/msg/vehicle_angular_velocity.hpp>
#include <px4_msgs/msg/estimator_status_flags.hpp>
#include <flyarm_control_interfaces/msg/control_snapshot.hpp>
#include <flyarm_control_interfaces/msg/live_start.hpp>
#include <flyarm_control_interfaces/msg/source_binding.hpp>
#include <flyarm_control_interfaces/msg/source_binding_ack.hpp>
#include <flyarm_control_interfaces/msg/runtime_stop.hpp>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <cstring>
#include <time.h>

namespace fs=std::filesystem;
namespace core=flyarm_state_cpp;
namespace msg=flyarm_control_interfaces::msg;
static std::uint64_t monotonic_ns(){timespec t{};core::require(clock_gettime(CLOCK_MONOTONIC,&t)==0,"CLOCK_MONOTONIC failed");
  return static_cast<std::uint64_t>(t.tv_sec)*1000000000ULL+static_cast<std::uint64_t>(t.tv_nsec);}
static std::array<std::uint8_t,32> parse_hash(const std::string& text){core::require(text.size()==64,"Expected lowercase SHA256");
  auto digit=[](char c)->unsigned{if(c>='0'&&c<='9')return c-'0';if(c>='a'&&c<='f')return c-'a'+10;throw std::runtime_error("Invalid lowercase SHA256");};
  std::array<std::uint8_t,32>a{};for(std::size_t i=0;i<32;++i)a[i]=16*digit(text[2*i])+digit(text[2*i+1]);return a;}
static std::string escaped(const std::string& s){std::string o;for(char c:s){if(c=='"'||c=='\\')o+='\\';if(c=='\n')o+="\\n";else o+=c;}return o;}

// Preallocated raw-wire evidence. No diagnostic DDS publisher, Python object,
// JSON encoding or output-file write is on the ACTIVE path. CSV reads are the
// physical simulator input and remain independently audited at the writer.
class WireEvidence {
 public:
  WireEvidence(){bytes_.reserve(capacity);}
  template<class M> void append(std::uint32_t kind,std::uint64_t wall,std::uint64_t sequence,double sim,const M& message){
    rclcpp::SerializedMessage buffer; rclcpp::Serialization<M> serializer;serializer.serialize_message(&message,&buffer);
    const auto& raw=buffer.get_rcl_serialized_message();
    core::require(bytes_.size()+32+raw.buffer_length<=capacity,"Raw source evidence capacity exceeded");
    put(kind);put(wall);put(sequence);put(sim);put(static_cast<std::uint32_t>(raw.buffer_length));
    bytes_.insert(bytes_.end(),raw.buffer,raw.buffer+raw.buffer_length);++records_;
  }
  void write(const fs::path& path){std::ofstream f(path,std::ios::binary);f.write("FSCdr001",8);f.write(reinterpret_cast<const char*>(bytes_.data()),bytes_.size());
    f.close();core::require(bool(f),"Source CDR evidence write failed");}
  std::uint64_t records()const{return records_;}
 private:
  template<class T>void put(T value){const auto* p=reinterpret_cast<const std::uint8_t*>(&value);bytes_.insert(bytes_.end(),p,p+sizeof value);}
  static constexpr std::size_t capacity=128*1024*1024;
  std::vector<std::uint8_t>bytes_;std::uint64_t records_{};
};

class StateSource final:public rclcpp::Node {
 public:
  StateSource():Node("isolated_raw_state_source") {
    core::require(declare_parameter<bool>("simulation_live",false),"Explicit simulation_live required; hardware is unqualified");
    auto session=declare_parameter<std::int64_t>("expected_session_id",0),epoch=declare_parameter<std::int64_t>("expected_epoch",0);
    core::require(session>0&&epoch>0,"Explicit positive session and epoch required");session_=session;epoch_=epoch;
    hash_text_=declare_parameter<std::string>("configuration_sha256","");hash_=parse_hash(hash_text_);
    const auto state_path=declare_parameter<std::string>("state_csv","");const auto packet_path=declare_parameter<std::string>("packet_csv","");
    for(const auto& p:{state_path,packet_path})core::require(fs::path(p).is_absolute()&&fs::is_regular_file(p),"Existing absolute physical TS20 CSV required");
    state_path_=state_path;packet_path_=packet_path;
    states_=std::make_unique<core::CsvTail>(state_path);packets_=std::make_unique<core::CsvTail>(packet_path);
    ready_path_=declare_parameter<std::string>("ready_file","");output_=declare_parameter<std::string>("evidence_directory","");
    core::require(fs::path(ready_path_).is_absolute()&&!fs::exists(ready_path_)&&output_.is_absolute()&&!fs::exists(output_),
      "New absolute readiness/evidence paths required (no overwrites)");fs::create_directories(output_);
    const auto nan=std::numeric_limits<double>::quiet_NaN();
    bus_period_=declare_parameter<double>("bus_period_s",nan);delay_=declare_parameter<double>("delay_s",nan);
    history_=std::make_unique<core::JointHistory>(bus_period_,delay_);
    const auto gains=declare_parameter<std::vector<double>>("position_gains",std::vector<double>{});
    core::require(gains.size()==3&&core::all_finite(gains),"Three frozen position gains required");std::copy(gains.begin(),gains.end(),gains_.begin());
    const auto kind=declare_parameter<std::string>("trajectory_kind","");core::require(kind=="hold"||kind=="figure8","Explicit hold or figure8 trajectory required");
    trajectory_.figure8=kind=="figure8";trajectory_.ax=declare_parameter<double>("amplitude_x",0);trajectory_.az=declare_parameter<double>("amplitude_z",0);
    trajectory_.period=declare_parameter<double>("period_s",12);trajectory_.start=declare_parameter<double>("start_s",2);trajectory_.ramp=declare_parameter<double>("ramp_s",2);
    const auto cycles=declare_parameter<std::int64_t>("cycles",1);core::require(cycles>=1&&cycles<=10000,"Invalid trajectory cycles");trajectory_.cycles=cycles;trajectory_.validate();
    samples_.reserve(100000);
    const auto latest=rclcpp::QoS(1).best_effort();
    source_=create_publisher<msg::ControlSnapshot>("/flyarm/isolated/v1/source",latest);
    stop_out_=create_publisher<msg::RuntimeStop>("/flyarm/isolated/v1/stop",rclcpp::QoS(1).reliable());
    binding_ack_=create_publisher<msg::SourceBindingAck>("/flyarm/isolated/v1/source_binding_ack",rclcpp::QoS(1).reliable());
    position_sub_=create_subscription<px4_msgs::msg::VehicleLocalPosition>("/fmu/out/vehicle_local_position_v1",latest,
      [this](px4_msgs::msg::VehicleLocalPosition::SharedPtr p){invoke([&]{const auto at=received(0);position_=*p;evidence_.append(1,at,receipts_[0].sequence,clock_,*p);});});
    attitude_sub_=create_subscription<px4_msgs::msg::VehicleAttitude>("/fmu/out/vehicle_attitude",latest,
      [this](px4_msgs::msg::VehicleAttitude::SharedPtr p){invoke([&]{const auto at=received(1);attitude_=*p;evidence_.append(2,at,receipts_[1].sequence,clock_,*p);});});
    angular_sub_=create_subscription<px4_msgs::msg::VehicleAngularVelocity>("/fmu/out/vehicle_angular_velocity",latest,
      [this](px4_msgs::msg::VehicleAngularVelocity::SharedPtr p){invoke([&]{const auto at=received(2);angular_=*p;evidence_.append(3,at,receipts_[2].sequence,clock_,*p);});});
    flags_sub_=create_subscription<px4_msgs::msg::EstimatorStatusFlags>("/fmu/out/estimator_status_flags",latest,
      [this](px4_msgs::msg::EstimatorStatusFlags::SharedPtr p){invoke([&]{const auto at=received(3);flags_=*p;evidence_.append(4,at,receipts_[3].sequence,clock_,*p);});});
    clock_sub_=create_subscription<rosgraph_msgs::msg::Clock>("/clock",latest,[this](rosgraph_msgs::msg::Clock::SharedPtr p){invoke([&]{
      core::require(p->clock.sec>=0&&p->clock.nanosec<=999999999,"Invalid /clock fields");const double value=p->clock.sec+p->clock.nanosec*1e-9;
      core::require(!core::finite(clock_)||value>=clock_-1e-9,"/clock regressed");clock_=value;const auto at=received(4);evidence_.append(5,at,receipts_[4].sequence,clock_,*p);});});
    start_sub_=create_subscription<msg::LiveStart>("/flyarm/isolated/v1/live_start",rclcpp::QoS(1).reliable(),[this](msg::LiveStart::SharedPtr s){invoke([&]{start(*s);});});
    binding_sub_=create_subscription<msg::SourceBinding>("/flyarm/isolated/v1/source_binding",rclcpp::QoS(1).reliable(),
      [this](msg::SourceBinding::SharedPtr b){invoke([&]{bind(*b);});});
    stop_sub_=create_subscription<msg::RuntimeStop>("/flyarm/isolated/v1/stop",rclcpp::QoS(1).reliable(),[this](msg::RuntimeStop::SharedPtr s){invoke([&]{
      if(s->session_id!=session_||s->epoch!=epoch_)return;
      evidence_.append(8,monotonic_ns(),0,clock_,*s);stopped_=true;});});
    timer_=create_wall_timer(std::chrono::milliseconds(1),[this]{invoke([this]{poll();});});
  }
  bool done()const{return stopped_||!failure_.empty();}bool failed()const{return !failure_.empty();}
  void finish(){if(finished_)return;finished_=true;evidence_.write(output_/"source_wire.cdr");
    std::ofstream f(output_/"source_selections.csv");f<<"sequence,issued_ns,select_end_ns,publish_end_ns,control_s,joint_s,state_s,position_sequence,attitude_sequence,angular_sequence,flags_sequence,clock_sequence,command_count,packet_count,generation\n"<<std::setprecision(17);
    for(const auto& s:samples_)f<<s.sequence<<','<<s.issued<<','<<s.selected<<','<<s.published<<','<<s.control<<','<<s.joint<<','<<s.state<<','
      <<s.receipts[0]<<','<<s.receipts[1]<<','<<s.receipts[2]<<','<<s.receipts[3]<<','<<s.receipts[4]<<','<<s.commands<<','<<s.packets<<','<<s.generation<<'\n';
    f.close();core::require(bool(f),"Source selection evidence write failed");
    std::ofstream report(output_/"source_final.json");report<<std::setprecision(17)<<"{\n\"schema_version\":1,\n\"source_kind\":\"cpp_raw_px4_ts20_v1\",\n\"session_id\":"<<session_<<",\n\"epoch\":"<<epoch_
      <<",\n\"configuration_sha256\":\""<<hash_text_<<"\",\n\"ready\":"<<(ready_?"true":"false")<<",\n\"started\":"<<(started_?"true":"false")
      <<",\n\"bound\":"<<(bound_?"true":"false")<<",\n\"stopped\":"<<(stopped_?"true":"false")<<",\n\"snapshots\":"<<sequence_<<",\n\"wire_records\":"<<evidence_.records()<<",\n\"failure\":\""<<escaped(failure_)
      <<"\",\n\"last_readiness_error\":\""<<escaped(readiness_error_)<<"\",\n\"state_csv\":\""<<escaped(state_path_)<<"\",\n\"packet_csv\":\""<<escaped(packet_path_)
      <<"\",\n\"state_rows_read\":"<<states_->rows()<<",\n\"packet_rows_read\":"<<packets_->rows()<<",\n\"hardware_qualified\":false\n}\n";
    report.close();core::require(bool(report),"Source final evidence write failed");
  }
 private:
  template<class F>void invoke(F&& f){if(done())return;try{f();}catch(const std::exception& e){failure_=e.what();
    msg::RuntimeStop stop;stop.session_id=session_;stop.epoch=epoch_;stop_out_->publish(stop);}}
  std::uint64_t received(std::size_t i){const auto now=monotonic_ns();receipts_[i]={receipts_[i].sequence+1,now,clock_};return now;}
  core::EstimatedState estimate(double now,std::uint64_t wall){
    core::require(bool(guard_),"Raw source has no original-reference binding");
    return guard_->read(now,wall,position_plain(),quaternion(),omega_plain(),flags_plain(),receipts_);
  }
  void bind(const msg::SourceBinding& b){
    if(b.session_id!=session_||b.epoch!=epoch_)return;
    const auto now=monotonic_ns();msg::SourceBindingAck ack;ack.session_id=session_;ack.epoch=epoch_;ack.configuration_sha256=hash_;
    ack.binding_sequence=b.binding_sequence;ack.binding_issued_monotonic_ns=b.issued_monotonic_ns;ack.observed_monotonic_ns=now;
    // Retain the actual rejected handoff, not only a textual error summary.
    evidence_.append(9,now,b.binding_sequence,clock_,b);
    try{
      ack.rejection_code=1;
      core::require(ready_&&!bound_&&!started_&&b.schema_version==1&&b.binding_sequence==1&&b.configuration_sha256==hash_&&
        b.issued_monotonic_ns>0&&b.issued_monotonic_ns<=now&&now-b.issued_monotonic_ns<=25000000,"Invalid/stale/duplicate original source binding");
      core::require(b.px4_source_contract_commit=="d6f12ad1c4f70ad3230afd7d86e971421e02fef4"&&b.frame_mode=="raw_ev_world"&&
        b.target_frozen&&!b.fit_estimator_translation,"Binding requires frozen unfitted pinned raw-EV reference");
      core::require(b.position_gains==gains_&&b.bus_period_s==bus_period_&&b.delay_s==delay_,"Binding static gains/bus/delay differ from frozen startup");
      const core::Origin proposed{b.ref_lat,b.ref_lon,b.ref_alt,b.ref_timestamp,b.xy_global,b.z_global,b.xy_reset_counter,
        b.z_reset_counter,b.heading_reset_counter,b.vxy_reset_counter,b.vz_reset_counter,b.quat_reset_counter};
      ack.rejection_code=2;auto candidate=std::make_unique<core::EstimatorGuard>(proposed);
      candidate->read(clock_,now,position_plain(),quaternion(),omega_plain(),flags_plain(),receipts_);
      history_->select(clock_,0);
      origin_=proposed;guard_=std::move(candidate);bound_=true;ack.accepted=true;ack.rejection_code=0;
      evidence_.append(10,monotonic_ns(),b.binding_sequence,clock_,ack);binding_ack_->publish(ack);
    }catch(...){ack.accepted=false;if(!ack.rejection_code)ack.rejection_code=2;
      evidence_.append(10,monotonic_ns(),b.binding_sequence,clock_,ack);binding_ack_->publish(ack);throw;}
  }
  void start(const msg::LiveStart& s){if(s.session_id!=session_||s.epoch!=epoch_)return;
    const auto now=monotonic_ns();evidence_.append(6,now,0,clock_,s);
    core::require(!started_&&ready_&&bound_,"LiveStart before READY/binding or duplicate LiveStart");
    core::require(s.configuration_sha256==hash_&&s.issued_monotonic_ns>0&&s.issued_monotonic_ns<=now&&now-s.issued_monotonic_ns<=25000000,
      "LiveStart hash or 25 ms issue age invalid");
    core::require(core::finite(s.origin_s)&&s.origin_s>=0&&core::finite(s.ref_alt)&&std::abs(s.ref_alt-origin_.alt)<=1e-6&&
      core::all_finite(s.target_position_enu),"LiveStart origin/frame/target invalid");
    start_origin_=s.origin_s;last_published_clock_=s.origin_s;center_=s.target_position_enu;started_=true;
  }
  void poll(){const auto issued=monotonic_ns();
    // Single executor: raw callbacks cannot replace any part of this selection.
    const bool state_end=states_->poll([this](const core::CsvTail& c){history_->append(core::joint_row(c));});
    const bool packet_end=packets_->poll([this](const core::CsvTail& c){history_->append(core::packet_row(c));});
    if(!started_){
      if(core::finite(clock_))history_->trim_ready(clock_);
      if(!ready_&&state_end&&packet_end&&source_->get_subscription_count()>0&&binding_ack_->get_subscription_count()>0&&core::finite(clock_)){
        try{for(const auto& r:receipts_)core::require(r.sequence>0&&core::finite(r.sim_s),"Raw PX4 stream not received after /clock");
          history_->select(clock_,0);readiness_error_.clear();
        }catch(const std::exception& e){readiness_error_=e.what();return;}
        std::ofstream f(ready_path_);f<<"cpp_raw_px4_ts20_v1: raw streams/causal TS20/DDS ready; UNBOUND; awaiting original SourceBinding then LiveStart\n";f.close();
        core::require(bool(f),"Cannot persist source READY");ready_=true;
      }return;
    }
    core::require(state_end&&packet_end,"Active physical CSV source failed to catch up within bounded read");
    if(clock_<=last_published_clock_)return;
    core::require(samples_.size()<100000,"Source selection evidence overflow");
    const double task_time=clock_-start_origin_;const auto joints=history_->select(clock_,task_time);
    // Commit estimator guard only after the joint selection has succeeded.
    const auto state=estimate(clock_,issued);const auto target=trajectory_.sample(task_time,center_);
    msg::ControlSnapshot s;s.schema_version=1;s.session_id=session_;s.epoch=epoch_;s.sequence=sequence_+1;s.replay_only=false;s.configuration_sha256=hash_;
    s.issued_monotonic_ns=issued;s.control_time_s=clock_;s.task_time_s=task_time;s.dt_s=.02;s.state_time_s=state.state_time;s.joint_time_s=joints.row.sim;
    s.position_enu=state.p;s.velocity_enu=state.v;s.acceleration_enu=state.a;s.rotation_row_major=state.R;s.omega_body_flu=state.omega;
    s.q=joints.row.q_physical;s.dq=joints.row.dq;s.reference_r=joints.row.r;s.reference_time_s=joints.reference_time;s.bus_origin_s=joints.bus_origin;
    s.target_position_enu=target.p;s.target_velocity_enu=target.v;s.target_acceleration_enu=target.a;s.position_gains=gains_;
    s.command_count=joints.commands.size();s.packet_count=joints.packets.size();
    for(std::size_t i=0;i<joints.commands.size();++i){s.command_time_s[i]=joints.commands[i].when;
      for(std::size_t j=0;j<2;++j)s.command_velocity[2*i+j]=joints.commands[i].u[j];}
    for(std::size_t i=0;i<joints.packets.size();++i){const auto& p=joints.packets[i];s.packet_send_time_s[i]=p.send;
      for(std::size_t j=0;j<2;++j){s.packet_q_ref[2*i+j]=p.q[j];s.packet_dq_ref[2*i+j]=p.dq[j];s.packet_tau_ff[2*i+j]=p.tau[j];s.packet_kp[2*i+j]=p.kp[j];s.packet_kd[2*i+j]=p.kd[j];}}
    const auto selected=monotonic_ns();
    // Complete immutable wire evidence before publish; record time is selection
    // completion, not subscriber arrival.
    evidence_.append(7,selected,s.sequence,clock_,s);
    source_->publish(s);const auto published=monotonic_ns();
    Selection trace;trace.sequence=s.sequence;trace.issued=issued;trace.selected=selected;trace.published=published;trace.control=clock_;trace.joint=joints.row.sim;
    trace.state=state.state_time;trace.commands=s.command_count;trace.packets=s.packet_count;trace.generation=joints.generation;
    for(std::size_t i=0;i<5;++i)trace.receipts[i]=receipts_[i].sequence;
    samples_.push_back(trace);sequence_=s.sequence;last_published_clock_=clock_;
  }
  // Helpers also keep READY validation and ACTIVE validation on the same exact
  // core; READY uses a guard copy so it cannot commit ACTIVE guard history.
  core::Position position_plain()const{core::Position p;p.p={position_.x,position_.y,position_.z};p.v={position_.vx,position_.vy,position_.vz};p.a={position_.ax,position_.ay,position_.az};
    p.origin={position_.ref_lat,position_.ref_lon,position_.ref_alt,position_.ref_timestamp,position_.xy_global,position_.z_global,position_.xy_reset_counter,
      position_.z_reset_counter,position_.heading_reset_counter,position_.vxy_reset_counter,position_.vz_reset_counter,attitude_.quat_reset_counter};
    p.xy_valid=position_.xy_valid;p.z_valid=position_.z_valid;p.vxy_valid=position_.v_xy_valid;p.vz_valid=position_.v_z_valid;p.heading_good=position_.heading_good_for_control;p.dead_reckoning=position_.dead_reckoning;return p;}
  core::Q4 quaternion()const{return {attitude_.q[0],attitude_.q[1],attitude_.q[2],attitude_.q[3]};}
  core::V3 omega_plain()const{return {angular_.xyz[0],angular_.xyz[1],angular_.xyz[2]};}
  core::Flags flags_plain()const{return {flags_.cs_ev_pos,flags_.cs_ev_vel,flags_.cs_ev_hgt,flags_.cs_gnss_pos,flags_.cs_gnss_vel,flags_.cs_gps_hgt,
    flags_.cs_baro_hgt,flags_.cs_inertial_dead_reckoning,flags_.cs_wind_dead_reckoning};}
  struct Selection{std::uint64_t sequence{},issued{},selected{},published{},generation{};double control{},joint{},state{};
    std::array<std::uint64_t,5>receipts{};std::uint32_t commands{},packets{};};
  bool ready_{},bound_{},started_{},stopped_{},finished_{};std::string failure_,readiness_error_,ready_path_,hash_text_,state_path_,packet_path_;fs::path output_;
  std::uint64_t session_{},epoch_{},sequence_{};std::array<std::uint8_t,32>hash_{};core::Origin origin_;
  double clock_{std::numeric_limits<double>::quiet_NaN()},start_origin_{},last_published_clock_{},bus_period_{},delay_{};
  std::array<core::Receipt,5>receipts_{};core::V3 gains_{},center_{};core::Trajectory trajectory_;WireEvidence evidence_;std::vector<Selection>samples_;
  std::unique_ptr<core::EstimatorGuard>guard_;std::unique_ptr<core::JointHistory>history_;std::unique_ptr<core::CsvTail>states_,packets_;
  px4_msgs::msg::VehicleLocalPosition position_;px4_msgs::msg::VehicleAttitude attitude_;px4_msgs::msg::VehicleAngularVelocity angular_;px4_msgs::msg::EstimatorStatusFlags flags_;
  rclcpp::Publisher<msg::ControlSnapshot>::SharedPtr source_;rclcpp::Publisher<msg::RuntimeStop>::SharedPtr stop_out_;
  rclcpp::Publisher<msg::SourceBindingAck>::SharedPtr binding_ack_;
  rclcpp::Subscription<msg::SourceBinding>::SharedPtr binding_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr position_sub_;rclcpp::Subscription<px4_msgs::msg::VehicleAttitude>::SharedPtr attitude_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleAngularVelocity>::SharedPtr angular_sub_;rclcpp::Subscription<px4_msgs::msg::EstimatorStatusFlags>::SharedPtr flags_sub_;
  rclcpp::Subscription<rosgraph_msgs::msg::Clock>::SharedPtr clock_sub_;rclcpp::Subscription<msg::LiveStart>::SharedPtr start_sub_;rclcpp::Subscription<msg::RuntimeStop>::SharedPtr stop_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};
int main(int argc,char**argv){rclcpp::init(argc,argv);std::shared_ptr<StateSource>node;
  try{node=std::make_shared<StateSource>();rclcpp::executors::SingleThreadedExecutor executor;executor.add_node(node);
    while(rclcpp::ok()&&!node->done())executor.spin_once(std::chrono::milliseconds(20));
    const bool failed=node->failed();node->finish();rclcpp::shutdown();return failed?1:0;
  }catch(const std::exception& e){std::cerr<<e.what()<<'\n';if(node){try{node->finish();}catch(const std::exception& write){std::cerr<<write.what()<<'\n';}}
    rclcpp::shutdown();return 1;}
}
