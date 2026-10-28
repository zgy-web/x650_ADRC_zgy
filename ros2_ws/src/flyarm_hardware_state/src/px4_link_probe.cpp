// Bench-only state adapter and one diagnostic command. No flight setpoint,
// Offboard, arming, actuator publisher, file output, or automatic retries.
#include "flyarm_hardware_state/link_contract.hpp"
#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_command_ack.hpp>
#include <chrono>
#include <iomanip>
#include <iostream>
#include <map>
#include <vector>

namespace pm=px4_msgs::msg;
using namespace flyarm_hw;
double steady_now(){return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();}
template<std::size_t N>std::array<double,N> doubles(const std::array<float,N>&x){
  std::array<double,N> y{};std::copy(x.begin(),x.end(),y.begin());return y;
}
void num(double x){if(std::isfinite(x))std::cout<<x;else std::cout<<"null";}
template<std::size_t N>void arr(const std::array<double,N>&x){std::cout<<'[';for(std::size_t i=0;i<N;++i){if(i)std::cout<<',';num(x[i]);}std::cout<<']';}

class LinkProbe:public rclcpp::Node {
 public:
  LinkProbe():Node("flyarm_px4_hardware_link_probe"){
    static_assert(pm::VehicleCommand::VEHICLE_CMD_RUN_PREARM_CHECKS==PREARM_CHECK_COMMAND);
    static_assert(pm::VehicleStatus::ARMING_STATE_DISARMED==1);
    static_assert(pm::VehicleStatus::NAVIGATION_STATE_OFFBOARD==14);
    static_assert(pm::VehicleStatus::HIL_STATE_OFF==0);
    mode_=declare_parameter<std::string>("mode","shadow");
    duration_=declare_parameter<double>("duration_s",60.0);
    settle_=declare_parameter<double>("settle_s",15.0);
    source_component_=declare_parameter<int>("source_component",191);
    if(get_parameter("use_sim_time").as_bool()||(mode_!="shadow"&&mode_!="diagnostic_ack")||
       duration_<30||duration_>120||settle_<5||settle_>duration_-10||source_component_<191||source_component_>220)
      throw std::invalid_argument("Invalid bench parameters");
    const auto qos=rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();
    att_=create_subscription<pm::VehicleAttitude>(topic("vehicle_attitude",pm::VehicleAttitude::MESSAGE_VERSION),qos,
      [this](pm::VehicleAttitude::ConstSharedPtr m){
        reset("attitude",{m->quat_reset_counter});
        body_.attitude.update(m->timestamp,m->timestamp_sample,steady_now());body_.q_ned_frd=doubles(m->q);
      });
    pos_=create_subscription<pm::VehicleLocalPosition>(topic("vehicle_local_position",pm::VehicleLocalPosition::MESSAGE_VERSION),qos,
      [this](pm::VehicleLocalPosition::ConstSharedPtr m){
        reset("position",{m->xy_reset_counter,m->z_reset_counter,m->vxy_reset_counter,m->vz_reset_counter,m->heading_reset_counter});
        body_.position.update(m->timestamp,m->timestamp_sample,steady_now());
        body_.p_ned={m->x,m->y,m->z};body_.v_ned={m->vx,m->vy,m->vz};
        body_.xy_valid=m->xy_valid;body_.z_valid=m->z_valid;body_.vxy_valid=m->v_xy_valid;
        body_.vz_valid=m->v_z_valid;body_.heading_valid=m->heading_good_for_control;
      });
    odom_=create_subscription<pm::VehicleOdometry>(topic("vehicle_odometry",pm::VehicleOdometry::MESSAGE_VERSION),qos,
      [this](pm::VehicleOdometry::ConstSharedPtr m){
        reset("odometry",{m->reset_counter});
        body_.odometry.update(m->timestamp,m->timestamp_sample,steady_now());body_.omega_frd=doubles(m->angular_velocity);
      });
    status_=create_subscription<pm::VehicleStatus>(topic("vehicle_status",pm::VehicleStatus::MESSAGE_VERSION),qos,
      [this](pm::VehicleStatus::ConstSharedPtr m){
        body_.status.update(m->timestamp,m->timestamp,steady_now());
        body_.arming_state=m->arming_state;body_.nav_state=m->nav_state;body_.hil_state=m->hil_state;
        body_.armed_seen|=m->arming_state==pm::VehicleStatus::ARMING_STATE_ARMED;
        body_.offboard_seen|=m->nav_state==pm::VehicleStatus::NAVIGATION_STATE_OFFBOARD;
        body_.hil_seen|=m->hil_state!=pm::VehicleStatus::HIL_STATE_OFF;
        body_.failsafe=m->failsafe;body_.system=m->system_id;body_.component=m->component_id;
      });
    if(mode_=="diagnostic_ack"){
      // RELIABLE is compatible with the PX4 reliable reader. The response is best effort.
      command_=create_publisher<pm::VehicleCommand>("/fmu/in/vehicle_command",rclcpp::QoS(1).reliable().durability_volatile());
      ack_=create_subscription<pm::VehicleCommandAck>(topic("vehicle_command_ack",pm::VehicleCommandAck::MESSAGE_VERSION),qos,
        [this](pm::VehicleCommandAck::ConstSharedPtr m){
          const auto now=steady_now();
          if(transaction_.observe({m->timestamp,m->command,m->result,m->target_system,m->target_component,m->from_external},now)){
            std::cout<<"{\"event\":\"matched_diagnostic_ack\",\"command\":"<<m->command
              <<",\"result\":"<<unsigned(m->result)<<",\"source_timestamp_us\":"<<m->timestamp
              <<",\"rtt_ms\":"<<transaction_.rtt_ms()<<"}\n"<<std::flush;
          }else ++ignored_acks_;
        });
    }
    start_=steady_now();last_report_=start_;
    timer_=create_wall_timer(std::chrono::milliseconds(20),[this](){tick();});
  }
  bool passed()const{return complete_&&observed_state_&&!unsafe_&&!transport_fault_&&ticks_>0&&
    (mode_=="shadow"||transaction_.accepted());}
 private:
  void reset(const std::string &key,std::vector<unsigned> values){
    const auto found=resets_.find(key);
    if(found!=resets_.end()&&found->second!=values)body_.reset_seen=true;
    resets_[key]=std::move(values);
  }
  void tick(){
    const double now=steady_now(),elapsed=now-start_;
    const auto b=adapt_body(body_,now);
    unsafe_|=body_.armed_seen||body_.offboard_seen||body_.hil_seen;
    if(elapsed>=settle_){
      ++ticks_;++reasons_[reason_name(b.reason)];
      const bool transport_ok=body_.attitude.fresh(now,.1)&&body_.position.fresh(now,.1)&&body_.odometry.fresh(now,.1)&&body_.status.fresh(now,1);
      observed_state_|=transport_ok;transport_fault_|=!transport_ok;
      if(b.ready()){
        // An encoding candidate only; it is deliberately never published.
        candidate_=shadow_hold(b,static_cast<std::uint64_t>(get_clock()->now().nanoseconds()/1000));
        ++hold_candidates_;
      }
      if(command_&&!transaction_.sent()&&!unsafe_&&transport_ok&&elapsed<duration_-5&&bench_guard(body_,now)==StateReason::ready&&
         body_.attitude.fresh(now,.1)&&finite(enu_flu(body_.q_ned_frd))&&command_->get_subscription_count()>0){
        pm::VehicleCommand m{};
        m.timestamp=static_cast<std::uint64_t>(get_clock()->now().nanoseconds()/1000);
        m.command=PREARM_CHECK_COMMAND;m.target_system=body_.system;m.target_component=body_.component;
        m.source_system=PROBE_SOURCE_SYSTEM;m.source_component=source_component_;m.from_external=true;
        transaction_.begin({m.timestamp,m.command,m.target_system,m.target_component,m.source_system,m.source_component},now);
        command_->publish(m);
        std::cout<<"{\"event\":\"diagnostic_command_sent\",\"command\":"<<m.command
          <<",\"timestamp_us\":"<<m.timestamp<<",\"target_system\":"<<unsigned(m.target_system)
          <<",\"target_component\":"<<unsigned(m.target_component)<<",\"source_component\":"<<m.source_component<<"}\n"<<std::flush;
      }
    }
    const bool final=elapsed>=duration_||unsafe_||transaction_.expired(now);
    if(final||now-last_report_>=1){
      complete_=final&&elapsed>=duration_;report(now,b,final);last_report_=now;
    }
    if(final)rclcpp::shutdown();
  }
  void report(double now,const BodySnapshot &b,bool final){
    std::cout<<std::boolalpha<<std::setprecision(12)<<"{\"schema\":\"flyarm.hardware_link.v1\",\"mode\":\""<<mode_
      <<"\",\"final\":"<<final<<",\"complete\":"<<complete_<<",\"elapsed_s\":"<<now-start_
      <<",\"observed_state\":"<<observed_state_<<",\"bench_checks_pass\":"<<passed()
      <<",\"body_reason\":\""<<reason_name(b.reason)<<"\",\"body_ready\":"<<b.ready()
      <<",\"position_enu_m\":";arr(b.p_enu);std::cout<<",\"velocity_enu_m_s\":";arr(b.v_enu);
    std::cout<<",\"q_enu_flu_wxyz\":";arr(b.q_enu_flu);std::cout<<",\"omega_flu_rad_s\":";arr(b.omega_flu);
    std::cout<<",\"ticks\":"<<ticks_<<",\"shadow_hold_candidates\":"<<hold_candidates_
      <<",\"flight_setpoints_sent\":0,\"offboard_requests_sent\":0,\"arm_requests_sent\":0"
      <<",\"mpc_executed\":false,\"joint_feedback_present\":false,\"flight_ready\":false"
      <<",\"diagnostic_commands_sent\":"<<(transaction_.sent()?1:0)<<",\"ack_received\":"<<transaction_.received()
      <<",\"ack_accepted\":"<<transaction_.accepted()<<",\"ack_result\":"<<transaction_.result()
      <<",\"ack_rtt_ms\":";if(transaction_.received())num(transaction_.rtt_ms());else std::cout<<"null";
    std::cout<<",\"ignored_acks\":"<<ignored_acks_<<",\"armed_seen\":"<<body_.armed_seen
      <<",\"offboard_seen\":"<<body_.offboard_seen<<",\"hil_seen\":"<<body_.hil_seen
      <<",\"state_reset_seen\":"<<body_.reset_seen<<",\"transport_fault\":"<<transport_fault_<<",\"reasons\":{";
    bool first=true;for(const auto &kv:reasons_){if(!first)std::cout<<',';first=false;std::cout<<'"'<<kv.first<<"\":"<<kv.second;}
    std::cout<<"}}\n"<<std::flush;
  }
  BodyInput body_;PvaCommand candidate_;AckTransaction transaction_;
  std::map<std::string,std::vector<unsigned>> resets_;
  std::map<std::string,std::uint64_t> reasons_;
  std::string mode_;double duration_{},settle_{},start_{},last_report_{};int source_component_{};
  bool complete_{},observed_state_{},unsafe_{},transport_fault_{};
  std::uint64_t ticks_{},hold_candidates_{},ignored_acks_{};
  rclcpp::Subscription<pm::VehicleAttitude>::SharedPtr att_;
  rclcpp::Subscription<pm::VehicleLocalPosition>::SharedPtr pos_;
  rclcpp::Subscription<pm::VehicleOdometry>::SharedPtr odom_;
  rclcpp::Subscription<pm::VehicleStatus>::SharedPtr status_;
  rclcpp::Subscription<pm::VehicleCommandAck>::SharedPtr ack_;
  rclcpp::Publisher<pm::VehicleCommand>::SharedPtr command_;
  rclcpp::TimerBase::SharedPtr timer_;
};
int main(int argc,char **argv){
  rclcpp::init(argc,argv);
  try{auto n=std::make_shared<LinkProbe>();rclcpp::spin(n);const bool pass=n->passed();if(rclcpp::ok())rclcpp::shutdown();return pass?0:2;}
  catch(const std::exception &e){std::cerr<<"hardware_link_error: "<<e.what()<<'\n';if(rclcpp::ok())rclcpp::shutdown();return 3;}
}
