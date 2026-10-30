// Attitude-only ownership/exit probe. ESC POWER MUST REMAIN DISCONNECTED.
// Read real PX4 feedback; publish OCM + fixed measured attitude/zero thrust.
// No arming, mode request, position injection or actuator override exists here.
#include "flyarm_hardware_state/offboard_bench_contract.hpp"
#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <px4_msgs/msg/manual_control_setpoint.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/vehicle_attitude_setpoint.hpp>
#include <chrono>
#include <iostream>
#include <poll.h>
#include <unistd.h>

namespace pm=px4_msgs::msg;
double bench_now(){return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();}
// A duplicate does not renew freshness. A regression latches a permanent fault.
void receive_stamp(flyarm_hw::Stats& s,std::uint64_t stamp){
  if(s.count&&stamp==s.stamp){++s.repeats;return;}
  s.add(bench_now(),stamp);
}
class Bench:public rclcpp::Node {
 public:
  Bench():Node("flyarm_offboard_bench") {
    if(!declare_parameter<bool>("approved_power_disconnected_bench",false))throw std::runtime_error("Bench approval required");
    scenario_=declare_parameter<std::string>("scenario","manual_takeover");
    if(scenario_!="manual_takeover"&&scenario_!="stream_loss")throw std::runtime_error("Unknown scenario");
    created_=bench_now();
    auto qos=rclcpp::QoS(1).best_effort().durability_volatile();
    status_=create_subscription<pm::VehicleStatus>(flyarm_hw::topic("vehicle_status",pm::VehicleStatus::MESSAGE_VERSION),qos,
      [this](pm::VehicleStatus::ConstSharedPtr m){receive_stamp(s_.status,m->timestamp);s_.arm=m->arming_state;s_.nav=m->nav_state;
        s_.hil=m->hil_state;s_.system=m->system_id;s_.component=m->component_id;s_.failsafe=m->failsafe;});
    attitude_=create_subscription<pm::VehicleAttitude>(flyarm_hw::topic("vehicle_attitude",pm::VehicleAttitude::MESSAGE_VERSION),qos,
      [this](pm::VehicleAttitude::ConstSharedPtr m){receive_stamp(s_.attitude,m->timestamp);std::copy(m->q.begin(),m->q.end(),s_.q.begin());s_.reset=m->quat_reset_counter;});
    manual_=create_subscription<pm::ManualControlSetpoint>(flyarm_hw::topic("manual_control_setpoint",pm::ManualControlSetpoint::MESSAGE_VERSION),qos,
      [this](pm::ManualControlSetpoint::ConstSharedPtr m){receive_stamp(s_.manual,m->timestamp);s_.rc_valid=m->valid&&m->data_source==1;
        s_.throttle=m->throttle;s_.roll=m->roll;s_.pitch=m->pitch;s_.yaw=m->yaw;});
    ocm_=create_publisher<pm::OffboardControlMode>("/fmu/in/offboard_control_mode",rclcpp::QoS(1).reliable().durability_volatile());
    constexpr auto version=pm::VehicleAttitudeSetpoint::MESSAGE_VERSION;
    att_=create_publisher<pm::VehicleAttitudeSetpoint>(std::string("/fmu/in/vehicle_attitude_setpoint")+(version?"_v"+std::to_string(version):""),rclcpp::QoS(1).reliable().durability_volatile());
    timer_=create_wall_timer(std::chrono::milliseconds(20),[this]{tick();});
    std::cout<<std::boolalpha<<"{\"event\":\"bench_started\",\"zero_thrust\":true,\"maximum_stream_s\":60,\"arm_requests\":0,\"mode_requests\":0}\n"<<std::flush;
  }
 private:
  void finish(const char* reason){
    if(done_)return;done_=true;
    std::cout<<"{\"event\":\"bench_final\",\"reason\":\""<<reason<<"\",\"sent\":"<<sent_
      <<",\"offboard_seen\":"<<gate_.offboard_seen<<",\"nav\":"<<s_.nav<<",\"armed\":"<<(s_.arm==2)
      <<",\"arm_requests\":0,\"mode_requests\":0,\"physical_stop_verified\":false}\n"<<std::flush;rclcpp::shutdown();
  }
  void tick(){
    const double now=bench_now();if(done_)return;
    if(now-created_>=90){finish("process_deadline");return;}
    pollfd fd{STDIN_FILENO,POLLIN,0};const int ready=::poll(&fd,1,0);
    if(ready<0){finish("control_pipe_error");return;}
    if(ready>0&&(fd.revents&(POLLIN|POLLHUP))){
      char bytes[16];const auto n=::read(STDIN_FILENO,bytes,sizeof(bytes));
      if(n<=0){finish("control_pipe_closed");return;}
      input_.append(bytes,std::size_t(n));
      if(input_.size()>6){finish("invalid_start");return;}
      if(input_.find('\n')!=std::string::npos){
        if(input_!="start\n"||started_||!readers()||!gate_.begin(s_,now)){finish("start_rejected");return;}
        started_=true;input_.clear();std::cout<<"{\"event\":\"stream_started\"}\n"<<std::flush;
      }
    }
    if(!started_){
      if(!ready_reported_&&readers()&&flyarm_hw::OffboardBenchContract::healthy(s_,now)&&s_.arm==1&&s_.nav==1){
        ready_reported_=true;std::cout<<"{\"event\":\"ready\"}\n"<<std::flush;}
      if(now-created_>=30)finish("start_timeout");return;
    }
    if(retired_){
      if(now-last_print_>=.2)print_state(now);
      if(now-retired_at_>=6)finish(gate_.reason);
      return;
    }
    if(!readers())gate_.retire("dds_reader_lost");
    if(!gate_.tick(s_,now,scenario_=="stream_loss"?3.:0.)){
      retired_=true;retired_at_=now;
      std::cout<<"{\"event\":\"stream_stopped\",\"reason\":\""<<gate_.reason<<"\",\"sent\":"<<sent_<<"}\n"<<std::flush;return;
    }
    const auto stamp=std::uint64_t(get_clock()->now().nanoseconds()/1000);
    if(!stamp||stamp<=last_stamp_){gate_.retire("publication_clock_regressed");retired_=true;retired_at_=now;return;}
    last_stamp_=stamp;
    pm::VehicleAttitudeSetpoint a{};a.timestamp=stamp;a.yaw_sp_move_rate=0;a.thrust_body={0,0,0};
    for(unsigned i=0;i<4;++i)a.q_d[i]=static_cast<float>(gate_.target[i]);
    pm::OffboardControlMode o{};o.timestamp=stamp;o.attitude=true;
    att_->publish(a);ocm_->publish(o);++sent_;
    if(now-last_print_>=.2)print_state(now);
  }
  bool readers()const{return att_->get_subscription_count()>0&&ocm_->get_subscription_count()>0;}
  void print_state(double now){last_print_=now;
    std::cout<<"{\"event\":\"bench_state\",\"elapsed\":"<<now-created_<<",\"nav\":"<<s_.nav
      <<",\"armed\":"<<(s_.arm==2)<<",\"failsafe\":"<<s_.failsafe<<",\"sent\":"<<sent_
      <<",\"retired\":"<<retired_<<",\"status_stamp\":"<<s_.status.stamp<<"}\n"<<std::flush;}
  flyarm_hw::BenchState s_;flyarm_hw::OffboardBenchContract gate_;std::string input_,scenario_;
  bool started_{},retired_{},ready_reported_{},done_{};double created_{},retired_at_{},last_print_{};
  unsigned sent_{};std::uint64_t last_stamp_{};
  rclcpp::Subscription<pm::VehicleStatus>::SharedPtr status_;
  rclcpp::Subscription<pm::VehicleAttitude>::SharedPtr attitude_;
  rclcpp::Subscription<pm::ManualControlSetpoint>::SharedPtr manual_;
  rclcpp::Publisher<pm::OffboardControlMode>::SharedPtr ocm_;
  rclcpp::Publisher<pm::VehicleAttitudeSetpoint>::SharedPtr att_;
  rclcpp::TimerBase::SharedPtr timer_;
};
int main(int argc,char**argv){rclcpp::init(argc,argv);try{auto n=std::make_shared<Bench>();rclcpp::spin(n);return 0;}
catch(const std::exception&e){std::cerr<<e.what()<<'\n';if(rclcpp::ok())rclcpp::shutdown();return 2;}}
