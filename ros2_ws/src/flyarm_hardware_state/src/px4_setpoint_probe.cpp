// Disarmed bench payload probe: only TrajectorySetpoint is published.
// No OffboardControlMode, VehicleCommand, actuator, position injection or files.
#include "flyarm_hardware_state/px4_message_adapter.hpp"
#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <iostream>
#include <iomanip>
#include <poll.h>
#include <unistd.h>

namespace pm=px4_msgs::msg;
double steady_s(){return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();}
template<class A>void array_json(const A& a){std::cout<<'[';bool first=true;for(auto x:a){if(!first)std::cout<<',';first=false;if(std::isfinite(x))std::cout<<x;else std::cout<<"null";}std::cout<<']';}

class SetpointProbe:public rclcpp::Node {
 public:
  SetpointProbe():Node("flyarm_disarmed_setpoint_probe") {
    if(!declare_parameter<bool>("approved_bench_probe",false))throw std::invalid_argument("Explicit bench probe flag required");
    budget_=declare_parameter<double>("startup_budget_s",30.);
    if(!std::isfinite(budget_)||budget_<2||budget_>30)throw std::invalid_argument("Startup budget outside [2,30]");
    const auto version=pm::VehicleStatus::MESSAGE_VERSION;
    status_=create_subscription<pm::VehicleStatus>("/fmu/out/vehicle_status"+(version?"_v"+std::to_string(version):""),
      rclcpp::QoS(1).best_effort().durability_volatile(),[this](pm::VehicleStatus::ConstSharedPtr m){
        if(m->timestamp>receiver_.body.status.stamp)++fresh_count_;
        receiver_.receive(*m,steady_s());
      });
    pub_=create_publisher<pm::TrajectorySetpoint>("/fmu/in/trajectory_setpoint",rclcpp::QoS(1).reliable().durability_volatile());
    created_=steady_s();
    timer_=create_wall_timer(std::chrono::milliseconds(10),[this]{tick();});
    std::cout<<std::setprecision(12)<<std::boolalpha;
    std::cout<<"{\"event\":\"probe_started\",\"send_window_s\":10,\"maximum_count\":200,\"offboard_publishers\":0,\"vehicle_command_publishers\":0,\"actuator_publishers\":0}\n"<<std::flush;
  }
  bool passed()const{return complete_;}
 private:
  void finish(bool pass,const char* reason){
    if(done_)return;
    done_=true;complete_=pass;
    const auto& b=receiver_.body;
    std::cout<<"{\"event\":\"probe_final\",\"complete\":"<<pass<<",\"reason\":\""<<reason
      <<"\",\"sent\":"<<sent_<<",\"pattern_counts\":";array_json(counts_);
    std::cout<<",\"fresh_status_count\":"<<fresh_count_<<",\"armed_seen\":"<<b.armed_seen
      <<",\"offboard_seen\":"<<b.offboard_seen<<",\"hil_seen\":"<<b.hil_seen
      <<",\"status_fault\":"<<b.status.fault<<",\"maximum_status_receipt_age_s\":"<<max_age_
      <<",\"arm_requests_sent\":0,\"offboard_requests_sent\":0,\"actuator_commands_sent\":0}\n"<<std::flush;
    rclcpp::shutdown();
  }
  void tick(){
    if(done_)return;
    const double now=steady_s();const auto& b=receiver_.body;
    const auto guard=flyarm_hw::bench_guard(b,now);
    if(b.armed_seen||b.offboard_seen||b.hil_seen||b.status.fault){finish(false,"unsafe_or_regressed_status");return;}
    const bool healthy=guard==flyarm_hw::StateReason::ready&&fresh_count_>=3&&pub_->get_subscription_count()>0;
    if(started_&&!healthy){finish(false,"status_or_reader_lost");return;}
    if(healthy)max_age_=std::max(max_age_,now-b.status.receipt);
    if(!started_&&now-created_>budget_){finish(false,"start_timeout");return;}
    if(!ready_&&healthy){ready_=true;std::cout<<"{\"event\":\"ready\"}\n"<<std::flush;}
    pollfd fd{STDIN_FILENO,POLLIN,0};const int polled=::poll(&fd,1,0);
    if(polled<0){finish(false,"stdin_poll_failed");return;}
    if(polled>0&&(fd.revents&(POLLIN|POLLHUP))){
      char bytes[64];const auto n=::read(STDIN_FILENO,bytes,sizeof(bytes));
      if(n<=0){finish(false,"control_pipe_closed");return;}
      input_.append(bytes,std::size_t(n));
      if(input_.size()>6){finish(false,"invalid_start_message");return;}
      if(input_.find('\n')!=std::string::npos){
        if(input_!="start\n"||started_||!healthy){finish(false,"invalid_start_message");return;}
        input_.clear();started_=true;start_=now;next_=now;
        std::cout<<"{\"event\":\"sending_started\"}\n"<<std::flush;
      }
    }
    if(!started_)return;
    const double elapsed=now-start_;
    if(elapsed>=13){finish(sent_>=100&&counts_[0]>=10&&counts_[1]>=10&&counts_[2]>=10,"window_finished");return;}
    if(elapsed>=10||sent_>=200||now<next_)return;
    // Do not catch up with a burst after a scheduling pause.
    next_=now+.05;const int pattern=elapsed<3?0:(elapsed<6?1:2);
    const double scale=pattern==0?1.:(pattern==1?-1.:.5);
    const auto stamp=std::uint64_t(get_clock()->now().nanoseconds()/1000);
    if(!stamp||stamp<=last_stamp_){finish(false,"publication_clock_regressed");return;}
    last_stamp_=stamp;
    const auto m=flyarm_hw::trajectory_message(flyarm_hw::encode_enu_pva(stamp,
      {.125*scale,-.25*scale,.5*scale},{.03125*scale,-.0625*scale,.015625*scale},
      {.0625*scale,.125*scale,-.03125*scale},0));
    pub_->publish(m);++sent_;++counts_[pattern];
    std::cout<<"{\"event\":\"setpoint_sent\",\"seq\":"<<sent_<<",\"pattern\":"<<pattern
      <<",\"elapsed_s\":"<<elapsed<<",\"timestamp\":"<<stamp<<",\"position\":";array_json(m.position);
    std::cout<<",\"velocity\":";array_json(m.velocity);std::cout<<",\"acceleration\":";array_json(m.acceleration);
    std::cout<<",\"yaw\":"<<m.yaw<<",\"jerk\":[null,null,null],\"yawspeed\":null,\"null_represents_nan\":true}\n"<<std::flush;
  }
  flyarm_hw::Px4BodyReceiver receiver_;
  bool ready_{},started_{},done_{},complete_{};
  double budget_{},created_{},start_{},next_{},max_age_{};
  unsigned sent_{},fresh_count_{};std::uint64_t last_stamp_{};std::array<unsigned,3> counts_{};std::string input_;
  rclcpp::Subscription<pm::VehicleStatus>::SharedPtr status_;
  rclcpp::Publisher<pm::TrajectorySetpoint>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};
int main(int argc,char** argv){
  rclcpp::init(argc,argv);
  try{auto n=std::make_shared<SetpointProbe>();rclcpp::spin(n);const bool ok=n->passed();if(rclcpp::ok())rclcpp::shutdown();return ok?0:2;}
  catch(const std::exception& e){std::cerr<<"setpoint_probe_error: "<<e.what()<<'\n';if(rclcpp::ok())rclcpp::shutdown();return 3;}
}
