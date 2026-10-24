#include "flyarm_control_gateway/runtime_wire.hpp"
#include "flyarm_control_gateway/consumption_csv.hpp"
#include "flyarm_control_gateway/consumption_lifecycle.hpp"
#include <flyarm_control_interfaces/msg/actuator_state.hpp>
#include <flyarm_control_interfaces/msg/live_start.hpp>
#include <fstream>
#include <filesystem>

class ConsumptionSource final: public rclcpp::Node {
 public:
  ConsumptionSource():Node("isolated_actual_consumption_source") {
    if(!declare_parameter<bool>("simulation_live",false))throw std::invalid_argument("Explicit simulation required");
    session_=declare_parameter<std::int64_t>("expected_session_id",0);
    epoch_=declare_parameter<std::int64_t>("expected_epoch",0);
    hash_=flyarm_runtime_wire::parse_hash(declare_parameter<std::string>("configuration_sha256",""));
    auto path=declare_parameter<std::string>("state_csv","");
    ready_=declare_parameter<std::string>("ready_file","");
    if(!session_ || !epoch_ || !std::filesystem::path(path).is_absolute() || !std::filesystem::is_regular_file(path))
      throw std::invalid_argument("Existing actual plugin CSV and explicit session required");
    input_.open(path,std::ios::binary);
    if(!input_)throw std::runtime_error("Cannot read actual plugin CSV");
    pub_=create_publisher<flyarm_control_interfaces::msg::ActuatorState>("/flyarm/isolated/v1/actuator_state",rclcpp::QoS(256).best_effort());
    start_=create_subscription<flyarm_control_interfaces::msg::LiveStart>(
      "/flyarm/isolated/v1/live_start",rclcpp::QoS(1).reliable(),[this](flyarm_control_interfaces::msg::LiveStart::SharedPtr s) {
        if(s->session_id!=session_ || s->epoch!=epoch_)return;
        if(s->configuration_sha256!=hash_ || !std::isfinite(s->origin_s) || s->origin_s<0)
          throw std::runtime_error("Consumption LiveStart identity/origin invalid");
        lifecycle_.start(s->issued_monotonic_ns,flyarm_runtime_wire::monotonic_ns());
        origin_=s->origin_s;
      });
    timer_=create_wall_timer(std::chrono::milliseconds(1),[this]{poll();});
  }
 private:
  void poll() {
    auto now=flyarm_runtime_wire::monotonic_ns();
    lifecycle_.begin_poll(now);
    input_.clear();input_.read(buffer_.data(),buffer_.size());
    auto count=input_.gcount();
    auto row=parser_.append(std::string(buffer_.data(),static_cast<std::size_t>(count)));
    const bool caught_up=count<static_cast<std::streamsize>(buffer_.size());
    const bool publish=lifecycle_.finish_poll(flyarm_runtime_wire::monotonic_ns(),caught_up,bool(row));
    if(publish) {
      if(row->sim<=origin_)throw std::runtime_error("Consumption new row does not follow actual LiveStart origin");
      flyarm_control_interfaces::msg::ActuatorState s;s.session_id=session_;s.epoch=epoch_;
      // Preserve the beginning of this bounded read, not a later timestamp
      // that would erase parser time from the recipient's freshness check.
      s.observed_monotonic_ns=now;s.sim_s=row->sim;
      s.consumed_counter=row->counter;s.consumed_velocity=row->velocity;s.consumed_sim_s=row->consumed;
      pub_->publish(s);
    }
    // Discovery and readiness-file I/O run only before the live epoch. They
    // must not be charged to, or executed inside, an active polling interval.
    if(lifecycle_.phase()==flyarm_control_gateway::ConsumptionLifecycle::STARTING &&
       caught_up && row && pub_->get_subscription_count()>=1) {
      if(!ready_.empty()) {
        std::ofstream f(ready_);f<<"actual complete plugin CSV; DDS matched; awaiting LiveStart and new-row barrier\n";f.close();
        if(!f)throw std::runtime_error("Cannot persist consumption readiness");
      }
      lifecycle_.ready();
    }
  }
  std::uint64_t session_{},epoch_{};std::string ready_;double origin_{};flyarm_control_gateway::Hash hash_{};
  flyarm_control_gateway::ConsumptionLifecycle lifecycle_;
  std::ifstream input_;std::array<char,1024*1024> buffer_{};flyarm_control_gateway::ConsumptionCsv parser_;
  rclcpp::Publisher<flyarm_control_interfaces::msg::ActuatorState>::SharedPtr pub_;rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<flyarm_control_interfaces::msg::LiveStart>::SharedPtr start_;
};
int main(int argc,char **argv) {
  rclcpp::init(argc,argv);
  try {rclcpp::spin(std::make_shared<ConsumptionSource>());rclcpp::shutdown();return 0;}
  catch(const std::exception &e) {std::cerr<<e.what()<<'\n';rclcpp::shutdown();return 1;}
}
