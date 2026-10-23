#include "flyarm_control_gateway/runtime_wire.hpp"
#include "flyarm_control_gateway/consumption_csv.hpp"
#include <flyarm_control_interfaces/msg/actuator_state.hpp>
#include <fstream>
#include <filesystem>

class ConsumptionSource final: public rclcpp::Node {
 public:
  ConsumptionSource():Node("isolated_actual_consumption_source") {
    if(!declare_parameter<bool>("simulation_live",false))throw std::invalid_argument("Explicit simulation required");
    session_=declare_parameter<std::int64_t>("expected_session_id",0);
    epoch_=declare_parameter<std::int64_t>("expected_epoch",0);
    auto path=declare_parameter<std::string>("state_csv","");
    ready_=declare_parameter<std::string>("ready_file","");
    if(!session_ || !epoch_ || !std::filesystem::path(path).is_absolute() || !std::filesystem::is_regular_file(path))
      throw std::invalid_argument("Existing actual plugin CSV and explicit session required");
    input_.open(path,std::ios::binary);
    if(!input_)throw std::runtime_error("Cannot read actual plugin CSV");
    pub_=create_publisher<flyarm_control_interfaces::msg::ActuatorState>("/flyarm/isolated/v1/actuator_state",rclcpp::QoS(256).best_effort());
    timer_=create_wall_timer(std::chrono::milliseconds(1),[this]{poll();});
  }
 private:
  void poll() {
    auto now=flyarm_runtime_wire::monotonic_ns();
    // A stalled reader cannot relabel its accumulated old records as fresh.
    if(ready_written_ && now-last_poll_>25'000'000)throw std::runtime_error("Consumption reader polling gap exceeds 25 ms");
    last_poll_=now;
    input_.clear();input_.read(buffer_.data(),buffer_.size());
    auto count=input_.gcount();
    auto row=parser_.append(std::string(buffer_.data(),static_cast<std::size_t>(count)));
    // Drain the historical preflight file before advertising readiness. A full
    // buffer may still be historical; it must not produce a fresh receipt.
    if(count==static_cast<std::streamsize>(buffer_.size()))return;
    if(row) {
      flyarm_control_interfaces::msg::ActuatorState s;s.session_id=session_;s.epoch=epoch_;
      s.observed_monotonic_ns=flyarm_runtime_wire::monotonic_ns();s.sim_s=row->sim;
      s.consumed_counter=row->counter;s.consumed_velocity=row->velocity;s.consumed_sim_s=row->consumed;
      pub_->publish(s);
      if(!ready_written_ && !ready_.empty() && pub_->get_subscription_count()>=1) {
        std::ofstream f(ready_);f<<"actual complete plugin CSV; DDS matched\n";f.close();ready_written_=true;
      }
    }
  }
  std::uint64_t session_{},epoch_{},last_poll_{};std::string ready_;bool ready_written_{};
  std::ifstream input_;std::array<char,1024*1024> buffer_{};flyarm_control_gateway::ConsumptionCsv parser_;
  rclcpp::Publisher<flyarm_control_interfaces::msg::ActuatorState>::SharedPtr pub_;rclcpp::TimerBase::SharedPtr timer_;
};
int main(int argc,char **argv) {
  rclcpp::init(argc,argv);
  try {rclcpp::spin(std::make_shared<ConsumptionSource>());rclcpp::shutdown();return 0;}
  catch(const std::exception &e) {std::cerr<<e.what()<<'\n';rclcpp::shutdown();return 1;}
}
