#include "x650_mpc/px4_replay_adapter.hpp"
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <iostream>
#include <functional>

namespace pm=px4_msgs::msg;
void check(bool ok,const char* text){if(!ok)throw std::runtime_error(text);}
void near(double a,double b){check(std::abs(a-b)<1e-9,"golden numeric mismatch");}
void rejects(const std::function<void()>& fn) {
  bool caught=false;try{fn();}catch(const std::invalid_argument&){caught=true;}
  check(caught,"Invalid input was accepted");
}
sensor_msgs::msg::JointState joints() {
  sensor_msgs::msg::JointState j;j.header.stamp.sec=1;j.header.stamp.nanosec=4'000'000;
  j.name={"elbow","shoulder"};j.position={.25,-.125};j.velocity={-.5,.75};return j;
}
x650_mpc::Px4ReplayAdapter healthy() {
  x650_mpc::Px4ReplayAdapter a({"shoulder","elbow"});
  pm::VehicleAttitude att;att.timestamp=att.timestamp_sample=1'000'000;att.q={1,0,0,0};
  pm::VehicleLocalPosition p;p.timestamp=p.timestamp_sample=1'000'000;
  p.x=2;p.y=3;p.z=-4;p.vx=1;p.vy=2;p.vz=3;p.ax=4;p.ay=5;p.az=6;p.ref_alt=123.5;
  p.xy_valid=p.z_valid=p.v_xy_valid=p.v_z_valid=p.heading_good_for_control=true;
  pm::VehicleOdometry o;o.timestamp=o.timestamp_sample=1'000'000;o.angular_velocity={1,2,3};
  pm::VehicleStatus s;s.timestamp=1'000'000;s.system_id=s.component_id=1;s.arming_state=1;
  a.receiver.receive(att,10);a.receiver.receive(p,10);a.receiver.receive(o,10);a.receiver.receive(s,10);
  a.receive_joints(joints(),10);return a;
}
int main() {
  static_assert(pm::TrajectorySetpoint::MESSAGE_VERSION==0);
  static_assert(pm::VehicleLocalPosition::MESSAGE_VERSION==1);
  auto a=healthy();const auto s=a.state(1.008,10.001);
  check(s.position==x650::V3{3,2,4},"position origin/sign");
  check(s.velocity==x650::V3{2,1,-3},"velocity axes");
  check(s.acceleration==x650::V3{5,4,-6},"world acceleration must not gain gravity");
  check(s.omega_body==x650::V3{1,-2,-3},"body angular velocity");
  check(s.q==x650::V2{-.125,.25}&&s.dq==x650::V2{.75,-.5},"joint names were ignored");
  const x650::R9 expected{0,-1,0,1,0,0,0,0,1};for(int i=0;i<9;++i)near(s.rotation[i],expected[i]);
  near(s.stamp,1);near(s.joint_stamp,1.004);near(s.control_time,1.008);
  x650::StepResult result;result.position_reference={1,2,3};result.velocity_feedforward={4,5,6};
  result.acceleration_feedforward={7,8,9};result.command={10,20,30,.1,.2};
  auto m=x650_mpc::encode_mpc_output(result,1'008'000,0);
  check(m.position==std::array<float,3>{2,1,-3},"origin offset added");
  check(m.velocity==std::array<float,3>{5,4,-6},"velocity added twice");
  check(m.acceleration==std::array<float,3>{8,7,-9},"acceleration encoding");
  check(m.yaw==float(std::acos(-1.)/2)&&std::isnan(m.yawspeed),"yaw units/unused yaw rate");
  rclcpp::Serialization<pm::TrajectorySetpoint> codec;rclcpp::SerializedMessage bytes;pm::TrajectorySetpoint back;
  codec.serialize_message(&m,&bytes);codec.deserialize_message(&bytes,&back);
  check(m.timestamp==back.timestamp&&m.position==back.position&&m.velocity==back.velocity&&m.acceleration==back.acceleration&&m.yaw==back.yaw,"CDR fields changed");
  for(int i=0;i<3;++i)check(std::isnan(back.jerk[i]),"NaN jerk lost");check(std::isnan(back.yawspeed),"NaN yaw speed lost");
  a=healthy();a.receiver.body.xy_valid=false;rejects([&]{a.state(1.008,10.001);});
  a=healthy();a.receiver.body.heading_valid=false;rejects([&]{a.state(1.008,10.001);});
  a=healthy();rejects([&]{a.state(1.008,10.2);});
  a=healthy();rejects([&]{a.state(.999,10.001);});
  a=healthy();a.state(1.004,10.001); // Decimal control time == integer joint stamp.
  rejects([&]{a.state(1.004-1e-9,10.001);}); // One nanosecond future is still rejected.
  a=healthy();auto decimal_joint=joints();decimal_joint.header.stamp.sec=42;decimal_joint.header.stamp.nanosec=284'000'000;
  a.receive_joints(decimal_joint,10.001);
  a.receiver.body.position.sample=a.receiver.body.attitude.sample=a.receiver.body.odometry.sample=42'276'000;
  const auto decimal_state=a.state(42.284,10.002);
  check(decimal_state.joint_stamp==decimal_state.control_time,"Equal integer instants differ at native double boundary");
  a=healthy();rejects([&]{a.state(1.2,10.001);});
  a=healthy();a.receiver.body.q_ned_frd={0,0,0,0};rejects([&]{a.state(1.008,10.001);});
  a=healthy();a.receiver.acceleration_ned[0]=NAN;rejects([&]{a.state(1.008,10.001);});
  a=healthy();auto j=joints();j.name={"elbow","elbow"};rejects([&]{a.receive_joints(j,10.01);});
  rejects([&]{a.state(1.008,10.02);});
  a=healthy();j=joints();j.velocity.clear();rejects([&]{a.receive_joints(j,10.01);});
  a=healthy();j=joints();j.position[0]=NAN;rejects([&]{a.receive_joints(j,10.01);});
  a=healthy();j=joints();j.header.stamp.nanosec=0;a.receive_joints(j,10.01);rejects([&]{a.state(1.008,10.02);});
  a=healthy();pm::VehicleAttitude att;att.timestamp=att.timestamp_sample=1'001'000;att.q={1,0,0,0};att.quat_reset_counter=1;
  a.receiver.receive(att,10.001);rejects([&]{a.state(1.008,10.002);});
  a=healthy();a.receiver.body.status.update(900'000,900'000,10.001);rejects([&]{a.state(1.008,10.002);});
  a=healthy();a.receiver.body.odometry.sample-=60'000;rejects([&]{a.state(1.008,10.001);});
  a=healthy();a.receiver.body.armed_seen=true;rejects([&]{a.state(1.008,10.001);});
  a=healthy();a.receiver.body.offboard_seen=true;rejects([&]{a.state(1.008,10.001);});
  rejects([&]{x650_mpc::encode_mpc_output(result,0,0);});
  result.position_reference[0]=1e300;rejects([&]{x650_mpc::encode_mpc_output(result,1,0);});
  std::cout<<"PASS: golden frame/units/origin/acceleration, named joints, CDR float32/NaN, freshness/future/reset/invalid feedback, no doubled velocity, disarmed bench contract\n";
}
