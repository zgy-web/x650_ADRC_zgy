cmake_minimum_required(VERSION 3.8)
project(flyarm_control_interfaces)
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ControlSnapshot.msg"
  "msg/ControlProposal.msg"
  "msg/GateEvent.msg"
  "msg/DrainRequest.msg"
  "msg/RecordingReceipt.msg"
)
ament_export_dependencies(rosidl_default_runtime)
ament_package()
