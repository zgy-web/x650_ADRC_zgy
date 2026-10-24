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
  "msg/IngressStatus.msg"
  "msg/DispatchTick.msg"
  "msg/RuntimeStop.msg"
  "msg/AcceptedCommand.msg"
  "msg/LiveStart.msg"
  "msg/SourceBinding.msg"
  "msg/SourceBindingAck.msg"
  "msg/ActuatorState.msg"
  "msg/ExecutionReceipt.msg"
  "msg/ActuatorSend.msg"
)
ament_export_dependencies(rosidl_default_runtime)
ament_package()
