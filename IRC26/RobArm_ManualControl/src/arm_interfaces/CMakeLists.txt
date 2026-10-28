cmake_minimum_required(VERSION 3.8)
project(arm_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

# Keep this list in sync with msg/. tests/check_interfaces.py verifies it.
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/UserInput.msg"
  "msg/MotorCommand.msg"
  "msg/ArmCommand.msg"
  "msg/MotorState.msg"
  "msg/ArmState.msg"
  "msg/ArmDiagnostics.msg"
  DEPENDENCIES std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
