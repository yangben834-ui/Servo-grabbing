cmake_minimum_required(VERSION 3.8)
project(h3001_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/H3001ArmStatus.msg"
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
