cmake_minimum_required(VERSION 3.5)
project(rh_p12_rn_control)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(dynamixel_sdk REQUIRED)

add_executable(control src/control.cpp)

ament_target_dependencies(control rclcpp dynamixel_sdk std_msgs)

install(TARGETS
  control
  DESTINATION lib/${PROJECT_NAME})

ament_package()
