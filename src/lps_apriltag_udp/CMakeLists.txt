cmake_minimum_required(VERSION 3.8)
project(lps_apriltag_udp)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(isaac_ros_apriltag_interfaces REQUIRED)

add_executable(apriltag_udp_sender src/apriltag_udp_sender.cpp)
ament_target_dependencies(apriltag_udp_sender
  rclcpp
  isaac_ros_apriltag_interfaces
)

install(TARGETS apriltag_udp_sender
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
