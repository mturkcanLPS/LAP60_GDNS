cmake_minimum_required(VERSION 3.14)
project(lps_serial_sender)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(isaac_ros_apriltag_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(vpi 3.2 REQUIRED)

add_executable(serial_sender_node src/serial_sender_node.cpp)

target_include_directories(serial_sender_node PRIVATE
    include
    ${OpenCV_INCLUDE_DIRS}
)

ament_target_dependencies(serial_sender_node
    rclcpp
    isaac_ros_apriltag_interfaces
    sensor_msgs
    cv_bridge
)

target_link_libraries(serial_sender_node
    vpi
    ${OpenCV_LIBS}
)

install(TARGETS serial_sender_node
    DESTINATION lib/${PROJECT_NAME}
)

ament_package()
