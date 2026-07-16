cmake_minimum_required(VERSION 3.14)
project(vpi_optical_flow)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(vpi 3.2 REQUIRED)

add_executable(vpi_optical_flow_node
    vpi_optical_flow_node.cpp
)

target_include_directories(vpi_optical_flow_node PRIVATE
    ${OpenCV_INCLUDE_DIRS}
)

ament_target_dependencies(vpi_optical_flow_node
    rclcpp
    sensor_msgs
    cv_bridge
)

target_link_libraries(vpi_optical_flow_node
    vpi
    ${OpenCV_LIBS}
)

install(TARGETS vpi_optical_flow_node
    DESTINATION lib/${PROJECT_NAME}
)

ament_package()
