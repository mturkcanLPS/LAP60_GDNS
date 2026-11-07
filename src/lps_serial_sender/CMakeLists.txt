cmake_minimum_required(VERSION 3.14)
project(lps_serial_sender)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(isaac_ros_apriltag_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
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
    geometry_msgs
    cv_bridge
)

target_link_libraries(serial_sender_node
    vpi
    ${OpenCV_LIBS}
)

# --- CPU AprilTag node (Isaac GPU ile A/B icin; libapriltag) ---
find_library(APRILTAG_LIB apriltag)
if(APRILTAG_LIB)
    add_executable(cpu_apriltag_node src/cpu_apriltag_node.cpp)
    target_include_directories(cpu_apriltag_node PRIVATE ${OpenCV_INCLUDE_DIRS})
    ament_target_dependencies(cpu_apriltag_node
        rclcpp isaac_ros_apriltag_interfaces sensor_msgs cv_bridge)
    target_link_libraries(cpu_apriltag_node ${APRILTAG_LIB} ${OpenCV_LIBS})
    install(TARGETS cpu_apriltag_node DESTINATION lib/${PROJECT_NAME})
else()
    message(WARNING "libapriltag bulunamadi — cpu_apriltag_node atlaniyor "
                    "(kurmak icin: sudo apt install libapriltag-dev)")
endif()

install(TARGETS serial_sender_node
    DESTINATION lib/${PROJECT_NAME}
)

ament_package()
