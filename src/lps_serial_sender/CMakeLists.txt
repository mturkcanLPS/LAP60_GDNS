cmake_minimum_required(VERSION 3.14)
project(lps_serial_sender)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(isaac_ros_apriltag_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)   # infra2 optik akis (Vector3Stamped)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(vpi 3.2 REQUIRED)

# --- HRVS SDK ---
set(HRVS_SDK_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/third_party/hrvs_sdk/radar_system)
set(HRVS_SDK_LIB  ${HRVS_SDK_ROOT}/lib/hrvs_sdk_arm64.a)
set(HRVS_PAL_SRCS
    ${HRVS_SDK_ROOT}/src/pal/uart_linux.cpp
    ${HRVS_SDK_ROOT}/src/pal/gpio_pin_linux.cpp
    ${HRVS_SDK_ROOT}/src/pal/time_stdlib.cpp
    ${HRVS_SDK_ROOT}/src/pal/thread_stdlib.cpp
)

add_executable(serial_sender_node
    src/serial_sender_node.cpp
    ${HRVS_PAL_SRCS}
)

target_include_directories(serial_sender_node PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${OpenCV_INCLUDE_DIRS}
    ${HRVS_SDK_ROOT}/include
)

target_compile_definitions(serial_sender_node PRIVATE
    MSG_LOGGING
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
    ${HRVS_SDK_LIB}
    gpiod
    -Wl,-rpath,/usr/lib/aarch64-linux-gnu
)

install(TARGETS serial_sender_node
    DESTINATION lib/${PROJECT_NAME}
)

# --- CPU AprilTag node (libapriltag; GPU/Isaac ile parametreyle secilebilir) ---
# serial_sender'dan BAGIMSIZ: HRVS SDK'ya ihtiyac duymaz, sadece libapriltag.
find_library(APRILTAG_LIB apriltag)
if(APRILTAG_LIB)
    add_executable(cpu_apriltag_node src/cpu_apriltag_node.cpp)
    target_include_directories(cpu_apriltag_node PRIVATE ${OpenCV_INCLUDE_DIRS}
      ${CMAKE_CURRENT_SOURCE_DIR}/include)   # stamp_seconds.hpp
    ament_target_dependencies(cpu_apriltag_node
        rclcpp isaac_ros_apriltag_interfaces sensor_msgs cv_bridge)
    target_link_libraries(cpu_apriltag_node ${APRILTAG_LIB} ${OpenCV_LIBS})
    install(TARGETS cpu_apriltag_node DESTINATION lib/${PROJECT_NAME})
else()
    message(WARNING "libapriltag bulunamadi — cpu_apriltag_node atlaniyor "
                    "(kurmak icin: sudo apt install libapriltag-dev)")
endif()

# --- infra2: VPI akis (-> /infra2/optical_flow -> serial opticflow2) + video ---
# Seri port / HRVS kodu YOK; paketin tek yazari serial_sender. serial_sender
# ile ortak basliklari kullanir (ayni fuzyon/akis hesabi).
add_executable(infra2_recorder_node src/infra2_recorder_node.cpp)
target_include_directories(infra2_recorder_node PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${OpenCV_INCLUDE_DIRS}
)
ament_target_dependencies(infra2_recorder_node
    rclcpp isaac_ros_apriltag_interfaces sensor_msgs geometry_msgs cv_bridge)
target_link_libraries(infra2_recorder_node vpi ${OpenCV_LIBS}
    -Wl,-rpath,/usr/lib/aarch64-linux-gnu)
install(TARGETS infra2_recorder_node DESTINATION lib/${PROJECT_NAME})

# --- tag_fusion.hpp esdegerlik testi (eski serial_sender tagCb ile bit-bit) ---
# Calistirma: ros2 run lps_serial_sender test_tag_fusion
add_executable(test_tag_fusion test/test_tag_fusion.cpp)
target_include_directories(test_tag_fusion PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(test_tag_fusion ${OpenCV_LIBS})
install(TARGETS test_tag_fusion DESTINATION lib/${PROJECT_NAME})

# --- IR ucgeni sentetik dogrulama (donanimsiz calisir) ---
# Node ile AYNI kodu (include/lps_serial_sender/ir_triangle.hpp) kullanir.
# Calistirma: ros2 run lps_serial_sender test_ir_triangle
add_executable(test_ir_triangle test/test_ir_triangle.cpp)
target_include_directories(test_ir_triangle PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(test_ir_triangle ${OpenCV_LIBS})
install(TARGETS test_ir_triangle DESTINATION lib/${PROJECT_NAME})

# --- Otopilot RX ayristirici testi (0x49 KAPAT; donanimsiz calisir) ---
# Node ile AYNI kodu (include/lps_serial_sender/autopilot_rx.hpp) kullanir.
# Calistirma: ros2 run lps_serial_sender test_autopilot_rx
add_executable(test_autopilot_rx test/test_autopilot_rx.cpp)
target_include_directories(test_autopilot_rx PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
install(TARGETS test_autopilot_rx DESTINATION lib/${PROJECT_NAME})

ament_package()
