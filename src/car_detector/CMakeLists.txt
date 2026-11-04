cmake_minimum_required(VERSION 3.16)
project(car_detector)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wno-unused-parameter)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(realsense2_camera_msgs REQUIRED)   # IR->renkli extrinsics (iki kamera)
find_package(OpenCV REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_check_modules(GST REQUIRED gstreamer-1.0 gstreamer-app-1.0 gstreamer-video-1.0)
pkg_check_modules(GLIB REQUIRED glib-2.0 gobject-2.0)

# ---- DeepStream 7.1 (Jetson) ----
set(DS_DIR /opt/nvidia/deepstream/deepstream)
set(DS_INC ${DS_DIR}/sources/includes)
set(DS_LIB ${DS_DIR}/lib)

add_executable(car_detector_node src/car_detector_node.cpp)

target_include_directories(car_detector_node PRIVATE
  ${DS_INC} ${GST_INCLUDE_DIRS} ${GLIB_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

target_link_directories(car_detector_node PRIVATE ${DS_LIB})

target_link_libraries(car_detector_node
  ${GST_LIBRARIES} ${GLIB_LIBRARIES} ${OpenCV_LIBS}
  nvdsgst_meta nvds_meta)

ament_target_dependencies(car_detector_node
  rclcpp sensor_msgs geometry_msgs ament_index_cpp realsense2_camera_msgs)

# DeepStream .so'lari calisma aninda bulunsun
set_target_properties(car_detector_node PROPERTIES
  BUILD_RPATH ${DS_LIB} INSTALL_RPATH ${DS_LIB})

install(TARGETS car_detector_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

ament_package()
