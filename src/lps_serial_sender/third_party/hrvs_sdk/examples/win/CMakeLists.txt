cmake_minimum_required(VERSION 3.10.2)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(CMAKE_CXX_COMPILER "g++")
set(HRVS_SDK_PATH ../../radar_system)
set(THREADS_PREFER_PTHREAD_FLAG ON)
project(hrvs_example_win LANGUAGES C CXX)
find_package(Threads REQUIRED)
file(GLOB SOURCES 
    ${HRVS_SDK_PATH}/src/pal/uart_windows.cpp
    ${HRVS_SDK_PATH}/src/pal/gpio_pin_arduino.cpp
    ${HRVS_SDK_PATH}/src/pal/thread_stdlib.cpp
    ${HRVS_SDK_PATH}/src/pal/time_stdlib.cpp
    ${PROJECT_SOURCE_DIR}/src/main.cpp)
add_executable(${PROJECT_NAME} ${SOURCES})
target_include_directories(${PROJECT_NAME} PRIVATE ${HRVS_SDK_PATH}/include)
target_link_libraries(${PROJECT_NAME} PRIVATE ${PROJECT_SOURCE_DIR}/${HRVS_SDK_PATH}/lib/hrvs_sdk_win.lib)
target_link_libraries(${PROJECT_NAME} PRIVATE Threads::Threads)
target_compile_definitions(${PROJECT_NAME} PRIVATE MSG_LOGGING)