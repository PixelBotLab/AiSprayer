cmake_minimum_required(VERSION 3.10)
project(cpp_demo LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(Threads REQUIRED)

# Define SDK paths (relative to cpp_demo directory)
set(SDK_INTERFACE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../third_party/robot_sdk/sdk_24.03/interface")
set(SDK_LIB_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../third_party/robot_sdk/sdk_24.03/cpp/linux/linux_gcc-9_x64_v2.0.4")

# Set build RPATH so the executable knows where to find libnrc_host.so at runtime without LD_LIBRARY_PATH
set(CMAKE_BUILD_RPATH "${SDK_LIB_DIR}")
set(CMAKE_INSTALL_RPATH "${SDK_LIB_DIR}")

add_executable(cpp_demo
    main.cpp
    inexbot_driver.cpp
    inexbot_driver.h
)

target_include_directories(cpp_demo PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${SDK_INTERFACE_DIR}
)

target_link_libraries(cpp_demo PRIVATE
    ${SDK_LIB_DIR}/libnrc_host.so
    Threads::Threads
)
