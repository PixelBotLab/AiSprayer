cmake_minimum_required(VERSION 3.16)
project(orbbec_camera_service LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# Optimization flags for RK3588 (4x Cortex-A76 + 4x Cortex-A55) with ARM NEON / SIMD / FP16
set(ARM_ARCH_FLAGS "-mcpu=cortex-a76.cortex-a55 -mtune=cortex-a76 -ftree-vectorize -fomit-frame-pointer")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 ${ARM_ARCH_FLAGS} -Wall -Wno-unused-function -pthread -fopenmp")

# Resolve Project Root and Third Party Directory
get_filename_component(PROJECT_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../../../../../.." ABSOLUTE)
set(THIRD_PARTY_INSTALL_DIR "${PROJECT_ROOT}/third_party/install")

message(STATUS "Project Root: ${PROJECT_ROOT}")
message(STATUS "Third Party Install Dir: ${THIRD_PARTY_INSTALL_DIR}")

# Include directories
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${THIRD_PARTY_INSTALL_DIR}/include
    ${THIRD_PARTY_INSTALL_DIR}/include/rockchip
    ${THIRD_PARTY_INSTALL_DIR}/include/libobsensor
    /usr/include
)

# Link directories
link_directories(
    ${THIRD_PARTY_INSTALL_DIR}/lib
    /usr/lib/aarch64-linux-gnu
)

# Find OpenCV
find_package(OpenCV REQUIRED core imgproc imgcodecs calib3d)
message(STATUS "Found OpenCV: ${OpenCV_VERSION}")

# Find yaml-cpp
find_package(yaml-cpp QUIET)
if(NOT yaml-cpp_FOUND)
    find_library(YAML_CPP_LIB yaml-cpp REQUIRED)
else()
    set(YAML_CPP_LIB yaml-cpp)
endif()

# Source files
set(SOURCES
    src/main.cpp
    src/camera_driver.cpp
    src/rga_processor.cpp
    src/mpp_encoder.cpp
    src/zlm_streamer.cpp
    src/corner_detector.cpp
    src/async_disk_writer.cpp
    src/http_server.cpp
)

# Executable target
add_executable(orbbec_camera_service ${SOURCES})

# Target link libraries
target_link_libraries(orbbec_camera_service
    PRIVATE
    ${OpenCV_LIBS}
    ${YAML_CPP_LIB}
    ${THIRD_PARTY_INSTALL_DIR}/lib/libOrbbecSDK.so
    ${THIRD_PARTY_INSTALL_DIR}/lib/librga.so
    ${THIRD_PARTY_INSTALL_DIR}/lib/librockchip_mpp.so
    ${THIRD_PARTY_INSTALL_DIR}/lib/libmk_api.so
    pthread
    dl
)

# Output directory
set_target_properties(orbbec_camera_service PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/bin"
)
