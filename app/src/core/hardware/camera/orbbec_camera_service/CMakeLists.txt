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

# follow 把 Eigen3::Eigen / OpenMP::OpenMP_CXX 作为 PUBLIC 依赖往外传，而**非 GLOBAL 的
# imported target 不会从子目录向上冒**：不在本作用域先 find_package，configure 就会以
# "no such target" 收场。这两个包板子上都是现成的（/usr/share/eigen3/cmake、系统 OpenMP）。
find_package(OpenMP REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

# ----------------------------------------------------------------------------------
# follow 作为**库**编进本进程，而不是再起一个 follow_node。
#
# 理由不是性能，是设备归属：Orbbec 是独占设备，两个进程各开一路 pipeline 只会有一路拿到
# 和彩色不对齐的深度、或者干脆打不开还装作在跑。follow 要的就是"本进程已经在交付的那一路
# D2C 帧"，所以它只能是消费者。
#
# FOLLOW_STANDALONE 由 follow/CMakeLists.txt 按"是否被单独 configure"自己判断：作为子项目
# 时只建 follow（算法核心）+ follow_config（读同一个 yaml 的 follow: 块）。**绝不链
# follow_device** —— 那里面是第二个 Context/Pipeline/取流线程，同进程两份 driver 比两份
# 进程更难查。也绝不链 follow_health：状态由本进程自己的 http_server 报，两处报同一个
# 状态早晚分成两个答案。
# ----------------------------------------------------------------------------------
add_subdirectory("${PROJECT_ROOT}/follow" "${CMAKE_BINARY_DIR}/follow")

# Source files
set(SOURCES
    src/main.cpp
    src/camera_driver.cpp
    src/rga_processor.cpp
    src/mpp_encoder.cpp
    src/zlm_streamer.cpp
    src/corner_detector.cpp
    src/async_disk_writer.cpp
    src/follow_worker.cpp
    src/http_server.cpp
)

# Executable target
add_executable(orbbec_camera_service ${SOURCES})

# Target link libraries
target_link_libraries(orbbec_camera_service
    PRIVATE
    follow            # 工位跟随的算法核心（含 vendored small_gicp）
    follow_config     # 读 configs/aisprayer_config.yaml 里那个 follow: 块
    OpenMP::OpenMP_CXX  # small_gicp 的并行后端就是 OMP；编译有 -fopenmp，链接也得带上
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
