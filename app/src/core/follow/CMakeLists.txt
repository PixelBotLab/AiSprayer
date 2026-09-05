cmake_minimum_required(VERSION 3.16)
project(follow_odometry LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()

include("${CMAKE_CURRENT_SOURCE_DIR}/../../../cmake/platform.cmake")

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${AIS_ARCH_FLAGS} -Wall -Wextra")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")
set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "-O3 -g")

set(PROJECT_ROOT "${AIS_PROJECT_ROOT}")
set(THIRD_PARTY_INSTALL_DIR "${AIS_THIRD_PARTY_DIR}")
set(SMALL_GICP_SRC_DIR "${PROJECT_ROOT}/third_party/src/small_gicp")

message(STATUS "Project Root: ${PROJECT_ROOT}")

set(_AIS_FOLLOW_LINK_DIRS ${THIRD_PARTY_INSTALL_DIR}/lib)
if(AIS_SYS_LIBDIR)
    list(APPEND _AIS_FOLLOW_LINK_DIRS ${AIS_SYS_LIBDIR})
endif()
link_directories(${_AIS_FOLLOW_LINK_DIRS})

# ----------------------------------------------------------------------------------
# Dependencies
# ----------------------------------------------------------------------------------

# OpenCV: core/imgproc/imgcodecs/features2d（CPU 前端用 goodFeaturesToTrack + SIFT float 描述子）
find_package(OpenCV REQUIRED core imgproc imgcodecs features2d)
message(STATUS "Found OpenCV: ${OpenCV_VERSION}")

# Eigen: Umeyama、Isometry3d、6x6 特征值
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

# OpenMP: small_gicp 的并行后端默认就是 OMP（TBB 是可选编译开关）
find_package(OpenMP)
if(NOT OpenMP_CXX_FOUND)
  if(APPLE)
    message(FATAL_ERROR "OpenMP not found. brew install libomp && export OpenMP_ROOT=$(brew --prefix libomp)")
  else()
    message(FATAL_ERROR "OpenMP is required for follow/small_gicp")
  endif()
endif()

# small_gicp: vendored in third_party/src (pinned v1.0.1, master@2024)。
# 注意它不是纯 header-only —— align()/preprocess_points() 的实现编译在 helper 库里。
if(NOT EXISTS "${SMALL_GICP_SRC_DIR}/CMakeLists.txt")
  message(FATAL_ERROR
      "small_gicp not found at ${SMALL_GICP_SRC_DIR}.\n"
      "  git clone -b master https://github.com/koide3/small_gicp ${SMALL_GICP_SRC_DIR}")
endif()
set(BUILD_SHARED_LIBS OFF CACHE BOOL "" FORCE)          # 静态链进 follow_node，避免污染 third_party/install
set(BUILD_WITH_TBB OFF CACHE BOOL "" FORCE)
set(BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(BUILD_BENCHMARKS OFF CACHE BOOL "" FORCE)
set(BUILD_PYTHON_BINDINGS OFF CACHE BOOL "" FORCE)
add_subdirectory("${SMALL_GICP_SRC_DIR}" "${CMAKE_BINARY_DIR}/small_gicp" EXCLUDE_FROM_ALL)

# 三方头文件的 -Wunused-parameter 之类噪声不要盖掉我们自己代码的告警
set_target_properties(small_gicp PROPERTIES
    INTERFACE_SYSTEM_INCLUDE_DIRECTORIES "${SMALL_GICP_SRC_DIR}/include")

# RKNN SuperPoint：生产 yaml 为 kind: cpu，默认不链 librknnrt。板上有模型时再 -DFOLLOW_ENABLE_RKNN=ON。
option(FOLLOW_ENABLE_RKNN "Build with the RKNN SuperPoint front-end" OFF)

# ----------------------------------------------------------------------------------
# libfollow
# ----------------------------------------------------------------------------------

set(FOLLOW_SOURCES
    src/cloud.cpp
    src/device_lock.cpp
    src/matching.cpp
    src/uncertainty.cpp
    src/pose_io.cpp
    src/frontend.cpp
    src/reference_map.cpp
    src/teach_core.cpp
    src/odometry.cpp
)

add_library(follow STATIC ${FOLLOW_SOURCES})

target_include_directories(follow PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(follow PUBLIC
    ${OpenCV_LIBS}
    Eigen3::Eigen
    OpenMP::OpenMP_CXX
    small_gicp            # vendored 目标在 add_subdirectory 里没有 namespace 别名
)

if(FOLLOW_ENABLE_RKNN)
  if(NOT EXISTS "${THIRD_PARTY_INSTALL_DIR}/lib/librknnrt.so")
    message(FATAL_ERROR "FOLLOW_ENABLE_RKNN=ON 但缺少 ${THIRD_PARTY_INSTALL_DIR}/lib/librknnrt.so")
  endif()
  target_include_directories(follow SYSTEM PUBLIC ${THIRD_PARTY_INSTALL_DIR}/include)
  target_link_libraries(follow PUBLIC "${THIRD_PARTY_INSTALL_DIR}/lib/librknnrt.so")
  target_compile_definitions(follow PUBLIC HAS_RKNN)
endif()

# ----------------------------------------------------------------------------------
# 配置层：只长 yaml-cpp，不长厂商 SDK，也不长 httplib。
#
# 单独一层是给"follow 作为库编进相机服务"这条路径准备的：orbbec_service 要读的是同一个
# configs/aisprayer_config.yaml 里的同一个 follow: 块，配置解析必须复用而不是抄一份（抄的那份
# 会静默地不认新键）。而它绝不该顺带链上 follow_device —— 那边有第二个 ob::Pipeline。
# ----------------------------------------------------------------------------------

find_package(yaml-cpp REQUIRED)

add_library(follow_config STATIC src/config_loader.cpp)
target_include_directories(follow_config PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(follow_config PUBLIC follow yaml-cpp::yaml-cpp)

# ----------------------------------------------------------------------------------
# 只有"被单独 configure"时才建的东西：验收工具、设备层、可执行、单测。
#
# 判据不用 PROJECT_IS_TOP_LEVEL（那是 3.21+ 才有的），自己比目录即可：3.16 上它必须照样成立，
# 否则老 cmake 配置出来的默认值会**静默少掉一半目标**，那比构建失败更难查。
# 守卫写成早 return，而不是把后面 100 行整体缩进 —— 后者会把每条注释的上下文搅乱。
# ----------------------------------------------------------------------------------

if("${CMAKE_CURRENT_SOURCE_DIR}" STREQUAL "${CMAKE_SOURCE_DIR}")
  set(_follow_is_top_level ON)
else()
  set(_follow_is_top_level OFF)
endif()
option(FOLLOW_STANDALONE "Build follow's own executables, tools and tests" ${_follow_is_top_level})

if(NOT FOLLOW_STANDALONE)
  message(STATUS "follow: 作为子项目被消费，只构建 follow / follow_config（工具与测试已跳过）")
  return()
endif()

# ----------------------------------------------------------------------------------
# P3 验收工具（follow_synth / gen_synth）
#
# 刻意不链进 libfollow：渲染器是验收脚手架，产品代码一行都不碰它。反过来 libfollow 也不
# 依赖 yaml-cpp —— 只有清单读写要它。
# ----------------------------------------------------------------------------------

add_library(follow_synth STATIC
    tools/synth_scene.cpp
    tools/sequence_io.cpp
)
target_include_directories(follow_synth PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/tools)
target_link_libraries(follow_synth PUBLIC follow yaml-cpp::yaml-cpp)

set(FOLLOW_SCAN_DIR "${PROJECT_ROOT}/data/template_group/2026-08-25_215601")

add_executable(gen_synth tools/gen_synth.cpp)
target_link_libraries(gen_synth PRIVATE follow_synth)
target_compile_definitions(gen_synth PRIVATE FOLLOW_SCAN_DIR="${FOLLOW_SCAN_DIR}")

# 验收门本体：读清单跑产品代码，退出码非零就是没过
add_executable(follow_replay tools/replay_main.cpp)
target_link_libraries(follow_replay PRIVATE follow_synth)

# ----------------------------------------------------------------------------------
# P4 设备层：单独一个库，**故意不让 libfollow 依赖厂商 SDK**。
#
# 理由是验收而不是洁癖：合成门（follow_replay）和全部单测跑的是同一套数学核心，一旦
# orbbec_capture.cpp 混进 libfollow，没插相机的机器上就再也构建不了这些检查了。设备相关的
# 断言只在板子上成立，所以它们的可执行入口也单独放（tools/capture_selftest.cpp）。
# ----------------------------------------------------------------------------------

if(AIS_ORBBEC_LIB)
  set(ORBBEC_SDK_LIB "${AIS_ORBBEC_LIB}")
else()
  find_library(ORBBEC_SDK_LIB NAMES OrbbecSDK libOrbbecSDK
    HINTS "${THIRD_PARTY_INSTALL_DIR}/lib")
  if(NOT ORBBEC_SDK_LIB)
    set(ORBBEC_SDK_LIB "${THIRD_PARTY_INSTALL_DIR}/lib/libOrbbecSDK.so")
  endif()
endif()

if(ORBBEC_SDK_LIB AND EXISTS "${ORBBEC_SDK_LIB}")
  # teach.cpp 放这里而不是 follow_app：它的签名里有 OrbbecCapture，符号只有链上设备层才解得开。
  # follow_app 里绝不能进这个文件 —— test_config 只链 follow_app，它必须能在没插相机的机器上链起来。
  add_library(follow_device STATIC src/orbbec_capture.cpp src/teach.cpp)
  target_include_directories(follow_device PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
  target_include_directories(follow_device SYSTEM PUBLIC ${THIRD_PARTY_INSTALL_DIR}/include)
  target_link_libraries(follow_device PUBLIC follow ${ORBBEC_SDK_LIB})
  set_target_properties(follow_device PROPERTIES BUILD_RPATH "${THIRD_PARTY_INSTALL_DIR}/lib")

  add_executable(follow_capture_selftest tools/capture_selftest.cpp)
  target_link_libraries(follow_capture_selftest PRIVATE follow_device)
  set_target_properties(follow_capture_selftest PROPERTIES BUILD_RPATH "${THIRD_PARTY_INSTALL_DIR}/lib")
else()
  message(STATUS "未检测到 Orbbec SDK（${ORBBEC_SDK_LIB}），跳过 follow_device / follow_node / follow_pose / follow_probe_device")
endif()

# 预算裁决：真实工位图上量 CPU 前端吃多少毫秒（决定 follow_node 的默认分辨率）。只链
# libfollow —— 它读的是 dump 下来的裸帧，不需要也不该需要厂商 SDK。
add_executable(follow_bench_frontend tools/bench_frontend.cpp)
target_link_libraries(follow_bench_frontend PRIVATE follow)

# ----------------------------------------------------------------------------------
# P4 进程装配层：健康端点 + 把配置层和健康层合起来的那个名字。
#
# 单独一层的原因和 follow_device 那条一样是构建可达性：libfollow 是算法核心，一旦它
# 因为 yaml-cpp / httplib 而长进依赖，没装这些的机器（以及所有纯数学单测）就构建不了。
# 配置的"读哪种格式"和 HTTP 的"用哪个库"都不该出现在 depth_to_cloud 的传递依赖里。
#
# 拆成 follow_config / follow_health 两块是因为相机服务只要前者：它自己有 http_server，
# 再链一份 httplib 的健康服务就是两个进程抢着报同一个状态。follow_app 这个名字保留成
# INTERFACE（= 配置 + 健康），所以 test_config 的写法不用改。链 follow_device 的可执行
# （follow_node / follow_pose）必须和设备层待在同一个 Orbbec 存在性守卫里。
# ----------------------------------------------------------------------------------

find_package(Threads REQUIRED)

add_library(follow_health STATIC src/health_server.cpp)
target_include_directories(follow_health PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_include_directories(follow_health SYSTEM PUBLIC ${THIRD_PARTY_INSTALL_DIR}/include)
target_link_libraries(follow_health PUBLIC follow Threads::Threads)

add_library(follow_app INTERFACE)
target_link_libraries(follow_app INTERFACE follow_config follow_health)

if(ORBBEC_SDK_LIB AND EXISTS "${ORBBEC_SDK_LIB}")
  add_executable(follow_node src/node_main.cpp)
  target_link_libraries(follow_node PRIVATE follow_app follow_device)
  set_target_properties(follow_node PROPERTIES BUILD_RPATH "${THIRD_PARTY_INSTALL_DIR}/lib")

  # 手持演示：拿着相机移动，控制台逐帧打印位姿（坐标 + 姿态）并给出可自量的精度小结。
  # 和 follow_node 链同一套库是有意的 —— 演示与生产共用示教实现、共用 Tracker、共用 to_dobot，
  # 这样"演示里数字对"才蕴含"发臂的数也是这么算的"，而不是两套代码各自对。
  add_executable(follow_pose tools/pose_main.cpp)
  target_link_libraries(follow_pose PRIVATE follow_app follow_device)
  set_target_properties(follow_pose PROPERTIES BUILD_RPATH "${THIRD_PARTY_INSTALL_DIR}/lib")

  # P4 设备问答：只在插着相机的板子上有意义。**故意不链 libfollow** —— 它是现场诊断工具，
  # 不该把产品库的符号拖进一条只问设备的可执行文件里。副作用要写明白：不链 libfollow 也就
  # 不拿 .orbbec.lock，而相机服务现在持这把锁 ⇒ 后端在跑时它会和 libusb 抢设备。先停后端。
  add_executable(follow_probe_device tools/probe_device.cpp)
  target_include_directories(follow_probe_device PRIVATE ${THIRD_PARTY_INSTALL_DIR}/include)
  target_link_libraries(follow_probe_device PRIVATE ${ORBBEC_SDK_LIB})
  set_target_properties(follow_probe_device PROPERTIES BUILD_RPATH "${THIRD_PARTY_INSTALL_DIR}/lib")
endif()

# ----------------------------------------------------------------------------------
# Tests
# ----------------------------------------------------------------------------------

enable_testing()
find_package(GTest REQUIRED)

add_executable(test_small_gicp_api test/test_small_gicp_api.cpp)
target_link_libraries(test_small_gicp_api PRIVATE small_gicp Eigen3::Eigen OpenMP::OpenMP_CXX GTest::gtest GTest::gtest_main)
add_test(NAME small_gicp_api COMMAND test_small_gicp_api)

add_executable(test_core test/test_core.cpp)
target_link_libraries(test_core PRIVATE follow GTest::gtest GTest::gtest_main)
target_compile_definitions(test_core PRIVATE FOLLOW_SCAN_DIR="${FOLLOW_SCAN_DIR}")
add_test(NAME core COMMAND test_core)

# P2：稠密配准回真值、退化说实话、参考地图可冻结可校验
add_executable(test_registration test/test_registration.cpp)
target_link_libraries(test_registration PRIVATE follow GTest::gtest GTest::gtest_main)
add_test(NAME registration COMMAND test_registration)

# P4 配置层与健康快照：纯逻辑，不需要相机也不需要网络（所以没插设备的机器上照样能跑）。
add_executable(test_config test/test_config.cpp)
target_link_libraries(test_config PRIVATE follow_app GTest::gtest GTest::gtest_main)
add_test(NAME config COMMAND test_config)
