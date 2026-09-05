cmake_minimum_required(VERSION 3.16)
project(aisprayer_visioncpp LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()

include("${CMAKE_CURRENT_SOURCE_DIR}/../../../cmake/platform.cmake")
if(AIS_ARCH_FLAGS)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${AIS_ARCH_FLAGS}")
endif()
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra")

find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(OpenCV REQUIRED core imgproc imgcodecs photo)
find_package(yaml-cpp REQUIRED)
if(TARGET yaml-cpp::yaml-cpp)
  set(VISIONCPP_YAML yaml-cpp::yaml-cpp)
else()
  set(VISIONCPP_YAML yaml-cpp)
endif()

set(POISSON_SRC "${CMAKE_CURRENT_SOURCE_DIR}/third_party/PoissonRecon/Src")
if(NOT EXISTS "${POISSON_SRC}/Reconstructors.h")
  message(FATAL_ERROR "Vendored PoissonRecon missing at ${POISSON_SRC}")
endif()

add_library(visioncpp STATIC
  src/depth_map.cpp
  src/mask_set.cpp
  src/calibration.cpp
  src/point_cloud.cpp
  src/mesh.cpp
  src/plane_slicer.cpp
  src/normal_smoother.cpp
  src/tool_frame.cpp
  src/auto_path_planner.cpp
  src/poisson_fallback.cpp
)
target_include_directories(visioncpp PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(visioncpp PUBLIC
  Eigen3::Eigen
  ${OpenCV_LIBS}
  ${VISIONCPP_YAML}
)
target_compile_options(visioncpp PRIVATE -O3 -DNDEBUG -fno-fast-math)

add_library(visioncpp_poisson STATIC src/poisson_recon.cpp src/recon_pipeline.cpp)
target_include_directories(visioncpp_poisson PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${POISSON_SRC}
)
target_link_libraries(visioncpp_poisson PUBLIC visioncpp)
target_compile_options(visioncpp_poisson PRIVATE
  -O3 -DNDEBUG -DRELEASE -w
)
if(UNIX)
  target_link_libraries(visioncpp_poisson PUBLIC pthread)
endif()

add_executable(vision_cli src/cli_main.cpp)
target_link_libraries(vision_cli PRIVATE visioncpp_poisson)

get_filename_component(REPO_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../../../.." ABSOLUTE)

enable_testing()
add_executable(test_auto_path test/test_auto_path.cpp)
target_link_libraries(test_auto_path PRIVATE visioncpp)
target_compile_definitions(test_auto_path PRIVATE REPO_ROOT="${REPO_ROOT}")
add_test(NAME test_auto_path COMMAND test_auto_path)

add_executable(test_recon test/test_recon.cpp)
target_link_libraries(test_recon PRIVATE visioncpp_poisson)
target_compile_definitions(test_recon PRIVATE REPO_ROOT="${REPO_ROOT}")
add_test(NAME test_recon COMMAND test_recon)
