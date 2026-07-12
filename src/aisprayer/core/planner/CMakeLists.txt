cmake_minimum_required(VERSION 3.14)
project(aisprayer_planner)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Automatically add the local deps/install directory to CMake's search path
list(APPEND CMAKE_PREFIX_PATH "${CMAKE_SOURCE_DIR}/deps/install")

# Find cxxopts for CLI argument parsing
find_package(cxxopts REQUIRED)

# Find JSON serialization and threading support
find_package(nlohmann_json REQUIRED)
find_package(Threads REQUIRED)
find_package(yaml-cpp REQUIRED)

# Find PCL and VTK
find_package(PCL REQUIRED COMPONENTS common io features surface)
add_definitions(${PCL_DEFINITIONS})

# Find Noether TPP
find_package(noether_tpp REQUIRED)

# Find Tesseract
find_package(tesseract REQUIRED COMPONENTS
    environment
    common
)

# Find Tesseract Planning
find_package(tesseract_planning REQUIRED COMPONENTS
    command_language
    motion_planners
    motion_planners_trajopt
)

link_directories(${CMAKE_SOURCE_DIR}/deps/install/lib)

# Set RPATH so the executable can find Tesseract shared libraries at runtime
set(CMAKE_BUILD_WITH_INSTALL_RPATH TRUE)
set(CMAKE_INSTALL_RPATH "${CMAKE_SOURCE_DIR}/deps/install/lib")
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

add_library(planner_common STATIC
    src/common/types.cpp
    src/common/thread_pool.cpp
)

target_include_directories(planner_common PUBLIC
    ${CMAKE_SOURCE_DIR}/include
)

target_link_libraries(planner_common PUBLIC
    Threads::Threads
)

add_library(planner_process STATIC
    src/process/process_planner.cpp
)

target_include_directories(planner_process PUBLIC
    ${CMAKE_SOURCE_DIR}/include
    ${PCL_INCLUDE_DIRS}
)

target_link_libraries(planner_process PUBLIC
    planner_common
    nlohmann_json::nlohmann_json
    yaml-cpp::yaml-cpp
    noether::noether_tpp
    ${PCL_LIBRARIES}
)

add_executable(process_planner
    src/process/process_main.cpp
)

target_link_libraries(process_planner PRIVATE
    planner_process
    cxxopts::cxxopts
)

add_library(planner_motion STATIC
    src/motion/motion_planner.cpp
)

target_include_directories(planner_motion PUBLIC
    ${CMAKE_SOURCE_DIR}/include
)

target_link_libraries(planner_motion PUBLIC
    planner_common
    nlohmann_json::nlohmann_json
    yaml-cpp::yaml-cpp
    tesseract::environment
    tesseract::common
    tesseract::command_language
    tesseract::motion_planners
    tesseract::motion_planners_trajopt
    qpOASES
    osqp
)

add_executable(motion_planner
    src/motion/motion_main.cpp
)

target_link_libraries(motion_planner PRIVATE
    planner_motion
    cxxopts::cxxopts
)
