cmake_minimum_required(VERSION 3.10)
project(cr5_kinematics_cpp)

set(CMAKE_CXX_STANDARD 14)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -fPIC")

add_library(ur_kin SHARED
    cr5_ur_kin.cpp
    cr5_kinematics.cpp
    cr5_path_opt.cpp
    c_wrapper.cpp
)
target_include_directories(ur_kin PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

# Standalone IK benchmark / functional test (same protocol as test_cr5_kinematics.py)
add_executable(test_cr5_kinematics
    test_cr5_kinematics.cpp
    cr5_ur_kin.cpp
    cr5_kinematics.cpp
)
target_include_directories(test_cr5_kinematics PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
