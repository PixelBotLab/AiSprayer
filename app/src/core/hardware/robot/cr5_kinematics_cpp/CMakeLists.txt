cmake_minimum_required(VERSION 3.10)
project(cr5_kinematics_cpp)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -fPIC")

add_library(ur_kin SHARED 
    cr5_ur_kin.cpp 
    cr5_kinematics.cpp 
    c_wrapper.cpp
)

target_include_directories(ur_kin PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
