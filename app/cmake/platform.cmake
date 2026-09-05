# Shared host/platform detection for motion, follow, and orbbec_camera_service.
# Include after project(). Do not probe /dev here — that breaks cross-compile and CI.
include_guard(GLOBAL)

get_filename_component(AIS_APP_DIR "${CMAKE_CURRENT_LIST_DIR}/.." ABSOLUTE)
get_filename_component(AIS_PROJECT_ROOT "${AIS_APP_DIR}/.." ABSOLUTE)
set(AIS_THIRD_PARTY_DIR "${AIS_PROJECT_ROOT}/third_party/install")
set(AIS_THIRD_PARTY_LIB "${AIS_THIRD_PARTY_DIR}/lib")
set(AIS_THIRD_PARTY_INC "${AIS_THIRD_PARTY_DIR}/include")

if(APPLE)
    set(AIS_PLATFORM "macos")
elseif(UNIX)
    set(AIS_PLATFORM "linux")
else()
    set(AIS_PLATFORM "unknown")
endif()

# Apple Silicon is arm64; that is not RK3588.
if(AIS_PLATFORM STREQUAL "linux" AND CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
    set(_AIS_RK_DEFAULT ON)
else()
    set(_AIS_RK_DEFAULT OFF)
endif()
option(ENABLE_RK3588 "Rockchip RK3588 MPP/RGA (and optional RKNN)" ${_AIS_RK_DEFAULT})

set(AIS_ARCH_FLAGS "")
set(AIS_SYS_LIBDIR "")
if(ENABLE_RK3588)
    # Identical to the previous per-module hardcoding. Do not "improve" this string.
    set(AIS_ARCH_FLAGS "-mcpu=cortex-a76.cortex-a55 -mtune=cortex-a76 -ftree-vectorize -fomit-frame-pointer")
    add_compile_definitions(HAS_RK3588=1 HAS_RGA=1 HAS_MPP=1)
    foreach(_lib IN ITEMS libOrbbecSDK.so librga.so librockchip_mpp.so libmk_api.so)
        if(NOT EXISTS "${AIS_THIRD_PARTY_LIB}/${_lib}")
            message(FATAL_ERROR
                "ENABLE_RK3588=ON but missing ${AIS_THIRD_PARTY_LIB}/${_lib}. "
                "Run app/scripts/build.sh, or configure with -DENABLE_RK3588=OFF.")
        endif()
    endforeach()
    set(AIS_ORBBEC_LIB "${AIS_THIRD_PARTY_LIB}/libOrbbecSDK.so")
    set(AIS_RGA_LIB "${AIS_THIRD_PARTY_LIB}/librga.so")
    set(AIS_MPP_LIB "${AIS_THIRD_PARTY_LIB}/librockchip_mpp.so")
    set(AIS_MK_API_LIB "${AIS_THIRD_PARTY_LIB}/libmk_api.so")
    set(AIS_SYS_LIBDIR "/usr/lib/aarch64-linux-gnu")
else()
    find_library(AIS_ORBBEC_LIB NAMES OrbbecSDK libOrbbecSDK
        HINTS "${AIS_THIRD_PARTY_LIB}")
    find_library(AIS_MK_API_LIB NAMES mk_api libmk_api
        HINTS "${AIS_THIRD_PARTY_LIB}")
    find_library(AIS_OPENH264_LIB NAMES openh264
        HINTS "${AIS_THIRD_PARTY_LIB}")
endif()

message(STATUS "AiSprayer platform: ${AIS_PLATFORM} (${CMAKE_SYSTEM_PROCESSOR}), ENABLE_RK3588=${ENABLE_RK3588}")
