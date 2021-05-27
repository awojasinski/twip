cmake_minimum_required(VERSION 3.19 FATAL_ERROR)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT CMAKE_BUILD_TYPE)
    message(STATUS "CMAKE_BUILD_TYPE not set, defaulting to DEBUG")
    set(CMAKE_BUILD_TYPE DEBUG)
else()
    message(STATUS "CMAKE_BUILD_TYPE: ${CMAKE_BUILD_TYPE}")
endif()


if(NOT CMAKE_TOOLCHAIN_FILE)
    message(FATAL_ERROR "CMAKE_TOOLCHAIN_FILE not set")
endif()
message(STATUS "CMAKE_TOOLCHAIN_FILE: ${CMAKE_TOOLCHAIN_FILE}")


project(twip C CXX ASM)

include(cmake/utils.cmake)

add_subdirectory(hal)
add_subdirectory(app)
add_subdirectory(libs/cli)
add_subdirectory(libs/uart)
add_subdirectory(libs/fifo)
add_subdirectory(libs/mpu9250)
add_subdirectory(libs/logger)
add_subdirectory(libs/encoder)