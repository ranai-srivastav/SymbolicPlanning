cmake_minimum_required(VERSION 3.16.3)
project(planning_hw3)

set(CMAKE_CXX_STANDARD 14)

# Enable compile_commands.json for IDEs
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

include_directories(include)

add_executable(planner
        src/planner.cpp
)

target_compile_definitions(planner PRIVATE ENVS_DIR="${CMAKE_SOURCE_DIR}/envs")

# Debug configuration: No optimizations, full debugging info
set(CMAKE_CXX_FLAGS_DEBUG "-g -O0 -DDEBUG")

# Release configuration: Maximum optimizations
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG")

# RelWithDebInfo: Optimizations with debug info (useful alternative)
set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "-O2 -g -DNDEBUG")

# MinSizeRel: Optimize for size
set(CMAKE_CXX_FLAGS_MINSIZEREL "-Os -DNDEBUG")

# Set default build type to Debug if not specified
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Debug CACHE STRING "Choose the type of build (Debug or Release)" FORCE)
endif()

message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")