cmake_minimum_required(VERSION 3.10)
project(VepRykControl)

# Set C++ standard
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Add compiler flags
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -O2")

# Find SDL2 package
find_package(PkgConfig REQUIRED)
pkg_check_modules(SDL2 REQUIRED sdl2)

# Include directories
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/common)
include_directories(${SDL2_INCLUDE_DIRS})

# Joystick handler library
add_library(joystick_handler 
    input/joystick_handler.cpp
)
target_link_libraries(joystick_handler ${SDL2_LIBRARIES})

# Test joystick executable
add_executable(test_joystick 
    input/test_joystick.cpp
)
target_link_libraries(test_joystick joystick_handler ${SDL2_LIBRARIES})

# Copy config.json to build directory automatically
configure_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/input/config.json
    ${CMAKE_CURRENT_BINARY_DIR}/config.json
    COPYONLY
)

message(STATUS "Building VepRyk Control")
message(STATUS "SDL2 version: ${SDL2_VERSION}")
message(STATUS "Config file will be copied to build directory")
