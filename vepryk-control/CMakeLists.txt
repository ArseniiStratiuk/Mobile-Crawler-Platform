cmake_minimum_required(VERSION 3.10)
project(VeprykControl CXX)

# Set C++ standard
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Add compiler flags
add_compile_options(-Wall -Wextra -O3)

# Find SDL2 package
find_package(PkgConfig REQUIRED)
pkg_check_modules(SDL2 REQUIRED sdl2)

# Include directories
include_directories(${CMAKE_CURRENT_SOURCE_DIR})
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/common)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../headers)
include_directories(${SDL2_INCLUDE_DIRS})

# Common library (config loader)
add_library(common
    common/config_loader.cpp
)

# Network library (MAVLink sender + SSH bridge)
add_library(network
    network/ssh_bridge.cpp
    network/mavlink_sender.cpp
)

# Input library (Joystick handler)
add_library(input
    input/joystick_handler.cpp
)
target_link_libraries(input ${SDL2_LIBRARIES})

# Main executable
add_executable(vepryk_control
    main.cpp
)
target_link_libraries(vepryk_control network input common ${SDL2_LIBRARIES})

# Network test executable
add_executable(test_network
    network/test_network.cpp
)
target_link_libraries(test_network network common)

# Joystick test executable
add_executable(test_joystick
    input/test_joystick.cpp
)
target_link_libraries(test_joystick input common ${SDL2_LIBRARIES})

# Copy config.json to build directory automatically
configure_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/config.json
    ${CMAKE_CURRENT_BINARY_DIR}/config.json
    COPYONLY
)

# Installation
install(TARGETS vepryk_control test_network test_joystick
    RUNTIME DESTINATION bin
)
install(FILES config.json
    DESTINATION bin
)

message(STATUS "Building Vepryk Control System")
message(STATUS "  C++ Standard: ${CMAKE_CXX_STANDARD}")
message(STATUS "  MAVLink headers: ${CMAKE_CURRENT_SOURCE_DIR}/../headers")
message(STATUS "  SDL2 version: ${SDL2_VERSION}")
message(STATUS "  Config file: config.json")
