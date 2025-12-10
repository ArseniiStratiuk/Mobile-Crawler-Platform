cmake_minimum_required(VERSION 3.10)
project(VeprykControl CXX)

# Set C++ standard
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Add compiler flags
add_compile_options(-Wall -Wextra -O3)

# Find packages
find_package(PkgConfig REQUIRED)
pkg_check_modules(SDL2 REQUIRED sdl2)
pkg_check_modules(LIBAV REQUIRED libavcodec libavformat libavutil libswscale)

# Find Qt5
find_package(Qt5 COMPONENTS Core Widgets Network REQUIRED)
set(CMAKE_AUTOMOC ON)

# Include directories
include_directories(${CMAKE_CURRENT_SOURCE_DIR})
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/common)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../headers)
include_directories(${SDL2_INCLUDE_DIRS})
include_directories(${LIBAV_INCLUDE_DIRS})

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

# Joystick test executable
add_executable(test_joystick
    input/test_joystick.cpp
)
target_link_libraries(test_joystick input common ${SDL2_LIBRARIES})

# GUI executable (integrated joystick + MAVLink + Qt5 + Video)
add_executable(vepryk_gui
    GUI/main.cpp
    GUI/main_window.cpp
    GUI/mavlink_thread.cpp
    GUI/video_thread.cpp
    GUI/video_display.cpp
)
target_link_libraries(vepryk_gui 
    network 
    input 
    common 
    ${SDL2_LIBRARIES}
    ${LIBAV_LIBRARIES}
    Qt5::Core
    Qt5::Widgets
    Qt5::Network
)

# Copy config.json to build directory automatically
configure_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/config.json
    ${CMAKE_CURRENT_BINARY_DIR}/config.json
    COPYONLY
)

# Installation
install(TARGETS vepryk_control vepryk_gui test_joystick
    RUNTIME DESTINATION bin
)
install(FILES config.json
    DESTINATION bin
)

message(STATUS "Building Vepryk Control System")
message(STATUS "  C++ Standard: ${CMAKE_CXX_STANDARD}")
message(STATUS "  MAVLink headers: ${CMAKE_CURRENT_SOURCE_DIR}/../headers")
message(STATUS "  SDL2 version: ${SDL2_VERSION}")
message(STATUS "  Qt5 version: ${Qt5_VERSION}")
message(STATUS "  Config file: config.json")
