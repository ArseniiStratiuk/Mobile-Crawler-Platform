cmake_minimum_required(VERSION 3.10)
project(SSHBridge)

# Set C++ standard
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Add compiler flags
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -O2")

# Find libssh package
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBSSH REQUIRED libssh)

# Computer executable (SSH client)
add_executable(computer computer.cpp)
target_include_directories(computer PRIVATE ${LIBSSH_INCLUDE_DIRS})
target_link_libraries(computer ${LIBSSH_LIBRARIES})

# Raspberry executable (SSH server receiver)
add_executable(raspberry raspberry.cpp)

# Print build info
message(STATUS "Building SSH Bridge")
message(STATUS "Computer client will be linked with libssh")
message(STATUS "Raspberry receiver is standalone")
