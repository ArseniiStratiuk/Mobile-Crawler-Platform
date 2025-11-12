cmake_minimum_required(VERSION 3.10)

project(mavlink_control C)

add_compile_options(-Wall -Wextra -Wpedantic)

add_executable(mavlink_control src/main.c)

target_include_directories(mavlink_control PRIVATE headers)
