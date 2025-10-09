cmake_minimum_required(VERSION 3.10)

project(mavlink_connection C)

add_compile_options(-Wall -Wextra)

add_executable(mavlink_connection src/main.c)

target_include_directories(mavlink_connection PRIVATE headers)
