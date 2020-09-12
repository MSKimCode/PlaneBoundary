cmake_minimum_required(VERSION 3.14.7)

project(detect)

add_executable(detect "main.cpp" "header.hpp")
target_link_libraries(detect worm)
