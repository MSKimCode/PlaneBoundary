cmake_minimum_required(VERSION 3.14.7)

project(worm)

set(SRC_DIR "${CMAKE_CURRENT_SOURCE_DIR}/src")
file(GLOB_RECURSE SOURCE_FILES RELATIVE ${SRC_DIR} *.cpp *.hpp)

add_executable(worm ${SOURCE_FILES})
