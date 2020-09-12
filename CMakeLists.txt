cmake_minimum_required(VERSION 3.14.7)

project(worm)

set(WORM_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/include")

file(GLOB_RECURSE SOURCE_FILES RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}" "*.cpp" "*.hpp")
list(FILTER SOURCE_FILES EXCLUDE REGEX "CMakeFiles/[^;]+;?")

add_library(worm INTERFACE)

target_include_directories(worm INTERFACE
    $<BUILD_INTERFACE:${WORM_INCLUDE_DIR}>
    $<INSTALL_INTERFACE:include>)

set(TEST TRUE)

if(TEST)
    add_subdirectory(test)
endif()
