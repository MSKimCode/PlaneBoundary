cmake_minimum_required(VERSION 3.14.7)

project(worm)

file(GLOB_RECURSE SOURCE_FILES RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}" "*.cpp" "*.hpp")
list(FILTER SOURCE_FILES EXCLUDE REGEX "CMakeFiles/[^;]+;?")

message("=== Source Files ===")
foreach(F IN LISTS SOURCE_FILES)
    message("${F}")
endforeach()

add_executable(worm ${SOURCE_FILES})
