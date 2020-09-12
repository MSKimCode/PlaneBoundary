cmake_minimum_required(VERSION 3.14.7)

project(worm)

set(WORM_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/include")

file(GLOB_RECURSE SOURCE_FILES RELATIVE "${WORM_INCLUDE_DIR}" "*.hpp")
list(FILTER SOURCE_FILES EXCLUDE REGEX "CMakeFiles/[^;]+;?")

add_library(worm INTERFACE)
target_compile_features(worm INTERFACE cxx_std_17)

target_include_directories(worm INTERFACE
    $<BUILD_INTERFACE:${WORM_INCLUDE_DIR}>
    $<INSTALL_INTERFACE:include>)

set(TEST TRUE)

if(TEST)
    if(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
        set(CMAKE_CXX_FLAGS "-std=c++17 -stdlib=libc++ ${CMAKE_CXX_FLAGS}")
    endif()
    add_subdirectory(src)
    add_subdirectory(test)
endif()
