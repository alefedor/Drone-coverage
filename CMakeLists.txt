cmake_minimum_required(VERSION 3.8)
project(Drone_sweeping)

set(CMAKE_CXX_STANDARD 11)

set(CMAKE_BUILD_TYPE Debug)

set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -g -D_GLIBCXX_DEBUG")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O2")

include_directories(include)

set(SOURCE_FILES src/drone_sweeping.cpp)
add_executable(Drone_sweeping ${SOURCE_FILES})