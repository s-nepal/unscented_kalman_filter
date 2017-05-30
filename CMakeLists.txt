project(UnscentedKF)

set(CMAKE_BUILD_TYPE Debug)
cmake_minimum_required (VERSION 3.2)

add_definitions(-std=c++0x)

set(sources
   src/ukf.cpp
   src/main.cpp
   src/tools.cpp)

add_executable(UnscentedKF ${sources})
