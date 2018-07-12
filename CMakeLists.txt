cmake_minimum_required(VERSION 3.0 FATAL_ERROR)

project(downsampling_compare)

message("The compiler used is ${CMAKE_CXX_COMPILER} or ${CMAKE_C_COMPILER} or ${CMAKE_C_COMPILER_ID} or${CMAKE_CXX_COMPILER_ID}")
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS} -O3")

find_package(PCL 1.8 REQUIRED COMPONENTS)
#find_package(Eigen3 3.3 REQUIRED NO_MODULE)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (downsampling_compare downsampling_compare.cpp)
target_link_libraries (downsampling_compare ${PCL_LIBRARIES})# Eigen3::Eigen )
