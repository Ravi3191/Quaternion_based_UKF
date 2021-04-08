cmake_minimum_required(VERSION 3.1)
project (Cpp_based_FCN)
include(CheckFunctionExists)

add_definitions(-std=c++17)

set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS, "${CXX_FLAGS}")

include_directories(include Breast_Cancer_Data)
find_package (Eigen3 3.3 REQUIRED NO_MODULE)

add_library(data_handler src/data_handler.cpp)
target_link_libraries (data_handler Eigen3::Eigen)

add_library(ukf src/ukf.cpp)
target_link_libraries (ukf data_handler Eigen3::Eigen)


add_executable (filter src/filter.cpp)
target_link_libraries (filter Eigen3::Eigen ukf data_handler)