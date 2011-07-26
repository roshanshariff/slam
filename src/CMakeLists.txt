cmake_minimum_required (VERSION 2.8)

project (slam CXX)
add_executable (simulator simulator.cpp)

include_directories (${PROJECT_SOURCE_DIR})
set (CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/Modules/")

find_package (Boost COMPONENTS program_options REQUIRED)
include_directories(${Boost_INCLUDE_DIRS})
target_link_libraries (simulator ${Boost_LIBRARIES})

find_package (Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})
