cmake_minimum_required (VERSION 2.8)

project (slam CXX)

find_package (Boost COMPONENTS program_options REQUIRED)
find_package (Eigen3 REQUIRED)

include_directories(${Boost_INCLUDE_DIRS})
include_directories (${PROJECT_SOURCE_DIR})

add_executable (simulator simulator.cpp)
target_link_libraries (simulator ${Boost_LIBRARIES})
