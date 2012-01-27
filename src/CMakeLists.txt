cmake_minimum_required (VERSION 2.8)

project (slam CXX)
add_executable (slam
    main.cpp planar_robot/waypoint_controller.cpp planar_robot/landmark_sensor.cpp
    utility/utility.cpp utility/cowtree.cpp)

set (CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/Modules/")

set (CMAKE_INCLUDE_CURRENT_DIR true)

find_package (Boost 1.48.0 COMPONENTS program_options filesystem system REQUIRED)
include_directories(${Boost_INCLUDE_DIRS})
target_link_libraries (slam ${Boost_LIBRARIES})

find_package (Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})
