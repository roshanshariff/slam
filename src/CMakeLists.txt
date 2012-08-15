cmake_minimum_required (VERSION 2.8)
project (slam CXX)

set (SLAM_SRCS main.cpp planar_robot/waypoint_controller.cpp planar_robot/landmark_sensor.cpp
    utility/utility.cpp utility/cowtree.cpp simulator/gnuplot_process.cpp simulator/slam_plotter.cpp
    simulator/time_series_plotter.cpp)

add_executable (slam ${SLAM_SRCS})

set (CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${PROJECT_SOURCE_DIR}/cmake/Modules/")

set (CMAKE_INCLUDE_CURRENT_DIR true)

find_package (Boost 1.48.0 COMPONENTS program_options filesystem system REQUIRED)
include_directories(${Boost_INCLUDE_DIRS})
target_link_libraries (slam ${Boost_LIBRARIES})

find_package (Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

find_package (CSparse REQUIRED)
include_directories(${CSPARSE_INCLUDE_DIR})
target_link_libraries(slam ${CSPARSE_LIBRARY})

find_package (G2O REQUIRED)
include_directories (${G2O_INCLUDE_DIR})
target_link_libraries (slam ${G2O_CORE_LIBRARY} ${G2O_STUFF_LIBRARY} ${G2O_SOLVER_CSPARSE} ${G2O_SOLVER_CSPARSE_EXTENSION})

set (CMAKE_BUILD_TYPE Release)

add_definitions ("-std=c++0x -Wall -Wextra -Werror -Wno-unused-parameter")
#add_definitions ("-std=c++0x -Wall -Werror -Wno-c++11-narrowing")
