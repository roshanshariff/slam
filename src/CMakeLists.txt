cmake_minimum_required (VERSION 2.8.8)
project (slam CXX)
enable_language (C)

add_library (planar_robot STATIC
    planar_robot/waypoint_controller.cpp planar_robot/landmark_sensor.cpp
    planar_robot/velocity_model.cpp planar_robot/range_bearing_model.cpp
    planar_robot/rms_error.cpp)

add_library (simulator STATIC
    simulator/simulator.cpp simulator/gnuplot_process.cpp simulator/slam_plotter.cpp
    simulator/time_series_plotter.cpp)

add_library (slam_impl STATIC
    slam/slam_data.cpp slam/mcmc_slam.cpp slam/multi_mcmc.cpp slam/g2o_slam.cpp
    slam/g2o_clustering.cpp
    #slam/fastslam.cpp slam/fastslam_mcmc.cpp
    )

add_library (utility STATIC
    utility/utility.cpp utility/cowtree.cpp)

add_library (nnls STATIC
    utility/nnls.cpp utility/nnls.c)

add_library (dataset STATIC
    dataset.cpp)

add_executable (slam main.cpp)
target_link_libraries (slam planar_robot simulator slam_impl utility
                      dataset nnls)

set (CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${PROJECT_SOURCE_DIR}/cmake/Modules/")

set (CMAKE_INCLUDE_CURRENT_DIR true)

set (CMAKE_BUILD_TYPE Release)

find_package (Boost 1.50.0 COMPONENTS program_options filesystem system timer REQUIRED)
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

if (CMAKE_COMPILER_IS_GNUCXX)
    add_definitions ("-Werror -Wall -Wextra -march=native -mno-avx -Wno-unused-local-typedefs")
    set_property (TARGET lib_nnls PROPERTY COMPILE_FLAGS "-Wno-maybe-uninitialized")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
endif()

if (CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_definitions ("-Werror -Wall -Wno-error=c++11-narrowing -Wno-error=unused-variable")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -stdlib=libc++")
endif()
