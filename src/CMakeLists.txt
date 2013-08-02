cmake_minimum_required (VERSION 2.8.8)
project (slam CXX)

add_library (lib_planar_robot OBJECT
    planar_robot/waypoint_controller.cpp planar_robot/landmark_sensor.cpp
    planar_robot/velocity_model.cpp planar_robot/range_bearing_model.cpp
    planar_robot/rms_error.cpp)
    
add_library (lib_simulator OBJECT
    simulator/simulator.cpp simulator/gnuplot_process.cpp simulator/slam_plotter.cpp
    simulator/time_series_plotter.cpp)
    
add_library (lib_slam OBJECT
    slam/slam_data.cpp slam/mcmc_slam.cpp slam/multi_mcmc.cpp slam/g2o_slam.cpp
    slam/g2o_clustering.cpp
    #slam/fastslam.cpp slam/fastslam_mcmc.cpp
    )
    
add_library (lib_utility OBJECT
    utility/utility.cpp utility/cowtree.cpp)

add_executable (slam
    main.cpp $<TARGET_OBJECTS:lib_planar_robot> $<TARGET_OBJECTS:lib_simulator>
    $<TARGET_OBJECTS:lib_slam> $<TARGET_OBJECTS:lib_utility>)

add_executable (dataset
    dataset.cpp $<TARGET_OBJECTS:lib_planar_robot> $<TARGET_OBJECTS:lib_utility>)

set (CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${PROJECT_SOURCE_DIR}/cmake/Modules/")

set (CMAKE_INCLUDE_CURRENT_DIR true)

set (CMAKE_BUILD_TYPE Release)

find_package (Boost 1.50.0 COMPONENTS program_options filesystem system timer REQUIRED)
include_directories(${Boost_INCLUDE_DIRS})
target_link_libraries (slam ${Boost_LIBRARIES})
target_link_libraries (dataset ${Boost_LIBRARIES})

find_package (Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

find_package (CSparse REQUIRED)
include_directories(${CSPARSE_INCLUDE_DIR})
target_link_libraries(slam ${CSPARSE_LIBRARY})
target_link_libraries(dataset ${CSPARSE_LIBRARY})

find_package (G2O REQUIRED)
include_directories (${G2O_INCLUDE_DIR})
target_link_libraries (slam ${G2O_CORE_LIBRARY} ${G2O_STUFF_LIBRARY} ${G2O_SOLVER_CSPARSE} ${G2O_SOLVER_CSPARSE_EXTENSION})
target_link_libraries (dataset ${G2O_CORE_LIBRARY} ${G2O_STUFF_LIBRARY} ${G2O_SOLVER_CSPARSE} ${G2O_SOLVER_CSPARSE_EXTENSION})

if (CMAKE_COMPILER_IS_GNUCXX)
    add_definitions ("-std=c++11 -Werror -Wall -Wextra -march=native -mno-avx -Wno-unused-local-typedefs")
endif()

if (CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_definitions ("-std=c++11 -stdlib=libc++ -Werror -Wall -Wno-error=c++11-narrowing -Wno-error=unused-variable")
endif()
