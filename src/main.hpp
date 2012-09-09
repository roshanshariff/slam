//
//  main.hpp
//  slam
//
//  Created by Roshan Shariff on 2012-09-09.
//  Copyright (c) 2012 University of Alberta. All rights reserved.
//

#ifndef slam_main_hpp
#define slam_main_hpp

#include "planar_robot/waypoint_controller.hpp"
#include "planar_robot/landmark_sensor.hpp"

using controller_type = planar_robot::waypoint_controller;
using sensor_type = planar_robot::landmark_sensor;

using control_model_type = controller_type::model_type;
using observation_model_type = sensor_type::model_type;

#endif
