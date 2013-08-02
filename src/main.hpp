//
//  main.hpp
//  slam
//
//  Created by Roshan Shariff on 2012-09-09.
//  Copyright (c) 2012 University of Alberta. All rights reserved.
//

#ifndef slam_main_hpp
#define slam_main_hpp

namespace planar_robot {
    struct range_bearing_model;
    struct range_only_model;
    struct velocity_slip_model;
    struct velocity_model;
    class waypoint_controller;
    class landmark_sensor;
}

using control_model_type = planar_robot::velocity_model;
using observation_model_type = planar_robot::range_only_model;

using controller_type = planar_robot::waypoint_controller;
using sensor_type = planar_robot::landmark_sensor;

#include "planar_robot/waypoint_controller.hpp"
#include "planar_robot/landmark_sensor.hpp"

#endif
