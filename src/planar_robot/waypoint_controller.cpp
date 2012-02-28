/*
 * waypoint_controller.cpp
 *
 *  Created on: 2011-08-26
 *      Author: roshan
 */

#include <string>
#include <fstream>
#include <cstdlib>
#include <cassert>
#include <iostream>

#include <boost/math/constants/constants.hpp>

#include "planar_robot/waypoint_controller.hpp"


using namespace planar_robot;


auto waypoint_controller::control (const pose& state) -> model_type {
    
    {
        position target;
        while ((target = -state+current_waypoint()).distance() <= proximity) next_waypoint();
        
        const double target_direction = target.direction();
        
        const double alpha = dt() / smoothing;
        direction_deriv *= 1 - alpha;
        direction_deriv += alpha * (target_direction - direction)/dt();
        
        direction = target_direction;
    }
    
    double steering_change = param_P*direction/dt() + param_D*direction_deriv;
    
    if (std::abs(steering_change) > max_steering_rate*dt()) {
        steering_change = (steering_change < 0 ? -1 : 1) * max_steering_rate * dt();
    }
    
    double old_steering = steering;
    steering += steering_change;
    
    if (std::abs(steering) > max_steering) {
        steering = (steering < 0 ? -1 : 1) * max_steering;
    }
    
    return model_builder.vector_builder
    (model_type::vector_type (speed, (old_steering+steering)/2, 0));
}


auto waypoint_controller::program_options () -> boost::program_options::options_description {
    
    namespace po = boost::program_options;
    
    const double RAD_PER_DEG = boost::math::constants::pi<double>() / 180;
    
    po::options_description robot_options ("Robot Parameters");
    robot_options.add_options()
    ("dt", po::value<double>()->default_value(0.25),
     "time delta per update, in seconds")
    ("robot-speed", po::value<double>()->default_value(3.0),
     "speed of the robot, in meter/second")
    ("robot-max-steering", po::value<double>()->default_value(30.0*RAD_PER_DEG),
     "maximum steering angle, in radian/second")
    ("robot-max-steering-rate", po::value<double>()->default_value(20.0*RAD_PER_DEG),
     "maximum change in steering, in radian/second^2");
    
    po::options_description velmodel_options ("Velocity Motion Model Parameters");
    velmodel_options.add_options()
    ("velmodel-alpha1", po::value<double>()->default_value(0.1))
    ("velmodel-alpha2", po::value<double>()->default_value(0.0))
    ("velmodel-alpha3", po::value<double>()->default_value(1.0*RAD_PER_DEG))
    ("velmodel-alpha4", po::value<double>()->default_value(0.1))
    ("velmodel-alpha5", po::value<double>()->default_value(0.1*RAD_PER_DEG))
    ("velmodel-alpha6", po::value<double>()->default_value(0.01*RAD_PER_DEG));
    
    po::options_description pid_options ("PID Controller Parameters");
    pid_options.add_options()
    ("controller-paramP", po::value<double>()->default_value(0.5))
    ("controller-paramD", po::value<double>()->default_value(2.0))
    ("controller-smoothing", po::value<double>()->default_value(0.5));
    
    po::options_description waypoint_options ("Waypoints");
    waypoint_options.add_options()
    ("waypoint-file", po::value<std::string>(),
     "filename of waypoints file")
    ("waypoint-proximity", po::value<double>()->default_value(1.0),
     "distance at which waypoint is considered reached, in m")
    ("waypoint-repetitions", po::value<double>()->default_value(1.0),
     "number of times to traverse waypoints");
    
    po::options_description controller_options ("Waypoint Controller Options");
    controller_options.add(robot_options).add(velmodel_options).add(pid_options).add(waypoint_options);
    return controller_options;
}


waypoint_controller::waypoint_controller (const boost::program_options::variables_map& options)
: model_builder (velocity_model::builder (options["velmodel-alpha1"].as<double>(),
                                          options["velmodel-alpha2"].as<double>(),
                                          options["velmodel-alpha3"].as<double>(),
                                          options["velmodel-alpha4"].as<double>(),
                                          options["velmodel-alpha5"].as<double>(),
                                          options["velmodel-alpha6"].as<double>(),
                                          options["dt"].as<double>())),
speed               (options["robot-speed"].as<double>()),
max_steering        (options["robot-max-steering"].as<double>()),
max_steering_rate   (options["robot-max-steering-rate"].as<double>()),
proximity           (options["waypoint-proximity"].as<double>()),
repetitions         (options["waypoint-repetitions"].as<double>()),
param_P             (options["controller-paramP"].as<double>()),
param_D             (options["controller-paramD"].as<double>()),
smoothing           (options["controller-smoothing"].as<double>())
{
    if (options.count("waypoint-file")) {
        double x, y;
        std::ifstream waypoint_file (options["waypoint-file"].as<std::string>().c_str());
        while (waypoint_file >> x >> y) waypoints.push_back (position::cartesian (x, y));
    }
    if (smoothing < dt()) smoothing = dt();
}


