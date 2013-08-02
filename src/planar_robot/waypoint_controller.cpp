/*
 * waypoint_controller.cpp
 *
 *  Created on: 2011-08-26
 *      Author: roshan
 */

#include "planar_robot/waypoint_controller.hpp"


auto planar_robot::waypoint_controller
::program_options () -> boost::program_options::options_description {
    
    namespace po = boost::program_options;
    
    const double RAD_PER_DEG = boost::math::constants::pi<double>() / 180;
    
    po::options_description controller_options ("Waypoint Controller Options");
    
    po::options_description robot_options ("Robot Parameters");
    robot_options.add_options()
    ("robot-speed", po::value<double>()->default_value(3.0),
     "speed of the robot, in meter/second")
    ("robot-max-steering", po::value<double>()->default_value(30.0*RAD_PER_DEG),
     "maximum steering angle, in radian/second")
    ("robot-max-steering-rate", po::value<double>()->default_value(20.0*RAD_PER_DEG),
     "maximum change in steering, in radian/second^2");
    controller_options.add (robot_options);
    
    po::options_description pid_options ("PID Controller Parameters");
    pid_options.add_options()
    ("controller-paramP", po::value<double>()->default_value(0.5))
    ("controller-paramD", po::value<double>()->default_value(2.0))
    ("controller-smoothing", po::value<double>()->default_value(0.5));
    controller_options.add (pid_options);
    
    po::options_description waypoint_options ("Waypoints");
    waypoint_options.add_options()
    ("waypoint-file", po::value<std::string>(),
     "filename of waypoints file")
    ("waypoint-proximity", po::value<double>()->default_value(1.0),
     "distance at which waypoint is considered reached, in m")
    ("waypoint-repetitions", po::value<double>()->default_value(1.0),
     "number of times to traverse waypoints");
    controller_options.add (waypoint_options);
    
    return controller_options;
}


planar_robot::waypoint_controller
::waypoint_controller (const boost::program_options::variables_map& options)
: speed           (options["robot-speed"].as<double>()),
max_steering      (options["robot-max-steering"].as<double>()),
max_steering_rate (options["robot-max-steering-rate"].as<double>()),
proximity         (options["waypoint-proximity"].as<double>()),
repetitions       (options["waypoint-repetitions"].as<double>()),
param_P           (options["controller-paramP"].as<double>()),
param_D           (options["controller-paramD"].as<double>()),
smoothing         (options["controller-smoothing"].as<double>())
{
    if (options.count("waypoint-file")) {
        double x, y;
        std::ifstream waypoint_file (options["waypoint-file"].as<std::string>().c_str());
        while (waypoint_file >> x >> y) waypoints.push_back (position::cartesian (x, y));
    }
}
