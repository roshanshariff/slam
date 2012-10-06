/*
 * waypoint_controller.hpp
 *
 *  Created on: 2011-08-26
 *      Author: roshan
 */

#ifndef _PLANAR_ROBOT_WAYPOINT_CONTROLLER_HPP
#define _PLANAR_ROBOT_WAYPOINT_CONTROLLER_HPP

#include <vector>
#include <functional>
#include <cstddef>
#include <fstream>

#include <boost/program_options.hpp>
#include <boost/math/constants/constants.hpp>

#include "planar_robot/pose.hpp"
#include "planar_robot/position.hpp"
#include "planar_robot/velocity_model.hpp"

#include "main.hpp"


namespace planar_robot {
    
    template <class ControlModel> class waypoint_controller {
        
    public:
        
        using model_type = ControlModel;
        using model_builder_type = typename model_type::builder;
        
        waypoint_controller (const boost::program_options::variables_map&);
        
        static auto program_options () -> boost::program_options::options_description;
        
        model_type control (const pose& state);
        
    private:
        
        model_builder_type model_builder;
        
        std::vector<position> waypoints;
        double speed, max_steering, max_steering_rate, proximity, repetitions;
        double param_P, param_D, smoothing;
                
        double steering = 0;
        std::size_t waypoint = 0;
        
        double direction = 0;
        double direction_deriv = 0;
        
        void next_waypoint () {
            ++waypoint;
            direction = 0;
            direction_deriv = 0;
            std::cout << "Seeking waypoint " << (1 + waypoint%waypoints.size())
            << " of " << waypoints.size() << std::endl;
        }
        
        const position& current_waypoint () const {
            return waypoints[waypoint % waypoints.size()];
        }

    public:
        
        pose initial_state () const {
            return waypoints.empty() ? pose() : pose::from_position (waypoints.front(), 0.0);
        }
        
        bool finished () const {
            return waypoint >= waypoints.size() * repetitions;
        }
        
        double dt () const { return model_builder.dt(); }
        
    };
    
} // namespace planar_robot


template <class ControlModel>
auto planar_robot::waypoint_controller<ControlModel>
::control (const pose& state) -> model_type {
    
    {
        position target;
        while ((target = -state+current_waypoint()).distance() <= proximity) {
            next_waypoint();
        }
        
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
    
    return model_builder.from_steering (speed, (old_steering+steering)/2);
}


template <class ControlModel>
auto planar_robot::waypoint_controller<ControlModel>
::program_options () -> boost::program_options::options_description {
    
    namespace po = boost::program_options;
    
    const double RAD_PER_DEG = boost::math::constants::pi<double>() / 180;
    
    po::options_description controller_options ("Waypoint Controller Options");
    
    controller_options.add (model_builder_type::program_options());

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


template <class ControlModel>
planar_robot::waypoint_controller<ControlModel>
::waypoint_controller (const boost::program_options::variables_map& options)
: model_builder   (options, options["dt"].as<double>()),
speed             (options["robot-speed"].as<double>()),
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
    if (smoothing < dt()) smoothing = dt();
}


extern template class planar_robot::waypoint_controller<control_model_type>;

#endif //_PLANAR_ROBOT_WAYPOINT_CONTROLLER_HPP
