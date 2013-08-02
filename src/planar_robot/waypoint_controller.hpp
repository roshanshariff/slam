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
#include <algorithm>

#include <boost/program_options.hpp>
#include <boost/math/constants/constants.hpp>

#include "planar_robot/pose.hpp"
#include "planar_robot/position.hpp"
#include "planar_robot/velocity_model.hpp"

#include "main.hpp"


namespace planar_robot {
    
    class waypoint_controller {
        
    public:
        
        waypoint_controller (const boost::program_options::variables_map&);
        
        static auto program_options () -> boost::program_options::options_description;
        
        template <class ControlModel>
        auto control (const pose& state, double dt) -> typename ControlModel::vector_type;
        
    private:
        
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
        
    };
    
} // namespace planar_robot


template <class ControlModel>
auto planar_robot::waypoint_controller
::control (const pose& state, const double dt) -> typename ControlModel::vector_type {
    
    {
        position target;
        while ((target = -state+current_waypoint()).distance() <= proximity) {
            next_waypoint();
        }
        
        const double target_direction = target.direction();
        
        const double alpha = dt / std::max(dt, smoothing);
        direction_deriv *= 1 - alpha;
        direction_deriv += alpha * (target_direction - direction)/dt;
        
        direction = target_direction;
    }
    
    double steering_change = param_P*direction/dt + param_D*direction_deriv;
    
    if (std::abs(steering_change) > max_steering_rate*dt) {
        steering_change = (steering_change < 0 ? -1 : 1) * max_steering_rate * dt;
    }
    
    double old_steering = steering;
    steering += steering_change;
    
    if (std::abs(steering) > max_steering) {
        steering = (steering < 0 ? -1 : 1) * max_steering;
    }
    
    return ControlModel::from_steering (speed, (old_steering+steering)/2);
}


#endif //_PLANAR_ROBOT_WAYPOINT_CONTROLLER_HPP
