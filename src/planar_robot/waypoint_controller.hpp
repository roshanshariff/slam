#ifndef _PLANAR_ROBOT_WAYPOINT_CONTROLLER_HPP
#define _PLANAR_ROBOT_WAYPOINT_CONTROLLER_HPP

#include <vector>
#include <functional>
#include <cstddef>

#include <boost/program_options.hpp>

#include "planar_robot/pose.hpp"
#include "planar_robot/position.hpp"
#include "planar_robot/velocity_model.hpp"
#include "slam/vector_model.hpp"

namespace planar_robot {
    
    
    class waypoint_controller {
        
    public:
        
        using model_type = velocity_model;
        
        waypoint_controller (const boost::program_options::variables_map&);
        
        static boost::program_options::options_description program_options ();
        
        model_type control (const pose& state);
        
    private:
        
        model_type::builder model_builder;
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

#endif //_PLANAR_ROBOT_WAYPOINT_CONTROLLER_HPP
