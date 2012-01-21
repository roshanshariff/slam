#ifndef _PLANAR_ROBOT_WAYPOINT_CONTROLLER_HPP
#define _PLANAR_ROBOT_WAYPOINT_CONTROLLER_HPP

#include <vector>
#include <functional>

#include <boost/program_options.hpp>

#include "planar_robot/pose.hpp"
#include "planar_robot/position.hpp"
#include "planar_robot/velocity_model.hpp"
#include "slam/vector_model.hpp"

namespace planar_robot {
    
    
    class waypoint_controller {
        
    public:
        
        typedef vector_model_adapter<velocity_model> model_type;
        
        waypoint_controller (const boost::program_options::variables_map&);
        
        static boost::program_options::options_description program_options ();
        
        model_type control (const pose& state);
        
    private:
        
        model_type::builder model_builder;
        std::vector<position> waypoints;
        double speed, steering_max, steering_rate, proximity, repetitions;
        
        double current_steering;
        size_t current_waypoint;
        
    public:
        
        pose initial_state () const {
            return waypoints.empty() ? pose() : pose::from_position (waypoints.front(), 0.0);
        }
        
        bool finished () const {
            return current_waypoint >= waypoints.size() * repetitions;
        }
        
    };
    
    
} // namespace planar_robot

#endif //_PLANAR_ROBOT_WAYPOINT_CONTROLLER_HPP
