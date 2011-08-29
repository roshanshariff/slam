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


struct waypoint_controller {

	typedef vector_model_adapter<velocity_model> model_type;

	static boost::program_options::options_description program_options ();
	static waypoint_controller parse_options (const boost::program_options::variables_map&);

	model_type control (const pose& state);

private:

	const double speed, steering_max, steering_rate;

	const std::vector<position> waypoints;
	const double proximity, repetitions;

	const model_type::builder model_builder;

	double current_steering;
	size_t current_waypoint;

	waypoint_controller (double speed_, double steering_max_, double steering_rate_,
			const std::vector<position>& waypoints_, double proximity_, double repetitions_,
			const velocity_model::builder& velocity_model_builder_)
	: speed(speed_), steering_max(steering_max_), steering_rate(steering_rate_),
	  waypoints(waypoints_), proximity(proximity_), repetitions(repetitions_),
	  model_builder(velocity_model_builder_), current_steering(0.0), current_waypoint(0) { }

public:

	pose initial_state () const { return pose::from_position (waypoints.front(), 0.0); }
	bool finished () const { return current_waypoint >= waypoints.size() * repetitions; }

};


} // namespace planar_robot

#endif //_PLANAR_ROBOT_WAYPOINT_CONTROLLER_HPP
