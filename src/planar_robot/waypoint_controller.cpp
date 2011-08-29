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

#include <boost/math/constants/constants.hpp>

#include "planar_robot/waypoint_controller.hpp"

namespace po = boost::program_options;

using namespace planar_robot;

po::options_description waypoint_controller::program_options () {

	const double RAD_PER_DEG = boost::math::constants::pi<double>() / 180;

	po::options_description robot_options ("Robot Parameters");
	robot_options.add_options()
	("time-delta", po::value<double>()->default_value(0.25),
		"time delta per update, in seconds")
	("robot-speed", po::value<double>()->default_value(3.0),
		"speed of the robot, in meter/second")
	("robot-steering-max", po::value<double>()->default_value(30.0),
		"maximum steering angle, in degrees")
	("robot-steering-rate", po::value<double>()->default_value(20.0),
		"maximum change in steering, in degree/second");

	po::options_description velmodel_options ("Velocity Motion Model Parameters");
	velmodel_options.add_options()
	("velmodel-alpha1", po::value<double>()->default_value(0.1))
	("velmodel-alpha2", po::value<double>()->default_value(1.0))
	("velmodel-alpha3", po::value<double>()->default_value(1.0*RAD_PER_DEG))
	("velmodel-alpha4", po::value<double>()->default_value(0.05*RAD_PER_DEG))
	("velmodel-alpha5", po::value<double>()->default_value(0.1*RAD_PER_DEG))
	("velmodel-alpha6", po::value<double>()->default_value(0.005*RAD_PER_DEG));

	po::options_description waypoint_options ("Waypoints");
	waypoint_options.add_options()
	("waypoint-file", po::value<std::string>(),
		"filename of waypoints file")
	("waypoint-proximity", po::value<double>()->default_value(1.0),
		"distance at which waypoint is considered reached, in m");
	("waypoint-repetitions", po::value<double>()->default_value(1.0),
		"number of times to traverse waypoints");

	po::options_description controller_options ("Waypoint Controller Options");
	controller_options.add(robot_options).add(velmodel_options).add(waypoint_options);
	return controller_options;
}


waypoint_controller waypoint_controller::parse_options (const po::variables_map& options) {

	const double RAD_PER_DEG = boost::math::constants::pi<double>() / 180;

	std::vector<position> waypoints;
	if (options.count("waypoint-file")) {
		double x, y;
		std::ifstream waypoint_file (options["waypoint-file"].as<std::string>().c_str());
		while (waypoint_file >> x >> y) waypoints.push_back (position::cartesian (x, y));
	}

	const double dt = options["time-delta"].as<double>();

	return waypoint_controller (
		options["robot-speed"].as<double>() * dt,
		options["robot-steering-max"].as<double>() * RAD_PER_DEG,
		options["robot-steering-rate"].as<double>() * dt * RAD_PER_DEG,
		waypoints,
		options["waypoint-proximity"].as<double>(),
		options["waypoint-repetitions"].as<double>(),
		velocity_model::builder (
			options["velmodel-alpha1"].as<double>(),
			options["velmodel-alpha2"].as<double>(),
			options["velmodel-alpha3"].as<double>(),
			options["velmodel-alpha4"].as<double>(),
			options["velmodel-alpha5"].as<double>(),
			options["velmodel-alpha6"].as<double>(),
			options["time-delta"].as<double>()
		)
	);
}


waypoint_controller::model_type waypoint_controller::control (const pose& state) {

	if (!waypoints.empty()) {

		position to_waypoint = -state + waypoints[current_waypoint % waypoints.size()];

		double steering_change = to_waypoint.direction() - current_steering;
		if (std::abs(steering_change) > steering_rate) {
			steering_change = steering_change < 0 ? -steering_rate : steering_rate;
		}

		current_steering += steering_change;
		if (std::abs(current_steering) > steering_max) {
			current_steering = current_steering < 0 ? -steering_max : steering_max;
		}

		if (to_waypoint.distance() < proximity) {
			++current_waypoint;
		}

	}

	return model_builder.vector_builder (model_type::vector_type (speed, current_steering, 0));
}
