/*
 * waypoint_controller.cpp
 *
 *  Created on: 2011-08-26
 *      Author: roshan
 */

#include <string>
#include <fstream>

#include <boost/math/constants/constants.hpp>

#include "planar_robot/landmark_sensor.hpp"

namespace po = boost::program_options;

using namespace planar_robot;

po::options_description landmark_sensor::program_options () {

	const double RAD_PER_DEG = boost::math::constants::pi<double>() / 180;

	po::options_description sensor_options ("Range-Bearing Sensor Options");
	sensor_options.add_options()
	("sensor-range-max", po::value<double>()->default_value(30),
		"maximum sensor range, in m")
	("sensor-range-stddev", po::value<double>()->default_value(0.1),
		"standard deviation of the range component of sensor noise, in m")
	("sensor-bearing-stddev", po::value<double>()->default_value(1.0*RAD_PER_DEG),
		"standard deviation of the bearing component of sensor noise, in rad")
	("landmark-file", po::value<std::string>(),
		"filename of landmarks file");

	return sensor_options;
}


landmark_sensor::landmark_sensor (const po::variables_map& options)
: max_range (options["sensor-range-max"].as<double>()),
hits (0),
model_builder (range_bearing_model::builder (options["sensor-range-stddev"].as<double>(),
                                             options["sensor-bearing-stddev"].as<double>()))
{
    if (options.count("landmark-file")) {
		double x, y;
		std::ifstream landmark_file (options["landmark-file"].as<std::string>().c_str());
		while (landmark_file >> x >> y) landmarks.push_back (position::cartesian (x, y));
	}
}
