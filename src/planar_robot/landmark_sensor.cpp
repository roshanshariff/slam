/*
 * waypoint_controller.cpp
 *
 *  Created on: 2011-08-26
 *      Author: roshan
 */

#include "planar_robot/landmark_sensor.hpp"

planar_robot::landmark_sensor::landmark_sensor(
    const boost::program_options::variables_map& options)
    : max_range(options["sensor-range-max"].as<double>()),
      min_range(options["sensor-range-min"].as<double>()) {
  if (options.count("landmark-file")) {
    double x, y;
    std::ifstream landmark_file(
        options["landmark-file"].as<std::string>().c_str());
    while (landmark_file >> x >> y)
      landmarks.push_back(position::cartesian(x, y));
  }
}

auto planar_robot::landmark_sensor::program_options()
    -> boost::program_options::options_description {

  namespace po = boost::program_options;
  po::options_description sensor_options("Landmark Sensor Options");

  sensor_options.add_options() //
      ("sensor-range-max", po::value<double>()->default_value(30),
       "maximum sensor range, in m") //
      ("sensor-range-min", po::value<double>()->default_value(1),
       "minimum sensor range, in m") //
      ("landmark-file", po::value<std::string>(), "filename of landmarks file");

  return sensor_options;
}
