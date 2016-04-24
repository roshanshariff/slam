//
//  range_bearing_model.cpp
//  slam
//
//  Created by Roshan Shariff on 2012-09-16.
//  Copyright (c) 2012 University of Alberta. All rights reserved.
//

#include "planar_robot/range_bearing_model.hpp"

namespace po = boost::program_options;

using namespace planar_robot;

auto range_bearing_model::builder::program_options()
    -> po::options_description {

  const double RAD_PER_DEG = boost::math::constants::pi<double>() / 180;

  po::options_description model_options("Range-Bearing Sensor Options");
  model_options.add_options() //
      ("sensor-range-stddev", po::value<double>()->default_value(0.1),
       "standard deviation of the range component of sensor noise, in m") //
      ("sensor-bearing-stddev",
       po::value<double>()->default_value(1.0 * RAD_PER_DEG),
       "standard deviation of the bearing component of sensor noise, in rad");

  return model_options;
}

range_bearing_model::builder::builder(const po::variables_map& options)
    : builder(options["sensor-range-stddev"].as<double>(),
              options["sensor-bearing-stddev"].as<double>()) {}

auto range_only_model::builder::program_options() -> po::options_description {

  po::options_description model_options("Range-Only Sensor Options");
  model_options.add_options() //
      ("sensor-range-stddev", po::value<double>()->default_value(0.1),
       "standard deviation of the range component of sensor noise, in m");

  return model_options;
}

range_only_model::builder::builder(const po::variables_map& options)
    : builder(options["sensor-range-stddev"].as<double>()) {}
