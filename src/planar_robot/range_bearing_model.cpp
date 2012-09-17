//
//  range_bearing_model.cpp
//  slam
//
//  Created by Roshan Shariff on 2012-09-16.
//  Copyright (c) 2012 University of Alberta. All rights reserved.
//

#include "planar_robot/range_bearing_model.hpp"

#include <boost/math/constants/constants.hpp>

namespace po = boost::program_options;

using namespace planar_robot;


auto range_bearing_model::builder::program_options () -> po::options_description {
    
    const double RAD_PER_DEG = boost::math::constants::pi<double>() / 180;
    
    po::options_description model_options ("Range-Bearing Sensor Options");
    model_options.add_options()
    ("sensor-range-stddev", po::value<double>()->default_value(0.1),
     "standard deviation of the range component of sensor noise, in m")
    ("sensor-bearing-stddev", po::value<double>()->default_value(1.0*RAD_PER_DEG),
     "standard deviation of the bearing component of sensor noise, in rad");

    return model_options;
}


range_bearing_model::builder::builder (const po::variables_map& options)
: builder (options["sensor-range-stddev"].as<double>(),
           options["sensor-bearing-stddev"].as<double>())
{ }


auto range_only_model::builder::program_options () -> po::options_description {
    
    po::options_description model_options ("Range-Only Sensor Options");
    model_options.add_options()
    ("sensor-range-stddev", po::value<double>()->default_value(0.1),
     "standard deviation of the range component of sensor noise, in m");
    
    return model_options;
}


range_only_model::builder::builder (const po::variables_map& options)
: builder (options["sensor-range-stddev"].as<double>())
{ }


auto range_only_model::operator() (random_source& random) const -> vector_type {
    vector_type result;
    result(0) = stddev()(0)*random.normal() + mean()(0);
    result(1) = (2*random.uniform() - 1) * boost::math::constants::pi<double>();
    return result;
}


auto range_only_model::likelihood (const vector_type& x) const -> double {
    const double bearing_likelihood = 1 / (2 * boost::math::constants::pi<double>());
    const double root_two_pi = boost::math::constants::root_two_pi<double>();
    const double std_range = (x(0) - mean()(0)) / stddev()(0);
    return bearing_likelihood * std::exp(-0.5*std_range*std_range) / (root_two_pi*stddev()(0));
}


auto range_only_model::log_likelihood (const vector_type& x) const -> double {
    const double log_two_pi = std::log(2*boost::math::constants::pi<double>());
    const double std_range = (x(0) - mean()(0)) / stddev()(0);
    return -1.5*log_two_pi - 0.5*std_range*std_range - std::log(stddev()(0));
}

