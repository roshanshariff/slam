//
//  velocity_model.cpp
//  slam
//
//  Created by Roshan Shariff on 2012-10-04.
//  Copyright (c) 2012 University of Alberta. All rights reserved.
//

#include "planar_robot/velocity_model.hpp"

namespace po = boost::program_options;

using namespace planar_robot;


auto velocity_slip_model::builder::program_options () -> po::options_description {
    
    const double RAD_PER_DEG = boost::math::constants::pi<double>() / 180;
    
    po::options_description model_options ("Velocity-Slip Motion Model Parameters");
    model_options.add_options()
    ("velmodel-alpha1", po::value<double>()->default_value(0.1))
    ("velmodel-alpha2", po::value<double>()->default_value(0.0))
    ("velmodel-alpha3", po::value<double>()->default_value(1.0*RAD_PER_DEG))
    ("velmodel-alpha4", po::value<double>()->default_value(0.1))
    ("velmodel-alpha5", po::value<double>()->default_value(0.1*RAD_PER_DEG))
    ("velmodel-alpha6", po::value<double>()->default_value(0.01*RAD_PER_DEG));
    
    return model_options;
}


velocity_slip_model::builder::builder (const po::variables_map& options, double dt)
: builder (options["velmodel-alpha1"].as<double>(),
           options["velmodel-alpha2"].as<double>(),
           options["velmodel-alpha3"].as<double>(),
           options["velmodel-alpha4"].as<double>(),
           options["velmodel-alpha5"].as<double>(),
           options["velmodel-alpha6"].as<double>(),
           dt)
{ }


auto velocity_model::builder::program_options () -> po::options_description {
    
    const double RAD_PER_DEG = boost::math::constants::pi<double>() / 180;
    
    po::options_description model_options ("Velocity Motion Model Parameters");
    model_options.add_options()
    ("velmodel-alpha1", po::value<double>()->default_value(0.1))
    ("velmodel-alpha2", po::value<double>()->default_value(0.0))
    ("velmodel-alpha3", po::value<double>()->default_value(1.0*RAD_PER_DEG))
    ("velmodel-alpha4", po::value<double>()->default_value(0.1));

    return model_options;
}


velocity_model::builder::builder (const po::variables_map& options, double dt)
: builder (options["velmodel-alpha1"].as<double>(),
           options["velmodel-alpha2"].as<double>(),
           options["velmodel-alpha3"].as<double>(),
           options["velmodel-alpha4"].as<double>(),
           dt)
{ }
