//
//  options.hpp
//  slam
//
//  Created by Roshan Shariff on 12-01-20.
//  Copyright (c) 2012 University of Alberta. All rights reserved.
//

#ifndef slam_options_hpp
#define slam_options_hpp

#include <string>
#include <utility>

#include <boost/program_options.hpp>

template <class T>
T remember_option (boost::program_options::variables_map& options, const std::string& key, T default_value) {
    if (options[key].empty()) {
        options.insert (std::make_pair (key, boost::program_options::variable_value (default_value, false)));
        return default_value;
    }
    else return options[key].as<T>();
}

#endif
