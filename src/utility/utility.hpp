//
//  utility.hpp
//  slam
//
//  Created by Roshan Shariff on 11-09-04.
//  Copyright 2011 University of Alberta. All rights reserved.
//

#ifndef slam_utility_hpp
#define slam_utility_hpp

#include <cstdio>
#include <string>
#include <utility>

#include <boost/shared_ptr.hpp>
#include <boost/program_options.hpp>


boost::shared_ptr<FILE> open_file (const char* filename, const char* mode);

boost::shared_ptr<FILE> open_process (const char* command, const char* mode);


template <class T>
T remember_option (boost::program_options::variables_map& options, const std::string& key, T default_value) {
    if (options[key].empty()) {
        options.insert (std::make_pair (key, boost::program_options::variable_value (default_value, false)));
        return default_value;
    }
    else return options[key].as<T>();
}


template <class Iter>
struct iter_pair_range : std::pair<Iter,Iter> {
    iter_pair_range (const std::pair<Iter,Iter>& x) : std::pair<Iter,Iter>(x) { }
    iter_pair_range (Iter begin, Iter end) : std::pair<Iter,Iter>(begin, end) { }
    Iter begin() const { return this->first; }
    Iter end() const { return this->second; }
};


template <class Iter>
inline iter_pair_range<Iter> as_range (std::pair<Iter,Iter> const& range) { return range; }

template <class Iter>
inline iter_pair_range<Iter> as_range (Iter begin, Iter end) { return iter_pair_range<Iter>(begin, end); }


#endif
