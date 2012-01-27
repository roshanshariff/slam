//
//  slam_result.hpp
//  slam
//
//  Created by Roshan Shariff on 12-01-26.
//  Copyright (c) 2012 University of Alberta. All rights reserved.
//

#ifndef slam_slam_result_hpp
#define slam_slam_result_hpp

#include <boost/shared_ptr.hpp>
#include <boost/container/flat_map.hpp>

#include "utility/bitree.hpp"

template <class State, class Feature>
class slam_result {
    
    typedef bitree<State> trajectory_type;
    typedef boost::container::flat_map<size_t, Feature> map_estimate_type;

public:
    
    virtual State state_estimate () const = 0;
    
    virtual boost::shared_ptr<const trajectory_type> trajectory_estimate () const = 0;
    
    virtual boost::shared_ptr<const map_estimate_type> map_estimate () const = 0;
    
};

#endif
