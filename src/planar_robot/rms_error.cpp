//
//  mean_sq_error.cpp
//  slam
//
//  Created by Roshan Shariff on 2012-09-06.
//  Copyright (c) 2012 University of Alberta. All rights reserved.
//

#include <cassert>
#include <cmath>
#include <algorithm>

#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/stats.hpp>
#include <boost/accumulators/statistics/mean.hpp>

#include "planar_robot/rms_error.hpp"
#include "utility/bitree.hpp"
#include "utility/flat_map.hpp"

double planar_robot::trajectory_rmse (const planar_trajectory& a, const planar_trajectory& b) {

    assert (a.size() == b.size());
    
    using namespace boost::accumulators;
    accumulator_set<double, stats<tag::mean>> acc;
    
    pose a_pos, b_pos;
    
    for (size_t i = 0; i < a.size(); ++i) {
        a_pos += a[i];
        b_pos += b[i];
        acc ((-a_pos + b_pos).distance_squared());
    }

    return std::sqrt (mean (acc));
}


double planar_robot::map_rmse (const planar_map& landmarks, const planar_map& estimates) {
    
    using namespace boost::accumulators;
    accumulator_set<double, stats<tag::mean>> acc;
    
    auto landmark = landmarks.cbegin();
    
    for (auto estimate = estimates.cbegin(); estimate != estimates.cend(); ++estimate) {
        
        while (landmark != landmarks.cend() && landmark->first != estimate->first) {
            ++landmark;
        }
        
        assert (landmark != landmarks.cend());
        
        acc ((estimate->second.to_vector() - landmark->second.to_vector()).squaredNorm());
    }
    
    return std::sqrt (mean (acc));
}
