//
//  slam_rmse.hpp
//  slam
//
//  Created by Roshan Shariff on 2012-09-28.
//  Copyright (c) 2012 University of Alberta. All rights reserved.
//

#ifndef slam_slam_rmse_hpp
#define slam_slam_rmse_hpp

#include <cmath>
#include <utility>

#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/stats.hpp>
#include <boost/accumulators/statistics/mean.hpp>

#include "slam/interfaces.hpp"

namespace slam {
    
    template <class State, class Feature>
    auto rms_error (const slam_result<State, Feature>& a, const slam_result<State, Feature>& b) -> double {
    
        using namespace boost::accumulators;
        accumulator_set<double, stats<tag::mean>> acc;
        
        {
            const auto& at = a.get_trajectory();
            const auto& bt = b.get_trajectory();
            auto t = std::min (at.size(), bt.size());
            
            State a_state, b_state;
            
            for (size_t i = 0; i < t; ++i) {
                a_state += at[i];
                b_state += bt[i];
                acc (State::subtract(a_state.to_vector(), b_state.to_vector()).squaredNorm());
            }
        }
        
        auto iter = b.get_feature_map().cbegin();
        const auto end = b.get_feature_map().cend();
        
        for (const auto& feature : a.get_feature_map()) {
            
            while (iter != end && iter->first < feature.first) ++iter;
            if (iter == end) break;
            if (iter->first != feature.first) continue;
            
            acc (Feature::subtract (feature.second.to_vector(), iter->second.to_vector()).squaredNorm());
        }
        
        return std::sqrt (mean (acc));
    }
    
}

#endif
