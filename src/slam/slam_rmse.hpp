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
            
            for (size_t i = 1; i <= t; ++i) {
                acc (State::subtract(at.accumulate(i).to_vector(), bt.accumulate(i).to_vector())
                     .squaredNorm());
            }
        }

        {
            auto iter = b.get_feature_map().begin();
            const auto end = b.get_feature_map().end();

            for (const auto& id_feature : a.get_feature_map()) {
                
                const auto id = id_feature.first;
                while (iter != end && iter->first < id) ++iter;
                if (iter == end) break;
                if (iter->first != id) continue;
                
                auto af = -a.get_initial_state() + id_feature.second;
                auto bf = -b.get_initial_state() + (*iter).second;
                acc (Feature::subtract (af.to_vector(), bf.to_vector()).squaredNorm());
            }
        }
        
        return std::sqrt (mean (acc));
    }
    
}

#endif
