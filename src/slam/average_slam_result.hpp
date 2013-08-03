//
//  average_slam_result.h
//  slam
//
//  Created by Roshan Shariff on 2012-09-14.
//  Copyright (c) 2012 University of Alberta. All rights reserved.
//

#ifndef __slam__average_slam_result__
#define __slam__average_slam_result__

#include <memory>
#include <utility>
#include <vector>

#include <boost/range/adaptor/indirected.hpp>
#include <Eigen/Core>

#include "slam/interfaces.hpp"
#include "slam/slam_result_impl.hpp"
#include "utility/utility.hpp"

namespace slam {
    
    namespace average_slam_result_details {
        
        template <class T> class avg_acc {
            
            using vec_t = typename T::vector_type;
            
            vec_t base;
            vec_t sum_innov;
            unsigned int n = 0;
            
        public:
            
            avg_acc () {
                base.setZero();
                sum_innov.setZero();
            }
            
            void accumulate (const T& new_value) {
                if (n == 0) base = new_value.to_vector();
                else sum_innov += T::subtract (base, new_value.to_vector());
                ++n;
            }
            
            T get () const {
                assert (n > 0);
                return T::from_vector (T::subtract (base, sum_innov/n));
            }
            
        };
                
    }
    
    template <class State, class Feature, class Container>
    auto average_slam_result (const Container& results)
    -> std::unique_ptr<slam_result<State, Feature>> {
        
        assert (results.begin() != results.end());
        
        using namespace boost::adaptors;
        using average_slam_result_details::avg_acc;
        
        std::vector<avg_acc<State>> state_avgs;
        utility::flat_map<featureid_type, avg_acc<Feature>> feature_avgs;
        
        for (const auto& result : indirect(results)) {
            
            const auto& initial_state = result.get_initial_state();
            const auto& trajectory = result.get_trajectory();
            const auto& map = result.get_feature_map();
            
            if (state_avgs.size() < trajectory.size()) {
                state_avgs.resize (trajectory.size());
            }
            
            for (size_t t = 0; t < trajectory.size(); ++t) {
                state_avgs[t].accumulate (trajectory.accumulate (t+1));
            }
            
            for (const auto& feature : map) {
                feature_avgs[feature.first].accumulate (-initial_state + feature.second);
            }
        }
        
        auto result = utility::make_unique<slam_result_impl<State, Feature>>();
        
        // Calculate average trajectory
        
        auto& trajectory = result->get_trajectory();
        trajectory.reserve (state_avgs.size());
        
        for (const auto& state_avg : state_avgs) {
            trajectory.push_back_accumulated (state_avg.get());
        }
        
        // Calculate average map
        
        auto& map = result->get_feature_map();
        map.reserve (feature_avgs.size());
        
        for (const auto& feature_avg : feature_avgs) {
            map.emplace_hint (map.end(), feature_avg.first, feature_avg.second.get());
        }
        
        return std::move(result);
    }
    
}


#endif /* defined(__slam__average_slam_result__) */
