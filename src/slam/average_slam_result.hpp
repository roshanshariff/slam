//
//  average_slam_result.h
//  slam
//
//  Created by Roshan Shariff on 2012-09-14.
//  Copyright (c) 2012 University of Alberta. All rights reserved.
//

#ifndef __slam__average_slam_result__
#define __slam__average_slam_result__

#include <Eigen/Core>
#include <boost/shared_ptr.hpp>

#include "slam/interfaces.hpp"
#include "utility/bitree.hpp"
#include "utility/flat_map.hpp"
#include "utility/vector.hpp"


namespace slam {

    template <class State, class Feature>
    class average_slam_result : public slam_result<State, Feature> {
        
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
        
        using slam_result_type = slam_result<State, Feature>;
        
        timestep_type m_timestep;
        utility::bitree<State> m_trajectory;
        utility::flat_map<featureid_type, Feature> m_map;
        
    public:
        
        template <class ResultPtrIter>
        average_slam_result (ResultPtrIter results_begin, ResultPtrIter results_end);
        
        virtual void timestep (timestep_type t) override {
            assert (t <= current_timestep());
        }
        
        virtual auto current_timestep () const override -> timestep_type {
            return m_timestep;
        }
        
        virtual auto get_state (timestep_type t) const override -> State {
            return m_trajectory.accumulate (t);
        }
        
        virtual auto get_feature (featureid_type id) const override -> Feature {
            return m_map.at(id);
        }
        
        virtual auto get_trajectory () const override -> const decltype(m_trajectory)& {
            return m_trajectory;
        }
        
        virtual auto get_feature_map () const override -> const decltype(m_map)&  {
            return m_map;
        }
        
    };
    
}


template <class State, class Feature>
template <class ResultPtrIter>
slam::average_slam_result<State, Feature>
::average_slam_result (ResultPtrIter results_begin, ResultPtrIter results_end) {
    
    assert (results_begin != results_end);
    m_timestep = (*results_begin)->current_timestep();
    
    utility::vector<avg_acc<State>> state_avgs;
    utility::flat_map<featureid_type, avg_acc<Feature>> feature_avgs;
    
    for (auto iter = results_begin; iter != results_end; ++iter) {
        
        const auto& trajectory = (*iter)->get_trajectory();
        const auto& map = (*iter)->get_feature_map();
        
        if (state_avgs.size() < trajectory.size()) {
            state_avgs.resize (trajectory.size());
        }
        
        for (size_t t = 0; t < trajectory.size(); ++t) {
            state_avgs[t].accumulate (trajectory.accumulate (t+1));
        }

        for (const auto& feature : map) {
            feature_avgs[feature.first].accumulate (feature.second);
        }
    }
    
    // Calculate average trajectory
    
    m_trajectory.reserve (state_avgs.size());
    
    State state;
    for (const auto& state_avg : state_avgs) {
        State next_state = state_avg.get();
        m_trajectory.push_back (-state + next_state);
        state = next_state;
    }

    // Calculate average map
        
    m_map.reserve (feature_avgs.size());
    
    for (const auto& feature_avg : feature_avgs) {
        m_map.emplace_hint (m_map.end(), feature_avg.first, feature_avg.second.get());
    }
}



#endif /* defined(__slam__average_slam_result__) */
