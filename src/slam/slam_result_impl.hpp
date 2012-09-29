//
//  slam_result_impl.hpp
//  slam
//
//  Created by Roshan Shariff on 2012-09-28.
//  Copyright (c) 2012 University of Alberta. All rights reserved.
//

#ifndef slam_slam_result_impl_hpp
#define slam_slam_result_impl_hpp

#include "slam/interfaces.hpp"
#include "utility/bitree.hpp"
#include "utility/flat_map.hpp"


namespace slam {
    
    template <class State, class Feature>
    class slam_result_impl : public slam_result<State, Feature> {
        
        utility::bitree<State> m_trajectory;
        utility::flat_map<featureid_type, Feature> m_map;

    public:
        
        using slam_result_type = slam_result<State, Feature>;
        
        slam_result_impl () = default;
        
        explicit slam_result_impl (const slam_result_type& o)
        : m_trajectory (o.get_trajectory()), m_map (o.get_feature_map()) { }
        
        auto operator= (const slam_result_type& o) -> slam_result_impl& {
            m_trajectory = o.get_trajectory();
            m_map = o.get_feature_map();
            return *this;
        }
        
        virtual void timestep (timestep_type t) override {
            assert (t <= current_timestep());
        }
        
        virtual auto current_timestep () const override -> timestep_type {
            return timestep_type (m_trajectory.size());
        }
        
        virtual auto get_state (timestep_type t) const override -> State {
            return m_trajectory.accumulate (t);
        }
        
        virtual auto get_feature (featureid_type id) const override -> Feature {
            return m_map.at (id);
        }
        
        virtual auto get_trajectory () const override -> const decltype(m_trajectory)& {
            return m_trajectory;
        }
        
        virtual auto get_feature_map () const override -> const decltype(m_map)& {
            return m_map;
        }
        
        auto get_trajectory () -> decltype(m_trajectory)& {
            return m_trajectory;
        }
        
        auto get_feature_map () -> decltype(m_map)& {
            return m_map;
        }
        
    };

}

#endif
