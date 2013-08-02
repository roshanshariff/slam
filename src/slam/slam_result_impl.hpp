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
    class slam_result_impl : public virtual slam_result<State, Feature> {
        
        using slam_result_type = slam_result<State, Feature>;

    public:
        using typename slam_result_type::state_type;
        using typename slam_result_type::feature_type;
        using typename slam_result_type::trajectory_type;
        using typename slam_result_type::feature_map_type;
        
    private:
        state_type m_initial_state;
        trajectory_type m_trajectory;
        feature_map_type m_map;

    public:
        
        slam_result_impl () { };
        
        explicit slam_result_impl (const slam_result_type& o)
        : m_initial_state(o.get_initial_state()), m_trajectory(o.get_trajectory()),
        m_map(o.get_feature_map()) { }
        
        auto operator= (const slam_result_type& o) -> slam_result_impl& {
            m_initial_state = o.get_initial_state();
            m_trajectory = o.get_trajectory();
            m_map = o.get_feature_map();
            return *this;
        }
        
        /** Member functions from timestep_listener */
        
        virtual void timestep (timestep_type t) override {
            assert (t <= current_timestep());
        }
        
        /** Member functions from data_source */
        
        virtual auto current_timestep () const -> timestep_type override {
            return timestep_type (m_trajectory.size());
        }
        
        /** Member functions from slam_result */
        
        virtual auto get_initial_state () const -> state_type override {
            return m_initial_state;
        }
        
        virtual auto get_state (timestep_type t) const -> state_type override {
            return get_initial_state() + m_trajectory.accumulate (t);
        }
        
        virtual auto get_feature (featureid_type id) const -> feature_type override {
            return m_map.at (id);
        }
        
        virtual auto get_trajectory () const -> const trajectory_type& override {
            return m_trajectory;
        }
        
        virtual auto get_feature_map () const -> const feature_map_type& override {
            return m_map;
        }
        
        void set_initial_state (const state_type& state) {
            m_initial_state = state;
        }
        
        auto get_trajectory () -> trajectory_type& {
            return m_trajectory;
        }
        
        auto get_feature_map () -> feature_map_type& {
            return m_map;
        }
        
    };

    
    template <class ControlModel, class ObsModel>
    using slam_result_of_impl = slam_result_impl<typename ControlModel::associated_type, typename ObsModel::associated_type>;
    
}

#endif
