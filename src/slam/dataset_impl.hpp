//
//  dataset_impl.hpp
//  slam
//
//  Created by Roshan Shariff on 2012-10-19.
//  Copyright (c) 2012 University of Alberta. All rights reserved.
//

#ifndef slam_dataset_impl_hpp
#define slam_dataset_impl_hpp

#include <vector>

#include "slam/interfaces.hpp"
#include "utility/bitree.hpp"
#include "utility/flat_map.hpp"

namespace slam {
    
    
    template <class ControlModel, class ObservationModel>
    class dataset_impl : public dataset<ControlModel, ObservationModel> {
        
        using dataset_type = dataset<ControlModel, ObservationModel>;
        
    public:
        using typename dataset_type::control_type;
        using typename dataset_type::observation_type;
        using typename dataset_type::observation_info;
        using typename dataset_type::observation_collection;
        using typename dataset_type::observation_range;
        
    private:
        std::vector<control_type> m_controls;
        utility::bitree<double> m_timedeltas;
        observation_collection m_observations;
        
    public:
        
        /** Member functions from dataset */
        
        virtual auto current_timestep () const -> timestep_type override {
            return timestep_type (m_controls.size());
        }
        
        virtual auto control (timestep_type t) const -> const control_type& override {
            return m_controls.at(t);
        }
        
        virtual auto timedelta (timestep_type t) const -> double override {
            return m_timedeltas[t];
        }
        
        virtual auto timestamp (timestep_type t) const -> double override {
            return m_timedeltas.accumulate(t);
        }
        
        virtual auto timestep_at (double timestamp) const -> timestep_type override {
            return timestep_type (m_timedeltas.binary_search (timestamp));
        }

        virtual auto observations () const -> observation_range override {
            return m_observations;
        }
        
        virtual auto observations_at (timestep_type t) const -> observation_range override {
            return m_observations.equal_range(t);
        }
        
        /** Add data to dataset */
        
        void add_control (double dt, const control_type& control) {
            m_timedeltas.push_back (dt);
            m_controls.push_back (control);
        }
        
        void add_observation (timestep_type t, featureid_type id, observation_type obs) {
            m_observations.emplace (t, observation_info {id, obs});
        }
        
        void add_observation_now (featureid_type id, observation_type obs) {
            m_observations.emplace_hint (m_observations.cend(),
                                         current_timestep(), observation_info{id, obs});
        }
        
    };


} // namespace slam

#endif
