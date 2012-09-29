#ifndef _SLAM_SLAM_DATA_HPP
#define _SLAM_SLAM_DATA_HPP

#include <functional>
#include <utility>
#include <cassert>
#include <memory>

#include "slam/interfaces.hpp"
#include "utility/vector.hpp"
#include "utility/map.hpp"
#include "utility/flat_map.hpp"
#include "utility/listeners.hpp"
#include "utility/utility.hpp"

#include "main.hpp"


namespace slam {
    
    
    /** This class stores a record of all state changes and observations, as probability distributions.
     It is also responsible for notifying listeners when new state changes and observations are added.
     ActionModel and ObservationModel are the types of distributions over state changes and
     observations, respectively. */
    template <class ControlModel, class ObservationModel>
    class slam_data : public data_source {
        
    public:
        using feature_observations = utility::flat_map<timestep_type, ObservationModel>;

    private:
        using feature_collection = utility::map<featureid_type, feature_observations>;
        
    public:
        using feature_iterator = typename feature_collection::const_iterator;
        
        class observation_info {

            feature_iterator f;
            std::size_t i;

        public:

            observation_info (feature_iterator f, std::size_t i) : f(f), i(i) { }

            auto iterator () const -> feature_iterator { return f; }
            auto index () const -> std::size_t { return i; }
            
            auto id () const -> featureid_type { return f->first; }
            auto observation () const -> const ObservationModel& { return (f->second.begin()+i)->second; }
        };
        
        struct listener : public timestep_listener {
            virtual void control (timestep_type, const ControlModel&) = 0;
            virtual void observation (timestep_type, const observation_info&) = 0;
        };
        
    private:

        using observation_collection = utility::flat_multimap<timestep_type, observation_info>;
        using observation_range = iter_pair_range<typename observation_collection::const_iterator>;

        feature_collection m_features;
        observation_collection m_observations;
        
        utility::vector<ControlModel> m_controls;
        utility::listeners<listener> m_listeners;
        
    public:
        
        slam_data () = default;
        
        /** Noncopyable */
        slam_data (const slam_data&) = delete;
        slam_data& operator= (const slam_data&) = delete;
        
        /** Member functions from data_source */
        
        virtual timestep_type current_timestep () const override {
            return timestep_type (m_controls.size());
        }
        
        virtual void timestep (timestep_type timestep) override {
            assert (timestep == current_timestep());
            using namespace std::placeholders;
            m_listeners.for_each (std::bind (&listener::timestep, _1, timestep));
        }
        
        virtual void completed () override {
            using namespace std::placeholders;
            m_listeners.for_each (std::bind (&listener::completed, _1));
        }
        
        /** Retrieve controls. */
        
        const ControlModel& control (timestep_type timestep) const {
            return m_controls.at(timestep);
        }
        
        /** Retrieve observations. */
        
        observation_range observations () const {
            return observation_range (m_observations.begin(), m_observations.end());
        }
        
        observation_range observations_at (timestep_type t) const {
            // TODO: Use equal_range when boost bug is fixed
            return observation_range (m_observations.lower_bound(t), m_observations.upper_bound(t));
        }
        
        /** Retrieve features. */
        
        bool feature_observed (featureid_type f) const { return m_features.find(f) != m_features.end(); }
        
        feature_iterator get_feature_iterator (featureid_type featureid) const {
            return m_features.find(featureid);
        }
        
        const feature_observations& get_observations (featureid_type featureid) const {
            return m_features.at(featureid);
        }
        
        /** Add new data */
        
        void add_control (const ControlModel&);
        void add_observation (featureid_type, const ObservationModel&);
        
        /** Add new listener */
        
        void add_listener (const std::shared_ptr<listener>& l) { m_listeners.add(l); }
        
    };
    
} // namespace slam


template <class ControlModel, class ObservationModel>
void slam::slam_data<ControlModel, ObservationModel>
::add_control (const ControlModel& control) {
    
    timestep_type t = current_timestep();
    m_controls.push_back (control);

    using namespace std::placeholders;
    m_listeners.for_each (std::bind (&listener::control, _1, t, std::cref(control)));
}


template <class ControlModel, class ObservationModel>
void slam::slam_data<ControlModel, ObservationModel>
::add_observation (featureid_type id, const ObservationModel& obs) {
    
    const timestep_type t = current_timestep();
    
    auto feature_iter = m_features.find (id);
    
    if (feature_iter == m_features.end()) {
        feature_iter = m_features.emplace_hint (feature_iter, id, feature_observations());
    }
    
    auto& feature_obs = feature_iter->second;
    
    auto obs_iter = feature_obs.emplace_hint (feature_obs.end(), t, obs);
    assert (obs_iter+1 == feature_obs.end());
    
    const std::size_t index = obs_iter - feature_obs.begin();
    
    auto obs_info_iter = m_observations.emplace_hint (m_observations.end(), t,
                                                      observation_info (feature_iter, index));
    assert (obs_info_iter+1 == m_observations.end());
    
    using namespace std::placeholders;
    m_listeners.for_each (std::bind (&listener::observation, _1, t, std::cref(obs_info_iter->second)));
}


extern template class slam::slam_data<control_model_type, observation_model_type>;


#endif //_SLAM_SLAM_DATA_HPP
