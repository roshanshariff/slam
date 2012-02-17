#ifndef _SLAM_SLAM_DATA_HPP
#define _SLAM_SLAM_DATA_HPP

#include <vector>
#include <map>
#include <functional>
#include <utility>
#include <cassert>

#include <boost/shared_ptr.hpp>
#include <boost/bind.hpp>
#include <boost/ref.hpp>

#include "slam/interfaces.hpp"
#include "utility/listeners.hpp"
#include "utility/flat_map.hpp"


namespace slam {
    
    
    /** This class stores a record of all state changes and observations, as probability distributions.
     It is also responsible for notifying listeners when new state changes and observations are added.
     ActionModel and ObservationModel are the types of distributions over state changes and
     observations, respectively. */
    template <class ControlModel, class ObservationModel>
    struct slam_data : public data_source {

        using feature_data_type = utility::flat_map<timestep_type, ObservationModel>;
        
        class observation_data_type {

            featureid_type id;
            const feature_data_type* data;
            std::size_t ix;

        public:

            observation_data_type (featureid_type id, const feature_data_type& data, std::size_t ix)
            : id(id), data(&data), ix(ix) { }

            featureid_type feature_id () const { return id; }
            const feature_data_type& feature_data () const { return *data; }
            const ObservationModel& observation () const { return (data->begin()+ix)->second; }
            std::size_t index () const { return ix; }
        };
        
        struct listener : public timestep_listener {
            virtual void control (timestep_type, const ControlModel&) = 0;
            virtual void observation (timestep_type, const observation_data_type&) = 0;
        };
        
    private:
        
        std::vector<ControlModel> controls;
        std::map<featureid_type, feature_data_type> features;
        utility::flat_multimap<timestep_type, observation_data_type> observations;
        
        utility::listeners<listener> listeners;
        
        using observation_iterator = typename utility::flat_multimap<timestep_type, observation_data_type>::const_iterator;

    public:
        
        slam_data () = default;
        
        /** Noncopyable */
        slam_data (const slam_data&) = delete;
        slam_data& operator= (const slam_data&) = delete;
        
        /** Member functions from data_source */
        
        virtual timestep_type current_timestep () const override {
            return timestep_type (controls.size());
        }
        
        virtual void timestep (timestep_type timestep) override {
            assert (timestep == current_timestep());
            listeners.for_each (boost::bind (&listener::timestep, _1, timestep));
        }
        
        /** Retrieve controls. */
        
        const ControlModel& control (timestep_type timestep) const {
            return controls.at(timestep);
        }
        
        /** Retrieve observations. */
        
        std::size_t count_observations () const { return observations.size(); }
        std::size_t count_observations (timestep_type t) const { return observations.count (t); }
        
        observation_iterator observations_begin () const {
            return observations.begin();
        }
        
        observation_iterator observations_end () const {
            return observations.end();
        }
        
        observation_iterator observations_begin (timestep_type t) const {
            return observations.lower_bound(t);
        }
        
        observation_iterator observations_end (timestep_type t) const {
            return observations.upper_bound(t);
        }
        
        std::pair<observation_iterator, observation_iterator> observations_at (timestep_type t) const {
            // TODO: Use equal_range when boost bug is fixed
            return std::make_pair (observations_begin(t), observations_end(t));
        }
        
        /** Retrieve features. */
        
        bool feature_observed (featureid_type f) const { return features.find(f) != features.end(); }
        
        const feature_data_type& feature_data (featureid_type featureid) const {
            return features.at(featureid);
        }
        
        /** Add new data */
        
        void add_control (const ControlModel&);
        void add_observation (featureid_type, const ObservationModel&);
        
        /** Add new listener */
        
        void add_listener (const boost::shared_ptr<listener>& l) { listeners.add(l); }
        
    };
    

    template <class ControlModel, class ObservationModel>
    void slam_data<ControlModel, ObservationModel>
    ::add_control (const ControlModel& control) {

        timestep_type t = current_timestep();
        
        controls.push_back (control);
        listeners.for_each (boost::bind (&listener::control, _1, t, boost::cref(control)));
    }
    
    
    template <class ControlModel, class ObservationModel>
    void slam_data<ControlModel, ObservationModel>
    ::add_observation (featureid_type featureid, const ObservationModel& obs) {
        
        timestep_type t = current_timestep();
        
        auto& feature_data = features[featureid];
        std::size_t index = feature_data.size();
        
        feature_data.emplace_hint (feature_data.end(), t, obs);
        
        observation_data_type obs_data (featureid, feature_data, index);
        observations.emplace_hint (observations.end(), t, obs_data);
        
        listeners.for_each (boost::bind (&listener::observation, _1, t, boost::cref(obs_data)));
    }
    
    /*
     
    double log_likelihood (boost::shared_ptr<const slam_result_type> result) const {
        
        boost::shared_ptr<const bitree<state_type> > trajectory = result->get_trajectory();
        boost::shared_ptr<const boost::container::flat_map<size_t, feature_type> > map = result->get_map();
        
        assert (trajectory->size() <= current_timestep());
        
        double log_likelihood = 0;
        
        for (timestep_t t = 0; t < trajectory->size(); ++t) {
            log_likelihood += control(t).log_likelihood ((*trajectory)[t]);
        }
        
        typename boost::container::flat_map<size_t, feature_type>::const_iterator map_iter = map->begin();
        for (; map_iter != map->end(); ++map_iter) {
            
            const feature_data_type& data = feature_data (map_iter->first);
            feature_type observation = map_iter->second;
            timestep_t observation_base = 0;
            
            typename feature_data_type::const_iterator obs_iter = data.begin();
            for (; obs_iter != data.end(); ++obs_iter) {
                
                const state_type& state_change = trajectory->accumulate (observation_base, obs_iter->first);
                observation = -state_change + observation;
                observation_base = obs_iter->first;
                
                const ObservationModel& distribution = obs_iter->second;
                log_likelihood += distribution.log_likelihood (observation);
            }
        }
        
        return log_likelihood;
    }
    
    
    double likelihood (boost::shared_ptr<const slam_result_type> result) const {
        return std::exp (log_likelihood (result));
    }
    
*/
    
} // namespace slam

#endif //_SLAM_SLAM_DATA_HPP
