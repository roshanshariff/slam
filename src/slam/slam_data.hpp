#ifndef _SLAM_SLAM_DATA_HPP
#define _SLAM_SLAM_DATA_HPP

#include <vector>
#include <map>
#include <cassert>

#include <boost/shared_ptr.hpp>
#include <boost/enable_shared_from_this.hpp>
#include <boost/bind.hpp>
#include <boost/ref.hpp>
#include <boost/utility.hpp>

#include "utility/arraymap.hpp"

/** This class stores a record of all state changes and observations, as probability distributions.
    It is also responsible for notifying listeners when new state changes and observations are added.
    ActionModel and ObservationModel are the types of distributions over state changes and
    observations, respectively. */
template <class ControlModel, class ObservationModel>
class slam_data : boost::noncopyable {

public:

	/** Use the native machine unsigned integral type to identify features and observations. This is 64
      bits on amd64 machines and 32 bits on i386 machines. */
	typedef size_t timestep_t;
	typedef size_t featureid_t;

	/** Feature observations are stored as an arraymap from the time of an observation to the observation's
     distribution. Since observations are expected to be added in chronological order, this is much
     more efficient then using a tree-based map. */
	typedef arraymap<timestep_t, ObservationModel> observation_data_type;
	
    class listener : boost::noncopyable, public boost::enable_shared_from_this<listener> {
        
        boost::shared_ptr<const slam_data> data_ptr;
        
    public:
        
        void connect (boost::shared_ptr<const slam_data>);
        
        bool is_connected () const { return data_ptr.get() != 0; }
        
        const slam_data& data() const { assert (is_connected()); return *data_ptr; }
        
        void disconnected () { data_ptr.reset(); }
        
        virtual void add_control (timestep_t, const ControlModel&) = 0;
        
        virtual void add_observation (timestep_t, featureid_t, const ObservationModel&) = 0;
        
        virtual void end_observation (timestep_t) = 0;
        
        virtual void end_simulation (timestep_t) = 0;
        
        virtual ~listener () { }
        
    };
        
private:

	/** Actions are stored as a vector of the corresponding distributions. */
	std::vector<ControlModel> m_controls;

	std::map<featureid_t, observation_data_type> m_observations;

    mutable std::vector<boost::weak_ptr<listener> > m_listeners;
    
    template <class Functor>
    void foreach_listener (const Functor& f) {
        typename std::vector<boost::weak_ptr<listener> >::iterator iter = m_listeners.begin();
        while (iter != m_listeners.end()) {
            if (boost::shared_ptr<listener> l = iter->lock()) {
                f(l.get());
                ++iter;
            }
            else {
                iter = m_listeners.erase (iter);
            }
        }
    }
    
public:
    
	timestep_t current_timestep () const { return m_controls.size(); }

	/** Retrieve the state change specified by the given id. */
	const ControlModel& control (timestep_t timestep) const {
		assert (timestep < current_timestep());
		return m_controls[timestep];
	}

	/** Retrieve the observations of the feature specified by the given id. */
	const observation_data_type& observations (featureid_t feature) const {
		typename std::map<featureid_t, observation_data_type>::const_iterator i = m_observations.find(feature);
		assert (i != m_observations.end());
		return i->second;
	}

	/** Add a new state change to the end of the list. */
	void add_control (const ControlModel& control) {
		timestep_t timestep = current_timestep();
		m_controls.push_back (control);
        foreach_listener (boost::bind (&listener::add_control, _1, timestep, boost::cref(control)));
	}

	/** Add a new observation of the specified feature, taken at the current time. */
	void add_observation (const featureid_t feature, const ObservationModel& obs) {
		timestep_t timestep = current_timestep();
		m_observations[feature][timestep] = obs;
        foreach_listener (boost::bind (&listener::add_observation, _1, timestep, feature, boost::cref(obs)));
	}

	void end_observation () {
        foreach_listener (boost::bind (&listener::end_observation, _1, current_timestep()));
	}
    
    void end_simulation () {
        foreach_listener (boost::bind (&listener::end_simulation, _1, current_timestep()));
    }

};

template <class ControlModel, class ObservationModel>
void slam_data<ControlModel, ObservationModel>::listener::connect (boost::shared_ptr<const slam_data> data_ptr_) {
    data_ptr = data_ptr_;
    data_ptr->m_listeners.push_back (this->shared_from_this());
}

#endif //_SLAM_SLAM_DATA_HPP
