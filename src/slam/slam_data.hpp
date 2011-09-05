#ifndef _SLAM_SLAM_DATA_HPP
#define _SLAM_SLAM_DATA_HPP

#include <vector>
#include <map>
#include <cassert>

#include <boost/signals2.hpp>

#include "utility/arraymap.hpp"

/** This class stores a record of all state changes and observations, as probability distributions.
    It is also responsible for notifying listeners when new state changes and observations are added.
    ActionModel and ObservationModel are the types of distributions over state changes and
    observations, respectively. */
template <class ControlModel, class ObservationModel>
class slam_data {

public:

	/** Use the native machine unsigned integral type to identify features and observations. This is 64
      bits on amd64 machines and 32 bits on i386 machines. */
	typedef size_t timestep_t;
	typedef size_t featureid_t;

	/** The types of distributions over state changes and observations, respectively. */
	typedef ControlModel control_model_type;
	typedef ObservationModel observation_model_type;
    
    /** The types of controls and observations */
    typedef typename control_model_type::result_type control_type;
    typedef typename observation_model_type::result_type observation_type;

	/** Feature observations are stored as an arraymap from the time of an observation to the observation's
      distribution. Since observations are expected to be added in chronological order, this is much
      more efficient then using a tree-based map. */
	typedef arraymap<timestep_t, observation_model_type> observation_data_type;

	/** Action signal handlers are passed the action id just added, whereas observation signal handlers
      are passed the feature id being observed and the time of the observation. */
	typedef boost::signals2::signal<void (timestep_t, control_model_type)> control_signal_type;
	typedef boost::signals2::signal<void (timestep_t, featureid_t, observation_model_type)> observation_signal_type;
	typedef boost::signals2::signal<void (timestep_t)> timestep_signal_type;

	/** Listeners for action and observation signals. */
	typedef typename control_signal_type::slot_type control_listener_type;
	typedef typename observation_signal_type::slot_type observation_listener_type;
	typedef typename timestep_signal_type::slot_type timestep_listener_type;

private:

	/** Actions are stored as a vector of the corresponding distributions. */
	std::vector<control_model_type> m_controls;

	std::map<featureid_t, observation_data_type> m_observations;

	mutable control_signal_type m_control_signal;
	mutable observation_signal_type m_observation_signal;
	mutable timestep_signal_type m_timestep_signal;

public:

	timestep_t current_timestep () const { return m_controls.size(); }

	/** Retrieve the state change specified by the given id. */
	const control_model_type& control (timestep_t timestep) const {
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
	void add_control (const control_model_type& control) {
		timestep_t timestep = current_timestep();
		m_controls.push_back (control);
		m_control_signal (timestep, control);
	}

	/** Add a new observation of the specified feature, taken at the current time. */
	void add_observation (const featureid_t feature, const observation_model_type& obs) {
		timestep_t timestep = current_timestep();
		m_observations[feature][timestep] = obs;
		m_observation_signal (timestep, feature, obs);
	}

	void timestep () const {
		m_timestep_signal (current_timestep());
	}

	/** Connect a listener to receive notifications of new state changes. */
	boost::signals2::connection connect_control_listener (const control_listener_type& l) const {
		return m_control_signal.connect(l);
	}

	/** Connect a listener to receive notifications of new observations. */
	boost::signals2::connection connect_observation_listener (const observation_listener_type& l) const {
		return m_observation_signal.connect(l);
	}

	boost::signals2::connection connect_timestep_listener (const timestep_listener_type& l) const {
		return m_timestep_signal.connect(l);
	}

};  

#endif //_SLAM_SLAM_DATA_HPP
