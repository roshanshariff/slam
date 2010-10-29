#ifndef _SLAM_SLAM_DATA_HPP
#define _SLAM_SLAM_DATA_HPP

#include <vector>
#include <map>
#include <cassert>

#include <boost/signals2.hpp>

#include "utilities/arraymap.hpp"

/** This class stores a record of all state changes and observations, as probability distributions.
    It is also responsible for notifying listeners when new state changes and observations are added.
    ActionModel and ObservationModel are the types of distributions over state changes and
    observations, respectively. */
template <class ActionModel, class ObservationModel>
class slam_data {

public:

  /** Use the native machine unsigned integral type to identify features and observations. This is 64
      bits on amd64 machines and 32 bits on i386 machines. */
  typedef size_t featureid_t;
  typedef size_t actionid_t;

  /** The types of distributions over state changes and observations, respectively. */
  typedef ActionModel action_model_type;
  typedef ObservationModel observation_model_type;

  /** Actions are stored as a vector of the corresponding distributions. */
  typedef std::vector<action_model_type> action_data_type;

  /** Feature observations are stored as an arraymap from the time of an observation to the observation's
      distribution. Since observations are expected to be added in chronological order, this is much
      more efficient then using a tree-based map. */
  typedef arraymap<actionid_t, observation_model_type> feature_data_type;

  /** Features are stored as a map from the feature_id to the feature's observation data. */
  typedef std::map<featureid_t, feature_data_type> observation_data_type;

  /** Action signal handlers are passed the action id just added, whereas observation signal handlers
      are passed the feature id being observed and the time of the observation. */
  typedef boost::signals2::signal<void (actionid_t)> action_signal_type;
  typedef boost::signals2::signal<void (featureid_t, actionid_t)> observation_signal_type;

  /** Listeners for action and observation signals. */
  typedef typename action_signal_type::slot_type action_listener_type;
  typedef typename observation_signal_type::slot_type observation_listener_type;

private:

  action_data_type _actions;
  observation_data_type _observations;
  mutable action_signal_type _action_signal;
  mutable observation_signal_type _observation_signal;

public:

  const action_data_type& actions () const { return _actions; }
  const observation_data_type& observations () const { return _observations; }

  /** Retrieve the state change specified by the given id. */
  const action_model_type& action (actionid_t id) const {
    assert (id < actions().size());
    return actions()[id];
  }

  /** Retrieve the observations of the feature specified by the given id. */
  const feature_data_type& observation (featureid_t id) const {
    typename observation_data_type::const_iterator i = observations().find(id);
    assert (i != observations().end());
    return i->second;
  }

  /** Add a new state change to the end of the list. */
  actionid_t add_action (const action_model_type& action) {
    actionid_t action_id = actions().size();
    _actions.push_back (action);
    _action_signal (action_id); // Trigger the signal to notify listeners of the addition.
    return action_id;
  }

  /** Add a new observation of the specified feature, taken at the current time. */
  void add_observation (const featureid_t feature_id, const observation_model_type& obs) {
    _observations[feature_id][actions().size()] = obs;
    _observation_signal(feature_id, actions().size()); // Trigger the signal to notify listeners.
  }

  /** Connect a listener to recieve notifications of new state changes. */
  boost::signals2::connection connect_action_listener (const action_listener_type& l) const {
    return _action_signal.connect(l);
  }

  /** Connect a listener to recieve notifications of new observations. */
  boost::signals2::connection connect_observation_listener (const observation_listener_type& l) const {
    return _observation_signal.connect(l);
  }

};  

#endif //_SLAM_SLAM_DATA_HPP
