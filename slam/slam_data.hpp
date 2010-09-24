#ifndef _SLAM_SLAM_DATA_HPP
#define _SLAM_SLAM_DATA_HPP

#include <vector>
#include <map>
#include <cassert>
#include <boost/signals2.hpp>

#include "utilities/arraymap.hpp"

template <class ActionModel, class ObservationModel>
class slam_data {

public:

  typedef size_t featureid_t;
  typedef size_t actionid_t;

  typedef ActionModel action_model_type;
  typedef ObservationModel observation_model_type;

  typedef std::vector<action_model_type> action_data_type;
  typedef arraymap<actionid_t, observation_model_type> feature_data_type;
  typedef std::map<featureid_t, feature_data_type> observation_data_type;

  typedef boost::signals2::signal<void (actionid_t)> action_signal_type;
  typedef boost::signals2::signal<void (featureid_t, actionid_t)> observation_signal_type;

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

  const action_model_type& action (actionid_t id) const {
    assert (id < actions().size());
    return actions()[id];
  }

  const feature_data_type& observation (featureid_t id) const {
    typename observation_data_type::const_iterator i = observations().find(id);
    assert (i != observations().end());
    return i->second;
  }

  actionid_t add_action (const action_model_type& action) {
    actionid_t action_id = actions().size();
    _actions.push_back (action);
    _action_signal (action_id);
    return action_id;
  }

  void add_observation (featureid_t feature_id, const observation_model_type& obs) {
    _observations[feature_id][actions().size()] = obs;
    _observation_signal(feature_id, actions().size());
  }

  boost::signals2::connection connect_action_listener (const action_listener_type& l)
    const {
    return _action_signal.connect(l);
  }

  boost::signals2::connection connect_observation_listener (const observation_listener_type& l)
    const {
    return _observation_signal.connect(l);
  }

};  

#endif //_SLAM_SLAM_DATA_HPP
