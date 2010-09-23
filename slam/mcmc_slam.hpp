#ifndef _SLAM_MCMC_SLAM_HPP
#define _SLAM_MCMC_SLAM_HPP

#include <vector>
#include <utility>
#include <cassert>
#include <cmath>

#include "utilities/random.hpp"
#include "utilities/bitree.hpp"
#include "utilities/arraymap.hpp"


const int MCMC_STEPS = 100;


template <class ActionModel, class ObservationModel>
class mcmc_slam {

  typedef size_t actionid_t;
  typedef size_t featureid_t;

public:

  typedef ActionModel action_model_type;
  typedef ObservationModel observation_model_type;
  typedef typename action_model_type::result_type action_type;
  typedef typename observation_model_type::result_type observation_type;
  
  typedef std::vector<action_model_type> action_data_type;
  typedef std::vector<arraymap<actionid_t, observation_model_type> > observation_data_type;

private:

  struct feature_data {
    bool observed;
    actionid_t parent_action;
    observation_type estimate;
  };

  const action_data_type& action_data;
  const observation_data_type& observation_data;

  random_source& random;

  bitree<action_type> action_estimates;
  bitree<double> action_weights;

  std::vector<feature_data> feature_estimates;
  bitree<double> feature_weights;

  double action_edge_weight (const action_model_type& model, const action_type& estimate) {
    const double ACTION_SPACE_DIM = 3.0;
    double likelihood = model.likelihood(estimate) + 0.0001;
    return ACTION_SPACE_DIM * std::pow (likelihood, -1.0/ACTION_SPACE_DIM);
  }

  double observation_edge_weight (const observation_model_type& model, const observation_type& estimate) {
    const double OBSERVATION_SPACE_DIM = 2.0;
    double likelihood = model.likelihood(estimate) + 0.0001;
    return OBSERVATION_SPACE_DIM * std::pow (likelihood, -1.0/OBSERVATION_SPACE_DIM);
  }

public:

  void initialize ();
  void update ();
  const bitree<action_type>& get_action_estimates () const { return action_estimates; }

  std::vector<observation_type> get_feature_estimates () const {
    std::vector<observation_type> estimates;
    for (featureid_t i = 0; i < feature_estimates.size(); ++i) {
      if (!feature_estimates[i].observed) continue;
      const feature_data& feature = feature_estimates[i];
      action_type parent_action = action_estimates.accumulate(feature.parent_action);
      estimates.push_back(parent_action + feature.estimate);
    }
    return estimates;
  }

private:

  void initialize_actions ();
  void initialize_features ();
  
  void update_action (const actionid_t action_id);
  double action_change (const actionid_t action_id) const;

  void update_feature (const featureid_t feature_id);

public:

  mcmc_slam (const action_data_type& _action_data,
	     const observation_data_type& _observation_data,
	     random_source& _random)
    : action_data(_action_data),
      observation_data(_observation_data),
      random(_random)
  { initialize(); }

};


template <class ActionModel, class ObservationModel>
void mcmc_slam<ActionModel, ObservationModel>::initialize () {
  initialize_actions();
  initialize_features();
}


template <class ActionModel, class ObservationModel>
void mcmc_slam<ActionModel, ObservationModel>::initialize_actions () {
  if (action_data.size() < action_estimates.size()) {
    action_estimates.resize (action_data.size());
    action_weights.resize (action_data.size());
  }
  else {
    action_estimates.reserve (action_data.size());
    action_weights.reserve (action_data.size());

    for (actionid_t i = action_estimates.size(); i < action_data.size(); ++i) {
      action_type estimate = action_data[i].mean();
      action_estimates.push_back (estimate);
      action_weights.push_back (action_edge_weight (action_data[i], estimate));
    }
  }
}


template <class ActionModel, class ObservationModel>
void mcmc_slam<ActionModel, ObservationModel>::initialize_features () {

  feature_estimates.resize (observation_data.size());
  feature_weights.resize (observation_data.size());

  for (featureid_t i = 0; i < feature_estimates.size(); ++i) {

    const arraymap<actionid_t, observation_model_type>& observations = observation_data[i];
    feature_data& feature = feature_estimates[i];

    if (observations.empty()) {
      feature.observed = false;
      feature_weights[i] = 0;
    }
    else if (!feature.observed) {
      feature.observed = true;
      std::cout << "Observed: " << i << std::endl;

      const std::pair<const actionid_t, observation_model_type>& first_observation = 
	observations.front();

      feature.parent_action = first_observation.first;
      const observation_model_type& distribution = first_observation.second;
      feature.estimate = distribution.mean();
      feature_weights[i] = observation_edge_weight (distribution, feature.estimate);
    }
  }
}


template <class ActionModel, class ObservationModel>
void mcmc_slam<ActionModel, ObservationModel>::update () {

  int action_updates = 0;
  int feature_updates = 0;

  for (int i = 0; i < MCMC_STEPS; ++i) {

    const double action_weight = action_weights.accumulate();
    const double feature_weight = feature_weights.accumulate();

    if (action_weight == 0 && feature_weight == 0) return;

    double action_range, feature_range;
    do {
      action_range = random.uniform() * (action_weight + feature_weight);
      feature_range = action_range - action_weight;
    }
    while (!(action_range < action_weight || feature_range < feature_weight));
      
    if (action_range < action_weight) {
      ++action_updates;
      actionid_t action_id = action_weights.binary_search (action_range);
      assert (action_id < action_estimates.size());
      update_action (action_id);
    }
    else if (feature_weight > 0) {
      ++feature_updates;
      featureid_t feature_id = feature_weights.binary_search (feature_range);
      assert (feature_id < feature_estimates.size());
      update_feature (feature_id);
    }

  }

  std::cout << action_updates << " action updates, " << feature_updates << " feature updates\n";

}

template <class ActionModel, class ObservationModel>
void mcmc_slam<ActionModel, ObservationModel>::update_action (const actionid_t id) {

  const action_type new_estimate = action_data[id](random);
  const double new_action_weight = action_edge_weight(action_data[id], new_estimate);

  const action_type old_estimate = action_estimates.at(id);
  const double old_action_weight = action_weights.at(id);

  double acceptance_probability = new_action_weight / old_action_weight;
  acceptance_probability /= action_change(id);

  action_estimates[id] = new_estimate;
  action_weights[id] = new_action_weight;
  acceptance_probability *= action_change(id);

  if (random.uniform() >= acceptance_probability) {
    action_estimates[id] = old_estimate;
    action_weights[id] = old_action_weight;
  }

}


template <class ActionModel, class ObservationModel>
double mcmc_slam<ActionModel, ObservationModel>::action_change (const actionid_t action_id) const {

  double result = 1.0;

  for (featureid_t i = 0; i < feature_estimates.size(); ++i) {

    const arraymap<actionid_t, observation_model_type>& observations = observation_data[i];
    const feature_data& feature = feature_estimates[i];

    if (!feature.observed) continue;

    typename arraymap<actionid_t, observation_model_type>::const_iterator i, end;

    if (feature.parent_action <= action_id) {
      i = observations.upper_bound(action_id);
      end = observations.end();
    }
    else {
      i = observations.begin();
      end = observations.upper_bound(action_id);
    }

    observation_type observation = feature.estimate;
    actionid_t previous_action = feature.parent_action;

    for (; i != end; ++i) {
      observation = -action_estimates.accumulate(previous_action, i->first) + observation;
      previous_action = i->first;
      const observation_model_type& distribution = i->second;
      result *= distribution.likelihood(observation);
    }
  }

  return result;
}


template <class ActionModel, class ObservationModel>
void mcmc_slam<ActionModel, ObservationModel>::update_feature (const featureid_t id) {

  const arraymap<actionid_t, observation_model_type>& observations = observation_data[id];
  feature_data& feature = feature_estimates[id];

  const observation_model_type& distribution
    = observations.find(feature.parent_action)->second;
  
  const observation_type new_estimate = distribution(random);
  const double new_feature_weight = observation_edge_weight(distribution, new_estimate);

  double acceptance_probability = new_feature_weight / feature_weights.at(id);

  typename arraymap<actionid_t, observation_model_type>::const_iterator i = observations.begin();

  observation_type new_observation = new_estimate;
  observation_type old_observation = feature.estimate;
  actionid_t previous_action = feature.parent_action;

  for (; i != observations.end(); ++i) {

    if (i->first == feature.parent_action) continue;

    action_type action_difference  = action_estimates.accumulate(previous_action, i->first);
    new_observation = -action_difference + new_observation;
    old_observation = -action_difference + old_observation;
    previous_action = i->first;

    const observation_model_type& distribution = i->second;
    acceptance_probability *= distribution.likelihood(new_observation) 
      / distribution.likelihood(old_observation);
  }

  if (random.uniform() < acceptance_probability) {
    feature.estimate = new_estimate;
    feature_weights[id] = new_feature_weight;
  }

}

#endif //_SLAM_MCMC_SLAM_HPP
