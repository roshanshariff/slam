#include <vector>
#include <utility>
#include <cassert>

#include "utilities/random.hpp"
#include "utilities/arraymap.hpp"
#include "utilities/cumulative_sequence.hpp"


template <class ActionModel, class ObservationModel>
class mcmc_slam {

public:

  typedef ActionModel action_model_type;
  typedef ObservationModel observation_model_type;
  typedef typename ActionModel::result_type action_type;
  typedef typename ObservationModel::result_type observation_type;
  
  typedef std::vector<action_model_type> action_data_type;
  typedef std::vector<arraymap<size_t, observation_model_type> > observation_data_type;

private:

  struct feature_data {
    bool observed;
    size_t parent_action;
    observation_type estimate;
  };

  const action_data_type& action_data;
  const observation_data_type observation_data;

  cumulative_sequence<action_type> action_estimates;
  cumulative_sequence<double> action_weights;

  std::vector<feature_data> feature_estimates;
  cumulative_sequence<double> feature_weights;

  random_source& random;

  template <class Distribution, class Estimate>
  double edge_weight (const Distribution& distribution, const Estimate& estimate) {
    return 1.0 / distribution.likelihood(estimate);
  }

public:

  void initialize ();
  void update ();

private:

  void initialize_actions ();
  void initialize_features ();
  
  void update_action (size_t action_id);
  void update_feature (size_t feature_id);

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

    for (size_t i = action_estimates.size(); i < action_data.size(); ++i) {
      action_type estimate = action_data[i].mean();
      action_estimates.push_back (estimate);
      action_weights.push_back (edge_weight (action_data[i], estimate));
    }
  }
}


template <class ActionModel, class ObservationModel>
void mcmc_slam<ActionModel, ObservationModel>::initialize_features () {

  feature_estimates.resize (observation_data.size());
  feature_weights.resize (observation_data.size());

  for (size_t i = 0; i < feature_estimates.size(); ++i) {

    const arraymap<size_t, observation_model_type>& observations = observations[i];
    feature_data& feature = feature_estimates[i];

    if (observations.empty()) {
      feature.observed = false;
      feature_weights.set (i, 0);
    }
    else if (!feature.observed) {
      feature.observed = true;

      const std::pair<const size_t, observation_model_type>& first_observation = 
	observations.front();

      feature.parent_action = first_observation.first;
      const observation_model_type& distribution = first_observation.second;
      feature.estimate = distribution.mean();
      feature_weights.set (i, edge_weight (distribution, feature.estimate));
    }
  }
}

template <class ActionModel, class ObservationModel>
void mcmc_slam<ActionModel, ObservationModel>::update () {

  double action_weight = action_weights.accumulate();
  double feature_weight = feature_weights.accumulate();

  double action_range, feature_range;
  do {
    action_range = random.uniform() * (action_weight + feature_weight);
    feature_range = action_range - action_weight;
  }
  while (!(action_range < action_weight || feature_range < feature_weight));
      
  if (action_range < action_weight) {
    size_t action_id = action_weights.binary_search (action_range);
    assert (action_id < action_estimates.size());
    update_action (action_id);
  }
  else if (feature_weight > 0) {
    size_t feature_id = feature_weights.binary_search (feature_range);
    assert (feature_id < feature_estimates.size());
    update_feature (feature_id);
  }

}


template <class ActionModel, class ObservationModel>
void mcmc_slam<ActionModel, ObservationModel>::update_action (size_t id) {

  const action_type new_estimate = action_data[id](random);
  const double new_action_weight = edge_weight(action_data[id], new_estimate);

  const action_type old_estimate = action_estimates.get(id);
  const double old_action_weight = action_weights.get(id);

  double acceptance_probability = new_action_weight / old_action_weight;
  acceptance_probability /= compute_action_changed(id);

  action_estimates.set(id, new_estimate);
  action_weights.set(id, new_action_weight);
  acceptance_probability *= compute_action_changed(id);

  if (random.uniform() >= acceptance_probability) {
    action_estimates.set(id, old_estimate);
    action_weights.set(id, old_action_weight);
  }

}


template <class ActionModel, class ObservationModel>
double compute_action_changed (const size_t action_id) const {

  double result = 1.0;

  for (size_t i = 0; i < feature_estimates.size(); ++i) {

    const arraymap<size_t, observation_model_type>& observations = observation_data[i];
    const feature_data& feature = feature_estimates[i];

    if (!feature.observed) continue;

    arraymap<size_t, observation_model_type>::const_iterator i, end;

    if (feature.parent_action <= action_id) {
      i = observations.upper_bound(action_id);
      end = observations.end();
    }
    else {
      i = observations.begin();
      end = observations.upper_bound(action_id);
    }

    observation_type observation = feature.estimate;
    size_t previous_action = feature.parent_action;

    for (; i != end; ++i) {
      observation = -action_estimates.accumulate(previous_action, i->first) + observation;
      previous_action = i->first;
      const observation_model& distribution = i->second;
      result *= distribution.likelihood(observation);
    }
  }

  return result;
}


template <class ActionModel, class ObservationModel>
void mcmc_slam<ActionModel, ObservationModel>::update_feature (const size_t id) {

  const arraymap<size_t, observation_model_type>& observations = observation_data[id];
  feature_data& feature = feature_estimates[id];

  const observation_model_type& distribution
    = observations.find(feature.parent_action)->second;
  
  const observation_type new_estimate = distribution(random);
  const double new_feature_weight = edge_weight(distribution, new_estimate);

  double acceptance_probability = new_feature_weight / feature_weights.get(id);

  arraymap<size_t, observation_model_type>::const_iterator i = observations.begin();

  observation_type new_observation = new_estimate;
  observation_type old_observation = feature.estimate;
  size_t previous_action = feature.parent_action;

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
    feature_weights.set(id, new_feature_weight);
  }

}
