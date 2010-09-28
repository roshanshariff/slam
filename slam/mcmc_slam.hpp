#ifndef _SLAM_MCMC_SLAM_HPP
#define _SLAM_MCMC_SLAM_HPP

#include <vector>
#include <map>
#include <utility>
#include <cassert>
#include <cmath>
#include <boost/bind.hpp>
#include <boost/signals2.hpp>

#include "slam/slam_data.hpp"
#include "utilities/random.hpp"
#include "utilities/bitree.hpp"
#include "utilities/arraymap.hpp"


template <class SlamData>
class mcmc_slam {

public:

  typedef SlamData slam_data_type;

  typedef typename slam_data_type::actionid_t actionid_t;
  typedef typename slam_data_type::featureid_t featureid_t;

  typedef typename slam_data_type::action_data_type action_data_type;
  typedef typename slam_data_type::feature_data_type feature_data_type;
  typedef typename slam_data_type::observation_data_type observation_data_type;

  typedef typename slam_data_type::action_model_type action_model_type;
  typedef typename slam_data_type::observation_model_type  observation_model_type;

  typedef typename action_model_type::result_type action_type;
  typedef typename observation_model_type::result_type observation_type;

private:

  struct feature_data {
    bool observed;
    actionid_t parent_action;
    observation_type estimate;
    feature_data () : observed(false) { }
  };

  random_source& random;

  const int mcmc_steps;
  const double action_dimensions;
  const double observation_dimensions;

  const slam_data_type& data;
  const boost::signals2::scoped_connection _act_conn;
  const boost::signals2::scoped_connection _obs_conn;

  bitree<action_type> action_estimates;
  bitree<double> action_weights;

  std::vector<feature_data> feature_estimates;
  bitree<double> feature_weights;

  template <class Model, class Label>
  double edge_weight (const Model& model, const Label& label, double importance) {
    return importance * std::pow (model.likelihood(label)+0.0001, -1.0/importance);
  }

  double action_edge_weight (const action_model_type& model, const action_type& estimate) {
    return edge_weight (model, estimate, action_dimensions);
  }

  double observation_edge_weight (const observation_model_type& model, const observation_type& estimate) {
    return edge_weight (model, estimate, observation_dimensions);
  }

public:

  void add_action (actionid_t action_id);
  void add_observation (featureid_t feature_id, actionid_t action_id);

  void update ();

  const bitree<action_type>& get_action_estimates () const { return action_estimates; }

  std::map<featureid_t, observation_type> get_feature_estimates () const {
    std::map<featureid_t, observation_type> estimates;
    for (featureid_t i = 0; i < feature_estimates.size(); ++i) {
      if (!feature_estimates[i].observed) continue;
      const feature_data& feature = feature_estimates[i];
      action_type parent_action = action_estimates.accumulate(feature.parent_action);
      estimates[i] = parent_action + feature.estimate;
    }
    return estimates;
  }

private:

  void update_action (const actionid_t action_id);
  double action_change (const actionid_t action_id) const;

  void update_feature (const featureid_t feature_id);

public:

  mcmc_slam (random_source& _random, const slam_data_type& _data, int num_steps,
	     double action_importance, double observation_importance)
    : random(_random), mcmc_steps(num_steps), action_dimensions(action_importance),
      observation_dimensions(observation_importance), data (_data),
      _act_conn (data.connect_action_listener (boost::bind (&mcmc_slam::add_action, this, _1))),
      _obs_conn (data.connect_observation_listener (boost::bind (&mcmc_slam::add_observation, this, _1, _2)))
  { }

};


template <class SlamData>
void mcmc_slam<SlamData>::add_action (actionid_t action_id) {
  assert (action_id == action_estimates.size());
  assert (action_id == action_weights.size());
  const action_model_type& action_model = data.action(action_id);
  action_estimates.push_back (action_model.mean());
  action_weights.push_back (action_edge_weight (action_model, action_estimates[action_id]));
}


template <class SlamData>
void mcmc_slam<SlamData>::add_observation (featureid_t feature_id, actionid_t action_id) {

  if (feature_estimates.size() <= feature_id) {
    feature_estimates.resize(feature_id+1);
    feature_weights.resize(feature_id+1);
  }

  assert (feature_id < feature_estimates.size());
  assert (feature_id < feature_weights.size());

  feature_data& feature = feature_estimates[feature_id];

  if (!feature.observed) {
    feature.observed = true;
    feature.parent_action = action_id;
    const observation_model_type& observation_model = data.observation(feature_id).find(action_id)->second;
    feature.estimate = observation_model.mean();
    feature_weights[feature_id] = observation_edge_weight (observation_model, feature.estimate);
    //std::cout << "setting feature " << feature_id << " parent to " << action_id << std::endl;
  }
}


template <class SlamData>
void mcmc_slam<SlamData>::update () {

  int action_updates = 0;
  int feature_updates = 0;

  for (int i = 0; i < mcmc_steps; ++i) {

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

  //std::cout << action_updates << " action updates, " << feature_updates << " feature updates\n";
}

template <class SlamData>
void mcmc_slam<SlamData>::update_action (const actionid_t id) {

  const action_type new_estimate = data.action(id)(random);
  const double new_action_weight = action_edge_weight(data.action(id), new_estimate);

  const action_type old_estimate = action_estimates.at(id);
  const double old_action_weight = action_weights.at(id);

  double acceptance_probability = -action_change(id);

  action_estimates[id] = new_estimate;
  action_weights[id] = new_action_weight;
  acceptance_probability += action_change(id);

  acceptance_probability = std::exp(acceptance_probability) * new_action_weight / old_action_weight;

  if (random.uniform() >= acceptance_probability) {
    action_estimates[id] = old_estimate;
    action_weights[id] = old_action_weight;
  }

}


template <class SlamData>
double mcmc_slam<SlamData>::action_change (const actionid_t action_id) const {

  double result = 0.0;
  typename observation_data_type::const_iterator i = data.observations().begin();

  for (; i != data.observations().end(); ++i) {

    const feature_data_type& observations = i->second;
    const feature_data& feature = feature_estimates[i->first];

    //std::cout << "feature " << i->first << std::endl;

    assert (feature.observed);

    typename feature_data_type::const_iterator j, end;

    if (feature.parent_action <= action_id) {
      j = observations.upper_bound(action_id);
      end = observations.end();
    }
    else {
      j = observations.begin();
      end = observations.upper_bound(action_id);
    }

    observation_type observation = feature.estimate;
    actionid_t previous_action = feature.parent_action;

    for (; j != end; ++j) {
      observation = -action_estimates.accumulate(previous_action, j->first) + observation;
      previous_action = j->first;
      const observation_model_type& distribution = j->second;
      result += distribution.log_likelihood(observation);
    }

  }

  return result;
}


template <class SlamData>
void mcmc_slam<SlamData>::update_feature (const featureid_t id) {

  const feature_data_type& observations = data.observation(id);
  feature_data& feature = feature_estimates[id];

  const observation_model_type& distribution
    = observations.find(feature.parent_action)->second;
  
  const observation_type new_estimate = distribution(random);
  const double new_feature_weight = observation_edge_weight(distribution, new_estimate);

  double acceptance_probability = 0.0;

  observation_type new_observation = new_estimate;
  observation_type old_observation = feature.estimate;
  actionid_t previous_action = feature.parent_action;

  typename feature_data_type::const_iterator i = observations.begin();

  for (; i != observations.end(); ++i) {

    if (i->first == feature.parent_action) continue;

    action_type action_difference  = action_estimates.accumulate(previous_action, i->first);
    new_observation = -action_difference + new_observation;
    old_observation = -action_difference + old_observation;
    previous_action = i->first;

    const observation_model_type& distribution = i->second;
    acceptance_probability += distribution.log_likelihood(new_observation);
    acceptance_probability -= distribution.log_likelihood(old_observation);
  }

  acceptance_probability = std::exp(acceptance_probability) * new_feature_weight / feature_weights.at(id);

  if (random.uniform() < acceptance_probability) {
    feature.estimate = new_estimate;
    feature_weights[id] = new_feature_weight;
  }

}

#endif //_SLAM_MCMC_SLAM_HPP
