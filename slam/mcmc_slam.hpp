#ifndef _SLAM_MCMC_SLAM_HPP
#define _SLAM_MCMC_SLAM_HPP

#include <map>
#include <utility>
#include <cassert>
#include <cmath>
#include <boost/bind.hpp>
#include <boost/signals2.hpp>

#include "slam/slam_data.hpp"
#include "utilities/random.hpp"
#include "utilities/bitree.hpp"


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

  struct feature_estimate {
    actionid_t parent_action;
    observation_type estimate;
    feature_estimate () : parent_action(0) { }
  };

  typedef std::map<featureid_t, feature_estimate> feature_estimates_type;

  random_source& random;

  const int mcmc_steps;
  const double action_dimensions;
  const double observation_dimensions;

  const slam_data_type& data;
  const boost::signals2::scoped_connection _act_conn;
  const boost::signals2::scoped_connection _obs_conn;

  bitree<action_type> action_estimates;
  bitree<double> action_weights;

  feature_estimates_type feature_estimates;
  bitree<double> feature_weights;

public:

  mcmc_slam (random_source& _random, const slam_data_type& _data, int num_steps,
	     double action_importance, double observation_importance);

  void add_action (actionid_t action_id);
  void add_observation (featureid_t feature_id, actionid_t action_id);

  void update ();

  const bitree<action_type>& get_action_estimates () const { return action_estimates; }
  std::map<featureid_t, observation_type> get_feature_estimates () const;

private:

  void update_action (const actionid_t action_id);
  double action_change (const actionid_t action_id) const;

  void update_feature (const featureid_t feature_id);

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

  feature_estimate& get_feature (featureid_t feature_id) {
    typename feature_estimates_type::iterator i = feature_estimates.find (feature_id);
    assert (i != feature_estimates.end());
    return i->second;
  }

  const feature_estimate& get_feature (featureid_t feature_id) const {
    typename feature_estimates_type::const_iterator i = feature_estimates.find (feature_id);
    assert (i != feature_estimates.end());
    return i->second;
  }

};


template <class SlamData>
mcmc_slam<SlamData>::mcmc_slam (random_source& _random, const slam_data_type& _data, int num_steps,
				double action_importance, double observation_importance)
  : random(_random), mcmc_steps(num_steps), action_dimensions(action_importance),
    observation_dimensions(observation_importance), data (_data),
    _act_conn (data.connect_action_listener (boost::bind (&mcmc_slam::add_action, this, _1))),
    _obs_conn (data.connect_observation_listener (boost::bind (&mcmc_slam::add_observation, this, _1, _2)))
{ }


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

  std::pair<typename feature_estimates_type::iterator, bool> result =
    feature_estimates.insert (std::make_pair (feature_id, feature_estimate ()));

  if (result.second) {

    feature_estimate& feature = result.first->second;
    feature.parent_action = action_id;

    const observation_model_type& distribution = data.observation(feature_id).find(action_id)->second;
    feature.estimate = distribution.mean();

    if (feature_weights.size() <= feature_id) feature_weights.resize(feature_id+1);
    feature_weights[feature_id] = observation_edge_weight (distribution, feature.estimate);
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
      update_action (action_id);
    }
    else if (feature_weight > 0) {
      ++feature_updates;
      featureid_t feature_id = feature_weights.binary_search (feature_range);
      update_feature (feature_id);
    }
  }
  //std::cout << action_updates << " action updates, " << feature_updates << " feature updates\n";
}

template <class SlamData>
void mcmc_slam<SlamData>::update_action (const actionid_t id) {

  assert (id < action_estimates.size());

  const action_type new_estimate = data.action(id)(random);
  const double new_action_weight = action_edge_weight(data.action(id), new_estimate);

  const action_type old_estimate = action_estimates.at(id);
  const double old_action_weight = action_weights.at(id);

  double acceptance_probability = 0.0;

  acceptance_probability -= action_change(id);
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

  typename feature_estimates_type::const_iterator i = feature_estimates.begin();
  typename observation_data_type::const_iterator j = data.observations().begin();

  for (; i != feature_estimates.end() && j != data.observations().end(); ++i, ++j) {

    assert (i->first == j->first);

    const feature_estimate& feature = i->second;
    const feature_data_type& observations = j->second;

    typename feature_data_type::const_iterator k, end;

    if (feature.parent_action <= action_id) {
      k = observations.upper_bound(action_id);
      end = observations.end();
    }
    else {
      k = observations.begin();
      end = observations.upper_bound(action_id);
    }

    observation_type observation = feature.estimate;
    actionid_t observation_base = feature.parent_action;

    for (; k != end; ++k) {

      const action_type action_difference = action_estimates.accumulate (observation_base, k->first);
      observation_base = k->first;

      const observation_model_type& distribution = k->second;

      observation = -action_difference + observation;
      result += distribution.log_likelihood(observation);
    }
  }

  return result;
}


template <class SlamData>
void mcmc_slam<SlamData>::update_feature (const featureid_t id) {

  feature_estimate& feature = get_feature (id);
  const feature_data_type& observations = data.observation (id);

  const observation_model_type& distribution
    = observations.find(feature.parent_action)->second;
  
  const observation_type new_estimate = distribution(random);
  const double new_feature_weight = observation_edge_weight(distribution, new_estimate);

  observation_type new_observation = new_estimate;
  observation_type old_observation = feature.estimate;
  actionid_t observation_base = feature.parent_action;

  double acceptance_probability = 0.0;

  typename feature_data_type::const_iterator i = observations.begin();
  for (; i != observations.end(); ++i) {

    if (i->first == feature.parent_action) continue;

    const action_type action_difference  = action_estimates.accumulate (observation_base, i->first);
    observation_base = i->first;

    const observation_model_type& distribution = i->second;

    new_observation = -action_difference + new_observation;
    acceptance_probability += distribution.log_likelihood(new_observation);

    old_observation = -action_difference + old_observation;
    acceptance_probability -= distribution.log_likelihood(old_observation);
  }

  acceptance_probability = std::exp(acceptance_probability) * new_feature_weight / feature_weights.at(id);

  if (random.uniform() < acceptance_probability) {
    feature.estimate = new_estimate;
    feature_weights[id] = new_feature_weight;
  }
}


template <class SlamData>
std::map<typename SlamData::featureid_t, typename SlamData::observation_model_type::result_type>
mcmc_slam<SlamData>::get_feature_estimates () const {
  std::map<featureid_t, observation_type> estimates;
  typename feature_estimates_type::const_iterator i = feature_estimates.begin();
  for (; i != feature_estimates.end(); ++i) {
    featureid_t feature_id = i->first;
    const feature_estimate& feature = i->second;
    action_type parent_action = action_estimates.accumulate(feature.parent_action);
    estimates.insert (std::make_pair (feature_id, parent_action + feature.estimate));
  }
  return estimates;
}


#endif //_SLAM_MCMC_SLAM_HPP
