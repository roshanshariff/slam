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

/** This class implements the MCMC SLAM algorithm. To use it, construct an instance passing in
    a reference to a slam_data object and a random generator. This class will register to listen
    to slam data events. Call mcmc_slam::update to perform the specified number of MCMC-SLAM
    iterations on the data available so far. */

template <class SlamData>
class mcmc_slam {

public:

  typedef SlamData slam_data_type;

  /** The types of action and feature identifiers, respectively. Usually same as size_t. */
  typedef typename slam_data_type::actionid_t actionid_t;
  typedef typename slam_data_type::featureid_t featureid_t;

  /** Convenient redefinitions of typedefs in slam_data_type. Please look there for details. */
  typedef typename slam_data_type::action_data_type action_data_type;
  typedef typename slam_data_type::feature_data_type feature_data_type;
  typedef typename slam_data_type::observation_data_type observation_data_type;

  /** The type of probability distributions for action and observation edges, respectively. */
  typedef typename slam_data_type::action_model_type action_model_type;
  typedef typename slam_data_type::observation_model_type  observation_model_type;

  /** The types of labels for action and observation edges, respectively. Action edge labels must
      form a group, with the binary + operator and unary - to form inverses. Additionally, there
      must be a defined addition action_type + observation type which gives the observation as
      seen from the given pose. */
  typedef typename action_model_type::result_type action_type;
  typedef typename observation_model_type::result_type observation_type;

private:

  /** All the information about each feature for MCMC SLAM. Stores a reference to the observations
      of this feature from slam_data, an estimate for this feature, and the parent action relative
      to which the estimate is computed. */
  struct feature_estimate {
    const feature_data_type& observations;
    observation_type estimate;
    actionid_t parent_action;
    feature_estimate (const feature_data_type& obs)
      : observations(obs), estimate(), parent_action() { }
  };

  typedef std::map<featureid_t, feature_estimate> feature_estimates_type;

  /** MCMC Constants. mcmc_steps is the number of iterations per update. The action and observation
      dimensions are a measure of the number of independent parameters represented by each type of
      edge. */
  const int mcmc_steps;
  const double action_dimensions;
  const double observation_dimensions;

  /** The slam_data object stores a history of all recorded observations. This class registers as a
      listener to slam_data using the scoped_connection objects, which will automatically be
      disconnected when this class is destroyed. */
  const slam_data_type& data;
  const boost::signals2::scoped_connection _act_conn;
  const boost::signals2::scoped_connection _obs_conn;

  /** The current action edge labels and the corresponding edge weights. */
  bitree<action_type> action_estimates;
  bitree<double> action_weights;

  /** The current feature edge labels and the corresponding edge weights. */
  feature_estimates_type feature_estimates;
  bitree<double> feature_weights;

public:

  mcmc_slam (const slam_data_type& _data, int num_steps,
	     double action_importance, double observation_importance);

  void add_action (actionid_t action_id);
  void add_observation (featureid_t feature_id, actionid_t action_id);

  void update (random_source&);

  const bitree<action_type>& get_action_estimates () const { return action_estimates; }
  std::map<featureid_t, observation_type> get_feature_estimates (const action_type& initial_state) const;

private:

  void update_action (random_source&, const actionid_t action_id);
  double action_change (const actionid_t action_id) const;

  void update_feature (random_source&, const featureid_t feature_id);

  /** Calculates the edge weight given the probability distribution and label. Uses the formula
      p_e(x) = k * J_e(x)^(-1/k) where p_e is the edge weight, x is the edge label, and J_e is
      the probability density of the label by its own distribution. k is a parameter expressing
      the number of independent dimensions in the probability distribution. */
  template <class Model, class Label>
  double edge_weight (const Model& model, const Label& label, double importance) {
    return importance * std::pow (model.likelihood(label)+0.0001, -1.0/importance);
  }

  /** Use action_dimensions as the importance for action edges. */
  double action_edge_weight (const action_model_type& model, const action_type& estimate) {
    return edge_weight (model, estimate, action_dimensions);
  }

  /** Use observation dimensions as the label for observation edges. */
  double observation_edge_weight (const observation_model_type& model, const observation_type& estimate) {
    return edge_weight (model, estimate, observation_dimensions);
  }

  /** Retrieve a reference to the given feature estimate by feature id. */
  feature_estimate& get_feature (featureid_t feature_id) {
    typename feature_estimates_type::iterator i = feature_estimates.find (feature_id);
    assert (i != feature_estimates.end());
    return i->second;
  }

  /** Retrieve a const reference to the given feature estimate by feature id. */
  const feature_estimate& get_feature (featureid_t feature_id) const {
    typename feature_estimates_type::const_iterator i = feature_estimates.find (feature_id);
    assert (i != feature_estimates.end());
    return i->second;
  }

};


/** Constructs an mcmc_slam object storing a reference to a random_source and a slam_data. Registers
    this class as a listener for slam data events. num_steps is the number of MCMC iterations per
    update, and action_importance and observation_importance are the number of independent
    parameters represented by each action and observation edge respectively. */
template <class SlamData>
mcmc_slam<SlamData>::mcmc_slam (const slam_data_type& _data, int num_steps,
				double action_importance, double observation_importance)
  : mcmc_steps(num_steps), action_dimensions(action_importance),
    observation_dimensions(observation_importance), data (_data),
    _act_conn (data.connect_action_listener (boost::bind (&mcmc_slam::add_action, this, _1))),
    _obs_conn (data.connect_observation_listener (boost::bind (&mcmc_slam::add_observation, this, _1, _2)))
{ }


/** Called by slam_data whenever a new action is added. */
template <class SlamData>
void mcmc_slam<SlamData>::add_action (actionid_t action_id) {
  // This action must immediately follow the previously added action.
  assert (action_id == action_estimates.size());
  assert (action_id == action_weights.size());
  // Retrieve the probability distribution for this action's labels.
  const action_model_type& action_model = data.action(action_id);
  // Use the mean of the distribution as the initial estimate.
  action_estimates.push_back (action_model.mean());
  // Compute the edge weight based on the distribution and the initial estimate.
  action_weights.push_back (action_edge_weight (action_model, action_estimates[action_id]));
}


/** Called by slam_data whenever a new observation is added. */
template <class SlamData>
void mcmc_slam<SlamData>::add_observation (featureid_t feature_id, actionid_t action_id) {

  // Try to insert a new feature_estimate storing a reference to that feature's observations.
  std::pair<typename feature_estimates_type::iterator, bool> result =
    feature_estimates.insert (std::make_pair (feature_id, feature_estimate (data.observation(feature_id))));

  // If insertion succeeds (i.e. feature has not been previously observed) initialise the estimate.
  if (result.second) {

    feature_estimate& feature = result.first->second;
    feature.parent_action = action_id; // Set parent of the action to be the first time it was seen

    const observation_model_type& distribution = feature.observations.find(action_id)->second;
    feature.estimate = distribution.mean(); // Use the mean of the distribution as the initial estimate

    // feature_weights must be large enough to hold this feature's weight.
    if (feature_weights.size() <= feature_id) feature_weights.resize(feature_id+1);

    // Compute the feature's edge weight using the distribution and the initial estimate.
    feature_weights[feature_id] = observation_edge_weight (distribution, feature.estimate);
  }
}


// Performs the MCMC SLAM iterations 'mcmc_steps' times.
template <class SlamData>
void mcmc_slam<SlamData>::update (random_source& random) {

  int action_updates = 0;
  int feature_updates = 0;

  for (int i = 0; i < mcmc_steps; ++i) {

    // The total action and feature edge weights.
    const double action_weight = action_weights.accumulate();
    const double feature_weight = feature_weights.accumulate();

    if (action_weight == 0 && feature_weight == 0) return;

    // Randomly select an action or a feature by its weight.
    double action_range, feature_range;
    do {
      action_range = random.uniform() * (action_weight + feature_weight);
      feature_range = action_range - action_weight;
    }
    while (!(action_range < action_weight || feature_range < feature_weight));
      
    if (action_range < action_weight) {
      ++action_updates;
      // Identify the action in whose interval the randomly selected number lies.
      actionid_t action_id = action_weights.binary_search (action_range);
      update_action (random, action_id);
    }
    else if (feature_weight > 0) {
      ++feature_updates;
      // Identify the feature in whose interval the randomly selected number lies.
      featureid_t feature_id = feature_weights.binary_search (feature_range);
      update_feature (random, feature_id);
    }
  }
  //std::cout << action_updates << " action updates, " << feature_updates << " feature updates\n";
}


/** Resamples the action edge given by id, computes the corresponding acceptance probability,
    and either accepts or rejects the change. */
template <class SlamData>
void mcmc_slam<SlamData>::update_action (random_source& random, const actionid_t id) {

  assert (id < action_estimates.size());

  // Generate a new estimate for this action as a random sample from the associated distribution,
  // and compute the edge weight for this new estimate.
  const action_type new_estimate = data.action(id)(random);
  const double new_action_weight = action_edge_weight(data.action(id), new_estimate);

  // The old action estimate and corresponding edge weight.
  const action_type old_estimate = action_estimates.at(id);
  const double old_action_weight = action_weights.at(id);

  double acceptance_probability = 0.0; // log of the acceptance probability.

  // Subtract the log probability of all affected edges, before the action edge is updated.
  acceptance_probability -= action_change(id);

  // Update the action edge to the new estimate.
  action_estimates[id] = new_estimate;
  action_weights[id] = new_action_weight;

  // Add the log probability of all edges affected, after the action edge is updated.
  acceptance_probability += action_change(id);

  // Compute the acceptance probability as the ratio of probabilities after and before the edge
  // was modified, multiplied by the ratio of the new and old edge weight.
  acceptance_probability = std::exp(acceptance_probability) * new_action_weight / old_action_weight;

  // If the change was not accepted, roll back the changes by reverting to the old estimate and weight.
  if (random.uniform() >= acceptance_probability) {
    action_estimates[id] = old_estimate;
    action_weights[id] = old_action_weight;
  }
}


/** Computes the log probability of all the edges whose labels change when the action edge given by
    action_id is updated. Changing an action splits the spanning tree of the inference graph into
    two subtrees, T1 and T2. T1 is the tree that contains action 0. A feature vertex lies in T1 if
    its parent action is before the one being changed, and it lies in T2 otherwise. If a feature vertex
    lies in T1, then the observation edges affected by the change are those after the change. Otherwise,
    if a feature vertex lies in T2 then the observations made before the change are affected. */
template <class SlamData>
double mcmc_slam<SlamData>::action_change (const actionid_t action_id) const {

  double result = 0.0; // Initialise log probability to zero.

  typename feature_estimates_type::const_iterator i = feature_estimates.begin();

  for (; i != feature_estimates.end(); ++i) { // iterate over all observed features.

    const feature_estimate& feature = i->second;

    typename feature_data_type::const_iterator j, end; // the range of observations to consider

    if (feature.parent_action <= action_id) {
      // The feature edge lies in T1 since its parent action is before the modified action edge.
      // The observations to consider are all those in T2, i.e. after the modified action.
      j = feature.observations.upper_bound(action_id);
      end = feature.observations.end();
    }
    else {
      // The feature edge lies in T2 since its parent action is after the modified action edge.
      // The features to consider are all those in T1, i.e. before the modified action.
      j = feature.observations.begin();
      end = feature.observations.upper_bound(action_id);
    }

    // The position of the feature, stored as an observation relative to the observation_base.
    // The initial position is stored as the estimate relative to the parent action.
    observation_type observation = feature.estimate;
    actionid_t observation_base = feature.parent_action;

    // Loop over all observations of this feature.
    for (; j != end; ++j) {

      // Each observation is relative to a different action. Computes the state change between the old
      // base of observation (which is initially the parent action) and the current base of the
      // observation.
      const action_type action_difference = action_estimates.accumulate (observation_base, j->first);
      observation_base = j->first;

      // Retrieve the distribution associated with this observation edge.
      const observation_model_type& distribution = j->second;

      // Rebase the observation relative to the current action, and accumulate its log likelihood
      // according to the associated distribution.
      observation = -action_difference + observation;
      result += distribution.log_likelihood(observation);
    }
  }

  return result;
}


/** Resamples the observation edge given by id, computes the corresponding acceptance probability,
    and either accepts or rejects the change. Changing an observation edge splits the spanning tree
    of the inference graph into two trees, T1 and T2. T1 contains all the action vertices and all other
    features, whereas T2 contains only the feature being updated. The affected edges are all observations
    of the feature whose estimate is being modified. */
template <class SlamData>
void mcmc_slam<SlamData>::update_feature (random_source& random, const featureid_t id) {

  // Retrieve the feature estimate to be updated.
  feature_estimate& feature = get_feature (id);

  // Find the associated distribution corresponding to the observation from the parent action of
  // this feature.
  const observation_model_type& distribution
    = feature.observations.find(feature.parent_action)->second;
  
  // Compute a new estimate of this feature's position as a random sample from the associated distribution,
  // and compute a new edge weight accordingly.
  const observation_type new_estimate = distribution(random);
  const double new_feature_weight = observation_edge_weight(distribution, new_estimate);

  // These variables will store the new and old estimates of the feature's position, computed relative to
  // some action. Initially, they are relative to the parent action of the feature.
  observation_type new_observation = new_estimate;
  observation_type old_observation = feature.estimate;
  actionid_t observation_base = feature.parent_action;

  double acceptance_probability = 0.0; // log of the acceptance probability.

  // Iterate over all observations of this feature.
  typename feature_data_type::const_iterator i = feature.observations.begin();
  for (; i != feature.observations.end(); ++i) {

    // Ignore the observation edge that is in the spanning tree of the inference graph.
    if (i->first == feature.parent_action) continue;

    // Each observation is relative to a different action. Computes the state change between the old
    // base of observation (which is initially the parent action) and the current base of the
    // observation.
    const action_type action_difference  = action_estimates.accumulate (observation_base, i->first);
    observation_base = i->first;

    // Retrieve the distribution associated with this observation edge.
    const observation_model_type& distribution = i->second;

    // Update the new observation by rebasing it relative to the current action and multiply the
    // corresponding probability to the running total (i.e. add the log probability).
    new_observation = -action_difference + new_observation;
    acceptance_probability += distribution.log_likelihood(new_observation);

    // Update the old observation by rebasing it relative to the current action and divide the
    // corresponding probability from the running total (i.e. subtract the log probability).
    old_observation = -action_difference + old_observation;
    acceptance_probability -= distribution.log_likelihood(old_observation);
  }

  // Compute the acceptance probability as the ratio of probabilities of the new and the old estimates,
  // multiplied by the ratio between the new and the old edge weights.
  acceptance_probability = std::exp(acceptance_probability) * new_feature_weight / feature_weights.at(id);

  // If the change was accepted, replace the old estimate and edge weight by the new ones.
  if (random.uniform() < acceptance_probability) {
    feature.estimate = new_estimate;
    feature_weights[id] = new_feature_weight;
  }
}


/** Returns current estimates of the positions of all observed features. */
template <class SlamData>
std::map<typename SlamData::featureid_t, typename SlamData::observation_model_type::result_type>
mcmc_slam<SlamData>::get_feature_estimates (const action_type& initial_state) const {
  std::map<featureid_t, observation_type> estimates;
  typename feature_estimates_type::const_iterator i = feature_estimates.begin();
  for (; i != feature_estimates.end(); ++i) { // iterate through each feature
    featureid_t feature_id = i->first;
    const feature_estimate& feature = i->second;
    // Compute the state of the robot at the time of this feature's parent action.
    action_type state = initial_state + action_estimates.accumulate(feature.parent_action);
    // Rebase the estimate to be relative to action 0.
    estimates.insert (std::make_pair (feature_id, state + feature.estimate));
  }
  return estimates;
}


#endif //_SLAM_MCMC_SLAM_HPP
