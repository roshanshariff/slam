#ifndef _SLAM_MCMC_SLAM_HPP
#define _SLAM_MCMC_SLAM_HPP

#include <map>
#include <utility>
#include <cassert>
#include <cmath>

#include <boost/bind.hpp>
#include <boost/signals2.hpp>

#include "slam/slam_data.hpp"
#include "utility/random.hpp"
#include "utility/bitree.hpp"

/** This class implements the MCMC SLAM algorithm. To use it, construct an instance passing in
    a reference to a slam_data object and a random generator. This class will register to listen
    to slam data events. Call mcmc_slam::update to perform the specified number of MCMC-SLAM
    iterations on the data available so far. */

template <class SlamData>
class mcmc_slam {

	typedef SlamData slam_data_type;

	/** The types of action and feature identifiers, respectively. Usually same as size_t. */
	typedef typename slam_data_type::timestep_t timestep_t;
	typedef typename slam_data_type::featureid_t featureid_t;

	/** Convenient redefinitions of typedefs in slam_data_type. Please look there for details. */
	typedef typename slam_data_type::observation_data_type observation_data_type;

	/** The type of probability distributions for action and observation edges, respectively. */
	typedef typename slam_data_type::control_model_type control_model_type;
	typedef typename slam_data_type::observation_model_type  observation_model_type;

	/** The types of labels for action and observation edges, respectively. Action edge labels must
      form a group, with the binary + operator and unary - to form inverses. Additionally, there
      must be a defined addition action_type + observation type which gives the observation as
      seen from the given pose. */
	typedef typename control_model_type::result_type control_type;
	typedef typename observation_model_type::result_type observation_type;

	/** All the information about each feature for MCMC SLAM. Stores a reference to the observations
      of this feature from slam_data, an estimate for this feature, and the time step relative
      to which the estimate is computed. */
	struct feature_estimate {
		const observation_data_type& observations;
		observation_type estimate;
		timestep_t parent_timestep;
		feature_estimate (const observation_data_type& obs, const observation_type& est, timestep_t t)
		: observations(obs), estimate(est), parent_timestep(t) { }
	};

	typedef std::map<featureid_t, feature_estimate> feature_estimates_type;

	/** The slam_data object stores a history of all recorded observations. This class registers as a
      listener to slam_data using the scoped_connection objects, which will automatically be
      disconnected when this class is destroyed. */
	const slam_data_type& data;
	const boost::signals2::scoped_connection m_control_conn;
	const boost::signals2::scoped_connection m_obs_conn;
	const boost::signals2::scoped_connection m_timestep_conn;

	random_source& random;

	/** MCMC Constants. mcmc_steps is the number of iterations per update. The action and observation
      dimensions are a measure of the number of independent parameters represented by each type of
      edge. */
	const int mcmc_steps;

	/** The current action edge labels and the corresponding edge weights. */
	bitree<control_type> state_estimates;
	bitree<double> state_weights;

	/** The current feature edge labels and the corresponding edge weights. */
	feature_estimates_type feature_estimates;
	bitree<double> feature_weights;

public:

	mcmc_slam (const slam_data_type&, random_source&, int num_steps);

	void add_control (timestep_t, const control_model_type&);
	void add_observation (timestep_t, featureid_t, const observation_model_type&);

	void update ();

	const bitree<control_type>& get_state_estimates () const { return state_estimates; }
	std::map<featureid_t, observation_type> get_feature_estimates (const control_type& initial_state) const;

private:

	void update_trajectory (timestep_t);
	double state_change (timestep_t) const;

	void update_feature (featureid_t);

	/** Calculates the edge weight given the probability distribution and label. Uses the formula
      p_e(x) = k * J_e(x)^(-1/k) where p_e is the edge weight, x is the edge label, and J_e is
      the probability density of the label by its own distribution. k is a parameter expressing
      the number of independent dimensions in the probability distribution. */
	template <class Model>
	static double edge_weight (const Model& model, const typename Model::result_type& label) {
		const double importance = Model::vector_dim;
		return importance * std::pow (model.likelihood(label), -1.0/importance);
	}

};


/** Constructs an mcmc_slam object storing a reference to a random_source and a slam_data. Registers
    this class as a listener for slam data events. num_steps is the number of MCMC iterations per
    update, and action_importance and observation_importance are the number of independent
    parameters represented by each action and observation edge respectively. */
template <class SlamData>
mcmc_slam<SlamData>::mcmc_slam (const slam_data_type& data_, random_source& random_, int mcmc_steps_)
: data(data_),
  m_control_conn(data.connect_control_listener(boost::bind(&mcmc_slam::add_control, this, _1, _2))),
  m_obs_conn(data.connect_observation_listener(boost::bind(&mcmc_slam::add_observation, this, _1, _2, _3))),
  m_timestep_conn(data.connect_timestep_listener(boost::bind(&mcmc_slam::update, this))),
  random(random_), mcmc_steps(mcmc_steps_) { }


/** Called by slam_data whenever a new control is added. */
template <class SlamData>
void mcmc_slam<SlamData>::add_control (timestep_t timestep, const control_model_type& control) {
	// This action must immediately follow the previously added action.
	assert (timestep == state_estimates.size());
	assert (timestep == state_weights.size());
	// Use the mean of the distribution as the initial estimate.
	state_estimates.push_back (control.mean());
	// Compute the edge weight based on the distribution and the initial estimate.
	state_weights.push_back (edge_weight (control, state_estimates.back()));
}


/** Called by slam_data whenever a new observation is added. */
template <class SlamData>
void mcmc_slam<SlamData>::add_observation (
		timestep_t timestep, featureid_t feature_id, const observation_model_type& obs
) {
	// Try to insert a new feature_estimate storing a reference to that feature's observations.
	std::pair<typename feature_estimates_type::iterator, bool> result = feature_estimates.insert (
			std::make_pair (feature_id, feature_estimate (
					data.observations(feature_id), obs.mean(), timestep
			))
	);

	// If insertion succeeds (i.e. feature has not been previously observed) initialise the estimate.
	if (result.second) {

		feature_estimate& feature = result.first->second;

		// feature_weights must be large enough to hold this feature's weight.
		if (feature_weights.size() <= feature_id) feature_weights.resize(feature_id+1);

		// Compute the feature's edge weight using the distribution and the initial estimate.
		feature_weights[feature_id] = edge_weight (obs, feature.estimate);

	}
}


// Performs the MCMC SLAM iterations 'mcmc_steps' times.
template <class SlamData>
void mcmc_slam<SlamData>::update () {

	int trajectory_updates = 0;
	int feature_updates = 0;

	for (int i = 0; i < mcmc_steps; ++i) {

		// The total action and feature edge weights.
		const double state_weight = state_weights.accumulate();
		const double feature_weight = feature_weights.accumulate();

		if (state_weight == 0 && feature_weight == 0) return;

		// Randomly select an action or a feature by its weight.
		double state_range, feature_range;
		do {
			state_range = random.uniform() * (state_weight + feature_weight);
			feature_range = state_range - state_weight;
		}
		while (!(state_range < state_weight || feature_range < feature_weight));

		if (state_range < state_weight) {
			++trajectory_updates;
			// Identify the time step in whose interval the randomly selected number lies.
			timestep_t timestep = state_weights.binary_search (state_range);
			update_trajectory (timestep);
		}
		else if (feature_weight > 0) {
			++feature_updates;
			// Identify the feature in whose interval the randomly selected number lies.
			featureid_t feature = feature_weights.binary_search (feature_range);
			update_feature (feature);
		}
	}
	//std::cout << action_updates << " action updates, " << feature_updates << " feature updates\n";
}


/** Resamples the action edge given by id, computes the corresponding acceptance probability,
    and either accepts or rejects the change. */
template <class SlamData>
void mcmc_slam<SlamData>::update_trajectory (const timestep_t timestep) {

	assert (timestep < state_estimates.size());

	// Generate a new estimate for this action as a random sample from the associated distribution,
	// and compute the edge weight for this new estimate.
	const control_type new_estimate = data.control(timestep)(random);
	const double new_weight = edge_weight(data.control(timestep), new_estimate);

	// The old action estimate and corresponding edge weight.
	const control_type old_estimate = state_estimates[timestep];
	const double old_weight = state_weights[timestep];

	double acceptance_probability = 0.0; // log of the acceptance probability.

	// Subtract the log probability of all affected edges, before the action edge is updated.
	acceptance_probability -= state_change(timestep);

	// Update the action edge to the new estimate.
	state_estimates[timestep] = new_estimate;
	state_weights[timestep] = new_weight;

	// Add the log probability of all edges affected, after the action edge is updated.
	acceptance_probability += state_change(timestep);

	// Compute the acceptance probability as the ratio of probabilities after and before the edge
	// was modified, multiplied by the ratio of the new and old edge weight.
	acceptance_probability = std::exp(acceptance_probability) * new_weight / old_weight;

	// If the change was not accepted, roll back the changes by reverting to the old estimate and weight.
	if (random.uniform() >= acceptance_probability) {
		state_estimates[timestep] = old_estimate;
		state_weights[timestep] = old_weight;
	}
}


/** Computes the log probability of all the edges whose labels change when the action edge given by
    action_id is updated. Changing an action splits the spanning tree of the inference graph into
    two subtrees, T1 and T2. T1 is the tree that contains action 0. A feature vertex lies in T1 if
    its parent action is before the one being changed, and it lies in T2 otherwise. If a feature vertex
    lies in T1, then the observation edges affected by the change are those after the change. Otherwise,
    if a feature vertex lies in T2 then the observations made before the change are affected. */
template <class SlamData>
double mcmc_slam<SlamData>::state_change (const timestep_t timestep) const {

	double result = 0.0; // Initialise log probability to zero.

	typename feature_estimates_type::const_iterator i = feature_estimates.begin();

	for (; i != feature_estimates.end(); ++i) { // iterate over all observed features.

		const feature_estimate& feature = i->second;

		typename observation_data_type::const_iterator j, end; // the range of observations to consider

		if (feature.parent_timestep <= timestep) {
			// The feature edge lies in T1 since its parent action is before the modified action edge.
			// The observations to consider are all those in T2, i.e. after the modified action.
			j = feature.observations.upper_bound(timestep);
			end = feature.observations.end();
		}
		else {
			// The feature edge lies in T2 since its parent action is after the modified action edge.
			// The features to consider are all those in T1, i.e. before the modified action.
			j = feature.observations.begin();
			end = feature.observations.upper_bound(timestep);
		}

		// The position of the feature, stored as an observation relative to the observation_base.
		// The initial position is stored as the estimate relative to the parent action.
		observation_type observation = feature.estimate;
		timestep_t observation_base = feature.parent_timestep;

		// Loop over all observations of this feature.
		for (; j != end; ++j) {

			// Each observation is relative to a different action. Computes the state change between the old
			// base of observation (which is initially the parent action) and the current base of the
			// observation.
			const control_type state_change = state_estimates.accumulate (observation_base, j->first);
			observation_base = j->first;

			// Retrieve the distribution associated with this observation edge.
			const observation_model_type& distribution = j->second;

			// Rebase the observation relative to the current action, and accumulate its log likelihood
			// according to the associated distribution.
			observation = -state_change + observation;
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
void mcmc_slam<SlamData>::update_feature (const featureid_t feature_id) {

	// Retrieve the feature estimate to be updated.
	typename feature_estimates_type::iterator feature_iter = feature_estimates.find (feature_id);
	assert (feature_iter != feature_estimates.end());
	feature_estimate& feature = feature_iter->second;

	// Find the associated distribution corresponding to the observation from the parent action of
	// this feature.
	const observation_model_type& distribution
	= feature.observations.find(feature.parent_timestep)->second;

	// Compute a new estimate of this feature's position as a random sample from the associated distribution,
	// and compute a new edge weight accordingly.
	const observation_type new_estimate = distribution(random);
	const double new_feature_weight = edge_weight(distribution, new_estimate);

	// These variables will store the new and old estimates of the feature's position, computed relative to
	// some action. Initially, they are relative to the parent action of the feature.
	observation_type new_observation = new_estimate;
	observation_type old_observation = feature.estimate;
	timestep_t observation_base = feature.parent_timestep;

	double acceptance_probability = 0.0; // log of the acceptance probability.

	// Iterate over all observations of this feature.
	typename observation_data_type::const_iterator i = feature.observations.begin();
	for (; i != feature.observations.end(); ++i) {

		// Ignore the observation edge that is in the spanning tree of the inference graph.
		if (i->first == feature.parent_timestep) continue;

		// Each observation is relative to a different action. Computes the state change between the old
		// base of observation (which is initially the parent action) and the current base of the
		// observation.
		const control_type action_difference  = state_estimates.accumulate (observation_base, i->first);
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
	acceptance_probability = std::exp(acceptance_probability) * new_feature_weight / feature_weights[feature_id];

	// If the change was accepted, replace the old estimate and edge weight by the new ones.
	if (random.uniform() < acceptance_probability) {
		feature.estimate = new_estimate;
		feature_weights[feature_id] = new_feature_weight;
	}
}


/** Returns current estimates of the positions of all observed features. */
template <class SlamData>
std::map<typename SlamData::featureid_t, typename SlamData::observation_model_type::result_type>
mcmc_slam<SlamData>::get_feature_estimates (const control_type& initial_state) const {
	std::map<featureid_t, observation_type> estimates;
	typename feature_estimates_type::const_iterator i = feature_estimates.begin();
	for (; i != feature_estimates.end(); ++i) { // iterate through each feature
		featureid_t feature_id = i->first;
		const feature_estimate& feature = i->second;
		// Rebase the estimate to be relative to action 0.
		estimates.insert (std::make_pair (feature_id,
				initial_state + state_estimates.accumulate(feature.parent_timestep) + feature.estimate
		));
	}
	return estimates;
}


#endif //_SLAM_MCMC_SLAM_HPP
