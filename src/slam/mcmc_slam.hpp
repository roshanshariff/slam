#ifndef _SLAM_MCMC_SLAM_HPP
#define _SLAM_MCMC_SLAM_HPP

#include <algorithm>
#include <cassert>
#include <cmath>
#include <iostream>
#include <map>
#include <memory>
#include <utility>
#include <vector>

#include <boost/program_options.hpp>
#include <boost/range/adaptor/map.hpp>
#include <boost/range/sub_range.hpp>

#include "slam/interfaces.hpp"
#include "slam/slam_data.hpp"
#include "utility/bitree.hpp"
#include "utility/flat_map.hpp"
#include "utility/random.hpp"
#include "utility/utility.hpp"

#include "main.hpp"

namespace slam {

template <class ControlModel, class ObservationModel>
class fastslam_mcmc;

/** This class implements the MCMC SLAM algorithm. To use it,
 construct an instance passing in a reference to a slam_data object
 and a random generator. This class will register to listen to slam
 data events. Call mcmc_slam::update to perform the specified number
 of MCMC-SLAM iterations on the data available so far. */

template <class ControlModel, class ObservationModel>
class mcmc_slam : public slam_result_of<ControlModel, ObservationModel> {

  friend class fastslam_mcmc<ControlModel, ObservationModel>;

  using slam_result_type = slam_result_of<ControlModel, ObservationModel>;
  using slam_data_type = slam_data<ControlModel, ObservationModel>;

  /** The types of labels for state and feature edges,
   respectively. State edge labels must form a group, with the binary
   + operator and unary - to form inverses.  Additionally, this group
   must act on feature_type by the operation state + feature. */

  using state_type = typename slam_result_type::state_type;
  using feature_type = typename slam_result_type::feature_type;
  using trajectory_type = typename slam_result_type::trajectory_type;
  using feature_map_type = typename slam_result_type::feature_map_type;

  /** Types used for storing observation and control data by slam_data */

  using feature_observations = typename slam_data_type::feature_observations;
  using feature_iterator = typename slam_data_type::feature_iterator;

  using feature_obs_range = boost::sub_range<const feature_observations>;

  /** For each observed feature, store a pointer to the feature's
   observations, the time step relative to which the feature estimate
   is stored, and the estimate itself. */

  class feature_estimate {

    feature_iterator feature_iter;

  public:
    feature_estimate(feature_iterator f, timestep_type t,
                     const feature_type& est)
        : feature_iter(f), parent_timestep(t), estimate(est) {}

    auto id() const -> featureid_type { return feature_iter->first; }
    auto observations() const -> const feature_observations& {
      return feature_iter->second;
    }

    timestep_type parent_timestep;
    feature_type estimate;
  };

  /** Descriptions of the state and feature edges to be used with update
   * (EdgeType&&) */

  struct state_edge {

    const timestep_type timestep;

    const ControlModel& distribution;
    typename utility::bitree<state_type>::reference estimate;
    typename utility::bitree<double>::reference weight;

    state_edge(mcmc_slam& mcmc, timestep_type t)
        : timestep(t), distribution(mcmc.data->control(t)),
          estimate(mcmc.state_estimates[t]), weight(mcmc.state_weights[t]) {}
  };

  struct feature_edge {

    const feature_estimate& feature;

    const ObservationModel& distribution;
    feature_type& estimate;
    typename utility::bitree<double>::reference weight;

    feature_edge(mcmc_slam& mcmc, std::size_t i)
        : feature(mcmc.feature_estimates[i]),
          distribution(feature.observations().at(feature.parent_timestep)),
          estimate(mcmc.feature_estimates[i].estimate),
          weight(mcmc.feature_weights[i]) {}
  };

  /** Private data members */

  const std::shared_ptr<const slam_data_type> data;
  std::shared_ptr<slam_result_type> initialiser;

  random_source random;

  trajectory_type state_estimates;
  utility::bitree<double> state_weights;

  std::vector<feature_estimate> feature_estimates;
  utility::bitree<double> feature_weights;

  // Map feature id to index in feature_estimates
  std::map<featureid_type, std::size_t> feature_index;

  // Cache of map estimate in the form required by get_feature_map()
  mutable feature_map_type map_estimate;

  // The next value expected by timestep
  timestep_type next_timestep;

  // Log likelihood of the current trajectory and map estimate
  double log_likelihood = 0;

  /** Private member functions */

  void add_state_edge();
  void add_feature_edge(const typename slam_data_type::observation_info&);

  template <class EdgeType>
  bool update(EdgeType&&, bool use_edge_weight);

  double edge_log_likelihood_ratio(const state_edge&, const state_type&) const;
  double edge_log_likelihood_ratio(const feature_edge&,
                                   const feature_type&) const;
  double obs_likelihood_ratio(const feature_estimate&,
                              timestep_type obs_timestep,
                              feature_obs_range obs_range,
                              feature_type new_obs) const;

  bool initialiser_available(timestep_type t) const {
    return initialiser && (initialiser->timestep(t), true);
  }

  static double edge_log_weight(double edge_log_likelihood, double edge_dim) {
    return std::log(edge_dim) - edge_log_likelihood / edge_dim;
  }

public:
  mcmc_slam(const decltype(data)& data, unsigned int seed)
      : data(data), random(seed) {}

  mcmc_slam(const mcmc_slam&) = delete;
  mcmc_slam& operator=(const mcmc_slam&) = delete;

  static boost::program_options::options_description program_options();

  double get_log_likelihood() const { return log_likelihood; }

  void set_initialiser(const decltype(initialiser)& init) {
    initialiser = init;
  }

  bool update();

  // Overridden virtual member functions of slam::slam_result

  virtual void timestep(timestep_type) override;

  virtual timestep_type current_timestep() const override {
    return timestep_type(state_estimates.size());
  }

  virtual feature_type get_feature(featureid_type id) const override {
    const feature_estimate& f = feature_estimates[feature_index.at(id)];
    return this->get_state(f.parent_timestep) + f.estimate;
  }

  virtual const trajectory_type& get_trajectory() const override {
    return state_estimates;
  }

  virtual const feature_map_type& get_feature_map() const override;

  class updater : public timestep_listener {

    std::shared_ptr<mcmc_slam> instance;
    unsigned int steps, end_steps;

  public:
    updater(const decltype(instance)& instance, unsigned int steps = 0,
            unsigned int end_steps = 0)
        : instance(instance), steps(steps), end_steps(end_steps) {}

    updater(const decltype(instance)&,
            const boost::program_options::variables_map&);

    virtual void timestep(timestep_type t) override {
      instance->timestep(t);
      for (unsigned int i = 0; i < steps; ++i) instance->update();
    }

    virtual void completed() override {
      instance->completed();
      for (unsigned int i = 0; i < end_steps; ++i) instance->update();
    }
  };
};

} // namespace slam

template <class ControlModel, class ObservationModel>
void slam::mcmc_slam<ControlModel, ObservationModel>::add_state_edge() {

  const ControlModel& control = data->control(current_timestep());
  const auto& proposal = control.proposal();

  const state_type estimate =
      initialiser_available(current_timestep() + 1)
          ? -initialiser->get_state(current_timestep())
                + initialiser->get_state(current_timestep() + 1)
          : proposal.initial_value(random);

  state_estimates.push_back(estimate);
  state_weights.push_back(std::exp(
      edge_log_weight(proposal.log_likelihood(estimate), proposal.vector_dim)));
  log_likelihood += control.log_likelihood(ControlModel::observe(estimate));
  assert(std::isfinite(log_likelihood));

  assert(state_estimates.size() == state_weights.size());
}

template <class ControlModel, class ObservationModel>
void slam::mcmc_slam<ControlModel, ObservationModel>::add_feature_edge(
    const typename slam_data_type::observation_info& obs) {

  const ObservationModel& observation = obs.observation();
  const auto& proposal = observation.proposal();

  const feature_type estimate =
      initialiser_available(current_timestep())
          ? -initialiser->get_state(current_timestep())
                + initialiser->get_feature(obs.id())
          : proposal.initial_value(random);

  feature_estimates.emplace_back(obs.iterator(), current_timestep(), estimate);
  feature_weights.push_back(std::exp(
      edge_log_weight(proposal.log_likelihood(estimate), proposal.vector_dim)));

  log_likelihood +=
      observation.log_likelihood(ObservationModel::observe(estimate));
  assert(std::isfinite(log_likelihood));

  assert(feature_estimates.size() == feature_weights.size());
}

template <class ControlModel, class ObservationModel>
void slam::mcmc_slam<ControlModel, ObservationModel>::timestep(
    const timestep_type timestep) {

  assert(timestep <= data->current_timestep());
  using namespace boost::adaptors;

  while (next_timestep <= timestep) {

    if (next_timestep > 0) add_state_edge();

    for (const auto& obs : values(data->observations_at(next_timestep))) {

      const auto insertion =
          feature_index.emplace(obs.id(), feature_estimates.size());
      const bool inserted = insertion.second;

      if (inserted) {
        add_feature_edge(obs);
        map_estimate.clear();
      }
      else {

        const auto t = next_timestep;
        const ObservationModel& observation = obs.observation();
        const auto& proposal = observation.proposal();

        const auto fi = insertion.first->second;
        feature_estimate& f = feature_estimates[fi];

        const state_type delta =
            state_estimates.accumulate(t, f.parent_timestep);
        const feature_type estimate = delta + f.estimate;

        log_likelihood +=
            observation.log_likelihood(ObservationModel::observe(estimate));
        assert(std::isfinite(log_likelihood));

        if (observation.more_accurate_than(
                f.observations().at(f.parent_timestep))) {

          // std::cout << "Changing parent of feature " << size_t(f.id()) << "
          // to timestep "
          //<< size_t(t) << "... " << std::flush;

          f.parent_timestep = t;
          f.estimate = estimate;

          if (!update(feature_edge(*this, fi), false)) {
            feature_weights[fi] = std::exp(edge_log_weight(
                proposal.log_likelihood(estimate), proposal.vector_dim));
          }
          else {
            // std::cout << "updated ";
          }
          // std::cout << "weight is " << double(feature_weights[fi]) <<
          // std::endl;
        }
      }
    }

    assert(next_timestep == current_timestep());
    ++next_timestep;
  }
}

// Performs the MCMC SLAM update step
template <class ControlModel, class ObservationModel>
auto slam::mcmc_slam<ControlModel, ObservationModel>::update() -> bool {

  const double state_weight = state_weights.accumulate();
  const double feature_weight = feature_weights.accumulate();

  if (state_weight == 0 && feature_weight == 0) return false;

  if ((state_weight + feature_weight) * random.uniform() < state_weight) {

    timestep_type timestep;
    do {
      timestep = timestep_type(
          state_weights.binary_search(state_weight * random.uniform()));
    } while (timestep >= current_timestep()
             && ((std::cerr << "stuck in timestep select loop\n"), true));

    return update(state_edge(*this, timestep), true);
  }
  else {

    std::size_t index;
    do {
      index = feature_weights.binary_search(feature_weight * random.uniform());
    } while (index >= feature_estimates.size()
             && ((std::cerr << "stuck in landmark select loop\n"), true));

    return update(feature_edge(*this, index), true);
  }
}

template <class ControlModel, class ObservationModel>
template <class EdgeType>
auto slam::mcmc_slam<ControlModel, ObservationModel>::update(
    EdgeType&& edge, bool use_edge_weight) -> bool {

  const auto& proposal = edge.distribution.proposal();
  const auto proposed = proposal(random);

  const double new_proposal_log_likelihood = proposal.log_likelihood(proposed);
  const double old_proposal_log_likelihood =
      proposal.log_likelihood(edge.estimate);
  const double proposal_log_ratio =
      new_proposal_log_likelihood - old_proposal_log_likelihood;
  assert(std::isfinite(proposal_log_ratio));

  const double log_ratio = edge_log_likelihood_ratio(edge, proposed);
  assert(std::isfinite(log_ratio));

  const double new_log_weight =
      edge_log_weight(new_proposal_log_likelihood, proposal.vector_dim);
  const double new_weight = std::exp(new_log_weight);
  assert(std::isfinite(new_weight));

  double normaliser = 1.0;
  double accept_log_ratio = log_ratio - proposal_log_ratio;

  if (use_edge_weight) {
    const double old_log_weight =
        edge_log_weight(old_proposal_log_likelihood, proposal.vector_dim);
    const double old_weight = std::exp(old_log_weight);
    const double weight_sum =
        state_weights.accumulate() + feature_weights.accumulate();
    normaliser += (new_weight - old_weight) / weight_sum;
    accept_log_ratio += new_log_weight - old_log_weight;
  }

  // std::cout << std::min(1.0,std::exp(accept_log_ratio)/normaliser) << '\n';

  if (normaliser * random.uniform() < std::exp(accept_log_ratio)) {
    edge.estimate = proposed;
    edge.weight = new_weight;
    log_likelihood += log_ratio;
    map_estimate.clear();
    return true;
  }
  else {
    return false;
  }
}

/** Computes the log probability of all the edges whose labels change when the
 action edge given by
 action_id is updated. Changing an action splits the spanning tree of the
 inference graph into
 two subtrees, T1 and T2. T1 is the tree that contains action 0. A feature
 vertex lies in T1 if
 its parent action is before the one being changed, and it lies in T2 otherwise.
 If a feature vertex
 lies in T1, then the observation edges affected by the change are those after
 the change. Otherwise,
 if a feature vertex lies in T2 then the observations made before the change are
 affected. */
template <class ControlModel, class ObservationModel>
auto slam::mcmc_slam<ControlModel, ObservationModel>::edge_log_likelihood_ratio(
    const state_edge& edge, const state_type& proposed) const -> double {

  double log_ratio =
      +edge.distribution.log_likelihood(ControlModel::observe(proposed))
      - edge.distribution.log_likelihood(ControlModel::observe(edge.estimate));

  for (const auto& f :
       feature_estimates) { // iterate over all observed features.

    auto middle = f.observations().upper_bound(edge.timestep);

    // Check whether the feature is in T2, and if so consider states before t.
    // Otherwise
    // consider states after t.

    if (edge.timestep < f.parent_timestep) {
      const state_type delta =
          state_estimates.accumulate(edge.timestep + 1, f.parent_timestep);
      log_ratio += obs_likelihood_ratio(f, edge.timestep,
                                        {f.observations().begin(), middle},
                                        proposed + delta + f.estimate);
    }
    else {
      const state_type delta =
          state_estimates.accumulate(edge.timestep, f.parent_timestep);
      log_ratio += obs_likelihood_ratio(f, edge.timestep + 1,
                                        {middle, f.observations().end()},
                                        -proposed + delta + f.estimate);
    }
  }

  return log_ratio;
}

template <class ControlModel, class ObservationModel>
auto slam::mcmc_slam<ControlModel, ObservationModel>::edge_log_likelihood_ratio(
    const feature_edge& edge, const feature_type& proposed) const -> double {

  return obs_likelihood_ratio(edge.feature, edge.feature.parent_timestep,
                              edge.feature.observations(), proposed);
}

template <class ControlModel, class ObservationModel>
auto slam::mcmc_slam<ControlModel, ObservationModel>::obs_likelihood_ratio(
    const feature_estimate& feature, timestep_type obs_timestep,
    feature_obs_range obs_range, feature_type new_obs) const -> double {

  double log_ratio = 0.0;

  feature_type old_obs = feature.estimate;
  old_obs = state_estimates.accumulate(obs_timestep, feature.parent_timestep)
            + old_obs;

  for (const auto& obs : obs_range) {

    const state_type state_change =
        state_estimates.accumulate(obs.first, obs_timestep);
    new_obs = state_change + new_obs;
    old_obs = state_change + old_obs;

    const ObservationModel& obs_model = obs.second;
    const double new_log_likelihood =
        obs_model.log_likelihood(ObservationModel::observe(new_obs));
    const double old_log_likelihood =
        obs_model.log_likelihood(ObservationModel::observe(old_obs));

    obs_timestep = obs.first;
    log_ratio += new_log_likelihood - old_log_likelihood;
  }

  return log_ratio;
}

template <class ControlModel, class ObservationModel>
auto slam::mcmc_slam<ControlModel, ObservationModel>::get_feature_map() const
    -> const feature_map_type& {

  if (map_estimate.size() != feature_estimates.size()) {

    map_estimate.clear();
    map_estimate.reserve(feature_estimates.size());

    for (const auto& id_index : feature_index) {
      const feature_estimate& f = feature_estimates[id_index.second];
      const feature_type obs = this->get_state(f.parent_timestep) + f.estimate;
      map_estimate.emplace_hint(map_estimate.end(), id_index.first, obs);
    }
  }

  assert(map_estimate.size() == feature_estimates.size());
  return map_estimate;
}

template <class ControlModel, class ObservationModel>
auto slam::mcmc_slam<ControlModel, ObservationModel>::program_options()
    -> boost::program_options::options_description {
  namespace po = boost::program_options;
  po::options_description options("MCMC-SLAM Parameters");
  options.add_options() //
      ("mcmc-slam-seed", po::value<unsigned int>(),
       "MCMC-SLAM random seed") //
      ("mcmc-steps", po::value<unsigned int>()->default_value(0),
       "MCMC steps per time step") //
      ("mcmc-end-steps", po::value<unsigned int>()->default_value(0),
       "MCMC steps after simulation");
  return options;
}

template <class ControlModel, class ObservationModel>
slam::mcmc_slam<ControlModel, ObservationModel>::updater::updater(
    const decltype(instance)& instance,
    const boost::program_options::variables_map& options)
    : updater(instance, options["mcmc-steps"].as<unsigned int>(),
              options["mcmc-end-steps"].as<unsigned int>()) {}

extern template class slam::mcmc_slam<control_model_type,
                                      observation_model_type>;

#endif //_SLAM_MCMC_SLAM_HPP
