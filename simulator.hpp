#ifndef _SIMULATOR_HPP
#define _SIMULATOR_HPP

#include <map>
#include <tr1/functional>

#include "slam/slam_data.hpp"
#include "slam/mcmc_slam.hpp"
#include "utilities/random.hpp"
#include "utilities/bitree.hpp"


template <class StateModelBuilder, class ObservationModelBuilder>
class simulator {

public:

  typedef typename StateModelBuilder::result_type state_model_type;
  typedef typename ObservationModelBuilder::result_type observation_model_type;

  typedef typename state_model_type::result_type state_type;
  typedef typename observation_model_type::result_type observation_type;
  typedef std::tr1::function<bool (observation_type)> observation_predicate_type;

  typedef typename StateModelBuilder::second_argument_type control_type;
  typedef std::tr1::function<control_type (double, bitree<state_type>)> controller_type;

  typedef slam_data<state_model_type, observation_model_type> slam_data_type;
  typedef std::map<typename slam_data_type::featureid_t, observation_type> map_type;

private:

  const map_type& landmarks;
  controller_type controller;
  state_type initial_state;

  StateModelBuilder state_model_builder;
  ObservationModelBuilder observation_model_builder;
  observation_predicate_type observation_predicate;

  slam_data_type data;
  mcmc_slam<slam_data_type> mcmc;

  bitree<state_type> state, expected_state;

  double simulation_time;
  unsigned long num_steps;
  unsigned long num_observations;

  void add_observations (random_source&, const state_type& state);

public:

  simulator (const map_type& _landmarks,
	     const controller_type& _controller,
	     const state_type& _initial_state,
	     const StateModelBuilder& _state_model_builder,
	     const ObservationModelBuilder& _observation_model_builder,
	     const observation_predicate_type& _observation_predicate,
	     unsigned int mcmc_steps, double state_dim, double obs_dim)
    : landmarks(_landmarks),
      controller(_controller),
      initial_state(_initial_state),
      state_model_builder(_state_model_builder),
      observation_model_builder(_observation_model_builder),
      observation_predicate(_observation_predicate),
      mcmc (data, mcmc_steps, state_dim, obs_dim),
      simulation_time(0.0), num_steps(0), num_observations(0)
  { }

  void operator() (random_source&, double dt);

  const map_type& get_landmarks () const { return landmarks; }
  const state_type& get_initial_state () const { return initial_state; }
  const bitree<state_type>& get_state () const { return state; }
  const bitree<state_type>& get_expected_state () const { return expected_state; }
  const bitree<state_type>& get_estimated_state () const { return mcmc.get_action_estimates(); }
  map_type get_feature_estimates () const { return mcmc.get_feature_estimates (initial_state); }

  double get_simulation_time () const { return simulation_time; }
  unsigned long get_num_steps () const { return num_steps; }
  unsigned long get_num_observations () const { return num_observations; }
  double get_observations_per_step () const { return (double)num_observations/num_steps; }

};


template <class StateModelBuilder, class ObservationModelBuilder>
void simulator<StateModelBuilder, ObservationModelBuilder>
::operator() (random_source& random, double dt) {

  add_observations (random, get_initial_state() + get_state().accumulate());
  mcmc.update(random);
  ++num_steps;
  simulation_time += dt;

  control_type control = controller (dt, get_state() /* get_estimated_state() */);
  state_model_type state_change_model = state_model_builder (dt, control);

  state.push_back (state_change_model (random));
  expected_state.push_back (state_change_model.mean());
  data.add_action (state_change_model);
}


template <class StateModelBuilder, class ObservationModelBuilder>
void simulator<StateModelBuilder, ObservationModelBuilder>
::add_observations (random_source& random, const state_type& state) {

  typename map_type::const_iterator i = landmarks.begin();
  for (; i != landmarks.end(); ++i) {

    const observation_type& feature = i->second;
    observation_type observation = observation_model_builder(-state+feature)(random);

    if (observation_predicate (observation)) {
      data.add_observation (i->first, observation_model_builder(observation));
      ++num_observations;
    }
  }
}


#endif //_SIMULATOR_HPP
