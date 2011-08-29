#ifndef _SIMULATOR_HPP
#define _SIMULATOR_HPP

#include <boost/bind.hpp>

#include "slam/slam_data.hpp"
#include "slam/mcmc_slam.hpp"
#include "utility/random.hpp"
#include "utility/bitree.hpp"


template <class Controller, class Sensor>
struct simulator {

	typedef Controller controller_type;
	typedef typename controller_type::model_type control_model_type;
	typedef typename control_model_type::result_type state_type;

	typedef Sensor sensor_type;
	typedef typename sensor_type::model_type observation_model_type;
	typedef typename observation_model_type::result_type observation_type;

	typedef slam_data<control_model_type, observation_model_type> slam_data_type;

	controller_type& controller;
	sensor_type& sensor;
	random_source& random;

	slam_data_type data;

	bitree<state_type> state;

	simulator (controller_type& controller_, sensor_type& sensor_, random_source& random_)
	: controller(controller_), sensor(sensor_), random(random_) { }

	void operator() ();

	state_type current_state () const { return controller.initial_state() + state.accumulate(); }
};


template <class Controller, class Sensor>
void simulator<Controller, Sensor>::operator() () {

	sensor.sense(current_state(), boost::bind(&slam_data_type::add_observation, &data, _1, _2), random);

	while (!controller.finished()) {

		control_model_type control = controller.control(current_state());
		state.push_back(control(random));
		data.add_control(control);

		sensor.sense(current_state(), boost::bind(&slam_data_type::add_observation, &data, _1, _2), random);

		data.timestep();
	}
}


#endif //_SIMULATOR_HPP
