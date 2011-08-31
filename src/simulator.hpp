#ifndef _SIMULATOR_HPP
#define _SIMULATOR_HPP

#include <string>
#include <cstdio>

#include <boost/bind.hpp>
#include <boost/filesystem.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/signals2.hpp>

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


template <class SlamImpl>
class print_map {

	typedef typename SlamImpl::control_type control_type;
	typedef typename SlamImpl::observation_type observation_type;
	typedef typename SlamImpl::featureid_t featureid_t;

	control_type initial_state;

	FILE* output;

public:

	print_map (const char* filename, const SlamImpl& slam, control_type initial_state_)
	: initial_state(initial_state_), output(std::fopen(filename, "w"))
	{
		if (output) slam.foreach_feature_estimate(boost::bind(&print_map::print_feature, this, _1, _2));
	}

	~print_map () {
		std::fclose (output);
	}

	void print_feature (featureid_t feature_id, observation_type obs) const {
		typename observation_type::vector_type v = (initial_state + obs).to_vector();
		std::fprintf (output, "%zu", feature_id);
		for (int i = 0; i < v.size(); ++i) std::fprintf (output, "\t%f", v(i));
		std::fprintf (output, "\n");
	}

};


template <class SlamImpl>
class print_trajectory {

	typedef typename SlamImpl::control_type control_type;

	FILE* output;

public:

	print_trajectory (const char* filename, const SlamImpl& slam, control_type initial_state)
	: output(std::fopen(filename, "w"))
	{
		if (output) {
			const bitree<control_type>& trajectory = slam.trajectory_estimate();
			for (size_t i = 0; i < trajectory.size(); ++i) {
				typename control_type::vector_type v = (initial_state + trajectory.accumulate(i)).to_vector();
				std::fprintf (output, "%zu", i);
				for (int j = 0; j < v.size(); ++j) std::fprintf (output, "\t%f", v(j));
				std::fprintf(output, "\n");
			}
		}
	}

	~print_trajectory () {
		std::fclose (output);
	}

};


template <class SlamData, class SlamImpl>
class print_incremental_info {

	typedef typename SlamImpl::control_type control_type;

	const SlamData& data;
	const SlamImpl& slam;
	control_type initial_state;

	boost::filesystem::path path;

	boost::signals2::scoped_connection conn;

public:

	print_incremental_info (const SlamData& data_, const SlamImpl& slam_,
			control_type initial_state_, boost::filesystem::path path_)
	: data(data_), slam(slam_), initial_state(initial_state_), path(path_) { }

	void print (typename SlamData::timestep_t timestep) const {

		boost::filesystem::create_directories (path);

		std::string trajectory_filename = "trajectory." + boost::lexical_cast<std::string>(timestep) + ".txt";
		print_trajectory<SlamImpl> ((path/trajectory_filename).c_str(), slam, initial_state);

		std::string map_filename = "map." + boost::lexical_cast<std::string>(timestep) + ".txt";
		print_map<SlamImpl> ((path/map_filename).c_str(), slam, initial_state);

	}

	void connect () { conn = data.connect_timestep_listener(boost::bind(&print_incremental_info::print, this, _1)); }

	void disconnect () { conn.release(); }

};

#endif //_SIMULATOR_HPP
