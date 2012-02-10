#ifndef _SIMULATOR_HPP
#define _SIMULATOR_HPP

#include <string>
#include <cstdio>
#include <cassert>

#include <boost/bind.hpp>
#include <boost/ref.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/enable_shared_from_this.hpp>
#include <boost/container/flat_map.hpp>
#include <boost/program_options.hpp>
#include <boost/utility.hpp>

#include "slam/interfaces.hpp"
#include "slam/slam_data.hpp"
#include "utility/random.hpp"
#include "utility/bitree.hpp"
#include "utility/listeners.hpp"
#include "utility/utility.hpp"


template <class Controller, class Sensor>
class simulator
: public slam::slam_result <typename Controller::model_type::result_type, typename Sensor::model_type::result_type>,
public boost::enable_shared_from_this<simulator<Controller, Sensor> > {

public:
    
	typedef typename Controller::model_type control_model_type;
	typedef typename Sensor::model_type observation_model_type;

    typedef typename control_model_type::result_type state_type;
    typedef typename observation_model_type::result_type feature_type;
    
	typedef slam::slam_data<control_model_type, observation_model_type> slam_data_type;

private:
    
    typedef utility::bitree<state_type> trajectory_type;
    typedef boost::container::flat_map<slam::featureid_type, feature_type> feature_map_type;

    // Data members
    
	random_source random;    
    boost::shared_ptr<slam_data_type> data;
    
    Controller controller;
    Sensor sensor;

    const state_type initial_state;
	trajectory_type trajectory;
    feature_map_type feature_map;

    utility::listeners<slam::timestep_listener> listeners;
    
public:
    
    simulator (boost::program_options::variables_map& options, unsigned int seed);
    
    void add_timestep_listener (const boost::shared_ptr<slam::timestep_listener>& l) {
        listeners.add(l);
    }
    
    void add_data_listener (const boost::shared_ptr<typename slam_data_type::listener>& l) {
        data->add_listener(l);
    }

    static boost::program_options::options_description program_options ();
    
	void operator() ();
    
    state_type get_initial_state () const { return initial_state; }
    
    boost::shared_ptr<const slam_data_type> get_slam_data () const { return data; }
    
    // Overridden virtual member functions of slam_result
    
    virtual void timestep (slam::timestep_type t) override {
        assert (t == current_timestep());
        listeners.for_each (boost::bind (&slam::timestep_listener::timestep, _1, t));
    }
    
    virtual slam::timestep_type current_timestep () const override { return data->current_timestep(); }
    
    virtual state_type get_state () const override { return trajectory.accumulate(); }
    
    virtual boost::shared_ptr<const trajectory_type> get_trajectory () const override {
        return boost::shared_ptr<const trajectory_type> (this->shared_from_this(), &trajectory);
    }
    
    virtual boost::shared_ptr<const feature_map_type> get_map () const override {
        return boost::shared_ptr<const feature_map_type> (this->shared_from_this(), &feature_map);
    }

};


template <class Controller, class Sensor>
simulator<Controller, Sensor>::simulator (boost::program_options::variables_map& options, unsigned int seed)
: random (remember_option (options, "sim-seed", seed)),
data(boost::make_shared<slam_data_type>()),
controller (options),
sensor (options),
initial_state (controller.initial_state())
{
    add_timestep_listener (data);
    for (size_t i = 0; i < sensor.num_features(); ++i) {
        feature_map.emplace_hint (feature_map.end(), i, -initial_state + sensor.get_feature (i));
    }
}


template <class Controller, class Sensor>
void simulator<Controller, Sensor>::operator() () {

    sensor.sense (get_initial_state() + get_state(), random,
                  boost::bind (&slam_data_type::add_observation, data.get(), _1, _2));
    timestep (current_timestep());
    
	while (!controller.finished()) {

		control_model_type control = controller.control (get_initial_state() + get_state());
		trajectory.push_back(control(random));
		data->add_control (control);

        sensor.sense (get_initial_state() + get_state(), random,
                      boost::bind (&slam_data_type::add_observation, data.get(), _1, _2));
        timestep (current_timestep());
	}
}


template <class Controller, class Sensor>
boost::program_options::options_description
simulator<Controller, Sensor>::program_options() {
    namespace po = boost::program_options;
    po::options_description options ("Simulator parameters");
    options.add_options()
    ("sim-seed", po::value<unsigned int>(), "Seed for control and observation noise");
    return options;
}


#endif //_SIMULATOR_HPP
