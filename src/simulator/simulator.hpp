#ifndef _SIMULATOR_HPP
#define _SIMULATOR_HPP

#include <string>
#include <cstdio>

#include <boost/bind.hpp>
#include <boost/ref.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/enable_shared_from_this.hpp>
#include <boost/container/flat_map.hpp>
#include <boost/program_options.hpp>
#include <boost/signals2.hpp>
#include <boost/utility.hpp>

#include "slam/slam_data.hpp"
#include "slam/slam_result.hpp"
#include "utility/random.hpp"
#include "utility/bitree.hpp"
#include "utility/utility.hpp"


template <class Controller, class Sensor>
class simulator
: public slam_result <typename Controller::model_type::result_type, typename Sensor::model_type::result_type>,
public boost::enable_shared_from_this<simulator<Controller, Sensor> > {

public:
    
	typedef typename Controller::model_type control_model_type;
	typedef typename Sensor::model_type observation_model_type;

    typedef typename control_model_type::result_type state_type;
    typedef typename observation_model_type::result_type feature_type;
    
	typedef slam_data<control_model_type, observation_model_type> slam_data_type;

    typedef typename slam_data_type::featureid_t featureid_t;
    typedef typename slam_data_type::timestep_t timestep_t;

    typedef boost::signals2::signal<void (timestep_t)> timestep_signal_type;
    
private:
    
    typedef bitree<state_type> trajectory_type;
    typedef boost::container::flat_map<featureid_t, feature_type> feature_map_type;

    // Data members
    
	random_source random;    
    slam_data_type data;
    
    Controller controller;
    Sensor sensor;

    const state_type initial_state;
	trajectory_type trajectory;
    feature_map_type feature_map;

    timestep_signal_type timestep_signal;
    
public:
    
    simulator (boost::program_options::variables_map& options, unsigned int seed);
    
    static boost::program_options::options_description program_options ();
    
	void operator() ();
    
    state_type get_initial_state () const { return initial_state; }
    
    boost::shared_ptr<const slam_data_type> get_slam_data () const {
        return boost::shared_ptr<const slam_data_type> (this->shared_from_this(), &data);
    }
    
    boost::signals2::connection connect_timestep_listener (const timestep_signal_type::slot_type& l) {
        return timestep_signal.connect (l);
    }
    
    // Overridden virtual member functions of slam_result
    
    virtual state_type get_state () const { return trajectory.accumulate(); }
    
    virtual boost::shared_ptr<const trajectory_type> get_trajectory () const {
        return boost::shared_ptr<const trajectory_type> (this->shared_from_this(), &trajectory);
    }
    
    virtual boost::shared_ptr<const feature_map_type> get_map () const {
        return boost::shared_ptr<const feature_map_type> (this->shared_from_this(), &feature_map);
    }

};


template <class Controller, class Sensor>
simulator<Controller, Sensor>::simulator (boost::program_options::variables_map& options, unsigned int seed)
: random (remember_option (options, "sim-seed", seed)),
controller (options),
sensor (options),
initial_state (controller.initial_state())
{
    for (size_t i = 0; i < sensor.num_features(); ++i) {
        feature_map.emplace_hint (feature_map.end(), i, -initial_state + sensor.get_feature (i));
    }
}


template <class Controller, class Sensor>
void simulator<Controller, Sensor>::operator() () {

    sensor.sense (get_state(), random, boost::bind (&slam_data_type::add_observation, boost::ref(data), _1, _2));
    data.end_observation();

    timestep_signal (data.current_timestep());
    
	while (!controller.finished()) {

		control_model_type control = controller.control (get_state());
		trajectory.push_back(control(random));
		data.add_control (control);

        sensor.sense (get_state(), random, boost::bind (&slam_data_type::add_observation, boost::ref(data), _1, _2));
        data.end_observation();
        
        timestep_signal (data.current_timestep());
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
