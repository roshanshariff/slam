#ifndef _SIMULATOR_HPP
#define _SIMULATOR_HPP

#include <string>
#include <cstdio>
#include <cassert>

#include <boost/bind.hpp>
#include <boost/ref.hpp>
#include <boost/shared_ptr.hpp>
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
: public slam::slam_result_of <typename Controller::model_type, typename Sensor::model_type> {
    
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
    
    double state_log_likelihood = 0;
    
    bool sim_completed = false;
    
    unsigned int sensor_skip;
    
    utility::listeners<slam::timestep_listener> listeners;
    
    void timestep () {
        listeners.for_each (boost::bind (&slam::timestep_listener::timestep, _1, current_timestep()));
    }
    
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
    
    double get_log_likelihood () const { return state_log_likelihood + sensor.get_log_likelihood(); }
    
    // Overridden virtual member functions of slam_result
    
    virtual void timestep (slam::timestep_type t) override {
        assert (t <= current_timestep());
    }
    
    virtual void completed () override {
        assert (sim_completed);
    }
    
    virtual slam::timestep_type current_timestep () const override {
        return data->current_timestep();
    }
    
    virtual state_type get_state (slam::timestep_type t) const override {
        assert (t <= trajectory.size());
        return trajectory.accumulate(t);
    }
    
    virtual feature_type get_feature (slam::featureid_type id) const override {
        return feature_map.at(id);
    }
    
    virtual const trajectory_type& get_trajectory () const override {
        return trajectory;
    }
    
    virtual const feature_map_type& get_feature_map () const override {
        return feature_map;
    }
    
};


template <class Controller, class Sensor>
simulator<Controller, Sensor>::simulator (boost::program_options::variables_map& options, unsigned int seed)
: random (remember_option (options, "sim-seed", seed)),
data(boost::make_shared<slam_data_type>()),
controller (options),
sensor (options),
initial_state (controller.initial_state()),
sensor_skip (options["sensor-skip"].as<unsigned int>())
{
    add_timestep_listener (data);
    for (size_t i = 0; i < sensor.num_features(); ++i) {
        feature_map.emplace_hint (feature_map.end(), i, -initial_state + sensor.get_feature (i));
    }
}


template <class Controller, class Sensor>
void simulator<Controller, Sensor>::operator() () {
    
    assert (!sim_completed);
    
    sensor.sense (get_initial_state() + get_state(current_timestep()), random,
                  boost::bind (&slam_data_type::add_observation, data.get(), _1, _2));
    timestep ();
    
    while (!controller.finished()) {
        
        auto control_dist = controller.control (get_initial_state() + get_state(current_timestep()));
        auto control = control_dist(random);
        
        trajectory.push_back (control);
        state_log_likelihood += control_dist.log_likelihood (control);
        
        data->add_control (control_dist);
        
        if (current_timestep() % sensor_skip == 0) {
            sensor.sense (get_initial_state() + get_state(current_timestep()), random,
                          boost::bind (&slam_data_type::add_observation, data.get(), _1, _2));
        }
        
        timestep ();
    }
    
    sim_completed = true;
    listeners.for_each (boost::bind (&slam::timestep_listener::completed, _1));
}


template <class Controller, class Sensor>
boost::program_options::options_description
simulator<Controller, Sensor>::program_options() {
    namespace po = boost::program_options;
    po::options_description options ("Simulator parameters");
    options.add_options()
    ("sim-seed", po::value<unsigned int>(), "Seed for control and observation noise")
    ("sensor-skip", po::value<unsigned int>()->default_value(8), "Take sensor readings every n time steps");
    return options;
}


#endif //_SIMULATOR_HPP
