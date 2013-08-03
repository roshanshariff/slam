#ifndef _SIMULATOR_HPP
#define _SIMULATOR_HPP

#include <string>
#include <cstdio>
#include <cassert>
#include <memory>
#include <functional>

#include <boost/program_options.hpp>

#include "slam/interfaces.hpp"
#include "slam/dataset_impl.hpp"
#include "slam/slam_result_impl.hpp"
#include "utility/random.hpp"
#include "utility/bitree.hpp"
#include "utility/utility.hpp"
#include "utility/flat_map.hpp"

#include "main.hpp"


template <class ControlModel, class ObservationModel, class Controller, class Sensor>
class simulator
: public slam::dataset_impl <ControlModel, ObservationModel>,
public slam::slam_result_of_impl <ControlModel, ObservationModel> {

    using dataset_impl_type = slam::dataset_impl<ControlModel, ObservationModel>;
    using slam_result_impl_type = slam::slam_result_of_impl<ControlModel, ObservationModel>;
    
public:
    
    using typename slam_result_impl_type::state_type;
    using typename slam_result_impl_type::feature_type;
    using typename slam_result_impl_type::trajectory_type;
    using typename slam_result_impl_type::feature_map_type;
    
    simulator (boost::program_options::variables_map& options, unsigned int seed,
               const typename ControlModel::builder& control_model_builder,
               const typename ObservationModel::builder& observation_model_builder,
               Controller& controller, const Sensor& sensor);
    
    static boost::program_options::options_description program_options ();
    
    virtual auto current_timestep () const -> slam::timestep_type override {
        return slam::dataset_impl<ControlModel, ObservationModel>::current_timestep();
    }
};


template <class ControlModel, class ObservationModel, class Controller, class Sensor>
simulator<ControlModel, ObservationModel, Controller, Sensor>
::simulator (boost::program_options::variables_map& options, unsigned int seed,
             const typename ControlModel::builder& control_model_builder,
             const typename ObservationModel::builder& observation_model_builder,
             Controller& controller, const Sensor& sensor) {
    
    random_source random (remember_option (options, "sim-seed", seed));
    const unsigned int sensor_skip = options["sensor-skip"].as<unsigned int>();
    const double dt = options["dt"].as<double>();
    
    {
        auto& map = this->get_feature_map();
        for (size_t i = 0; i < sensor.num_features(); ++i) {
            map.emplace_hint (map.end(), i, sensor.get_feature(i));
        }
    }

    this->set_initial_state (controller.initial_state());
    state_type state;
    
    auto current_state = [&]() { return this->get_initial_state() + state; };
    
    auto sense = [&]() {
        using namespace std::placeholders;
        sensor.template sense<ObservationModel> (observation_model_builder, random, current_state(),
                                                 std::bind (&simulator::add_observation_now, this, _1, _2));
    };

    sense();
    
    while (!controller.finished()) {
        
        const auto control = controller.template control<ControlModel> (current_state(), dt);
        this->add_control (dt, control);

        state += control_model_builder(control, dt).proposal()(random);
        this->get_trajectory().push_back_accumulated (state);
        
        if (this->current_timestep() % sensor_skip == 0) sense();
    }
}


template <class ControlModel, class ObservationModel, class Controller, class Sensor>
boost::program_options::options_description
simulator<ControlModel, ObservationModel, Controller, Sensor>::program_options() {
    namespace po = boost::program_options;
    po::options_description options ("Simulator parameters");
    options.add_options()
    ("dt", po::value<double>()->default_value(0.25), "time delta per update, in seconds")
    ("sim-seed", po::value<unsigned int>(), "Seed for control and observation noise")
    ("sensor-skip", po::value<unsigned int>()->default_value(8), "Take sensor readings every n time steps");
    return options;
}


extern template
class simulator<control_model_type, observation_model_type, controller_type, sensor_type>;


#endif //_SIMULATOR_HPP
