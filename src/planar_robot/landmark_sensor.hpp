/*
 * landmark_sensor.hpp
 *
 *  Created on: 2011-08-26
 *      Author: roshan
 */

#ifndef LANDMARK_SENSOR_HPP_
#define LANDMARK_SENSOR_HPP_

#include <vector>
#include <string>
#include <fstream>

#include <boost/program_options.hpp>
#include <boost/math/constants/constants.hpp>

#include "planar_robot/pose.hpp"
#include "planar_robot/position.hpp"
#include "planar_robot/range_bearing_model.hpp"
#include "slam/interfaces.hpp"
#include "utility/random.hpp"

#include "main.hpp"


namespace planar_robot {
    
    template <class ObservationModel> class landmark_sensor {
        
    public:
        
        using model_type = ObservationModel;
        using model_builder_type = typename model_type::builder;
        
        landmark_sensor (const boost::program_options::variables_map&);
        
        static auto program_options () -> boost::program_options::options_description;
        
        template <class Observer> void sense (const pose&, random_source&, Observer) const;
        
        auto num_features () const -> size_t { return landmarks.size(); }
        
        auto get_feature (size_t index) const -> const position& { return landmarks.at (index); }
        
    private:
        
        model_builder_type model_builder;
        
        double max_range, min_range;
        mutable unsigned long hits = 0;
        mutable double log_likelihood = 0;
        std::vector<position> landmarks;

    public:
        
        auto num_observations () -> unsigned long const { return hits; }
        auto get_log_likelihood () const -> double { return log_likelihood; }
        
    };
    
}


template <class ObservationModel>
auto planar_robot::landmark_sensor<ObservationModel>
::program_options () -> boost::program_options::options_description {
    
    namespace po = boost::program_options;
    po::options_description sensor_options ("Landmark Sensor Options");
    
    sensor_options.add_options()
    ("sensor-range-max", po::value<double>()->default_value(30), "maximum sensor range, in m")
    ("sensor-range-min", po::value<double>()->default_value(1), "minimum sensor range, in m")
    ("landmark-file", po::value<std::string>(),
     "filename of landmarks file");
    
    sensor_options.add (model_builder_type::program_options());
    return sensor_options;
}


template <class ObservationModel>
planar_robot::landmark_sensor<ObservationModel>
::landmark_sensor (const boost::program_options::variables_map& options)
: model_builder (options),
max_range       (options["sensor-range-max"].as<double>()),
min_range       (options["sensor-range-min"].as<double>())
{
    if (options.count("landmark-file")) {
        double x, y;
        std::ifstream landmark_file (options["landmark-file"].as<std::string>().c_str());
        while (landmark_file >> x >> y) landmarks.push_back (position::cartesian (x, y));
    }
}


template <class ObservationModel>
template <class Observer>
void planar_robot::landmark_sensor<ObservationModel>
::sense (const pose& state, random_source& random, Observer observer) const {
    for (size_t i = 0; i < landmarks.size(); ++i) {
        auto rel_pos = -state + landmarks[i];
        if (min_range < rel_pos.distance() && rel_pos.distance() < max_range) {
            ++hits;
            auto observation = ObservationModel::observe (rel_pos);
            auto model = model_builder (model_builder(observation)(random));
            observer (slam::featureid_type(i), model);
            log_likelihood += model.log_likelihood (observation);
        }
    }
}


extern template class planar_robot::landmark_sensor<observation_model_type>;

#endif /* LANDMARK_SENSOR_HPP_ */
