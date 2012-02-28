/*
 * landmark_sensor.hpp
 *
 *  Created on: 2011-08-26
 *      Author: roshan
 */

#ifndef LANDMARK_SENSOR_HPP_
#define LANDMARK_SENSOR_HPP_

#include <vector>
#include <boost/program_options.hpp>

#include "planar_robot/pose.hpp"
#include "planar_robot/position.hpp"
#include "planar_robot/range_bearing_model.hpp"
#include "slam/vector_model.hpp"
#include "slam/interfaces.hpp"
#include "utility/random.hpp"

namespace planar_robot {
    
    
    class landmark_sensor {
        
    public:
        
        typedef vector_model_adapter<range_bearing_model> model_type;
        
        landmark_sensor (const boost::program_options::variables_map&);
        
        static boost::program_options::options_description program_options ();
        
        template <class Observer> void sense (const pose&, random_source&, Observer) const;
        
        size_t num_features () const { return landmarks.size(); }
        
        const position& get_feature (size_t index) const { return landmarks.at (index); }
        
    private:
        
        double max_range, min_range;
        mutable unsigned long hits;
        mutable double log_likelihood = 0;
        model_type::builder model_builder;
        std::vector<position> landmarks;
        
        template <class Observer> class sense_helper;
        
    public:
        
        unsigned long num_observations () const { return hits; }
        double get_log_likelihood () const { return log_likelihood; }
    };
    
    
    template <class Observer>
    void landmark_sensor::sense (const pose& state, random_source& random, Observer observer) const {
        for (size_t i = 0; i < landmarks.size(); ++i) {
            position observation = -state + landmarks[i];
            auto model = model_builder (model_builder(observation)(random));
            if (min_range < model.mean().distance() && model.mean().distance() < max_range) {
                ++hits;
                observer (slam::featureid_type(i), model);
                log_likelihood += model.log_likelihood (observation);
            }
        }
    }
    
}


#endif /* LANDMARK_SENSOR_HPP_ */
