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
#include "utility/random.hpp"

namespace planar_robot {
    
    
    class landmark_sensor {
        
    public:
        
        typedef vector_model_adapter<range_bearing_model> model_type;
        
        landmark_sensor (const boost::program_options::variables_map&);
        
        static boost::program_options::options_description program_options ();
        
        template <class Observer>
        void sense (const pose&, random_source&, const Observer&) const;
        
        template <class FeatureFunctor>
        void for_each_feature (FeatureFunctor f) const;
        
    private:
        
        double max_range;
        mutable unsigned long hits;
        model_type::builder model_builder;
        std::vector<position> landmarks;
        
        template <class Observer> class sense_helper;
        
    public:
        
        unsigned long num_observations() const { return hits; }
        
    };
    
    
    template <class Observer>
    class landmark_sensor::sense_helper {
        
        const landmark_sensor& sensor;
        const pose& state;
        random_source& random;
        Observer observer;
        
    public:
        
        sense_helper (const landmark_sensor& sensor_, const pose& state_,
                      random_source& random_, const Observer& observer_)
        : sensor(sensor_), state(state_), random(random_), observer(observer_) { }
        
        void operator() (size_t i, const position& pos) {
            position obs = sensor.model_builder(-state + pos)(random);
            if (obs.distance() < sensor.max_range) {
                ++sensor.hits;
                observer (i, sensor.model_builder (obs));
            }
        }
    };
    
    
    template <class Observer>
    void landmark_sensor::sense (const pose& state, random_source& random, const Observer& observer) const {
        for_each_feature (sense_helper<Observer> (*this, state, random, observer));
    }
    
    
    template <class FeatureFunctor>
    void landmark_sensor::for_each_feature (FeatureFunctor f) const {
        for (size_t i = 0; i < landmarks.size(); ++i) {
            f (i, landmarks[i]);
        }
    }
    
    
}


#endif /* LANDMARK_SENSOR_HPP_ */
