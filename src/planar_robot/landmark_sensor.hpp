/*
 * landmark_sensor.hpp
 *
 *  Created on: 2011-08-26
 *      Author: roshan
 */

#ifndef LANDMARK_SENSOR_HPP_
#define LANDMARK_SENSOR_HPP_

#include <vector>
#include <tr1/functional>
#include <boost/program_options.hpp>

#include "planar_robot/pose.hpp"
#include "planar_robot/position.hpp"
#include "planar_robot/range_bearing_model.hpp"
#include "utility/random.hpp"
#include "slam/vector_model.hpp"

namespace planar_robot {
    
    
    struct landmark_sensor {
        
        typedef vector_model_adapter<range_bearing_model> model_type;
        
        typedef std::tr1::function<void (size_t, model_type)> observe_function_type;
        
        static boost::program_options::options_description program_options ();
        static landmark_sensor parse_options (const boost::program_options::variables_map&);
        
        void sense (const pose&, observe_function_type, random_source&) const;
        
        template <class FeatureFunctor>
        void for_each_feature (FeatureFunctor f) const;
        
    private:
        
        const double max_range;
        const model_type::builder observation_builder;
        const std::vector<position> landmarks;
        
        mutable unsigned long hits;
        
        landmark_sensor (double max_range_, const range_bearing_model::builder& observation_builder_,
                         const std::vector<position>& landmarks_)
        : max_range(max_range_), observation_builder(observation_builder_), landmarks(landmarks_), hits(0) { }
        
    public:
        
        unsigned long num_observations() const { return hits; }
        
    };
    
    
    template <class FeatureFunctor>
    void landmark_sensor::for_each_feature (FeatureFunctor f) const {
        for (size_t i = 0; i < landmarks.size(); ++i) {
            f (i, landmarks[i]);
        }
    }
    
    
}


#endif /* LANDMARK_SENSOR_HPP_ */
