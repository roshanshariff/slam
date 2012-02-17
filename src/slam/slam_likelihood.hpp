//
//  slam_likelihood.hpp
//  slam
//
//  Created by Roshan Shariff on 12-02-14.
//  Copyright (c) 2012 University of Alberta. All rights reserved.
//

#ifndef slam_slam_likelihood_hpp
#define slam_slam_likelihood_hpp

#include "slam/interfaces.hpp"
#include "slam/slam_data.hpp"

namespace slam {
    

    template <class ControlModel, class ObservationModel>
    double slam_log_likelihood (const slam_data<ControlModel, ObservationModel>&,
                                const mcmc_slam<ControlModel, ObservationModel>& estimate) {
        
        return estimate.get_log_likelihood();
    }
    

    template <class ControlModel, class ObservationModel>
    double slam_log_likelihood (const slam_data<ControlModel, ObservationModel>& data,
                                const slam_result_of<ControlModel, ObservationModel>& estimate) {
        
        double log_likelihood = 0;
        
        const auto& trajectory = estimate.get_trajectory();
        const auto& feature_map = estimate.get_feature_map();
        
        for (timestep_type t; t < trajectory.size(); ++t) {
            log_likelihood += data.control(t).log_likelihood (trajectory[t]);
        }
        
        for (const auto& feature : feature_map) {
            
            featureid_type id = feature.first;
            auto observation = feature.second;
            timestep_type timestep;
            
            assert (data.feature_observed (id));
            
            for (const auto& obs : data.feature_data(id)) {
                
                observation = trajectory.accumulate (obs.first, timestep) + observation;

                timestep = obs.first;
                const ObservationModel& distribution = obs.second;
                
                log_likelihood += distribution.log_likelihood (observation);
            }
        }
        
        return log_likelihood;
    }
    
    
} // namespace slam


#endif
