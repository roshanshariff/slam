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
    

//    template <class ControlModel, class ObservationModel>
//    double slam_log_likelihood (const slam_data<ControlModel, ObservationModel>&,
//                                const mcmc_slam<ControlModel, ObservationModel>& estimate) {
//        
//        return estimate.get_log_likelihood();
//    }
    

    template <class ControlModel, class ObservationModel>
    double slam_log_likelihood (const slam_data<ControlModel, ObservationModel>& data,
                                const slam_result_of<ControlModel, ObservationModel>& estimate) {
        
        double log_likelihood = 0;
        
        const auto& trajectory = estimate.get_trajectory();
        for (timestep_type t; t < trajectory.size(); ++t) {
            log_likelihood += data.control(t).log_likelihood (ControlModel::observe (trajectory[t]));
        }
        
        const auto initial_state = estimate.get_initial_state();
        for (const auto& id_feature : estimate.get_feature_map()) {
            
            const featureid_type id = id_feature.first;
            if (!data.feature_observed (id)) continue;
            
            auto feature = -initial_state + id_feature.second;

            timestep_type timestep {0};
            for (const auto& obs : data.get_observations(id)) {
                feature = trajectory.accumulate (obs.first, timestep) + feature;
                timestep = obs.first;
                const ObservationModel& distribution = obs.second;
                log_likelihood += distribution.log_likelihood (ObservationModel::observe (feature));
            }
        }
        
        return log_likelihood;
    }
    
    
} // namespace slam


#endif
