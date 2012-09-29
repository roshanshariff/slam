//
//  g2o_clustering.hpp
//  slam
//
//  Created by Roshan Shariff on 2012-09-28.
//  Copyright (c) 2012 University of Alberta. All rights reserved.
//

#ifndef slam_g2o_clustering_hpp
#define slam_g2o_clustering_hpp

#include <memory>
#include <vector>

#include "slam/interfaces.hpp"
#include "slam/slam_data.hpp"
#include "slam/slam_result_impl.hpp"
#include "slam/slam_likelihood.hpp"
#include "slam/slam_rmse.hpp"
#include "slam/g2o_slam.hpp"


namespace slam {
    
    template <class ControlModel, class ObservationModel>
    class g2o_clustering : public g2o_slam<ControlModel, ObservationModel>
    {
        
        using slam_data_type = slam_data<ControlModel, ObservationModel>;
        using slam_result_type = slam_result_of<ControlModel, ObservationModel>;
        
        using state_type = typename ControlModel::associated_type;
        using feature_type = typename ObservationModel::associated_type;
        
        struct cluster_type {
            
            slam_result_impl<state_type, feature_type> estimate;
            double log_likelihood;
            
            auto distance (const slam_result_type& candidate) -> double {
                return rms_error (estimate, candidate);
            }
        };
        
        std::shared_ptr<const slam_data_type> data;
        std::vector<cluster_type> clusters;
        
        double rmse_threshold = 1e-2;
        
    public:
        
        g2o_clustering(decltype(data) data) : g2o_slam<ControlModel, ObservationModel>(0), data(data) { }
        
        void add (const slam_result_type& candidate) {
            
            this->reinitialise (candidate);
            this->optimise (7);
            
            double log_likelihood = slam_log_likelihood (*data, *this);
        
            bool new_cluster = true;
            for (auto& cluster : clusters) {
                
                if (cluster.distance (*this) <= rmse_threshold) {
                    
                    if (log_likelihood > cluster.log_likelihood) {
                        cluster.estimate = *this;
                        cluster.log_likelihood = log_likelihood;
                    }
                    
                    new_cluster = false;
                    break;
                }
            }
            
            if (new_cluster) {
                clusters.push_back ({ slam_result_impl<state_type, feature_type>(*this), log_likelihood });
            }
        }
        
        auto get_clusters () const -> const decltype(clusters)& {
            return clusters;
        }
        
    };
}

#endif
