//
//  dataset.cpp
//  slam
//
//  Created by Roshan Shariff on 2012-10-07.
//  Copyright (c) 2012 University of Alberta. All rights reserved.
//

#include <cmath>
#include <set>
#include <boost/filesystem/fstream.hpp>

#include "slam/dataset_impl.hpp"
#include "slam/slam_result_impl.hpp"
#include "dataset.hpp"

auto read_range_only_data (boost::filesystem::path dir, const std::string& name)
-> range_only_data_type {

    using control_model_type = planar_robot::velocity_slip_model;
    using observation_model_type = planar_robot::range_only_model;
    
    using slam_result_impl_type = slam::slam_result_of_impl<control_model_type, observation_model_type>;
    auto ground_truth = std::make_shared<slam_result_impl_type>();
    
    double start_timestamp;

    /** Ground truth path from GPS */
    {
        boost::filesystem::ifstream input_GT (dir/name/(name+"_GT.txt"));
        double t, x, y, bearing;

        input_GT >> t >> x >> y >> bearing;

        start_timestamp = t;
        auto initial_state = planar_robot::pose::cartesian (x, y, bearing);

        ground_truth->set_initial_state (initial_state);
        
        while (input_GT >> t >> x >> y >> bearing) {
            auto state = -initial_state + planar_robot::pose::cartesian (x, y, bearing);
            ground_truth->get_trajectory().push_back_accumulated (state);
        }
    }

    /** Surveyed node locations */
    {
        boost::filesystem::ifstream input_TL (dir/name/(name+"_TL.txt"));
        double id, x, y;
        
        while (input_TL >> id >> x >> y) {
            slam::featureid_type feature_id (std::round (id));
            auto position = planar_robot::position::cartesian(x, y);
            ground_truth->get_feature_map().emplace(feature_id, position);
        }
    }
    
    using dataset_impl_type = slam::dataset_impl<control_model_type, observation_model_type>;
    auto dataset = std::make_shared<dataset_impl_type>();
    
    /** Odometry input */
    {
        boost::filesystem::ifstream input_DR (dir/name/(name+"_DR.txt"));
        double timestamp, prev_timestamp = start_timestamp;
        control_model_type::vector_type control; control.setZero();
        
        while (input_DR >> timestamp >> control(0) >> control(1)) {
            const double dt = timestamp - prev_timestamp;
            prev_timestamp = timestamp;
            dataset->add_control(dt, control/dt);
        }
    }
    
    /** Node range observations */
    {
        boost::filesystem::ifstream input_TD (dir/name/(name+"_TD.txt"));
        double timestamp, antenna, id;
        observation_model_type::vector_type observation;
        
        std::set<std::pair<slam::timestep_type, slam::featureid_type>> observed;
        
        while (input_TD >> timestamp >> antenna >> id >> observation(0)) {
            slam::featureid_type feature_id (std::round (id));
            slam::timestep_type timestep = dataset->timestep_at(timestamp - start_timestamp);
            if (observed.emplace(timestep, feature_id).second) {
                dataset->add_observation(timestep, feature_id, observation);
            }
        }
    }
    
    {
        boost::filesystem::ofstream controls (dir/name/"controls.txt");
        controls << "# dt, actual v, actual w, actual g, commanded v, commanded w\n";
        for (slam::timestep_type t {0}; t < dataset->current_timestep(); ++t) {
            double dt = dataset->timedelta(t);
            planar_robot::pose pose_delta = ground_truth->get_trajectory()[t];
            auto actual = planar_robot::velocity_slip_model::observe(pose_delta);
            actual /= dt;
            auto commanded = dataset->control(t);
            controls << dt << ' ' << actual(0) << ' ' << actual(1) << ' ' << actual(2) << ' '
            << commanded(0) << ' ' << commanded(1) << '\n';
        }
    }
    
    return std::make_tuple (std::move(dataset), std::move(ground_truth));
}
