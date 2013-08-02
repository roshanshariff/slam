//
//  dataset.cpp
//  slam
//
//  Created by Roshan Shariff on 2012-10-07.
//  Copyright (c) 2012 University of Alberta. All rights reserved.
//

#include <iostream>
#include <fstream>
#include <string>
#include <iomanip>
#include <vector>
#include <utility>
#include <algorithm>
#include <map>

#include <Eigen/Core>
#include <Eigen/SVD>
#include <boost/math/constants/constants.hpp>

#include "planar_robot/pose.hpp"
#include "planar_robot/velocity_model.hpp"
#include "planar_robot/range_bearing_model.hpp"
#include "planar_robot/rms_error.hpp"
#include "slam/interfaces.hpp"
#include "utility/bitree.hpp"
#include "utility/geometry.hpp"

using trajectory_type = utility::bitree<planar_robot::pose>;

struct dataset {
    
    
    std::vector<double> timestamps;
    planar_robot::pose initial_pose;
    trajectory_type trajectory;
    
    std::vector<planar_robot::velocity_model::vector_type> controls;
    trajectory_type dead_reckoning;
    
    std::map<slam::featureid_type, planar_robot::position> feature_map;
    
    struct observation_type {
        slam::timestep_type timestep;
        slam::featureid_type feature;
        planar_robot::range_only_model::vector_type observation;
    };
    
    std::vector<observation_type> observations;
    
    slam::timestep_type get_timestep (double timestamp) const;

    void read_controls (const std::string& filename_DR);
    void read_ground_truth (const std::string& filename_GT);
    void read_feature_map (const std::string& filename_TL);
    void read_observations (const std::string& filename_TD);
};

slam::timestep_type dataset::get_timestep (double timestamp) const {
    
    auto upper_bound = std::upper_bound (timestamps.cbegin(), timestamps.cend(), timestamp);
    slam::timestep_type t (upper_bound - timestamps.cbegin());
    
    if (t == 0) return t;
    else if (t == timestamps.size()) return t-1;
    else if (timestamps[t] - timestamp < timestamp - timestamps[t-1]) return t;
    else return t-1;
}

void dataset::read_ground_truth (const std::string& filename_GT) {
    
    std::ifstream in (filename_GT);
    double t, x, y, bearing;
    
    while (in >> t >> x >> y >> bearing) {
        
        
        auto new_pose = planar_robot::pose::cartesian (x, y, bearing);
        
        if (timestamps.empty()) {
            initial_pose = new_pose;
        }
        else {
            trajectory.push_back (-(initial_pose + trajectory.accumulate()) + new_pose);
        }
        
        timestamps.push_back (t);
    }
}

void dataset::read_controls (const std::string& filename_DR) {

    std::ifstream in (filename_DR);
    double t, v, w;
    
    while (in >> t >> v >> w) {

        controls.push_back ({v, w});
        dead_reckoning.push_back (planar_robot::velocity_model::inv_observe (controls.back()));
    }
}

void dataset::read_feature_map (const std::string& filename_TL) {
    
    std::ifstream in (filename_TL);
    double id, x, y;
    
    while (in >> id >> x >> y) {
        feature_map[slam::featureid_type(id)] = planar_robot::position::cartesian (x, y);
    }
}

void dataset::read_observations (const std::string& filename_TD) {
    
    std::ifstream in (filename_TD);
    double t, antenna, feature, range;
    
    while (in >> t >> antenna >> feature >> range) {
        planar_robot::range_only_model::vector_type obs;
        obs << range;
        observations.push_back ({ get_timestep(t), slam::featureid_type(feature), obs });
    }
}

Eigen::Matrix2d velocity_model_params (const dataset& data) {
    
    using namespace Eigen;

    MatrixXd controls (data.controls.size(), 2);
    for (std::size_t i = 0; i < data.controls.size(); ++i) {
        controls.row(i) = data.controls[i].transpose();
    }
    
    MatrixXd actual (data.trajectory.size(), 2);
    for (std::size_t i = 0; i < data.trajectory.size(); ++i) {
        actual.row(i) = planar_robot::velocity_model::observe (data.trajectory[i]).transpose();
    }
    
    MatrixXd deviations = (controls - actual).cwiseAbs2();
    
//    MatrixX4d output (deviations.rows(), 4);
//    output.leftCols<2>() = controls;
//    output.rightCols<2>() = actual;
//    std::cout << output << std::endl;
    
    Matrix2d params;
    auto svd = controls.cwiseAbs2().jacobiSvd (ComputeThinU | ComputeThinV);
    params.row(0) = svd.solve(deviations.col(0)).transpose();
    params.row(1) = svd.solve(deviations.col(1)).transpose();
    
    return params;
}

int main (int argc, char* argv[]) {
    
    if (argc < 2) {
        std::cerr << "Please specify data set name\n";
        return 1;
    }
    
    std::string base_name = argv[1];
    std::string base_path = "input/"+base_name+"/"+base_name;

    dataset data;
    data.read_ground_truth (base_path+"_GT.txt");
    data.read_controls (base_path+"_DR.txt");
    data.read_feature_map (base_path+"_TL.txt");
    data.read_observations (base_path+"_TD.txt");
    
//    std::cout << std::scientific << std::setprecision(16);
//    
//    auto pose = data.initial_pose;
//    std::cout << pose.x() << '\t' << pose.y() << '\t' << pose.bearing() << '\n';
//    for (const auto& pose_delta : data.dead_reckoning) {
//        pose += pose_delta;
//        std::cout << pose.x() << '\t' << pose.y() << '\t' << pose.bearing() << '\n';
//    }
    
    auto vmodel_params = velocity_model_params (data);
    std::cout << vmodel_params << std::endl;
    
    return 0;
}
