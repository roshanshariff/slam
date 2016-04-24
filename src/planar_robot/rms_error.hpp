//
//  mean_sq_error.h
//  slam
//
//  Created by Roshan Shariff on 2012-09-06.
//  Copyright (c) 2012 University of Alberta. All rights reserved.
//

#ifndef _PLANAR_ROBOT_RMS_ERROR_HPP
#define _PLANAR_ROBOT_RMS_ERROR_HPP

#include <utility>

#include "planar_robot/pose.hpp"
#include "planar_robot/position.hpp"
#include "slam/interfaces.hpp"

namespace planar_robot {

using planar_slam_result = slam::slam_result<pose, position>;

using planar_trajectory = planar_slam_result::trajectory_type;
using planar_map = planar_slam_result::feature_map_type;

double trajectory_rmse(const planar_trajectory& ground_truth,
                       const planar_trajectory& estimate,
                       const pose& initial_state);

double map_rmse(const planar_map& ground_truth, const planar_map& estimates,
                const pose& origin);

std::pair<double, double> map_traj_rmse(const planar_slam_result& ground_truth,
                                        const planar_slam_result& estimate);

std::pair<double, double>
map_traj_rmse_align_start(const planar_slam_result& ground_truth,
                          const planar_slam_result& estimate);

std::pair<double, double>
map_traj_rmse_align_best(const planar_slam_result& ground_truth,
                         const planar_slam_result& estimate);

pose estimate_initial_pose(const planar_map& ground_truth,
                           const planar_map& estimates);

} // namespace planar_robot

#endif /* defined(_PLANAR_ROBOT_RMS_ERROR_HPP) */
