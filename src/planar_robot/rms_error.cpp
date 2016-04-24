//
//  mean_sq_error.cpp
//  slam
//
//  Created by Roshan Shariff on 2012-09-06.
//  Copyright (c) 2012 University of Alberta. All rights reserved.
//

#include <algorithm>
#include <cassert>
#include <cmath>
#include <utility>

#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/mean.hpp>
#include <boost/accumulators/statistics/stats.hpp>

#include <Eigen/Geometry>

#include "planar_robot/rms_error.hpp"
#include "utility/bitree.hpp"
#include "utility/flat_map.hpp"

double planar_robot::trajectory_rmse(const planar_trajectory& ground_truth,
                                     const planar_trajectory& estimate,
                                     const pose& initial_state) {

  assert(ground_truth.size() >= estimate.size());

  using namespace boost::accumulators;
  accumulator_set<double, stats<tag::mean>> acc;

  for (std::size_t t = 0; t <= estimate.size(); ++t) {
    acc((-ground_truth.accumulate(t) + (initial_state + estimate.accumulate(t)))
            .distance_squared());
  }

  return std::sqrt(mean(acc));
}

double planar_robot::map_rmse(const planar_map& ground_truth,
                              const planar_map& estimates, const pose& origin) {

  using namespace boost::accumulators;
  accumulator_set<double, stats<tag::mean>> acc;

  auto landmark = ground_truth.cbegin();
  for (const auto& estimate : estimates) {

    while (landmark != ground_truth.cend()
           && landmark->first != estimate.first) {
      ++landmark;
    }
    assert(landmark != ground_truth.cend());

    acc((landmark->second.to_vector() - (origin + estimate.second).to_vector())
            .squaredNorm());
  }

  return std::sqrt(mean(acc));
}

std::pair<double, double>
planar_robot::map_traj_rmse(const planar_slam_result& ground_truth,
                            const planar_slam_result& estimate) {

  double map = map_rmse(ground_truth.get_feature_map(),
                        estimate.get_feature_map(), pose());
  double traj = trajectory_rmse(
      ground_truth.get_trajectory(), estimate.get_trajectory(),
      -ground_truth.get_initial_state() + estimate.get_initial_state());
  return std::make_pair(map, traj);
}

std::pair<double, double>
planar_robot::map_traj_rmse_align_start(const planar_slam_result& ground_truth,
                                        const planar_slam_result& estimate) {

  double map =
      map_rmse(ground_truth.get_feature_map(), estimate.get_feature_map(),
               ground_truth.get_initial_state());
  double traj = trajectory_rmse(ground_truth.get_trajectory(),
                                estimate.get_trajectory(), pose());
  return std::make_pair(map, traj);
}

std::pair<double, double>
planar_robot::map_traj_rmse_align_best(const planar_slam_result& ground_truth,
                                       const planar_slam_result& estimate) {

  pose isometry = estimate_initial_pose(ground_truth.get_feature_map(),
                                        estimate.get_feature_map());

  double map = map_rmse(ground_truth.get_feature_map(),
                        estimate.get_feature_map(), isometry);
  double traj =
      trajectory_rmse(ground_truth.get_trajectory(), estimate.get_trajectory(),
                      -ground_truth.get_initial_state() + isometry
                          + estimate.get_initial_state());
  return std::make_pair(map, traj);
}

auto planar_robot::estimate_initial_pose(const planar_map& ground_truth,
                                         const planar_map& estimates) -> pose {

  using namespace planar_robot;
  std::vector<std::pair<position, position>> features_vec;

  auto estimate = estimates.cbegin();
  for (const auto& landmark : ground_truth) {

    while (estimate != estimates.cend() && estimate->first != landmark.first) {
      ++estimate;
    }
    assert(estimate != estimates.cend());

    features_vec.emplace_back(landmark.second, estimate->second);
  }

  Eigen::Matrix2Xd landmarks(2, features_vec.size());
  Eigen::Matrix2Xd est_landmarks(2, features_vec.size());
  for (std::size_t i = 0; i < features_vec.size(); ++i) {
    landmarks.col(i) = features_vec[i].first.to_vector();
    est_landmarks.col(i) = features_vec[i].second.to_vector();
  }

  const Eigen::Isometry2d isometry{
      Eigen::umeyama(est_landmarks, landmarks, false)};
  return pose::from_trans_rot(
      isometry.translation(),
      Eigen::Rotation2Dd(0).fromRotationMatrix(isometry.linear()));
}
