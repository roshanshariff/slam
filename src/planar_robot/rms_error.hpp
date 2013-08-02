//
//  mean_sq_error.h
//  slam
//
//  Created by Roshan Shariff on 2012-09-06.
//  Copyright (c) 2012 University of Alberta. All rights reserved.
//

#ifndef _PLANAR_ROBOT_RMS_ERROR_HPP
#define _PLANAR_ROBOT_RMS_ERROR_HPP

#include "planar_robot/pose.hpp"
#include "planar_robot/position.hpp"
#include "slam/interfaces.hpp"

namespace planar_robot {
    
    using planar_slam_result = slam::slam_result <pose, position>;
    
    using planar_trajectory = planar_slam_result::trajectory_type;
    using planar_map = planar_slam_result::feature_map_type;

    double trajectory_rmse (const planar_trajectory& a, const planar_trajectory& b);

    double map_rmse (const pose& origin, const planar_map& landmarks,
                     const pose& est_origin, const planar_map& estimates);
    
}

#endif /* defined(_PLANAR_ROBOT_RMS_ERROR_HPP) */
