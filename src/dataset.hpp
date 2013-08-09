//
//  dataset.hpp
//  slam
//
//  Created by Roshan Shariff on 2013-08-07.
//  Copyright (c) 2013 University of Alberta. All rights reserved.
//

#ifndef dataset_hpp
#define dataset_hpp

#include <string>
#include <utility>
#include <memory>

#include <boost/filesystem/path.hpp>

#include "planar_robot/pose.hpp"
#include "planar_robot/position.hpp"
#include "planar_robot/velocity_model.hpp"
#include "planar_robot/range_bearing_model.hpp"
#include "slam/interfaces.hpp"

using range_only_data_type = std::tuple<
std::shared_ptr<slam::dataset<planar_robot::velocity_model, planar_robot::range_only_model>>,
std::shared_ptr<slam::slam_result<planar_robot::pose, planar_robot::position>>
>;

auto read_range_only_data (boost::filesystem::path dir, const std::string& name)
-> range_only_data_type;

#endif
