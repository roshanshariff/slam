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

#include "planar_robot/pose.hpp"
#include "planar_robot/velocity_model.hpp"
#include "planar_robot/rms_error.hpp"
#include "utility/bitree.hpp"

using trajectory_type = utility::bitree<planar_robot::pose>;

struct dataset {
    planar_robot::pose initial_pose;
    trajectory_type dead_reckoning;
    trajectory_type dead_reckoning_provided;
    
    void integrate_velocities (const std::string& filename_DR);
    void read_dead_reckoning (const std::string& filename_DRp);
};

void dataset::integrate_velocities (const std::string& filename_DR) {
    std::ifstream in (filename_DR);
    double timestamp, v, w;
    while (in >> timestamp >> v >> w) {
        dead_reckoning.push_back (planar_robot::velocity_model::inv_observe ({v, w}));
    }
}

void dataset::read_dead_reckoning (const std::string& filename_DRp) {
    std::ifstream in (filename_DRp);
    double timestamp, x, y, bearing;
    in >> timestamp >> x >> y >> bearing;
    initial_pose = planar_robot::pose::cartesian (x, y, bearing);
    while (in >> timestamp >> x >> y >> bearing) {
        planar_robot::pose new_pose = -initial_pose + planar_robot::pose::cartesian(x, y, bearing);
        dead_reckoning_provided.push_back (-dead_reckoning_provided.accumulate() + new_pose);
    }
}

int main () {
    
    dataset data;
    data.read_dead_reckoning ("input/Plaza2/Plaza2_DRp.txt");
    data.integrate_velocities("input/Plaza2/Plaza2_DR.txt");
    
    planar_robot::pose integrated = data.initial_pose;
    planar_robot::pose given = data.initial_pose;
    
    std::cout << std::scientific << std::setprecision (15);
    std::cout
    << integrated.x() << '\t' << given.x() << '\t'
    << integrated.y() << '\t' << given.y() << '\t'
    << integrated.bearing() << '\t' << given.bearing() << '\n';
    
    for (size_t i = 0; i < data.dead_reckoning.size(); ++i) {
        integrated += data.dead_reckoning[i];
        given += data.dead_reckoning_provided[i];
        std::cout
        << integrated.x() << '\t' << given.x() << '\t'
        << integrated.y() << '\t' << given.y() << '\t'
        << integrated.bearing() << '\t' << given.bearing() << '\n';
    }
    
    return 0;
}