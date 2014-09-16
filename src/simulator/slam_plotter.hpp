//
//  slam_plotter.hpp
//  slam
//
//  Created by Roshan Shariff on 12-01-27.
//  Copyright (c) 2012 University of Alberta. All rights reserved.
//

#ifndef slam_slam_plotter_hpp
#define slam_slam_plotter_hpp

#include <vector>
#include <string>
#include <memory>

#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>
#include <boost/optional.hpp>

#include "planar_robot/pose.hpp"
#include "planar_robot/position.hpp"
#include "slam/interfaces.hpp"
#include "simulator/gnuplot_process.hpp"


class slam_plotter : public slam::timestep_listener {
    
    using pose = planar_robot::pose;
    using position = planar_robot::position;

    using slam_result_type = slam::slam_result<pose, position>;

    struct data_source {
        std::shared_ptr<slam_result_type> source;
        bool autoscale_map;
        std::string trajectory_title;
        std::string landmark_title;
        std::string feature_point_style;
        std::string trajectory_line_style;
        std::string state_arrow_style;
    };

    /** Data members */
    
    boost::optional<boost::filesystem::path> output_dir;
    
    std::string title;
    gnuplot_process gnuplot;
    std::vector<data_source> data_sources;
    std::shared_ptr<slam_result_type> ground_truth;
    bool match_ground_truth = false;
    
    /** Implementation member functions. */
    void add_title (const std::string& title);
    void plot_map (const data_source&, const pose& origin);
    void plot_trajectory (const data_source&, slam::timestep_type t, const pose& origin);
    void plot_state (const data_source&, slam::timestep_type t, const pose& origin);
        
public:
    
    slam_plotter (boost::program_options::variables_map&);
    
    static boost::program_options::options_description program_options ();
    
    virtual void timestep (slam::timestep_type) override;
    
    virtual void completed () override;
    
    void set_ground_truth (std::shared_ptr<slam_result_type> source);
    
    void add_data_source (std::shared_ptr<slam_result_type> source, bool autoscale_map,
                          std::string trajectory_title, std::string landmark_title,
                          std::string feature_point_style, std::string trajectory_line_style,
                          std::string state_arrow_style);

    void plot (boost::optional<slam::timestep_type> timestep = {});
    
};

#endif
