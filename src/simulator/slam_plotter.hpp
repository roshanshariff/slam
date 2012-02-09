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

#include <boost/shared_ptr.hpp>
#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>
#include <boost/utility.hpp>
#include <boost/optional.hpp>

#include "planar_robot/pose.hpp"
#include "planar_robot/position.hpp"
#include "slam/slam_result.hpp"
#include "simulator/gnuplot_process.hpp"


class slam_plotter : boost::noncopyable {
    
    typedef size_t timestep_t;
    typedef size_t featureid_t;
    
    typedef planar_robot::pose pose;
    typedef planar_robot::position position;

    typedef slam_result<pose, position> slam_result_type;

    struct data_source {
        boost::shared_ptr<const slam_result_type> source;
        bool autoscale_map;
        std::string trajectory_title;
        std::string landmark_title;
        std::string feature_point_style;
        std::string trajectory_line_style;
        std::string state_arrow_style;
    };

    /** Data members */
    
    pose initial_pose;
    
    boost::optional<boost::filesystem::path> output_dir;
    
    std::string title;
    gnuplot_process gnuplot;
    std::vector<data_source> data_sources;
    
    /** Implementation member functions. */
    void add_title (const std::string& title);
    void plot_map (const data_source&);
    void plot_trajectory (const data_source&);
    void plot_state (const data_source&);
        
public:
    
    slam_plotter (boost::program_options::variables_map&, const pose& initial_state);
    
    static boost::program_options::options_description program_options ();
    
    void plot (timestep_t timestep);
    
    void add_data_source (boost::shared_ptr<const slam_result_type> source, bool autoscale_map,
                          std::string trajectory_title, std::string landmark_title,
                          std::string feature_point_style, std::string trajectory_line_style,
                          std::string state_arrow_style);

};

#endif
