//
//  gnuplot.hpp
//  slam
//
//  Created by Roshan Shariff on 12-01-27.
//  Copyright (c) 2012 University of Alberta. All rights reserved.
//

#ifndef slam_gnuplot_hpp
#define slam_gnuplot_hpp

#include <cstdio>
#include <cstring>
#include <utility>

#include <boost/container/vector.hpp>
#include <boost/shared_ptr.hpp>

#include "planar_robot/pose.hpp"
#include "planar_robot/position.hpp"
#include "slam/slam_result.hpp"

class gnuplot {
    
    typedef size_t timestep_t;
    typedef size_t featureid_t;
    
    typedef planar_robot::pose pose;
    typedef planar_robot::position position;

    typedef slam_result<pose, position> slam_result_type;
    typedef std::pair<double, double> record_type;

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
    
    boost::shared_ptr<FILE> gnuplot_process;
    boost::container::vector<record_type> buffer;
    boost::container::vector<data_source> data_sources;
    
    pose initial_pose;
    
    /** Implementation member functions. */
    int fputs (const char* str) { return std::fputs (str, gnuplot_process.get()); }
    void write_buffer ();
    void add_plot (size_t num_records);

    void plot_data_source (const data_source&);
    void plot_map (const data_source&);
    void plot_trajectory (const data_source&);
    void plot_state (const data_source&);
        
public:
    
    gnuplot (pose initial_pose);
    
    void plot (timestep_t timestep);
    
    void add_data_source (boost::shared_ptr<const slam_result_type> source, bool autoscale_map,
                          std::string trajectory_title, std::string landmark_title,
                          std::string feature_point_style, std::string trajectory_line_style,
                          std::string state_arrow_style);

};

#endif
