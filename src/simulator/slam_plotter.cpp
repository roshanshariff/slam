//
//  slam_plotter.cpp
//  slam
//
//  Created by Roshan Shariff on 12-01-27.
//  Copyright (c) 2012 University of Alberta. All rights reserved.
//

#include <cmath>
#include <cstdio>
#include <cassert>
#include <sstream>
#include <iomanip>

#include <boost/container/flat_map.hpp>

#include "simulator/slam_plotter.hpp"
#include "utility/bitree.hpp"
#include "utility/utility.hpp"


void slam_plotter::add_data_source (boost::shared_ptr<const slam_result_type> source, bool autoscale_map,
                               std::string trajectory_title, std::string landmark_title,
                               std::string feature_point_style, std::string trajectory_line_style,
                               std::string state_arrow_style)
{
    data_source source_info = {
        source, autoscale_map, trajectory_title, landmark_title,
        feature_point_style, trajectory_line_style, state_arrow_style
    };
    data_sources.push_back (source_info);
}


void slam_plotter::plot (size_t timestep) {
    
    if (output_dir) {
        std::ostringstream output_filename;
        output_filename << std::setfill('0') << std::setw(6) << timestep << ".png";
        boost::filesystem::path output_file = (*output_dir)/output_filename.str();
        std::fprintf (gnuplot.handle(), "set output '%s'\n", output_file.c_str());
    }

    if (!title.empty()) std::fprintf (gnuplot.handle(), "set title '%s'\n", title.c_str());
    else gnuplot.puts ("set title\n");
    
    //gnuplot.puts ("set key on inside center bottom horizontal Left reverse\n");
    gnuplot.puts ("set key on inside left top vertical Left reverse\n");
    gnuplot.puts ("set size ratio -1\n");
    gnuplot.puts ("set auto fix\n");
    gnuplot.puts ("set offsets graph 0.2, graph 0.05, graph 0.05, graph 0.05\n");

    std::vector<data_source>::const_iterator iter = data_sources.begin();
    for (; iter != data_sources.end(); ++iter) {
        plot_map (*iter);
        plot_trajectory (*iter);
        plot_state (*iter);
    }
    gnuplot.plot ();
    
    if (output_dir) gnuplot.puts ("set output\n");
}


void slam_plotter::add_title (const std::string& title) {
    if (!title.empty()) std::fprintf (gnuplot.handle(), "title '%s' ", title.c_str());
    else gnuplot.puts ("notitle ");
}


void slam_plotter::plot_map (const data_source& source) {
    
    typedef boost::container::flat_map<featureid_t, position> feature_map_type;
    boost::shared_ptr<const feature_map_type> map = source.source->get_map();
    
    for (feature_map_type::const_iterator iter = map->begin(); iter != map->end(); ++iter) {
        position pos = initial_pose + iter->second;
        gnuplot << pos.x() << pos.y();
    }

    gnuplot.plot (2);    
    if (!source.autoscale_map) gnuplot.puts ("noautoscale ");
    add_title (source.landmark_title);
    gnuplot.puts ("with points ");
    gnuplot.puts (source.feature_point_style.c_str());
}


void slam_plotter::plot_trajectory (const data_source& source) {
    
    boost::shared_ptr<const bitree<pose> > trajectory = source.source->get_trajectory();
    
    pose state = initial_pose;
    gnuplot << state.x() << state.y();

    for (size_t i = 0; i < trajectory->size(); ++i) {
        state += trajectory->get(i);
        gnuplot << state.x() << state.y();
    }
    
    gnuplot.plot (2);
    gnuplot.puts ("noautoscale notitle with lines ");
    gnuplot.puts (source.trajectory_line_style.c_str());
}


void slam_plotter::plot_state (const data_source& source) {

    pose state = initial_pose + source.source->get_state();
    
    double epsilon = 5;
    double xdelta = epsilon * std::cos (state.bearing());
    double ydelta = epsilon * std::sin (state.bearing());
    gnuplot << state.x() << state.y() << xdelta << ydelta;
    
    gnuplot.plot (4);
    gnuplot.puts ("noautoscale with vectors ");
    add_title (source.trajectory_title);
    gnuplot.puts (source.state_arrow_style.c_str());
}


boost::program_options::options_description slam_plotter::program_options () {
    namespace po = boost::program_options;
    po::options_description options ("SLAM Plotting Options");
    options.add_options()
    ("slam-plot-title", po::value<std::string>()->default_value("Simultaneous Localization and Mapping"),
     "Plot title")
    ("slam-plot-output-dir", po::value<std::string>(),
     "Output directory for plots (displayed on screen if unset)");
    return options;
}


slam_plotter::slam_plotter (boost::program_options::variables_map& options, const pose& initial_state)
: initial_pose(initial_state), title (options["slam-plot-title"].as<std::string>())
{
    if (options.count ("slam-plot-output-dir")) {
        output_dir = boost::filesystem::path (options["slam-plot-output-dir"].as<std::string>());
        boost::filesystem::create_directories (*output_dir);
        gnuplot.puts ("set terminal pngcairo font 'Sans,8' size 640, 480\n");
    }
}

