//
//  gnuplot.cpp
//  slam
//
//  Created by Roshan Shariff on 12-01-27.
//  Copyright (c) 2012 University of Alberta. All rights reserved.
//

#include <cmath>
#include <cassert>

#include <boost/container/flat_map.hpp>

#include "simulator/gnuplot.hpp"
#include "utility/bitree.hpp"
#include "utility/utility.hpp"


gnuplot::gnuplot (pose initial_pose) : initial_pose(initial_pose), num_plotted(0) {
    gnuplot_process = open_process("gnuplot -p", "w");
    fputs ("set key on outside center bottom horizontal Left reverse\n");
    fputs ("set size ratio -1\n");
    fputs ("set auto fix\n");
    fputs ("set offsets graph 0.05, graph 0.05, graph 0.05, graph 0.05\n");
}


void gnuplot::add_data_source (boost::shared_ptr<const slam_result_type> source, bool autoscale_map,
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


void gnuplot::plot (size_t timestep) {

    std::vector<data_source>::const_iterator iter = data_sources.begin();

    for (const char* separator = "plot "; iter != data_sources.end(); ++iter) {

        fputs (separator);
        separator = ", ";

        plot_map (*iter);
        fputs (", ");

        plot_trajectory (*iter);
        fputs (", ");

        plot_state (*iter);
    }
    finish_plotting();
}


void gnuplot::add_plot (size_t columns) {
    assert (buffer.size() >= num_plotted);
    size_t records = buffer.size() - num_plotted;
    assert (records % columns == 0);
    records /= columns;
    std::fprintf (gnuplot_process.get(), "'-' binary record=%zu format='%%%zufloat' ", records, columns);
    num_plotted = buffer.size();
}


void gnuplot::finish_plotting () {

    fputs("\n");

    const float* data = buffer.data();
    size_t remaining = buffer.size();
    while (remaining) remaining -= std::fwrite (data, sizeof(*data), remaining, gnuplot_process.get());

    buffer.clear();
    num_plotted = 0;
}


void gnuplot::add_title (const std::string& title) {
    if (!title.empty()) std::fprintf (gnuplot_process.get(), "title '%s' ", title.c_str());
    else fputs ("notitle ");
}


void gnuplot::plot_map (const data_source& source) {
    
    typedef boost::container::flat_map<featureid_t, position> feature_map_type;
    boost::shared_ptr<const feature_map_type> map = source.source->get_map();
    
    for (feature_map_type::const_iterator iter = map->begin(); iter != map->end(); ++iter) {
        position pos = initial_pose + iter->second;
        buffer.push_back (pos.x());
        buffer.push_back (pos.y());
    }

    add_plot (2);    
    if (!source.autoscale_map) fputs ("noautoscale ");
    add_title (source.landmark_title);
    fputs ("with points ");
    fputs (source.feature_point_style.c_str());
}


void gnuplot::plot_trajectory (const data_source& source) {
    
    boost::shared_ptr<const bitree<pose> > trajectory = source.source->get_trajectory();
    
    pose state = initial_pose;
    buffer.push_back (state.x());
    buffer.push_back (state.y());

    for (size_t i = 0; i < trajectory->size(); ++i) {
        state += trajectory->get(i);
        buffer.push_back (state.x());
        buffer.push_back (state.y());
    }
    
    add_plot (2);
    fputs ("noautoscale notitle with lines ");
    fputs (source.trajectory_line_style.c_str());
}


void gnuplot::plot_state (const data_source& source) {

    pose state = initial_pose + source.source->get_state();
    
    double epsilon = 5;
    double xdelta = epsilon * std::cos (state.bearing());
    double ydelta = epsilon * std::sin (state.bearing());
    buffer.push_back (state.x());
    buffer.push_back (state.y());
    buffer.push_back (xdelta);
    buffer.push_back (ydelta);
    
    add_plot (4);
    fputs ("noautoscale with vectors ");
    add_title (source.trajectory_title);
    fputs (source.state_arrow_style.c_str());
}

