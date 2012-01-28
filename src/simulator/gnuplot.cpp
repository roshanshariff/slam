//
//  gnuplot.cpp
//  slam
//
//  Created by Roshan Shariff on 12-01-27.
//  Copyright (c) 2012 University of Alberta. All rights reserved.
//

#include <cmath>

#include <boost/container/flat_map>

#include "simulator/gnuplot.hpp"
#include "utility/bitree.hpp"
#include "utility/utility.hpp"


gnuplot::gnuplot (pose initial_pose) : initial_pose(initial_pose) {
    gnuplot_process = open_process("gnuplot -p", "w");
    fputs ("set size square\n");
}


void gnuplot::plot (size_t timestep) {
    fputs ("plot ");
    for (size_t i = 0; i < data_sources.size(); ++i) {
        if (i > 0) fputs (", ");
        plot_data_source (data_sources[i]);
    }
    fputs ("\n");
    write_buffer();
}


void gnuplot::add_data_source (boost::shared_ptr<const slam_result_type> source,
                               bool autoscale_map, std::string feature_point_style,
                               std::string trajectory_line_style, std::string state_arrow_style) {
    data_source source_info = {
        source, autoscale_map, feature_point_style, trajectory_line_style, state_arrow_style
    };
    data_sources.push_back (source_info);
}


void gnuplot::write_buffer () {
    const record_type* record_ptr = buffer.data();
    const record_type* const end = buffer.data() + buffer.size();
    while (record_ptr < end) {
        record_ptr += std::fwrite(record_ptr, sizeof(record_type), end-record_ptr, gnuplot_process.get());
    }
    buffer.clear();
}


void gnuplot::add_plot (size_t num_records) {
    std::fprintf (gnuplot_process.get(), "'-' binary record=%zu format='%%double%%double' ");
}


void gnuplot::plot_data_source (const data_source& source) {
    plot_map (source);
    plot_trajectory (source);
    plot_state (source);
}


void gnuplot::plot_map (const data_source& source) {
    
    typedef boost::container::flat_map<featureid_t, position> feature_map_type;

    boost::shared_ptr<const feature_map_type> map = source.source->get_map();
    feature_map_type::const_iterator iter = map->begin();
    size_t num_records = 0;
    
    for (; iter != map->end(); ++iter) {
        position pos = initial_pose + iter->second();
        buffer.emplace_back (pos.x(), pos.y());
        ++num_records;
    }

    map.reset();
    
    add_plot (num_records);    
    if (!source.autoscale_map) fputs ("noautoscale ");
    fputs ("with points ");
    fputs (source.feature_point_style.c_str());
}


void gnuplot::plot_trajectory (const data_source& source) {
    
    boost::shared_ptr<const bitree<position> > trajectory = source.source->get_trajectory();
    
    pose state = initial_pose;
    buffer.emplace_back (state.x(), state.y());
    size_t num_records = 1;
    
    for (size_t i = 0; i < trajectory->size(); ++i) {
        state += trajectory->get(i);
        buffer.emplace_back (state.x(), state.y());
        ++num_records;
    }
    
    trajectory.reset();
    
    add_plot (num_records);
    fputs ("noautoscale with lines ");
    fputs (source.trajectory_line_style.c_str());
}


void gnuplot::plot_state (const data_source& source) {
    pose state = initial_pose + source.source->get_state();
    double epsilon = 0.1;
    double xdelta = epsilon * std::cos (state.bearing());
    double ydelta = epsilon * std::sin (state.bearing());
    buffer.emplace_back (state.x() - xdelta, state.y() - ydelta);
    buffer.emplace_back (xdelta, ydelta);
    fputs ("'-' binary record=1 format='%%double%%double%%double%%double' noautoscale with vectors ")
    fputs (source.state_arrow_style().c_str());
}

