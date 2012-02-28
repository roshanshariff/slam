//
//  time_series_plotter.cpp
//  slam
//
//  Created by Roshan Shariff on 12-02-14.
//  Copyright (c) 2012 University of Alberta. All rights reserved.
//

#include <cstdio>

#include "simulator/time_series_plotter.hpp"


void time_series_plotter::timestep (slam::timestep_type timestep) {
    
    const int xmax = timestep;
    const int xmin = xmax + 1 - history_capacity;
    std::fprintf (gnuplot.handle(), "set xrange [%d:%d]\n", xmin, xmax);
    
    bool use_y2 = false;
    
    for (auto& source : data_sources) {
        auto value = source.function (timestep);
        
        use_y2 = use_y2 || source.secondary_axis;
        
        auto& range = source.secondary_axis ? y2range : yrange;
        if (value < range.first) range.first = value;
        if (value > range.second) range.second = value;
        source.history.push_back (value);
    }
    std::fprintf (gnuplot.handle(), "set yrange [%f:%f]\n", yrange.first, yrange.second);
    std::fprintf (gnuplot.handle(), "set y2range [%f:%f]\n", y2range.first, y2range.second);
    
    if (use_y2) gnuplot.puts ("set y2tics border\n");
    
    for (auto& source : data_sources) {
        
        int t = int(timestep) - source.history.size();
        for (const auto& value : source.history) gnuplot << (++t) << value;

        gnuplot.plot (2);
        gnuplot.puts (source.secondary_axis ? "axes x1y2 " : "axes x1y1 ");

        if (!source.title.empty()) std::fprintf (gnuplot.handle(), "title '%s' ", source.title.c_str());
        else gnuplot.puts ("notitle ");
        
        gnuplot.puts ("with lines ");
        gnuplot.puts (source.style.c_str());
    }

    gnuplot.plot();
}
