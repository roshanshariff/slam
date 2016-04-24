//
//  time_series_plotter.hpp
//  slam
//
//  Created by Roshan Shariff on 12-02-14.
//  Copyright (c) 2012 University of Alberta. All rights reserved.
//

#ifndef slam_time_series_plotter_hpp
#define slam_time_series_plotter_hpp

#include <cstddef>
#include <functional>
#include <string>
#include <utility>
#include <vector>

#include <boost/circular_buffer.hpp>

#include "simulator/gnuplot_process.hpp"
#include "slam/interfaces.hpp"

class time_series_plotter : public slam::timestep_listener {

public:
  using function_type = std::function<float(slam::timestep_type)>;

  struct data_source {

    boost::circular_buffer<float> history;

    function_type function;
    std::string title;
    std::string style;
    bool secondary_axis;

    data_source(std::size_t capacity, const function_type& f,
                const std::string& title, const std::string& style,
                bool secondary)
        : history(capacity), function(f), title(title), style(style),
          secondary_axis(secondary) {}
  };

  time_series_plotter(std::size_t capacity) : history_capacity(capacity) {}

  virtual void timestep(slam::timestep_type) override;

  void add_data_source(const function_type& f, const std::string& title,
                       const std::string& style, bool secondary_axis = false) {

    data_sources.push_back(
        data_source(history_capacity, f, title, style, secondary_axis));
  }

private:
  gnuplot_process gnuplot;
  const std::size_t history_capacity;

  std::pair<float, float> yrange = std::make_pair(0.0, 1.0);
  std::pair<float, float> y2range = std::make_pair(0.0, 1.0);

  std::vector<data_source> data_sources;
};

#endif
