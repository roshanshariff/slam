//
//  gnuplot_process.hpp
//  slam
//
//  Created by Roshan Shariff on 12-02-01.
//  Copyright (c) 2012 University of Alberta. All rights reserved.
//

#ifndef slam_gnuplot_process_hpp
#define slam_gnuplot_process_hpp

#include <cstdio>
#include <vector>


class gnuplot_process {
    
    FILE* fh;
    std::vector<float> buffer;
    size_t buffer_queued = 0;
    
    bool plot_started = false;
    const bool debug_mode;
    
public:
    
    gnuplot_process (bool debug = false);
    gnuplot_process (const gnuplot_process&) = delete;
    gnuplot_process& operator= (const gnuplot_process&) = delete;
    virtual ~gnuplot_process ();
    
    FILE* handle () { return fh; }
    
    gnuplot_process& operator<< (float x) { buffer.push_back(x); return *this; }
    
    int puts (const char* str) { return std::fputs (str, fh); }
    
    void plot (size_t columns);
    void plot ();
};


#endif
