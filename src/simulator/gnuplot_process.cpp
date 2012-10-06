//
//  gnuplot_process.cpp
//  slam
//
//  Created by Roshan Shariff on 12-02-01.
//  Copyright (c) 2012 University of Alberta. All rights reserved.
//

#include <cassert>
#include <stdio.h>

#include "gnuplot_process.hpp"

gnuplot_process::gnuplot_process (bool debug) : debug_mode(debug) {
    if (!debug_mode) fh = popen ("gnuplot -p", "w");
    else fh = fopen ("debug-gnuplot.txt", "w");
}

gnuplot_process::~gnuplot_process () {
    if (fh) {
        if (!debug_mode) pclose (fh);
        else fclose (fh);
    }
}

void gnuplot_process::plot (size_t columns) {

    assert (buffer.size() >= buffer_queued);
    assert ((buffer.size() - buffer_queued) % columns == 0);

    size_t records = (buffer.size() - buffer_queued) / columns;
    buffer_queued += columns * records;

    if (records > 0) {

        if (!plot_started) { puts ("plot "); plot_started = true; }
        else puts (", ");        
        
        if (columns > 1) std::fprintf (fh, "'-' binary record=%zu format='%%%zufloat' ", records, columns);
        else std::fprintf (fh, "'-' binary array=%zu format='%%float' ", records);
    }
}


void gnuplot_process::plot () {
    
    puts ("\n");
    plot_started = false;
    
    const float* data = buffer.data();
    
    if (!debug_mode) {
        while (buffer_queued > 0) {
            size_t written = std::fwrite (data, sizeof(*data), buffer_queued, fh);
            data += written;
            buffer_queued -= written;
        }
    } else {
        size_t line_number = 0;
        while (buffer_queued > 0) {
            std::fprintf(fh, "%4zu: %f\n", ++line_number, *data);
            ++data;
            --buffer_queued;
        }
    }
    
    buffer.clear();
    fflush (fh);
}
