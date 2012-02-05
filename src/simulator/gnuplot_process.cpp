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

gnuplot_process::gnuplot_process ()
: fh (popen ("gnuplot -p", "w")), buffer_queued(0), plot_started(false) { }

gnuplot_process::~gnuplot_process () {
    if (fh) pclose (fh);
}

void gnuplot_process::plot (size_t columns) {

    assert (buffer.size() >= buffer_queued);

    if (!plot_started) { puts ("plot "); plot_started = true; }
    else puts (", ");

    size_t records = (buffer.size() - buffer_queued) / columns;
    buffer_queued += columns * records;

    if (columns > 1) std::fprintf (fh, "'-' binary record=%zu format='%%%zufloat' ", records, columns);
    else std::fprintf (fh, "'-' binary array=%zu format='%%float' ", records);
}


void gnuplot_process::plot () {
    
    puts ("\n");
    plot_started = false;
    
    const float* data = buffer.data();
    while (buffer_queued > 0) {
        size_t written = std::fwrite (data, sizeof(*data), buffer_queued, fh);
        data += written;
        buffer_queued -= written;
    }
    
    buffer.clear();
}
