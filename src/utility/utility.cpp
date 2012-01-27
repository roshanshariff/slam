//
//  utility.cpp
//  slam
//
//  Created by Roshan Shariff on 12-01-25.
//  Copyright (c) 2012 University of Alberta. All rights reserved.
//

#include <stdio.h>

#include "utility/utility.hpp"

boost::shared_ptr<FILE> open_file (const char* filename, const char* mode) {
    return boost::shared_ptr<FILE> (fopen (filename, mode), &fclose);
}

boost::shared_ptr<FILE> open_process (const char* command, const char* mode) {
    return boost::shared_ptr<FILE> (popen (command, mode), &pclose);
}

