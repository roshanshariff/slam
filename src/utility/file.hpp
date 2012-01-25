//
//  file.hpp
//  slam
//
//  Created by Roshan Shariff on 11-09-04.
//  Copyright 2011 University of Alberta. All rights reserved.
//

#ifndef slam_file_hpp
#define slam_file_hpp

#include <boost/shared_ptr.hpp>
#include <cstdio>

inline boost::shared_ptr<FILE> open_file (const char* filename, const char* mode) {
    return boost::shared_ptr<FILE> (std::fopen (filename, mode), &std::fclose);
}

inline boost::shared_ptr<FILE> open_process (const char* command, const char* mode) {
    return boost::shared_ptr<FILE> (std::popen (filename, mode), &std::pclose);
}

#endif
