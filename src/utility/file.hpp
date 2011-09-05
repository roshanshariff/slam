//
//  file.hpp
//  slam
//
//  Created by Roshan Shariff on 11-09-04.
//  Copyright 2011 University of Alberta. All rights reserved.
//

#ifndef slam_file_hpp
#define slam_file_hpp

#include <tr1/memory>
#include <cstdio>

inline std::tr1::shared_ptr<FILE> open_file (const char* filename, const char* mode) {
    return std::tr1::shared_ptr<FILE> (std::fopen (filename, mode), &std::fclose);
}

#endif
