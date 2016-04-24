//
//  utility.cpp
//  slam
//
//  Created by Roshan Shariff on 12-01-25.
//  Copyright (c) 2012 University of Alberta. All rights reserved.
//

#include <stdio.h>

#include "utility/utility.hpp"

std::shared_ptr<FILE> utility::open_file(const char* filename,
                                         const char* mode) {
  return std::shared_ptr<FILE>(fopen(filename, mode), &fclose);
}

std::shared_ptr<FILE> utility::open_process(const char* command,
                                            const char* mode) {
  return std::shared_ptr<FILE>(popen(command, mode), &pclose);
}
