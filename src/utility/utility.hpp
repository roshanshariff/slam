//
//  utility.hpp
//  slam
//
//  Created by Roshan Shariff on 11-09-04.
//  Copyright 2011 University of Alberta. All rights reserved.
//

#ifndef slam_utility_hpp
#define slam_utility_hpp

#include <cstdio>
#include <utility>

#include <boost/shared_ptr.hpp>

boost::shared_ptr<FILE> open_file (const char* filename, const char* mode);

boost::shared_ptr<FILE> open_process (const char* command, const char* mode);

template <class U, class Compare = std::less<U> >
class pair_compare_first {
    Compare cmp;
public:
    pair_compare_first (Compare cmp = Compare()) : cmp(cmp) { }
    template <class V> bool operator() (const std::pair<U,V>& a, const std::pair<U,V>& b) {
        return cmp (a.first, b.first);
    }
};

#endif
