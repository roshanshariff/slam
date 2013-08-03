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
#include <memory>
#include <type_traits>

namespace utility {
    
    std::shared_ptr<FILE> open_file (const char* filename, const char* mode);
    
    std::shared_ptr<FILE> open_process (const char* command, const char* mode);

    /** Implementation from http://isocpp.org/files/papers/N3656.txt */
    
    template<class T> struct _Unique_if {
        typedef std::unique_ptr<T> _Single_object;
    };
    
    template<class T> struct _Unique_if<T[]> {
        typedef std::unique_ptr<T[]> _Unknown_bound;
    };
    
    template<class T, size_t N> struct _Unique_if<T[N]> {
        typedef void _Known_bound;
    };
    
    template<class T, class... Args>
    typename _Unique_if<T>::_Single_object
    make_unique(Args&&... args) {
        return std::unique_ptr<T>(new T(std::forward<Args>(args)...));
    }
    
    template<class T>
    typename _Unique_if<T>::_Unknown_bound
    make_unique(size_t n) {
        typedef typename std::remove_extent<T>::type U;
        return std::unique_ptr<T>(new U[n]());
    }
    
    template<class T, class... Args>
    typename _Unique_if<T>::_Known_bound
    make_unique(Args&&...) = delete;
    
}

#endif
