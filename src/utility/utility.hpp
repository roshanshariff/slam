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
#include <string>
#include <utility>
#include <memory>
#include <type_traits>

#include <boost/program_options.hpp>


std::shared_ptr<FILE> open_file (const char* filename, const char* mode);

std::shared_ptr<FILE> open_process (const char* command, const char* mode);


template <class T>
T remember_option (boost::program_options::variables_map& options, const std::string& key, T default_value) {
    if (options[key].empty()) {
        options.insert (std::make_pair (key, boost::program_options::variable_value (default_value, false)));
        return default_value;
    }
    else return options[key].as<T>();
}


template <class Iter>
struct iter_pair_range : std::pair<Iter,Iter> {
    iter_pair_range (const std::pair<Iter,Iter>& x) : std::pair<Iter,Iter>(x) { }
    iter_pair_range (Iter begin, Iter end) : std::pair<Iter,Iter>(begin, end) { }
    auto begin() const -> Iter { return this->first; }
    auto end() const -> Iter { return this->second; }
    auto size() const -> decltype(Iter() - Iter()) { return end() - begin(); }
};


template <class Iter>
inline iter_pair_range<Iter> as_range (std::pair<Iter,Iter> const& range) { return range; }

template <class Iter>
inline iter_pair_range<Iter> as_range (Iter begin, Iter end) { return iter_pair_range<Iter>(begin, end); }

template <typename T, typename... Args>
auto make_unique_helper (std::false_type, Args&&... args) -> std::unique_ptr<T> {
    return std::unique_ptr<T> (new T (std::forward<Args>(args)...));
}

template <typename T, typename... Args>
auto make_unique_helper (std::true_type, Args&&... args) -> std::unique_ptr<T> {
    static_assert (std::extent<T>::value == 0, "make_unique<T[N]>() is forbidden, please use make_unique<T>()");
    using U = typename std::remove_extent<T>::type;
    return std::unique_ptr<T> (new U[sizeof...(Args)] { std::forward<Args>(args)... });
}

template <typename T, typename... Args>
auto make_unique (Args&&... args) -> std::unique_ptr<T> {
    return make_unique_helper<T> (std::is_array<T>(), std::forward<Args>(args)...);
}

#endif
