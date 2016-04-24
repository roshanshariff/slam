//
//  listeners.hpp
//  slam
//
//  Created by Roshan Shariff on 12-02-10.
//  Copyright (c) 2012 University of Alberta. All rights reserved.
//

#ifndef slam_listeners_hpp
#define slam_listeners_hpp

#include <algorithm>
#include <memory>
#include <vector>

namespace utility {

template <class Listener>
class listeners {

  mutable std::vector<std::weak_ptr<Listener>> weak_ptrs;

public:
  void add(const std::shared_ptr<Listener>& l) { weak_ptrs.push_back(l); }

  template <class Functor>
  void for_each(Functor) const;
};

template <class Listener>
template <class Functor>
void listeners<Listener>::for_each(Functor f) const {

  auto invoke_or_remove =
      [&f](const std::weak_ptr<Listener>& weak_ptr) -> bool {
    if (auto listener = weak_ptr.lock()) {
      f(listener.get());
      return false;
    }
    else {
      return true;
    }
  };

  weak_ptrs.erase(
      std::remove_if(weak_ptrs.begin(), weak_ptrs.end(), invoke_or_remove),
      weak_ptrs.end());
}

} // namespace utility

#endif
