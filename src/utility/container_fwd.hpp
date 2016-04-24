//
//  container_fwd.hpp
//  slam
//
//  Created by Roshan Shariff on 12-02-10.
//  Copyright (c) 2012 University of Alberta. All rights reserved.
//

#ifndef slam_container_fwd_hpp
#define slam_container_fwd_hpp

#include <memory>

#include <boost/container/container_fwd.hpp>

namespace utility {

using boost::container::flat_map;
using boost::container::flat_multimap;

template <class Grp, class Alloc = std::allocator<Grp>>
class bitree;

} // namespace utility

#endif
