//
//  interfaces.hpp
//  slam
//
//  Created by Roshan Shariff on 12-02-09.
//  Copyright (c) 2012 University of Alberta. All rights reserved.
//

#ifndef slam_interfaces_hpp
#define slam_interfaces_hpp

#include <cmath>
#include <cstddef>
#include <utility>
#include <vector>

#include <boost/range/sub_range.hpp>

#include "utility/container_fwd.hpp"
#include "utility/utility.hpp"

namespace slam {

struct timestep_type {
  explicit timestep_type(std::size_t timestep = 0) : timestep(timestep) {}
  operator std::size_t() const { return timestep; }
  bool operator==(timestep_type other) const {
    return timestep == other.timestep;
  }
  bool operator<(timestep_type other) const {
    return timestep < other.timestep;
  }

  timestep_type& operator++() {
    ++timestep;
    return *this;
  }
  timestep_type& operator--() {
    --timestep;
    return *this;
  }
  timestep_type& operator+=(int x) {
    timestep += x;
    return *this;
  }
  timestep_type& operator-=(int x) {
    timestep -= x;
    return *this;
  }

private:
  std::size_t timestep;
};

using namespace std::rel_ops;

inline timestep_type operator++(timestep_type& t, int) {
  auto copy = t;
  ++t;
  return copy;
}
inline timestep_type operator--(timestep_type& t, int) {
  auto copy = t;
  --t;
  return copy;
}

inline timestep_type operator+(timestep_type t, int x) {
  return t += x;
}
inline timestep_type operator-(timestep_type t, int x) {
  return t -= x;
}

struct featureid_type {
  explicit featureid_type(std::size_t featureid) : featureid(featureid) {}
  operator std::size_t() const { return featureid; }
  bool operator==(featureid_type other) const {
    return featureid == other.featureid;
  }
  bool operator<(featureid_type other) const {
    return featureid < other.featureid;
  }

private:
  std::size_t featureid;
};

struct timestep_listener {
  virtual void timestep(timestep_type) = 0;
  virtual void completed() {}
  virtual ~timestep_listener() = default;
};

struct data_source : public virtual timestep_listener {
  virtual auto current_timestep() const -> timestep_type = 0;
  virtual ~data_source() = default;
};

template <typename State, typename Feature>
struct slam_result : public data_source {

  using state_type = State;
  using feature_type = Feature;

  using trajectory_type = utility::bitree<state_type>;
  using feature_map_type = utility::flat_map<featureid_type, feature_type>;

  virtual auto get_initial_state() const -> state_type { return state_type(); };

  virtual auto get_state(timestep_type t) const -> state_type {
    assert(t <= current_timestep());
    return get_initial_state() + get_trajectory().accumulate(t);
  }

  virtual auto get_feature(featureid_type id) const -> feature_type {
    return get_feature_map().at(id);
  };

  virtual auto get_trajectory() const -> const trajectory_type& = 0;
  virtual auto get_feature_map() const -> const feature_map_type& = 0;

  virtual ~slam_result() = default;
};

template <class ControlModel, class ObsModel>
using slam_result_of = slam_result<typename ControlModel::associated_type,
                                   typename ObsModel::associated_type>;

template <class ControlModel, class ObservationModel>
struct dataset {

  using control_type = typename ControlModel::vector_type;
  using observation_type = typename ObservationModel::vector_type;

  struct observation_info {
    featureid_type id;
    observation_type observation;
  };

  using observation_collection =
      utility::flat_multimap<timestep_type, observation_info>;
  using observation_range = boost::sub_range<const observation_collection>;

  virtual auto current_timestep() const -> timestep_type = 0;

  virtual auto control(timestep_type) const -> const control_type& = 0;
  virtual auto timedelta(timestep_type) const -> double = 0;
  virtual auto timestamp(timestep_type) const -> double = 0;
  virtual auto timestep_at(double timestamp) const -> timestep_type = 0;

  virtual auto observations() const -> observation_range = 0;
  virtual auto observations_at(timestep_type) const -> observation_range = 0;
};
}

#endif
