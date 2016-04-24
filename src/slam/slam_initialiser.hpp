//
//  slam_initialiser.hpp
//  slam
//
//  Created by Roshan Shariff on 2012-10-03.
//  Copyright (c) 2012 University of Alberta. All rights reserved.
//

#ifndef slam_slam_initialiser_hpp
#define slam_slam_initialiser_hpp

#include <cassert>

#include "slam/interfaces.hpp"
#include "slam/slam_data.hpp"
#include "slam/slam_result_impl.hpp"
#include "utility/random.hpp"

namespace slam {

template <class ControlModel, class ObservationModel>
class slam_initialiser
    : public slam_data<ControlModel, ObservationModel>::listener,
      public slam_result_of_impl<ControlModel, ObservationModel> {

  using slam_data_type = slam_data<ControlModel, ObservationModel>;

  random_source random;

public:
  slam_initialiser(unsigned int seed) : random(seed) {}

  virtual void control(timestep_type t, const ControlModel& control) override {
    (void)t; // Silence unused variable warning
    assert(t == this->current_timestep());
    auto initial_estimate = control.proposal().initial_value(random);
    this->get_trajectory().push_back(initial_estimate);
  }

  virtual void
  observation(timestep_type t,
              const typename slam_data_type::observation_info& obs) override {
    assert(t == this->current_timestep());
    if (obs.index() == 0) {
      auto initial_estimate =
          obs.observation().proposal().initial_value(random);
      this->get_feature_map()[obs.id()] = this->get_state(t) + initial_estimate;
    }
  }
};
}

#endif
