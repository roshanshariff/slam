/*
 * landmark_sensor.hpp
 *
 *  Created on: 2011-08-26
 *      Author: roshan
 */

#ifndef LANDMARK_SENSOR_HPP_
#define LANDMARK_SENSOR_HPP_

#include <cstddef>
#include <fstream>
#include <string>
#include <vector>

#include <boost/math/constants/constants.hpp>
#include <boost/program_options.hpp>

#include "planar_robot/pose.hpp"
#include "planar_robot/position.hpp"
#include "planar_robot/range_bearing_model.hpp"
#include "slam/interfaces.hpp"
#include "utility/random.hpp"

#include "main.hpp"

namespace planar_robot {

class landmark_sensor {

public:
  landmark_sensor(const boost::program_options::variables_map&);

  static auto program_options() -> boost::program_options::options_description;

  template <class ObservationModel, class Observer>
  void sense(const typename ObservationModel::builder& model_builder,
             random_source&, const pose&, Observer) const;

  auto num_features() const -> std::size_t { return landmarks.size(); }
  auto get_feature(size_t index) const -> const position& {
    return landmarks.at(index);
  }

private:
  double max_range, min_range;
  mutable unsigned long hits = 0;
  std::vector<position> landmarks;

public:
  auto num_observations() -> unsigned long const { return hits; }
};

} // namespace planar_robot

template <class ObservationModel, class Observer>
void planar_robot::landmark_sensor::sense(
    const typename ObservationModel::builder& model_builder,
    random_source& random, const pose& state, Observer observer) const {

  for (std::size_t i = 0; i < landmarks.size(); ++i) {
    auto rel_pos = -state + landmarks[i];
    if (min_range < rel_pos.distance() && rel_pos.distance() < max_range) {
      ++hits;
      auto mean_obs = ObservationModel::observe(rel_pos);
      observer(slam::featureid_type(i), model_builder(mean_obs)(random));
    }
  }
}

#endif /* LANDMARK_SENSOR_HPP_ */
