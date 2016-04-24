//
//  g2o_clustering.hpp
//  slam
//
//  Created by Roshan Shariff on 2012-09-28.
//  Copyright (c) 2012 University of Alberta. All rights reserved.
//

#ifndef slam_g2o_clustering_hpp
#define slam_g2o_clustering_hpp

#include <algorithm>
#include <memory>
#include <vector>

#include <boost/range/adaptor/reversed.hpp>

#include "slam/g2o_slam.hpp"
#include "slam/interfaces.hpp"
#include "slam/slam_data.hpp"
#include "slam/slam_likelihood.hpp"
#include "slam/slam_result_impl.hpp"
#include "slam/slam_rmse.hpp"

#include "main.hpp"

namespace slam {

template <class ControlModel, class ObservationModel>
class g2o_clustering : public g2o_slam<ControlModel, ObservationModel> {

  using slam_result_type = slam_result_of<ControlModel, ObservationModel>;
  using slam_data_type = slam_data<ControlModel, ObservationModel>;

  using state_type = typename slam_result_type::state_type;
  using feature_type = typename slam_result_type::feature_type;

  struct cluster_type {

    slam_result_impl<state_type, feature_type> estimate;
    double log_likelihood;

    auto distance(const slam_result_type& candidate) -> double {
      return rms_error(estimate, candidate);
    }
  };

  std::shared_ptr<const slam_data_type> data;
  std::vector<cluster_type> clusters;

  double rmse_threshold = 2.5;

public:
  g2o_clustering(const decltype(data)& data,
                 const std::shared_ptr<slam_result_type>& init)
      : g2o_slam<ControlModel, ObservationModel>(init), data(data) {}

  void add(const slam_result_type& candidate) {

    using namespace boost::adaptors;

    this->reinitialise(candidate);
    this->optimise();

    double log_likelihood = slam_log_likelihood(*data, *this);

    bool new_cluster = true;
    for (auto& cluster : reverse(clusters)) {

      if (cluster.distance(*this) <= rmse_threshold) {

        if (log_likelihood > cluster.log_likelihood) {
          cluster.estimate = *this;
          cluster.log_likelihood = log_likelihood;
        }

        new_cluster = false;
        break;
      }
    }

    if (new_cluster) {
      clusters.push_back(
          {slam_result_impl<state_type, feature_type>(*this), log_likelihood});
    }
  }

  void sort_clusters() {
    std::sort(clusters.begin(), clusters.end(),
              [](const cluster_type& a, const cluster_type& b) {
                return a.log_likelihood > b.log_likelihood;
              });
  }

  auto get_clusters() const -> const decltype(clusters)& { return clusters; }
};

} // namespace slam

extern template class slam::g2o_clustering<control_model_type,
                                           observation_model_type>;

#endif
