//
//  fastslam_mcmc.hpp
//  slam
//
//  Created by Roshan Shariff on 12-03-06.
//  Copyright (c) 2012 University of Alberta. All rights reserved.
//

#ifndef slam_fastslam_mcmc_hpp
#define slam_fastslam_mcmc_hpp

#include <functional>
#include <iostream>
#include <memory>

#include <boost/program_options.hpp>

#include <Eigen/Eigen>

#include "slam/fastslam.hpp"
#include "slam/interfaces.hpp"
#include "slam/mcmc_slam.hpp"

#include "main.hpp"

namespace slam {

template <class ControlModel, class ObservationModel>
class fastslam_mcmc : public timestep_listener {

  using fastslam_type = fastslam<ControlModel, ObservationModel>;
  using mcmc_slam_type = mcmc_slam<ControlModel, ObservationModel>;

  std::shared_ptr<fastslam_type> m_fastslam;
  std::shared_ptr<mcmc_slam_type> m_mcmc_slam;

  typename fastslam_type::particle_type sample_particle() const;

  unsigned int mcmc_steps;
  unsigned int mcmc_end_steps;
  unsigned int mcmc_feature_updates;
  unsigned int num_feature_samples;

  bool resample_required = false;

  std::size_t mcmc_slam_vertices() const {
    return m_mcmc_slam->state_estimates.size()
           + m_mcmc_slam->feature_estimates.size();
  }

public:
  fastslam_mcmc(const fastslam_mcmc&) = delete;
  fastslam_mcmc& operator=(const fastslam_mcmc&) = delete;

  fastslam_mcmc(boost::program_options::variables_map& options);
  static boost::program_options::options_description program_options();

  virtual void timestep(timestep_type) override;

  virtual void completed() override;

  void set_fastslam(const decltype(m_fastslam)& p) { m_fastslam = p; }
  void set_mcmc_slam(const decltype(m_mcmc_slam)& p) { m_mcmc_slam = p; }
};
}

template <class ControlModel, class ObservationModel>
void slam::fastslam_mcmc<ControlModel, ObservationModel>::timestep(
    const timestep_type t) {

  if (m_mcmc_slam) {
    if (!m_fastslam) {
      auto vertices_before = mcmc_slam_vertices();
      m_mcmc_slam->timestep(t);
      auto vertices_after = mcmc_slam_vertices();
      unsigned int num_updates =
          mcmc_steps * (vertices_after - vertices_before);
      for (unsigned int i = 0; i < num_updates; ++i) m_mcmc_slam->update();
      return;
    }
  }
  else
    return;

  m_fastslam->timestep(t);
  if (t == 0) return;

  bool resample_required_previous = resample_required;
  resample_required = m_fastslam->resample_required();

  if (resample_required && resample_required_previous) {
    m_fastslam->get_trajectory();
    m_mcmc_slam->timestep(t);
    std::cout << "Reinitialising FastSLAM... ";
    m_fastslam->particles.reinitialize(
        m_fastslam->num_particles,
        std::bind(&fastslam_mcmc::sample_particle, this));
    std::cout << "done\n";
    resample_required = false;
  }
}

template <class ControlModel, class ObservationModel>
void slam::fastslam_mcmc<ControlModel, ObservationModel>::completed() {

  std::cout << "Running MCMC steps at end.\n";

  if (m_mcmc_slam) {
    for (unsigned int i = 0; i < mcmc_end_steps; ++i) m_mcmc_slam->update();
  }
}

template <class ControlModel, class ObservationModel>
auto slam::fastslam_mcmc<ControlModel, ObservationModel>::sample_particle()
    const -> typename fastslam_type::particle_type {

  auto num_updates = mcmc_steps;
  num_updates *= m_mcmc_slam->state_estimates.size()
                 + m_mcmc_slam->feature_estimates.size();

  for (std::size_t i = 0; i < num_updates; ++i) { m_mcmc_slam->update(); }

  typename fastslam_type::particle_type particle;

  for (timestep_type t(1); t <= m_mcmc_slam->current_timestep(); ++t) {
    particle.trajectory.previous =
        std::make_shared<decltype(particle.trajectory)>(particle.trajectory);
    particle.trajectory.state = m_mcmc_slam->get_trajectory().accumulate(t);
  }

  for (std::size_t fi = 0; fi < m_mcmc_slam->feature_estimates.size(); ++fi) {

    const auto& f = m_mcmc_slam->feature_estimates[fi];

    auto mcmc_feature = [&]() {
      return m_mcmc_slam->get_trajectory().accumulate(f.parent_timestep)
             + f.estimate;
    };

    using feature_dist = typename fastslam_type::feature_dist;
    using feature_vector = typename feature_dist::vector_type;
    constexpr int feature_dim = fastslam_type::vec::feature_dim;

    const feature_vector base = mcmc_feature().to_vector();
    Eigen::Matrix<double, feature_dim, Eigen::Dynamic> samples(
        feature_dim, num_feature_samples);

    for (unsigned int i = 0; i < num_feature_samples; ++i) {
      for (unsigned int j = 0; j < mcmc_feature_updates; ++j) {
        m_mcmc_slam->update(
            typename mcmc_slam_type::feature_edge(*m_mcmc_slam, fi), false);
      }
      feature_vector feature = mcmc_feature().to_vector();
      samples.col(i) = feature_dist::subtract(base, feature);
    }

    const feature_vector base_innov =
        samples.rowwise().sum() / (1 + num_feature_samples);

    for (unsigned int i = 0; i < num_feature_samples; ++i) {
      samples.col(i) = feature_dist::subtract(base_innov, samples.col(i));
    }

    samples /= std::sqrt(num_feature_samples);

    feature_dist feature;
    feature.vector_model().mean() = feature_dist::subtract(base, base_innov);
    feature.vector_model().chol_cov() =
        samples.transpose()
            .householderQr()
            .matrixQR()
            .template topLeftCorner<feature_dim, feature_dim>()
            .template triangularView<Eigen::Upper>()
            .transpose();

    particle.features.insert(f.id(), feature);
  }

  return particle;
}

template <class ControlModel, class ObservationModel>
auto slam::fastslam_mcmc<ControlModel, ObservationModel>::program_options()
    -> boost::program_options::options_description {
  namespace po = boost::program_options;
  po::options_description options("FastSLAM-MCMC Parameters");
  options.add_options() //
      ("mcmc-steps", po::value<unsigned int>()->default_value(1),
       "MCMC iterations per graph vertex") //
      ("mcmc-end-steps", po::value<unsigned int>()->default_value(0),
       "MCMC iterations after simulation") //
      ("mcmc-feature-updates", po::value<unsigned int>()->default_value(10),
       "MCMC iterations per feature") //
      ("feature-samples", po::value<unsigned int>()->default_value(100),
       "number of independent samples of each feature");
  return options;
}

template <class ControlModel, class ObservationModel>
slam::fastslam_mcmc<ControlModel, ObservationModel>::fastslam_mcmc(
    boost::program_options::variables_map& options)
    : mcmc_steps(options["mcmc-steps"].as<unsigned int>()),
      mcmc_end_steps(options["mcmc-end-steps"].as<unsigned int>()),
      mcmc_feature_updates(options["mcmc-feature-updates"].as<unsigned int>()),
      num_feature_samples(options["feature-samples"].as<unsigned int>()) {}

extern template class slam::fastslam_mcmc<control_model_type,
                                          fastslam_observation_model_type>;

#endif
