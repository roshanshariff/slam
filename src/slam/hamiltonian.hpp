#ifndef _SLAM_HAMILTONIAN_HPP
#define _SLAM_HAMILTONIAN_HPP

#include <algorithm>
#include <iterator>
#include <boost/range/adaptor/reversed.hpp>
#include <boost/range/irange.hpp>
#include <boost/range/iterator_range_core.hpp>
#include <Eigen/Eigen>

#include "planar_robot/range_bearing_model.hpp"
#include "planar_robot/velocity_model.hpp"
#include "slam/interfaces.hpp"
#include "slam/slam_data.hpp"
#include "slam/slam_result_impl.hpp"
#include "utility/flat_map.hpp"
#include "utility/random.hpp"

namespace slam {

template <class Model>
struct parametrization {

  using type = typename Model::associated_type;
  using vector_type = typename Model::vector_type;
  using matrix_type = typename Model::matrix_type;

  static auto apply(const type& x) -> vector_type { return Model::observe(x); }

  static auto inv(const vector_type& v) -> type {
    return Model::inv_observe(v);
  }

  static auto jacobian(const type& x) -> matrix_type {
    return Model::obs_jacobian(x);
  }

  static auto inv_jacobian(const vector_type& v) -> matrix_type {
    return Model::inv_obs_jacobian(v);
  }

  static auto grad_log_likelihood(const Model& model, const type& x)
      -> vector_type {
    return model.grad_log_likelihood(Model::observe(x));
  }

  static auto log_likelihood(const Model& model, const type& x) -> double {
    return model.log_likelihood(Model::observe(x));
  }
};

template <>
struct parametrization<planar_robot::range_only_model>
    : public parametrization<planar_robot::range_bearing_model> {

  static auto grad_log_likelihood(const planar_robot::range_only_model& model,
                                  const type& pos) -> vector_type {
    return model.grad_log_likelihood(
               planar_robot::range_only_model::observe(pos))(0)
           * (inv_jacobian(apply(pos)).transpose()
              * pos.to_vector().normalized());
  }

  static auto log_likelihood(const planar_robot::range_only_model& model,
                             const type& pos) -> double {
    return model.log_likelihood(planar_robot::range_only_model::observe(pos));
  }
};

using feature_timestep_map = utility::flat_map<featureid_type, timestep_type>;

template <class ControlModel, class ObservationModel>
auto find_feature_anchors(const slam_data<ControlModel, ObservationModel>& data)
    -> feature_timestep_map {
  feature_timestep_map anchor;
  for (const auto& t_obs : data.observations()) {
    const auto t = t_obs.first;
    const auto obs = t_obs.second;
    const auto id = obs.id();
    if (anchor.find(id) == anchor.end()
        || obs.observation().more_accurate_than(
               data.get_observations(id).at(anchor[id]))) {
      anchor[id] = t;
    }
  }
  return anchor;
}

template <class ControlModel, class ObservationModel>
auto sqrt_mass_reparam(const slam_data<ControlModel, ObservationModel>& data,
                       const feature_timestep_map& anchor) -> Eigen::VectorXd {

  static constexpr int state_dim = ControlModel::associated_type::vector_dim;
  static constexpr int feature_dim =
      ObservationModel::associated_type::vector_dim;

  Eigen::VectorXd sqrt_mass(state_dim * data.current_timestep()
                            + feature_dim * anchor.size());

  for (timestep_type t; t < data.current_timestep(); ++t) {
    sqrt_mass.segment<state_dim>(t * state_dim) =
        data.control(t).chol_cov_diag();
  }

  std::size_t feature_segment_idx = state_dim * data.current_timestep();

  for (const auto& id_anchor : anchor) {
    const auto& obs_model =
        data.get_observations(id_anchor.first).at(id_anchor.second);
    const double r_sigma = obs_model.chol_cov_diag()(0);
    const double theta_sigma = r_sigma / (obs_model.mean()(0) + 1);
    sqrt_mass.segment<feature_dim>(feature_segment_idx) =
        Eigen::Vector2d(r_sigma, theta_sigma);

    feature_segment_idx += feature_dim;
  }

  assert(feature_segment_idx == sqrt_mass.size());

  return sqrt_mass;
}

template <class ControlModel, class ObservationModel>
void update_state_reparam(
    const feature_timestep_map& anchor,
    slam_result_of_impl<ControlModel, ObservationModel>& estimate,
    const Eigen::VectorXd& update, double epsilon) {

  static constexpr int state_dim = ControlModel::associated_type::vector_dim;
  static constexpr int feature_dim =
      ObservationModel::associated_type::vector_dim;

  using StateParam = parametrization<ControlModel>;
  using FeatureParam = parametrization<ObservationModel>;

  auto& trajectory = estimate.get_trajectory();

  assert(update.size()
         == state_dim * trajectory.size() + feature_dim * anchor.size());

  for (timestep_type t; t < trajectory.size(); ++t) {
    trajectory[t] =
        StateParam::inv(StateParam::apply(trajectory[t])
                        + epsilon * update.segment<state_dim>(t * state_dim));
  }

  std::size_t feature_segment_idx = state_dim * trajectory.size();

  for (const auto& id_anchor : anchor) {
    const auto id = id_anchor.first;
    const auto base = estimate.get_state(id_anchor.second);
    auto vec = FeatureParam::apply(-base + estimate.get_feature(id))
               + epsilon * update.segment<feature_dim>(feature_segment_idx);
    estimate.get_feature_map()[id] = base + FeatureParam::inv(vec);

    feature_segment_idx += feature_dim;
  }

  assert(feature_segment_idx == update.size());
}

template <class ControlModel, class ObservationModel>
double reparam_log_likelihood(
    const slam_data<ControlModel, ObservationModel>& data,
    const slam_result_of<ControlModel, ObservationModel>& estimate) {

  using StateParam = parametrization<ControlModel>;
  using FeatureParam = parametrization<ObservationModel>;

  double log_likelihood = 0;

  for (timestep_type t; t < estimate.current_timestep(); ++t) {
    log_likelihood += StateParam::log_likelihood(data.control(t),
                                                 estimate.get_trajectory()[t]);
  }

  for (const auto& id_feature : estimate.get_feature_map()) {
    const auto id = id_feature.first;
    const auto feature = id_feature.second;
    for (const auto& t_obs : data.get_observations(id)) {
      auto rel_feature = -estimate.get_state(t_obs.first) + feature;
      log_likelihood += FeatureParam::log_likelihood(t_obs.second, rel_feature);
    }
  }

  std::cout << log_likelihood << '\n';
  return log_likelihood;
}

template <class ControlModel, class ObservationModel>
void reparam_gradient(
    const slam_data<ControlModel, ObservationModel>& data,
    const feature_timestep_map& anchor,
    const slam_result_of<ControlModel, ObservationModel>& estimate,
    Eigen::VectorXd& gradient) {

  using state_type = typename ControlModel::associated_type;
  using feature_type = typename ObservationModel::associated_type;
  using state_vector_type = typename state_type::vector_type;
  using feature_vector_type = typename feature_type::vector_type;
  static constexpr int state_dim = state_type::vector_dim;
  static constexpr int feature_dim = feature_type::vector_dim;

  using StateParam = parametrization<ControlModel>;
  using FeatureParam = parametrization<ObservationModel>;

  assert(gradient.size()
         == state_dim * estimte.current_timestep()
                + feature_dim * anchor.size());

  for (timestep_type t; t < estimate.current_timestep(); ++t) {
    auto grad = StateParam::grad_log_likelihood(data.control(t),
                                                estimate.get_trajectory()[t]);
    gradient.segment<state_dim>(t * state_dim) -= grad;
  }

  std::size_t feature_segment_idx = state_dim * estimate.current_timestep();

  for (const auto& id_feature : estimate.get_feature_map()) {

    const auto id = id_feature.first;
    const auto anchor_t = anchor.at(id);
    const auto feature = id_feature.second;

    const auto compute_grad = [&estimate, &gradient, &feature](
        auto observations, auto timesteps) -> feature_vector_type {

      feature_vector_type accum_grad;
      accum_grad.setZero();

      for (; !timesteps.empty(); timesteps.drop_front()) {

        const timestep_type t(timesteps.front());
        const timestep_type next_t(*std::next(timesteps.begin()));
        const auto rel_feature = -estimate.get_state(t) + feature;

        for (; !observations.empty() && observations.front().first == t;
             observations.drop_front()) {
          accum_grad += FeatureParam::jacobian(rel_feature).transpose()
                        * FeatureParam::grad_log_likelihood(
                              observations.front().second, rel_feature);
        }

        const auto state_delta =
            estimate.get_trajectory().accumulate(t, next_t);
        const auto new_rel_feature = -state_delta + rel_feature;

        const double theta_grad =
            FeatureParam::inv_jacobian(FeatureParam::apply(new_rel_feature))
                .col(1)
                .dot(accum_grad);
        const state_vector_type grad =
            StateParam::inv_jacobian(StateParam::apply(state_delta)).transpose()
            * state_vector_type(accum_grad(0), accum_grad(1), theta_grad);
        gradient.segment<state_dim>(std::min(t, next_t) * state_dim) -= grad;

        accum_grad = state_delta.get_rotation().inverse() * accum_grad;
      }

      return accum_grad;
    };

    const auto& observations = data.get_observations(id);
    const auto obs_range = boost::make_iterator_range(observations);

    feature_vector_type feature_grad;
    feature_grad.setZero();
    feature_grad +=
        compute_grad(obs_range, boost::irange<std::size_t>(0, anchor_t));
    feature_grad +=
        compute_grad(obs_range | boost::adaptors::reversed,
                     boost::irange<std::size_t, int>(
                         estimate.current_timestep(), anchor_t, -1));

    const auto rel_feature = -estimate.get_state(anchor_t) + feature;
    feature_grad =
        FeatureParam::inv_jacobian(FeatureParam::apply(rel_feature)).transpose()
        * feature_grad;
    feature_grad += FeatureParam::grad_log_likelihood(observations.at(anchor_t),
                                                      rel_feature);

    gradient.segment<feature_dim>(feature_segment_idx) -= feature_grad;
    feature_segment_idx += feature_dim;
  }

  assert(feature_segment_idx == gradient.size());
}

template <class ControlModel, class ObservationModel>
class reparam_hmc : public slam_result_of_impl<ControlModel, ObservationModel> {

  using slam_result_impl_type =
      slam_result_of_impl<ControlModel, ObservationModel>;
  using typename slam_result_impl_type::slam_result_type;

  std::shared_ptr<const slam_data<ControlModel, ObservationModel>> data;

  feature_timestep_map anchor;
  Eigen::VectorXd sqrt_mass_diag;

  random_source random;

public:
  reparam_hmc(const decltype(data)& data, const slam_result_type& initial_state,
              unsigned int seed)
      : slam_result_impl_type(initial_state), data(data),
        anchor(find_feature_anchors(*data)),
        sqrt_mass_diag(sqrt_mass_reparam(*data, anchor)), random(seed) {
    for (auto& id_feature : this->get_feature_map()) {
      const auto id = id_feature.first;
      const auto anchor_t = anchor.at(id);
      std::cout << "Feature " << id << " anchored to " << anchor_t << '\n';
      const auto init =
          data->get_observations(id).at(anchor_t).proposal().initial_value(
              random);
      id_feature.second = this->get_state(anchor_t) + init;
    }
  }

  bool update(const double epsilon, const int num_steps) {

    // Make a copy of the state
    slam_result_of_impl<ControlModel, ObservationModel> state(*this);

    // Sample momentum
    Eigen::VectorXd p(sqrt_mass_diag.size());
    for (int i = 0; i < p.size(); ++i) {
      p(i) = random.normal() / sqrt_mass_diag(i);
    }

    const auto current_p = p;

    // Find gradient
    Eigen::VectorXd gradient(sqrt_mass_diag.size());

    gradient.setZero();
    reparam_gradient(*data, anchor, state, gradient);
    gradient *= epsilon / 2;
    p -= gradient;

    for (int step = 0; step < num_steps; ++step) {

      // Make a position update
      update_state_reparam<ControlModel, ObservationModel>(anchor, state, p,
                                                           epsilon);

      // Make a momentum update
      gradient.setZero();
      reparam_gradient(*data, anchor, state, gradient);
      gradient *= step < num_steps - 1 ? epsilon : epsilon / 2;
      p -= gradient;
    }

    // No need to negate p since K is symmetric

    std::cout << "T: " << this->current_timestep()
              << "N: " << this->get_feature_map().size() << '\n';

    const double current_U = -reparam_log_likelihood(*data, *this);
    const double proposed_U = -reparam_log_likelihood(*data, state);

    const double current_K =
        (sqrt_mass_diag.asDiagonal() * current_p).squaredNorm();
    const double proposed_K = (sqrt_mass_diag.asDiagonal() * p).squaredNorm();

    const double accept_prob =
        std::exp(current_U - proposed_U + current_K - proposed_K);
    std::cout << "current U: " << current_U << ", proposed U: " << proposed_U
              << ", current K: " << current_K << ", proposed K: " << proposed_K
              << std::endl;
    if (random.uniform() < std::exp(accept_prob)) {
      slam_result_impl_type::operator=(state);
      return true;
    }
    else {
      return false;
    }
  }
};

} // namespace slam

#endif //_SLAM_HAMILTONIAN_HPP
