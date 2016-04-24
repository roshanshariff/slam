#ifndef _PLANAR_ROBOT_VELOCITY_MODEL_HPP
#define _PLANAR_ROBOT_VELOCITY_MODEL_HPP

#include <cmath>
#include <cstdlib>
#include <functional>
#include <iostream>

#include <boost/math/constants/constants.hpp>
#include <boost/program_options.hpp>

#include <Eigen/Core>

#include "planar_robot/pose.hpp"
#include "slam/interfaces.hpp"
#include "utility/geometry.hpp"
#include "utility/nnls.hpp"
#include "utility/random.hpp"

namespace planar_robot {

struct velocity_slip_model
    : public independent_normal_base<3, velocity_slip_model> {

  using base_type = independent_normal_base<3, velocity_slip_model>;
  using associated_type = pose;

  velocity_slip_model() = default;
  velocity_slip_model(const vector_type& mean, const vector_type& stddev)
      : base_type(mean, stddev) {}

  static auto subtract(const vector_type& a, const vector_type& b)
      -> vector_type {
    return a - b;
  }

  static auto observe(const associated_type& dp) -> vector_type {

    const double r = 0.5 * dp.distance_squared() / dp.y();

    if (std::isfinite(r)) {
      const double alpha = r >= 0 ? std::atan2(dp.x(), r - dp.y())
                                  : std::atan2(-dp.x(), dp.y() - r);
      return {r * alpha, alpha, wrap_angle(dp.bearing() - alpha)};
    }
    else {
      return {dp.x(), 0.0, dp.bearing()};
    }
  }

  static auto inv_observe(const vector_type& control) -> associated_type {

    const double s = control(0), alpha = control(1), beta = control(2),
                 r = s / alpha;

    if (std::isfinite(r)) {
      return pose::cartesian(r * std::sin(alpha), r - r * std::cos(alpha),
                             alpha + beta);
    }
    else {
      return pose::cartesian(s, 0.0, beta);
    }
  }

  static auto obs_jacobian(const associated_type& dp) -> matrix_type {

    const double x = dp.x(), y = dp.y();
    const double ry = 0.5 * dp.distance_squared();
    const double r = ry / y;

    Eigen::Matrix3d jacobian;

    if (std::isfinite(r)) {
      const double alpha = r >= 0 ? std::atan2(dp.x(), r - dp.y())
                                  : std::atan2(-dp.x(), dp.y() - r);
      // clang-format off
      jacobian <<
        alpha*x/y-1, alpha*(1-r/y)+(x/y), 0,
        -1/r,        x/ry,                0,
        1/r,         -x / ry,             1;
      // clang-format on
    }
    else {
      // clang-format off
      jacobian <<
        1, 0,    0,
        0, 2/x,  0,
        0, -2/x, 1;
      // clang-format on
    }

    return jacobian;
  }

  static auto inv_obs_jacobian(const vector_type& control) -> matrix_type {

    const double s = control(0), alpha = control(1);
    const double inv_alpha = 1 / alpha;

    Eigen::Matrix3d jacobian;

    if (std::isfinite(inv_alpha)) {
      const double r = s * inv_alpha;
      const double sin_alpha = std::sin(alpha), cos_alpha = std::cos(alpha);
      const double dx_ds = inv_alpha * sin_alpha,
                   dy_ds = inv_alpha * (1 - cos_alpha);
      // clang-format off
      jacobian <<
        dx_ds, r*(cos_alpha-dx_ds), 0,
        dy_ds, r*(sin_alpha-dy_ds), 0,
        0,     1, 1;
      // clang-format on
    }
    else {
      // clang-format off
      jacobian <<
        1, 0,   0,
        0, s/2, 0,
        0, 1,   1;
      // clang-format on
    }

    return jacobian;
  }

  static auto from_steering(double s, double alpha, double beta = 0.0)
      -> vector_type {
    return {s, alpha, beta};
  }

  class proposal_dist {

    const velocity_slip_model& model;

  public:
    explicit proposal_dist(decltype(model) model) : model(model) {}

    using result_type = associated_type;
    static constexpr int vector_dim = velocity_slip_model::vector_dim;

    auto operator()(random_source& random) const -> result_type {
      return inv_observe(model(random));
    }
    auto likelihood(const result_type& x) const -> double {
      return model.likelihood(observe(x));
    }
    auto log_likelihood(const result_type& x) const -> double {
      return model.log_likelihood(observe(x));
    }
    auto initial_value(random_source&) const -> result_type {
      return inv_observe(model.mean());
    }
  };

  auto proposal() const -> proposal_dist { return proposal_dist(*this); }

  class builder : public std::unary_function<vector_type, velocity_slip_model> {

    Eigen::Matrix3d mat_variance;

  public:
    builder(double a1, double a2, double a3, double a4, double a5, double a6) {
      // clang-format off
      mat_variance <<
        a1, a2, 0,
        a3, a4, 0,
        a5, a6, 0;
      // clang-format on
    }

    builder(const boost::program_options::variables_map&);

    static auto program_options()
        -> boost::program_options::options_description;

    auto operator()(const vector_type& control, double dt) const
        -> velocity_slip_model {
      return {dt * control,
              dt * (dt * mat_variance * control.cwiseAbs2()).cwiseSqrt()};
    }
  };
};

struct velocity_model : public independent_normal_base<2, velocity_model> {

  using base_type = independent_normal_base<2, velocity_model>;
  using associated_type = pose;

  velocity_model() = default;

  velocity_model(const vector_type& mean, const vector_type& stddev)
      : base_type(mean, stddev) {}

  static auto subtract(const vector_type& a, const vector_type& b)
      -> vector_type {
    return a - b;
  }

  static auto observe(const associated_type& dp) -> vector_type {
    auto observation = velocity_slip_model::observe(dp);
    return {observation(0), observation(1)};
  }

  static auto inv_observe(const vector_type& control) -> associated_type {
    return velocity_slip_model::inv_observe({control(0), control(1), 0.0});
  }

  static auto from_steering(double v, double w) -> vector_type {
    return {v, w};
  }

  class proposal_dist {

    const velocity_model& model;

  public:
    using result_type = associated_type;
    static constexpr int vector_dim = velocity_model::vector_dim;

    explicit proposal_dist(decltype(model) model) : model(model) {}
    auto operator()(random_source& random) const -> result_type {
      return inv_observe(model(random));
    }
    auto likelihood(const result_type& x) const -> double {
      return model.likelihood(observe(x));
    }
    auto log_likelihood(const result_type& x) const -> double {
      return model.log_likelihood(observe(x));
    }
    auto initial_value(random_source&) const -> result_type {
      return inv_observe(model.mean());
    }
  };

  auto proposal() const -> proposal_dist { return proposal_dist(*this); }

  class builder {

    Eigen::Matrix2d mat_variance;

  public:
    builder(const Eigen::Matrix2d mat_variance) : mat_variance(mat_variance) {}

    builder(double a1, double a2, double a3, double a4) {
      // clang-format off
      mat_variance <<
        a1, a2,
        a3, a4;
      // clang-format on
    }

    builder(const boost::program_options::variables_map&);

    static auto program_options()
        -> boost::program_options::options_description;

    auto operator()(const vector_type& control, double dt) const
        -> velocity_model {
      return {dt * control,
              dt * (dt * mat_variance * control.cwiseAbs2()).cwiseSqrt()};
    }
  };

  template <class ObservationModel, class Feature>
  static auto learn_from_data(
      const slam::dataset<velocity_model, ObservationModel>& dataset,
      const slam::slam_result<pose, Feature>& ground_truth) -> builder;
};

template <class ObservationModel, class Feature>
auto velocity_model::learn_from_data(
    const slam::dataset<velocity_model, ObservationModel>& dataset,
    const slam::slam_result<pose, Feature>& ground_truth) -> builder {

  const auto& trajectory = ground_truth.get_trajectory();
  assert(dataset.current_timestep() == ground_truth.current_timestep());
  const std::size_t timesteps = dataset.current_timestep();

  Eigen::MatrixX2d controls(std::size_t{timesteps}, 2);
  Eigen::MatrixX2d variance(std::size_t{timesteps}, 2);

  for (slam::timestep_type t{0}; t < timesteps; ++t) {
    const double dt = dataset.timedelta(t);
    controls.row(std::size_t{t}) = dt * (dt * dataset.control(t)).cwiseAbs2();
    variance.row(std::size_t{t}) =
        (observe(trajectory[t]) - dt * dataset.control(t)).cwiseAbs2();
  }

  Eigen::Matrix2d learned;
  learned.row(0) = utility::nnls(controls, variance.col(0));
  learned.row(1) = utility::nnls(controls, variance.col(1));

  std::cout << "Learned control model:\n" << learned << std::endl;
  return builder(learned);
}

} // namespace planar_robot

#endif //_PLANAR_ROBOT_VELOCITY_MODEL_HPP
