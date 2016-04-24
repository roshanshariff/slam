#ifndef _PLANAR_ROBOT_ODOMETRY_MODEL_HPP
#define _PLANAR_ROBOT_ODOMETRY_MODEL_HPP

#include <cstdlib>
#include <functional>

#include <Eigen/Core>

#include "planar_robot/pose.hpp"
#include "utility/geometry.hpp"
#include "utility/random.hpp"

namespace planar_robot {

struct odometry_model : public independent_normal_base<3, odometry_model> {

  using base_type = independent_normal_base<3, odometry_model>;
  using associated_type = pose;

  odometry_model() = default;
  odometry_model(const vector_type& mean, const vector_type& stddev)
      : base_type(mean, stddev) {}

  static auto subtract(const vector_type& a, const vector_type& b)
      -> vector_type {
    return vector_type(a(0) - b(0), wrap_angle(a(1) - b(1)),
                       wrap_angle(a(2) - b(2)));
  }

  static auto observe(const associated_type& dp) -> vector_type {
    return vector_type(dp.distance(), dp.direction(),
                       wrap_angle(dp.bearing() - dp.direction()));
  }

  static auto inv_observe -> associated_type(const vector_type& control) {
    return pose::polar(control(0), control(1), control(1) + control(2));
  }

  class proposal_dist {

    const odometry_model& model;

  public:
    explicit proposal_dist(decltype(model) model) : model(model) {}

    using result_type = associated_type;
    static constexpr int vector_dim = odometry_model::vector_dim;

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

  class builder : public std::unary_function<vector_type, odometry_model> {

    Eigen::Matrix3d mat_stddev;

  public:
    builder(double a1, double a2, double a3, double a4) {
      // clang-format off
      mat_stddev <<
        a3, a4, a4,
        a2, a1, 0,
        a2, 0, a1;
      // clang-format on
    }

    odometry_model operator()(const vector_type& control) const {
      vector_type stddev =
          mat_stddev * subtract(control, vector_type(0, 0, 0)).cwiseAbs();
      return odometry_model(control, stddev);
    }
  };
};

} // namespace planar_robot

#endif //_PLANAR_ROBOT_ODOMETRY_MODEL_HPP
