#ifndef _PLANAR_ROBOT_VELOCITY_MODEL_HPP
#define _PLANAR_ROBOT_VELOCITY_MODEL_HPP

#include <cstdlib>
#include <cmath>
#include <functional>

#include <Eigen/Core>

#include "planar_robot/pose.hpp"
#include "utility/random.hpp"
#include "utility/geometry.hpp"


namespace planar_robot {


  class velocity_control {

    Eigen::Vector3d control;

  public:

    velocity_control (double v, double w, double g = 0) : control(v, w, g) { }

    double v () const { return control.x(); }
    double w () const { return control.y(); }
    double g () const { return control.z(); }

  };


  inline velocity_control pose_delta_to_velocity (const pose& dp, double dt = 1.0) {
    if (dp.y() == 0) {
      return velocity_control (dp.x()/dt, 0.0, dp.bearing()/dt);
    }
    else {
      double r = 0.5 * dp.distance_squared() / dp.y();
      double theta = std::atan2 (dp.x(), r - dp.y());
      return velocity_control (r*theta/dt, theta/dt, wrap_angle(dp.bearing()-theta)/dt);
    }
  }


  inline pose velocity_to_pose_delta (const velocity_control& control, double dt = 1.0) {
    if (control.w() == 0) {
      return pose (control.v() * dt, 0.0, control.g() * dt);
    }
    else {
      double r = control.v() / control.w();
      double theta = control.w() * dt;
      return pose (r*std::sin(theta), r-r*std::cos(theta), theta+control.g()*dt);
    }
  }


  class velocity_model {

    normal_dist<double> v;
    normal_dist<double> w;
    normal_dist<double> g;
    double dt;

  public:

    typedef pose result_type;

    velocity_model (const velocity_control& mean, double v_sigma,
		    double w_sigma, double g_sigma, double _dt)
      : v (mean.v(), v_sigma), w (mean.w(), w_sigma),
	g (mean.g(), g_sigma), dt(_dt) { }

    pose mean () const {
      return velocity_to_pose_delta
	(velocity_control (v.mean(), w.mean(), g.mean()), dt);
    }

    pose operator() (random_source& random) const {
      return velocity_to_pose_delta
	(velocity_control (v(random), w(random), g(random)), dt);
    }

    double likelihood (const pose& p) const {
      velocity_control control = pose_delta_to_velocity (p, dt);
      return v.likelihood (control.v())
	* w.likelihood (control.w())
	* g.likelihood (control.g());
    }

    double log_likelihood (const pose& p) const {
      velocity_control control = pose_delta_to_velocity (p, dt);
      return v.log_likelihood (control.v())
	+ w.log_likelihood (control.w())
	+ g.log_likelihood (control.g());
    }

    class builder : public std::binary_function<double, velocity_control, velocity_model> {

      const double alpha1, alpha2, alpha3, alpha4, alpha5, alpha6;

    public:

      builder (double a1, double a2, double a3, double a4, double a5, double a6)
	: alpha1(a1), alpha2(a2), alpha3(a3), alpha4(a4), alpha5(a5), alpha6(a6) { }

      velocity_model operator() (double dt, const velocity_control& control) const {

	double v = std::abs (control.v());
	double w = std::abs (control.w());

	double v_sigma = alpha1 * v + alpha2 * w;
	double w_sigma = alpha3 * v + alpha4 * w;
	double g_sigma = alpha5 * v + alpha6 * w;

	return velocity_model (control, v_sigma, w_sigma, g_sigma, dt);
      }

    };

  };


} // namespace planar_robot

#endif //_PLANAR_ROBOT_VELOCITY_MODEL_HPP
