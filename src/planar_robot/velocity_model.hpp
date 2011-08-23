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


struct velocity_model : public independent_normal_base<3, velocity_model> {

	typedef pose state_type;

	velocity_model (const vector_type& mean, const vector_type& stddev)
	: independent_normal_base(mean, stddev) { }

	static vector_type subtract (const vector_type& a, const vector_type& b) { return a - b; }

	static vector_type from_state_change (const pose& dp) {
		if (dp.y() == 0) {
			return vector_type (dp.x(), 0.0, dp.bearing());
		}
		else {
			double r = 0.5 * dp.distance_squared() / dp.y();
			double theta = std::atan2 (dp.x(), r - dp.y());
			return vector_type (r*theta, theta, wrap_angle(dp.bearing()-theta));
		}
	}

	static pose to_state_change (const vector_type& control) {
		const double v = control(0), w = control(1), g = control(2);
		if (w == 0) {
			return pose (v, 0.0, g);
		}
		else {
			const double r = v / w;
			return pose (r*std::sin(w), r-r*std::cos(w), w+g);
		}
	}

	class builder : public std::unary_function<vector_type, velocity_model> {

		Eigen::Matrix3d mat_stddev;
		const double dt;

	public:

		builder (double a1, double a2, double a3, double a4, double a5, double a6, double _dt) : dt(_dt) {
			mat_stddev << a1, a2, 0,
					      a3, a4, 0,
					      a5, a6, 0;
		}

		velocity_model operator() (const vector_type& control) const {
			control *= dt;
			return velocity_model (control, mat_stddev*control.cwiseAbs());
		}

	};

};


} // namespace planar_robot

#endif //_PLANAR_ROBOT_VELOCITY_MODEL_HPP
