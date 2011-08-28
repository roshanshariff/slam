#ifndef _PLANAR_ROBOT_ODOMETRY_MODEL_HPP
#define _PLANAR_ROBOT_ODOMETRY_MODEL_HPP

#include <cstdlib>
#include <functional>

#include <Eigen/Core>

#include "planar_robot/pose.hpp"
#include "utility/random.hpp"
#include "utility/geometry.hpp"


namespace planar_robot {


struct odometry_model : public independent_normal_base<3, odometry_model> {

	typedef independent_normal_base<3, odometry_model> base_type;
	typedef base_type::vector_type vector_type;
	typedef base_type::matrix_type matrix_type;

	typedef pose associated_type;

	odometry_model () { }

	odometry_model (const vector_type& mean, const vector_type& stddev) : base_type(mean, stddev) { }

	static vector_type subtract (const vector_type& a, const vector_type& b) {
		return vector_type (a(0)-b(0), wrap_angle(a(1)-b(1)), wrap_angle(a(2)-b(2)));
	}

	static vector_type to_vector (const associated_type& dp) {
		return vector_type (dp.distance(), dp.direction(), wrap_angle(dp.bearing()-dp.direction()));
	}

	static associated_type from_vector (const vector_type& control) {
		return pose::polar (control(0), control(1), control(1)+control(2));
	}

	class builder : public std::unary_function<vector_type, odometry_model> {

		Eigen::Matrix3d mat_stddev;

	public:

		builder (double a1, double a2, double a3, double a4) {
			mat_stddev << a3, a4, a4,
					      a2, a1,  0,
					      a2,  0, a1;
		}

		odometry_model operator() (const vector_type& control) const {
			vector_type stddev = mat_stddev * subtract(control, vector_type(0,0,0)).cwiseAbs();
			return odometry_model (control, stddev);
		}

	};

};


} // namespace planar_robot

#endif //_PLANAR_ROBOT_ODOMETRY_MODEL_HPP
