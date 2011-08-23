#ifndef _PLANAR_ROBOT_RANGE_BEARING_MODEL_HPP
#define _PLANAR_ROBOT_RANGE_BEARING_MODEL_HPP

#include <functional>

#include <Eigen/Core>

#include "planar_robot/position.hpp"
#include "utility/random.hpp"
#include "utility/geometry.hpp"

namespace planar_robot {


struct range_bearing_model : public independent_normal_base<2, range_bearing_model> {

	typedef position feature_type;

	range_bearing_model (const vector_type& mean, const vector_type& stddev)
	: independent_normal_base(mean, stddev) { }

	static vector_type subtract (const vector_type& a, const vector_type& b) {
		return vector_type (a(0)-b(0), wrap_angle(a(1)-b(1)));
	}

	static vector_type from_feature (const position& f) {
		return vector_type (f.distance(), f.direction());
	}

	static position to_feature (const vector_type& observation) {
		return position::polar (observation(0), observation(1));
	}

	class builder : public std::unary_function<vector_type, range_bearing_model> {

		const vector_type stddev;

	public:

		builder (double range_stddev, bearing_stddev) : stddev(range_stddev, bearing_stddev) { }

		odometry_model operator() (const vector_type& observation) const {
			return odometry_model (observation, stddev);
		}

	};

};


} // namespace planar_robot

#endif //_PLANAR_ROBOT_RANGE_BEARING_MODEL_HPP
