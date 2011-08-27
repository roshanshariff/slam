#ifndef _PLANAR_ROBOT_POSITION_HPP
#define _PLANAR_ROBOT_POSITION_HPP

#include <complex>

#include <Eigen/Core>

#include "planar_robot/pose.hpp"


namespace planar_robot {


class position {

	std::complex<double> pos;

protected:

	position (const std::complex<double>& p) : pos(p) { }

public:

	static const int vector_dim = 2;
	typedef Eigen::Vector2d vector_type;

	static position cartesian (double x, double y) {
		return position (std::complex<double> (x, y));
	}

	static position polar (double distance, double direction) {
		return position (std::polar (distance, direction));
	}

	double x () const { return pos.real(); }
	double y () const { return pos.imag(); }
	double distance () const { return std::abs(pos); }
	double direction () const { return std::arg(pos); }
	double distance_squared () const { return std::norm(pos); }

	vector_type to_vector () const { return vector_type (x(), y()); }

	static position from_vector (const vector_type& v) { return cartesian (v(0), v(1)); }

	friend position operator+ (const pose&, const position&);

};



inline position operator+ (const pose& p, const position& o) {
	return position (p.translation + p.rotation*o.pos);
}


} // namespace planar_robot

#endif //_PLANAR_ROBOT_POSITION_HPP
