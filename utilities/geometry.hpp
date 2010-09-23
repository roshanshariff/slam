#ifndef _UTILITIES_GEOMETRY_HPP
#define _UTILITIES_GEOMETRY_HPP

#include <boost/math/constants/constants.hpp>
#include <cmath>

inline double wrap_angle (double angle, double base = 0.0) {
  const double PI = boost::math::constants::pi<double>();
  angle -= base;
  angle = fmod (angle, 2.0*PI);
  if (angle < -PI) angle += 2.0*PI;
  else if (angle >= PI) angle -= 2.0*PI;
  angle += base;
  return angle;
}

#endif //_UTILITIES_GEOMETRY_HPP
