#ifndef _UTILITY_GEOMETRY_HPP
#define _UTILITY_GEOMETRY_HPP

#include <cmath>

#include <boost/math/constants/constants.hpp>

inline double wrap_angle(double angle) {
  const double PI = boost::math::constants::pi<double>();
  angle = std::fmod(angle, 2.0 * PI);
  if (angle < -PI)
    angle += 2.0 * PI;
  else if (angle >= PI)
    angle -= 2.0 * PI;
  return angle;
}

#endif //_UTILITY_GEOMETRY_HPP
