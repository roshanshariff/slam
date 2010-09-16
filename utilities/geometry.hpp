#include <boost/math/constants/constants.hpp>
#include <cmath>

inline double wrap_angle (double angle, double base = 0.0) {
  using boost::math::constants::pi;
  angle -= base;
  angle = fmod (angle, 2.0*pi<double>());
  if (angle < -PI) angle += 2.0*pi<double>();
  else if (angle >= PI) angle -= 2.0*pi<double>();
  angle += base;
  return angle;
}
