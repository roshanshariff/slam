#ifndef _PLANAR_ROBOT_OBSERVATION_HPP
#define _PLANAR_ROBOT_OBSERVATION_HPP

#include <complex>

#include "utilities/random.hpp"
#include "utilities/geometry.hpp"
#include "planar_robot/pose.hpp"

namespace planar_robot {

  class observation {

    std::complex<double> position;

    observation (const std::complex<double>& p) : position(p) { }

  public:

    observation (double range = 0, double bearing = 0)
      : position(std::polar(range, bearing)) { }

    double x () const { return position.real(); }
    double y () const { return position.imag(); }
    double range () const { return std::abs(position); }
    double bearing () const { return std::arg(position); }

    friend observation operator+ (const pose&, const observation&);
    friend observation landmark (double x, double y);

  };


  inline observation operator+ (const pose& p, const observation& o) {
    return observation (p.translation + p.rotation*o.position);
  }


  inline observation landmark (double x, double y) {
    return observation (std::complex<double>(x, y));
  }


  class observation_dist {

    normal_dist<double> range;
    normal_dist<double> bearing;

  public:

    typedef observation result_type;

    observation_dist (const observation& mean = observation(),
		      double range_sigma = 1, double bearing_sigma = 1)
      : range (mean.range(), range_sigma),
	bearing (mean.bearing(), bearing_sigma) { }

    observation mean () const { return observation (range.mean(), bearing.mean()); }

    observation operator() (random_source& random) const {
      return observation (range(random), bearing(random));
    }

    double likelihood (const observation& o) const {
      return range.likelihood (o.range())
	* bearing.likelihood (wrap_angle (o.bearing(), bearing.mean()));
    }

    double log_likelihood (const observation& o) const {
      return range.log_likelihood (o.range())
	+ bearing.log_likelihood (wrap_angle (o.bearing(), bearing.mean()));
    }

  };

}

#endif //_PLANAR_ROBOT_OBSERVATION_HPP
