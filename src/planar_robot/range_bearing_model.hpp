#ifndef _PLANAR_ROBOT_RANGE_BEARING_MODEL_HPP
#define _PLANAR_ROBOT_RANGE_BEARING_MODEL_HPP

#include <functional>

#include "planar_robot/position.hpp"
#include "utilities/random.hpp"
#include "utilities/geometry.hpp"

namespace planar_robot {


  class range_bearing_model {

    normal_dist<double> range;
    normal_dist<double> bearing;

  public:

    typedef position result_type;

    range_bearing_model () : range(1.0, 1.0), bearing(0.0, 1.0) { }

    range_bearing_model (const position& mean,
		   double range_sigma, double bearing_sigma)
      : range (mean.range(), range_sigma),
	bearing (mean.bearing(), bearing_sigma) { }

    position mean () const {
      return position_polar (range.mean(), bearing.mean()); }

    position operator() (random_source& random) const {
      return position_polar (range(random), bearing(random));
    }

    double likelihood (const position& o) const {
      return range.likelihood (o.range())
	* bearing.likelihood (wrap_angle (o.bearing(), bearing.mean()));
    }

    double log_likelihood (const position& o) const {
      return range.log_likelihood (o.range())
	+ bearing.log_likelihood (wrap_angle (o.bearing(), bearing.mean()));
    }

    class builder : public std::unary_function<position, range_bearing_model> {

      const double range_sigma, bearing_sigma;

    public:

      typedef position observation_type;
      typedef range_bearing_model model_type;

      builder (double _range_sigma, double _bearing_sigma)
	: range_sigma(_range_sigma), bearing_sigma(_bearing_sigma) { }

      range_bearing_model operator() (const position& observation) const {
	return range_bearing_model (observation, range_sigma, bearing_sigma);
      }

    };
      

  };


} // namespace planar_robot

#endif //_PLANAR_ROBOT_RANGE_BEARING_MODEL_HPP
