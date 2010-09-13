#include <cmath>

#include "utilities/random_source.hpp"

template <class RealType>
class normal_dist {
  
  static const double PI = std::atan(1.0) * 4.0;
  static const double ALPHA = std::pow(2.0*PI, -0.5);
  static const double LN_ALPHA = -(std::log(2.0)+std::log(PI))/2.0;

  RealType _mean;
  RealType _sigma;

public:

  typedef RealType result_type;

  normal_dist (RealType mean_arg, RealType sigma_arg)
    : _mean(mean_arg), _sigma(sigma_arg) { }

  RealType mean () const { return _mean; }
  RealType sigma () const { return _sigma; }
  
  RealType operator() (random_source& random) const {
    return _mean + _sigma * RealType(random.normal());
  }

  double likelihood (RealType x) const {
    x -= _mean;
    x /= _sigma;
    return ALPHA * std::exp(-x*x/2.0) / _sigma;
  }

  double log_likelihood (RealType x) const {
    x -= _mean;
    x /= _sigma;
    return LN_ALPHA - x*x/2.0 - std::log(_sigma);
  }

};

