#ifndef _UTILITIES_RANDOM_HPP
#define _UTILITIES_RANDOM_HPP

#include <boost/random.hpp>
#include <boost/math/constants/constants.hpp>

struct random_source {

  typedef boost::mt19937 engine_type;
  typedef boost::variate_generator<engine_type&, boost::uniform_01<double> > uniform_type;
  typedef boost::variate_generator<uniform_type&, boost::normal_distribution<double> > normal_type;

  engine_type generator;
  uniform_type uniform;
  normal_type normal;

  random_source ()
    : generator(), uniform(generator, boost::uniform_01<double>()),
      normal(uniform, boost::normal_distribution<double>()) { }

};


template <class RealType>
class normal_dist {
  
  RealType _mean;
  RealType _sigma;

public:

  typedef RealType result_type;

  normal_dist (RealType mean_arg, RealType sigma_arg)
    : _mean(mean_arg), _sigma(sigma_arg) { }

  RealType mean () const { return _mean; }
  RealType sigma () const { return _sigma; }
  
  RealType operator() (random_source& random) const {
    return mean() + sigma() * RealType(random.normal());
  }

  double likelihood (RealType x) const {
    x = (x - mean()) / sigma();
    return std::exp(-x*x/2.0) / (sigma() * boost::math::constants::root_two_pi<double>());
  }

  double log_likelihood (RealType x) const {
    x = (x - mean()) / sigma();
    return -x*x/2.0 - std::log(sigma() * boost::math::constants::root_two_pi<double>());
  }

};

#endif //_UTILITIES_RANDOM_HPP
