#include <boost/random.hpp>

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
