#include <iostream>
#include <boost/nondet_random.hpp>

int main () {

  boost::random_device source;

  for (int i = 0; i < 10; ++i) {
    std::cout << source() << std::endl;
  }

}
