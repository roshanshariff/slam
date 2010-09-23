#include <string>
#include <iostream>

#include "utilities/arraymap.hpp"

using std::string;

int main () {

  arraymap<int, string> testmap;
  testmap[5] = "five";
  testmap[3] = "three";
  testmap[1] = "one";
  testmap[0] = "zero";
  testmap[2] = "two";
  testmap[4] = "four";

  std::cout << "size() = " << testmap.size() << std::endl;

}
