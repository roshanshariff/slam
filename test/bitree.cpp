#include <iostream>
#include <utility>
#include <cassert>
#include <tr1/random>

#include "utilities/bitree.hpp"
#include "utilities/random.hpp"

using namespace std;
using namespace std::tr1;

void extended_gcd (int a, int b, int& x, int& y, int& d) {
  x = 0;
  y = 1;
  int lastx = 1;
  int lasty = 0;
  while (b != 0) {
    int quotient = a / b;
    swap (a, b); b %= a;
    swap (x, lastx); x -= quotient*lastx;
    swap (y, lasty); y -= quotient*lasty;
  }
  x = lastx;
  y = lasty;
  d = a;
}

int inverse_modm (int x, int m) {
  int y, n, d;
  extended_gcd (x, m, y, n, d);
  assert(d == 1);
  while (y < 0) y += m;
  return y;
}

template <int P>
struct test_group {
  int a, b;
  test_group (int _a=0, int _b=1) : a(_a), b(_b) { reduce(); }
  void reduce () { a %= P; b %= P; }
  test_group<P>& operator+= (const test_group<P>& x) {
    a += b*x.a;
    b *= x.b;
    reduce();
    return *this;
  }
  test_group operator- () const {
    int b_inv = inverse_modm (b, P);
    return test_group(-a*b_inv, b_inv);
  }
  bool operator== (const test_group<P>& x) const {
    return ((a - x.a) % P == 0) && ((b - x.b) % P == 0);
  }
};

template <int P>
ostream& operator<< (ostream& out, const test_group<P> x) {
  out << '(' << x.a << ',' << x.b << ')';
  return out;
}

template <int P>
test_group<P> operator+ (test_group<P> x, const test_group<P>& y) {
  return x += y;
}

template <int P>
bool operator!= (const test_group<P>& x, const test_group<P>& y) {
  return !(x == y);
}

bool test_1 (random_source& random) {

  const int SIZE = 10000;

  bitree<int> seq;
  for (int i = 0; i < SIZE; ++i) {
    seq.push_back(i);
  }

  int sum = 0;
  for (int i = 0; i < int(seq.size()); ++i) {
    if (i != seq[i]) return false;
    if (seq.accumulate(i) != sum) return false;
    sum += i;
  }
  if (seq.accumulate(seq.size()) != sum) return false;

  return true;
}

bool test_2 (random_source& random) {
  
  const int PRIME = 32749;
  const int SIZE = 1000;
  const int TIMES = 1000;

  uniform_int<int> a (0, PRIME-1);
  uniform_int<int> b (1, PRIME-1);
  uniform_int<int> index (0, SIZE-1);

  typedef test_group<PRIME> group;  

  vector<group> elements;
  bitree<group> seq;
  for (int i = 0; i < SIZE; ++i) {
    group element (a(random.generator), b(random.generator));
    elements.push_back(element);
    seq.push_back(element);
  }

  for (int n = 0; n < TIMES; ++n) {
    group partial_sum;
    if (partial_sum != seq.accumulate(0)) return false;
    for (int i = 0; i < SIZE; ++i) {
      partial_sum += elements[i];
      if (seq.at(i) != elements[i]) return false;
      if (seq.accumulate(i+1) != partial_sum) return false;
    }

    group new_element (a(random.generator), b(random.generator));
    int new_index = index(random.generator);
    elements[new_index] = new_element;
    seq[new_index] = new_element;
  }

  return true;
}

bool test_3 (random_source& random) {

  const int SIZE = 10000;
  const int TIMES = 10000;

  uniform_int<int> element (0, 2);

  bitree<int> seq;
  for (int i = 0; i < SIZE; ++i) {
    seq.push_back(element(random.generator));
  }
  
  uniform_int<int> sum (0, seq.accumulate(seq.size())-1);
  for (int i = 0; i < TIMES; ++i) {
    int value = sum(random.generator);
    size_t position = seq.binary_search(value);
    if (!(seq.accumulate(position) <= value)) return false;
    if (!(value < seq.accumulate(position+1))) return false;
  }

  return true;
}

int main () {
  random_source random;
  cout << "Test 1: " << test_1(random) << endl;
  cout << "Test 2: " << test_2(random) << endl;
  cout << "Test 3: " << test_3(random) << endl;
}
