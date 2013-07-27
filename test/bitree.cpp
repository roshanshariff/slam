#include <iostream>
#include <utility>
#include <cassert>
#include <random>

#include "utility/bitree.hpp"
#include "utility/random.hpp"

using namespace std;
using utility::bitree;

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
    friend bool operator== (const test_group<P>& x, const test_group<P>& y) {
        return ((x.a - y.a) % P == 0) && ((x.b - y.b) % P == 0);
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
    
    const int SIZE = 100000;
    
    bitree<int> seq, seq2;
    int sum = 0;
    for (int i = 0; i < SIZE; ++i) {
        sum += i;
        seq.push_back(i);
        seq2.push_back_accumulated(sum);
    }
    
    sum = 0;
    for (int i = 0; i < int(seq.size()); ++i) {
        if (i != seq[i]) return false;
        if (i != seq2[i]) return false;
        if (seq.accumulate(i) != sum) return false;
        if (seq2.accumulate(i) != sum) return false;
        sum += i;
    }
    if (seq.accumulate() != sum) return false;
    if (seq2.accumulate() != sum) return false;
    
    return true;
}

bool test_2 (random_source& random) {
    
    const int PRIME = 32749;
    const int SIZE = 2000;
    const int TIMES = 10000;
    
    uniform_int_distribution<> a (0, PRIME-1);
    uniform_int_distribution<> b (1, PRIME-1);
    uniform_int_distribution<> index (0, SIZE-1);
    
    typedef test_group<PRIME> group;
    
    vector<group> elements;
    bitree<group> seq, seq2;
    group accumulated = group();
    for (int i = 0; i < SIZE; ++i) {
        group element (a(random), b(random));
        elements.push_back(element);
        seq.push_back(element);
        accumulated += element;
        seq2.push_back_accumulated (accumulated);
    }
    
    assert (seq.size() == seq2.size());
    
    for (int i = 0; i < SIZE; ++i) {
        assert (group(seq[i]) == group(seq2[i]));
    }
    
    for (int n = 0; n < TIMES; ++n) {
        group partial_sum;
        assert (partial_sum == seq.accumulate(0));
        assert (partial_sum == seq2.accumulate(0));
        for (int i = 0; i < SIZE; ++i) {
            partial_sum += elements[i];
            assert (group(seq[i]) == elements[i]);
            assert (group(seq2[i]) == elements[i]);
            assert (seq.accumulate(i+1) == partial_sum);
            assert (seq2.accumulate(i+1) == partial_sum);
        }
        
        group new_element (a(random), b(random));
        int new_index = index(random);
        elements[new_index] = new_element;
        seq[new_index] = new_element;
        seq2[new_index] = new_element;
    }
    
    for (size_t i = 0; i <= seq.size(); ++i) {
        for (size_t j = 0; j <= seq.size(); ++j) {
            assert (-seq.accumulate(i) + seq.accumulate(j) == seq.accumulate(i, j));
            assert (-seq2.accumulate(i) + seq2.accumulate(j) == seq2.accumulate(i, j));
        }
    }
    
    return true;
}

bool test_3 (random_source& random) {
    
    const int SIZE = 10000;
    const int TIMES = 10000;
    
    uniform_int_distribution<> element (0, 2);
    
    bitree<int> seq;
    for (int i = 0; i < SIZE; ++i) {
        seq.push_back(element(random));
    }
    
    uniform_int_distribution<> sum (0, seq.accumulate(seq.size())-1);
    for (int i = 0; i < TIMES; ++i) {
        int value = sum(random);
        size_t position = seq.binary_search(value);
        if (!(seq.accumulate(position) <= value)) return false;
        if (!(value < seq.accumulate(position+1))) return false;
    }
    
    return true;
}

bool test_4 (random_source& random) {
    
    const int SIZE = 10000;
    const int TIMES = 10000;
    
    uniform_int_distribution<> element (0, 2);
    
    bitree<int> seq;
    for (int i = 0; i < SIZE; ++i) {
        seq.push_back_accumulated(element(random));
    }
    
    uniform_int_distribution<> sum (0, seq.accumulate(seq.size())-1);
    for (int i = 0; i < TIMES; ++i) {
        int value = sum(random);
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
    cout << "Test 4: " << test_4(random) << endl;
}
