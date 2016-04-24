//
//  nnls.cpp
//  slam
//
//  Created by Roshan Shariff on 2013-08-13.
//  Copyright (c) 2013 University of Alberta. All rights reserved.
//

#include <cassert>
#include <stdexcept>

#include "utility/nnls.hpp"

extern "C" {
#include "utility/nnls.h"
}

auto utility::nnls(Eigen::MatrixXd A, Eigen::MatrixXd b, double* rnorm_ptr)
    -> Eigen::VectorXd {

  assert(A.innerStride() == 1);
  assert(A.rows() == b.rows());
  assert(b.innerStride() == 1);

  const int mda = A.outerStride();
  const int m = A.rows();
  const int n = A.cols();

  Eigen::VectorXd x(n);

  double rnorm;
  if (!rnorm_ptr) rnorm_ptr = &rnorm;

  Eigen::VectorXd w(n);
  Eigen::VectorXd zz(m);
  Eigen::VectorXi index(n);

  int mode;
  nnls_c(A.data(), &mda, &m, &n, b.data(), x.data(), rnorm_ptr, w.data(),
         zz.data(), index.data(), &mode);

  switch (mode) {
  case 1: // THE SOLUTION HAS BEEN COMPUTED SUCCESSFULLY
    return x;
  case 2: // THE DIMENSIONS OF THE PROBLEM ARE BAD. EITHER M .LE. 0 OR N .LE. 0.
    throw std::invalid_argument("nnls: negative number used as dimension");
  case 3: // ITERATION COUNT EXCEEDED.  MORE THAN 3*N ITERATIONS.
    throw std::logic_error("nnls: iteration count exceeded");
  default:
    throw std::logic_error("nnls: invalid value of mode");
  }
}
