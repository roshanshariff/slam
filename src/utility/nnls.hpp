//
//  nnls.hpp
//  slam
//
//  Created by Roshan Shariff on 2013-08-13.
//  Copyright (c) 2013 University of Alberta. All rights reserved.
//

#ifndef slam_nnls_hpp
#define slam_nnls_hpp

#include <Eigen/Core>

namespace utility {

auto nnls(Eigen::MatrixXd A, Eigen::MatrixXd b, double* rnorm_ptr = nullptr)
    -> Eigen::VectorXd;
}

#endif
