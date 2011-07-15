/*
 * unscented.hpp
 *
 *  Created on: Jul 12, 2011
 *      Author: rshariff
 */

#ifndef UNSCENTED_HPP_
#define UNSCENTED_HPP_

#include <cmath>
#include <vector>

#include <Eigen/Eigen>
#include <Eigen/Cholesky>

struct unscented_params {
    double alpha, kappa, beta;
};

template <class Matrix, class Vector, class Function>
void unscented_transform (Vector& mean, Matrix& cov, Function f, const unscented_params& p) {

    const int n = mean.size();
    const double lambda = p.alpha * p.alpha * (n + p.kappa) - n;

    Matrix sqrtP = std::sqrt(n + lambda) * cov.llt().matrixU();

    std::vector<Vector> sigmapts (2*n+1);
    sigmapts[0] = f(mean);

    mean = lambda * sigmapts[0];

    for (int i = 1; i <= n; ++i) {
	sigmapts[2*i-1] = f(mean + sqrtP.col(i-1));
	sigmapts[2*i] = f(mean - sqrtP.col(i-1));
	mean += (sigmapts[2*i-1] + sigmapts[2*i]) / 2;
    }

    mean /= n + lambda;

    sigmapts[0] -= mean;
    cov = sigmapts[0] * sigmapts[0].transpose() * (lambda + (n + lambda)*(1 - p.alpha*p.alpha + p.beta));

    for (int i = 1; i <=2*n+1; ++i) {
	sigmapts[i] -= mean;
	cov += sigmapts[i] * sigmapts[i].transpose() / 2;
    }

    cov /= n + lambda;

}


#endif /* UNSCENTED_HPP_ */
